#!/usr/bin/env python3
"""Execute host suites on Linux, preserving individual outcomes and logs."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time


def classify_output(status, contents):
    """Fail closed on sanitizer errors, required skips, and empty discovery."""
    sanitizer_error = bool(re.search(
        r'ERROR: (AddressSanitizer|LeakSanitizer)|runtime error:|UndefinedBehaviorSanitizer:|AddressSanitizer:DEADLYSIGNAL', contents))
    skipped = [line for line in contents.splitlines() if ' ... skipped ' in line]
    optional_name = "test_real_successful_session_trace (test_traces.TraceTests.test_real_successful_session_trace) ... skipped 'Optional local hardware capture is unavailable'"
    allowed_skips = [line for line in skipped if line == optional_name]
    unexpected_skips = [line for line in skipped if line not in allowed_skips]
    reported_skips = sum(map(int, re.findall(r'OK \(skipped=(\d+)\)', contents)))
    empty_discovery = bool(re.search(r'^Ran 0 tests\b', contents, re.MULTILINE))
    passed = (status == 0 and not sanitizer_error and not unexpected_skips
              and reported_skips == len(allowed_skips) and not empty_discovery)
    return dict(status='PASS' if passed else 'FAIL', skipped=skipped,
                optional_unavailable=allowed_skips, sanitizer_error=sanitizer_error,
                empty_discovery=empty_discovery)


def motion_storage_flags(root):
    """Select the private adapter only when the subject defines that helper."""
    source = (root/'app_program/subsystem2/motion.c').read_text(encoding='utf-8')
    compact = re.search(r'\bstatic\s+void\s+motion_calibration_store\s*\(', source)
    return ['-DMOTION_COMPACT_RETAINED=1'] if compact else []


def motion_estimation_module(root):
    """Resolve the immutable baseline or renamed candidate, without ambiguity."""
    directory = root/'app_program/subsystem2'
    names = ('estimate_motion', 'motion_estimation')
    present = {name: tuple((directory/(name+suffix)).is_file() for suffix in ('.c', '.h'))
               for name in names}
    complete = [name for name, pair in present.items() if pair == (True, True)]
    if len(complete) != 1 or any(any(pair) for name, pair in present.items() if name not in complete):
        raise ValueError('Expected exactly one complete motion estimation .c/.h pair: ' + str(present))
    return complete[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('gcc', 'clang', 'sanitize'), default='gcc')
    parser.add_argument('--check-c99', action='store_true')
    args = parser.parse_args()
    if sys.platform != 'linux':
        parser.error('Execution requires Linux; native Windows tests are not run.')
    root, output = args.root.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    compiler = 'gcc-13' if args.mode == 'gcc' else 'clang-18'
    base_flags = '-std=c11 -Wall -Wextra -Werror -O2 -g -pthread'
    if compiler.startswith('clang'):
        base_flags += ' -Wno-bitwise-instead-of-logical'
    flags = base_flags
    if args.mode == 'sanitize':
        flags += ' -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all'
    env = dict(os.environ, ASAN_OPTIONS='detect_leaks=1:halt_on_error=1',
               UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
    compiler_path = shutil.which(compiler)
    if compiler_path is None:
        parser.error('Missing pinned compiler: ' + compiler)
    tool_bin = output / 'tool-bin'
    tool_bin.mkdir()
    tooling = Path(__file__).resolve().parent
    compiler_proxy = tool_bin/'cc'
    compiler_proxy.write_text(f'#!{sys.executable}\nimport runpy\nrunpy.run_path({str(tooling/"compiler_proxy.py")!r}, run_name="__main__")\n', encoding='utf-8')
    compiler_proxy.chmod(0o755)
    env['COMPLIANCE_REAL_CC'] = compiler_path
    env['COMPLIANCE_COMPILE_LOG'] = str(output/'compile_commands.jsonl')
    compiler = str(compiler_proxy)
    env['PATH'] = str(tool_bin) + os.pathsep + env['PATH']
    results = []

    def run(name, command):
        started = time.monotonic()
        log = output / (name + '.log')
        try:
            with log.open('w', encoding='utf-8') as stream:
                stream.write('COMMAND ' + json.dumps(command) + '\n')
                stream.flush()
                status = subprocess.run(command, cwd=root, env=env, stdout=stream,
                                        stderr=subprocess.STDOUT, timeout=600).returncode
        except (OSError, subprocess.TimeoutExpired) as exc:
            with log.open('a', encoding='utf-8') as stream:
                stream.write('\nRUNNER ERROR: ' + str(exc) + '\n')
            status = 125
        contents = log.read_text(encoding='utf-8', errors='replace')
        # The old S5 recipe tolerates one geometric simulation miss. It must
        # never hide a sanitizer report from that intentionally non-gating case.
        outcome = classify_output(status, contents)
        results.append(dict(name=name, command=command, returncode=status,
                            **outcome,
                            duration_s=round(time.monotonic()-started, 3), log=log.name))
        print(f'{results[-1]["status"]}: {name}', flush=True)
        if outcome['status'] != 'PASS':
            print(contents[-5000:], flush=True)

    def make(name, folder, targets, makefile=None, instrument=True):
        cmd = ['make', '-C', folder]
        if makefile:
            cmd += ['-f', makefile]
        suite_flags = flags if instrument else base_flags
        if name in ('subsystem3', 'subsystem4', 'subsystem5', 'diagnostics', 'diagnostics_disabled'):
            suite_flags += ' -pedantic'
        cmd += list(targets) + [f'CC={compiler}', 'PYTHON=/usr/bin/python3', f'BUILD={output / "bin" / name}', f'CFLAGS={suite_flags}']
        run(name, cmd)

    run('environment', [sys.executable, '-c',
        'import platform,sys;print(platform.platform());print(sys.version)'])
    run('compiler', [compiler, '--version'])
    run('node', ['/usr/bin/node', '--version'])
    run('packages', ['dpkg-query', '-W', 'gcc-13', 'g++-13', 'clang-18', 'libc6',
                     'make', 'python3', 'nodejs', 'mosquitto', 'mosquitto-clients'])
    # The fixed baseline's Python codec test uses this historical path even
    # when Make's BUILD is overridden. An isolated link preserves that test
    # unchanged and directs it at this run's exact binary.
    legacy_build = root / 'build/buddy1'
    legacy_build.parent.mkdir(parents=True, exist_ok=True)
    legacy_build.symlink_to(output / 'bin/subsystem1', target_is_directory=True)
    env['BUDDY1_HOST_TEST'] = str(output / 'bin/subsystem1/host_test')
    for number in range(1, 6):
        targets = ['test']
        if number == 3:
            targets += ['codec', 'drive', 'feedback', 'replay']
        make(f'subsystem{number}', f'testing/subsystem{number}', targets)
    # Production disables the extra IMU diagnostics fields. Exercise its actual
    # layout and branches, including under sanitizers, with the same fixture.
    terrain_binary = output/'terrain_production_test'
    terrain_app = root/'app_program/subsystem4'
    terrain_fixture = tooling.parents[1]/'testing/subsystem4'
    run('terrain_production_build', [compiler, *shlex.split(flags), '-DAPP_IMU_DEBUG=0',
        '-I'+str(terrain_app), '-I'+str(terrain_fixture), str(terrain_fixture/'host_test.c'),
        *[str(terrain_app/(name+'.c')) for name in
          ('imu_sensor', 'imu_calibration', 'tilt_estimation', 'hump_detection', 'events_motion', 'terr_data')],
        '-lm', '-o', str(terrain_binary)])
    run('terrain_production', [str(terrain_binary)])
    make('integration', 'testing/integration', ['test'], 'controller_test.mk')
    make('diagnostics', 'testing/diagnostics', ['test', 'smoke'], 'firmware_test.mk')
    # Instrumentation adds compiler runtime constructors even to an otherwise
    # empty translation unit. Preserve the exact disabled-object expectation
    # with production-like flags; execute the enabled tests with sanitizers.
    make('diagnostics_disabled', 'testing/diagnostics', ['disabled'], 'firmware_test.mk', instrument=False)
    make('network_wifi', 'testing/network_wifi', ['test'])
    trace = Path(__file__).resolve().parents[2] / 'testing/compliance/control_debug_trace.c'
    trace_binary = output / 'control_debug_trace'
    run('control_debug_trace_build', [compiler, *shlex.split(flags), '-I' + str(root),
        str(trace), str(root / 'app_program/controller/control_debug.c'), '-o', str(trace_binary)])
    run('control_debug_trace', [str(trace_binary)])
    builder_test = root / 'testing/compliance/control_builder_test.c'
    if builder_test.is_file():
        builder_binary = output/'control_builder_test'
        run('control_builder_build', [compiler, *shlex.split(flags), '-I'+str(root),
            str(builder_test), str(root/'app_program/controller/control_debug.c'), '-o', str(builder_binary)])
        run('control_builder', [str(builder_binary)])
    motion_fixture = tooling.parents[1]/'testing/compliance/motion_storage_test.c'
    motion_stubs = tooling.parents[1]/'testing/subsystem3'
    motion_app = root/'app_program/subsystem2'
    estimation_module = motion_estimation_module(root)
    motion_binary = output/'motion_storage_test'
    run('motion_storage_build', [compiler, *shlex.split(flags),
        *motion_storage_flags(root), '-I'+str(root), '-I'+str(motion_app),
        '-I'+str(motion_stubs), '-include', str(motion_stubs/'motion_kernel_stub.h'),
        str(motion_fixture), *[str(motion_app/(name+'.c')) for name in
            ('ctrl_motion', 'motion_pack', estimation_module, 'speed_pid')],
        '-o', str(motion_binary)])
    run('motion_storage', [str(motion_binary)])
    barcode_fixture = tooling.parents[1]/'testing/compliance/barcode_work_test.c'
    barcode_binary = output/'barcode_work_test'
    run('barcode_work_build', [compiler, *shlex.split(flags), '-I'+str(root),
        str(barcode_fixture), '-o', str(barcode_binary)])
    run('barcode_work', [str(barcode_binary)])
    bcap_binary = output/'bcap_storage_test'
    run('bcap_storage_build', [compiler, *shlex.split(flags), '-I'+str(root),
        str(tooling.parents[1]/'testing/compliance/bcap_storage_test.c'),
        str(root/'app_program/controller/bc_capture.c'), '-lm', '-o', str(bcap_binary)])
    run('bcap_storage', [str(bcap_binary)])
    from instrument_harnesses import operator_source
    operator_fixture = operator_source(tooling.parents[1], root, output/'operator-observed.c')
    operator_binary = output/'operator_observed'
    operator_modules = [root/'app_program/controller'/(name+'.c') for name in
                        ('bc_capture', 'op_logic', 'integration_codec', 's3_drive')]
    operator_modules += [root/'app_program/subsystem3'/(name+'.c') for name in
                         ('navigation_codec', 'ir_acquisition', 'ir_calibration', 'ir_reflectance',
                          'line_detection', 'line_controller', 'bc_decoder', 'line_navigation')]
    operator_fixtures = tooling.parents[1]/'testing/integration'
    run('operator_observed_build', [compiler, *shlex.split(flags),
        *['-I'+str(path) for path in (operator_fixtures, operator_fixtures/'fakes',
                                    root/'app_program/controller')],
        str(operator_fixture), str(tooling/'traces/op_trace.c'),
        *map(str, operator_modules), '-lm', '-o', str(operator_binary)])
    run('operator_observed', [str(operator_binary)])
    # The existing S1 make targets cover 0/0 and 1/1. Exercise each independent
    # debug layout with the same assertions, including under ASan/UBSan.
    mqtt_fixtures = tooling.parents[1]/'testing/subsystem1'
    mqtt_app = root/'app_program/subsystem1'
    mqtt_driver = root/'lib/liblwip/sysdepend/pico_rp2040'
    for name, ir_debug, imu_debug, fixture in (
            ('mqtt_production', 1, 0, mqtt_fixtures/'host_test.c'),
            ('mqtt_imu', 0, 1, mqtt_fixtures/'host_test.c'),
            ('mqtt_receive', 1, 1, tooling.parents[1]/'testing/compliance/mqtt_receive_test.c')):
        mqtt_binary = output/name
        run(name+'_build', [compiler, *shlex.split(flags), '-DAPP_H', '-DAPP_TEST=1',
            '-DTM_MQTT=1', f'-DAPP_IR_DEBUG={ir_debug}', f'-DAPP_IMU_DEBUG={imu_debug}',
            '-include', str(mqtt_fixtures/'host_kernel.h'),
            *['-I'+str(path) for path in (mqtt_fixtures, mqtt_fixtures/'fakes', mqtt_app, mqtt_driver)],
            str(fixture),
            *[str(mqtt_app/(module+'.c')) for module in
                ('telemetry_codec', 'buddy1', 'command', 'mqtt_bridge', 'buddy1_test')],
            str(mqtt_driver/'mqtt_utk.c'), '-o', str(mqtt_binary)])
        run(name, [str(mqtt_binary)])
    if args.mode != 'sanitize':
        for folder in ('dashboard', 'change_requirement_1', 'diagnostics'):
            run('python_' + folder, [sys.executable, '-m', 'unittest', 'discover', '-v',
                                    '-s', 'testing/' + folder, '-p', 'test_*.py'])
        run('laptop_codec', [sys.executable, 'testing/subsystem1/test_laptop_codec.py'])
        run('frontend', ['/usr/bin/node', '--test', 'testing/dashboard/test_frontend_controls.js'])
        run('widths', [sys.executable, 'testing/change_requirement_1/check_widths.py'])
    if args.check_c99 and args.mode != 'sanitize':
        run('application_c99', [sys.executable, str(tooling/'check_host_c99.py'),
            '--root', str(root), '--commands', str(output/'compile_commands.jsonl'),
            '--output', str(output/'application-c99')])
    manifest = dict(recorded_at=datetime.now(timezone.utc).isoformat(),
                    revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                    mode=args.mode, results=results,
                    runner_image={key: os.environ.get(key) for key in ('ImageOS', 'ImageVersion', 'RUNNER_OS', 'RUNNER_ARCH')},
                    limits=['Host fakes do not prove physical timing or hardware behavior.',
                            'S5 turn=0.9 remains an explicitly documented geometric limitation.'])
    (output / 'results.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    failed = [item['name'] for item in results if item['status'] != 'PASS']
    print(f'{len(results)-len(failed)}/{len(results)} runner steps passed; failures: {failed}')
    return bool(failed)


if __name__ == '__main__':
    sys.exit(main())
