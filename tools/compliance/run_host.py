#!/usr/bin/env python3
"""Execute host suites on Linux, preserving individual outcomes and logs."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('gcc', 'clang', 'sanitize'), default='gcc')
    args = parser.parse_args()
    if sys.platform != 'linux':
        parser.error('Execution requires Linux; native Windows tests are not run.')
    root, output = args.root.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    compiler = 'gcc-13' if args.mode == 'gcc' else 'clang-18'
    flags = '-std=c11 -Wall -Wextra -Werror -O2 -g -pthread'
    if args.mode == 'sanitize':
        flags += ' -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all'
    if compiler.startswith('clang'):
        flags += ' -Wno-bitwise-instead-of-logical'
    env = dict(os.environ, ASAN_OPTIONS='detect_leaks=1:halt_on_error=1',
               UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
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
        sanitizer_error = bool(re.search(r'ERROR: (AddressSanitizer|LeakSanitizer)|runtime error:|UndefinedBehaviorSanitizer:|AddressSanitizer:DEADLYSIGNAL', contents))
        skipped = re.findall(r'OK \(skipped=(\d+)\)', contents)
        passed = status == 0 and not sanitizer_error and not skipped
        results.append(dict(name=name, command=command, returncode=status,
                            status='PASS' if passed else 'FAIL', skipped=skipped,
                            sanitizer_error=sanitizer_error,
                            duration_s=round(time.monotonic()-started, 3), log=log.name))
        print(f'{results[-1]["status"]}: {name}', flush=True)
        if not passed:
            print(contents[-5000:], flush=True)

    def make(name, folder, targets, makefile=None):
        cmd = ['make', '-C', folder]
        if makefile:
            cmd += ['-f', makefile]
        suite_flags = flags
        if name in ('subsystem3', 'subsystem4', 'subsystem5', 'diagnostics'):
            suite_flags += ' -pedantic'
        cmd += list(targets) + [f'CC={compiler}', f'BUILD={output / "bin" / name}', f'CFLAGS={suite_flags}']
        run(name, cmd)

    run('environment', [sys.executable, '-c',
        'import platform,sys;print(platform.platform());print(sys.version)'])
    run('compiler', [compiler, '--version'])
    run('packages', ['dpkg-query', '-W', 'gcc-13', 'g++-13', 'clang-18', 'libc6',
                     'make', 'python3', 'nodejs', 'mosquitto', 'mosquitto-clients'])
    for number in range(1, 6):
        targets = ['test']
        if number == 3:
            targets += ['codec', 'drive', 'feedback', 'replay']
        make(f'subsystem{number}', f'testing/subsystem{number}', targets)
    make('integration', 'testing/integration', ['test'], 'controller_test.mk')
    make('diagnostics', 'testing/diagnostics', ['test', 'smoke', 'disabled'], 'firmware_test.mk')
    make('network_wifi', 'testing/network_wifi', ['test'])
    trace = Path(__file__).resolve().parents[2] / 'testing/compliance/control_debug_trace.c'
    trace_binary = output / 'control_debug_trace'
    run('control_debug_trace_build', [compiler, *shlex.split(flags), '-I' + str(root),
        str(trace), str(root / 'app_program/controller/control_debug.c'), '-o', str(trace_binary)])
    run('control_debug_trace', [str(trace_binary)])
    if args.mode != 'sanitize':
        for folder in ('dashboard', 'change_requirement_1', 'diagnostics'):
            run('python_' + folder, [sys.executable, '-m', 'unittest', 'discover', '-v',
                                    '-s', 'testing/' + folder, '-p', 'test_*.py'])
        run('laptop_codec', [sys.executable, 'testing/subsystem1/test_laptop_codec.py'])
        run('frontend', ['node', '--test', 'testing/dashboard/test_frontend_controls.js'])
        run('widths', [sys.executable, 'testing/change_requirement_1/check_widths.py'])
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
