#!/usr/bin/env python3
"""Compare real baseline/candidate semantic traces and every recorded replay."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from replay_audit import audit
from instrument_harnesses import runtime_source, mqtt_source, operator_source
from run_host import motion_estimation_module, motion_storage_flags


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('baseline', 'candidate', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    if sys.platform != 'linux':
        parser.error('Differential native execution requires Linux.')
    roots = {label: getattr(args, label).resolve() for label in ('baseline', 'candidate')}
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    results = dict(revisions={name: subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root,
                                                          text=True).strip() for name, root in roots.items()},
                   checks=[], limits=['Host traces do not establish RTOS scheduling or physical equivalence.'])
    failed = False
    binaries = {}

    def build(label, name, sources, includes=(), flags=()):
        binary = output / (label + '-' + name)
        command = ['gcc-13', '-std=c11', '-Wall', '-Wextra', '-Werror', '-O2', '-g',
                   *flags, *['-I' + str(path) for path in includes], *map(str, sources), '-lm', '-o', str(binary)]
        with (output / (label + '-' + name + '-build.log')).open('w') as log:
            log.write('COMMAND ' + json.dumps(command) + '\n')
            log.flush()
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)
        return binary

    try:
        trace_harness = roots['candidate'] / 'testing/compliance/control_debug_trace.c'
        shared = roots['candidate'] / 'tools/compliance/traces'
        for label, root in roots.items():
            debug_binary = build(label, 'control_debug', [trace_harness, root / 'app_program/controller/control_debug.c'], [root])
            with (output / (label + '-control_debug.trace')).open('wb') as stream:
                subprocess.run([str(debug_binary)], stdout=stream, check=True, timeout=30)
            app = root / 'app_program'
            fixtures = roots['candidate'] / 'testing'
            estimation_module = motion_estimation_module(root)
            suites = {
                'motion': (2, ['motion', 'ctrl_motion', 'motion_pack', estimation_module, 'speed_pid'],
                           fixtures/'subsystem3', ['-include', str(fixtures/'subsystem3/motion_kernel_stub.h')]),
                'terrain': (4, ['imu_sensor', 'imu_calibration', 'tilt_estimation', 'hump_detection', 'events_motion', 'terr_data'],
                            fixtures/'subsystem4', ['-DAPP_IMU_DEBUG=1']),
                'terrain_production': (4, ['imu_sensor', 'imu_calibration', 'tilt_estimation', 'hump_detection', 'events_motion', 'terr_data'],
                                       fixtures/'subsystem4', ['-DAPP_IMU_DEBUG=0']),
                'obstacle': (5, ['profile_obstacle', 'scan_obstacle', 'avoidance_planner', 'line_recovery', 'avoid_obstacle', 's5_codec', 'ultrasonic_sensor'],
                             fixtures/'subsystem5', ['-DS5_HOST_TEST']),
            }
            for name, (number, modules, harness_dir, flags) in suites.items():
                directory = app / f'subsystem{number}'
                trace_name = 'terrain' if name == 'terrain_production' else name
                binary = build(label, name, [shared/(trace_name+'_trace.c'), *[directory/(module+'.c') for module in modules]],
                               [directory, harness_dir], flags)
                with (output / (label + '-' + name + '.trace')).open('wb') as stream:
                    subprocess.run([str(binary)], stdout=stream, check=True, timeout=120)
            binary = build(label, 'motion_storage',
                [fixtures/'compliance/motion_storage_test.c',
                 *[app/'subsystem2'/(name+'.c') for name in
                   ('ctrl_motion', 'motion_pack', estimation_module, 'speed_pid')]],
                [root, app/'subsystem2', fixtures/'subsystem3'],
                [*motion_storage_flags(root), '-include', str(fixtures/'subsystem3/motion_kernel_stub.h')])
            with (output/(label+'-motion_storage.trace')).open('wb') as stdout, (output/(label+'-motion_storage.log')).open('wb') as stderr:
                subprocess.run([str(binary)], stdout=stdout, stderr=stderr, check=True, timeout=120)
            binary = build(label, 'barcode_work', [fixtures/'compliance/barcode_work_test.c'], [root])
            with (output/(label+'-barcode_work.trace')).open('wb') as stdout, (output/(label+'-barcode_work.log')).open('wb') as stderr:
                subprocess.run([str(binary)], stdout=stdout, stderr=stderr, check=True, timeout=120)
            binary = build(label, 'bcap_storage',
                [fixtures/'compliance/bcap_storage_test.c', app/'controller/bc_capture.c'], [root])
            with (output/(label+'-bcap_storage.trace')).open('wb') as stdout, (output/(label+'-bcap_storage.log')).open('wb') as stderr:
                subprocess.run([str(binary)], stdout=stdout, stderr=stderr, check=True, timeout=120)
            runtime = runtime_source(roots['candidate'], root, output/(label+'-runtime.c'))
            runtime_sources = [app/'controller'/(name+'.c') for name in
                               ('bc_capture', 'integration_codec', 's3_drive', 'ir_debug', 'control_debug')]
            runtime_sources += [app/'subsystem3'/(name+'.c') for name in
                                ('navigation_codec', 'ir_acquisition', 'ir_calibration', 'ir_reflectance',
                                 'line_detection', 'line_controller', 'bc_decoder', 'line_navigation')]
            binary = build(label, 'runtime', [runtime, *runtime_sources],
                [fixtures/'integration', fixtures/'integration/fakes', root/'lib/liblwip/sysdepend/pico_rp2040'],
                ['-pthread', '-DAPP_IR_DEBUG=1', '-DTM_MQTT=1'])
            with (output/(label+'-runtime.trace')).open('wb') as stream:
                subprocess.run([str(binary)], stdout=stream, check=True, timeout=180)
            operator = operator_source(roots['candidate'], root, output/(label+'-operator.c'))
            operator_modules = [app/'controller'/(name+'.c') for name in
                                ('bc_capture', 'op_logic', 'integration_codec', 's3_drive')]
            operator_modules += [app/'subsystem3'/(name+'.c') for name in
                                 ('navigation_codec', 'ir_acquisition', 'ir_calibration', 'ir_reflectance',
                                  'line_detection', 'line_controller', 'bc_decoder', 'line_navigation')]
            binary = build(label, 'operator', [operator, shared/'op_trace.c', *operator_modules],
                           [fixtures/'integration', fixtures/'integration/fakes', app/'controller'], ['-pthread'])
            with (output/(label+'-operator.trace')).open('wb') as stream:
                subprocess.run([str(binary)], stdout=stream, check=True, timeout=180)
            mqtt = mqtt_source(roots['candidate'], output/(label+'-mqtt.c'))
            mqtt_sources = [app/'subsystem1'/(name+'.c') for name in
                            ('telemetry_codec', 'buddy1', 'command', 'mqtt_bridge', 'buddy1_test')]
            mqtt_sources += [root/'lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c']
            for name, ir_debug, imu_debug in (('mqtt', 1, 1), ('mqtt_plain', 0, 0),
                                             ('mqtt_production', 1, 0), ('mqtt_imu', 0, 1)):
                binary = build(label, name, [mqtt, *mqtt_sources],
                    [fixtures/'subsystem1', fixtures/'subsystem1/fakes', app/'subsystem1', root/'lib/liblwip/sysdepend/pico_rp2040'],
                    ['-pthread', '-DAPP_H', '-DAPP_TEST=1', '-DTM_MQTT=1',
                     f'-DAPP_IR_DEBUG={ir_debug}', f'-DAPP_IMU_DEBUG={imu_debug}',
                     '-include', str(fixtures/'subsystem1/host_kernel.h')])
                with (output/(label+'-'+name+'.trace')).open('wb') as stream:
                    subprocess.run([str(binary)], stdout=stream, check=True, timeout=180)
            binary = build(label, 'mqtt_receive', [fixtures/'compliance/mqtt_receive_test.c', *mqtt_sources],
                [fixtures/'subsystem1', fixtures/'subsystem1/fakes', app/'subsystem1', root/'lib/liblwip/sysdepend/pico_rp2040'],
                ['-pthread', '-DAPP_H', '-DAPP_TEST=1', '-DTM_MQTT=1',
                 '-DAPP_IR_DEBUG=1', '-DAPP_IMU_DEBUG=1', '-include', str(fixtures/'subsystem1/host_kernel.h')])
            with (output/(label+'-mqtt_receive.trace')).open('wb') as stdout, (output/(label+'-mqtt_receive.log')).open('wb') as stderr:
                subprocess.run([str(binary)], stdout=stdout, stderr=stderr, check=True, timeout=180)
            source = root / 'app_program/subsystem3'
            harness = root / 'testing/subsystem3'
            modules = ['ir_acquisition', 'ir_calibration', 'ir_reflectance', 'line_detection',
                       'line_controller', 'bc_decoder', 'line_navigation']
            # Each revision's harness is part of the recorded source SHA. The
            # semantic rows, rather than its PASS summary, are compared below.
            binary = build(label, 'navigation', [harness/'host_trace.c', *[source/(name+'.c') for name in modules]], [source, harness])
            with (output / (label + '-navigation.trace')).open('wb') as stdout, (output / (label + '-navigation.log')).open('wb') as stderr:
                subprocess.run([str(binary)], stdout=stdout, stderr=stderr, check=True, timeout=180)
            binaries[label] = build(label, 'replay', [roots['candidate']/'testing/compliance/barcode_replay_trace.c', source/'bc_decoder.c'], [root])
        for name, prefix in (('motion', b'M '), ('terrain', b'T '), ('terrain_production', b'T '), ('obstacle', b'O '),
                             ('runtime', b'RUNTIME '), ('mqtt', b'MQTT_'), ('mqtt_plain', b'MQTT_'),
                             ('mqtt_production', b'MQTT_'), ('mqtt_imu', b'MQTT_'),
                             ('mqtt_receive', b'MQSTORE '),
                             ('motion_storage', b'MSTORE '), ('barcode_work', b'BWORK '),
                             ('bcap_storage', b'BCSTORE '), ('operator', b'OPTRACE ')):
            before_trace = (output / ('baseline-' + name + '.trace')).read_bytes()
            after_trace = (output / ('candidate-' + name + '.trace')).read_bytes()
            if name.startswith('mqtt'):
                # Compare actual callbacks/fragments/payloads, never a harness
                # PASS summary; retain the complete stdout artifact separately.
                before_trace = b'\n'.join(line for line in before_trace.splitlines() if line.startswith(prefix))
                after_trace = b'\n'.join(line for line in after_trace.splitlines() if line.startswith(prefix))
            rows = sum(line.startswith(prefix) for line in before_trace.splitlines())
            minimum_rows = (802 if name == 'motion_storage' else
                            784 if name == 'barcode_work' else
                            619 if name == 'bcap_storage' else
                            2093 if name == 'mqtt_receive' else
                            # Complete fixed-baseline profile counts observed
                            # in run 44584191817; plain MQTT has 99 real rows.
                            {'mqtt': 269, 'mqtt_plain': 99,
                             'mqtt_production': 251, 'mqtt_imu': 116}.get(name, 11))
            matched = before_trace == after_trace and rows >= minimum_rows
            results['checks'].append(dict(name=name+'_semantic_trace', matched=matched, semantic_rows=rows,
                baseline_sha256=hashlib.sha256(before_trace).hexdigest(), candidate_sha256=hashlib.sha256(after_trace).hexdigest()))
            failed |= not matched
        before_debug = (output / 'baseline-control_debug.trace').read_bytes()
        after_debug = (output / 'candidate-control_debug.trace').read_bytes()
        debug_matches = before_debug == after_debug and len(before_debug.splitlines()) > 100
        results['checks'].append(dict(name='control_debug_wire_and_mutation_trace', matched=debug_matches,
                                      rows=len(before_debug.splitlines()),
                                      baseline_sha256=hashlib.sha256(before_debug).hexdigest(),
                                      candidate_sha256=hashlib.sha256(after_debug).hexdigest()))
        failed |= not debug_matches
        before = (output / 'baseline-navigation.trace').read_bytes()
        after = (output / 'candidate-navigation.trace').read_bytes()
        rows = sum(line.startswith((b'N ', b'B ', b'R ')) for line in before.splitlines())
        matches = before == after and rows > 100
        results['checks'].append(dict(name='navigation_semantic_trace', matched=matches, semantic_rows=rows,
                                      baseline_sha256=hashlib.sha256(before).hexdigest(), candidate_sha256=hashlib.sha256(after).hexdigest()))
        failed |= not matches
        # The accumulated audit preserves every unchanged assertion predicate.
        # Historical expectation failures remain failures while every case's
        # actual timestamped decoder work/state is compared independently.
        path = roots['candidate'] / 'testing/subsystem3/test_barcode_replay.py'
        replay = audit(path, binaries['baseline'], output/'recorded-replay', binaries['candidate'])
        results['checks'].append(dict(name='recorded_replay', cases=replay['cases'], mismatches=replay['mismatches'],
                                      expectation_failures=len(replay['failures']),
                                      matched=replay['cases'] >= 1008 and replay['mismatches'] == 0))
        failed |= replay['cases'] < 1008 or not replay['passed']
    except (AssertionError, OSError, subprocess.SubprocessError, ValueError) as exc:
        failed = True
        results['error'] = str(exc)
        print(f'FAIL: {exc}', file=sys.stderr)
    finally:
        results['passed'] = not failed
        (output / 'results.json').write_text(json.dumps(results, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(results, indent=2))
    return failed


if __name__ == '__main__':
    sys.exit(main())
