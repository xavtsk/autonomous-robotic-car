#!/usr/bin/env python3
"""Compare real baseline/candidate semantic traces and every recorded replay."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from replay_audit import audit


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
            suites = {
                'motion': (2, ['motion', 'ctrl_motion', 'motion_pack', 'estimate_motion', 'speed_pid'],
                           fixtures/'subsystem3', ['-include', str(fixtures/'subsystem3/motion_kernel_stub.h')]),
                'terrain': (4, ['imu_sensor', 'imu_calibration', 'tilt_estimation', 'hump_detection', 'events_motion', 'terr_data'],
                            fixtures/'subsystem4', ['-DAPP_IMU_DEBUG=1']),
                'obstacle': (5, ['profile_obstacle', 'scan_obstacle', 'avoidance_planner', 'line_recovery', 'avoid_obstacle', 's5_codec', 'ultrasonic_sensor'],
                             fixtures/'subsystem5', ['-DS5_HOST_TEST']),
            }
            for name, (number, modules, harness_dir, flags) in suites.items():
                directory = app / f'subsystem{number}'
                binary = build(label, name, [shared/(name+'_trace.c'), *[directory/(module+'.c') for module in modules]],
                               [directory, harness_dir], flags)
                with (output / (label + '-' + name + '.trace')).open('wb') as stream:
                    subprocess.run([str(binary)], stdout=stream, check=True, timeout=120)
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
        for name, prefix in (('motion', b'M '), ('terrain', b'T '), ('obstacle', b'O ')):
            before_trace = (output / ('baseline-' + name + '.trace')).read_bytes()
            after_trace = (output / ('candidate-' + name + '.trace')).read_bytes()
            rows = sum(line.startswith(prefix) for line in before_trace.splitlines())
            matched = before_trace == after_trace and rows > 10
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
