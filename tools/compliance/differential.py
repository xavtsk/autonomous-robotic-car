#!/usr/bin/env python3
"""Compare real baseline/candidate semantic traces and every recorded replay."""
import argparse
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace


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

    def build(label, name, sources, includes=()):
        binary = output / (label + '-' + name)
        command = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-O2', '-g',
                   *['-I' + str(path) for path in includes], *map(str, sources), '-o', str(binary)]
        with (output / (label + '-' + name + '-build.log')).open('w') as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)
        return binary

    try:
        trace_harness = roots['candidate'] / 'testing/compliance/control_debug_trace.c'
        for label, root in roots.items():
            debug_binary = build(label, 'control_debug', [trace_harness, root / 'app_program/controller/control_debug.c'], [root])
            with (output / (label + '-control_debug.trace')).open('wb') as stream:
                subprocess.run([str(debug_binary)], stdout=stream, check=True, timeout=30)
            source = root / 'app_program/subsystem3'
            harness = root / 'testing/subsystem3'
            modules = ['ir_acquisition', 'ir_calibration', 'ir_reflectance', 'line_detection',
                       'line_controller', 'bc_decoder', 'line_navigation']
            # Each revision's harness is part of the recorded source SHA. The
            # semantic rows, rather than its PASS summary, are compared below.
            binary = build(label, 'navigation', [harness/'host_trace.c', *[source/(name+'.c') for name in modules]], [source, harness])
            with (output / (label + '-navigation.trace')).open('wb') as stdout, (output / (label + '-navigation.log')).open('wb') as stderr:
                subprocess.run([str(binary)], stdout=stdout, stderr=stderr, check=True, timeout=180)
            binaries[label] = build(label, 'replay', [harness/'replay_barcode_record.c', source/'bc_decoder.c'], [source])
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
        # Reuse the existing fixture enumerator and expectations unchanged.
        # Every subprocess invocation receives identical samples and padding.
        # Preserve each full output, timestamp, rejection and unfinished state.
        path = roots['candidate'] / 'testing/subsystem3/test_barcode_replay.py'
        spec = importlib.util.spec_from_file_location('compliance_replay', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cases, mismatches = 0, 0
        with (output / 'recorded-replay.jsonl').open('w', encoding='utf-8') as evidence:
            def paired(command, **kwargs):
                nonlocal cases, mismatches
                context = inspect.currentframe().f_back.f_locals
                kwargs['timeout'] = 15
                baseline = subprocess.run([str(binaries['baseline']), *command[1:]], **kwargs)
                candidate = subprocess.run([str(binaries['candidate']), *command[1:]], **kwargs)
                matched = (baseline.returncode, baseline.stdout, baseline.stderr) == (candidate.returncode, candidate.stdout, candidate.stderr)
                cases += 1
                mismatches += not matched
                details = {key: context[key] for key in ('name', 'raw', 'fraction', 'reverse', 'lead', 'trail') if key in context}
                evidence.write(json.dumps(dict(case=cases, inputs=details,
                    input_sha256=hashlib.sha256(kwargs['input'].encode()).hexdigest(), matched=matched,
                    baseline=baseline.stdout, candidate=candidate.stdout)) + '\n')
                return candidate
            module.subprocess = SimpleNamespace(run=paired)
            original_argv = sys.argv
            try:
                sys.argv = [str(path), str(binaries['candidate'])]
                module.main()
            finally:
                sys.argv = original_argv
        results['checks'].append(dict(name='recorded_replay', cases=cases, mismatches=mismatches,
                                      matched=cases >= 1008 and mismatches == 0))
        failed |= cases < 1008 or mismatches != 0
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
