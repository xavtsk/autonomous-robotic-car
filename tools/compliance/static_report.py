#!/usr/bin/env python3
"""Collect independent compliance audits without hiding incomplete reviews."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import sys


def audit_verdict(code, contents):
    """A successful unittest exit cannot hide skipped or undiscovered checks."""
    skips = re.findall(r"^.*\.\.\. skipped .*$", contents, re.MULTILINE)
    empty = bool(re.search(r"^Ran 0 tests\b", contents, re.MULTILINE))
    if code == 0 and (skips or empty or re.search(r"OK \(skipped=\d+\)", contents)):
        code = 2
    return dict(returncode=code,
                status='PASS' if code == 0 else 'INCOMPLETE' if code == 2 else 'FAIL',
                skipped=skips, empty_discovery=empty)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--formatter', default='/usr/bin/clang-format-18')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    commands = [
        ('shell_syntax', ['/usr/bin/bash', '-n', 'tools/compliance/install_arm.sh']),
        ('formatter_version', [args.formatter, '--version']),
        ('formatting', [sys.executable, 'tools/check_c_style.py', '--formatter', args.formatter]),
        ('inventory_integrity', [sys.executable, 'testing/compliance/inventory.py', '--check']),
        ('inventory_completeness', [sys.executable, 'testing/compliance/inventory.py', '--check', '--require-complete']),
        ('barr', [sys.executable, 'tools/compliance/barr_check.py', '--json']),
        ('barr_matrix', [sys.executable, 'tools/compliance/barr_matrix.py', '--check']),
        ('alias_batch', [sys.executable, 'tools/compliance/verify_alias_batch.py', '--json',
                         '--candidate', 'c37570cbd155249b000b0580336cd18e68ac3ea0']),
        ('format_batch', [sys.executable, 'tools/compliance/verify_format_batch.py', '--json',
                          '--candidate', '6326fcbecc1519f1332cda7f1ffb24d7a498e046']),
        ('function_docs_batch', [sys.executable, 'tools/compliance/verify_comment_batch.py',
                                 '--manifest', 'docs/compliance/FUNCTION_DOCS_BATCH1.json',
                                 '--candidate', '4f5dd616198455d9784e7a60f3bd1edc873fa89e']),
        ('debug_contracts_batch', [sys.executable, 'tools/compliance/verify_comment_batch.py',
                                   '--manifest', 'docs/compliance/CONTROL_DEBUG_CONTRACTS.json',
                                   '--candidate', '07ad498a49428b7362c0e5bcc010f66f1bb650d8']),
        ('module_rename_batch', [sys.executable, 'tools/compliance/verify_module_rename.py',
                                 '--candidate', 'b8328bda4d2d59113fe647b96c5956412520bf4a']),
        ('header_pairs_batch', [sys.executable, 'tools/compliance/verify_header_pairs.py',
                                '--candidate', 'df346fb6dced5376b9b9b5e32014d296829b2f24']),
        ('barcode_phase_batch', [sys.executable, 'tools/compliance/verify_barcode_phase_batch.py',
                                 '--candidate', '74b95107aa41d816bd143e8abf7ef4fe556f5b18']),
        ('mqtt_format_batch', [sys.executable, 'tools/compliance/verify_comment_batch.py',
                               '--manifest', 'docs/compliance/evidence/MQTT_STATE_FORMAT.json',
                               '--candidate', '3213d9b645926ed094ac70cf5f8a47e07782f05b']),
        ('motion_contracts_batch', [sys.executable, 'tools/compliance/verify_comment_batch.py',
                                    '--manifest', 'docs/compliance/MOTION_FUNCTION_CONTRACTS.json',
                                    '--candidate', '0c5c53be2cea1393730d2c42ad306d75708cf4df']),
        ('ir_contracts_batch', [sys.executable, 'tools/compliance/verify_comment_batch.py',
                                '--manifest', 'docs/compliance/IR_FUNCTION_CONTRACTS.json',
                                '--candidate', '2a10215d5e760a693bff10ef0b39358ff5347d40']),
        ('barcode_prune_batch', [sys.executable, 'tools/compliance/verify_barcode_phase_batch.py',
                                 '--manifest', 'docs/compliance/evidence/barcode-prune-extraction.json',
                                 '--candidate', '10ffc2c0dfeb800727cf0be49780867c64cab3e0']),
        ('fake_guards_batch', [sys.executable, 'tools/compliance/verify_header_interfaces.py',
                               '--manifest', 'docs/compliance/FAKE_HEADER_GUARDS.json',
                               '--candidate', '31c63a33f9c656d4fd214545d91ba3070592728e']),
        ('navigation_header_batch', [sys.executable, 'tools/compliance/verify_header_interfaces.py',
                                     '--manifest', 'docs/compliance/NAVIGATION_TEST_INTERFACE.json',
                                     '--candidate', 'ed3433451ed462d9e01b4204ea102062b44c9618']),
        ('barcode_edge_batch', [sys.executable, 'tools/compliance/verify_barcode_phase_batch.py',
                                '--manifest', 'docs/compliance/evidence/barcode-edge-extraction.json',
                                '--candidate', '02890251f55214e93aaf5149d0c5141364e7c962']),
        ('standalone_header_batch', [sys.executable, 'tools/compliance/verify_header_pairs.py',
                                     '--manifest', 'docs/compliance/HEADER_PAIRS_BATCH2.json',
                                     '--candidate', '31719bc44e413b97c3f5c3e947695502d6fac77d']),
        ('codec_contracts_batch', [sys.executable, 'tools/compliance/verify_comment_batch.py',
                                   '--manifest', 'docs/compliance/CODEC_FUNCTION_CONTRACTS.json',
                                   '--candidate', '0ba528b193872e2b515d44cb2d33dbdbb8585f1e']),
        ('calculation_contracts_batch', [sys.executable, 'tools/compliance/verify_comment_batch.py',
                                         '--manifest', 'docs/compliance/CALCULATION_FUNCTION_CONTRACTS.json',
                                         '--candidate', '4ce9b14aa4db6c26ea5f7546fb906d937c98d43f']),
        ('audit_tests', [sys.executable, '-m', 'unittest', 'discover', '-v', '-s', 'testing/compliance', '-p', 'test_*.py']),
    ]
    for label, path in (('cr1_runner_syntax', 'testing/change_requirement_1/run_all.sh'),
                        ('motion_regression_syntax', 'testing/subsystem2/run_regression.sh'),
                        ('barcode_regression_syntax', 'testing/subsystem3/run_regression.sh')):
        commands.append((label, ['/usr/bin/bash', '-n', path]))
    results = []
    for name, command in commands:
        with (output/(name+'.log')).open('w', encoding='utf-8') as stream:
            try:
                code = subprocess.run(command, cwd=root, stdout=stream, stderr=subprocess.STDOUT, timeout=300).returncode
            except (OSError, subprocess.SubprocessError) as exc:
                stream.write(str(exc)+'\n')
                code = 2
        verdict = audit_verdict(code, (output/(name+'.log')).read_text(encoding='utf-8'))
        results.append(dict(name=name, command=command, **verdict))
        print(verdict['status'], name, flush=True)
    (output/'results.json').write_text(json.dumps(dict(recorded_at=datetime.now(timezone.utc).isoformat(),
        revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        results=results),indent=2)+'\n',encoding='utf-8')
    return any(row['returncode'] != 0 for row in results)


if __name__ == '__main__':
    sys.exit(main())
