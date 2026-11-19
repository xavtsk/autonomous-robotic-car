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
