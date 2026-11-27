#!/usr/bin/env python3
"""Run the submitted Linux host suites and record observed and regression verdicts."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess
import sys
import tempfile

if __package__:
    from .compliance.run_host import classify_output
else:
    from compliance.run_host import classify_output


ROOT = Path(__file__).resolve().parents[1]
DEPLOYED = '4ad2cc42595722cedb47102e4343298339040d40'
REPLAY_REFERENCE = ROOT / 'docs/compliance/evidence/post-demo-barcode-replay-20261127.json'
BASELINE_FILES = (
    'app_program/subsystem3/bc_decoder.c',
    'app_program/subsystem3/bc_decoder.h',
    'app_program/subsystem3/bc_quiet.h',
    'testing/subsystem3/replay_barcode_record.c',
)
# Source-only findings actually present in the submitted 8131e1a host fixtures.
BARR_FIXTURES = {
    ('testing/subsystem2/wheel_adapter_main.c', '4.3.f', 'Source includes another C source'),
    ('testing/subsystem2/wheel_kernel.h', '6.1.f', 'Macro PWM_CHx_DIV contains lowercase letters'),
    ('testing/subsystem3/motion_kernel_stub.h', '6.1.f', 'Macro XOSC_MHz contains lowercase letters'),
    ('testing/subsystem5/fakes/sys/sysdef.h', '6.1.f', 'Macro PWM_CHx_DIV contains lowercase letters'),
}
BARR_SOURCE_GAPS = {
    '4 mixed-ownership file(s): only recorded owned regions scanned; surrounding imported bodies are not BARR edit targets',
    'No Clang AST manifest supplied; function/type/profile coverage is unknown',
}
PASS, FAIL, INCOMPLETE = 'PASS', 'FAIL', 'INCOMPLETE'


def overall(verdicts):
    if not verdicts or INCOMPLETE in verdicts:
        return INCOMPLETE
    return FAIL if FAIL in verdicts else PASS


def execute(output, name, command, requirements=()):
    """Run each suite once. Keep missing or skipped tests visible to the caller."""
    command = list(map(str, command))
    print('Checking ' + name, flush=True)
    log = output / (name + '.log')
    missing = [program for program in (command[0], *requirements) if not shutil.which(program)]
    code = None
    note = ''
    with log.open('w', encoding='utf-8') as stream:
        print('$ ' + shlex.join(command), file=stream, flush=True)
        if missing:
            note = 'Missing ' + ', '.join(missing)
        else:
            try:
                code = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                      env=dict(os.environ, PYTHONOPTIMIZE='', PYTHONUNBUFFERED='1'),
                                      timeout=900).returncode
            except (OSError, subprocess.TimeoutExpired) as error:
                note = str(error)
        if note:
            print(note, file=stream)
    captured = log.read_text(encoding='utf-8', errors='replace').partition('\n')[2]
    observed = classify_output(code, captured)
    if code is None or observed['empty_discovery'] or observed['skipped'] != observed['optional_unavailable']:
        status = INCOMPLETE
    else:
        status = observed['status']
    return dict(name=name, command=command, returncode=code, log=log.name, strict=status,
                regression=status, note=note, skipped=observed['skipped'])


def bypass_verdict(receipt):
    if not receipt.get('complete'):
        return INCOMPLETE, INCOMPLETE, 'Incomplete bypass matrix'
    return (PASS if receipt['mission_passed'] else FAIL,
            PASS if receipt['regression_passed'] else FAIL,
            f"{receipt['passed_cases']}/{receipt['total_cases']} simulated missions succeeded")


def replay_verdict(receipt, reference, raw_code):
    """Compare outputs independently of the submitted test expectations."""
    if (receipt.get('execution_error') or receipt['cases'] < reference['cases']
            or receipt['assertions'] < reference['assertions'] or raw_code not in (0, 1)):
        return INCOMPLETE, INCOMPLETE, 'Incomplete recorded replay'
    failures = receipt['failures']
    strict = PASS if receipt['expectations_passed'] and raw_code == 0 else FAIL
    regression = PASS if receipt['mismatches'] == 0 else FAIL
    return (strict, regression, f"{receipt['cases']} cases, {len(failures)} failed expectations, "
            f"{receipt['mismatches']} output differences from {DEPLOYED[:7]}")


def attach_receipt(row, output, path, interpret, *args):
    try:
        strict, regression, note = interpret(json.loads(path.read_text(encoding='utf-8')), *args)
    except (OSError, ValueError, KeyError, TypeError) as error:
        strict = regression = INCOMPLETE
        note = f'Invalid or missing receipt: {error}'
    row.update(strict=strict, regression=regression, note=note, evidence=str(path.relative_to(output)))


def baseline_replay(output):
    """Compile the deployed replay harness against its own four immutable blobs."""
    directory = output / 'baseline'
    directory.mkdir()
    log = output / 'baseline-inputs.log'
    commands = []
    try:
        for source in BASELINE_FILES:
            command = ['git', 'cat-file', 'blob', DEPLOYED + ':' + source]
            commands.append(command)
            result = subprocess.run(command, cwd=ROOT, capture_output=True, check=True)
            (directory / Path(source).name).write_bytes(result.stdout)
    except (OSError, subprocess.CalledProcessError) as error:
        note = f'Cannot read the deployed input blobs. Use a full clone. {error}'
        log.write_text('\n'.join(map(shlex.join, commands)) + '\n' + note, encoding='utf-8')
        return dict(name='baseline-inputs', command=commands, log=log.name, strict=INCOMPLETE,
                    regression=INCOMPLETE, note=note, returncode=None)
    log.write_text('\n'.join(map(shlex.join, commands)) + '\n', encoding='utf-8')
    return execute(output, 'baseline-decoder', ['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                   '-pedantic', directory/'replay_barcode_record.c', directory/'bc_decoder.c',
                   '-o', directory/'replay'], ())


def source_verdict(report, code):
    if report.get('error') or code not in (0, 1, 2):
        return INCOMPLETE, INCOMPLETE, 'Source checker could not complete'
    findings = [(item['path'], item['rule'], item['message']) for item in report['findings']
                if item['severity'] == 'violation']
    status = report['status']
    if ({PASS: 0, FAIL: 1, INCOMPLETE: 2}.get(status) != code
            or len(findings) != report['violations'] or report['files_scanned'] < 304):
        return status, INCOMPLETE, 'Incomplete or inconsistent source-check receipt'
    for gap in set(report['gaps']) - BARR_SOURCE_GAPS:
        manual = re.fullmatch(r'(\d+) ledger entries remain unreviewed or pending deviation', gap)
        if not manual or int(manual[1]) > 54:
            return status, INCOMPLETE, 'New coverage gap: ' + gap
    # A second occurrence of an inherited problem is still a finding.
    known = len(findings) == len(set(findings)) and set(findings) <= BARR_FIXTURES
    return status, PASS if known else FAIL, f"{len(findings)} source violations, {report['review_findings']} review findings"


def source_review(output):
    row = execute(output, 'barr-source', [sys.executable, 'tools/compliance/barr_check.py', '--json'])
    row['category'] = 'source review'
    try:
        report = json.loads((output/row['log']).read_text(encoding='utf-8').split('\n', 1)[1])
        row['strict'], row['regression'], row['note'] = source_verdict(report, row['returncode'])
    except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
        row.update(strict=INCOMPLETE, regression=INCOMPLETE, note=str(error))
    return row


def run_suites(output):
    py = sys.executable
    suites = []
    def make(name, folder, targets, makefile=None, build=None):
        build = build or output/'bin'/name
        cmd = ['make', '-C', 'testing/' + folder]
        if makefile:
            cmd += ['-f', makefile]
        row = execute(output, name, [*cmd, *targets, 'CC=cc', 'PYTHON=' + py, 'BUILD=' + str(build)],
                      requirements=('cc', 'nm', 'sed'))
        suites.append(row)
        return row

    for n in range(1, 6):
        name = f'subsystem{n}'
        make(name, name, ['test', 'codec', 'drive', 'feedback'] if n == 3 else ['test'])
    bypass_file = output/'bin/subsystem5/bypass-results.json'
    bypass = dict(name='bypass-missions', log='subsystem5.log', command=suites[-1]['command'])
    attach_receipt(bypass, output, bypass_file, bypass_verdict)
    suites.append(bypass)

    make('vehicle-controller', 'integration', ['test'], 'controller_test.mk')
    make('device-diagnostics', 'diagnostics', ['test', 'smoke', 'disabled'], 'firmware_test.mk')
    make('wifi', 'network_wifi', ['test'])
    for folder in ('dashboard', 'diagnostics', 'change_requirement_1', 'submission'):
        requirements = ('mosquitto', 'mosquitto_pub', 'mosquitto_sub', 'cc') if folder == 'dashboard' else ()
        suites.append(execute(output, 'python-' + folder, [py, '-m', 'unittest', 'discover', '-v',
                              '-s', 'testing/' + folder, '-p', 'test_*.py'], requirements))
    suites.append(execute(output, 'dashboard-ui', ['node', '--test', 'testing/dashboard/test_frontend_controls.js']))
    suites.append(execute(output, 'field-widths', [py, 'testing/change_requirement_1/check_widths.py']))
    suites.append(execute(output, 'barr-matrix', [py, 'tools/compliance/barr_matrix.py', '--check']))
    suites.append(execute(output, 'barr-checker-tests', [py, '-m', 'unittest', 'discover', '-v',
                          '-s', 'testing/compliance', '-p', 'test_barr*.py']))
    suites.append(source_review(output))

    candidate_dir = output/'bin/replay'
    make('replay-margins', 'subsystem3', ['replay-margins'], build=candidate_dir)
    build = baseline_replay(output)
    suites.append(build)
    candidate = candidate_dir/'replay_barcode_record'
    suite = 'testing/subsystem3/test_barcode_replay.py'
    raw = execute(output, 'replay-expectations', [py, suite, candidate], requirements=(str(candidate),))
    suites.append(raw)
    reference = json.loads(REPLAY_REFERENCE.read_text(encoding='utf-8'))
    paired = output/'baseline/replay'
    audit = execute(output, 'replay-audit', [py, 'tools/compliance/replay_audit.py', '--suite', suite,
                    '--executable', candidate, '--paired-executable', paired, '--output', output/'replay'],
                    requirements=(str(candidate), str(paired)))
    attach_receipt(audit, output, output/'replay/results.json', replay_verdict, reference, raw['returncode'])
    # The fail-fast script separately checks CLI padding arguments. Any failure
    # here remains gating even when the baseline produced identical output.
    suites.append(audit)
    return suites


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='New output directory (default: unique run under build/)')
    parser.add_argument('--strict', action='store_true', help='Also return non-zero on inherited mission failures')
    args = parser.parse_args()
    if args.output:
        output = args.output.resolve()
        try:
            output.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            parser.error('--output must name a new directory')
    else:
        parent = ROOT/'build/submission-verification'
        parent.mkdir(parents=True, exist_ok=True)
        output = Path(tempfile.mkdtemp(prefix='run-', dir=parent))
    print('Submission verification: ' + str(output), flush=True)
    report = dict(recorded_at=datetime.now(timezone.utc).isoformat(), platform=platform.platform(),
                  deployed_replay_baseline=DEPLOYED, suites=[])
    try:
        report['revision'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        report['working_tree_changes'] = subprocess.check_output(
            ['git', 'status', '--porcelain', '--untracked-files=normal'], cwd=ROOT, text=True).splitlines()
        if sys.platform != 'linux':
            raise RuntimeError('Linux is required for the host executables and local MQTT broker.')
        report['suites'] = run_suites(output)
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        report['error'] = str(error)
    report['regression'] = overall([r['regression'] for r in report['suites']])
    report['strict'] = overall([r['strict'] for r in report['suites'] if r.get('category') != 'source review'])
    if 'error' in report:
        report['regression'] = report['strict'] = INCOMPLETE
    summary = ['| Suite | Strict | Regression | Log |', '| --- | --- | --- | --- |']
    for row in report['suites']:
        detail = (' — ' + row['note']) if row['note'] else ''
        summary.append(f"| {row['name']}{detail} | {row['strict']} | {row['regression']} | [log]({row['log']}) |")
    if 'error' in report:
        summary.append('\n' + report['error'])
    summary.append(f"\nStrict behaviour: **{report['strict']}**. Regression: **{report['regression']}**.")
    summary.append(f"Revision: `{report.get('revision', 'unavailable')}`.")
    printed = '\n'.join(summary) + '\n'
    print(printed)
    (output/'summary.md').write_text(printed, encoding='utf-8')
    (output/'results.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    verdict = report['strict'] if args.strict else report['regression']
    if INCOMPLETE in (report['strict'], report['regression']):
        return 2
    return 1 if FAIL in (verdict, report['regression']) else 0


if __name__ == '__main__':
    sys.exit(main())
