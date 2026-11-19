#!/usr/bin/env python3
"""Fail closed on missing, inconsistent or unobserved conditional-arm evidence.

Union only matching current-source observations. Preprocessor visibility is
neither runtime branch execution nor proof of physical timing.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from testing.compliance import inventory
from conditional_coverage import instrument
from build_targets import PROFILES
from baseline_fixture_coverage import PROFILES as BASELINE_FIXTURE_PROFILES, validate_receipt as validate_external_receipt

EXPECTED_PROFILES = {'host-gcc', 'host-clang'} | {'target-' + name for name in PROFILES} | BASELINE_FIXTURE_PROFILES
BRANCH_KEYS = ('id', 'source', 'line', 'kind', 'directive', 'depth')


def source_manifest(root):
    scope = inventory.read_json(root/'testing/compliance/inventory_scope.json')
    errors = inventory.repository_errors(root, scope)
    if errors:
        raise ValueError('; '.join(errors))
    hashes, branches = {}, {}
    for path in inventory.owned_files(root, scope):
        name = path.relative_to(root).as_posix()
        text = path.read_text(encoding='utf-8-sig')
        hashes[name] = hashlib.sha256(text.encode()).hexdigest()
        _, rows = instrument(text, name)
        if name in scope.get('mixed_files', {}):
            spans = inventory.mixed_region_spans(text, scope['mixed_files'][name])
            lines = [(text.count('\n', 0, start)+1, text.count('\n', 0, end-1)+1)
                     for start, end in spans]
            rows = [row for row in rows if any(a <= row['line'] <= b for a, b in lines)]
        for row in rows:
            if row['id'] in branches:
                raise ValueError('Duplicate expected arm identity')
            branches[row['id']] = row
    return hashes, branches


def combine(reports, hashes, expected_branches, revision, expected_profiles, root=None):
    """Validate each receipt; never trust its reported coverage/pass Boolean."""
    seen_profiles, observed, errors = set(), {}, []
    for name, report in reports:
        try:
            profile = report['profile']
            if Path(name).parts != (f'branches-{profile}-{revision}', 'results.json'):
                raise ValueError('Artifact directory does not match the declared profile/revision')
            if profile not in expected_profiles or profile in seen_profiles:
                raise ValueError('Unknown or duplicate profile: ' + str(profile))
            seen_profiles.add(profile)
            if report.get('revision') != revision or report['source_sha256'] != hashes:
                raise ValueError('Revision or complete source hash inventory differs')
            rows = report['branches']
            entries = {row['id']: row for row in rows}
            if len(entries) != len(rows) or set(entries) != set(expected_branches):
                raise ValueError('Conditional arm coverage inventory differs')
            for identity, row in entries.items():
                if any(row[key] != expected_branches[identity][key] for key in BRANCH_KEYS):
                    raise ValueError('Conditional arm metadata differs: ' + identity)
                if type(row['observed']) is not bool:
                    raise ValueError('Arm observation must be Boolean')
            samples = report['observations']
            if report['errors'] or not samples or any(row['returncode'] != 0 for row in samples):
                raise ValueError('Missing or failed preprocessing observation')
            pristine = report['pristine_observations']
            if (report['line_safety_errors'] or len(pristine) != len(samples) or
                    any(row['returncode'] != 0 for row in pristine)):
                raise ValueError('Missing, failed or unsafe pristine preprocessing')
            for original, sample in zip(pristine, samples):
                if ([part for part in original['command'] if part != '-dD'] != sample['command'] or
                        original['directory'] != sample['directory']):
                    raise ValueError('Pristine and instrumented compiler contexts differ')
            if profile in BASELINE_FIXTURE_PROFILES:
                validate_external_receipt(report, samples, hashes, revision, root)
            elif report.get('external_baseline') or report.get('dependency_observations'):
                raise ValueError('External dependencies require their distinct baseline-fixture profile')
            active = {identity for row in samples for identity in row['active']}
            if profile in BASELINE_FIXTURE_PROFILES and any(
                    identity not in entries or entries[identity]['source'] not in report['external_baseline']['fixture_sha256']
                    for identity in active):
                raise ValueError('Baseline-fixture observation contains a non-fixture marker')
            for identity, row in entries.items():
                if row['observed'] != (identity in active):
                    raise ValueError('Arm observation contradicts command receipts')
            for identity in entries.keys() & active:
                observed.setdefault(identity, []).append(profile)
        except (KeyError, TypeError, ValueError, OSError, subprocess.SubprocessError) as error:
            errors.append({'receipt': name, 'error': str(error)})
    missing_profiles = sorted(expected_profiles - seen_profiles)
    branches = [{**row, 'observed_profiles': sorted(observed.get(identity, []))}
                for identity, row in expected_branches.items()]
    missing_arms = [row for row in branches if not row['observed_profiles']]
    complete = not errors and not missing_profiles and not missing_arms
    return {'revision': revision, 'source_sha256': hashes,
            'expected_profiles': sorted(expected_profiles), 'received_profiles': sorted(seen_profiles),
            'missing_profiles': missing_profiles, 'errors': errors,
            'branches': branches, 'missing_arms': missing_arms,
            'coverage_complete': complete, 'status': 'PASS' if complete else 'INCOMPLETE',
            'limitations': ['Explicit preprocessing arms, including header guards.',
                           'No runtime branch execution, physical timing or compliance approval is inferred.',
                           'Failed observations cannot contribute to the union.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--receipts', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        root = args.root.resolve()
        hashes, branches = source_manifest(root)
        revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
        reports = [(str(path.relative_to(args.receipts)), json.loads(path.read_text(encoding='utf-8')))
                   for path in sorted(args.receipts.rglob('results.json'))]
        report = combine(reports, hashes, branches, revision, EXPECTED_PROFILES, root)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        report = {'status': 'INCOMPLETE', 'coverage_complete': False, 'error': str(error)}
    (args.output/'results.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    lines = ['# Conditional-compilation coverage', '', 'Status: **'+report['status']+'**.', '']
    if 'branches' in report:
        lines += [f"Observed {len(report['branches'])-len(report['missing_arms'])}/{len(report['branches'])} explicit arms.",
                  '', 'Missing profiles: '+(', '.join(report['missing_profiles']) or 'none')+'.', '',
                  '| Source | Line | Unobserved directive |', '| --- | ---: | --- |']
        for row in report['missing_arms']:
            directive = row['directive'].replace('|', '&#124;')
            lines.append(f"| {row['source']} | {row['line']} | `{directive}` |")
        lines += ['', 'Receipt errors:', '', '```json', json.dumps(report['errors'], indent=2), '```']
    else:
        lines.append(report['error'])
    (args.output/'SUMMARY.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(report['status']+': conditional-compilation union; see retained results.json and SUMMARY.md')
    return 0 if report['coverage_complete'] else 2


if __name__ == '__main__':
    sys.exit(main())
