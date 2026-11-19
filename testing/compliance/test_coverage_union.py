"""Evidence union must never turn stale, missing or failed data into coverage."""
from copy import deepcopy
from pathlib import Path
import re
import unittest
from unittest.mock import patch

from tools.compliance.coverage_union import combine, EXPECTED_PROFILES


class CoverageUnionTests(unittest.TestCase):
    def setUp(self):
        self.hashes = {'source.c': 'abc'}
        self.rows = {key: {'id': key, 'source': 'source.c', 'line': index+1,
                          'kind': 'if' if index == 0 else 'else',
                          'directive': '#if FEATURE' if index == 0 else '#else',
                          'depth': 1} for index, key in enumerate(('first', 'second'))}

    def receipt(self, profile, active):
        return {'profile': profile, 'revision': 'commit', 'source_sha256': self.hashes,
                'branches': [{**row, 'observed': key in active} for key, row in self.rows.items()],
                'observations': [{'returncode': 0, 'active': active,
                                  'command': ['cc', '-E', 'source.c'], 'directory': '/overlay'}],
                'pristine_observations': [{'returncode': 0, 'command': ['cc', '-E', '-dD', 'source.c'],
                                          'directory': '/overlay'}],
                'line_safety_errors': [], 'errors': [],
                'measurement_passed': True, 'coverage_complete': True}

    def union(self, *reports):
        return combine([(f"branches-{row['profile']}-commit/results.json", row) for row in reports],
                       self.hashes, self.rows, 'commit', {'target', 'host'})

    def test_matching_profiles_cover_complementary_arms(self):
        result = self.union(self.receipt('target', ['first']), self.receipt('host', ['second']))
        self.assertTrue(result['coverage_complete'])
        self.assertEqual(result['branches'][0]['observed_profiles'], ['target'])

    def test_missing_profile_and_missing_arm_remain_incomplete(self):
        result = self.union(self.receipt('target', ['first']))
        self.assertEqual(result['missing_profiles'], ['host'])
        self.assertEqual([row['id'] for row in result['missing_arms']], ['second'])

    def test_stale_sources_and_revisions_fail_without_contributing(self):
        for key, value in (('source_sha256', {'source.c': 'changed'}), ('revision', 'different')):
            bad = self.receipt('host', ['second']); bad[key] = value
            result = self.union(self.receipt('target', ['first']), bad)
            self.assertTrue(result['errors'])
            self.assertEqual([row['id'] for row in result['missing_arms']], ['second'])

    def test_failed_or_absent_command_cannot_claim_observation(self):
        for mutate in (lambda row: row['observations'][0].update(returncode=1),
                       lambda row: row.update(observations=[]),
                       lambda row: row.update(errors=['collector failure'])):
            bad = self.receipt('host', ['second']); mutate(bad)
            self.assertFalse(self.union(self.receipt('target', ['first']), bad)['coverage_complete'])

    def test_forged_arm_boolean_and_metadata_fail(self):
        for mutate in (lambda row: row['branches'][0].update(observed=True),
                       lambda row: row['branches'][0].update(line=77),
                       lambda row: row['branches'][0].update(observed=0),
                       lambda row: row['branches'].pop(),
                       lambda row: row['branches'].append(deepcopy(row['branches'][0]))):
            bad = self.receipt('host', ['second']); mutate(bad)
            result = self.union(self.receipt('target', ['first']), bad)
            self.assertFalse(result['coverage_complete'])
            self.assertTrue(result['errors'])

    def test_pristine_and_line_safety_evidence_cannot_be_skipped(self):
        for mutate in (lambda row: row.pop('pristine_observations'),
                       lambda row: row.update(pristine_observations=[]),
                       lambda row: row['pristine_observations'][0].update(returncode=1),
                       lambda row: row['pristine_observations'][0].update(command=['wrong']),
                       lambda row: row.update(line_safety_errors=['__LINE__ condition'])):
            bad = self.receipt('host', ['second']); mutate(bad)
            result = self.union(self.receipt('target', ['first']), bad)
            self.assertFalse(result['coverage_complete'])
            self.assertTrue(result['errors'])

    def test_unknown_or_duplicate_profile_is_not_counted_twice(self):
        for profile in ('target', 'unrequested'):
            result = self.union(self.receipt('target', ['first']), self.receipt(profile, ['second']))
            self.assertFalse(result['coverage_complete'])
            self.assertTrue(result['errors'])

    def test_empty_input_never_passes(self):
        result = self.union()
        self.assertFalse(result['coverage_complete'])
        self.assertEqual(len(result['missing_profiles']), 2)

    def test_receipt_cannot_impersonate_another_artifact_profile(self):
        for name in ('branches-target-commit/results.json',
                     'branches-host-other/results.json',
                     'branches-host-commit/nested/results.json'):
            result = combine([(name, self.receipt('host', ['first', 'second']))],
                             self.hashes, self.rows, 'commit', {'host'})
            self.assertFalse(result['coverage_complete'])
            self.assertIn('Artifact directory', result['errors'][0]['error'])

    def test_unowned_instrumentation_markers_do_not_change_denominator(self):
        result = self.union(self.receipt('target', ['first', 'imported']),
                            self.receipt('host', ['second']))
        self.assertTrue(result['coverage_complete'])
        self.assertEqual(len(result['branches']), 2)

    def test_external_dependencies_require_their_separate_profile_and_checkout(self):
        bad = self.receipt('host', ['first', 'second'])
        bad['external_baseline'] = {'revision': '4ad2cc42595722cedb47102e4343298339040d40'}
        self.assertTrue(self.union(bad)['errors'])
        profile = 'host-baseline-fixtures-gcc'
        receipt = self.receipt(profile, ['first', 'second'])
        result = combine([(f'branches-{profile}-commit/results.json', receipt)],
                         self.hashes, self.rows, 'commit', {profile})
        self.assertFalse(result['coverage_complete'])
        self.assertIn('source checkout', result['errors'][0]['error'])

    def test_baseline_fixture_profile_cannot_contribute_helper_or_unknown_markers(self):
        profile = 'host-baseline-fixtures-gcc'
        self.rows['second']['source'] = 'helper.h'
        for active in (['first', 'second'], ['first', 'unknown']):
            receipt = self.receipt(profile, active)
            receipt['external_baseline'] = {'fixture_sha256': {'source.c': 'abc'}}
            with patch('tools.compliance.coverage_union.validate_external_receipt'):
                result = combine([(f'branches-{profile}-commit/results.json', receipt)],
                                 self.hashes, self.rows, 'commit', {profile}, Path('.'))
            self.assertFalse(result['coverage_complete'])
            self.assertIn('non-fixture marker', result['errors'][0]['error'])

    def test_required_target_profiles_match_the_explicit_workflow_matrix(self):
        workflow = (Path(__file__).resolve().parents[2]/'.github/workflows/compliance.yml').read_text()
        declarations = re.findall(r'^\s+profile: \[([a-z0-9_, ]+)\]\s*$', workflow, re.MULTILINE)
        self.assertEqual(len(declarations), 1, 'A changed matrix format requires explicit audit review')
        actual = {'target-' + name.strip() for name in declarations[0].split(',')}
        self.assertEqual(actual | {'host-gcc', 'host-clang', 'host-baseline-fixtures-gcc',
                                   'host-baseline-fixtures-clang'}, EXPECTED_PROFILES)


if __name__ == '__main__':
    unittest.main()
