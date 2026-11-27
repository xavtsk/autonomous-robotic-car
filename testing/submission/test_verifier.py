"""Submission verdicts distinguish incomplete work, mission misses and regressions."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

from tools import verify_submission as verification


class VerificationTests(unittest.TestCase):
    def test_known_mission_miss_still_fails_strict(self):
        receipt = dict(complete=True, mission_passed=False, regression_passed=True,
                       total_cases=85, passed_cases=80)
        strict, regression, _ = verification.bypass_verdict(receipt)
        self.assertEqual((strict, regression), ('FAIL', 'PASS'))
        receipt['regression_passed'] = False
        self.assertEqual(verification.bypass_verdict(receipt)[:2], ('FAIL', 'FAIL'))

    def test_missing_or_aborted_matrix_is_incomplete(self):
        self.assertEqual(verification.bypass_verdict({'complete': False})[:2],
                         ('INCOMPLETE', 'INCOMPLETE'))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            row = {}
            receipt = root/'bypass.json'
            verification.attach_receipt(row, root, receipt, verification.bypass_verdict)
            self.assertEqual(row['regression'], 'INCOMPLETE')
            receipt.write_text(json.dumps({'complete': True}), encoding='utf-8')
            verification.attach_receipt(row, root, receipt, verification.bypass_verdict)
            self.assertEqual(row['strict'], 'INCOMPLETE')

    def test_replay_expectation_and_differential_are_independent(self):
        reference = {'cases': 2, 'assertions': 2}
        result = dict(cases=2, assertions=2, expectations_passed=False, failures=[{}], mismatches=0)
        self.assertEqual(verification.replay_verdict(result, reference, 1)[:2], ('FAIL', 'PASS'))
        result.update(expectations_passed=True, failures=[], mismatches=1)
        self.assertEqual(verification.replay_verdict(result, reference, 0)[:2], ('PASS', 'FAIL'))
        result['cases'] = 1
        self.assertEqual(verification.replay_verdict(result, reference, 0)[:2], ('INCOMPLETE', 'INCOMPLETE'))

    def test_missing_program_and_required_skip_create_an_incomplete_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            missing = verification.execute(root, 'missing', ['missing-submission-test-executable'])
            self.assertEqual(missing['strict'], 'INCOMPLETE')
            self.assertIsNone(missing['returncode'])
            self.assertTrue((root/missing['log']).is_file())
            skipped = verification.execute(root, 'skipped', [sys.executable, '-c',
                'print("test_required (...) ... skipped \'no device\'")'])
            self.assertEqual(skipped['regression'], 'INCOMPLETE')
            capture = ("test_real_successful_session_trace "
                       "(test_traces.TraceTests.test_real_successful_session_trace) ... skipped "
                       "'Optional local hardware capture is unavailable'\nOK (skipped=1)")
            optional = verification.execute(root, 'optional', [sys.executable, '-c', f'print({capture!r})'])
            self.assertEqual(optional['regression'], 'PASS')
            self.assertEqual(len(optional['skipped']), 1)

    def test_incomplete_overall_cannot_pass(self):
        self.assertEqual(verification.overall([]), 'INCOMPLETE')
        self.assertEqual(verification.overall(['PASS', 'INCOMPLETE', 'FAIL']), 'INCOMPLETE')
        self.assertEqual(verification.overall(['PASS', 'FAIL']), 'FAIL')
        self.assertEqual(verification.overall(['PASS', 'PASS']), 'PASS')

    def test_source_checker_execution_and_coverage_fail_closed(self):
        self.assertEqual(verification.source_verdict({'error': 'missing ledger'}, 2)[1], 'INCOMPLETE')
        report = dict(status='INCOMPLETE', files_scanned=304, gaps=list(verification.BARR_SOURCE_GAPS),
                      violations=0, findings=[], review_findings=0)
        self.assertEqual(verification.source_verdict(report, 2)[1], 'PASS')
        report['gaps'].append('missing owned source region')
        self.assertEqual(verification.source_verdict(report, 2)[1], 'INCOMPLETE')
        report.update(gaps=[], files_scanned=12)
        self.assertEqual(verification.source_verdict(report, 2)[1], 'INCOMPLETE')
        report.update(files_scanned=304, status='FAIL', violations=1, findings=[
            dict(path='firmware.c', rule='4.3.f', message='new finding', severity='violation')])
        self.assertEqual(verification.source_verdict(report, 1)[1], 'FAIL')


if __name__ == '__main__':
    unittest.main()
