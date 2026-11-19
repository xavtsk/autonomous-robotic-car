"""Audit evidence must distinguish executed checks from missing coverage."""
import importlib.util
from pathlib import Path
import unittest

PATH = Path(__file__).resolve().parents[2] / 'tools/compliance/static_report.py'
SPEC = importlib.util.spec_from_file_location('static_report', PATH)
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


class StaticVerdictTests(unittest.TestCase):
    def test_skip_is_incomplete_even_with_successful_exit(self):
        result = REPORT.audit_verdict(0, "test_parser ... skipped 'unavailable'\nOK (skipped=1)\n")
        self.assertEqual(result['status'], 'INCOMPLETE')
        self.assertNotEqual(result['returncode'], 0)
        self.assertEqual(len(result['skipped']), 1)

    def test_empty_discovery_is_incomplete(self):
        self.assertEqual(REPORT.audit_verdict(0, 'Ran 0 tests\nOK\n')['status'], 'INCOMPLETE')

    def test_failure_remains_failure(self):
        self.assertEqual(REPORT.audit_verdict(1, 'FAIL\n')['status'], 'FAIL')


if __name__ == '__main__':
    unittest.main()
