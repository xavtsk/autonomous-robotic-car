"""Runner verdict regressions; no C compiler, broker or robot is executed."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('run_host', ROOT/'tools/compliance/run_host.py')
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


class RunnerVerdictTests(unittest.TestCase):
    def test_swallowed_sanitizer_report_still_fails(self):
        for report in ('ERROR: AddressSanitizer: heap-buffer-overflow',
                       'ERROR: LeakSanitizer: detected memory leaks',
                       'runtime error: shift exponent 32 is too large',
                       'AddressSanitizer:DEADLYSIGNAL'):
            with self.subTest(report=report):
                result = RUNNER.classify_output(0, report + '\nknown simulation miss\n')
                self.assertEqual(result['status'], 'FAIL')
                self.assertTrue(result['sanitizer_error'])

    def test_only_named_optional_local_capture_may_be_unavailable(self):
        optional = "test_real_successful_session_trace (test_traces.TraceTests.test_real_successful_session_trace) ... skipped 'Optional local hardware capture is unavailable'"
        output = optional + '\nRan 116 tests in 5s\nOK (skipped=1)\n'
        result = RUNNER.classify_output(0, output)
        self.assertEqual(result['status'], 'PASS')
        self.assertEqual(result['optional_unavailable'], [optional])
        missing_mqtt = "test_real_mqtt (test_service.TransportTests.test_real_mqtt) ... skipped 'Mosquitto unavailable'\nOK (skipped=1)"
        self.assertEqual(RUNNER.classify_output(0, missing_mqtt)['status'], 'FAIL')
        self.assertEqual(RUNNER.classify_output(0, 'OK (skipped=1)')['status'], 'FAIL')

    def test_no_tests_and_nonzero_exit_cannot_pass(self):
        self.assertEqual(RUNNER.classify_output(0, 'Ran 0 tests in 0.000s\nOK')['status'], 'FAIL')
        self.assertEqual(RUNNER.classify_output(1, 'PASS: old output')['status'], 'FAIL')
        self.assertEqual(RUNNER.classify_output(0, 'Ran 7 tests in 1s\nOK')['status'], 'PASS')


if __name__ == '__main__':
    unittest.main()
