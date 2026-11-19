"""Bounded inverse-proof regressions; no native decoder execution."""
import importlib.util
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/compliance'))
SPEC = importlib.util.spec_from_file_location('barcode_proof', ROOT/'tools/compliance/verify_barcode_phase_batch.py')
PROOF = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROOF)
HELPERS = [{'phase': 'RW_PRIMARY', 'helper': 'work_primary', 'switch_exit_count': 2}]
BEFORE = '''void dispatch(void) {
    ++work_units;
    switch (phase) { case RW_PRIMARY:
        if (stop) { phase = 7; break; }
        next(p_r); break;
    }
}'''
AFTER = '''static void work_primary(s3_barcode_record_t *p_r) {
    s3_barcode_work_t *p_w = &p_r->work;
    if (stop) { phase = 7; return; }
    next(p_r); return;
}
void dispatch(void) {
    ++work_units;
    switch (phase) { case RW_PRIMARY: work_primary(p_r); break; }
}'''


class PhaseProofTests(unittest.TestCase):
    def test_inverse_recovers_complete_original(self):
        self.assertEqual(PROOF.tokens(BEFORE), PROOF.recover(AFTER, HELPERS))

    def test_comments_and_spacing_do_not_change_c_tokens(self):
        candidate = AFTER.replace('next(p_r)', 'next /* observe */ ( p_r )')
        self.assertEqual(PROOF.tokens(BEFORE), PROOF.recover(candidate, HELPERS))

    def test_body_and_dispatcher_drift_are_not_hidden(self):
        for candidate in (AFTER.replace('phase = 7', 'phase = 8'),
                          AFTER.replace('++work_units', '--work_units')):
            self.assertNotEqual(PROOF.tokens(BEFORE), PROOF.recover(candidate, HELPERS))

    def test_changed_binding_and_nonvoid_return_fail_closed(self):
        for candidate in (AFTER.replace('&p_r->work', '&other->work'),
                          AFTER.replace('return;', 'return 1;', 1)):
            with self.assertRaises(ValueError):
                PROOF.recover(candidate, HELPERS)

    def test_missing_or_duplicate_phase_call_fails_closed(self):
        for candidate in (AFTER.replace('work_primary(p_r)', 'other(p_r)'),
                          AFTER+'case RW_PRIMARY: work_primary(p_r); break;'):
            with self.assertRaises(ValueError):
                PROOF.recover(candidate, HELPERS)

    def test_unrecorded_extra_exit_fails_closed(self):
        with self.assertRaises(ValueError):
            PROOF.recover(AFTER.replace('next(p_r);', 'if (extra) return; next(p_r);'), HELPERS)


if __name__ == '__main__':
    unittest.main()
