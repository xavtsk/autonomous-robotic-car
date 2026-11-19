"""Verify recorded formatting evidence against source, including negative cases."""
import unittest

from tools.compliance import verify_format_batch as verifier


class FormatBatchTests(unittest.TestCase):
    before = '/* Keep this notice. */\nint value = 3;\n'
    after = '/** @file sample.c\n * @brief Count samples. */\n/* Keep this notice. */\nint value=3;\n/* End of file. */\n\n'

    def row(self, before=None, after=None):
        return {"before_lf_sha256": verifier.lf_digest(before or self.before),
                "after_lf_sha256": verifier.lf_digest(after or self.after),
                "token_sha256": verifier.fingerprint(self.before)}

    def test_recorded_batch_recomputes_source_proof(self):
        self.assertTrue(verifier.compare(self.before, self.after, self.row())["tokens_identical"])

    def test_checkout_line_endings_do_not_break_git_source_comparison(self):
        self.assertTrue(verifier.compare(self.before.replace('\n', '\r\n'), self.after, self.row())["tokens_identical"])

    def test_stale_or_modified_baseline_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Baseline source'):
            verifier.compare(self.before.replace('3', '4'), self.after, self.row())

    def test_changed_code_cannot_be_certified_by_updating_only_byte_hash(self):
        changed = self.after.replace('value=3', 'value=4')
        with self.assertRaisesRegex(ValueError, 'Preprocessing tokens'):
            verifier.compare(self.before, changed, self.row(after=changed))

    def test_lost_comment_cannot_be_certified_by_updating_only_byte_hash(self):
        changed = self.after.replace('/* Keep this notice. */\n', '')
        with self.assertRaisesRegex(ValueError, 'comment or notice'):
            verifier.compare(self.before, changed, self.row(after=changed))


if __name__ == '__main__':
    unittest.main()
