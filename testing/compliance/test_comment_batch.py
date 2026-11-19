"""Trust-boundary tests for documentation-only preservation receipts."""
import unittest

from tools.compliance import verify_comment_batch as verifier


class CommentBatchTests(unittest.TestCase):
    def compare(self, before, after):
        return verifier.compare(before, after, {
            "before_lf_sha256": verifier.digest(before),
            "after_lf_sha256": verifier.digest(after)})

    def test_inserted_docs_keep_code_and_notices(self):
        self.compare('/* notice */\nint x;\n/* end */',
                     '/** module */\n/* notice */\n/** variable */\nint x;\n/* end */')

    def test_reordered_or_changed_notices_fail(self):
        for after in ('/* end */ int x; /* notice */',
                      '/* new notice */ int x; /* end */'):
            with self.assertRaisesRegex(ValueError, "comment"):
                self.compare('/* notice */ int x; /* end */', after)

    def test_code_changes_are_not_hidden_by_truthy_manifest_flags(self):
        with self.assertRaisesRegex(ValueError, "tokens"):
            self.compare('int x = 1;', 'int x = 2;')

    def test_macro_adjacency_and_directive_boundaries_are_bound(self):
        for before, after in (('#define F(x) x\n', '#define F (x) x\n'),
                              ('#define X 1\nint x;', '#define X 1 int x;')):
            with self.assertRaisesRegex(ValueError, "semantics"):
                self.compare(before, after)

    def test_comment_line_splice_and_trigraph_changes_fail(self):
        for before, after in (('// note\nint x;', '// note\\\nint x;'),
                              ('int x; ? ? /\nint y;', 'int x; ??/\nint y;')):
            with self.assertRaises(ValueError):
                self.compare(before, after)

    def test_source_hash_mismatch_fails_before_proof(self):
        with self.assertRaisesRegex(ValueError, "Baseline hash"):
            verifier.compare('int x;', 'int x;', {
                'before_lf_sha256': '0'*64, 'after_lf_sha256': verifier.digest('int x;')})


if __name__ == '__main__':
    unittest.main()
