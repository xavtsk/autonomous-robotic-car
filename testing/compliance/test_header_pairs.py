"""Fail-closed checks for a no-export header batch."""
import unittest

from tools.compliance import verify_header_pairs as verifier


class HeaderPairTests(unittest.TestCase):
    before = '#include <assert.h>\n\nint main(void) { return 0; }\n'
    after = '#include "sample.h"\n' + before
    row = {"header_path": "testing/sample.h", "include_line": 1}

    def test_single_added_include_inverts_exactly(self):
        verifier.compare_source(self.before, self.after, self.row)

    def test_changed_test_expression_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "beyond"):
            verifier.compare_source(self.before, self.after.replace("return 0", "return 1"), self.row)

    def test_duplicate_include_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "beyond"):
            verifier.compare_source(self.before, '#include "sample.h"\n' + self.after, self.row)

    def test_comments_do_not_add_an_exported_api(self):
        verifier.check_empty_header('/** @file sample.h */\n#ifndef SAMPLE_H\n#define SAMPLE_H\n#endif /* SAMPLE_H */\n', "SAMPLE_H")

    def test_hidden_declaration_or_extra_macro_is_rejected(self):
        for insertion in ("int sample(void);", "#define VALUE 1", '#include "other.h"'):
            with self.subTest(insertion=insertion), self.assertRaisesRegex(ValueError, "beyond"):
                verifier.check_empty_header('#ifndef SAMPLE_H\n#define SAMPLE_H\n' + insertion + '\n#endif\n', "SAMPLE_H")


if __name__ == "__main__":
    unittest.main()
