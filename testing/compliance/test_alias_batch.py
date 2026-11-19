"""Independent checks for the scope of the mechanical alias verifier."""
import unittest

from tools.compliance import verify_alias_batch as verifier


class AliasBatchTests(unittest.TestCase):
    before = '#include <float.h>\n_Static_assert(sizeof(float) == 4, "size");\nfloat sample = 0.25f;\n'
    after = '#include "../app_types.h"\n#include <float.h>\nBUILD_ASSERT(sizeof(float32_t) == 4, sample_check_t);\nfloat32_t sample = 0.25f;\n'
    row = {"inserted_includes": ['#include "../app_types.h"\n'],
           "assertions": [{"identifier": "sample_check_t", "condition": "sizeof(float) == 4"}]}

    def test_aliases_and_assertions_reverse_exactly(self):
        self.assertTrue(verifier.compare(self.before, self.after, self.row)["tokens_identical"])

    def test_literals_and_comments_are_not_type_aliases(self):
        text = '"float32_t" /* float64_t */ float32_t value;'
        self.assertEqual(verifier.invert_aliases(text), '"float32_t" /* float64_t */ float value;')

    def test_changed_angle_header_is_not_hidden_by_alias_reversal(self):
        with self.assertRaisesRegex(ValueError, "Unexpected token"):
            verifier.compare(self.before, self.after.replace('<float.h>', '<float32_t.h>'), self.row)

    def test_different_float_literal_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unexpected token"):
            verifier.compare(self.before, self.after.replace('0.25f', '0.5f'), self.row)

    def test_weakened_compile_time_condition_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "condition changed"):
            verifier.compare(self.before, self.after.replace('== 4', '<= 4'), self.row)

    def test_lost_assertion_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "count changed"):
            verifier.compare(self.before, self.after.replace('BUILD_ASSERT(sizeof(float32_t) == 4, sample_check_t);', ''), self.row)


if __name__ == '__main__':
    unittest.main()
