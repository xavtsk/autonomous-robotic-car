"""Adversarial scope checks for header guard and declaration movement proofs."""
import unittest

from tools.compliance import verify_header_interfaces as verifier


class HeaderInterfaceTests(unittest.TestCase):
    guard = "TESTING_FAKE_H"
    original = "/** @file fake.h */\nvoid fake(void);\n"

    def guarded(self, body=None):
        return (f"#ifndef {self.guard}\n#define {self.guard}\n\n" +
                (self.original if body is None else body) +
                f"#endif /* {self.guard} */\n\n")

    def test_guard_only_change_is_accepted(self):
        verifier.compare_guard(self.original, self.guarded(), self.guard)

    def test_hidden_header_signature_change_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "beyond"):
            verifier.compare_guard(self.original, self.guarded(self.original.replace("void fake", "int fake")), self.guard)

    def test_mismatched_guard_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "delimiters"):
            verifier.compare_guard(self.original, self.guarded().replace("#define TESTING_FAKE_H", "#define WRONG_H"), self.guard)

    def test_existing_navigation_declaration_moves_to_header(self):
        before = '#include "test_core.h"\nvoid test_navigation (void);\n\nint main(void) { return 0; }\n'
        after = '#include "test_core.h"\n#include "test_navigation.h"\nint main(void) { return 0; }\n'
        verifier.compare_navigation(before, after, "testing/subsystem3/test_core.c", "/* contract */\n")
        with self.assertRaisesRegex(ValueError, "beyond"):
            verifier.compare_navigation(before, after.replace("return 0", "return 1"), "testing/subsystem3/test_core.c", "/* contract */\n")

    def test_documentation_cannot_hide_executable_code(self):
        with self.assertRaisesRegex(ValueError, "C tokens"):
            verifier.compare_navigation("", "", "testing/subsystem3/test_navigation.c", "int hidden;\n")

    def test_shared_header_allows_only_existing_void_interface(self):
        header = f"#ifndef {verifier.NAV_GUARD}\n#define {verifier.NAV_GUARD}\nvoid test_navigation(void);\n#endif\n"
        verifier.check_navigation_header(header)
        with self.assertRaisesRegex(ValueError, "prototype"):
            verifier.check_navigation_header(header.replace("void test_navigation", "int test_navigation"))


if __name__ == "__main__":
    unittest.main()
