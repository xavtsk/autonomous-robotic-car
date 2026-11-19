from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from tools import check_c_style as style


class StyleCheckTests(unittest.TestCase):
    def test_module_name_collision_and_missing_header(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "motion_alpha.c"
            header = Path(directory) / "motion_alpha.h"
            other = Path(directory) / "motion_another.c"
            source.write_text('#include "motion_alpha.h"\n')
            header.write_text("")
            other.write_text("")
            self.assertEqual(style.module_errors([source, header]), [])
            errors = style.module_errors([source, header, other])
            self.assertEqual(len(errors), 3)
            self.assertTrue(any("collide" in error for error in errors))
            self.assertTrue(any("missing" in error for error in errors))
            self.assertTrue(any("include" in error for error in errors))

    def test_counts_comments_tabs_and_long_lines(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.c"
            path.write_text("/* " + "x" * 80 + " */\n\treturn;\n")
            errors = style.text_errors(path)
            self.assertEqual(len(errors), 2)
            self.assertIn(":1:", errors[0])
            self.assertIn(":2:", errors[1])

    def test_missing_formatter_does_not_pass(self):
        status, errors = style.check([], None)
        self.assertEqual(status, 2)
        self.assertIn("INCOMPLETE", errors[-1])

    @patch.object(style.subprocess, "run")
    def test_formatter_rejection_is_reported(self, run):
        run.return_value = subprocess.CompletedProcess([], 1, "", "missing braces")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.c"
            original = "int value;\n\n"
            path.write_text(original)
            status, errors = style.check([path], "formatter")
            self.assertEqual(status, 1)
            self.assertIn("missing braces", errors[-1])
            self.assertIn("--style=file", run.call_args.args[0])
            self.assertNotIn("-i", run.call_args.args[0])
            self.assertEqual(path.read_text(), original)

    @patch.object(style.subprocess, "run")
    def test_success_requires_formatter_and_text_checks(self, run):
        run.return_value = subprocess.CompletedProcess([], 0, "int value;\n", "")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.c"
            path.write_text("int value;\n\n")
            self.assertEqual(style.check([path], "formatter"), (0, []))
            path.write_text("int  value;\n\n")
            self.assertEqual(style.check([path], "formatter")[0], 1)

    def test_empty_selection_is_incomplete(self):
        self.assertEqual(style.check([], "formatter")[0], 2)


if __name__ == "__main__":
    unittest.main()
