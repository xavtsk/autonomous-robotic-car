"""Formatter checks retain the selected EOF rule without hiding other drift."""
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from tools import check_c_style as style


class CStyleTests(unittest.TestCase):
    def check_text(self, current, formatted, code=0):
        with TemporaryDirectory() as directory:
            path = Path(directory)/'sample.h'; path.write_text(current, encoding='utf-8')
            with patch.object(style.subprocess, 'run', return_value=SimpleNamespace(
                    stdout=formatted, stderr='formatter error' if code else '', returncode=code)):
                return style.check([path], 'pinned-formatter')

    def test_standard_final_blank_is_not_formatter_violation(self):
        self.assertEqual(self.check_text('int x;\n\n/* End of file. */\n\n',
                                         'int x;\n\n/* End of file. */\n')[0], 0)

    def test_missing_or_excess_final_blank_still_fails(self):
        for current in ('int x;\n', 'int x;\n\n\n'):
            self.assertEqual(self.check_text(current, 'int x;\n')[0], 1)

    def test_non_eof_formatting_difference_is_not_hidden(self):
        self.assertEqual(self.check_text('int  x;\n\n', 'int x;\n')[0], 1)

    def test_tabs_width_and_formatter_errors_still_fail(self):
        for text in ('int\tx;\n\n', '/*' + 'x'*81 + '*/\n\n'):
            self.assertEqual(self.check_text(text, text)[0], 1)
        self.assertEqual(self.check_text('int x;\n\n', '', code=1)[0], 1)

    def test_missing_formatter_is_explicitly_incomplete(self):
        self.assertEqual(style.check([], None)[0], 2)


if __name__ == '__main__':
    unittest.main()
