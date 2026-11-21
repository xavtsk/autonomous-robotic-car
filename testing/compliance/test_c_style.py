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
                    stdout=formatted, stderr='formatter error' if code else '', returncode=code)) as run:
                result = style.check([path], 'pinned-formatter')
                self.assertEqual(run.call_args.kwargs['encoding'], 'utf-8')
                return result

    def test_utf8_sensor_units_match_the_formatter_output(self):
        text = '/* Accuracy ±1 degree. */\n'
        self.assertEqual(self.check_text(text + '\n', text)[0], 0)

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

    def test_conditional_spacing_preserves_literals_comments_and_indent(self):
        source = ('v = test   ? "literal   ?" : other; /* text   ? */\n'
                  'v = test\n    ? a : b;\n')
        self.assertEqual(style.canonical_format(source),
                         source.replace("test   ?", "test ?") + "\n")

    def test_grouped_pointer_spacing_only_affects_declarators(self):
        source = ("void (*p_callback)(void);\n"
                  "(*p_callback)();\n"
                  "void (* /* borrowed */ p_other)(void);\n"
                  "/* void (*p_example)(void); */\n")
        expected = source.replace("void (*p_callback)", "void ( * p_callback)")
        self.assertEqual(style.canonical_format(source), expected + "\n")
        self.assertEqual(style.canonical_format(expected), expected + "\n")

    def test_switch_terminators_align_without_moving_nested_loop_breaks(self):
        source = ("switch (state)\n{\ncase 0:\n    while (waiting)\n    {\n"
                  "        break;\n    }\n    break; /* done */\ndefault:\n"
                  "    switch (next)\n    {\n    case 1:\n        break;\n"
                  "    }\n    break;\n}\nwhile (busy)\n{\n    break;\n}\n")
        expected = source.replace("    break; /* done */", "break; /* done */")
        expected = expected.replace("case 1:\n        break;", "case 1:\n    break;")
        expected = expected.replace("    }\n    break;\n}", "    }\nbreak;\n}")
        self.assertEqual(style.canonical_format(source), expected + "\n")
        self.assertEqual(style.canonical_format(expected), expected + "\n")


if __name__ == '__main__':
    unittest.main()
