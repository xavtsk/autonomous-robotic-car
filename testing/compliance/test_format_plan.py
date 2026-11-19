"""Check token preservation and read-only preparation of a formatting batch."""
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.compliance import format_owned as formatter


class FingerprintTests(unittest.TestCase):
    def test_whitespace_comments_and_eof_do_not_change_fingerprint(self):
        before = 'int count = 3;\n'
        after = '/** @file sample.c */\nint   count=3;\n/* End of file. */\n\n'
        self.assertEqual(formatter.fingerprint(before), formatter.fingerprint(after))

    def test_compound_operator_cannot_be_hidden_as_whitespace(self):
        self.assertNotEqual(formatter.fingerprint('value++;'), formatter.fingerprint('value + +;'))

    def test_function_like_macro_adjacency_is_preserved(self):
        self.assertNotEqual(formatter.fingerprint('#define X(a) (a)\n'), formatter.fingerprint('#define X (a) (a)\n'))

    def test_directive_newline_boundary_is_semantic(self):
        self.assertNotEqual(formatter.fingerprint('#define VALUE 3\n+ 4\n'), formatter.fingerprint('#define VALUE 3 + 4\n'))

    def test_spliced_macro_formatting_is_equivalent(self):
        self.assertEqual(formatter.fingerprint('#define X(a) \\\n (a)\n'), formatter.fingerprint('#define X(a) (a)\n'))

    def test_string_literal_bytes_are_preserved(self):
        self.assertNotEqual(formatter.fingerprint('"a b"'), formatter.fingerprint('"ab"'))

    def test_angle_header_name_whitespace_is_semantic(self):
        self.assertNotEqual(formatter.fingerprint('#include <dir/name.h>\n'), formatter.fingerprint('#include <dir / name.h>\n'))

    def test_trigraph_conversion_precedes_line_splicing(self):
        self.assertNotEqual(formatter.fingerprint('??=define VALUE 3\n'), formatter.fingerprint('? ? = define VALUE 3\n'))
        self.assertEqual(formatter.fingerprint('#define VALUE ??/\n 3\n'), formatter.fingerprint('#define VALUE 3\n'))

    def test_macro_adjacency_with_comments_or_digraph_directive(self):
        self.assertNotEqual(formatter.fingerprint('#/**/define X(a) (a)\n'), formatter.fingerprint('#/**/define X (a) (a)\n'))
        self.assertNotEqual(formatter.fingerprint('%:define X(a) (a)\n'), formatter.fingerprint('%:define X (a) (a)\n'))


class ProposalTests(unittest.TestCase):
    def test_existing_purpose_is_used_and_not_guessed(self):
        self.assertEqual(formatter.leading_brief('/*\n * File: sample.c\n * Purpose: Read the encoder count.\n */\n'), 'Read the encoder count.')
        self.assertIsNone(formatter.leading_brief('#include "sample.h"\n'))

    def test_wrapped_purpose_is_not_cut_off(self):
        source = '/*\n * Purpose: Correct straight-line motion and\n *          encoder-based turns.\n *\n * Units: metres.\n */'
        self.assertEqual(formatter.leading_brief(source), 'Correct straight-line motion and encoder-based turns.')

    def test_license_stays_first_and_unchanged(self):
        source = '/* Copyright 2020. License text. */\nint count;\n'
        after = formatter.add_module_and_eof(source, 'sample.c', 'Count valid samples.')
        self.assertTrue(after.startswith('/* Copyright 2020. License text. */\n'))
        self.assertIn('@file sample.c', after)
        self.assertEqual(formatter.fingerprint(source), formatter.fingerprint(after))
        self.assertTrue(after.endswith('/* End of file. */\n\n'))

    def test_dry_run_proposal_never_writes_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / 'sample.c'
            source = b'int count;\n'
            path.write_bytes(source)
            row = {'path': 'sample.c', 'format_allowed': True, 'module_brief': 'Count valid samples.',
                   'before_sha256': hashlib.sha256(source).hexdigest()}
            results, changes, gaps = formatter.propose(root, {'files': [row]}, {'include_globs': ['*.c']})
            self.assertEqual(path.read_bytes(), source)
            self.assertEqual(len(changes), 1)
            self.assertEqual(gaps, [])
            self.assertTrue(results[0]['tokens_identical'])

    def test_formatter_code_change_is_rejected_before_write(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / 'sample.c'
            source = b'int count = 3;\n'
            path.write_bytes(source)
            row = {'path': 'sample.c', 'format_allowed': True, 'module_brief': 'Count valid samples.',
                   'before_sha256': hashlib.sha256(source).hexdigest()}
            with patch.object(formatter, 'format_text', return_value='int count = 4;\n'):
                with self.assertRaisesRegex(ValueError, 'tokens'):
                    formatter.propose(root, {'files': [row]}, {'include_globs': ['*.c']}, 'formatter')
            self.assertEqual(path.read_bytes(), source)


if __name__ == '__main__':
    unittest.main()
