"""Instrumentation anchor tests: exact semantics, flexible formatting."""
from pathlib import Path
import re
import tempfile
import unittest

from tools.compliance import instrument_harnesses as harness
from tools.compliance.format_owned import TOKEN


def code_tokens(text):
    return [match[0] for match in TOKEN.finditer(text) if match.lastgroup != 'comment']


class AnchorTests(unittest.TestCase):
    def test_whitespace_and_interleaved_comments_match_once(self):
        source = 'before();\n call ( value , /* retained condition */ sizeof value ) ;\nafter();'
        result = harness.replace_once(source, 'call(value,sizeof value);', 'call(value,sizeof value); observe();')
        self.assertEqual(code_tokens(result), code_tokens('before();call(value,sizeof value);observe();after();'))
        self.assertTrue(result.startswith('before();\n '))
        self.assertTrue(result.endswith('\nafter();'))

    def test_comments_and_strings_do_not_supply_code_matches(self):
        source = '/* call(value); */ const char * text = "call(value);"; call ( value );'
        result = harness.replace_once(source, 'call(value);', 'observe();')
        self.assertIn('/* call(value); */', result)
        self.assertIn('"call(value);"', result)
        self.assertTrue(result.endswith('observe();'))

    def test_duplicate_code_matches_fail_despite_different_spacing(self):
        with self.assertRaisesRegex(ValueError, 'review instrumentation'):
            harness.replace_once('call(value); call ( value ) ;', 'call(value);', 'observe();')

    def test_operator_or_literal_change_fails_closed(self):
        for source in ['assert(value <= 3);', 'assert(value == 4);']:
            with self.subTest(source=source):
                with self.assertRaises(ValueError):
                    harness.replace_once(source, 'assert(value == 3);', 'observe();')

    def test_empty_or_comment_only_anchor_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'no C tokens'):
            harness.replace_once('call();', '/* comment */', 'observe();')

    def test_real_harnesses_instrument_without_mutating_original_files(self):
        root = Path(__file__).resolve().parents[2]
        paths = [root/'testing/integration/runtime_test.c', root/'testing/subsystem1/host_test.c']
        originals = [path.read_bytes() for path in paths]
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)
            runtime = harness.runtime_source(root, root, destination/'runtime.c').read_text()
            mqtt = harness.mqtt_source(root, destination/'mqtt.c').read_text()
            self.assertIn('trace_runtime("cycle")', runtime)
            self.assertIn('trace_runtime("receive")', runtime)
            self.assertIn('trace_publish(topic,payload,len,qos,retain)', mqtt)
            # Only sensor_sample_task belongs to the unbraced setjmp if. The
            # assertion and observer must return to the enclosing indentation;
            # placing them on its body line fails GCC -Wmisleading-indentation.
            cycle_tail = runtime[runtime.index('sensor_sample_task(0, NULL);'):]
            self.assertTrue(cycle_tail.startswith(
                'sensor_sample_task(0, NULL);\n    assert(!locked);\n'
                '    trace_runtime("cycle");\n}'))
        self.assertEqual([path.read_bytes() for path in paths], originals)

    def test_operator_observer_preserves_fixture_tokens_and_subject_selection(self):
        root = Path(__file__).resolve().parents[2]
        original_path = root/'testing/integration/operator_test.c'
        original = original_path.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)/'observed.c'
            # The subject path deliberately differs from the shared fixture.
            subject = Path(directory)/'separate-baseline'
            observed = harness.operator_source(root, subject, destination).read_text()
            self.assertNotIn('"../../app_program/', observed)
            self.assertIn((subject/'app_program/controller/op_logic.h').as_posix(), observed)
            self.assertLess(observed.index('#define APP_H'), observed.index('op_trace.h'))
            self.assertLess(observed.index('operator_step(&op'), observed.index('op_trace_write('))
            recovered, count = re.subn(r'^#include "[^"\n]*/op_trace\.h"\n', '', observed, flags=re.M)
            self.assertEqual(count, 1)
            recovered, count = re.subn(r'const size_t trace_console_begin = console_len;', '', recovered)
            self.assertEqual(count, 1)
            recovered, count = re.subn(
                r'const op_trace_context_t trace_context = \{[^;]+;\s*'
                r'op_trace_write\([^;]+;', '', recovered)
            self.assertEqual(count, 1)
            recovered = recovered.replace('"'+(subject/'app_program').as_posix()+'/', '"../../app_program/')
            self.assertEqual(code_tokens(recovered), code_tokens(original.decode()))
        self.assertEqual(original_path.read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
