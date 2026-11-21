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
    def test_operator_ports_select_only_the_reviewed_callback_pair(self):
        root = Path(__file__).resolve().parents[2]
        path = Path('app_program/controller/op_logic.h')
        current = (root/path).read_text()
        self.assertEqual(harness.operator_port_flags(root), [])
        with tempfile.TemporaryDirectory() as directory:
            subject = Path(directory)
            header = subject/path
            header.parent.mkdir(parents=True)
            legacy = harness.replace_once(current, '(*p_submit)', '(*submit)')
            legacy = harness.replace_once(legacy, '(*p_print)', '(*print)')
            header.write_text(legacy)
            self.assertEqual(harness.operator_port_flags(subject), ['-DOP_TRACE_LEGACY_PORT=1'])
            header.write_text(harness.replace_once(legacy, '(*print)', '(*p_print)'))
            with self.assertRaises(ValueError):
                harness.operator_port_flags(subject)

    def test_private_storage_flags_require_exact_reviewed_declarations(self):
        root = Path(__file__).resolve().parents[2]
        original = (root/'app_program/controller/s3_integration.c').read_text()
        self.assertEqual(harness.integration_storage_flags(root), ['-DTEST_S3_COMPACT_STORAGE=1'])
        with tempfile.TemporaryDirectory() as directory:
            subject = Path(directory)
            source = subject/'app_program/controller/s3_integration.c'
            source.parent.mkdir(parents=True)
            grouped = re.sub(r'(\b(?:b_has_command|b_has_stop|b_has_status|b_has_ir_debug) : 1);\s*bool ',
                             r'\1, ', original)
            source.write_text(grouped)
            self.assertEqual(harness.integration_storage_flags(subject), ['-DTEST_S3_COMPACT_STORAGE=1'])
            source.write_text(grouped.replace('bool b_has_command : 1,', 'bool b_has_command : 2,'))
            with self.assertRaisesRegex(ValueError, 'Harness changed'):
                harness.integration_storage_flags(subject)
            source.write_text(original.replace('uint8_t sensor : 2;', 'uint8_t sensor : 3;'))
            with self.assertRaisesRegex(ValueError, 'Harness changed'):
                harness.integration_storage_flags(subject)
            source.write_text(original.replace('s3_shared_flags_t g_shared;',
                                               's3_sampling_flags_t g_shared;'))
            with self.assertRaisesRegex(ValueError, 'Harness changed'):
                harness.integration_storage_flags(subject)
            anonymous = original
            for name in ('shared', 'sampling'):
                fields = re.search(
                    r'typedef struct\s*(\{[^{}]*\}) s3_' + name + r'_flags_t;',
                    anonymous)
                self.assertIsNotNone(fields)
                anonymous = anonymous.replace(fields[0], '').replace(
                    f'static s3_{name}_flags_t g_{name};',
                    f'static struct {fields[1]} g_{name};')
            source.write_text(anonymous)
            self.assertEqual(harness.integration_storage_flags(subject),
                             ['-DTEST_S3_COMPACT_STORAGE=1'])
            older_fields = {new: old for old, new in harness.INTEGRATION_PREDICATE_FIELDS.items()}
            older_compact = TOKEN.sub(
                lambda token: older_fields.get(token[0], token[0])
                if token.lastgroup == 'identifier' else token[0], original)
            source.write_text(older_compact)
            self.assertEqual(harness.integration_storage_flags(subject),
                             ['-DTEST_S3_COMPACT_STORAGE=1',
                              '-DTEST_S3_LEGACY_PREDICATES=1'])
            legacy = '''static struct {
                unsigned pending : 1, stop : 1, status_ready : 1, reset_pending : 1;
            } g_shared;
            static bool g_reset_waiting; static bool g_reset_remote;
            static bool g_collecting; static bool g_ir_ready, g_ir_seen;
            static bool g_control_ready, g_control_seen;
            static s3_sensor_id_t g_sensor; static ir_surface_t g_label;'''
            source.write_text(legacy)
            self.assertEqual(harness.integration_storage_flags(subject), [])
            source.write_text(legacy.replace('s3_sensor_id_t', 'uint16_t'))
            with self.assertRaisesRegex(ValueError, 'Harness changed'):
                harness.integration_storage_flags(subject)
            source.write_text('/* '+legacy+' */')
            with self.assertRaisesRegex(ValueError, 'Unrecognized original'):
                harness.integration_storage_flags(subject)

    def test_private_aliases_follow_subject_and_preserve_original_assertions(self):
        root = Path(__file__).resolve().parents[2]
        fixture = (root/'testing/integration/runtime_main.c').read_text()
        self.assertNotIn('#include "../../app_program/controller/s3_integration.c"', fixture)
        fixture_header = (root/'testing/integration/runtime_main.h').read_text()
        self.assertIn('#include "layout_access.h"', fixture_header)
        header = (root/'testing/integration/layout_access.h').read_text()
        self.assertIn('#if defined(TEST_S3_COMPACT_STORAGE) && TEST_S3_COMPACT_STORAGE', header)
        for name, member in harness.INTEGRATION_STORAGE_NAMES.items():
            self.assertIn(f'#define TEST_S3_{name[2:].upper()} ({member})', header)
            self.assertIn(f'#define TEST_S3_{name[2:].upper()} ({name})', header)
        for name, member in harness.integration_storage_aliases(root).items():
            self.assertIn(f'#define TEST_S3_{name[2:].upper()} ({member})', header)
        self.assertNotIn('assert(', header)
        self.assertNotIn('TEST_S3_COMPACT_STORAGE',
                         (root/'app_program/controller/s3_integration.c').read_text())

    def test_manual_make_checks_detection_and_keeps_command_line_flags(self):
        root = Path(__file__).resolve().parents[2]
        for relative in ['testing/integration/controller_test.mk',
                         'testing/subsystem4/Makefile']:
            with self.subTest(makefile=relative):
                makefile = (root/relative).read_text()
                self.assertIn('PYTHON ?= python3', makefile)
                self.assertIn('ifneq ($(.SHELLSTATUS),0)\n$(error Private storage detection failed;', makefile)
                self.assertIn('override CFLAGS += $(PRIVATE_STORAGE_FLAGS)', makefile)

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
        paths = [root/'testing/integration/runtime_main.c', root/'testing/subsystem1/host_main.c']
        originals = [path.read_bytes() for path in paths]
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)
            runtime = harness.runtime_source(root, root, destination/'runtime.c').read_text()
            mqtt = harness.mqtt_source(root, destination/'mqtt.c').read_text()
            self.assertIn('trace_runtime("cycle")', runtime)
            self.assertIn('trace_runtime("receive")', runtime)
            self.assertIn('trace_publish(p_topic,p_payload,len,qos,retain)', mqtt)
            # The assertion and observer must also run after the task longjmps.
            cycle_tail = runtime[runtime.index('s3_integration_runtime_test_cycle();'):]
            self.assertTrue(cycle_tail.startswith(
                's3_integration_runtime_test_cycle();\n    }\n    assert(!g_locks);\n'
                '    trace_runtime("cycle");\n}'))
        self.assertEqual([path.read_bytes() for path in paths], originals)

    def test_operator_observer_preserves_fixture_tokens_and_subject_selection(self):
        root = Path(__file__).resolve().parents[2]
        original_path = root/'testing/integration/operator_main.c'
        original = original_path.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)/'observed.c'
            # The subject path deliberately differs from the shared fixture.
            subject = Path(directory)/'separate-baseline'
            observed = harness.operator_source(root, subject, destination).read_text()
            self.assertNotIn('"../../app_program/', observed)
            self.assertIn((subject/'app_program/controller/op_logic.h').as_posix(), observed)
            self.assertLess(observed.index('#include "operator_main.h"'), observed.index('op_trace.h'))
            self.assertLess(observed.index('op_logic_step(&g_operator'), observed.index('op_trace_write('))
            recovered, count = re.subn(r'^#include "[^"\n]*/op_trace\.h"\n', '', observed, flags=re.M)
            self.assertEqual(count, 1)
            recovered, count = re.subn(r'const size_t trace_console_begin = g_console_length;', '', recovered)
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
