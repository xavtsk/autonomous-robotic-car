"""Coverage markers must follow C logical lines and fail on ambiguous input."""
import importlib.util
from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import sys
import re
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

TOOLS = Path(__file__).resolve().parents[2]/'tools/compliance'
sys.path.insert(0, str(TOOLS))
SPEC = importlib.util.spec_from_file_location('conditional_coverage', TOOLS/'conditional_coverage.py')
COVERAGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COVERAGE)


class ConditionalCoverageTests(unittest.TestCase):
    def compiler(self):
        compiler = shutil.which('gcc-13')
        if compiler is None and Path('C:/msys64/ucrt64/bin/gcc.exe').is_file():
            compiler = 'C:/msys64/ucrt64/bin/gcc.exe'
        self.assertIsNotNone(compiler, 'A C preprocessor is required for the coverage audit test')
        return compiler

    def collect(self, source, header='', defines=()):
        """Exercise both real preprocess passes; never compile/run a C binary."""
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            root = folder/'subject'
            (root/'app_program').mkdir(parents=True)
            (root/'vendor').mkdir()
            fixture = root/'app_program/fixture.c'
            fixture.write_text(source)
            imported = root/'vendor/imported.h'
            imported.write_text(header)
            before = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in (fixture, imported)}
            commands = folder/'commands.json'
            commands.write_text(json.dumps([{'directory': str(root),
                'arguments': [self.compiler(), *defines, '-c', str(fixture), '-o', 'unused.o']}]))
            output = folder/'evidence'
            argv = ['conditional_coverage.py', '--root', str(root), '--commands', str(commands),
                    '--output', str(output), '--profile', 'test-profile']
            with patch.object(sys, 'argv', argv), patch.object(sys, 'platform', 'linux'), \
                 patch.object(COVERAGE.inventory, 'read_json', return_value={}), \
                 patch.object(COVERAGE.inventory, 'owned_files', return_value=[fixture]), \
                 patch.object(COVERAGE.subprocess, 'check_output', return_value='test-revision\n'), \
                 redirect_stdout(io.StringIO()):
                status = COVERAGE.main()
            report = json.loads((output/'results.json').read_text())
            self.assertEqual(before, {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                     for path in (fixture, imported)})
            self.assertFalse((root/'unused.o').exists())
            self.assertFalse((output/'source-overlay').exists())
            self.assertTrue(report['pristine_observations'])
            for row in report['pristine_observations']:
                self.assertIn('-dD', row['command'])
                self.assertTrue((output/row['log']).is_file())
                self.assertTrue((output/row['macro_definitions_log']).is_file())
                if row['returncode']:
                    self.assertTrue((output/row['failed_preprocessing_output']).is_file())
            return status, report

    def test_direct_line_and_inactive_elif_fail_before_instrumentation(self):
        for source in ('#if 1\n#endif\n#if __LINE__ == 3\nint a;\n#endif\n',
                       '#if 0\nint a;\n#elif __LINE__ == 3\nint b;\n#endif\n'):
            with self.subTest(source=source):
                with self.assertRaisesRegex(ValueError, '__LINE__'):
                    COVERAGE.instrument(source, 'fixture.c')
                status, report = self.collect(source)
                self.assertNotEqual(status, 0)
                self.assertTrue(report['line_safety_errors'])
                self.assertEqual(report['observations'], [])

    def test_local_alias_chain_reaching_condition_fails(self):
        source = '#define LOCATION __LINE__\n#define FEATURE LOCATION\n#if FEATURE > 0\nint a;\n#endif\n'
        with self.assertRaisesRegex(ValueError, 'FEATURE -> LOCATION -> __LINE__'):
            COVERAGE.instrument(source, 'fixture.c')

    def test_imported_and_command_line_aliases_are_observed_pristine(self):
        source = '#include "../vendor/imported.h"\n#if FEATURE > 0\nint a;\n#endif\n'
        for header, defines in (('#define FEATURE LOCATION\n#define LOCATION __LINE__\n', ()),
                                ('#define FEATURE CLI_LOCATION\n', ('-DCLI_LOCATION=__LINE__',))):
            with self.subTest(defines=defines):
                status, report = self.collect(source, header, defines)
                self.assertNotEqual(status, 0)
                self.assertIn('__LINE__', ' '.join(report['line_safety_errors']))
                self.assertEqual(report['observations'], [])

    def test_explicit_line_controls_are_rejected(self):
        for directive in ('#line 100 "other.c"', '%:line 100', '# 100 "other.c"'):
            with self.subTest(directive=directive), self.assertRaisesRegex(ValueError, 'line control'):
                COVERAGE.instrument(directive+'\n#if 1\nint a;\n#endif\n', 'fixture.c')

    def test_reporting_macro_and_defined_only_tests_remain_supported(self):
        source = '#include "../vendor/imported.h"\n#if defined(REPORT) && defined __LINE__\nvoid f(void) { REPORT(); }\n#endif\n'
        status, report = self.collect(source, '#define REPORT() printf("line=%d", __LINE__)\n')
        self.assertEqual(status, 0)
        self.assertEqual(report['line_safety_errors'], [])
        self.assertTrue(report['measurement_passed'])
        self.assertTrue(report['observations'])

    def test_token_pasting_cannot_manufacture_unchecked_line_builtin(self):
        source = '#include "../vendor/imported.h"\n#if JOIN(__LI, NE__) > 0\nint a;\n#endif\n'
        header = '#define JOIN(a,b) JOIN_INNER(a,b)\n#define JOIN_INNER(a,b) a ## b\n'
        status, report = self.collect(source, header)
        self.assertNotEqual(status, 0)
        self.assertIn('token-pasting', ' '.join(report['line_safety_errors']))

    def test_line_sensitive_include_selection_also_fails(self):
        errors = COVERAGE.line_safety_errors({'x.c': '#define HEADER __LINE__\n#include HEADER\n'})
        self.assertTrue(any('#include depends on line value' in error for error in errors))

    def test_pristine_failure_is_retained_and_cannot_contribute_markers(self):
        status, report = self.collect('#if 1\n#error deliberate pristine failure\n#endif\n')
        self.assertNotEqual(status, 0)
        self.assertNotEqual(report['pristine_observations'][0]['returncode'], 0)
        self.assertEqual(report['observations'], [])
        self.assertFalse(report['measurement_passed'])
        self.assertTrue(any('Pristine preprocessing failed' in error for error in report['errors']))

    def test_inactive_owned_alias_and_redefinitions_are_conservative(self):
        source = '#if 0\n#define FEATURE __LINE__\n#endif\n#define FEATURE 1\n#if FEATURE\nint a;\n#endif\n'
        self.assertTrue(COVERAGE.line_safety_errors({'x.c': source}))

    def test_comments_and_literal_line_words_do_not_trigger(self):
        source = '/* #line 9 */\n#define TEXT "__LINE__"\n#if 1\nconst char *s = TEXT;\n#endif\n'
        self.assertEqual(COVERAGE.line_safety_errors({'x.c': source}), [])

    def test_comments_literals_and_spliced_comments_are_not_arms(self):
        source = '/*\n#if BAD\n*/\nconst char *s = "#if BAD";\n// comment \\\n#if BAD\n#if GOOD\nint x;\n#endif\n'
        changed, rows = COVERAGE.instrument(source, 'a.c')
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['directive'], '#if GOOD')
        self.assertIn(source[:source.index('#if GOOD')], changed)

    def test_marker_follows_complete_multiline_condition(self):
        source = '#if FIRST && \\\n    SECOND\nint x;\n#elif THIRD\nint y;\n#else\nint z;\n#endif\n'
        changed, rows = COVERAGE.instrument(source, 'a.c')
        self.assertEqual([row['kind'] for row in rows], ['if', 'elif', 'else'])
        self.assertTrue(changed.startswith('#if FIRST && \\\n    SECOND\n\n#pragma message'))
        self.assertEqual(len({row['id'] for row in rows}), 3)

    def test_duplicate_conditions_have_distinct_ids_and_depth(self):
        _, rows = COVERAGE.instrument('#if A\n#if A\n#endif\n#else\n#endif\n', 'a.c')
        self.assertEqual([row['depth'] for row in rows], [1, 2, 1])
        self.assertEqual(len({row['id'] for row in rows}), 3)

    def test_invalid_structure_or_trigraph_is_explicit(self):
        for source in ('#else\n', '#if A\n', '#if A\n#else\n#elif B\n#endif\n', '??=if A\n'):
            with self.subTest(source=source), self.assertRaises(ValueError):
                COVERAGE.instrument(source, 'a.c')

    def test_compiler_options_preserve_forced_include(self):
        command = COVERAGE.preprocess_arguments(
            ['cc', '-include', 'fixture.c', '-DFOO=1', '-c', 'a.c', 'b.c', '-o', 'a.o', '-MMD',
             '-MFobject.d', '-MTobject.o', '-MQquoted.o', '-lm'], 'overlay/a.c')
        self.assertEqual(command, ['cc', '-include', 'fixture.c', '-DFOO=1', '-E', '-P', 'overlay/a.c'])

    def test_end_without_newline_and_digraph(self):
        changed, rows = COVERAGE.instrument('%:if A\nx;\n%:endif', 'a.c')
        self.assertEqual(len(rows), 1)
        self.assertTrue(changed.endswith('%:endif'))

    def test_real_preprocessor_observes_only_selected_arm(self):
        compiler = self.compiler()
        text, rows = COVERAGE.instrument('#if CHOICE\nint a;\n#else\nint b;\n#endif\n', 'fixture.c')
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'fixture.c'
            source.write_text(text)
            for choice, expected in ((1, rows[0]['id']), (0, rows[1]['id'])):
                command = COVERAGE.preprocess_arguments(
                    [compiler, '-MMD', '-MF'+str(Path(directory)/'unused.d'), '-MTunused.o',
                     f'-DCHOICE={choice}', '-c', str(source), '-o', str(Path(directory)/'unused.o')], source)
                result = subprocess.run(command,
                                        capture_output=True, text=True, check=True)
                observed = re.findall(COVERAGE.MARKER+r'([0-9a-f]{24})', result.stdout)
                self.assertEqual(observed, [expected])
                self.assertFalse((Path(directory)/'unused.d').exists())


if __name__ == '__main__':
    unittest.main()
