"""Coverage markers must follow C logical lines and fail on ambiguous input."""
import importlib.util
from pathlib import Path
import sys
import re
import shutil
import subprocess
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[2]/'tools/compliance'
sys.path.insert(0, str(TOOLS))
SPEC = importlib.util.spec_from_file_location('conditional_coverage', TOOLS/'conditional_coverage.py')
COVERAGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COVERAGE)


class ConditionalCoverageTests(unittest.TestCase):
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
            ['cc', '-include', 'fixture.c', '-DFOO=1', '-c', 'a.c', 'b.c', '-o', 'a.o', '-MMD', '-lm'], 'overlay/a.c')
        self.assertEqual(command, ['cc', '-include', 'fixture.c', '-DFOO=1', '-E', '-P', 'overlay/a.c'])

    def test_end_without_newline_and_digraph(self):
        changed, rows = COVERAGE.instrument('%:if A\nx;\n%:endif', 'a.c')
        self.assertEqual(len(rows), 1)
        self.assertTrue(changed.endswith('%:endif'))

    def test_real_preprocessor_observes_only_selected_arm(self):
        compiler = shutil.which('gcc-13')
        if compiler is None and Path('C:/msys64/ucrt64/bin/gcc.exe').is_file():
            compiler = 'C:/msys64/ucrt64/bin/gcc.exe'
        self.assertIsNotNone(compiler, 'A C preprocessor is required for the coverage audit test')
        text, rows = COVERAGE.instrument('#if CHOICE\nint a;\n#else\nint b;\n#endif\n', 'fixture.c')
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'fixture.c'
            source.write_text(text)
            for choice, expected in ((1, rows[0]['id']), (0, rows[1]['id'])):
                result = subprocess.run([compiler, '-E', '-P', f'-DCHOICE={choice}', str(source)],
                                        capture_output=True, text=True, check=True)
                observed = re.findall(COVERAGE.MARKER+r'([0-9a-f]{24})', result.stdout)
                self.assertEqual(observed, [expected])


if __name__ == '__main__':
    unittest.main()
