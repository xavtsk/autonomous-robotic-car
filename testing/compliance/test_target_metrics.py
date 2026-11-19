"""Do not hide duplicate symbols, stack increases, or missing measurements."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

PATH = Path(__file__).resolve().parents[2]/'tools/compliance/target_metrics.py'
SPEC = importlib.util.spec_from_file_location('target_metrics', PATH)
METRICS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(METRICS)


class TargetMetricsTests(unittest.TestCase):
    def test_duplicate_local_names_preserved(self):
        rows = METRICS.symbol_rows('g_value b 20000000 4\ng_value b 20000008 8\nend A 20000100\n')
        self.assertEqual([row['bytes'] for row in rows], [4, 8])
        changes = METRICS.compare_rows(rows, rows[:1], ('name', 'kind'), ('bytes',))
        self.assertEqual(changes[0]['baseline'], [(4,), (8,)])
        self.assertEqual(changes[0]['candidate'], [(4,)])

    def test_stack_increase_and_new_function_are_reported(self):
        old = [dict(report='a.su', function='process', bytes=40, qualifier='static')]
        new = [dict(report='a.su', function='process', bytes=48, qualifier='dynamic,bounded'),
               dict(report='a.su', function='helper', bytes=8, qualifier='static')]
        changes = METRICS.compare_rows(old, new, ('report', 'function'), ('bytes', 'qualifier'))
        self.assertEqual(len(changes), 2)
        process = next(row for row in changes if row['function'] == 'process')
        self.assertEqual(process['candidate'], [(48, 'dynamic,bounded')])

    def test_source_lines_do_not_make_fake_stack_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory)/'a.su'
            file.write_text('../a.c:12:3:process\t200\tstatic\n')
            before = METRICS.stack_rows(Path(directory))
            file.write_text('../a.c:45:3:process\t200\tstatic\n')
            after = METRICS.stack_rows(Path(directory))
            self.assertFalse(METRICS.compare_rows(before, after, ('report', 'function'), ('bytes', 'qualifier')))

    def test_missing_or_invalid_evidence_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                METRICS.stack_rows(Path(directory))
        with self.assertRaises(ValueError):
            METRICS.symbol_rows('unexpected output')


if __name__ == '__main__':
    unittest.main()
