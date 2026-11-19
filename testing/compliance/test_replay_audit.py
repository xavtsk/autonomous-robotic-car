"""The diagnostic collector must retain failed expectations and continue."""
import ast
import importlib.util
from pathlib import Path
import unittest

MODULE = Path(__file__).resolve().parents[2] / 'tools/compliance/replay_audit.py'
SPEC = importlib.util.spec_from_file_location('replay_audit', MODULE)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class AssertionCollectorTests(unittest.TestCase):
    def test_failure_is_recorded_and_later_checks_still_run(self):
        records = []

        def collect(condition, message, line):
            records.append((condition, message() if not condition else None, line))

        scope = {'_record_assertion': collect}
        source = 'assert False, "original failure"\nassert True, 1 / 0\ncompleted = True\n'
        tree = ast.fix_missing_locations(AUDIT.CollectAssertions().visit(ast.parse(source)))
        exec(compile(tree, '<diagnostic-test>', 'exec'), scope)
        self.assertEqual(records, [(False, 'original failure', 1), (True, None, 2)])
        self.assertTrue(scope['completed'])

    def test_predicate_side_effect_executes_exactly_once(self):
        calls = []
        scope = {'predicate': lambda: calls.append('called') or False,
                 '_record_assertion': lambda condition, message, line: None}
        tree = ast.fix_missing_locations(AUDIT.CollectAssertions().visit(
            ast.parse('assert predicate()')))
        exec(compile(tree, '<diagnostic-test>', 'exec'), scope)
        self.assertEqual(calls, ['called'])


if __name__ == '__main__':
    unittest.main()
