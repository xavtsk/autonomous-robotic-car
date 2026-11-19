"""Fixture provenance checks only; no C compiler or executable is invoked."""
import hashlib
from contextlib import redirect_stderr
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.compliance.fixture_binding import FIXED_BASELINE, select_fixture_root
from tools.compliance import fixture_binding, run_host


class FixtureBindingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.subject, self.tooling = self.directory/'subject', self.directory/'tooling'
        self.receipt = self.directory/'fixture-bindings.json'
        for root in (self.subject, self.tooling):
            for name, content in (
                    ('testing/compliance/test.c', b'#include "test.h"\n'),
                    ('testing/compliance/test.h', b'/* paired header */\n'),
                    ('testing/subsystem1/fakes/fake.h', b'#define FAKE 1\n'),
                    ('tools/compliance/traces/trace.c', b'/* observer */\n')):
                path = root/name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(content)

    def select(self, revision='candidate'):
        return select_fixture_root(self.subject, self.tooling, revision, 'candidate', self.receipt)

    def test_identical_candidate_binds_subject_and_records_each_input(self):
        before = {str(p): p.read_bytes() for root in (self.subject, self.tooling)
                  for p in root.rglob('*') if p.is_file()}
        self.assertEqual(self.select(), self.subject)
        report = json.loads(self.receipt.read_text())
        self.assertEqual(report['status'], 'PASS')
        self.assertEqual(report['selection'], 'byte-identical-candidate-subject')
        self.assertEqual(report['subject_sha256'], report['tooling_sha256'])
        self.assertEqual(report['selected_sha256'], report['subject_sha256'])
        self.assertEqual(len(report['selected_sha256']), 4)
        self.assertEqual(report['selected_sha256']['testing/subsystem1/fakes/fake.h'],
                         hashlib.sha256(b'#define FAKE 1\n').hexdigest())
        for path, data in before.items():
            self.assertEqual(Path(path).read_bytes(), data)

    def test_fixed_baseline_keeps_new_shared_fixtures(self):
        (self.subject/'testing/compliance/test.c').write_bytes(b'old assertions\n')
        (self.subject/'testing/compliance/test.h').unlink()
        self.assertEqual(self.select(FIXED_BASELINE), self.tooling)
        report = json.loads(self.receipt.read_text())
        self.assertEqual(report['selection'], 'fixed-baseline-shared-tooling')
        self.assertFalse(report['subject_files_compared'])
        self.assertNotIn('subject_sha256', report)

    def test_stale_revision_cannot_fall_back_to_tooling(self):
        with self.assertRaisesRegex(ValueError, 'revisions differ'):
            self.select('stale-candidate')
        self.assertEqual(json.loads(self.receipt.read_text())['status'], 'FAIL')

    def test_changed_source_or_fake_header_is_rejected_with_receipt(self):
        for name in ('testing/compliance/test.c', 'testing/subsystem1/fakes/fake.h'):
            with self.subTest(name=name):
                path = self.subject/name
                previous = path.read_bytes()
                path.write_bytes(previous+b'/* changed */\n')
                with self.assertRaisesRegex(ValueError, 'fixtures differ'):
                    self.select()
                report = json.loads(self.receipt.read_text())
                self.assertEqual(report['status'], 'FAIL')
                self.assertEqual(report['differences']['changed'], [name])
                self.assertNotIn('selected_root', report)
                path.write_bytes(previous)

    def test_missing_and_extra_headers_fail_closed(self):
        missing = 'testing/compliance/test.h'
        (self.subject/missing).unlink()
        extra = 'testing/compliance/stale.h'
        (self.subject/extra).write_text('/* extra */\n')
        with self.assertRaises(ValueError):
            self.select()
        report = json.loads(self.receipt.read_text())
        self.assertEqual(report['differences']['missing'], [missing])
        self.assertEqual(report['differences']['extra'], [extra])

    def test_line_ending_difference_is_not_normalized_away(self):
        (self.subject/'testing/compliance/test.c').write_bytes(b'#include "test.h"\r\n')
        with self.assertRaises(ValueError):
            self.select()

    def test_linked_inputs_are_rejected_without_following_them(self):
        original = Path.is_symlink
        with patch.object(Path, 'is_symlink', lambda path: path.name == 'fake.h' or original(path)):
            with self.assertRaisesRegex(ValueError, 'Linked fixture input'):
                self.select()
        self.assertEqual(json.loads(self.receipt.read_text())['status'], 'FAIL')

    def test_same_checkout_is_valid_and_missing_tool_tree_is_not(self):
        selected = select_fixture_root(self.tooling, self.tooling, 'candidate', 'candidate', self.receipt)
        self.assertEqual(selected, self.tooling)
        (self.tooling/'tools/compliance/traces/trace.c').unlink()
        (self.tooling/'tools/compliance/traces').rmdir()
        with self.assertRaisesRegex(ValueError, 'Missing or linked fixture tree'):
            self.select()

    def test_host_runner_retains_binding_failure_without_compiling(self):
        output = self.directory/'runner-output'
        arguments = ['run_host.py', '--root', str(self.subject), '--output', str(output)]
        with patch.object(run_host.sys, 'argv', arguments), \
                patch.object(run_host.sys, 'platform', 'linux'), \
                patch.dict(run_host.sys.modules, {'fixture_binding': fixture_binding}), \
                patch.object(run_host.shutil, 'which', return_value='/usr/bin/gcc-13'), \
                patch.object(run_host.subprocess, 'check_output', side_effect=['stale', 'candidate']), \
                patch.object(run_host.subprocess, 'run') as compile_or_execute, \
                redirect_stderr(io.StringIO()):
            self.assertEqual(run_host.main(), 1)
        compile_or_execute.assert_not_called()
        report = json.loads((output/'results.json').read_text())
        self.assertEqual(report['results'][0]['status'], 'FAIL')
        self.assertEqual(report['results'][0]['name'], 'fixture_bindings')
        self.assertEqual(json.loads((output/'fixture-bindings.json').read_text())['status'], 'FAIL')


if __name__ == '__main__':
    unittest.main()
