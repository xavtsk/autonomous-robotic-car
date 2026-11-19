"""Historical filename adapters must not silently substitute candidate sources."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.compliance import module_compat as compat


class TiltCompatibilityTests(unittest.TestCase):
    def make_pair(self, root, name):
        directory = root/'app_program/subsystem4'
        directory.mkdir(parents=True, exist_ok=True)
        for suffix in ('.c', '.h'):
            (directory/(name+suffix)).write_bytes(b'/* test input */\n')
        return directory

    def test_exactly_one_complete_pair_is_required(self):
        names = ('tilt_estimation.c', 'tilt_estimation.h', 'tilt.c', 'tilt.h')
        for mask in range(16):
            with self.subTest(mask=mask), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                directory = root/'app_program/subsystem4'
                directory.mkdir(parents=True)
                for index, name in enumerate(names):
                    if mask & (1 << index):
                        (directory/name).touch()
                if mask in (3, 12):
                    self.assertEqual(compat.tilt_module(root), 'tilt_estimation' if mask == 3 else 'tilt')
                else:
                    with self.assertRaises(ValueError):
                        compat.tilt_module(root)

    def test_baseline_forwarder_binds_absolute_dependency_and_preserves_source(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            root = base/'subject'
            directory = self.make_pair(root, 'tilt_estimation')
            before = {path.name: path.read_bytes() for path in directory.iterdir()}
            digest = hashlib.sha256(before['tilt_estimation.h']).hexdigest()
            receipt = base/'receipt.json'
            with patch.object(compat, 'BASELINE_TILT_HEADER_SHA256', digest):
                name, includes = compat.prepare_tilt(root, compat.FIXED_BASELINE, base/'derived', receipt)
            self.assertEqual(name, 'tilt_estimation')
            self.assertEqual(includes, [(base/'derived').resolve()])
            report = json.loads(receipt.read_text())
            generated = (includes[0]/'tilt.h').read_bytes()
            self.assertIn(('#include "'+(directory/'tilt_estimation.h').resolve().as_posix()+'"').encode(), generated)
            self.assertEqual(report['derived_header']['bytes_sha256'], hashlib.sha256(generated).hexdigest())
            self.assertEqual(report['derived_header']['dependency_bytes_sha256'], digest)
            self.assertEqual({path.name: path.read_bytes() for path in directory.iterdir()}, before)

    def test_candidate_uses_real_header_without_generating_an_alias(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            self.make_pair(base/'subject', 'tilt')
            self.assertEqual(compat.prepare_tilt(base/'subject', 'candidate', base/'unused', base/'receipt.json'), ('tilt', []))
            self.assertFalse((base/'unused').exists())
            self.assertEqual(json.loads((base/'receipt.json').read_text())['selection'], 'candidate-real-header')

    def test_linked_source_or_directory_is_rejected(self):
        for linked_name in ('tilt.h', 'subsystem4'):
            with self.subTest(linked_name=linked_name), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                self.make_pair(root, 'tilt')
                original = Path.is_symlink
                with patch.object(Path, 'is_symlink', lambda path: path.name == linked_name or original(path)):
                    with self.assertRaisesRegex(ValueError, 'Linked tilt'):
                        compat.tilt_module(root)

    def test_wrong_revision_or_stale_baseline_header_keeps_failure_receipt(self):
        for name, revision in (('tilt', compat.FIXED_BASELINE), ('tilt_estimation', 'candidate'),
                               ('tilt_estimation', compat.FIXED_BASELINE), ('tilt', '')):
            with self.subTest(name=name, revision=revision), tempfile.TemporaryDirectory() as temp:
                base = Path(temp)
                self.make_pair(base/'subject', name)
                with self.assertRaises(ValueError):
                    compat.prepare_tilt(base/'subject', revision, base/'derived', base/'receipt.json')
                self.assertEqual(json.loads((base/'receipt.json').read_text())['status'], 'FAIL')
                self.assertFalse((base/'derived').exists())

    def test_output_cannot_touch_subject_or_reuse_stale_directory(self):
        for inside in (False, True):
            with self.subTest(inside=inside), tempfile.TemporaryDirectory() as temp:
                base = Path(temp)
                root = base/'subject'
                directory = self.make_pair(root, 'tilt_estimation')
                digest = hashlib.sha256((directory/'tilt_estimation.h').read_bytes()).hexdigest()
                output = root/'derived' if inside else base/'stale'
                if not inside:
                    output.mkdir()
                with patch.object(compat, 'BASELINE_TILT_HEADER_SHA256', digest), self.assertRaises((OSError, ValueError)):
                    compat.prepare_tilt(root, compat.FIXED_BASELINE, output, base/'receipt.json')
                self.assertEqual(json.loads((base/'receipt.json').read_text())['status'], 'FAIL')
                self.assertFalse((output/'tilt.h').exists())


if __name__ == '__main__':
    unittest.main()
