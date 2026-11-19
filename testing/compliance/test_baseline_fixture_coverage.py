"""Preprocess current fixtures only; never execute a C test program."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from contextlib import redirect_stdout
import io
from copy import deepcopy

TOOLS = Path(__file__).resolve().parents[2]/'tools/compliance'
sys.path.insert(0, str(TOOLS))
import baseline_fixture_coverage as external
import conditional_coverage as coverage
import module_compat


class BaselineFixtureCoverageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root, self.baseline = self.base/'tooling', self.base/'baseline'
        self.fixture = self.root/'testing/compliance/fixture.c'
        self.fixture.parent.mkdir(parents=True)
        (self.root/'tools/compliance/traces').mkdir(parents=True)
        self.fixture.write_text('#include "baseline.h"\n#if OLD_LAYOUT\nint old_value;\n#else\nint new_value;\n#endif\n')
        self.baseline.mkdir()
        self.header = self.baseline/'baseline.h'
        self.header.write_bytes(b'#define OLD_LAYOUT 1\n')
        self.blobs = {'baseline.h': external.git_blob(self.header.read_bytes())}
        self.binding = self.base/'fixture-bindings.json'
        fixtures = external.fixture_hashes(self.root)
        self.binding.write_text(json.dumps(dict(status='PASS', selection='fixed-baseline-shared-tooling',
            subject_revision=external.FIXED_BASELINE, tooling_revision='candidate',
            subject_root=str(self.baseline), tooling_root=str(self.root), selected_root=str(self.root),
            selected_sha256=fixtures, tooling_sha256=fixtures)))
        self.output = self.base/'evidence'
        self.output.mkdir()
        self.overlay = self.output/'overlay'
        shutil.copytree(self.root, self.overlay)
        self.compiler = shutil.which('gcc-13')
        if self.compiler is None and Path('C:/msys64/ucrt64/bin/gcc.exe').is_file():
            self.compiler = 'C:/msys64/ucrt64/bin/gcc.exe'
        self.assertIsNotNone(self.compiler, 'Coverage test requires an actual preprocessor')

    def context(self):
        with patch.object(external.subprocess, 'check_output', return_value=external.FIXED_BASELINE+'\n'), \
                patch.object(external, 'baseline_tree', return_value=self.blobs):
            return external.ExternalBaseline(self.root, self.baseline, self.binding, None, 'candidate')

    def command(self, context):
        command = coverage.preprocess_arguments([self.compiler, '-I'+str(self.baseline),
            '-D', 'UNCHANGED=1', '-c', str(self.fixture), '-o', 'unused.o'],
            self.overlay/self.fixture.relative_to(self.root))
        return context.map_context(command, self.baseline, self.root, self.overlay)

    def test_actual_preprocessor_observes_only_current_fixture_fallback_arm(self):
        context = self.context()
        command, cwd = self.command(context)
        self.assertEqual(cwd, self.baseline)
        self.assertEqual(command[command.index('-D')+1], 'UNCHANGED=1')
        dependency = context.observe(command, cwd, self.output, 0, self.overlay)
        self.assertEqual(dependency['errors'], [])
        self.assertEqual(dependency['returncode'], 0)
        self.assertEqual({row['kind'] for row in dependency['dependencies']}, {'tooling', 'baseline'})
        changed, branches = coverage.instrument(self.fixture.read_text(), 'testing/compliance/fixture.c')
        (self.overlay/'testing/compliance/fixture.c').write_text(changed)
        process = subprocess.run(command, cwd=cwd, capture_output=True, text=True, check=True)
        self.assertIn(branches[0]['id'], process.stdout)
        self.assertNotIn(branches[1]['id'], process.stdout)
        self.assertNotIn(coverage.MARKER, self.header.read_text())
        context.verify_preservation()
        self.assertTrue(context.record['source_unchanged'])
        self.assertFalse((self.baseline/'unused.o').exists())
        self.assertEqual(self.fixture.read_text().count(coverage.MARKER), 0)
        sample = dict(command=command, directory=str(cwd))
        report = dict(external_baseline=context.record, dependency_observations=[dependency],
                      compile_commands_sha256='0'*64)
        hashes = {'testing/compliance/fixture.c': hashlib.sha256(self.fixture.read_text().encode()).hexdigest()}
        with patch.object(external, 'baseline_tree', return_value=self.blobs), \
                patch.object(external.subprocess, 'check_output', return_value=self.header.read_bytes()):
            external.validate_receipt(report, [sample], hashes, 'candidate', self.root)
            mutations = (
                lambda row: row['external_baseline'].update(source_unchanged=False),
                lambda row: row['external_baseline'].update(revision='other'),
                lambda row: row['external_baseline'].update(fixture_sha256={}),
                lambda row: row['external_baseline'].update(root=str(self.base/'wrong')),
                lambda row: row.update(dependency_observations=[]),
                lambda row: row['dependency_observations'][0].update(command=['wrong']),
                lambda row: row['dependency_observations'][0]['dependencies'][0].update(bytes_sha256='0'*64),
                lambda row: row['dependency_observations'][0]['dependencies'][0].update(kind='unknown'),
                lambda row: row['dependency_observations'][0]['dependencies'].append(
                    dict(kind='system', path='/unbound/header.h', bytes_sha256='0'*64)),
            )
            for index, mutate in enumerate(mutations):
                with self.subTest(mutation=index):
                    bad = deepcopy(report)
                    mutate(bad)
                    with self.assertRaises(ValueError):
                        external.validate_receipt(bad, [sample], hashes, 'candidate', self.root)

    def test_dirty_consumed_baseline_header_is_retained_as_failure(self):
        context = self.context()
        self.header.write_text('#define OLD_LAYOUT 0\n')
        command, cwd = self.command(context)
        receipt = context.observe(command, cwd, self.output, 0, self.overlay)
        self.assertTrue(receipt['errors'])
        self.assertIn('Dirty or untracked', receipt['errors'][0])
        self.assertTrue((self.output/receipt['output']).exists())

    def test_full_collector_retains_external_binding_without_touching_baseline(self):
        helper = self.root/'app_program/build_assert.h'
        helper.parent.mkdir()
        helper.write_text('#ifndef HELPER_H\n#define HELPER_H\n#endif\n')
        self.fixture.write_text('#include "../../app_program/build_assert.h"\n'+self.fixture.read_text())
        binding = json.loads(self.binding.read_text())
        binding['selected_sha256'] = binding['tooling_sha256'] = external.fixture_hashes(self.root)
        self.binding.write_text(json.dumps(binding))
        commands = self.base/'commands.jsonl'
        commands.write_text(json.dumps(dict(directory=str(self.baseline), arguments=[self.compiler,
            '-I'+str(self.baseline), '-c', str(self.fixture), '-o', 'unused.o']))+'\n')
        output = self.base/'full-collector'
        arguments = ['conditional_coverage.py', '--root', str(self.root), '--external-baseline',
            str(self.baseline), '--fixture-bindings', str(self.binding), '--commands', str(commands),
            '--output', str(output), '--profile', 'host-baseline-fixtures-gcc']
        def revision(command, **kwargs):
            self.assertEqual(command, ['git', 'rev-parse', 'HEAD'])
            return (external.FIXED_BASELINE if Path(kwargs['cwd']) == self.baseline else 'candidate')+'\n'
        before = self.fixture.read_bytes(), self.header.read_bytes()
        with patch.object(sys, 'argv', arguments), patch.object(sys, 'platform', 'linux'), \
                patch.object(coverage.inventory, 'read_json', return_value={}), \
                patch.object(coverage.inventory, 'owned_files', return_value=[self.fixture, helper]), \
                patch.object(external.subprocess, 'check_output', side_effect=revision), \
                patch.object(external, 'baseline_tree', return_value=self.blobs), redirect_stdout(io.StringIO()):
            self.assertEqual(coverage.main(), 0)
        report = json.loads((output/'results.json').read_text())
        self.assertTrue(report['measurement_passed'])
        self.assertFalse(report['coverage_complete'])
        self.assertEqual(sum(row['observed'] for row in report['branches']), 1)
        self.assertFalse(next(row['observed'] for row in report['branches'] if row['source']=='app_program/build_assert.h'))
        self.assertIn('tooling-helper', {row['kind'] for row in report['dependency_observations'][0]['dependencies']})
        self.assertEqual(len(report['dependency_observations']), 1)
        self.assertTrue(report['external_baseline']['source_unchanged'])
        self.assertFalse((output/'source-overlay').exists())
        self.assertEqual(before, (self.fixture.read_bytes(), self.header.read_bytes()))

    def test_explicit_tilt_forwarder_is_bound_and_observed_as_derived(self):
        directory = self.baseline/'app_program/subsystem4'
        directory.mkdir(parents=True)
        header = directory/'tilt_estimation.h'
        header.write_bytes(self.header.read_bytes())
        (directory/'tilt_estimation.c').write_text('/* fixture module */\n')
        checksum = external.digest(header.read_bytes())
        self.blobs[header.relative_to(self.baseline).as_posix()] = external.git_blob(header.read_bytes())
        module_receipt = self.base/'tilt-module.json'
        with patch.object(module_compat, 'BASELINE_TILT_HEADER_SHA256', checksum):
            _, includes = module_compat.prepare_tilt(self.baseline, external.FIXED_BASELINE,
                self.base/'derived', module_receipt)
        with patch.object(external.subprocess, 'check_output', return_value=external.FIXED_BASELINE+'\n'), \
                patch.object(external, 'baseline_tree', return_value=self.blobs), \
                patch.object(external, 'BASELINE_TILT_HEADER_SHA256', checksum):
            context = external.ExternalBaseline(self.root, self.baseline, self.binding, module_receipt, 'candidate')
        command, cwd = self.command(context)
        command = [*command[:-1], '-include', str(includes[0]/'tilt.h'), command[-1]]
        command, cwd = context.map_context(command, cwd, self.root, self.overlay)
        receipt = context.observe(command, cwd, self.output, 0, self.overlay)
        self.assertEqual(receipt['errors'], [])
        self.assertIn('derived', {row['kind'] for row in receipt['dependencies']})
        (includes[0]/'tilt.h').write_text('/* stale */\n')
        with self.assertRaisesRegex(ValueError, 'changed during observation'):
            context.verify_preservation()

    def test_changed_dependency_after_observation_fails_preservation(self):
        context = self.context()
        command, cwd = self.command(context)
        self.assertFalse(context.observe(command, cwd, self.output, 0, self.overlay)['errors'])
        self.header.write_text('#define OLD_LAYOUT 2\n')
        with self.assertRaisesRegex(ValueError, 'changed during observation'):
            context.verify_preservation()

    def test_stale_fixture_binding_and_wrong_baseline_revision_fail(self):
        self.fixture.write_text('int changed;\n')
        with self.assertRaisesRegex(ValueError, 'fixture binding'):
            self.context()
        with patch.object(external.subprocess, 'check_output', return_value='other\n'), \
                self.assertRaisesRegex(ValueError, 'not the fixed baseline'):
            external.ExternalBaseline(self.root, self.baseline, self.binding, None, 'candidate')

    def test_unknown_input_roots_and_working_directories_fail(self):
        context = self.context()
        for directory, include in ((self.base/'outside', self.baseline),
                                    (self.baseline, self.base/'outside')):
            with self.subTest(directory=directory, include=include), self.assertRaisesRegex(ValueError, 'Unknown'):
                context.map_context([self.compiler, '-I'+str(include), '-E', str(self.fixture)],
                                    directory, self.root, self.overlay)
        for option in ('-save-temps', '-Wp,-MD,source.d', '-MJsource.json', '-ftime-trace'):
            with self.subTest(option=option), self.assertRaisesRegex(ValueError, 'Output-producing'):
                context.map_context([self.compiler, option, '-E', str(self.fixture)],
                                    self.baseline, self.root, self.overlay)
        with patch.object(Path, 'read_bytes', side_effect=AssertionError('Unknown dependency must not be read')):
            with self.assertRaisesRegex(ValueError, 'Unknown consumed dependency root'):
                context.dependency(self.base/'unknown/header.h', self.overlay)

    def test_private_configuration_is_rejected_without_reading_its_contents(self):
        directory = self.baseline/'config'
        directory.mkdir()
        (directory/'wifi_secrets.h').touch()
        with self.assertRaisesRegex(ValueError, 'network configuration'):
            self.context()

    def test_consumed_symlink_is_rejected_and_preprocessor_failure_retained(self):
        context = self.context()
        command, cwd = self.command(context)
        original = Path.is_symlink
        with patch.object(Path, 'is_symlink', lambda path: path == self.header or original(path)):
            receipt = context.observe(command, cwd, self.output, 0, self.overlay)
        self.assertIn('Linked coverage dependency', receipt['errors'][0])
        self.fixture.write_text('#error intentional dependency failure\n')
        (self.overlay/'testing/compliance/fixture.c').write_text(self.fixture.read_text())
        failed = context.observe(command, cwd, self.output, 1, self.overlay)
        self.assertNotEqual(failed['returncode'], 0)
        self.assertTrue(failed['errors'])
        self.assertTrue((self.output/failed['output']).exists())

    def test_gcc_and_clang_virtual_command_line_markers_are_explicit(self):
        context = self.context()
        command, cwd = self.command(context)
        source = self.overlay/self.fixture.relative_to(self.root)
        markers = '\n'.join('# 1 '+json.dumps(name) for name in
            ('<built-in>', '<command-line>', '<command line>', str(source), str(self.header)))
        with patch.object(external.subprocess, 'run', return_value=subprocess.CompletedProcess(command, 0, markers, '')):
            receipt = context.observe(command, cwd, self.output, 0, self.overlay)
        self.assertEqual(receipt['errors'], [])
        self.assertEqual(len(receipt['dependencies']), 2)


if __name__ == '__main__':
    unittest.main()
