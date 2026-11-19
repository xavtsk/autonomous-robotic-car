"""Documentation receipts must not turn partial generation into compliance."""
from pathlib import Path
import hashlib
import json
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from tools.compliance import build_docs as docs


class DocumentationTests(unittest.TestCase):
    def test_configuration_keeps_missing_documentation_visible(self):
        config = docs.configuration(Path('source'), Path('output'))
        for line in ('EXTRACT_ALL = NO', 'EXTRACT_STATIC = YES',
                     'WARN_IF_UNDOCUMENTED = YES', 'WARN_NO_PARAMDOC = YES',
                     'GENERATE_XML = YES', 'ENABLE_PREPROCESSING = NO'):
            self.assertIn(line, config.splitlines())
        self.assertNotIn('INPUT_FILTER', config)
        self.assertNotIn('HTML_TIMESTAMP', config)

    def test_configuration_rejects_ambiguous_path(self):
        with self.assertRaises(ValueError):
            docs.configuration(Path('quoted"source'), Path('output'))

    def test_compiler_report_requires_exact_current_file_set(self):
        bindings = {'a.c': {'source_location_sha256': 'same', 'source_content_sha256': 'prose'}}
        report = {'files': {'a.c': {'source_location_sha256': 'same', 'source_content_sha256': 'prose'}}, 'functions': [{}]}
        docs.validate_functions(report, bindings)
        for broken in (
            {'files': {}, 'functions': [{}]},
            {'files': {'a.c': {'source_location_sha256': 'old'}}, 'functions': [{}]},
            {'files': {'a.c': {'source_location_sha256': 'same'}}, 'functions': [{}]},
            {'files': report['files'], 'functions': []},
        ):
            with self.subTest(broken=broken), self.assertRaises(ValueError):
                docs.validate_functions(broken, bindings)

    def test_same_line_comment_changes_invalidate_documentation_report(self):
        before = '/** @brief Valid contract. */\nint function(void);\n'
        after = '/* Removed contract. */\nint function(void);\n'
        location = docs.inventory.source_location_digest(before)
        self.assertEqual(location, docs.inventory.source_location_digest(after))
        report = {'files': {'a.h': {'source_location_sha256': location,
                                   'source_content_sha256': hashlib.sha256(before.encode()).hexdigest()}},
                  'functions': [{}]}
        binding = {'a.h': {'source_location_sha256': location,
                           'source_content_sha256': hashlib.sha256(after.encode()).hexdigest()}}
        with self.assertRaisesRegex(ValueError, 'documentation source content'):
            docs.validate_functions(report, binding)

    def test_comparison_uses_location_not_just_function_name(self):
        rows = [dict(path='a.c', name='same', line=10, end_line=20,
                     profiles=['production'], method='libclang-ast', definition_documented=True),
                dict(path='b.c', name='same', line=10, end_line=20,
                     profiles=['production'], method='libclang-recovery', definition_documented=False)]
        result = docs.compare_functions({'functions': rows}, {('a.c', 11, 'same')})
        self.assertEqual(result['missing_observed_definitions'], [])
        self.assertEqual(result['compiler_recovery_variants'], 1)
        self.assertEqual(result['compiler_undocumented_variants'], 1)
        for wrong in ({('b.c', 11, 'same')}, {('a.c', 21, 'same')}, set()):
            self.assertEqual(len(docs.compare_functions({'functions': rows}, wrong)['missing_observed_definitions']), 1)

    def test_xml_locations_and_compound_paths_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staged = root/'source'
            staged.mkdir()
            xml = root/'xml'
            xml.mkdir()
            (xml/'index.xml').write_text('<doxygenindex><compound refid="a"/></doxygenindex>')
            (xml/'a.xml').write_text('<doxygen><memberdef kind="function"><name>fun</name>'
                                    '<location bodyfile="a.c" bodystart="12"/></memberdef></doxygen>')
            self.assertEqual(docs.observed_functions(xml, staged), {('a.c', 12, 'fun')})
            for body_start in ('', '0', '-1', 'abc'):
                (xml/'a.xml').write_text('<doxygen><memberdef kind="function"><name>fun</name>'
                                        '<location bodyfile="a.c" bodystart="'+body_start+'"/></memberdef></doxygen>')
                with self.subTest(body_start=body_start), self.assertRaisesRegex(ValueError, 'body start'):
                    docs.observed_functions(xml, staged)
            (xml/'index.xml').write_text('<doxygenindex><compound refid="../escape"/></doxygenindex>')
            with self.assertRaisesRegex(ValueError, 'compound'):
                docs.observed_functions(xml, staged)
            (xml/'index.xml').write_text('<doxygenindex><compound refid="a"/></doxygenindex>')
            (xml/'a.xml').write_text('<doxygen><memberdef kind="function"><name>fun</name>'
                                    '<location bodyfile="../outside.c" bodystart="12"/></memberdef></doxygen>')
            with self.assertRaisesRegex(ValueError, 'outside'):
                docs.observed_functions(xml, staged)

    def run_generation(self, root, output_text='', returncode=0, mutate=False):
        source = root/'a.c'
        source.write_text('int fun(void) { return 0; }\n')
        report_path = root/'functions.json'
        report_path.write_text(json.dumps({'coverage_complete': True}))
        output = root/'output'

        def stage(_root, destination):
            destination.mkdir()
            staged = destination/'a.c'
            staged.write_bytes(source.read_bytes())
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            return {'a.c': {'original_bytes_sha256': digest, 'staged_sha256': digest}}

        def generate(_command, **kwargs):
            kwargs['stdout'].write(output_text)
            (output/'reference/warnings.log').write_text('')
            if mutate:
                source.write_text('int fun(void) { return 1; }\n')
            return SimpleNamespace(returncode=returncode)

        comparison = dict(missing_observed_definitions=[], compiler_recovery_variants=0,
                          compiler_undocumented_variants=0)
        with patch.object(docs, 'stage_sources', side_effect=stage), \
                patch.object(docs, 'validate_functions'), \
                patch.object(docs.subprocess, 'check_output', return_value='1.9.8\n'), \
                patch.object(docs.subprocess, 'run', side_effect=generate), \
                patch.object(docs, 'observed_functions', return_value=set()), \
                patch.object(docs, 'compare_functions', return_value=comparison), \
                patch('builtins.print'):
            code = docs.main(['--root', str(root), '--output', str(output),
                              '--function-report', str(report_path)])
        return code, json.loads((output/'results.json').read_text())

    def test_configuration_diagnostics_outside_warning_log_block_completion(self):
        for diagnostic in ('warning: obsolete configuration tag\n',
                           'error: ignored option\n', 'unexpected diagnostic\n'):
            with self.subTest(diagnostic=diagnostic), tempfile.TemporaryDirectory() as directory:
                code, result = self.run_generation(Path(directory), diagnostic)
                self.assertEqual(code, 2)
                self.assertEqual(result['status'], 'INCOMPLETE')
                self.assertEqual(result['warning_lines'], 0)
                self.assertEqual(result['generation_diagnostic_lines'], 1)
                self.assertTrue(result['source_unchanged'])

    def test_only_quiet_complete_generation_can_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            code, result = self.run_generation(Path(directory), '\n  \n')
            self.assertEqual(code, 0)
            self.assertTrue(result['coverage_complete'])
            self.assertTrue(result['source_unchanged'])

    def test_failed_generation_still_reports_source_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            code, result = self.run_generation(Path(directory), 'failed\n', 1, True)
            self.assertEqual(code, 2)
            self.assertFalse(result['source_unchanged'])
            self.assertEqual(result['generation_returncode'], 1)
            self.assertEqual(len(result['errors']), 2)


if __name__ == '__main__':
    unittest.main()
