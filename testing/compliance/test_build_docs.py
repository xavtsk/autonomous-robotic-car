"""Documentation receipts must not turn partial generation into compliance."""
from pathlib import Path
import hashlib
import tempfile
import unittest

from tools.compliance import build_docs as docs


class DocumentationTests(unittest.TestCase):
    def test_configuration_keeps_missing_documentation_visible(self):
        config = docs.configuration(Path('source'), Path('output'))
        for line in ('EXTRACT_ALL = NO', 'EXTRACT_STATIC = YES',
                     'WARN_IF_UNDOCUMENTED = YES', 'WARN_NO_PARAMDOC = YES',
                     'GENERATE_XML = YES', 'ENABLE_PREPROCESSING = NO'):
            self.assertIn(line, config.splitlines())
        self.assertNotIn('INPUT_FILTER', config)

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


if __name__ == '__main__':
    unittest.main()
