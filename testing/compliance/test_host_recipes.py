"""Check subject/source isolation and flags without executing native programs."""
import json
from pathlib import Path
import tempfile
import unittest

from tools.compliance.host_recipes import additional_host_recipes, embedded_decoder_fixture


class AdditionalHostRecipeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name).resolve()
        self.subject, self.fixtures, self.output = (self.base/name for name in ('subject', 'fixtures', 'output'))
        self.output.mkdir()
        self.body = b'#include "paired.h"\n#include "../../app_program/subsystem3/bc_decoder.c"\nassert(value == 17);\n'
        for name in ('fit_error', 'incremental'):
            path = self.fixtures/('testing/subsystem3/test_barcode_'+name+'.c')
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(self.body)
        decoder = self.subject/'app_program/subsystem3/bc_decoder.c'
        decoder.parent.mkdir(parents=True)
        decoder.write_bytes(b'/* subject implementation */\n')

    def test_baseline_embedded_fixture_changes_only_subject_include(self):
        relative = 'testing/subsystem3/test_barcode_fit_error.c'
        chosen, receipt = embedded_decoder_fixture(self.subject, self.fixtures, relative, self.output/'copy.c')
        changed = chosen.read_bytes()
        self.assertIn((self.subject/'app_program/subsystem3/bc_decoder.c').as_posix().encode(), changed)
        self.assertEqual(changed.replace(receipt['relocated_include'].encode(),
                                         receipt['original_include'].encode()), self.body)
        self.assertEqual((self.fixtures/relative).read_bytes(), self.body)
        self.assertTrue(receipt['inverse_bytes_equal'])

    def test_candidate_source_stays_inside_measured_tree(self):
        relative = 'testing/subsystem3/test_barcode_incremental.c'
        chosen, receipt = embedded_decoder_fixture(self.fixtures, self.fixtures, relative, self.output/'unused.c')
        self.assertEqual(chosen, self.fixtures/relative)
        self.assertFalse((self.output/'unused.c').exists())
        self.assertEqual(receipt['selection'], 'candidate-subject')

    def test_missing_or_duplicate_include_is_rejected(self):
        relative = 'testing/subsystem3/test_barcode_fit_error.c'
        for body in (b'assert(value == 17);\n', self.body+self.body):
            with self.subTest(body=body):
                (self.fixtures/relative).write_bytes(body)
                with self.assertRaisesRegex(ValueError, 'exactly one'):
                    embedded_decoder_fixture(self.subject, self.fixtures, relative, self.output/'copy.c')

    def test_all_recipes_execute_unique_recorded_binaries_with_sanitizers(self):
        flags = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-O2', '-g', '-pthread',
                 '-fsanitize=address,undefined', '-fno-sanitize-recover=all']
        recipes = additional_host_recipes(self.subject, self.fixtures, self.output, '/recording-cc', flags)
        self.assertEqual({r['name'] for r in recipes}, {'barcode_fit_error', 'barcode_incremental',
            'barcode_bench', 'subsystem1_sizes', 'subsystem2_sizes', 'subsystem3_sizes', 'course_zero_axle'})
        outputs = []
        for recipe in recipes:
            self.assertEqual(recipe['build'][0], '/recording-cc')
            for flag in flags:
                self.assertIn(flag, recipe['build'])
            binary = recipe['build'][recipe['build'].index('-o')+1]
            self.assertEqual(recipe['execute'], [binary])
            self.assertFalse(Path(binary).exists())
            outputs.append(binary)
        self.assertEqual(len(set(outputs)), 7)
        for recipe in recipes:
            if recipe['name'].endswith('_sizes'):
                self.assertIn('-fshort-enums', recipe['build'])
        course = next(r for r in recipes if r['name']=='course_zero_axle')
        self.assertIn('-DS3_SENSOR_TO_AXLE_MM=0', course['build'])
        self.assertIn(str(self.subject/'app_program/controller/s3_drive.c'), course['build'])
        self.assertEqual(json.loads((self.output/'additional-host-recipes.json').read_text())['status'], 'PREPARED')

    def test_bad_fixture_retains_failed_preparation_receipt(self):
        (self.fixtures/'testing/subsystem3/test_barcode_fit_error.c').write_bytes(b'changed fixture\n')
        with self.assertRaises(ValueError):
            additional_host_recipes(self.subject, self.fixtures, self.output, '/recording-cc', ['-O2'])
        report = json.loads((self.output/'additional-host-recipes.json').read_text())
        self.assertEqual(report['status'], 'FAIL')
        self.assertTrue(report['errors'])
        self.assertEqual(report['recipes'], [])


if __name__ == '__main__':
    unittest.main()
