"""Check subject/source isolation and flags without executing native programs."""
import json
from pathlib import Path
import tempfile
import unittest

from tools.compliance.host_recipes import (additional_host_recipes,
                                           decoder_boundary_recipe,
                                           decoder_with_probes,
                                           motion_storage_source)


class AdditionalHostRecipeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name).resolve()
        self.subject, self.fixtures, self.output = (self.base/name for name in ('subject', 'fixtures', 'output'))
        self.output.mkdir()
        fixture_dir = self.fixtures/'testing/subsystem3'
        fixture_dir.mkdir(parents=True)
        for name in ('work_quota_main', 'fit_score_main'):
            (fixture_dir/(name+'.c')).write_text('#include "'+name+'.h"\n')
        decoder = self.subject/'app_program/subsystem3/bc_decoder.c'
        decoder.parent.mkdir(parents=True)
        self.decoder = b'/* subject implementation */\n'
        decoder.write_bytes(self.decoder)
        shared_decoder = self.fixtures/'app_program/subsystem3/bc_decoder.c'
        shared_decoder.parent.mkdir(parents=True)
        self.fit_probes = b'#ifdef BC_DECODER_FIT_TEST\n/* shared fit probes */\n#endif'
        self.work_probes = b'#ifdef BC_DECODER_WORK_TEST\n/* shared work probes */\n#endif'
        shared_decoder.write_bytes(b'/* shared implementation */\n'+self.fit_probes+b'\n'+self.work_probes+b'\n')
        decoder.with_suffix('.h').write_text(
            'typedef struct { uint16_t cursor, n, best; } s3_barcode_work_t;\n')
        controller = self.subject/'app_program/controller'
        controller.mkdir()
        (controller/'s3_drive.h').write_text('motion_result_t (*drive)(float meters, float speed_mps);\n')

    def test_baseline_appends_only_selected_probe_bodies(self):
        relative = 'app_program/subsystem3/bc_decoder.c'
        chosen, receipt = decoder_with_probes(self.subject, self.fixtures,
            self.output/'copy.c', 'BC_DECODER_WORK_TEST', 'work_quota_main.h')
        self.assertEqual(chosen.read_bytes(), self.decoder +
                         b'\n#include "work_quota_main.h"\n\n'+self.work_probes+b'\n')
        self.assertEqual((self.subject/relative).read_bytes(), self.decoder)
        self.assertEqual(receipt['selection'], 'fixed-decoder-with-probes')

    def test_candidate_source_stays_inside_measured_tree(self):
        relative = 'app_program/subsystem3/bc_decoder.c'
        chosen, receipt = decoder_with_probes(self.fixtures, self.fixtures,
            self.output/'unused.c', 'BC_DECODER_WORK_TEST', 'work_quota_main.h')
        self.assertEqual(chosen, self.fixtures/relative)
        self.assertFalse((self.output/'unused.c').exists())
        self.assertEqual(receipt['selection'], 'subject-decoder-probes')

    def test_boundary_recipe_uses_the_subjects_work_phases(self):
        relative = 'app_program/subsystem3/bc_decoder.c'
        phases = b'enum { RW_PRIMARY = 1, RW_FIT_BEGIN, RW_DONE };\n'
        (self.subject/relative).write_bytes(self.decoder + phases)
        shared = self.fixtures/relative
        shared.write_bytes(shared.read_bytes() +
                           b'#ifdef BC_DECODER_BOUNDARY_TEST\n/* probes */\n#endif\n')
        stage = self.output/'boundaries'
        sources, includes, options = decoder_boundary_recipe(self.subject, self.fixtures, stage)
        self.assertEqual(sources, [self.fixtures/'testing/compliance/decode_work_main.c',
                                   stage/'decoder.c'])
        self.assertIn(stage, includes)
        self.assertIn('-DBC_DECODER_BOUNDARY_TEST', options)
        self.assertEqual((stage/'decode_work_phases.h').read_bytes(), phases)
        self.assertTrue((stage/'decoder.c').read_bytes().startswith(self.decoder + phases))

    def test_missing_or_duplicate_probe_block_is_rejected(self):
        relative = 'app_program/subsystem3/bc_decoder.c'
        for body in (b'/* Missing probe. */\n', self.work_probes+b'\n'+self.work_probes):
            with self.subTest(body=body):
                (self.fixtures/relative).write_bytes(body)
                with self.assertRaisesRegex(ValueError, 'Expected one shared'):
                    decoder_with_probes(self.subject, self.fixtures, self.output/'copy.c',
                                        'BC_DECODER_WORK_TEST', 'work_quota_main.h')

    def test_motion_storage_probe_keeps_nested_layout_guards(self):
        relative = 'app_program/subsystem2/motion.c'
        for root in (self.subject, self.fixtures):
            (root/relative).parent.mkdir()
        subject = b'/* original motion */\n'
        probes = (b'#ifdef MOTION_STORAGE_PROBE\n#ifdef MOTION_COMPACT_RETAINED\n'
                  b'/* compact */\n#else\n/* wide */\n#endif\n'
                  b'#endif /* MOTION_STORAGE_PROBE */')
        (self.subject/relative).write_bytes(subject)
        (self.fixtures/relative).write_bytes(probes + b'\n/* later implementation */\n')
        selected = motion_storage_source(self.subject, self.fixtures, self.output/'storage')
        self.assertEqual(selected.read_bytes(), subject +
                         b'\n#include "motion_storage_main.h"\n\n'+probes+b'\n')

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
        incremental = next(r for r in recipes if r['name']=='barcode_incremental')
        fit = next(r for r in recipes if r['name']=='barcode_fit_error')
        fit_decoder = self.output/'additional-host/fit_score_main_decoder.c'
        self.assertIn(str(fit_decoder), fit['build'])
        self.assertIn(str(self.fixtures/'testing/subsystem3/fit_score_main.c'), fit['build'])
        self.assertIn('-DBC_DECODER_FIT_TEST', fit['build'])
        self.assertEqual(fit_decoder.read_bytes(), self.decoder +
                         b'\n#include "fit_score_main.h"\n\n'+self.fit_probes+b'\n')
        self.assertIn('-DTEST_EXTREMA_COUNT_MEMBER=n', incremental['build'])
        self.assertIn('-DBC_DECODER_WORK_TEST', incremental['build'])
        self.assertIn('-DS3_BARCODE_SYNC_REFERENCE', incremental['build'])
        self.assertIn('-DCOURSE_DRIVE_MEMBER=drive', course['build'])
        self.assertIn('-DS3_SENSOR_TO_AXLE_MM=0', course['build'])
        self.assertIn(str(self.subject/'app_program/controller/s3_drive.c'), course['build'])
        self.assertEqual(json.loads((self.output/'additional-host-recipes.json').read_text())['status'], 'PREPARED')

    def test_bad_fixture_retains_failed_preparation_receipt(self):
        (self.fixtures/'app_program/subsystem3/bc_decoder.c').write_bytes(b'no probe block\n')
        with self.assertRaises(ValueError):
            additional_host_recipes(self.subject, self.fixtures, self.output, '/recording-cc', ['-O2'])
        report = json.loads((self.output/'additional-host-recipes.json').read_text())
        self.assertEqual(report['status'], 'FAIL')
        self.assertTrue(report['errors'])
        self.assertEqual(report['recipes'], [])


if __name__ == '__main__':
    unittest.main()
