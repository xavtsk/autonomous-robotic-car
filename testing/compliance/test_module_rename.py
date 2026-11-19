"""Protect estimator renaming from hidden API or source-order changes."""
import unittest

from tools.compliance import verify_module_rename as rename


class ModuleRenameTests(unittest.TestCase):
    before = '/** @file estimate_motion.c */\n#include "estimate_motion.h"\nfloat motion_estimation_scale(void) { return 0.4f; }\n'

    def test_only_filename_text_changes_and_api_stays_the_same(self):
        after = self.before.replace('estimate_motion', 'motion_estimation')
        self.assertTrue(rename.compare_source(self.before, after)['tokens_identical_after_include_inversion'])

    def test_changed_numeric_expression_is_rejected(self):
        after = self.before.replace('estimate_motion', 'motion_estimation').replace('0.4f', '0.5f')
        with self.assertRaises(ValueError):
            rename.compare_source(self.before, after)

    def test_exported_api_rename_is_rejected(self):
        after = self.before.replace('estimate_motion', 'motion_estimation').replace('motion_estimation_scale', 'estimator_scale')
        with self.assertRaises(ValueError):
            rename.compare_source(self.before, after)

    def test_runtime_filename_string_is_not_silently_changed(self):
        before = 'const char * label = "estimate_motion.c";'
        with self.assertRaisesRegex(ValueError, 'outside'):
            rename.compare_source(before, before.replace('estimate_motion', 'motion_estimation'))

    def test_renamed_object_keeps_old_order_and_root_group(self):
        before = ['app_program/z.c', 'app_program/subsystem2/ctrl_motion.c',
                  'app_program/subsystem2/estimate_motion.c', 'app_program/subsystem2/motion.c']
        after = [p.replace('estimate_motion.c', 'motion_estimation.c') for p in before]
        self.assertEqual([p.replace('motion_estimation.c', 'estimate_motion.c')
                          for p in rename.expected_app_order(after, True)],
                         rename.expected_app_order(before, False))
        self.assertNotEqual(rename.expected_app_order(after, False), rename.expected_app_order(after, True))


if __name__ == '__main__':
    unittest.main()
