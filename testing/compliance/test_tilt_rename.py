"""Adversarial checks for a filename rename whose basename is also an API."""
import unittest

from tools.compliance.verify_tilt_rename import (
    OLD_C, NEW_C, application_order, check_paths, compare_source,
)


class TiltRenameTests(unittest.TestCase):
    def test_exact_filenames_change_but_the_function_identifier_does_not(self):
        old = ('/** @file tilt_estimation.c */\n#include "tilt_estimation.h"\n'
               'int tilt_estimation(void) { return 1; }\n')
        new = ('/** @file tilt.c */\n#include "tilt.h"\n'
               'int tilt_estimation(void) { return 1; }\n')
        self.assertTrue(compare_source(old, new)["tokens_identical_after_include_inversion"])
        with self.assertRaises(ValueError):
            compare_source(old, new.replace('int tilt_estimation', 'int tilt'))

    def test_log_strings_and_macro_includes_are_not_filename_permissions(self):
        for old in ('char *log = "tilt_estimation.c";\n',
                    '#define HEADER "tilt_estimation.h"\n#include HEADER\n'):
            with self.subTest(old=old), self.assertRaises(ValueError):
                compare_source(old, old.replace('tilt_estimation.', 'tilt.'))

    def test_operation_edits_and_guard_changes_are_rejected(self):
        for old, new in (
            ('int tilt_heading(void) { return 0; }', 'int tilt_heading(void) { return 1; }'),
            ('#ifndef BUDDY4_TILT_ESTIMATION_H\n#define BUDDY4_TILT_ESTIMATION_H\n#endif\n',
             '#ifndef TILT_H\n#define TILT_H\n#endif\n'),
        ):
            with self.subTest(new=new), self.assertRaises(ValueError):
                compare_source(old, new)

    def test_current_source_order_preserves_existing_estimator_sort_key(self):
        before = ['app_program/app_main.c', 'app_program/subsystem2/motion.c',
                  'app_program/subsystem2/motion_estimation.c',
                  'app_program/subsystem4/terr_data.c', OLD_C, OLD_C[:-1]+'h',
                  'app_program/subsystem5/avoid_obstacle.c']
        after = [path.replace('/tilt_estimation.', '/tilt.') for path in before]
        check_paths(before, after)
        order = application_order(before)
        self.assertLess(order.index('app_program/subsystem2/motion_estimation.c'),
                        order.index('app_program/subsystem2/motion.c'))

    def test_partial_ambiguous_and_colliding_pairs_are_rejected(self):
        before = [OLD_C, OLD_C[:-1]+'h']
        good = [NEW_C, NEW_C[:-1]+'h']
        for bad in (good[:1], good+[OLD_C], good+['testing/tilt.c']):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                check_paths(before, bad)

    def test_rename_cannot_conceal_an_unrelated_link_order_change(self):
        before = [OLD_C, OLD_C[:-1]+'h', 'app_program/subsystem5/avoid_obstacle.c']
        after = [NEW_C, NEW_C[:-1]+'h', 'app_program/subsystem5/other.c']
        with self.assertRaises(ValueError):
            check_paths(before, after)


if __name__ == '__main__':
    unittest.main()
