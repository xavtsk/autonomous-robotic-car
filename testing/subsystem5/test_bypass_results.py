"""Regression checks for handling known misses and incomplete simulator runs."""

import unittest

from bypass_results import assess, read_baseline


class BypassResultsTest(unittest.TestCase):
    wall = "wall"
    row = "CASE\twall\t0\t1\t1\t-1\tFOLLOW\tCOMPLETE\t\t1\t1\t3.5\n"
    miss = row + "SUMMARY\t1\t0\n"
    reference = {wall: (-1.0, {"collision"}, True)}

    def test_known_collision_is_still_a_mission_failure(self):
        rows, errors, _ = assess(self.miss, 1, self.reference)
        self.assertFalse(rows[0]["passed"])
        self.assertEqual(errors, [])

    def test_worse_clearance_and_loss_of_reacquisition(self):
        observed = self.miss.replace("0\t1\t1\t-1", "0\t0\t1\t-1.01")
        _, errors, _ = assess(observed, 1, self.reference)
        self.assertTrue(any("clearance fell" in error for error in errors))
        self.assertTrue(any("lost line" in error for error in errors))

    def test_new_collision_and_extra_fault_in_known_case(self):
        _, errors, _ = assess(self.miss, 1, {self.wall: (0.5, set(), True)})
        self.assertTrue(any("new failure(s): collision" in error for error in errors))
        observed = self.miss.replace("FOLLOW\tCOMPLETE\t", "FAULT\tFAULT\tNO_LINE")
        _, errors, _ = assess(observed, 1, self.reference)
        self.assertTrue(any("NO_LINE" in error for error in errors))

    def test_improved_case_passes_both_verdicts(self):
        observed = self.miss.replace("0\t1\t1\t-1", "1\t1\t1\t0.2").replace(
            "SUMMARY\t1", "SUMMARY\t0")
        rows, errors, _ = assess(observed, 0, self.reference)
        self.assertTrue(rows[0]["passed"])
        self.assertEqual(errors, [])

    def test_missing_duplicate_crashed_and_non_finite_are_incomplete(self):
        for text, code in ((self.row, 1), (self.miss, 2), (self.miss, 0),
                           (self.miss.replace("wall", "wrong case"), 1),
                           (self.row + self.miss, 1),
                           (self.miss.replace("-1", "nan"), 1)):
            with self.subTest(text=text, code=code), self.assertRaises(ValueError):
                assess(text, code, self.reference)

    def test_reference_covers_85_runs_and_five_actual_misses(self):
        profiles = read_baseline()
        self.assertEqual(sum(len(rows) for rows in profiles.values()), 85)
        self.assertEqual(sum(bool(failures) for rows in profiles.values()
                             for _, failures, _ in rows.values()), 5)
