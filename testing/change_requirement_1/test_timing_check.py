"""Synthetic checker regressions only; these fixtures are not hardware evidence."""
from contextlib import redirect_stdout
import io
from pathlib import Path
import tempfile
import unittest

from testing.change_requirement_1 import timing_check as timing


def summary(source, maximum, *, minimum=5000, samples=100, valid=1):
    counter = "" if samples is None else f"[DIAG] timing source={source} samples={samples}\n"
    return counter + (f"[DIAG] interval source={source} gap_valid={valid} "
                      f"gap_min_us={minimum} gap_max_us={maximum} exec_max_us=100\n")


class TimingCheckTests(unittest.TestCase):
    def run_check(self, text, periods=None):
        with tempfile.TemporaryDirectory() as tmp:
            capture = Path(tmp) / "synthetic.log"
            capture.write_text(text, encoding="utf-8")
            output = io.StringIO()
            with redirect_stdout(output):
                result = timing.main(capture, periods)
            return result, output.getvalue()

    def test_defaults_follow_independent_firmware_macros(self):
        self.assertEqual(timing.source_periods(), {"motion": 10000, "ir": 5000})
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for source, (name, macro) in timing.PERIOD_DEFINITIONS.items():
                header = root / name
                header.parent.mkdir(parents=True, exist_ok=True)
                header.write_text(f"#define {macro} ({7 if source == 'ir' else 12}U) /* ms */\n")
            self.assertEqual(timing.source_periods(root), {"motion": 12000, "ir": 7000})

    def test_unrecognised_source_macro_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(OSError):
                timing.source_periods(Path(tmp))

    def test_15ms_ir_gap_cannot_pass_5ms_profile(self):
        result, output = self.run_check(summary("motion", 10000) + summary("ir", 15000))
        self.assertEqual(result, 1)
        self.assertIn("| ir | 5 | 100 | 5.000 | 15.000 | 0.100 | **FAIL** |", output)

    def test_two_period_boundaries_for_each_source(self):
        for motion, infrared, expected in [(19999, 9999, 0), (20000, 9999, 1),
                                           (19999, 10000, 1), (20000, 10000, 1)]:
            with self.subTest(motion=motion, infrared=infrared):
                result, _ = self.run_check(summary("motion", motion) + summary("ir", infrared))
                self.assertEqual(result, expected)

    def test_explicit_old_profile_uses_its_period(self):
        text = summary("motion", 10000) + summary("ir", 15000)
        result, _ = self.run_check(text, {"motion": 10000, "ir": 10000})
        self.assertEqual(result, 0)

    def test_reset_does_not_hide_earlier_large_gap(self):
        text = summary("motion", 10000) + summary("ir", 15000, samples=400)
        text += summary("ir", 5000, samples=10)
        self.assertEqual(self.run_check(text)[0], 1)

    def test_missing_or_empty_evidence_does_not_pass(self):
        self.assertEqual(self.run_check("")[0], 2)
        self.assertEqual(self.run_check(summary("motion", 10000))[0], 1)
        for count in (None, 0, 1):
            with self.subTest(samples=count):
                text = summary("motion", 10000) + summary("ir", 5000, samples=count)
                self.assertEqual(self.run_check(text)[0], 1)
        self.assertEqual(self.run_check(summary("motion", 10000) +
                                       summary("ir", 5000, valid=0))[0], 1)

    def test_bad_intervals_fail_and_initial_unavailable_interval_is_allowed(self):
        for minimum, maximum, valid in ((0, 5000, 1), (6000, 5000, 1), (5000, 5000, 2)):
            text = summary("motion", 10000) + summary("ir", maximum, minimum=minimum, valid=valid)
            self.assertEqual(self.run_check(text)[0], 1)
        text = summary("motion", 10000) + summary("ir", 0, minimum=0, valid=0, samples=1)
        text += summary("ir", 5000, samples=10)
        self.assertEqual(self.run_check(text)[0], 0)

    def test_cli_accepts_archived_period_overrides(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = Path(tmp) / "old.log"
            capture.write_text(summary("motion", 10000) + summary("ir", 15000))
            with redirect_stdout(io.StringIO()):
                result = timing.cli([str(capture), "--motion-period-ms", "10", "--ir-period-ms", "10"])
            self.assertEqual(result, 0)


if __name__ == "__main__":
    unittest.main()
