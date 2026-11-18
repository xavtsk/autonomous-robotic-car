"""Per-buddy CSV output from live and historical run archives."""
import csv
import io
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from dashboard.ir_protocol import decode_ir, encode_ir
from dashboard.run_csv import BUDDY_NAMES, encode_run_csv


def entry(buddy, kind, **data):
    return dict(buddy=buddy, kind=kind, received_at="2026-11-18T10:00:01+00:00", data=data)


def archive(*entries, **run_details):
    run = dict(id="test-run", started_at="2026-11-18T10:00:00+00:00", ended_at="2026-11-18T10:00:20+00:00",
               active=False, end_reason="Stopped")
    run.update(run_details)
    return dict(run=run, entries=list(entries), dropped_entries=0, mode="live")


def read_csv(log, buddy):
    return list(csv.DictReader(io.StringIO(encode_run_csv(log, buddy).decode("utf-8-sig"))))


class RunCsvTests(unittest.TestCase):
    def test_each_export_has_its_own_records(self):
        log = archive(
            entry(1, "heartbeat", alive=True, faults_valid=True, faults={"active": ["imu_fault"], "mask": 4}),
            entry(2, "motion", left_mms=-2, right_mms=22),
            entry(3, "state", state="line_following"),
            entry(4, "terrain", tilt_valid=True, pitch_deg=-2.5, roll_deg=0.0),
            entry(5, "avoidance", range_mm=100, state="moving", route="left", no_echo=False),
        )
        for buddy, (record_type, column, expected) in enumerate((
            ("heartbeat", "active_faults", "imu_fault"),
            ("motion", "left_speed_mm_s", "-2"),
            ("state", "navigation_state", "line_following"),
            ("terrain", "pitch_deg", "-2.5"),
            ("avoidance", "range_mm", "100"),
        ), 1):
            with self.subTest(buddy=BUDDY_NAMES[buddy]):
                row, = read_csv(log, buddy)
                self.assertEqual(row["run_id"], "test-run")
                self.assertEqual(row["recorded_mode"], "live")
                self.assertEqual(row["run_end_reason"], "Stopped")
                self.assertEqual(row["received_at"], "2026-11-18T10:00:01+00:00")
                self.assertEqual(row["record_type"], record_type)
                self.assertEqual(row[column], expected)

    def test_buddy3_valid_darkness_and_barcode_outcomes(self):
        frame = decode_ir(encode_ir(10000, 8, [100, 200, 4000], [0, 500, 1000], (1, 0, 2), normalised_valid=5))
        preview, accepted, unmapped, last_status = read_csv(archive(
            entry(3, "ir_preview", **frame),
            entry(3, "barcode", letter="A", action="left"),
            entry(3, "barcode", letter="Z", action="unknown"),
            entry(3, "navigation", barcode_status="rejected", line_flags=0, line_error=0),
        ), 3)
        self.assertEqual(preview["left_darkness_0_1000"], "0")
        self.assertEqual(preview["right_darkness_0_1000"], "")
        self.assertEqual(preview["right_raw_adc"], "200")
        self.assertEqual(preview["right_darkness_valid"], "False")
        self.assertEqual(preview["barcode_darkness_0_1000"], "1000")
        self.assertEqual(preview["barcode_surface"], "black")
        self.assertEqual(accepted["decoder_accepted"], "True")
        self.assertEqual(accepted["mapped_action_known"], "True")
        self.assertEqual(unmapped["decoder_accepted"], "True")
        self.assertEqual(unmapped["mapped_action_known"], "False")
        self.assertEqual(last_status["last_decoder_outcome"], "rejected")
        self.assertEqual(last_status["decoder_accepted"], "")
        self.assertEqual(last_status["line_error"], "")

    def test_buddy4_legacy_summary_and_absent_tilt_heights(self):
        legacy, = read_csv(archive(hump_count=2, peak_mm=12), 4)
        self.assertEqual(legacy["record_type"], "run_summary")
        self.assertEqual((legacy["run_hump_count"], legacy["run_highest_hump_mm"]), ("2", "12"))
        self.assertEqual(legacy["pitch_deg"], "")
        terrain, zero_count = read_csv(archive(
            entry(4, "terrain", tilt_valid=False, pitch_deg=None, roll_deg=None),
            entry(4, "hump", count=0, height_mm=0),
            hump_count=0,
        ), 4)
        self.assertEqual((terrain["tilt_valid"], terrain["pitch_deg"]), ("False", ""))
        self.assertEqual((zero_count["hump_count"], zero_count["reported_peak_hump_mm"]), ("0", ""))

    def test_missing_buddy_and_spreadsheet_formulas(self):
        log = archive(entry(1, "control_reply", reason="@unexpected"), end_reason="=test()")
        existing, = read_csv(log, 1)
        no_telemetry, = read_csv(log, 5)
        self.assertEqual((existing["run_end_reason"], existing["reason"]), ("'=test()", "'@unexpected"))
        self.assertIn("No telemetry", no_telemetry["note"])
        self.assertEqual(no_telemetry["range_mm"], "")
        with self.assertRaises(ValueError):
            encode_run_csv(log, 6)


if __name__ == "__main__":
    unittest.main()
