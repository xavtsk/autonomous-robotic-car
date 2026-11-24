"""SQLite run archive, including compatibility with Buddy 4 summaries."""
from contextlib import closing
from pathlib import Path
import sqlite3
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

from dashboard import run_log


class RunLogTests(unittest.TestCase):
    def setUp(self):
        self.folder = TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name)
        self.now = 0

    def archive(self):
        return run_log.RunLog(self.path, clock=lambda: self.now)

    @staticmethod
    def run_record(run_id, **fields):
        return dict({"id": run_id, "started_at": "2026-11-17T10:00:00Z",
                     "ended_at": None, "hump_count": None, "peak_mm": None}, **fields)

    def test_migrates_old_summaries_and_preserves_unconfirmed_end(self):
        path = self.path / "buddy4-runs.sqlite3"
        with closing(sqlite3.connect(path)) as db, db:
            db.execute("""CREATE TABLE runs (id TEXT PRIMARY KEY, started_at TEXT NOT NULL,
                ended_at TEXT, hump_count INTEGER, peak_mm INTEGER)""")
            db.execute("INSERT INTO runs VALUES ('legacy', '2026-11-17T09:00:00Z', NULL, 3, 12)")
        archive = self.archive()
        legacy, = archive.snapshot(None, False)["runs"]
        self.assertEqual((legacy["hump_count"], legacy["peak_mm"]), (3, 12))
        self.assertIsNone(legacy["end_reason"])
        self.assertIsNone(legacy["ended_at"])
        self.assertFalse(legacy["has_details"])
        self.assertEqual(legacy["buddy_entries"], {})
        self.assertEqual(archive.download("legacy")["entries"], [])

        archive.record(self.run_record("observed"))
        archive.append("observed", 4, "terrain", "2026-11-17T10:00:01Z", {"pitch_deg": 1.2})
        archive.snapshot("observed", True)
        restarted = self.archive()
        observed, legacy = restarted.snapshot(None, False)["runs"]
        self.assertIsNone(legacy["end_reason"])
        self.assertEqual(observed["end_reason"], "Dashboard restarted")
        self.assertIsNone(observed["ended_at"])
        self.assertTrue(observed["has_details"])
        self.assertEqual(observed["buddy_entries"], {"4": 1})
        self.assertFalse(observed["active"])

    def test_checkpoint_persists_interleaved_entries_without_a_browser(self):
        archive = self.archive()
        archive.record(self.run_record("run"))
        data = {"left_mm_s": 120}
        archive.append("run", 2, "motion", "2026-11-17T10:00:01Z", data)
        data["left_mm_s"] = 999
        archive.append("run", 1, "status", "2026-11-17T10:00:02Z", {"faults": 0})
        self.now += run_log.CHECKPOINT_S
        archive.append("run", 2, "motion", "2026-11-17T10:00:03Z", {"left_mm_s": 130})
        with closing(sqlite3.connect(archive.path)) as db:
            self.assertEqual(db.execute("SELECT COUNT(*) FROM run_entries").fetchone()[0], 3)

        archive.record(self.run_record("run", ended_at="2026-11-17T10:00:04Z",
                                       end_reason="Stopped", hump_count=0))
        saved = self.archive().download("run")
        self.assertEqual(saved["schema_version"], 1)
        self.assertEqual(saved["run"]["end_reason"], "Stopped")
        self.assertEqual(saved["run"]["hump_count"], 0)
        self.assertEqual([(e["buddy"], e["data"]) for e in saved["entries"]],
                         [(2, {"left_mm_s": 120}), (1, {"faults": 0}), (2, {"left_mm_s": 130})])
        self.assertEqual(saved["dropped_entries"], 0)

    def test_retains_latest_entries_per_stream_and_reports_truncation(self):
        with patch.object(run_log, "MAX_ENTRIES_PER_STREAM", 3):
            archive = self.archive()
            archive.record(self.run_record("run"))
            archive.append("run", 5, "scan", "first", {"scan_id": 1})
            for step in range(4):
                archive.append("run", 2, "motion", f"buffer_{step}", {"step": step})
            archive.snapshot("run", True)
            for step in range(4, 6):
                archive.append("run", 2, "motion", f"next_{step}", {"step": step})
            saved = archive.download("run")
            self.assertEqual(saved["dropped_entries"], 3)
            self.assertEqual([e["received_at"] for e in saved["entries"]],
                             ["first", "buffer_3", "next_4", "next_5"])
            self.assertEqual(archive.snapshot("run", True)["runs"][0]["buddy_entries"], {"2": 3, "5": 1})
            filtered = self.archive().download("run", buddy=5)
            self.assertEqual([(e["buddy"], e["kind"]) for e in filtered["entries"]], [(5, "scan")])
            self.assertEqual(self.archive().download("run")["dropped_entries"], 3)

    def test_terminal_evidence_survives_restart_without_another_message(self):
        archive = self.archive()
        archive.record(self.run_record("run"))
        archive.record(self.run_record("run", ended_at="end", end_reason="Stopped"))
        archive.append("run", 3, "state", "end", {"state": "stopped"})
        entry, = self.archive().download("run")["entries"]
        self.assertEqual(entry["data"], {"state": "stopped"})

    def test_buddy4_legacy_wheel_rate_uses_archived_motion_samples(self):
        archive = self.archive()
        archive.record(self.run_record("run"))
        archive.append("run", 2, "motor_feedback", "first", dict(measured_left=0, measured_right=116, timestamp_us=32))
        old, = archive.download("run", buddy=4)["entries"]
        self.assertEqual(old, dict(buddy=4, kind="encoder_turn", received_at="first",
                                   data=dict(timestamp_us=32, encoder_turn_dps=57.3)))
        archive.append("run", 4, "encoder_turn", "second", dict(timestamp_us=64, encoder_turn_dps=None))
        new, = archive.download("run", buddy=4)["entries"]
        self.assertEqual(new["data"], dict(timestamp_us=64, encoder_turn_dps=None))

    def test_retries_pending_updates_after_database_lock(self):
        archive = self.archive()
        archive.record(self.run_record("run"))
        with closing(sqlite3.connect(archive.path)) as lock, lock:
            lock.execute("BEGIN EXCLUSIVE")
            archive.append("run", 3, "barcode", "seen", {"letter": "A"})
            self.assertIn("locked", archive.snapshot("run", True)["error"])
        self.assertIsNone(archive.snapshot("run", True)["error"])
        self.assertEqual([entry["data"] for entry in archive.download("run")["entries"]], [{"letter": "A"}])

    def test_older_logs_still_download_by_id(self):
        archive = self.archive()
        for index in range(run_log.MAX_RUN_LOGS + 1):
            identifier = str(index)
            archive.record(self.run_record(identifier, started_at=f"2026-11-17T10:00:{index:02}Z"))
            archive.append(identifier, 1, "status", "seen", {"alive": True})
        self.assertEqual(len(archive.snapshot(None, False)["runs"]), 20)
        reopened = self.archive()
        self.assertEqual([run["id"] for run in reopened.snapshot(None, False)["runs"]],
                         [str(index) for index in range(20, 0, -1)])
        self.assertEqual(archive.download("0")["entries"][0]["data"], {"alive": True})
        with self.assertRaises(KeyError):
            archive.download("missing")


if __name__ == "__main__":
    unittest.main()
