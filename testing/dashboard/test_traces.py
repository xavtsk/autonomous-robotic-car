"""Trace handling tests use the actual firmware text format, without hardware."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from dashboard import traces

SUMMARY = "[BCAP] run summary: episodes=1 not_stored=7 decoded=1 letters=Z rejected=0 timing_error=0 overflow=0 | raw:dark4:flags:mm:dt\n"
FULL = SUMMARY + "[BCAP] episode 1 t=20ms samples=3\nBC1 0: 100:0:1:0:9 3500:250:34:1:10 200:4:5:2:0\n[BCAP] end\n"


class TraceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.folder = Path(self.tmp.name)
        self.store = traces.TraceStore(self.folder)

    def tearDown(self):
        self.tmp.cleanup()

    def test_roundtrip_persistence_and_clock(self):
        meta = self.store.import_text("capture.txt", FULL)
        self.assertTrue(meta["complete"])
        self.assertFalse(meta["missing_records"])
        self.assertEqual(meta["summary"]["not_stored"], 7)
        detail = self.store.get(meta["id"])
        self.assertEqual([s["t_ms"] for s in detail["samples"]], [20, 30, 30])
        self.assertEqual(detail["samples"][1]["darkness"], 1000)
        self.assertTrue(detail["timestamps_approximate"])
        reloaded = traces.TraceStore(self.folder)
        self.assertEqual(reloaded.get(meta["id"]), detail)
        filename, raw = reloaded.download(meta["id"])
        self.assertEqual(raw, FULL)
        self.assertTrue(filename.startswith("capture_"))

    def test_missing_end_and_suffix_are_not_complete(self):
        for text in (FULL.replace("[BCAP] end\n", ""), FULL.replace("[BCAP] end", "[BCAP] end extra")):
            meta = self.store.import_text("partial.txt", text)
            self.assertFalse(meta["complete"])
            self.assertTrue(meta["missing_records"])

    def test_new_summary_cuts_previous_live_capture(self):
        first = self.store.ingest(SUMMARY)
        self.assertEqual(self.store.ingest("[BCAP] episode 1 t=20ms samples=3"), first)
        second = self.store.ingest(FULL)
        self.assertNotEqual(first, second)
        self.assertFalse(self.store.get(first)["complete"])
        self.assertTrue(self.store.get(second)["complete"])
        self.assertIsNone(self.store.ingest("unrelated log"))
        self.assertNotEqual(self.store.download(first)[0], self.store.download(second)[0])

    def test_missing_samples_never_invent_timestamps(self):
        text = SUMMARY + "[BCAP] episode 1 t=20ms samples=3\nBC1 1: 3500:250:34:1:10 200:4:5:2:10\n[BCAP] end\n"
        result = self.store.get(self.store.import_text("gap.txt", text)["id"])
        self.assertFalse(result["complete"])
        self.assertTrue(result["missing_records"])
        self.assertTrue(all(s["t_ms"] is None for s in result["samples"]))

    def test_saturated_interval_unknown_but_complete_transfer(self):
        result = traces.parse_trace(FULL.replace("34:1:10", "34:1:255"))
        self.assertTrue(result["complete"])
        self.assertEqual([s["t_ms"] for s in result["samples"]], [20, None, None])

    def test_duplicate_mqtt_row_is_deduplicated_by_episode_and_offset(self):
        row = "BC1 0: 100:0:1:0:9 3500:250:34:1:10 200:4:5:2:0\n"
        result = traces.parse_trace(FULL.replace("[BCAP] end", row + "[BCAP] end"))
        self.assertTrue(result["complete"])
        self.assertEqual(result["sample_count"], 3)
        header = "[BCAP] episode 1 t=20ms samples=3\n"
        replayed = FULL.replace(header, header + header) + "[BCAP] end\n"
        self.assertTrue(traces.parse_trace(replayed)["complete"])

    def test_malformed_fields_missing_headers_and_repeated_rows(self):
        cases = [FULL.replace("100:0:1:0:9", "9000:0:1:0:9"),
                 FULL.replace("100:0:1:0:9", "broken"),
                 FULL.replace("[BCAP] episode 1 t=20ms samples=3\n", ""),
                 FULL.replace("[BCAP] end", "BC1 0: 100:0:1:0:9\n[BCAP] end"),
                 FULL + "BC1 3: 100:0:1:0:9\n"]
        for text in cases:
            with self.subTest(text=text):
                result = traces.parse_trace(text)
                self.assertFalse(result["complete"])
                self.assertTrue(result["warnings"])

    def test_log_prefixes_and_arbitrary_text(self):
        prefixed = "\n".join("22:16:13  DEBUG  " + line for line in FULL.splitlines())
        self.assertTrue(self.store.import_text("terminal.txt", prefixed)["complete"])
        for text in ("random text", "[BCAP] end", "[BCAP] run summary: invalid"):
            with self.assertRaises(ValueError):
                self.store.import_text("bad.txt", text)

    def test_safe_paths_and_duplicate_filenames(self):
        for name in ("../outside.txt", "/absolute.txt", "C:\\evil.txt", "..", "bad\x00.txt"):
            with self.assertRaises(ValueError):
                self.store.import_text(name, FULL)
        first = self.store.import_text("same.txt", FULL)
        second = self.store.import_text("same.txt", FULL)
        self.assertNotEqual(first["id"], second["id"])
        for ident in ("../outside", "", "same.txt", None):
            with self.assertRaises(KeyError):
                self.store.download(ident)

    def test_byte_and_sample_bounds(self):
        with patch.object(traces, "MAX_INPUT_BYTES", 250):
            meta = self.store.import_text("big.txt", FULL + "unused\n" * 200)
            self.assertTrue(meta["truncated"])
            self.assertFalse(meta["complete"])
            self.assertLessEqual(len(self.store.download(meta["id"])[1].encode()), 250)
            self.assertEqual(traces._bounded("x" * 300), ("", True))
        with patch.object(traces, "MAX_SAMPLES", 2):
            result = traces.parse_trace(FULL.replace("samples=3", "samples=2"))
            self.assertLessEqual(result["sample_count"], 2)
            self.assertTrue(result["truncated"])
            self.assertFalse(result["complete"])

    def test_recent_list_rotation_preserves_evidence_files(self):
        untouched = self.folder / "personal.txt"
        untouched.write_text("keep", encoding="utf-8")
        with patch.object(traces, "MAX_TRACES", 2):
            records = [self.store.import_text("capture.txt", FULL) for _ in range(3)]
            self.assertEqual(len(self.store.list()), 2)
            with self.assertRaises(KeyError):
                self.store.get(records[0]["id"])
            self.assertTrue((self.folder / f"trace-{records[0]['id']}.txt").exists())
            self.assertEqual(untouched.read_text(), "keep")

    def test_real_successful_session_trace(self):
        path = ROOT / "barcode_trace_20261027_221613.txt"
        if not path.exists():
            self.skipTest("Optional local hardware capture is unavailable")
        result = self.store.import_text(path.name, path.read_text())
        self.assertTrue(result["complete"])
        self.assertEqual(result["summary"]["letters"], "ZZ")
        self.assertEqual(result["summary"]["decoded"], 2)
        self.assertEqual(result["sample_count"], 1927)


if __name__ == "__main__":
    unittest.main()
