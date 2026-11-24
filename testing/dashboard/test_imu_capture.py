import csv
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

from dashboard import imu_capture
from dashboard import hump_events
from dashboard.state import DashboardState
from dashboard.traces import TraceStore
from dashboard.transport import telemetry_topics


def packet(run, sequence, *samples, boot=0x10203040, version=1):
    header = struct.pack("<BBHII", version, len(samples), run, sequence, boot)
    row = "<IhhhhhhBB" if version == 1 else "<IhhhhhhBBiiBB"
    return header + b"".join(struct.pack(row, *sample) for sample in samples)


def sample(timestamp, flags=3, safety=0):
    return timestamp, -15, 25, 1240, -5, 10, 1210, flags, safety


def csv_rows(data):
    return list(csv.DictReader(io.StringIO(data.decode("utf-8"))))


class ImuCaptureTests(unittest.TestCase):
    def test_discard_log_survives_stop_next_run_reconnect_and_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            state = DashboardState(TraceStore(folder))
            topic = hump_events.TOPIC
            self.assertIn(topic, telemetry_topics(state, state.heartbeat_topic))
            first = struct.pack("<BBHIIIIHH", 1, 10, 8, 0x629, 1, 75638, 13244, 9730, 0)
            second = struct.pack("<BBHIIIIHH", 1, 17, 9, 0x629, 2, 95761, 3155, 7942, 1)
            state.ingest(topic, first, retained=True)
            state.ingest(topic, first[:-1])
            state.ingest(topic, bytes([2]) + first[1:])
            state.ingest(topic, first[:1] + bytes([255]) + first[2:])
            self.assertEqual(state.snapshot()["counters"]["malformed"], 3)
            self.assertEqual(state.snapshot()["hump_events"]["files"], [])
            state.ingest(topic, first)
            state.ingest(topic, second)
            state.ingest(topic, first)
            snapshot = state.snapshot()["hump_events"]
            self.assertEqual(len(snapshot["files"]), 2)
            latest, prior = snapshot["files"]
            rows = csv_rows(state.download_imu(prior))
            self.assertEqual(len(rows), 1)
            self.assertEqual((rows[0]["rejection_reason"], rows[0]["firmware_run"],
                              rows[0]["peak_um"], rows[0]["crossing_travel_um"]),
                             ("reverse_travel", "8", "9730", "13244"))
            state = DashboardState(TraceStore(folder))
            self.assertEqual(state.snapshot()["hump_events"]["latest"], snapshot["latest"])
            state.ingest(topic, second)
            rows = csv_rows(state.download_imu(latest))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["firmware_dropped_events"], "1")
            with self.assertRaises(KeyError):
                state.download_imu("hump-../../config/wifi_secrets.h")

    def test_capture_is_disabled_by_default(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            state = DashboardState(TraceStore(folder))
            state.ingest(imu_capture.TOPIC, packet(12, 0, sample(20000)))
            self.assertNotIn(imu_capture.TOPIC, telemetry_topics(state, state.heartbeat_topic))
            self.assertIsNone(state.snapshot()["imu_captures"])
            self.assertEqual(state.snapshot()["counters"]["received"], 0)
            self.assertFalse((folder / "imu").exists())

    def test_capture_has_signed_axes_wrap_timing_gaps_and_detector_flags(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            state = DashboardState(TraceStore(folder), capture_imu=True)
            topic = imu_capture.TOPIC
            self.assertIn(topic, telemetry_topics(state, state.heartbeat_topic))
            state.ingest(topic, packet(12, 0xfffffffe, sample(0xfffffffb), sample(15, 0x33)))
            state.ingest(topic, packet(12, 0xffffffff, sample(15, 0x33), sample(20015, 0x43, 1)))
            state.ingest(topic, packet(12, 3, sample(80015, 4)))
            name = state.snapshot()["imu_captures"]["captures"][0]["name"]
            rows = csv_rows(state.download_imu(name))
            self.assertEqual([row["sample_sequence"] for row in rows], ["4294967294", "4294967295", "0", "3"])
            self.assertEqual([row["observed_elapsed_us"] for row in rows], ["0", "20", "20020", "80020"])
            self.assertEqual([row["missing_samples_before"] for row in rows], ["", "0", "0", "2"])
            self.assertEqual([rows[0]["raw_x_mg"], rows[0]["calibrated_z_mg"]], ["-15", "1210"])
            self.assertEqual([rows[1]["confirmation_pending"], rows[1]["upward_gate"]], ["1", "1"])
            self.assertEqual([rows[2]["tilted_lift_gate"], rows[2]["safety"]], ["1", "lifted"])
            self.assertEqual([rows[3]["raw_z_mg"], rows[3]["calibrated_z_mg"], rows[3]["sensor_overrun"]],
                             ["", "", "1"])

            state.ingest(topic, packet(13, 4, sample(100015)))
            state.ingest(topic, packet(13, 0, sample(20), boot=0x22334455))
            reopened = DashboardState(TraceStore(folder), capture_imu=True)
            captures = reopened.snapshot()["imu_captures"]["captures"]
            self.assertEqual(len(captures), 3)
            self.assertEqual({c["boot_tag"] for c in captures}, {"10203040", "22334455"})
            self.assertEqual(csv_rows(reopened.download_imu(name)), rows)
            with self.assertRaises(KeyError):
                reopened.download_imu("../../config/wifi_secrets.h")

    def test_hump_events_and_distances_are_saved_alongside_v1_samples(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            state = DashboardState(TraceStore(folder), capture_imu=True)
            topic = imu_capture.TOPIC
            missing = -(1 << 31)
            state.ingest(topic, packet(6, 0, sample(20000)))
            state.ingest(topic, packet(
                6, 1,
                (*sample(40000), missing, 0, 0x41, 0),
                (*sample(60000), 122, 122, 0x22, 0),
                (*sample(80000), 144, 122, 0x30, 9),
                (*sample(100000), -24, 122, 0x04, 10),
                (*sample(120000), 90, 7150, 0x49, 0),
                version=2,
            ))
            reopened = DashboardState(TraceStore(folder), capture_imu=True)
            capture, = reopened.snapshot()["imu_captures"]["captures"]
            rows = csv_rows(reopened.download_imu(capture["name"]))
            self.assertEqual([row["sample_sequence"] for row in rows], ["0", "1", "2", "3", "4", "5"])
            self.assertEqual([row["hump_events"] for row in rows],
                             ["", "armed", "started", "acceleration_skipped", "discarded", "armed|completed"])
            self.assertEqual([row["wheel_delta_um"] for row in rows], ["", "", "122", "144", "-24", "90"])
            self.assertEqual([row["crossing_travel_um"] for row in rows], ["", "0", "122", "122", "122", "7150"])
            self.assertEqual([row["hump_active"] for row in rows], ["", "0", "1", "1", "0", "0"])
            self.assertEqual([row["hump_armed"] for row in rows], ["", "1", "0", "0", "0", "1"])
            self.assertEqual([row["hump_rejection_reason"] for row in rows],
                             ["", "none", "none", "acceleration_spike", "reverse_travel", "none"])
            self.assertTrue(all(row["calibrated_z_mg"] == "1210" for row in rows))

    def test_rejects_incompatible_hump_packet_count_and_fields(self):
        good = (*sample(20000), 122, 122, 0, 0)
        for bad in (packet(0, 0, *(good,) * 6, version=2),
                    packet(0, 0, (*good[:-2], 0x80, 0), version=2),
                    packet(0, 0, (*good[:-1], 19), version=2),
                    packet(0, 0, (*good[:-3], -1, 0, 0), version=2),
                    packet(0, 0, good, version=2)[:-1]):
            with self.assertRaises(ValueError):
                imu_capture.decode_imu(bad)

    def test_malformed_and_retained_packets_dont_create_captures(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = DashboardState(TraceStore(Path(tmp)), capture_imu=True)
            topic = imu_capture.TOPIC
            good = packet(0, 0, sample(20000))
            state.ingest(topic, good, retained=True)
            for bad in (good[:-1], packet(0, 0), packet(0, 0, sample(20000, 0x80)),
                        packet(0, 0, sample(20000, safety=3))):
                state.ingest(topic, bad)
            snapshot = state.snapshot()
            self.assertEqual(snapshot["counters"]["malformed"], 4)
            self.assertEqual(snapshot["counters"]["received"], 0)
            self.assertEqual(snapshot["imu_captures"]["captures"], [])

    def test_size_limit_preserves_saved_samples_and_allows_next_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            state = DashboardState(TraceStore(folder), capture_imu=True)
            state.ingest(imu_capture.TOPIC, packet(8, 0, sample(20000)))
            first = state.snapshot()["imu_captures"]["captures"][0]["name"]
            limit = (folder / "imu" / first).stat().st_size + 10
            with patch.object(imu_capture, "MAX_CAPTURE_BYTES", limit):
                state.ingest(imu_capture.TOPIC, packet(8, 2, sample(60000)))
                state.ingest(imu_capture.TOPIC, packet(8, 3, sample(80000)))
                limited, = state.snapshot()["imu_captures"]["captures"]
                self.assertTrue(limited["limited"])
                self.assertEqual(limited["name"], first)
                row, = csv_rows(state.download_imu(first))
                self.assertEqual(row["sample_sequence"], "0")
                state.ingest(imu_capture.TOPIC, packet(9, 4, sample(100000)))
            newest, previous = state.snapshot()["imu_captures"]["captures"]
            self.assertEqual(newest["run"], 9)
            self.assertFalse(newest["limited"])
            self.assertEqual(previous["name"], first)
            reopened = DashboardState(TraceStore(folder), capture_imu=True)
            saved = {c["name"]: c for c in reopened.snapshot()["imu_captures"]["captures"]}
            self.assertTrue(saved[first]["limited"])


if __name__ == "__main__":
    unittest.main()
