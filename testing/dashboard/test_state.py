"""Protocol and command-state regression tests without a robot or broker."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from dashboard.state import DashboardState
from dashboard.traces import TraceStore


class StateTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.now = 100.0
        self.state = DashboardState(TraceStore(Path(self.tmp.name)), clock=lambda: self.now)
        self.state.set_connection("connected")
        self.sent = []
        self.state.transport = type("Transport", (), {"publish": lambda _, t, p: self.sent.append((t, p))})()

    def tearDown(self):
        self.tmp.cleanup()

    def test_unknown_not_zero_and_valid_reverse_speed(self):
        self.assertFalse(self.state.snapshot()["sections"]["motion"]["valid"])
        self.state.ingest("car/telemetry/motion", bytes.fromhex("033c7602"))
        motion = self.state.snapshot()["sections"]["motion"]
        self.assertEqual(motion["data"], dict(left_mms=120, right_mms=-118))
        self.assertEqual(motion["freshness"], "live")
        self.assertFalse(motion["stale"])

    def test_all_seven_protocol_telemetry_types_are_available_to_dashboard(self):
        packets = {"motion": "033c7602", "barcode": "0356",
                   "hump": "0302002d00", "heading": "036701",
                   "obstacle": "03b4c00801", "state": "0301", "faults": "0331"}
        for name, value in packets.items():
            self.state.ingest("car/telemetry/" + name, bytes.fromhex(value))
        sections = self.state.snapshot()["sections"]
        self.assertEqual(set(sections), set(packets))
        self.assertTrue(all(item["valid"] for item in sections.values()))
        self.assertEqual(sections["hump"]["data"], dict(count=2, height_mm=45))
        self.assertEqual(sections["heading"]["data"], dict(heading_deg=359))
        self.assertEqual(sections["obstacle"]["data"], dict(distance_mm=180, width_mm=140, route="left"))
        self.assertEqual(sections["barcode"]["data"], dict(letter="W", action="left"))

    def test_bad_packets_and_retained_packets_do_not_prove_liveness(self):
        self.state.ingest("car/status", b"\x03\x03", retained=True)
        self.state.ingest("car/telemetry/motion", b"invalid")
        self.assertEqual(self.state.snapshot()["car"]["status"], "unknown")
        self.assertEqual(self.state.snapshot()["counters"]["malformed"], 1)

    def test_heartbeat_not_motion_freshness_and_event_age(self):
        self.state.ingest("car/telemetry/motion", bytes.fromhex("03000000"))
        self.state.ingest("car/telemetry/barcode", b"\x03\x59")
        self.now += 5
        self.state.ingest("mtk3smp-pico/heartbeat", b"\x03\x03")
        snapshot = self.state.snapshot()
        self.assertEqual(snapshot["car"]["status"], "online")
        self.assertTrue(snapshot["sections"]["motion"]["stale"])
        self.assertFalse(snapshot["sections"]["barcode"]["stale"])
        self.assertEqual(snapshot["sections"]["barcode"]["freshness"], "last_report")
        self.now += 16
        self.assertEqual(self.state.snapshot()["car"]["status"], "stale")

    def test_snapshot_zero_height_valid_but_age_unknown(self):
        self.state.ingest("car/reply/snapshot", bytes([3, 4]) + bytes(12))
        sections = self.state.snapshot()["sections"]
        self.assertEqual(sections["hump"]["data"], dict(count=0, height_mm=0))
        self.assertTrue(sections["hump"]["valid"])
        self.assertEqual(sections["hump"]["freshness"], "cached")
        self.assertTrue(sections["hump"]["stale"])
        self.assertFalse(sections["motion"]["valid"])

    def test_map_reply_timeout_and_no_automatic_resend(self):
        self.state.ingest("car/status", b"\x03\x03")
        letters = dict(forward="A", left="B", right="C", uturn="D")
        self.state.request("map", letters)
        self.assertEqual(self.sent, [("car/cmd/barcode_map", bytes.fromhex("03208801"))])
        self.assertEqual(self.state.snapshot()["commands"]["map"]["status"], "pending")
        with self.assertRaises(ValueError):
            self.state.request("snapshot")
        self.now += 9
        self.assertEqual(self.state.snapshot()["commands"]["map"]["status"], "timeout")
        self.state.ingest("car/reply/barcode_map", b"\x03\x00")
        self.assertEqual(self.state.snapshot()["commands"]["map"]["status"], "timeout")
        self.assertEqual(len(self.sent), 1)
        self.assertIn("Unsolicited/late", self.state.snapshot()["events"][0]["message"])

    def test_busy_and_disconnect_do_not_unlock_map(self):
        self.state.ingest("car/status", b"\x03\x03")
        self.state.request("map", dict(forward="A", left="Z", right="Y", uturn="C"))
        self.state.ingest("car/reply/barcode_map", b"\x03\x02")
        self.state.set_connection("disconnected")
        self.state.set_connection("connected")
        self.assertEqual(self.state.snapshot()["commands"]["map_lock"], "locked")
        # Explicit retry can reach the robot after its physical reboot; the
        # firmware's response, rather than a browser latch, gates configuration.
        self.state.request("map", dict(forward="A", left="B", right="C", uturn="D"))
        self.state.ingest("car/reply/barcode_map", b"\x03\x00")
        self.assertEqual(self.state.snapshot()["commands"]["map"]["status"], "accepted")

    def test_demo_never_publishes_and_modes_stay_separate(self):
        self.state.mode = "demo"
        self.state.demo_select("ready")
        self.state.request("snapshot")
        self.now += 1
        self.state.demo_tick()
        self.assertEqual(self.sent, [])
        self.assertEqual(self.state.snapshot()["commands"]["snapshot"]["status"], "accepted")
        self.state.demo_select("fault")
        self.assertIn("motor_stall", self.state.snapshot()["sections"]["faults"]["data"]["active"])
        self.state.demo_select("wifi_loss")
        self.assertTrue(self.state.snapshot()["sections"]["motion"]["stale"])

    def test_buffers_stay_bounded(self):
        for _ in range(400):
            self.state.ingest("car/status", b"\x03\x03")
            self.state.add_event(1, "info", "x")
        snapshot = self.state.snapshot()
        self.assertEqual(len(snapshot["events"]), 160)
        self.assertEqual(len(snapshot["diagnostics"]["raw_messages"]), 120)


if __name__ == "__main__":
    unittest.main()
