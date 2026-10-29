"""Route reconstruction from car/telemetry/pose, without a robot or broker."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from dashboard.route import RouteTracker, MAX_SENT_POINTS
from dashboard.state import DashboardState, POSE_TOPIC
from dashboard.traces import TraceStore


def pose_bytes(cm, heading):
    return bytes([3]) + ((cm % 4096) | (heading << 12)).to_bytes(3, "little")


class RouteTrackerTests(unittest.TestCase):
    def setUp(self):
        self.route = RouteTracker(clock=lambda: 0.0)

    def drive(self, start_cm, end_cm, heading, step=5):
        """Feed poses every `step` cm from start to end at a fixed heading."""
        direction = 1 if end_cm >= start_cm else -1
        for d in range(start_cm + direction * step, end_cm + direction, direction * step):
            self.route.pose(d % 4096, heading % 360, "t")

    def last(self):
        return self.route.snapshot()["runs"][0]

    def test_square_left_turns_closes_near_start(self):
        self.route.pose(0, 0, "t")                       # START
        self.drive(0, 100, 0)                            # up
        self.route.pose(100, 90, "t")                    # spin left on the spot
        self.drive(100, 200, 90)                         # left (-x)
        self.route.pose(200, 180, "t")
        self.drive(200, 300, 180)                        # down
        self.route.pose(300, 270, "t")
        self.drive(300, 400, 270)                        # right (+x), back to START
        run = self.last()
        end = run["points"][-1]
        self.assertAlmostEqual(end["x"], 0.0, delta=0.5)
        self.assertAlmostEqual(end["y"], 0.0, delta=0.5)
        self.assertEqual(run["path_m"], 4.0)
        self.assertTrue(run["active"])
        self.assertFalse(run["joined_mid_run"])
        # Leg directions: up first, then -x after a LEFT turn.
        first_leg = run["points"][20]
        self.assertAlmostEqual(first_leg["y"], 100.0, delta=0.5)
        self.assertAlmostEqual(run["points"][41]["x"], -100.0, delta=0.5)

    def test_distance_unwraps_across_4096_and_reverse(self):
        self.route.pose(0, 0, "t")
        self.drive(0, 4200, 0, step=50)                  # 42 m, crosses the wrap
        self.assertEqual(self.last()["distance_m"], 42.0)
        self.drive(4200, 4150, 0, step=10)               # reverse 50 cm
        run = self.last()
        self.assertEqual(run["distance_m"], 41.5)
        self.assertEqual(run["path_m"], 42.5)
        self.assertAlmostEqual(run["points"][-1]["y"], 4150.0, delta=0.5)

    def test_reverse_past_start(self):
        self.route.pose(0, 0, "t")
        self.drive(0, -30, 0, step=10)
        run = self.last()
        self.assertEqual(run["distance_m"], -0.3)
        self.assertAlmostEqual(run["points"][-1]["y"], -30.0, delta=0.5)

    def test_markers_at_latest_pose_and_run_end(self):
        self.route.pose(0, 0, "t0")
        self.drive(0, 120, 0)
        self.route.telemetry("barcode", dict(letter="Z", action="left"), "t1")
        self.route.telemetry("state", dict(state="turning_left"), "t2")
        self.route.telemetry("hump", dict(count=1, height_mm=21), "t3")
        self.route.telemetry("obstacle", dict(distance_mm=180, width_mm=140, route="left"), "t4")
        self.route.telemetry("state", dict(state="line_following"), "t5")   # no marker
        self.route.telemetry("state", dict(state="stopped"), "t6")
        run = self.last()
        kinds = [m["kind"] for m in run["markers"]]
        self.assertEqual(kinds, ["barcode", "turn", "hump", "obstacle", "end"])
        self.assertEqual(run["markers"][0]["label"], "Barcode Z → left")
        self.assertEqual(run["markers"][0]["d"], 120)
        self.assertAlmostEqual(run["markers"][0]["y"], 120.0, delta=0.5)
        self.assertFalse(run["active"])
        # Events after the run ended are not placed on it.
        self.route.telemetry("barcode", dict(letter="Y", action="right"), "t7")
        self.assertEqual(len(self.last()["markers"]), 5)

    def test_new_run_after_stop_and_after_lost_stop_report(self):
        self.route.pose(0, 0, "a")
        self.drive(0, 80, 0)
        self.route.telemetry("state", dict(state="stopped"), "b")
        self.route.pose(0, 0, "c")                       # next START
        self.assertEqual([r["id"] for r in self.route.snapshot()["runs"]], [2, 1])
        self.drive(0, 90, 0)
        self.route.pose(0, 0, "d")                       # stop report lost, new START
        runs = self.route.snapshot()["runs"]
        self.assertEqual([r["id"] for r in runs], [3, 2, 1])
        self.assertFalse(runs[1]["active"])
        self.assertTrue(runs[0]["active"])

    def test_heading_zero_at_exactly_40_96_m_is_not_a_new_run(self):
        self.route.pose(0, 0, "t")
        self.drive(0, 4200, 0, step=8)                   # passes wrapped value 0 at 4096
        runs = self.route.snapshot()["runs"]
        self.assertEqual(len(runs), 1)
        self.assertEqual(runs[0]["distance_m"], 42.0)

    def test_joined_mid_run(self):
        self.route.pose(1234, 45, "t")
        run = self.last()
        self.assertTrue(run["joined_mid_run"])
        self.assertEqual(run["distance_m"], 0.0)
        self.route.pose(1244, 45, "t")
        self.assertEqual(self.last()["distance_m"], 0.1)

    def test_browser_payload_is_decimated(self):
        self.route.pose(0, 0, "t")
        for d in range(1, 5001):
            self.route.pose(d % 4096, 0, "t")
        run = self.last()
        self.assertEqual(run["point_count"], 5001)
        self.assertLessEqual(len(run["points"]), MAX_SENT_POINTS + 1)
        self.assertEqual(run["points"][-1]["d"], 5000)   # latest point always kept


class DashboardRouteTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = DashboardState(TraceStore(Path(self.tmp.name)), clock=lambda: 100.0)
        self.state.set_connection("connected")

    def tearDown(self):
        self.tmp.cleanup()

    def test_pose_and_barcode_reach_the_route(self):
        self.state.ingest(POSE_TOPIC, pose_bytes(0, 0))
        self.state.ingest(POSE_TOPIC, pose_bytes(25, 0))
        self.state.ingest("car/telemetry/barcode", bytes.fromhex("0359"))  # Z, left
        snapshot = self.state.snapshot()
        run = snapshot["route"]["runs"][0]
        self.assertEqual(run["distance_m"], 0.25)
        self.assertEqual(run["markers"][0]["label"], "Barcode Z → left")
        self.assertEqual(snapshot["diagnostics"]["raw_messages"][-1]["topic"], POSE_TOPIC)
        self.assertNotIn("pose", snapshot["sections"])   # live-only, not a snapshot section

    def test_malformed_pose_rejected(self):
        before = self.state.snapshot()["counters"]["malformed"]
        self.state.ingest(POSE_TOPIC, bytes.fromhex("03008016"))           # heading 360
        self.state.ingest(POSE_TOPIC, bytes.fromhex("030000"))             # wrong length
        snapshot = self.state.snapshot()
        self.assertEqual(snapshot["counters"]["malformed"], before + 2)
        self.assertEqual(snapshot["route"]["runs"], [])

    def test_demo_draws_a_route(self):
        now = [0.0]
        state = DashboardState(TraceStore(Path(self.tmp.name)), mode="demo", clock=lambda: now[0])
        state.demo_select("following")
        for _ in range(300):                                               # 30 s of demo
            now[0] += 0.1
            state.demo_ir_tick()
        run = state.snapshot()["route"]["runs"][0]
        self.assertTrue(run["active"])
        self.assertGreater(run["path_m"], 1.0)


if __name__ == "__main__":
    unittest.main()
