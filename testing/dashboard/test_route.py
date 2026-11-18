"""Route reconstruction from car/telemetry/pose, without a robot or broker."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from dashboard.control_protocol import TOPIC as CONTROL_TOPIC, encode_control
from dashboard.ir_protocol import IR_TOPIC, encode_ir
from dashboard.route import RouteTracker, MAX_SENT_POINTS
from dashboard.run_control_protocol import STATUS_TOPIC, encode_status
from dashboard.state import DashboardState, POSE_TOPIC
from dashboard.traces import TraceStore


def pose_bytes(cm, heading):
    return bytes([3]) + ((cm % 4096) | (heading << 12)).to_bytes(3, "little")


def terrain_bytes(peak=None, *, tilt=True, climb=None):
    bits = int(tilt) | (2 if tilt else 0) << 23
    if climb is not None:
        bits |= 1 << 26 | climb << 27
    if peak is not None:
        bits |= 1 << 36 | peak << 37
    return bytes([3]) + bits.to_bytes(6, "little")


def hump_bytes(count, peak):
    return bytes([3]) + (count | peak << 16).to_bytes(4, "little")


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
        self.now = 100.0
        self.state = DashboardState(TraceStore(Path(self.tmp.name)), clock=lambda: self.now)
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

    def test_hump_logs_survive_stop_next_start_and_service_restart(self):
        ingest = self.state.ingest
        state_topic = "car/telemetry/state"
        terrain_topic = "car/telemetry/terrain"
        hump_topic = "car/telemetry/hump"
        ingest(terrain_topic, terrain_bytes(peak=90))  # previous drive, no boundary observed
        self.assertEqual(self.state.snapshot()["run_log"]["runs"], [])
        ingest(state_topic, b"\x03\x01")
        ingest(POSE_TOPIC, pose_bytes(0, 0))  # state and pose describe the same drive
        ingest(terrain_topic, terrain_bytes(climb=8))
        run, = self.state.snapshot()["run_log"]["runs"]
        self.assertEqual((run["hump_count"], run["peak_mm"]), (0, None))

        ingest(terrain_topic, terrain_bytes(peak=10))
        run, = self.state.snapshot()["run_log"]["runs"]
        self.assertEqual((run["hump_count"], run["peak_mm"]), (None, 10))
        ingest(hump_topic, hump_bytes(1, 10))
        ingest(hump_topic, hump_bytes(1, 10))  # repeat, not a second detection
        ingest(terrain_topic, terrain_bytes(peak=14))
        ingest(state_topic, b"\x03\x00")
        self.now += 0.5
        ingest(hump_topic, hump_bytes(2, 14))  # latest cumulative report arrived after STOP
        ingest(hump_topic, hump_bytes(1, 10))  # an older cumulative report cannot roll it back

        ingest(state_topic, b"\x03\x01")
        ingest(terrain_topic, terrain_bytes(tilt=False))
        newest, previous = self.state.snapshot()["run_log"]["runs"]
        self.assertEqual((newest["hump_count"], newest["peak_mm"]), (None, None))
        self.assertEqual((previous["hump_count"], previous["peak_mm"]), (2, 14))
        ingest(terrain_topic, terrain_bytes())
        ingest(state_topic, b"\x03\x07")  # fault ends the run too
        self.now += 3
        ingest(terrain_topic, terrain_bytes(peak=99))  # idle after the stopped run
        self.assertEqual(self.state.snapshot()["traces"], [])
        reopened = DashboardState(TraceStore(Path(self.tmp.name)))
        newest, previous = reopened.snapshot()["run_log"]["runs"]
        self.assertEqual((newest["hump_count"], newest["peak_mm"]), (0, None))
        self.assertEqual((previous["hump_count"], previous["peak_mm"]), (2, 14))
        self.assertTrue(all(run["ended_at"] and not run["active"] for run in (newest, previous)))

    def test_periodic_controller_status_records_runs_without_pose_or_state_events(self):
        def status(flags):
            self.state.ingest(STATUS_TOPIC, encode_status(robot_time_us=int(self.now * 1e6), flags=flags))

        status(3)  # idle before a drive
        self.assertEqual(self.state.snapshot()["run_log"]["runs"], [])
        status(9)  # running
        status(3)  # a stopped status may arrive ahead of the last queued pose
        self.now += 0.5
        self.state.ingest(POSE_TOPIC, pose_bytes(5, 0))
        self.now += 0.5
        status(3)
        self.assertTrue(self.state.snapshot()["run_log"]["runs"][0]["active"])
        self.now += 0.5
        status(3)
        run, = self.state.snapshot()["run_log"]["runs"]
        self.assertFalse(run["active"])
        self.assertIsNotNone(run["ended_at"])
        status(9)
        self.state.set_connection("connecting")
        run = self.state.snapshot()["run_log"]["runs"][0]
        self.assertFalse(run["active"])
        self.assertIsNone(run["ended_at"])

    def test_periodic_start_waits_for_boundary_before_logging_a_peak(self):
        ingest = self.state.ingest
        state_topic = "car/telemetry/state"
        terrain_topic = "car/telemetry/terrain"
        hump_topic = "car/telemetry/hump"
        ingest(state_topic, b"\x03\x01")
        ingest(hump_topic, hump_bytes(3, 20))
        ingest(state_topic, b"\x03\x00")
        ingest(STATUS_TOPIC, encode_status(flags=9))
        ingest(hump_topic, hump_bytes(3, 20))
        ingest(terrain_topic, terrain_bytes(peak=20))
        newest, previous = self.state.snapshot()["run_log"]["runs"]
        self.assertEqual((newest["hump_count"], newest["peak_mm"]), (None, None))
        self.assertEqual((previous["hump_count"], previous["peak_mm"]), (3, 20))

        ingest(terrain_topic, terrain_bytes())
        ingest(hump_topic, hump_bytes(1, 10))
        newest, previous = self.state.snapshot()["run_log"]["runs"]
        self.assertEqual((newest["hump_count"], newest["peak_mm"]), (1, 10))
        self.assertEqual((previous["hump_count"], previous["peak_mm"]), (3, 20))

    def test_run_download_keeps_telemetry_from_all_five_buddies(self):
        ingest = self.state.ingest
        state_topic = "car/telemetry/state"
        barcode_topic = "car/telemetry/barcode"
        ingest("car/telemetry/motion", bytes.fromhex("033c7602"))  # before START
        ingest(POSE_TOPIC, pose_bytes(0, 0), retained=True)
        self.assertEqual(self.state.snapshot()["run_log"]["runs"], [])

        ingest(state_topic, b"\x03\x01")
        ingest("car/status", bytes.fromhex("0303"))
        ingest(POSE_TOPIC, pose_bytes(0, 0))
        ingest(barcode_topic, bytes.fromhex("0359"), retained=True)  # old Z, left
        ingest(barcode_topic, bytes.fromhex("0356"))  # W, left
        ingest("car/telemetry/terrain", terrain_bytes(climb=8))
        ingest("car/telemetry/avoidance", bytes.fromhex("037186e5e02c"))
        ingest("car/telemetry/scan", bytes.fromhex("030ff40132680128f8c2069a3a"))
        ingest(state_topic, b"\x03\x00")
        self.now += 0.5
        ingest("car/telemetry/hump", hump_bytes(1, 14))
        self.now += 3
        ingest(barcode_topic, bytes.fromhex("0359"))  # idle Z, left

        first, = self.state.snapshot()["run_log"]["runs"]
        self.assertIsNotNone(first["ended_at"])
        self.assertTrue(first["end_reason"])
        self.assertTrue(first["has_details"])
        archive = self.state.download_run(first["id"])
        self.assertEqual(archive["schema_version"], 1)
        self.assertEqual(archive["run"]["id"], first["id"])
        self.assertEqual((archive["run"]["hump_count"], archive["run"]["peak_mm"]), (1, 14))
        entries = archive["entries"]
        self.assertEqual({entry["buddy"] for entry in entries}, {1, 2, 3, 4, 5})
        self.assertTrue(all(entry["received_at"] for entry in entries))
        details = {entry["kind"]: entry["data"] for entry in entries}
        self.assertTrue(details["heartbeat"]["alive"])
        self.assertEqual(details["pose"]["distance_cm_mod"], 0)
        self.assertEqual(details["terrain"]["climb_mm"], 8)
        self.assertEqual(details["hump"], {"count": 1, "height_mm": 14})
        self.assertEqual(details["scan"]["left_clear_mm"], 180)
        self.assertEqual([entry["data"]["state"] for entry in entries if entry["kind"] == "state"],
                         ["line_following", "stopped"])
        self.assertEqual([entry["data"]["letter"] for entry in entries if entry["kind"] == "barcode"], ["W"])
        self.assertNotIn("motion", details)

        ingest(state_topic, b"\x03\x01")
        ingest(barcode_topic, bytes.fromhex("0359"))
        self.state.set_connection("connecting")
        second, _ = self.state.snapshot()["run_log"]["runs"]
        self.assertFalse(second["active"])
        self.assertIsNone(second["ended_at"])
        self.assertTrue(second["end_reason"])
        reopened = DashboardState(TraceStore(Path(self.tmp.name)))
        self.assertEqual(reopened.download_run(first["id"])["entries"], entries)
        new_entries = reopened.download_run(second["id"])["entries"]
        self.assertEqual([entry["data"]["letter"] for entry in new_entries if entry["kind"] == "barcode"], ["Z"])

    def test_run_download_samples_periodic_motion_and_keeps_stop_event(self):
        self.state.ingest("car/telemetry/state", b"\x03\x01")
        for _ in range(5):
            self.state.ingest("car/telemetry/motion", bytes.fromhex("033c7602"))
            self.now += 0.1
        self.now += 1
        self.state.ingest("car/telemetry/motion", bytes.fromhex("033c7602"))
        self.state.ingest("car/telemetry/state", b"\x03\x00")
        run, = self.state.snapshot()["run_log"]["runs"]
        entries = self.state.download_run(run["id"])["entries"]
        motion = [entry["data"] for entry in entries if entry["kind"] == "motion"]
        self.assertEqual(motion, [{"left_mms": 120, "right_mms": -118}] * 2)
        self.assertIn({"state": "stopped"}, [entry["data"] for entry in entries if entry["kind"] == "state"])

    def test_buddy3_archive_records_sampled_darkness_and_decoder_changes(self):
        def ir(sequence, validity=7):
            return encode_ir(sequence * 50_000, sequence, (400, 800, 3200), (61, 207, 928),
                             surfaces=(1, 1, 2), normalised_valid=validity)

        ingest = self.state.ingest
        ingest(IR_TOPIC, ir(1))  # idle preview is available live but isn't part of a run
        ingest("car/telemetry/state", b"\x03\x01")
        ingest(IR_TOPIC, ir(2))
        self.now += 0.1
        ingest(IR_TOPIC, ir(3))  # inside the logging interval
        self.now += 0.15
        ingest(IR_TOPIC, ir(3))  # duplicate stays excluded even when the interval has passed
        ingest(IR_TOPIC, ir(4, validity=3))

        for sequence, (status, count) in enumerate(((0, 20), (2, 20), (2, 20), (3, 20), (1, 21))):
            self.now += 0.05
            ingest(CONTROL_TOPIC, encode_control(timestamp_us=sequence * 200_000, sequence=sequence,
                                               drive_state=1, barcode_status=status,
                                               barcode_events=count))
        ingest("car/telemetry/barcode", bytes.fromhex("0356"))  # W decoded, left
        ingest("car/telemetry/state", b"\x03\x00")
        ingest(IR_TOPIC, ir(5))

        run, = self.state.snapshot()["run_log"]["runs"]
        entries = self.state.download_run(run["id"])["entries"]
        previews = [entry["data"] for entry in entries if entry["kind"] == "ir_preview"]
        self.assertEqual([frame["sequence"] for frame in previews], [2, 4])
        self.assertEqual([previews[0]["sensors"][channel]["darkness"] for channel in ("left", "right", "barcode")],
                         [61, 207, 928])
        barcode_sensor = previews[1]["sensors"]["barcode"]
        self.assertIsNone(barcode_sensor["darkness"])
        self.assertFalse(barcode_sensor["valid"])
        self.assertEqual(barcode_sensor["raw"], 3200)
        outcomes = [entry["data"]["barcode_status"] for entry in entries if entry["kind"] == "navigation"]
        self.assertEqual(outcomes, ["none", "rejected", "timing error", "decoded"])
        self.assertIn({"letter": "W", "action": "left"},
                      [entry["data"] for entry in entries if entry["kind"] == "barcode"])

    def test_run_download_keeps_short_hump_and_avoidance_transitions(self):
        ingest = self.state.ingest
        ingest("car/telemetry/state", b"\x03\x01")
        ingest("car/telemetry/terrain", terrain_bytes())
        ingest("car/telemetry/terrain", terrain_bytes(climb=8))
        first_avoidance = int.from_bytes(bytes.fromhex("7186e5e02c"), "little")
        next_avoidance = (first_avoidance & ~(7 << 34)) | 4 << 34  # leg pass changed to extra
        for bits in (first_avoidance, next_avoidance):
            ingest("car/telemetry/avoidance", b"\x03" + bits.to_bytes(5, "little"))
        run, = self.state.snapshot()["run_log"]["runs"]
        entries = self.state.download_run(run["id"])["entries"]
        self.assertEqual([entry["data"]["climb_mm"] for entry in entries if entry["kind"] == "terrain"], [None, 8])
        passes = [entry["data"]["pass_step"] for entry in entries if entry["kind"] == "avoidance"]
        self.assertEqual(passes, ["step", "extra"])

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
