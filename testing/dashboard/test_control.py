"""Controller diagnostics: wire independence, no invented histories/resources."""
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools"))
from dashboard.control_protocol import FIELDS,TOPIC,decode_control,encode_control
from dashboard.debug_history import DebugHistory
from dashboard.state import DashboardState
from dashboard.traces import TraceStore
from buddy1_live import Folder,handle


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        folder=tempfile.TemporaryDirectory();cls.addClassCleanup(folder.cleanup)
        binary=Path(folder.name)/"control_test"
        subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror",str(ROOT/"testing/integration/control_debug_test.c"),
                        str(ROOT/"app_program/controller/control_debug.c"),"-o",str(binary)],check=True)
        output=subprocess.check_output([str(binary)],text=True)
        cls.vector=bytes.fromhex(next(line.split()[1] for line in output.splitlines() if line.startswith("CONTROL_VECTOR ")))

    def test_independent_c_python_wire_equality_and_signed_boundaries(self):
        frame=decode_control(self.vector);d=frame["data"]
        self.assertEqual(sum(width for _,width,_ in FIELDS),551)
        self.assertEqual(d["encoder_left"],-524288)
        self.assertEqual(d["encoder_right"],524287)
        self.assertEqual(d["setpoint_right"],-50)
        self.assertEqual(d["duty_right"],-100)
        self.assertEqual(d["junction_events"],65535)
        self.assertEqual(encode_control(**d),self.vector)

    def test_invalid_lengths_padding_enums_and_masks(self):
        bad=bytearray(self.vector);bad[-1]|=0x80
        for payload in (self.vector[:-1],self.vector+b"\0",bytes([2])+self.vector[1:],bad):
            with self.assertRaises(ValueError):decode_control(payload)
        for values in (dict(motion_state=6),dict(drive_state=7),dict(line_error=101),dict(duty_left=-101),
                       dict(motion_valid=1),dict(barcode_black_limit=1001),dict(encoder_left=524288)):
            with self.assertRaises(ValueError):encode_control(**values)

    def test_laptop_rates_completed_seconds_and_bounded_history(self):
        now=[0.0];history=DebugHistory(lambda:now[0],"live")
        history.receive(16);now[0]=0.9;history.receive(70)
        self.assertEqual(history.snapshot(True)["rates"],[])
        now[0]=2.1
        self.assertEqual(history.snapshot(True)["rates"],
                         [dict(t_ms=0,messages=2,bytes=86),dict(t_ms=1000,messages=0,bytes=0)])
        for i in range(1500):
            now[0]=i
            history.report("motion",{"left_mms":i},True);history.receive(2);history.connection("connected")
        self.assertEqual(len(history.telemetry["motion"]),300)
        self.assertEqual(len(history.traffic),120)
        self.assertEqual(len(history.connections),60)

    def test_cached_snapshots_do_not_become_live_graph_points(self):
        with tempfile.TemporaryDirectory() as tmp:
            state=DashboardState(TraceStore(Path(tmp)))
            state._update("motion",dict(left_mms=100,right_mms=100),source="snapshot")
            self.assertEqual(state.debug_snapshot()["telemetry"]["motion"],[])
            state._update("motion",dict(left_mms=0,right_mms=0))
            state._update("motion",{},valid=False)
            points=state.debug_snapshot()["telemetry"]["motion"]
            self.assertEqual(points[0]["data"]["left_mms"],0)
            self.assertIsNone(points[1]["data"])

    def test_controller_ingest_events_stale_valid_zero_and_terminal(self):
        now=[1.0]
        with tempfile.TemporaryDirectory() as tmp:
            state=DashboardState(TraceStore(Path(tmp)),clock=lambda:now[0]);state.set_connection("connected")
            state.ingest(TOPIC,self.vector);state.ingest(TOPIC,self.vector,retained=True)
            self.assertEqual(state.debug_snapshot()["controller"]["received"],1)
            self.assertTrue(any("Controller following" in e["message"] for e in state.events))
            now[0]+=2;self.assertTrue(state.debug_snapshot()["controller"]["stale"])
            state.ingest(TOPIC,b"bad");self.assertEqual(state.counters["malformed"],1)
            state.set_connection("disconnected")
            self.assertEqual(state.debug_snapshot()["controller"]["latest"]["data"]["setpoint_right"],-50)
        out=io.StringIO();handle(TOPIC+" "+self.vector.hex(),Folder(True),out)
        self.assertIn("CONTROLLER",out.getvalue())

    def test_controller_history_wrap_and_bound(self):
        history=DebugHistory(lambda:1,"live")
        for i in range(500):
            history.controller.add(decode_control(encode_control(timestamp_us=(0xfffffff0+i*200000)&0xffffffff,
                                                                 sequence=i%256,motion_available=1,motion_valid=1)))
        self.assertEqual(len(history.controller.samples),300)
        self.assertEqual(history.controller.snapshot(True)["clock_restarts"],0)

    def test_demo_does_not_invent_buddy4_or_buddy5_measurements(self):
        with tempfile.TemporaryDirectory() as tmp:
            state=DashboardState(TraceStore(Path(tmp)),mode="demo");state.demo_select("following")
            history=state.debug_snapshot()
            self.assertEqual(history["mode"],"demo")
            self.assertGreater(history["controller"]["received"],0)
            for name in ("heading","hump","obstacle"):
                self.assertEqual(history["telemetry"][name],[])
