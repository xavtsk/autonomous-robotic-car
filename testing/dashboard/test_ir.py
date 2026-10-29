"""Independent C/Python IR wire agreement, validity, gaps and bounded plotting."""
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from dashboard.ir_protocol import IRBuffer, decode_ir, encode_ir
from dashboard.state import DashboardState
from dashboard.traces import TraceStore
from buddy1_live import Folder, handle


class IRTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        folder = tempfile.TemporaryDirectory()
        cls.addClassCleanup(folder.cleanup)
        binary = Path(folder.name) / "ir_test"
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
                        str(ROOT / "testing/integration/ir_debug_test.c"),
                        str(ROOT / "app_program/controller/ir_debug.c"), "-o", str(binary)], check=True)
        output = subprocess.check_output([str(binary)], text=True)
        cls.vector = bytes.fromhex(next(line.split()[1] for line in output.splitlines() if line.startswith("IR_VECTOR ")))

    def test_real_c_codec_matches_python_and_preserves_zero(self):
        decoded = decode_ir(self.vector)
        self.assertEqual(decoded["timestamp_us"], 0xfffffff0)
        self.assertEqual(decoded["sequence"], 255)
        self.assertEqual(decoded["sensors"]["left"]["darkness"], 0)
        self.assertTrue(decoded["sensors"]["left"]["valid"])
        self.assertEqual(decoded["sensors"]["right"]["raw"], 4095)
        self.assertEqual(decoded["sensors"]["barcode"]["darkness"], 555)
        self.assertEqual(encode_ir(0xfffffff0, 255, [100, 4095, 2300], [0, 1000, 555], [1, 2, 2]), self.vector)

    def test_invalid_length_version_padding_and_reserved_fields(self):
        packets = [self.vector[:-1], bytes([2]) + self.vector[1:], self.vector + b"\0"]
        padding = bytearray(self.vector); padding[-1] |= 0x80; packets.append(padding)
        base = int.from_bytes(self.vector[1:], "little")
        for value in ((base & ~(1023 << 68)) | 1023 << 68,
                      (base & ~(3 << 104)) | 3 << 104,
                      base & ~(7 << 98)):
            packets.append(b"\x01" + value.to_bytes(15, "little"))
        for packet in packets:
            with self.subTest(packet=bytes(packet).hex()), self.assertRaises(ValueError):
                decode_ir(packet)

    def test_raw_and_normalised_validity_are_independent(self):
        packet = encode_ir(10, 1, [100, 900, 3000], [0, 800, 900], raw_valid=5, normalised_valid=1)
        sensors = decode_ir(packet)["sensors"]
        self.assertIsNone(sensors["right"]["raw"])
        self.assertEqual(sensors["barcode"]["raw"], 3000)
        self.assertIsNone(sensors["barcode"]["darkness"])
        self.assertTrue(sensors["left"]["valid"])

    def test_clock_sequence_wrap_duplicates_gaps_and_restart(self):
        now = [1.0]
        buffer = IRBuffer(lambda: now[0])
        self.assertTrue(buffer.add(decode_ir(self.vector)))
        self.assertFalse(buffer.add(decode_ir(self.vector)))
        frame = decode_ir(encode_ir(49984, 0, [100] * 3, [20] * 3))
        buffer.add(frame)
        self.assertEqual(buffer.samples[-1]["t_ms"], 50)
        buffer.add(decode_ir(encode_ir(149984, 2, [100] * 3, [20] * 3)))
        self.assertEqual(buffer.skipped, 1)
        buffer.add(decode_ir(encode_ir(650000, 12, [100] * 3, [20] * 3)))
        self.assertTrue(buffer.samples[-1]["gap_before"])
        buffer.add(decode_ir(encode_ir(100, 0, [100] * 3, [20] * 3)))
        self.assertEqual(len(buffer.samples), 1)
        self.assertEqual(buffer.restarts, 1)
        now[0] += 1.1
        self.assertTrue(buffer.snapshot(True)["stale"])

    def test_buffers_are_bounded_and_disconnect_does_not_make_zeros(self):
        buffer = IRBuffer(lambda: 1)
        for i in range(1500):
            buffer.add(decode_ir(encode_ir(i * 50000, i % 256, [100] * 3, [300] * 3)))
        self.assertEqual(len(buffer.samples), 1200)
        snapshot = buffer.snapshot(False)
        self.assertTrue(snapshot["stale"])
        self.assertEqual(snapshot["latest"]["sensors"]["left"]["darkness"], 300)

    def test_dashboard_state_and_terminal_viewer_use_binary_ir(self):
        with tempfile.TemporaryDirectory() as folder:
            state = DashboardState(TraceStore(Path(folder)))
            state.set_connection("connected")
            state.ingest("car/debug/ir", self.vector)
            self.assertEqual(state.sensor_snapshot()["received"], 1)
            self.assertEqual(state.snapshot()["diagnostics"]["ir"]["latest"]["sensors"]["right"]["darkness"], 1000)
            state.ingest("car/debug/ir", self.vector, retained=True)
            self.assertEqual(state.sensor_snapshot()["received"], 1)
            state.ingest("car/debug/ir", b"bad")
            self.assertEqual(state.snapshot()["counters"]["malformed"], 1)
        output = io.StringIO()
        handle("car/debug/ir " + self.vector.hex(), Folder(True), output)
        self.assertIn("left=0 right=1000 barcode=555", output.getvalue())


if __name__ == "__main__":
    unittest.main()
