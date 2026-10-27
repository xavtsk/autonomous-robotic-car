"""Independent laptop decoder checks against packets emitted by the real C codec."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("codec", ROOT / "tools/buddy1_mqtt_codec.py")
codec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(codec)


class LaptopCodecTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        output = subprocess.check_output([str(ROOT / "build/buddy1/host_test")], text=True)
        print(output, end="")
        cls.vectors = {line.split()[1]: bytes.fromhex(line.split()[2])
                       for line in output.splitlines() if line.startswith("VECTOR ")}

    def test_c_packets_match_expected_values(self):
        expected = {
            "motion": dict(left_mms=120, right_mms=-118),
            "barcode": dict(letter="W", action="left"),
            "hump": dict(count=2, height_mm=45),
            "heading": dict(heading_deg=359),
            "obstacle": dict(distance_mm=180, width_mm=140, route="left"),
            "state": dict(state="line_following"),
            "faults": dict(mask=49, active=["motor_stall", "wifi_loss", "ir_sensor_fault"]),
        }
        snapshot = codec.decode("car/reply/snapshot", self.vectors["car/reply/snapshot"])
        for name, value in expected.items():
            with self.subTest(section=name):
                topic = "car/telemetry/" + name
                self.assertEqual(codec.decode(topic, self.vectors[topic]), value)
                self.assertEqual(snapshot[name], dict(valid=True, **value))
        self.assertTrue(codec.decode("car/status", self.vectors["car/status"])["alive"])

    def test_known_wire_examples(self):
        self.assertEqual(codec.decode("car/telemetry/motion", bytes.fromhex("023c7602")),
                         dict(left_mms=120, right_mms=-118))
        self.assertEqual(codec.decode("car/telemetry/motion", bytes.fromhex("02000000")),
                         dict(left_mms=0, right_mms=0))
        self.assertEqual(codec.encode_map("A", "B", "C", "D").hex(), "02208801")
        self.assertEqual(codec.decode("car/cmd/barcode_map", bytes.fromhex("02208801")),
                         dict(forward="A", left="B", right="C", uturn="D"))

    def test_all_invalid_inputs_rejected(self):
        cases = [
            ("car/telemetry/motion", "020000"),
            ("car/telemetry/motion", "0200000000"),
            ("car/telemetry/motion", "01000000"),
            ("car/telemetry/motion", "03000000"),
            ("car/telemetry/motion", "02000080"),
            ("car/telemetry/motion", "02000100"),
            ("car/telemetry/barcode", "021f"),
            ("car/telemetry/barcode", "02e0"),
            ("car/telemetry/heading", "026801"),
            ("car/telemetry/obstacle", "020000000003"),
            ("car/telemetry/faults", "0240"),
            ("car/status", "0200"),
            ("car/status", "0205"),
            ("car/cmd/snapshot", ""),
            ("car/cmd/barcode_map", "02000000"),
            ("car/cmd/barcode_map", "01208801"),
            ("car/cmd/snapshot", "01"),
        ]
        for topic, hex_value in cases:
            with self.subTest(topic=topic, value=hex_value), self.assertRaises(ValueError):
                codec.decode(topic, bytes.fromhex(hex_value))

    def test_valid_zero_is_not_invalid(self):
        # Snapshot validity bit 2 = hump; the whole body of the hump is zero.
        packet = bytes([codec.VERSION, 4]) + bytes(14)
        snapshot = codec.decode("car/reply/snapshot", packet)
        self.assertEqual(snapshot["hump"], dict(valid=True, count=0, height_mm=0))
        self.assertEqual(snapshot["motion"], dict(valid=False))

    def test_all_six_fault_bits_and_status_top_bit(self):
        for mask in range(64):
            decoded = codec.decode("car/telemetry/faults", bytes([codec.VERSION, mask]))
            self.assertEqual(decoded["mask"], mask)
            self.assertEqual(decoded["active"], [name for i, name in enumerate(codec.FAULTS)
                                                  if mask & (1 << i)])
            heartbeat = codec.decode("car/status", bytes([codec.VERSION, 3 | (mask << 2)]))
            self.assertEqual(heartbeat["faults"], decoded)
        self.assertEqual(codec.decode("car/telemetry/faults", bytes.fromhex("0220"))["active"],
                         ["ir_sensor_fault"])
        packet = bytearray([codec.VERSION, 64]) + bytearray(14)
        # IR bit at snapshot body offset 116, then first padding bit 117.
        packet[15] = 0x10
        self.assertEqual(codec.decode("car/reply/snapshot", packet)["faults"]["mask"], 32)
        packet[15] |= 0x20
        with self.assertRaises(ValueError):
            codec.decode("car/reply/snapshot", packet)


if __name__ == "__main__":
    unittest.main()
