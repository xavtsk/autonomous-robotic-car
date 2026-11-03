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
        # Pose is live-only (no snapshot section).
        self.assertEqual(codec.decode("car/telemetry/pose", self.vectors["car/telemetry/pose"]),
                         dict(distance_cm_mod=2049, heading_deg=90))
        self.assertNotIn("pose", snapshot)

    def test_tilt_wire(self):
        self.assertEqual(codec.decode("car/telemetry/tilt", self.vectors["car/telemetry/tilt"]),
                         dict(tilt_valid=True, pitch_deg=12.3, roll_deg=-5.1, terrain="climbing"))
        self.assertNotIn("tilt", codec.decode("car/reply/snapshot", self.vectors["car/reply/snapshot"]))
        self.assertEqual(codec.decode("car/telemetry/tilt", bytes.fromhex("0300000001")),
                         dict(tilt_valid=False, pitch_deg=None, roll_deg=None, terrain="moving"))
        # Independent encoder for every pitch/roll code at the limits.
        for pitch, roll in ((-900, 900), (900, -900), (0, 0), (-1, 1)):
            body = 1 | ((pitch & 0x7ff) << 1) | ((roll & 0x7ff) << 12) | (7 << 23)
            payload = bytes([3]) + body.to_bytes(4, "little")
            self.assertEqual(codec.decode("car/telemetry/tilt", payload),
                             dict(tilt_valid=True, pitch_deg=pitch / 10, roll_deg=roll / 10, terrain="impact"))
        for bad in ("03f1070000",   # pitch +101.6
                    "0302000000",   # unavailable but nonzero angle
                    "0300000004",   # padding bit
                    "03000000",     # length
                    "0200000000"):  # version
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                codec.decode("car/telemetry/tilt", bytes.fromhex(bad))

    def test_pose_wire_and_unwrap(self):
        self.assertEqual(codec.decode("car/telemetry/pose", bytes.fromhex("03ff7f16")),
                         dict(distance_cm_mod=4095, heading_deg=359))
        # Forward across the wrap, reverse, and reverse across zero.
        self.assertEqual(codec.unwrap_distance(4090, 5), 4101)
        self.assertEqual(codec.unwrap_distance(4101, 4095 & 4095), 4095)
        self.assertEqual(codec.unwrap_distance(3, 4094), -2)
        self.assertEqual(codec.unwrap_distance(0, 2047), 2047)
        self.assertEqual(codec.unwrap_distance(0, 2048), -2048)

    def test_known_wire_examples(self):
        self.assertEqual(codec.decode("car/telemetry/motion", bytes.fromhex("033c7602")),
                         dict(left_mms=120, right_mms=-118))
        self.assertEqual(codec.decode("car/telemetry/motion", bytes.fromhex("03000000")),
                         dict(left_mms=0, right_mms=0))
        self.assertEqual(codec.encode_map("A", "B", "C", "D").hex(), "03208801")
        self.assertEqual(codec.decode("car/cmd/barcode_map", bytes.fromhex("03208801")),
                         dict(forward="A", left="B", right="C", uturn="D"))

    def test_all_invalid_inputs_rejected(self):
        cases = [
            ("car/telemetry/motion", "030000"),
            ("car/telemetry/motion", "0300000000"),
            ("car/telemetry/motion", "01000000"),
            ("car/telemetry/motion", "02000000"),
            ("car/telemetry/motion", "04000000"),
            ("car/telemetry/motion", "03000080"),
            ("car/telemetry/motion", "03000100"),
            ("car/telemetry/barcode", "031f"),
            ("car/telemetry/barcode", "03e0"),
            ("car/telemetry/heading", "036801"),
            ("car/telemetry/pose", "03008016"),
            ("car/telemetry/pose", "03000020"),
            ("car/telemetry/pose", "030000"),
            ("car/telemetry/obstacle", "030000000003"),
            ("car/telemetry/faults", "0340"),
            ("car/status", "0300"),
            ("car/status", "0305"),
            ("car/cmd/snapshot", ""),
            ("car/cmd/barcode_map", "03000000"),
            ("car/cmd/barcode_map", "01208801"),
            ("car/cmd/snapshot", "01"),
        ]
        for topic, hex_value in cases:
            with self.subTest(topic=topic, value=hex_value), self.assertRaises(ValueError):
                codec.decode(topic, bytes.fromhex(hex_value))

    def test_valid_zero_is_not_invalid(self):
        # Snapshot validity bit 2 = hump; the whole body of the hump is zero.
        packet = bytes([codec.VERSION, 4]) + bytes(12)
        snapshot = codec.decode("car/reply/snapshot", packet)
        self.assertEqual(snapshot["hump"], dict(valid=True, count=0, height_mm=0))
        self.assertEqual(snapshot["motion"], dict(valid=False))

    def test_minimum_width_terrain_ranges_and_padding(self):
        self.assertEqual(codec.WIDTHS,(18,8,25,9,26,3,6))
        self.assertEqual(codec.SNAPSHOT_BYTES,14)
        for height in range(512):
            packet=bytes([codec.VERSION])+(65535 | height<<16).to_bytes(4,"little")
            if height<=500:
                self.assertEqual(codec.decode("car/telemetry/hump",packet),dict(count=65535,height_mm=height))
            else:
                with self.assertRaises(ValueError):codec.decode("car/telemetry/hump",packet)
        for distance in (0,29,30,4000,4001,4095):
            packet=bytes([codec.VERSION])+(distance | 4095<<12 | 2<<24).to_bytes(4,"little")
            if 30<=distance<=4000:
                self.assertEqual(codec.decode("car/telemetry/obstacle",packet),
                                 dict(distance_mm=distance,width_mm=4095,route="right"))
            else:
                with self.assertRaises(ValueError):codec.decode("car/telemetry/obstacle",packet)
        with self.assertRaises(ValueError):
            codec.decode("car/telemetry/hump",bytes([codec.VERSION,0,0,0,2]))

    def test_all_six_fault_bits_and_status_top_bit(self):
        for mask in range(64):
            decoded = codec.decode("car/telemetry/faults", bytes([codec.VERSION, mask]))
            self.assertEqual(decoded["mask"], mask)
            self.assertEqual(decoded["active"], [name for i, name in enumerate(codec.FAULTS)
                                                  if mask & (1 << i)])
            heartbeat = codec.decode("car/status", bytes([codec.VERSION, 3 | (mask << 2)]))
            self.assertEqual(heartbeat["faults"], decoded)
        self.assertEqual(codec.decode("car/telemetry/faults", bytes.fromhex("0320"))["active"],
                         ["ir_sensor_fault"])
        packet = bytearray([codec.VERSION, 64]) + bytearray(12)
        # IR bit at snapshot body offset 101, then first padding bit 102.
        packet[13] = 0x20
        self.assertEqual(codec.decode("car/reply/snapshot", packet)["faults"]["mask"], 32)
        packet[13] |= 0x40
        with self.assertRaises(ValueError):
            codec.decode("car/reply/snapshot", packet)


if __name__ == "__main__":
    unittest.main()
