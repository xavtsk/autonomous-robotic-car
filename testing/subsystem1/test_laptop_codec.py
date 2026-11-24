"""Independent laptop decoder checks against packets emitted by the real C codec."""
import importlib.util
import os
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
        binary = Path(os.environ.get("BUDDY1_HOST_TEST", str(ROOT / "build/buddy1/host_test")))
        output = subprocess.check_output([str(binary.resolve())], text=True)
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
        # Pose (distance/heading since START) is also in the snapshot since 19 Nov 2026.
        self.assertEqual(codec.decode("car/telemetry/pose", self.vectors["car/telemetry/pose"]),
                         dict(distance_cm_mod=2049, heading_deg=90))
        self.assertEqual(snapshot["pose"], dict(valid=True, distance_cm_mod=2049, heading_deg=90))
        self.assertEqual(len(self.vectors["car/reply/snapshot"]), 17)

    def test_snapshot_pose_section_and_legacy_length(self):
        def packet(pose_valid, cm, deg, pad=0):
            body = pose_valid << 102 | cm << 103 | deg << 115 | pad << 124
            return bytes([codec.VERSION]) + body.to_bytes(16, "little")
        for cm, deg in ((0, 0), (4095, 359), (1234, 270)):
            self.assertEqual(codec.decode("car/reply/snapshot", packet(1, cm, deg))["pose"],
                             dict(valid=True, distance_cm_mod=cm, heading_deg=deg))
        self.assertEqual(codec.decode("car/reply/snapshot", packet(0, 0, 0))["pose"], dict(valid=False))
        for bad in (packet(0, 5, 0), packet(1, 0, 360), packet(1, 0, 0, pad=1),
                    packet(1, 0, 0)[:-1] + b"\0\0"):
            with self.subTest(bad=bad.hex()), self.assertRaises(ValueError):
                codec.decode("car/reply/snapshot", bad)
        # A 14-byte reply from older firmware still decodes, without pose.
        self.assertNotIn("pose", codec.decode("car/reply/snapshot", bytes([codec.VERSION]) + bytes(13)))

    def test_buddy5_wire(self):
        self.assertEqual(codec.decode("car/telemetry/avoidance", self.vectors["car/telemetry/avoidance"]),
                         dict(range_mm=412, no_echo=False, servo_deg=150, state="moving", fault="none",
                              route="right", leg=3, pass_step="step", swapped=True))
        self.assertEqual(codec.decode("car/telemetry/avoidance", bytes.fromhex("030200000000")),
                         dict(range_mm=None, no_echo=True, servo_deg=None, state="idle", fault="none",
                              route="unknown", leg=0, pass_step="approach", swapped=False))
        self.assertEqual(codec.decode("car/telemetry/scan", self.vectors["car/telemetry/scan"]),
                         dict(valid=True, scan_id=7, closest_mm=250, width_mm=400, left_clear_mm=180,
                              right_clear_mm=320, forward_mm=380, left_edge_mm=150, right_edge_mm=-250))
        failed = bytes([3]) + ((9 << 1) | (4000 << 69) | (4000 << 82)).to_bytes(12, "little")
        self.assertEqual(codec.decode("car/telemetry/scan", failed)["valid"], False)
        self.assertIsNone(codec.decode("car/telemetry/scan", failed)["closest_mm"])
        for topic, bad in (("car/telemetry/avoidance", "030400000000"),   # mm without a valid range
                           ("car/telemetry/avoidance", "034306000000"),   # valid 400 mm + no echo
                           ("car/telemetry/avoidance", "030000000040"),   # padding
                           ("car/telemetry/avoidance", "0300000000"),     # length
                           ("car/telemetry/scan", "03" + "00" * 12),      # invalid but edges -4000
                           ("car/telemetry/scan", "03" + "00" * 11)):     # length
            with self.subTest(topic=topic, bad=bad), self.assertRaises(ValueError):
                codec.decode(topic, bytes.fromhex(bad))

    def test_scan_face_width_interval(self):
        topic = "car/telemetry/scan"
        legacy = self.vectors[topic]
        header = bytes([4]) + legacy[1:]
        def packet(low, high):
            return header + (low | high << 12).to_bytes(3, "little")
        for low, high in ((123, 177), (0, 1), (4095, 4095)):
            with self.subTest(low=low, high=high):
                self.assertEqual(codec.decode(topic, packet(low, high)),
                                 dict(codec.decode(topic, legacy), face_width_min_mm=low, face_width_max_mm=high))
        invalid_profile = bytes([4]) + ((4000 << 69) | (4000 << 82)).to_bytes(12, "little") + packet(1, 2)[13:]
        for bad in (header, packet(0, 0), packet(200, 150), packet(1, 2)[:-1], invalid_profile,
                    header[:12] + bytes([header[12] | 0x80]) + packet(1, 2)[13:],
                    legacy + packet(1, 2)[13:]):
            with self.subTest(bad=bad.hex()), self.assertRaises(ValueError):
                codec.decode(topic, bad)

    def test_terrain_wire(self):
        full = dict(tilt_valid=True, pitch_deg=12.3, roll_deg=-5.1, state="climbing",
                    on_hump=True, climb_mm=23, peak_valid=True, peak_mm=41,
                    turn_valid=True, turn_dps=-87, safety="none")
        self.assertEqual(codec.decode("car/telemetry/terrain", self.vectors["car/telemetry/terrain"]), full)
        self.assertNotIn("terrain", codec.decode("car/reply/snapshot", self.vectors["car/reply/snapshot"]))
        # Legacy 7-byte packet (firmware before 19 Nov 2026): no turn rate/safety.
        self.assertEqual(codec.decode("car/telemetry/terrain", bytes.fromhex("03000000010000")),
                         dict(tilt_valid=False, pitch_deg=None, roll_deg=None, state="moving",
                              on_hump=False, climb_mm=None, peak_valid=False, peak_mm=None,
                              turn_valid=False, turn_dps=None, safety=None))

        def pack9(turn, safety):
            body = (2 << 23 | (turn is not None) << 46 | ((turn or 0) & 0x3ff) << 47 | safety << 57)
            return bytes([3]) + body.to_bytes(8, "little")
        for turn, safety in ((None, 0), (360, 1), (-360, 2), (0, 0), (-1, 1)):
            self.assertEqual(codec.decode("car/telemetry/terrain", pack9(turn, safety)),
                             dict(tilt_valid=False, pitch_deg=None, roll_deg=None, state="moving",
                                  on_hump=False, climb_mm=None, peak_valid=False, peak_mm=None,
                                  turn_valid=turn is not None, turn_dps=turn,
                                  safety=codec.SAFETY[safety]))
        for bad in (pack9(361, 0), pack9(-361, 0), pack9(None, 3),
                    bytes([3]) + (1 << 47).to_bytes(8, "little"),    # turn without valid
                    bytes([3]) + (1 << 59).to_bytes(8, "little"),    # padding
                    pack9(0, 0)[:-1] + b"\0\0"):                    # length 10
            with self.subTest(bad=bad.hex()), self.assertRaises(ValueError):
                codec.decode("car/telemetry/terrain", bad)

        def pack(pitch, roll, state, climb, peak):
            body = (1 | (pitch & 0x7ff) << 1 | (roll & 0x7ff) << 12 | state << 23 |
                    (climb is not None) << 26 | (climb or 0) << 27 |
                    (peak is not None) << 36 | (peak or 0) << 37)
            return bytes([3]) + body.to_bytes(6, "little")
        for pitch, roll, climb, peak in ((-900, 900, 0, 500), (900, -900, 500, 0), (0, 0, None, None), (-1, 1, 1, 499)):
            self.assertEqual(codec.decode("car/telemetry/terrain", pack(pitch, roll, 7, climb, peak)),
                             dict(tilt_valid=True, pitch_deg=pitch / 10, roll_deg=roll / 10, state="impact",
                                  on_hump=climb is not None, climb_mm=climb,
                                  peak_valid=peak is not None, peak_mm=peak,
                                  turn_valid=False, turn_dps=None, safety=None))
        for bad in ("03f10700000000",   # pitch +101.6
                    "03020000000000",   # unavailable but nonzero angle
                    "03000000080000",   # climb height without a hump
                    "0300000000a000",   # peak height without a valid peak
                    "03000000fcff0f",   # climb 511 mm
                    "03000000000040",   # padding bit
                    "030000000000",     # length
                    "02000000000000"):  # version
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                codec.decode("car/telemetry/terrain", bytes.fromhex(bad))

    def test_scan_sample_wire(self):
        self.assertEqual(codec.decode("car/telemetry/scan_sample", self.vectors["car/telemetry/scan_sample"]),
                         dict(scan_id=7, peek=False, index=3, count=13, angle_deg=75, valid=True, distance_mm=412))

        def pack(sid, peek, index, count, angle, mm):
            body = (sid | peek << 8 | index << 9 | (count - 1) << 13 | (angle - 30) // 5 << 17 |
                    (mm is not None) << 22 | (mm or 0) << 23)
            return bytes([3]) + body.to_bytes(5, "little")
        for angle in range(30, 151, 5):
            for index, count in ((0, 1), (15, 16), (4, 13)):
                for mm in (30, 4000, None):
                    self.assertEqual(codec.decode("car/telemetry/scan_sample", pack(255, 1, index, count, angle, mm)),
                                     dict(scan_id=255, peek=True, index=index, count=count, angle_deg=angle,
                                          valid=mm is not None, distance_mm=mm))
        for bad in (pack(1, 0, 1, 1, 90, 200),                        # index outside count
                    pack(1, 0, 0, 1, 90, 29), pack(1, 0, 0, 1, 90, 4001),
                    bytes([3]) + (25 << 17).to_bytes(5, "little"),     # angle 155
                    bytes([3]) + (5 << 23).to_bytes(5, "little"),      # distance without valid
                    bytes([3]) + (1 << 35).to_bytes(5, "little"),      # padding
                    pack(1, 0, 0, 1, 90, 200)[:-1]):                   # length
            with self.subTest(bad=bad.hex()), self.assertRaises(ValueError):
                codec.decode("car/telemetry/scan_sample", bad)

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
        self.assertEqual((codec.SNAPSHOT_BYTES, codec.SNAPSHOT_LEGACY_BYTES), (17, 14))
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
