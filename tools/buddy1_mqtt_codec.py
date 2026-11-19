#!/usr/bin/env python3
"""Laptop decoder and setup-command encoder for Buddy 1 binary protocol v3.

Standard library only. Feed the original MQTT payload bytes to decode(); JSON
is produced on the laptop for display, never sent as robot application data.
"""
import argparse
import json
from pathlib import Path

VERSION = 3
NAMES = ("motion", "barcode", "hump", "heading", "obstacle", "state", "faults")
WIDTHS = (18, 8, 25, 9, 26, 3, 6)
POSE_BITS = 12 + 9        # distance since START (cm mod 4096) + heading since START
# 19 Nov 2026: pose (valid bit + 21 bits) appended after the original 102 bits.
SNAPSHOT_LEGACY_BITS = 7 + sum(WIDTHS)
SNAPSHOT_BITS = SNAPSHOT_LEGACY_BITS + 1 + POSE_BITS
SNAPSHOT_BYTES = 1 + (SNAPSHOT_BITS + 7) // 8
SNAPSHOT_LEGACY_BYTES = 1 + (SNAPSHOT_LEGACY_BITS + 7) // 8
STATES = ("stopped", "line_following", "turning_left", "turning_right", "uturn",
          "avoiding_obstacle", "complete", "error")
ACTIONS = ("unknown", "forward", "left", "right", "uturn")
FAULTS = ("motor_stall", "encoder_fault", "imu_fault", "no_echo", "wifi_loss", "ir_sensor_fault")
POSE_DISTANCE_BITS = 12
POSE_DISTANCE_MOD = 1 << POSE_DISTANCE_BITS
assert POSE_BITS == POSE_DISTANCE_BITS + 9
TILT_BITS = 11            # signed 0.1 degree, -90.0..+90.0 (Buddy 4 widths)
TILT_LIMIT_X10 = 900
TURN_BITS = 10            # signed whole deg/s, -360..+360 (Buddy 4 width)
TURN_LIMIT_DPS = 360
# 59: tilt, state, climb, run peak, compass turn rate, lifted/overturned (19 Nov 2026)
TERRAIN_BITS = 1 + 2 * TILT_BITS + 3 + 10 + 10 + 1 + TURN_BITS + 2
TERRAIN_LEGACY_BITS = 46  # firmware before 19 Nov 2026: no turn rate / safety
SAFETY = ("none", "lifted", "overturned")
# Buddy 5 (car/telemetry/avoidance and car/telemetry/scan, live only).
S5_STATES = ("idle", "waiting_for_stop", "scanning", "moving", "finishing", "complete", "fault")
S5_FAULTS = ("none", "bad_range", "no_path", "scan_failed", "deadline", "no_line",
             "motion_error", "interrupted")
S5_PASSES = ("approach", "peek", "peeking", "step", "extra")
S5_ROUTES = ("unknown", "left", "right")
AVOIDANCE_BITS = 38
SCAN_BITS = 95
SCAN_SAMPLE_BITS = 35     # one reading: id, peek, index, count-1, angle, valid, mm
TERRAIN_STATES = ("unknown", "stationary", "moving", "accelerating", "turning",
                  "climbing", "descending", "impact")


def unwrap_distance(previous_cm, wrapped_cm):
    """Continue a run's distance from a pose packet's wrapped 12-bit value.

    previous_cm is the last unwrapped distance (0 at START). The step is taken
    as the nearest value in -2048..+2047 cm, so reversing gives a negative step.
    """
    step = (wrapped_cm - previous_cm) % POSE_DISTANCE_MOD
    if step >= POSE_DISTANCE_MOD // 2:
        step -= POSE_DISTANCE_MOD
    return previous_cm + step


def _check(payload, bits):
    if len(payload) != 1 + (bits + 7) // 8:
        raise ValueError("wrong payload length")
    if payload[0] != VERSION:
        raise ValueError("unsupported protocol version")
    value = int.from_bytes(payload[1:], "little")
    if value >> bits:
        raise ValueError("nonzero padding bits")
    return value


def _section(name, value):
    if name == "motion":
        speeds = []
        for wheel in range(2):
            raw = (value >> (9 * wheel)) & 0x1ff
            mag, reverse = raw & 255, bool(raw & 256)
            if not mag and reverse:
                raise ValueError("negative zero speed")
            speeds.append(mag * (-2 if reverse else 2))
        return dict(left_mms=speeds[0], right_mms=speeds[1])
    if name == "barcode":
        letter, action = value & 31, value >> 5
        if letter > 25 or action >= len(ACTIONS):
            raise ValueError("reserved barcode/action")
        return dict(letter=chr(65 + letter), action=ACTIONS[action])
    if name == "hump":
        if value >> 16 > 500:
            raise ValueError("hump height outside 0..500 mm")
        return dict(count=value & 65535, height_mm=value >> 16)
    if name == "heading":
        if value > 359:
            raise ValueError("invalid heading")
        return dict(heading_deg=value)
    if name == "obstacle":
        route = value >> 24
        distance = value & 4095
        if route > 2 or not 30 <= distance <= 4000:
            raise ValueError("reserved route")
        return dict(distance_mm=distance, width_mm=(value >> 12) & 4095,
                    route=("unknown", "left", "right")[route])
    if name == "state":
        return dict(state=STATES[value])
    if name == "faults":
        return dict(mask=value, active=[name for i, name in enumerate(FAULTS) if value & (1 << i)])
    raise ValueError("unknown telemetry section")


def encode_map(forward, left, right, uturn):
    letters = (forward, left, right, uturn)
    if len(set(letters)) != 4 or any(len(c) != 1 or not "A" <= c <= "Z" for c in letters):
        raise ValueError("mapping requires four different uppercase A-Z letters")
    bits = sum((ord(c) - 65) << (5 * i) for i, c in enumerate(letters))
    return bytes([VERSION]) + bits.to_bytes(3, "little")


def decode(topic, payload):
    """Decode original MQTT payload bytes, rejecting malformed packets."""
    payload = bytes(payload)
    name = topic.rsplit("/", 1)[-1]
    if topic == "car/cmd/snapshot":
        if payload != bytes([VERSION]):
            raise ValueError(f"snapshot request must contain only version {VERSION}")
        return dict(command="snapshot")
    if topic == "car/cmd/barcode_map":
        value = _check(payload, 20)
        letters = [chr(65 + ((value >> (5 * i)) & 31)) for i in range(4)]
        if encode_map(*letters) != payload:
            raise ValueError("noncanonical mapping")
        return dict(zip(("forward", "left", "right", "uturn"), letters))
    if topic == "car/reply/barcode_map":
        value = _check(payload, 2)
        return dict(ok=value == 0, result=("ok", "invalid", "busy", "internal")[value])
    if topic == "car/reply/snapshot":
        legacy = len(payload) == SNAPSHOT_LEGACY_BYTES   # firmware before 19 Nov 2026: no pose
        value = _check(payload, SNAPSHOT_LEGACY_BITS if legacy else SNAPSHOT_BITS)
        flags, offset, result = value & 127, 7, {}
        for i, (section, width) in enumerate(zip(NAMES, WIDTHS)):
            raw = (value >> offset) & ((1 << width) - 1)
            valid = bool(flags & (1 << i))
            if not valid and raw:
                raise ValueError("nonzero data in invalid snapshot section")
            result[section] = dict(valid=valid, **(_section(section, raw) if valid else {}))
            offset += width
        if not legacy:
            valid = bool(value >> offset & 1)
            raw = (value >> (offset + 1)) & ((1 << POSE_BITS) - 1)
            heading = raw >> POSE_DISTANCE_BITS
            if not valid and raw:
                raise ValueError("nonzero data in invalid snapshot section")
            if heading > 359:
                raise ValueError("invalid heading")
            result["pose"] = dict(valid=valid, **(dict(distance_cm_mod=raw & (POSE_DISTANCE_MOD - 1),
                                                       heading_deg=heading) if valid else {}))
        return result
    if topic == "car/status" or name == "heartbeat":
        value = _check(payload, 8)
        if not value & 1 or (not value & 2 and value >> 2):
            raise ValueError("invalid heartbeat status")
        return dict(alive=True, faults_valid=bool(value & 2),
                    faults=_section("faults", value >> 2) if value & 2 else None)
    if topic == "car/telemetry/pose":
        value = _check(payload, POSE_DISTANCE_BITS + 9)
        heading = value >> POSE_DISTANCE_BITS
        if heading > 359:
            raise ValueError("invalid heading")
        return dict(distance_cm_mod=value & (POSE_DISTANCE_MOD - 1), heading_deg=heading)
    if topic == "car/telemetry/avoidance":
        value = _check(payload, AVOIDANCE_BITS)
        fields, shift = {}, 0
        for name, width in (("range_valid", 1), ("no_echo", 1), ("range_mm", 12), ("servo_deg", 8),
                            ("state", 3), ("fault", 3), ("route", 2), ("leg", 4), ("pass", 3),
                            ("swapped", 1)):
            fields[name] = (value >> shift) & ((1 << width) - 1)
            shift += width
        if fields["range_mm"] > 4000 or (fields["range_valid"] and (
                fields["range_mm"] < 30 or fields["no_echo"])) or (
                not fields["range_valid"] and fields["range_mm"]):
            raise ValueError("invalid front range")
        if fields["servo_deg"] > 180 or fields["state"] >= len(S5_STATES) or \
                fields["route"] >= len(S5_ROUTES) or fields["pass"] >= len(S5_PASSES):
            raise ValueError("Buddy 5 field out of range")
        return dict(range_mm=fields["range_mm"] if fields["range_valid"] else None,
                    no_echo=bool(fields["no_echo"]),
                    servo_deg=fields["servo_deg"] or None,
                    state=S5_STATES[fields["state"]], fault=S5_FAULTS[fields["fault"]],
                    route=S5_ROUTES[fields["route"]], leg=fields["leg"],
                    pass_step=S5_PASSES[fields["pass"]], swapped=bool(fields["swapped"]))
    if topic == "car/telemetry/scan":
        value = _check(payload, SCAN_BITS)
        fields, shift = {}, 0
        for name, width in (("valid", 1), ("scan_id", 8), ("closest_mm", 12), ("width_mm", 12),
                            ("left_clear_mm", 12), ("right_clear_mm", 12), ("forward_mm", 12),
                            ("left_edge_mm", 13), ("right_edge_mm", 13)):
            fields[name] = (value >> shift) & ((1 << width) - 1)
            shift += width
        fields["left_edge_mm"] -= 4000
        fields["right_edge_mm"] -= 4000
        values = ("closest_mm", "width_mm", "left_clear_mm", "right_clear_mm", "forward_mm",
                  "left_edge_mm", "right_edge_mm")
        if any(fields[k] > 4000 for k in values if k != "width_mm"):
            raise ValueError("scan field out of range")
        if fields["valid"]:
            if fields["closest_mm"] < 30 or fields["left_edge_mm"] < fields["right_edge_mm"]:
                raise ValueError("inconsistent obstacle profile")
            out = {k: fields[k] for k in values}
        else:
            if any(fields[k] for k in values):
                raise ValueError("invalid profile with nonzero values")
            out = {k: None for k in values}
        return dict(valid=bool(fields["valid"]), scan_id=fields["scan_id"], **out)
    if topic == "car/telemetry/scan_sample":
        value = _check(payload, SCAN_SAMPLE_BITS)
        fields, shift = {}, 0
        for name, width in (("scan_id", 8), ("peek", 1), ("index", 4), ("count", 4), ("angle", 5),
                            ("valid", 1), ("distance_mm", 12)):
            fields[name] = (value >> shift) & ((1 << width) - 1)
            shift += width
        count, angle = fields["count"] + 1, 30 + 5 * fields["angle"]
        if fields["index"] >= count or angle > 150:
            raise ValueError("scan sample index/angle out of range")
        if fields["valid"] and not 30 <= fields["distance_mm"] <= 4000 or \
                not fields["valid"] and fields["distance_mm"]:
            raise ValueError("invalid scan sample distance")
        return dict(scan_id=fields["scan_id"], peek=bool(fields["peek"]), index=fields["index"],
                    count=count, angle_deg=angle, valid=bool(fields["valid"]),
                    distance_mm=fields["distance_mm"] if fields["valid"] else None)
    if topic == "car/telemetry/terrain":
        legacy = len(payload) == 1 + (TERRAIN_LEGACY_BITS + 7) // 8
        value = _check(payload, TERRAIN_LEGACY_BITS if legacy else TERRAIN_BITS)
        valid = bool(value & 1)
        angles = []
        for shift in (1, 1 + TILT_BITS):
            raw = (value >> shift) & ((1 << TILT_BITS) - 1)
            angles.append(raw - (1 << TILT_BITS) if raw >> (TILT_BITS - 1) else raw)
        state = (value >> 23) & 7
        on_hump, climb = bool(value >> 26 & 1), (value >> 27) & 511
        peak_valid, peak = bool(value >> 36 & 1), (value >> 37) & 511
        if not valid and any(angles):
            raise ValueError("unavailable tilt with nonzero angles")
        if any(abs(a) > TILT_LIMIT_X10 for a in angles):
            raise ValueError("tilt outside -90.0..+90.0 degrees")
        if climb > 500 or peak > 500 or (not on_hump and climb) or (not peak_valid and peak):
            raise ValueError("invalid hump height field")
        turn_valid, turn, safety = False, 0, None
        if not legacy:
            turn_valid = bool(value >> 46 & 1)
            turn = (value >> 47) & ((1 << TURN_BITS) - 1)
            turn = turn - (1 << TURN_BITS) if turn >> (TURN_BITS - 1) else turn
            safety = (value >> 57) & 3
            if safety >= len(SAFETY):
                raise ValueError("reserved safety code")
            if abs(turn) > TURN_LIMIT_DPS or (not turn_valid and turn):
                raise ValueError("invalid turn rate")
            safety = SAFETY[safety]
        return dict(tilt_valid=valid,
                    pitch_deg=angles[0] / 10 if valid else None,
                    roll_deg=angles[1] / 10 if valid else None,
                    state=TERRAIN_STATES[state],
                    on_hump=on_hump, climb_mm=climb if on_hump else None,
                    peak_valid=peak_valid, peak_mm=peak if peak_valid else None,
                    turn_valid=turn_valid, turn_dps=turn if turn_valid else None, safety=safety)
    if topic.startswith("car/telemetry/") and name in NAMES:
        return _section(name, _check(payload, WIDTHS[NAMES.index(name)]))
    raise ValueError("unknown topic; use car/status when decoding a custom heartbeat topic")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="operation", required=True)
    read = sub.add_parser("decode")
    read.add_argument("--topic", required=True)
    source = read.add_mutually_exclusive_group(required=True)
    source.add_argument("--hex")
    source.add_argument("--file", type=Path, help="file containing raw MQTT payload bytes")
    mapping = sub.add_parser("encode-map")
    for name in ("forward", "left", "right", "uturn"):
        mapping.add_argument("--" + name, required=True)
    sub.add_parser("encode-snapshot")
    args = parser.parse_args()
    try:
        if args.operation == "decode":
            payload = bytes.fromhex(args.hex) if args.hex is not None else args.file.read_bytes()
            print(json.dumps(decode(args.topic, payload), indent=2))
        elif args.operation == "encode-map":
            print(encode_map(args.forward, args.left, args.right, args.uturn).hex())
        else:
            print(bytes([VERSION]).hex())
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
