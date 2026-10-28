#!/usr/bin/env python3
"""Laptop decoder and setup-command encoder for Buddy 1 binary protocol v2.

Standard library only. Feed the original MQTT payload bytes to decode(); JSON
is produced on the laptop for display, never sent as robot application data.
"""
import argparse
import json
from pathlib import Path

VERSION = 2
NAMES = ("motion", "barcode", "hump", "heading", "obstacle", "state", "faults")
WIDTHS = (18, 8, 32, 9, 34, 3, 6)
STATES = ("stopped", "line_following", "turning_left", "turning_right", "uturn",
          "avoiding_obstacle", "complete", "error")
ACTIONS = ("unknown", "forward", "left", "right", "uturn")
FAULTS = ("motor_stall", "encoder_fault", "imu_fault", "no_echo", "wifi_loss", "ir_sensor_fault")
POSE_DISTANCE_BITS = 12
POSE_DISTANCE_MOD = 1 << POSE_DISTANCE_BITS


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
        return dict(count=value & 65535, height_mm=value >> 16)
    if name == "heading":
        if value > 359:
            raise ValueError("invalid heading")
        return dict(heading_deg=value)
    if name == "obstacle":
        route = value >> 32
        if route > 2:
            raise ValueError("reserved route")
        return dict(distance_mm=value & 65535, width_mm=(value >> 16) & 65535,
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
        value = _check(payload, 117)
        flags, offset, result = value & 127, 7, {}
        for i, (section, width) in enumerate(zip(NAMES, WIDTHS)):
            raw = (value >> offset) & ((1 << width) - 1)
            valid = bool(flags & (1 << i))
            if not valid and raw:
                raise ValueError("nonzero data in invalid snapshot section")
            result[section] = dict(valid=valid, **(_section(section, raw) if valid else {}))
            offset += width
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
