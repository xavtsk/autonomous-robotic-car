#!/usr/bin/env python3
"""Change Requirement 1: every packed field uses the minimum bits for its range.

For each field: valid range (lo..hi) and step -> number of distinct codes ->
minimum bits = ceil(log2(codes)). The script fails if a field's width is not
the minimum, unless the row states why (a wrapping counter/clock whose width
is set by how far it may move between reads, not by a range).

It also checks that the fields of each MQTT topic add up to the widths the
laptop decoder (tools/buddy1_mqtt_codec.py) uses, so this table cannot drift
from the code. The C side is checked by testing/subsystem1 (exhaustive
pack/unpack over the full ranges and rejection of out-of-range values).

Run from the repository root:  python3 testing/change_requirement_1/check_widths.py
"""
from math import ceil, log2
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import buddy1_mqtt_codec as codec  # noqa: E402

FLAG = (0, 1, 1)
WRAP = "wrapping counter/clock: width set by how far it moves between reads"

# (group, field, (lo, hi, step), bits, reason-if-not-minimum)
FIELDS = [
    # Buddy 1 MQTT payloads (protocol v3). Byte 0 of each payload is the version.
    ("motion", "left wheel speed magnitude, mm/s", (0, 510, 2), 8, ""),
    ("motion", "left wheel reverse", FLAG, 1, ""),
    ("motion", "right wheel speed magnitude, mm/s", (0, 510, 2), 8, ""),
    ("motion", "right wheel reverse", FLAG, 1, ""),
    ("barcode", "letter A-Z", (0, 25, 1), 5, ""),
    ("barcode", "action unknown/fwd/left/right/uturn", (0, 4, 1), 3, ""),
    ("hump", "hump count", (0, 65535, 1), 16, ""),
    ("hump", "highest hump, mm", (0, 500, 1), 9, ""),
    ("heading", "heading, degrees", (0, 359, 1), 9, ""),
    ("obstacle", "distance, mm", (30, 4000, 1), 12, ""),
    ("obstacle", "width, mm", (0, 4095, 1), 12, ""),
    ("obstacle", "route unknown/left/right", (0, 2, 1), 2, ""),
    ("state", "robot state", (0, 7, 1), 3, ""),
    ("faults", "fault bitmask: stall, encoder, IMU, no-echo, Wi-Fi, IR", (0, 63, 1), 6, ""),
    ("pose", "distance since START, cm (mod 4096)", (0, 4095, 1), 12, WRAP),
    ("pose", "heading since START, degrees", (0, 359, 1), 9, ""),
    ("terrain", "tilt valid", FLAG, 1, ""),
    ("terrain", "pitch, 0.1 deg", (-900, 900, 1), 11, ""),
    ("terrain", "roll, 0.1 deg", (-900, 900, 1), 11, ""),
    ("terrain", "terrain state", (0, 7, 1), 3, ""),
    ("terrain", "on hump", FLAG, 1, ""),
    ("terrain", "climb so far, mm", (0, 500, 1), 9, ""),
    ("terrain", "run peak valid", FLAG, 1, ""),
    ("terrain", "run peak, mm", (0, 500, 1), 9, ""),
    ("avoidance", "front range valid", FLAG, 1, ""),
    ("avoidance", "no echo", FLAG, 1, ""),
    ("avoidance", "front range, mm", (30, 4000, 1), 12, ""),
    ("avoidance", "servo angle, deg (0 = not positioned)", (0, 180, 1), 8, ""),
    ("avoidance", "Buddy 5 state", (0, 6, 1), 3, ""),
    ("avoidance", "Buddy 5 fault", (0, 7, 1), 3, ""),
    ("avoidance", "route", (0, 2, 1), 2, ""),
    ("avoidance", "bypass leg", (0, 10, 1), 4, ""),
    ("avoidance", "pass step", (0, 4, 1), 3, ""),
    ("avoidance", "side swapped", FLAG, 1, ""),
    ("scan", "profile valid", FLAG, 1, ""),
    ("scan", "scan id (low 8 bits)", (0, 255, 1), 8, WRAP),
    ("scan", "closest, mm", (30, 4000, 1), 12, ""),
    ("scan", "width, mm", (0, 4095, 1), 12, ""),
    ("scan", "left clearance, mm", (0, 4000, 1), 12, ""),
    ("scan", "right clearance, mm", (0, 4000, 1), 12, ""),
    ("scan", "depth to far face, mm", (0, 4000, 1), 12, ""),
    ("scan", "left edge, mm (+4000 offset)", (-4000, 4000, 1), 13, ""),
    ("scan", "right edge, mm (+4000 offset)", (-4000, 4000, 1), 13, ""),
    ("status", "alive", FLAG, 1, ""),
    ("status", "fault field known", FLAG, 1, ""),
    ("status", "fault bitmask", (0, 63, 1), 6, ""),
    ("map_reply", "result ok/invalid/closed/internal", (0, 3, 1), 2, ""),
    ("envelope", "outbox topic selector (14 topics)", (0, 13, 1), 4, ""),
    ("envelope", "incoming command kind", (0, 2, 1), 2, ""),
    # Buddy 4 internal 12-byte packet (shared snapshot and 8-slot event queue).
    ("b4 packet", "timestamp, us", (0, 2**32 - 1, 1), 32, WRAP),
    ("b4 packet", "terrain state", (0, 7, 1), 3, ""),
    ("b4 packet", "flags: tilt/turn/peak valid, calibrated, fault", (0, 31, 1), 5, ""),
    ("b4 packet", "pitch, 0.1 deg", (-900, 900, 1), 11, ""),
    ("b4 packet", "roll, 0.1 deg", (-900, 900, 1), 11, ""),
    ("b4 packet", "turn rate, deg/s", (-360, 360, 1), 10, ""),
    ("b4 packet", "highest hump, mm", (0, 500, 1), 9, ""),
    ("b4 packet", "hump completed", FLAG, 1, ""),
    ("b4 packet", "impact", FLAG, 1, ""),
    ("b4 packet", "hump in progress", FLAG, 1, ""),
    ("b4 packet", "climb so far, mm", (0, 500, 1), 9, ""),
    # Buddy 5 internal 23-byte task snapshot.
    ("b5 packet", "forward reading time, us", (0, 2**32 - 1, 1), 32, WRAP),
    ("b5 packet", "scan time, us", (0, 2**32 - 1, 1), 32, WRAP),
    ("b5 packet", "scan id (0 = none, wraps)", (0, 65535, 1), 16, WRAP),
    ("b5 packet", "forward distance, mm", (30, 4000, 1), 12, ""),
    ("b5 packet", "forward valid", FLAG, 1, ""),
    ("b5 packet", "no echo", FLAG, 1, ""),
    ("b5 packet", "scan done", FLAG, 1, ""),
    ("b5 packet", "profile valid", FLAG, 1, ""),
    ("b5 packet", "closest, mm", (30, 4000, 1), 12, ""),
    ("b5 packet", "width, mm", (0, 4095, 1), 12, ""),
    ("b5 packet", "left clearance, mm", (0, 4000, 1), 12, ""),
    ("b5 packet", "right clearance, mm", (0, 4000, 1), 12, ""),
    ("b5 packet", "depth, mm", (0, 4000, 1), 12, ""),
    ("b5 packet", "left edge, mm", (-4000, 4000, 1), 13, ""),
    ("b5 packet", "right edge, mm", (-4000, 4000, 1), 13, ""),
    ("b5 sample", "distance, mm", (0, 4000, 1), 12, ""),
    ("b5 sample", "angle index (30..150 deg, 5 deg steps)", (0, 24, 1), 5, ""),
    ("b5 sample", "valid", FLAG, 1, ""),
    # Controller -> telemetry Buddy 5 status word (16 bits, one writer).
    ("s5 status", "state", (0, 6, 1), 3, ""),
    ("s5 status", "fault", (0, 7, 1), 3, ""),
    ("s5 status", "route", (0, 2, 1), 2, ""),
    ("s5 status", "leg", (0, 10, 1), 4, ""),
    ("s5 status", "pass step", (0, 4, 1), 3, ""),
    ("s5 status", "side swapped", FLAG, 1, ""),
    # Buddy 5 private avoidance state (single owner, C bit-fields).
    ("s5 private", "aim turn, deg", (-40, 40, 1), 7, ""),
    ("s5 private", "bend at detection, % (controller -> Buddy 5)", (-100, 100, 1), 8, ""),
    # 17 Nov 2026 bypass state (single owner, C bit-fields).
    ("s5 private", "heading vs passing heading, deg", (-90, 90, 1), 8, ""),
    ("s5 private", "pending side turn away, deg", (0, 10, 1), 4, ""),
    ("s5 private", "side turns made", (0, 3, 1), 2, ""),
    ("s5 private", "hand-back centring tries", (0, 2, 1), 2, ""),
    ("s5 private", "consecutive blocked readings", (0, 2, 1), 2, ""),
    ("s5 private", "sideways travel after turning away, mm", (0, 511, 1), 9, ""),
    ("s5 private", "distance from the line at the last side peek, mm", (0, 4095, 1), 12, ""),
]

# Total bits per topic used by the laptop decoder and C codec.
TOTALS = {
    "motion": codec.WIDTHS[0], "barcode": codec.WIDTHS[1], "hump": codec.WIDTHS[2],
    "heading": codec.WIDTHS[3], "obstacle": codec.WIDTHS[4], "state": codec.WIDTHS[5],
    "faults": codec.WIDTHS[6], "pose": codec.POSE_DISTANCE_BITS + 9,
    "terrain": codec.TERRAIN_BITS, "avoidance": codec.AVOIDANCE_BITS, "scan": codec.SCAN_BITS,
    "b4 packet": 93, "b5 packet": 182, "b5 sample": 18, "s5 status": 16,
}


def min_bits(lo, hi, step):
    codes = (hi - lo) // step + 1
    return max(1, ceil(log2(codes))), codes


def main():
    bad = 0
    print("| Group | Field | Range (step) | Codes | Min bits | Used bits | OK |")
    print("|---|---|---|---:|---:|---:|---|")
    sums = {}
    for group, name, (lo, hi, step), bits, why in FIELDS:
        need, codes = min_bits(lo, hi, step)
        ok = bits == need or (why and bits >= need)
        mark = "yes" if bits == need else ("yes (" + why + ")" if ok else "**NO**")
        bad += not ok
        sums[group] = sums.get(group, 0) + bits
        rng = f"{lo}..{hi}" + (f" ({step})" if step != 1 else "")
        print(f"| {group} | {name} | {rng} | {codes} | {need} | {bits} | {mark} |")
    print()
    print("| Group | Sum of fields | Bits in codec | OK |")
    print("|---|---:|---:|---|")
    for group, total in TOTALS.items():
        ok = sums.get(group) == total
        bad += not ok
        print(f"| {group} | {sums.get(group)} | {total} | {'yes' if ok else '**NO**'} |")
    snap = 7 + sum(codec.WIDTHS)
    ok = snap == codec.SNAPSHOT_BITS
    bad += not ok
    print(f"| snapshot (7 validity + 7 sections) | {snap} | {codec.SNAPSHOT_BITS} | {'yes' if ok else '**NO**'} |")
    print()
    print("RESULT:", "PASS" if not bad else f"FAIL ({bad} problems)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
