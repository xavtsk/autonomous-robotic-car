#!/usr/bin/env python3
"""Check the declared Change Requirement 1 field inventory and source bindings.

For each field: valid range (lo..hi) and step -> number of distinct codes ->
minimum bits = ceil(log2(codes)). The script fails if a field's width is not
the minimum, unless the row states why (a wrapping counter/clock whose width
is set by how far it may move between reads, not by a range).

This is an explicit inventory, not a parser proving that every application
field is included. Topic totals are compared with the laptop codec. Selected
internal layouts and newly packed state are also checked against C source;
the remaining C layouts need their codec regression tests and manual review.

Run from the repository root:  python3 testing/change_requirement_1/check_widths.py
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import buddy1_mqtt_codec as codec  # noqa: E402
from dashboard import run_control_protocol as remote  # noqa: E402

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
    ("terrain", "turn rate valid (19 Nov 2026)", FLAG, 1, ""),
    ("terrain", "compass turn rate, deg/s", (-360, 360, 1), 10, ""),
    ("terrain", "safety none/lifted/overturned", (0, 2, 1), 2, ""),
    ("avoidance", "front range valid", FLAG, 1, ""),
    ("avoidance", "no echo", FLAG, 1, ""),
    ("avoidance", "front range, mm", (30, 4000, 1), 12, ""),
    ("avoidance", "servo angle, deg (0 = not positioned)", (0, 180, 1), 8, ""),
    ("avoidance", "Buddy 5 state", (0, 6, 1), 3, ""),
    ("avoidance", "Buddy 5 fault", (0, 7, 1), 3, ""),
    ("avoidance", "route", (0, 2, 1), 2, ""),
    ("avoidance", "bypass leg (9 = reversing, 10 = aiming)", (0, 10, 1), 4, ""),
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
    ("scan_sample", "scan id (low 8 bits, 19 Nov 2026)", (0, 255, 1), 8, WRAP),
    ("scan_sample", "side peek", FLAG, 1, ""),
    ("scan_sample", "reading index", (0, 15, 1), 4, ""),
    ("scan_sample", "readings in this scan", (1, 16, 1), 4, ""),
    ("scan_sample", "angle index (30..150 deg, 5 deg steps)", (0, 24, 1), 5, ""),
    ("scan_sample", "reading valid", FLAG, 1, ""),
    ("scan_sample", "distance, mm (0 = no reading)", (0, 4000, 1), 12, ""),
    ("status", "alive", FLAG, 1, ""),
    ("status", "fault field known", FLAG, 1, ""),
    ("status", "fault bitmask", (0, 63, 1), 6, ""),
    ("map_reply", "result ok/invalid/closed/internal", (0, 3, 1), 2, ""),
    ("control_reply", "action start/stop/reset", (1, 3, 1), remote.REPLY_ACTION_BITS, ""),
    ("control_reply", "queued/executed/rejected/cancelled", (0, 3, 1), remote.REPLY_RESULT_BITS, ""),
    ("control_reply", "reason", (0, 7, 1), remote.REPLY_REASON_BITS, ""),
    ("control_reply", "request id", (0, 2**32 - 1, 1), 32, WRAP),
    ("control_reply", "link epoch", (0, 2**32 - 1, 1), 32, WRAP),
    ("envelope", "outbox topic selector (15 topics)", (0, 14, 1), 4, ""),
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
    ("b4 packet", "safety none/lifted/overturned", (0, 2, 1), 2, ""),
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
    ("s5 status", "leg (9 = reversing, 10 = aiming)", (0, 10, 1), 4, ""),
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

# These retained state fields were added to the audit after the 19 Nov review.
# They are checked against the named C bit-field declarations below, in addition
# to checking their range mathematics. Struct padding is measured separately.
STATE_FIELDS = {
    "b4 queue": [("head", (0, 7, 1), 3), ("count", (0, 8, 1), 4)] +
                [(name, FLAG, 1) for name in
                 ("b_has_published", "b_has_lost_events", "b_is_reset_pending",
                  "b_should_recalibrate", "b_has_pending_mag_cal")],
    "remote request": [("request_id", (0, 2**32 - 1, 1), 32),
                       ("epoch", (0, 2**32 - 1, 1), 32),
                       ("time_us", (0, 2**32 - 1, 1), 32),
                       ("action", (0, 3, 1), 2), ("b_is_request_valid", FLAG, 1)],
    "remote state": [("flags", (0, 15, 1), 4), ("head", (0, 5, 1), 3),
                     ("count", (0, 6, 1), 3)] +
                    [(name, FLAG, 1) for name in
                     (
                         "b_is_connected", "b_has_published_status", "b_is_stop_held", "b_has_active_request"
                     )],
    "barcode work flags": [(name, FLAG, 1) for name in
                           ("b_is_faint", "b_is_rising", "b_has_crossed", "b_is_strong", "b_does_bias_vary",
                            "b_has_candidate", "b_allows_noise", "b_is_ambiguous", "b_is_provisional",
                            "b_is_strict_fit_broken", "b_is_early")],
    "barcode record flags": [(name, FLAG, 1) for name in
                             ("b_accepts_faint_quiet", "b_blocks_retry", "b_did_try_early_fit",
                              "b_has_early_candidate")],
    "drive state": [("reset_result", (0, 3, 1), 2)] +
                   [(name, FLAG, 1) for name in
                    ("b_has_external_hold", "b_is_configured", "b_waits_for_stop",
                     "b_is_advancing", "b_has_touched_line", "b_is_searching",
                     "b_has_obstacle_snapshot", "b_is_bypassing")],
    "terrain report cache": [("sent_height", (0, 500, 1), 9),
                             ("b_sent_fault", FLAG, 1),
                             ("b_has_unreported_impact", FLAG, 1)],
}
for group, fields in STATE_FIELDS.items():
    for name, limits, bits in fields:
        FIELDS.append((group, name, limits, bits, WRAP if bits == 32 else ""))

STATE_SOURCES = {
    "b4 queue": ("app_program/subsystem4/buddy4.c", "subsystem4_queue_state_t"),
    "remote request": ("app_program/controller/remote_control.h", "remote_control_request_t"),
    "remote state": ("app_program/controller/s3_integration.c", "remote_mailbox_t"),
    "barcode work flags": ("app_program/subsystem3/bc_decoder.h", "bc_decoder_work_t"),
    "barcode record flags": ("app_program/subsystem3/bc_decoder.h", "bc_decoder_record_t"),
    "drive state": ("app_program/controller/s3_drive.h", "s3_drive_t"),
    "terrain report cache": ("app_program/controller/terr_link.c", "terrain_control_t"),
}

# Total bits per topic used by the laptop decoder and C codec.
TOTALS = {
    "motion": codec.WIDTHS[0], "barcode": codec.WIDTHS[1], "hump": codec.WIDTHS[2],
    "heading": codec.WIDTHS[3], "obstacle": codec.WIDTHS[4], "state": codec.WIDTHS[5],
    "faults": codec.WIDTHS[6], "pose": codec.POSE_DISTANCE_BITS + 9,
    "terrain": codec.TERRAIN_BITS, "avoidance": codec.AVOIDANCE_BITS, "scan": codec.SCAN_BITS,
    "scan_sample": codec.SCAN_SAMPLE_BITS,
    "control_reply": remote.REPLY_BITS + 32 + 32,
    "b4 packet": 95, "b5 packet": 182, "b5 sample": 18, "s5 status": 16,
}


def min_bits(lo, hi, step):
    if step <= 0 or hi < lo:
        raise ValueError("Invalid range or resolution")
    codes = (hi - lo) // step + 1
    return max(1, (codes - 1).bit_length()), codes


def without_comments(text):
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.DOTALL)


def integer_macros(text):
    return {name: int(number) for name, number in re.findall(
        r"^\s*#define\s+(\w+)\s+\(?\s*(\d+)[uUlL]*\s*\)?\s*$",
        without_comments(text), re.MULTILINE)}


def function_body(text, name):
    text = without_comments(text)
    match = re.search(rf"\b{re.escape(name)}\s*\([^;{{}}]*\)\s*\{{", text)
    if not match:
        raise ValueError(f"Missing function {name}")
    start = match.end()
    depth = 1
    for index in range(start, len(text)):
        depth += (text[index] == "{") - (text[index] == "}")
        if depth == 0:
            return text[start:index]
    raise ValueError(f"Unclosed function {name}")


def source_errors(root=ROOT):
    """Fail if a bound declaration/call changes; never infer whole-source coverage."""
    errors = []
    header = (root / "app_program/controller/remote_control.h").read_text(encoding="utf-8")
    macros = integer_macros(header)
    for suffix in ("ACTION_BITS", "RESULT_BITS", "REASON_BITS", "BITS", "SIZE", "VERSION"):
        if macros.get("REMOTE_REPLY_" + suffix) != getattr(remote, "REPLY_" + suffix):
            errors.append(f"Remote C/Python reply {suffix} differs or is missing")
    if remote.REPLY_BITS != sum((remote.REPLY_ACTION_BITS, remote.REPLY_RESULT_BITS,
                                remote.REPLY_REASON_BITS)):
        errors.append("Remote reply meaningful widths do not sum to REPLY_BITS")
    if remote.REPLY_SIZE != 1 + (remote.REPLY_BITS + 7) // 8 + 4 + 4:
        errors.append("Remote reply byte count does not match packed widths and IDs")

    body = function_body((root / "app_program/subsystem4/terr_data.c").read_text(
        encoding="utf-8"), "terr_data_pack")
    widths = [int(width) for width in re.findall(
        r"\bput_bits\s*\([^;]*?,\s*(\d+)[uUlL]*\s*\)\s*;", body)]
    expected = [field[3] for field in FIELDS if field[0] == "b4 packet"]
    if widths != expected or len(widths) != len(re.findall(r"\bput_bits\s*\(", body)):
        errors.append(f"Buddy 4 terr_data_pack widths {widths} differ from inventory {expected}")

    for group, (name, struct_name) in STATE_SOURCES.items():
        text = without_comments((root / name).read_text(encoding="utf-8"))
        if group == "remote state" and not re.search(
                rf"\bstatic\s+{struct_name}\s+g_remote\s*;", text):
            errors.append(f"g_remote is not declared as {struct_name}")
        match = re.search(rf"\bstruct\s*\{{([^{{}}]*)\}}\s*{struct_name}\s*;", text, re.DOTALL)
        if not match:
            errors.append(f"Missing bound state declaration {struct_name}")
            continue
        for field, _, bits in STATE_FIELDS[group]:
            if bits == 32:
                present = re.search(rf"\buint32_t\s+{field}\s*;", match[1]) is not None
            else:
                declaration = re.search(rf"\b{field}\s*:\s*(\w+)\s*;", match[1])
                width = declaration[1] if declaration else ""
                actual = int(width) if width.isdigit() else macros.get(width)
                present = actual == bits
            if not present:
                errors.append(f"{struct_name}.{field} does not declare the expected {bits} bits")
    return errors


def main():
    bad = 0
    print(f"Scope: {len(FIELDS)} declared fields; selected C-source bindings and codec totals.")
    print("This is not an exhaustive inventory of every application field.\n")
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
    snap = 7 + sum(codec.WIDTHS) + 1 + codec.POSE_DISTANCE_BITS + 9   # + pose valid + pose (19 Nov 2026)
    ok = snap == codec.SNAPSHOT_BITS
    bad += not ok
    print(f"| snapshot (7 validity + 7 sections + pose validity + pose) | {snap} | {codec.SNAPSHOT_BITS} | {'yes' if ok else '**NO**'} |")
    try:
        errors = source_errors()
    except (OSError, ValueError) as error:
        errors = [str(error)]
    for error in errors:
        print("SOURCE MISMATCH:", error)
    bad += len(errors)
    print("Selected C-source bindings:", "PASS" if not errors else "FAIL")
    print()
    print("RESULT:", "PASS" if not bad else f"FAIL ({bad} problems)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
