"""Spreadsheet-ready views of decoded telemetry saved in a run archive."""
import csv
import io


BUDDY_NAMES = {
    1: "Radio and telemetry",
    2: "Motor and encoder",
    3: "Line and barcode",
    4: "Tilt and humps",
    5: "Obstacle avoidance",
}

_COMMON = ("run_id", "buddy", "recorded_mode", "run_started_at", "run_ended_at", "run_active",
           "run_end_reason", "run_entries_dropped", "received_at", "record_type", "note")
_IR_CHANNELS = ("left", "right", "barcode")
_FIELDS = {
    1: {
        "heartbeat": ("alive", "faults_valid", ("fault_mask", "faults.mask"), ("active_faults", "faults.active")),
        "faults": (("fault_mask", "mask"), ("active_faults", "active")),
        "control_status": ("robot_time_us", "epoch", "last_id", "available", "running", "can_start", "can_reset"),
        "control_reply": (("request_id", "id"), "epoch", ("command", "action"), "result", "reason"),
        "barcode_map_reply": ("ok", "result"),
        "snapshot_reply": ("valid_sections",),
    },
    2: {
        "motion": (("left_speed_mm_s", "left_mms"), ("right_speed_mm_s", "right_mms")),
        "pose": (("wrapped_distance_cm", "distance_cm_mod"), "distance_cm", "heading_deg"),
        "heading": ("heading_deg",),
        "motor_feedback": (
            ("robot_time_us", "timestamp_us"), ("motor_state", "state"), ("motor_fault", "fault"), "braked",
            ("left_setpoint_mm_s", "setpoint_left"), ("right_setpoint_mm_s", "setpoint_right"),
            ("left_speed_mm_s", "measured_left"), ("right_speed_mm_s", "measured_right"),
            ("left_duty_percent", "duty_left"), ("right_duty_percent", "duty_right"),
            ("left_encoder_count", "encoder_left"), ("right_encoder_count", "encoder_right"),
        ),
    },
    3: {
        "ir_preview": (
            ("robot_time_us", "timestamp_us"), "sequence",
            *((f"{name}_{label}", f"sensors.{name}.{key}")
              for name in _IR_CHANNELS
              for label, key in (("darkness_0_1000", "darkness"), ("raw_adc", "raw"),
                                 ("darkness_valid", "valid"), ("raw_valid", "raw_valid"), ("surface", "surface"))),
        ),
        "state": (("navigation_state", "state"),),
        "barcode": (("barcode_letter", "letter"), ("barcode_action", "action"), "decoder_accepted",
                    "mapped_action_known"),
        "navigation": (
            ("robot_time_us", "timestamp_us"), ("controller_state", "state"), ("controller_reason", "reason"),
            ("pending_action", "pending"), "line_error", "line_flags", "line_observation_valid",
            ("last_decoder_outcome", "barcode_status"), ("barcode_events_since_boot", "barcode_events"),
            ("junction_events_since_boot", "junction_events"), ("line_lost_events_since_boot", "lost_events"),
            "sample_gap_us", "run_max_gap_us",
        ),
    },
    4: {
        "terrain": ("tilt_valid", "pitch_deg", "roll_deg", ("terrain_state", "state"), "on_hump",
                    ("live_hump_height_mm", "climb_mm"), ("peak_height_valid", "peak_valid"),
                    ("reported_peak_hump_mm", "peak_mm")),
        "hump": (("hump_count", "count"), ("reported_peak_hump_mm", "height_mm")),
    },
    5: {
        "avoidance": ("range_mm", "range_valid", "no_echo", "servo_deg", ("bypass_state", "state"),
                      ("bypass_fault", "fault"), "route", "leg", "pass_step", "swapped"),
        "scan": ("scan_id", ("scan_valid", "valid"), "closest_mm", "width_mm",
                 "left_clear_mm", "right_clear_mm", "forward_mm", "left_edge_mm", "right_edge_mm"),
        "obstacle": ("distance_mm", "width_mm", "route"),
    },
}


def _column(field):
    return (field, field) if isinstance(field, str) else field


def _cell(data, path):
    for key in path.split("."):
        if not isinstance(data, dict):
            return None
        data = data.get(key)
    if isinstance(data, (tuple, list)):
        data = " | ".join(str(value) for value in data)
    if isinstance(data, str) and data.lstrip().startswith(("=", "+", "-", "@")):
        return "'" + data  # CSV quoting alone does not prevent spreadsheet formulas.
    return data


def _record_data(buddy, kind, data):
    if buddy == 3 and kind == "barcode":
        return dict(data, decoder_accepted=True, mapped_action_known=data.get("action") not in (None, "unknown"))
    if buddy == 3 and kind == "navigation" and "line_flags" in data:
        valid = bool(data["line_flags"] & 1)
        return dict(data, line_observation_valid=valid, line_error=data.get("line_error") if valid else None)
    if buddy == 4 and kind == "hump" and data.get("count") == 0:
        return dict(data, height_mm=None)
    if buddy == 5 and kind == "avoidance" and "range_valid" not in data:
        return dict(data, range_valid=data.get("range_mm") is not None)
    return data


def encode_run_csv(archive, buddy):
    """Return one buddy's timestamped evidence, including metadata-only runs."""
    if type(buddy) is not int or buddy not in _FIELDS:
        raise ValueError("Buddy must be from 1 to 5")
    fields_by_kind = _FIELDS[buddy]
    buddy_columns = dict.fromkeys(_column(field)[0] for fields in fields_by_kind.values() for field in fields)
    run = archive["run"]
    common = dict(run_id=_cell(run, "id"), buddy=buddy, recorded_mode=_cell(archive, "mode"),
                  run_started_at=_cell(run, "started_at"), run_ended_at=_cell(run, "ended_at"), run_active=run.get("active"),
                  run_end_reason=_cell(run, "end_reason"), run_entries_dropped=archive.get("dropped_entries"))
    if buddy == 4:
        common.update(run_hump_count=run.get("hump_count"), run_highest_hump_mm=run.get("peak_mm"))
        buddy_columns = dict(run_hump_count=None, run_highest_hump_mm=None, **buddy_columns)
    output = io.StringIO(newline="")
    writer = csv.DictWriter(output, fieldnames=(*_COMMON, *buddy_columns))
    writer.writeheader()
    have_rows = False
    for entry in archive["entries"]:
        kind = entry["kind"]
        if entry["buddy"] != buddy or kind not in fields_by_kind:
            continue
        data = _record_data(buddy, kind, entry["data"])
        row = {column: _cell(data, source) for column, source in map(_column, fields_by_kind[kind])}
        writer.writerow(dict(common, received_at=_cell(entry, "received_at"), record_type=kind, **row))
        have_rows = True
    if not have_rows:
        writer.writerow(dict(common, record_type="run_summary", note="No telemetry was recorded for this buddy."))
    return output.getvalue().encode("utf-8-sig")
