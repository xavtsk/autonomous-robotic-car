"""Decode and archive Buddy 4 accelerometer debug packets."""
import csv
from datetime import datetime, timezone
import io
import re
import struct
from uuid import uuid4


TOPIC = "car/debug/imu"
HEADER = struct.Struct("<BBHII")
SAMPLE_FORMATS = {
    1: (struct.Struct("<IhhhhhhBB"), 8),
    2: (struct.Struct("<IhhhhhhBBiiBB"), 5),
}
RAW_VALID, EVALUATED, OVERRUN, IMPACT, PENDING, UPWARD, TILT = (1 << bit for bit in range(7))
SAFETY = ("none", "lifted", "overturned")
HUMP_EVENTS = ("armed", "started", "discarded", "completed", "acceleration_skipped")
HUMP_ACTIVE = 1 << 5
HUMP_ARMED = 1 << 6
HUMP_REASONS = (
    "none", "invalid_imu", "imu_gap", "invalid_tilt", "odometry_unavailable", "odometry_baseline",
    "odometry_step_out_of_range", "safety_or_impact", "turn_rate", "acceleration_spike", "reverse_travel",
    "invalid_geometry", "crossing_timeout", "left_wheel_raised", "waiting_for_level", "below_start_height",
    "no_forward_travel", "insufficient_crossing_travel", "calibration_pending",
)
DISTANCE_UNAVAILABLE = -(1 << 31)
COUNTER_MASK = 0xffffffff
MAX_CAPTURE_BYTES = 2 * 1024 * 1024
MAX_RECENT_CAPTURES = 40
FILENAME = re.compile(r"imu-(\d{8}T\d{9}Z)-([0-9a-f]{8})-r(\d{5})-([0-9a-f]{8})\.csv")
FIELDS = (
    "received_at_utc", "firmware_boot_tag", "firmware_run", "sample_sequence", "sensor_timestamp_us",
    "observed_elapsed_us", "missing_samples_before", "raw_valid", "detector_evaluated",
    "sensor_overrun", "raw_x_mg", "raw_y_mg", "raw_z_mg", "calibrated_x_mg", "calibrated_y_mg",
    "calibrated_z_mg", "impact", "confirmation_pending", "upward_gate", "tilted_lift_gate", "safety",
)
HUMP_FIELDS = (
    "wheel_delta_um", "crossing_travel_um", "hump_events", "hump_active", "hump_armed", "hump_rejection_reason",
)


def decode_imu(payload):
    if len(payload) < HEADER.size:
        raise ValueError("Truncated IMU capture header")
    version, count, run, sequence, boot_tag = HEADER.unpack_from(payload)
    if version not in SAMPLE_FORMATS:
        raise ValueError("Invalid IMU capture version")
    row, max_samples = SAMPLE_FORMATS[version]
    if not 1 <= count <= max_samples or len(payload) != HEADER.size + count * row.size:
        raise ValueError("Invalid IMU capture count or length")
    samples = tuple(row.iter_unpack(payload[HEADER.size:]))
    if any(sample[7] & 0x80 or sample[8] >= len(SAFETY) for sample in samples):
        raise ValueError("Invalid IMU capture flags or safety state")
    if version == 2:
        for *_, travel, flags, reason in samples:
            if flags & 0x80 or reason >= len(HUMP_REASONS) or (travel < 0 and travel != DISTANCE_UNAVAILABLE):
                raise ValueError("Invalid IMU hump diagnostic")
    return boot_tag, run, sequence, samples


def hump_columns(hump):
    if not hump:
        return (None,) * len(HUMP_FIELDS)
    delta, travel, flags, reason = hump
    return (
        None if delta == DISTANCE_UNAVAILABLE else delta,
        None if travel == DISTANCE_UNAVAILABLE else travel,
        "|".join(label for bit, label in enumerate(HUMP_EVENTS) if flags & (1 << bit)),
        int(bool(flags & HUMP_ACTIVE)), int(bool(flags & HUMP_ARMED)),
        HUMP_REASONS[reason],
    )


class ImuCaptures:
    """Bounded CSV captures, written by the dashboard state owner under its lock."""

    def __init__(self, data_dir):
        self.folder = data_dir / "imu"
        self.error = None
        self._key = self._path = self._sequence = self._time = None
        self._elapsed = self._bytes = 0
        self._limited = False
        self._recent = []
        for path in sorted(self.folder.glob("imu-*.csv"), reverse=True):
            if len(self._recent) == MAX_RECENT_CAPTURES:
                break
            info = self._metadata(path.name)
            if info and not path.is_symlink() and path.is_file():
                self._recent.append(dict(info, limited=path.with_suffix(".limited").is_file()))

    @staticmethod
    def _metadata(name):
        match = FILENAME.fullmatch(name)
        if not match:
            return None
        first, boot_tag, run, _ = match.groups()
        try:
            received_at = datetime.strptime(first, "%Y%m%dT%H%M%S%fZ").replace(tzinfo=timezone.utc)
        except ValueError:
            return None
        return dict(name=name, received_at=received_at.isoformat(), boot_tag=boot_tag, run=int(run))

    def _path_for(self, received_at, boot_tag, run):
        stamp = datetime.fromisoformat(received_at).astimezone(timezone.utc)
        key = stamp.strftime("%Y%m%dT%H%M%S%f")[:-3] + "Z"
        return self.folder / f"imu-{key}-{boot_tag:08x}-r{run:05d}-{uuid4().hex[:8]}.csv"

    def record(self, batch, received_at):
        boot_tag, run, first_sequence, samples = batch
        key = (boot_tag, run)
        if key != self._key:
            self._path = self._sequence = self._time = None
            self._elapsed = self._bytes = 0
            self._limited = False
            self._key = key
        if self._limited:
            return

        sequence, timestamp, elapsed = self._sequence, self._time, self._elapsed
        text = io.StringIO(newline="")
        writer = csv.writer(text, lineterminator="\n")
        for offset, sample in enumerate(samples):
            current = (first_sequence + offset) & COUNTER_MASK
            source_time, x, y, z, cx, cy, cz, flags, safety, *hump = sample
            missing = None
            if sequence is not None:
                step = (current - sequence) & COUNTER_MASK
                if step == 0 or step >= 0x80000000:
                    continue
                elapsed += (source_time - timestamp) & COUNTER_MASK
                missing = step - 1
            raw_valid, evaluated = bool(flags & RAW_VALID), bool(flags & EVALUATED)
            writer.writerow((
                received_at, f"{boot_tag:08x}", run, current, source_time, elapsed, missing,
                int(raw_valid), int(evaluated), int(bool(flags & OVERRUN)),
                *((x, y, z) if raw_valid else (None,) * 3),
                *((cx, cy, cz) if evaluated else (None,) * 3),
                int(bool(flags & IMPACT)), int(bool(flags & PENDING)), int(bool(flags & UPWARD)),
                int(bool(flags & TILT)), SAFETY[safety], *hump_columns(hump),
            ))
            sequence, timestamp = current, source_time
        data = text.getvalue().encode("ascii")
        if not data:
            return

        if self._bytes + len(data) > MAX_CAPTURE_BYTES:
            self._limited = True
            self._recent[0] = dict(self._recent[0], limited=True)
            try:
                self._path.with_suffix(".limited").touch(exist_ok=False)
            except OSError:
                self.error = "Could not save the raw IMU capture limit status."
            return

        start_file = self._path is None
        path = self._path_for(received_at, boot_tag, run) if start_file else self._path
        if start_file:
            data = (",".join(FIELDS + HUMP_FIELDS) + "\n").encode("ascii") + data
        try:
            if start_file:
                self.folder.mkdir(exist_ok=True)
            if path.is_symlink():
                raise OSError("Invalid capture file")
            with path.open("xb" if start_file else "ab") as stream:
                stream.write(data)
        except OSError:
            self.error = "Could not save raw IMU samples to disk."
            # Start another segment on recovery, keeping the last saved sample sequence.
            self._path, self._bytes = None, 0
            return

        if start_file:
            self._path, self._bytes = path, 0
            self._recent.insert(0, dict(self._metadata(path.name), limited=False))
            del self._recent[MAX_RECENT_CAPTURES:]
        self._bytes += len(data)
        self._sequence, self._time, self._elapsed = sequence, timestamp, elapsed
        self.error = None

    def snapshot(self):
        return dict(captures=list(self._recent), error=self.error)

    def download(self, name):
        if not FILENAME.fullmatch(name):
            raise KeyError("Unknown IMU capture")
        path = self.folder / name
        if path.is_symlink() or not path.is_file():
            raise KeyError("Unknown IMU capture")
        with path.open("rb") as stream:
            data = stream.read(MAX_CAPTURE_BYTES + 1)
        if len(data) > MAX_CAPTURE_BYTES:
            raise ValueError("Invalid IMU capture size")
        return data
