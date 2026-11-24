"""Archive discarded hump crossings by the boot and run that produced them."""
import csv
import re
import struct

from .imu_capture import HUMP_REASONS

TOPIC = "car/debug/hump_event"
PACKET = struct.Struct("<BBHIIIIHH")
FIELDS = ("received_at_utc", "firmware_boot_tag", "firmware_run", "event_sequence",
          "sensor_timestamp_us", "rejection_reason", "crossing_travel_um", "peak_um",
          "firmware_dropped_events")
NAME = re.compile(r"hump-([0-9a-f]{8})-r(\d{5})-([0-9a-f]{6})\.csv")
RECENT_FILES = 40


def decode_event(payload):
    if len(payload) != PACKET.size:
        raise ValueError("Invalid hump event length")
    version, reason, run, boot, sequence, source_us, travel, peak, dropped = PACKET.unpack(payload)
    if version != 1 or reason >= len(HUMP_REASONS):
        raise ValueError("Invalid hump event version or reason")
    return dict(firmware_boot_tag=f"{boot:08x}", firmware_run=run, event_sequence=sequence,
                sensor_timestamp_us=source_us, rejection_reason=HUMP_REASONS[reason],
                crossing_travel_um=travel, peak_um=peak, firmware_dropped_events=dropped)


class HumpEventLog:
    """One dashboard writer. Each CSV contains at most 1024 distinct events."""

    def __init__(self, folder):
        self.folder = folder / "imu"
        self.error = None
        self.latest = None
        self._active = None
        self._seen = set()
        paths = [p for p in self.folder.glob("hump-*.csv")
                 if NAME.fullmatch(p.name) and not p.is_symlink() and p.is_file()]
        self.files = [p.name for p in sorted(paths, key=lambda p: (p.stat().st_mtime_ns, p.name),
                                             reverse=True)[:RECENT_FILES]]
        if self.files:
            try:
                self._open_segment(self.files[0])
            except (OSError, UnicodeError, ValueError, csv.Error):
                self.error = "Could not read the saved hump events."

    def _open_segment(self, name):
        path = self.folder / name
        if path.is_symlink():
            raise ValueError("Invalid hump event file")
        seen = set()
        last = None
        if path.exists():
            with path.open(encoding="ascii", newline="") as stream:
                rows = csv.DictReader(stream)
                if rows.fieldnames != list(FIELDS):
                    raise ValueError("Invalid hump event header")
                for row in rows:
                    seen.add(int(row["event_sequence"]))
                    last = row
        self._active, self._seen = name, seen
        if last and self.latest is None:
            self.latest = {key: (value if key in ("received_at_utc", "firmware_boot_tag",
                                                  "rejection_reason") else int(value))
                           for key, value in last.items()}

    def record(self, data, received_at):
        seq = data["event_sequence"]
        name = f"hump-{data['firmware_boot_tag']}-r{data['firmware_run']:05d}-{seq >> 10:06x}.csv"
        row = dict(received_at_utc=received_at, **data)
        try:
            if self._active != name:
                self._open_segment(name)
            if seq in self._seen:
                return False
            self.folder.mkdir(exist_ok=True)
            path = self.folder / name
            if path.is_symlink():
                raise ValueError("Invalid hump event file")
            with path.open("a", encoding="ascii", newline="") as stream:
                writer = csv.DictWriter(stream, FIELDS, lineterminator="\n")
                if stream.tell() == 0:
                    writer.writeheader()
                writer.writerow(row)
        except (OSError, UnicodeError, ValueError, csv.Error):
            self.error = "Could not save the hump event to disk."
            self._active = None
            return False
        self._seen.add(seq)
        self.files = ([name] + [prior for prior in self.files if prior != name])[:RECENT_FILES]
        self.latest, self.error = row, None
        return True

    def snapshot(self):
        return dict(latest=self.latest, files=list(self.files), error=self.error)

    def download(self, name):
        path = self.folder / name
        if not NAME.fullmatch(name) or path.is_symlink() or not path.is_file():
            raise KeyError(name)
        # 1024 rows of validated fixed-width integers, timestamps and reasons.
        with path.open("rb") as stream:
            data = stream.read(256 * 1024 + 1)
        if len(data) > 256 * 1024:
            raise ValueError("Invalid hump event file size")
        return data
