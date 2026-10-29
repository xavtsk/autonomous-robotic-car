"""Bounded, local storage of the firmware's observational BCAP captures.

Samples retain the firmware's flags: bits 0–1 are surface (0 unknown, 1 white,
2 black); bits 2–4 are decoder status (0 none, 1 decoded, 2 rejected, 3 timing
error, 4 overflow); bit 5 collecting, bit 6 rearming, bit 7 gap above 30 ms.
darkness is dark4 * 4, on the firmware's 0–1000 scale. t_ms is reconstructed
from the episode start and integer dt fields, so it is approximate. The first
sample is at the episode start: its dt describes a preceding, unstored sample.
A missing sample or saturated dt=255 makes subsequent timestamps unknown.

"complete" means the advertised capture arrived intact with its end marker.
It does NOT mean the entire run was captured: summary.not_stored separately
counts firmware capture opportunities that could not fit in its fixed pool.
"truncated" is a dashboard storage limit; "missing_records" is a broken or
incomplete transfer. These are deliberately separate from decoder overflow.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import threading
import uuid
from .barcode_protocol import decode_barcode

MAX_INPUT_BYTES = 2 * 1024 * 1024
MAX_TRACES = 40
MAX_SAMPLES = 30000
MAX_EPISODES = 256
_ID = re.compile(r"^[a-f0-9]{32}$")
_SUMMARY = re.compile(
    r"\[BCAP\] run summary: episodes=(\d+) not_stored=(\d+) decoded=(\d+) "
    r"letters=([A-Z]+|-) rejected=(\d+) timing_error=(\d+) overflow=(\d+)"
    r" \| raw:dark4:flags:mm:dt$")
_HEADER = re.compile(r"\[BCAP\] episode (\d+) t=(\d+)ms samples=(\d+)$")
_ROW = re.compile(r"BC(\d+) (\d+):\s*(.*)$")
_LOG_PREFIX = re.compile(r"^\d{2}:\d{2}:\d{2}(?:\.\d+)?\s+(?:DEBUG\s+)?")
_KEYS = ("episodes", "not_stored", "decoded", "letters", "rejected",
         "timing_error", "overflow")


def _line(text: str) -> str:
    return _LOG_PREFIX.sub("", text.strip())


def _bounded(text: str) -> tuple[str, bool]:
    data = text.encode("utf-8")
    if len(data) <= MAX_INPUT_BYTES:
        return text, False
    # Keep whole lines; never reinterpret a partial token as a smaller value.
    data = data[:MAX_INPUT_BYTES]
    last_newline = data.rfind(b"\n")
    return data[:last_newline + 1].decode("utf-8", errors="ignore"), True


def _safe_name(name: str) -> str:
    if not isinstance(name, str) or not name or any(c in name for c in "/\\\x00") or name in (".", ".."):
        raise ValueError("Use a plain trace filename, without a directory.")
    safe = re.sub(r"[^A-Za-z0-9._ -]", "_", name).strip(" .")[:128]
    if not safe:
        raise ValueError("Trace filename is empty.")
    return safe if safe.lower().endswith(".txt") else safe + ".txt"


def parse_trace(text: str, truncated: bool = False) -> dict:
    """Parse a single capture without assuming missing data or exact timing."""
    summary = dict.fromkeys(_KEYS)
    samples, headers, warnings, rows = [], {}, [], {}
    ended = False
    broken = False
    recognized = False

    def warn(message: str) -> None:
        if message not in warnings and len(warnings) < 16:
            warnings.append(message)

    for original in text.splitlines():
        line = _line(original)
        if not (line.startswith("[BCAP]") or re.match(r"BC\d+ ", line)):
            continue
        recognized = True
        if ended and line == "[BCAP] end":
            continue
        if ended:
            broken = True
            warn("Capture records occur after the end marker.")
            continue
        match = _SUMMARY.fullmatch(line)
        if match:
            if summary["episodes"] is not None:
                broken = True
                warn("More than one run summary is present.")
                continue
            values = match.groups()
            if any(len(v) > 10 or int(v) > 0xFFFFFFFF for i, v in enumerate(values) if i != 3) or len(values[3]) > 256:
                broken = True
                warn("Summary values are out of range.")
                continue
            summary = {key: value if key == "letters" else int(value)
                       for key, value in zip(_KEYS, values)}
            continue
        match = _HEADER.fullmatch(line)
        if match:
            if any(len(v) > 10 for v in match.groups()):
                broken = True
                warn("Episode header values are out of range.")
                continue
            episode, start, expected = map(int, match.groups())
            if episode in headers and headers[episode]["start_ms"] == start and headers[episode]["expected_samples"] == expected:
                continue
            if not 1 <= episode <= MAX_EPISODES or start > 0xFFFFFFFF or expected > MAX_SAMPLES or episode in headers:
                broken = True
                warn("Invalid or repeated episode header.")
                continue
            headers[episode] = {"episode": episode, "start_ms": start,
                                "expected_samples": expected, "received_samples": 0,
                                "next_index": 0, "clock": start}
            continue
        if line == "[BCAP] end":
            ended = True
            continue
        match = _ROW.fullmatch(line)
        if match:
            if len(match[1]) > 10 or len(match[2]) > 10:
                broken = True
                warn("Sample indices are out of range.")
                continue
            episode, index = int(match[1]), int(match[2])
            tokens = match[3].split()
            row_key = (episode, index)
            if row_key in rows:
                if rows[row_key] != tokens:
                    broken = True
                    warn("Conflicting retransmission at the same sample offset.")
                continue  # Identical MQTT redelivery is not capture loss.
            rows[row_key] = tokens
            header = headers.get(episode)
            if header is None:
                broken = True
                warn("Samples arrived without their episode header.")
                continue
            if index != header["next_index"]:
                broken = True
                header["clock"] = None
                warn("Missing, repeated or out-of-order sample records.")
                if index < header["next_index"]:
                    continue  # Do not duplicate samples from retransmitted rows.
            if not tokens:
                broken = True
                warn("Empty sample row.")
            for token in tokens:
                if len(samples) >= MAX_SAMPLES:
                    truncated = True
                    warn("Dashboard sample limit reached.")
                    break
                parts = token.split(":")
                if len(parts) != 5 or any(not p.isascii() or not p.isdigit() or len(p) > 5 for p in parts):
                    broken = True
                    header["clock"] = None
                    warn("Malformed sample data.")
                    index += 1
                    continue
                raw, dark4, flags, mm, dt = map(int, parts)
                if not (raw <= 4095 and dark4 <= 250 and flags <= 255 and mm <= 65535 and dt <= 255):
                    broken = True
                    header["clock"] = None
                    warn("Sample value outside the firmware format.")
                    index += 1
                    continue
                if index >= header["expected_samples"]:
                    broken = True
                    warn("More sample data than the episode advertised.")
                if index != 0 and header["clock"] is not None:
                    if dt == 255:
                        header["clock"] = None
                        warn("A saturated interval makes subsequent timestamps unknown.")
                    else:
                        header["clock"] += dt
                samples.append({"episode": episode, "t_ms": header["clock"],
                                "raw": raw, "darkness": dark4 * 4, "flags": flags,
                                "mm": mm, "dt_ms": dt})
                header["received_samples"] += 1
                index += 1
            header["next_index"] = index
            continue
        broken = True
        warn("Unrecognized or malformed capture record.")

    if summary["episodes"] is None:
        broken = True
        warn("Missing valid run summary.")
    elif summary["episodes"] != len(headers) or set(headers) != set(range(1, min(summary["episodes"], MAX_EPISODES) + 1)):
        broken = True
        warn("Missing or unexpected episode headers.")
    if any(h["received_samples"] != h["expected_samples"] for h in headers.values()):
        broken = True
        warn("An episode has missing or extra samples.")
    if not ended:
        warn("End marker has not arrived; capture may be incomplete.")
    if truncated:
        warn("Dashboard storage limit truncated this capture.")
    if summary["not_stored"]:
        warn("Firmware could not store every capture opportunity; not_stored is separate from transfer completeness.")
    return {"summary": summary, "samples": samples,
            "episodes": [{k: v for k, v in h.items() if k not in ("clock", "next_index")} for h in headers.values()],
            "sample_count": len(samples), "complete": ended and not broken and not truncated,
            "truncated": truncated, "missing_records": broken or not ended,
            "warnings": warnings, "timestamps_approximate": True, "recognized": recognized}


@dataclass
class _Record:
    id: str
    name: str
    created_at: str
    chunks: list[str] = field(default_factory=list)
    byte_count: int = 0
    truncated: bool = False
    parsed: dict | None = None


class TraceStore:
    """Thread-safe local capture archive. Stores at most 40 bounded captures.

    Only trace-<uuid>.json/txt files owned by this store are read. Older records
    leave the recent in-memory list but stay on disk as test evidence.
    Filenames supplied by the browser are display/download names, never paths.
    """

    def __init__(self, folder: Path):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._records: dict[str, _Record] = {}
        self._active: str | None = None
        self._binary: dict[int, str] = {}
        candidates = sorted(self.folder.glob("trace-*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
        for meta in candidates:
            if len(self._records) >= MAX_TRACES:
                break
            trace_id = meta.stem.removeprefix("trace-")
            raw_path = self._path(trace_id, "txt") if _ID.fullmatch(trace_id) else None
            if raw_path is None or meta.is_symlink() or raw_path.is_symlink():
                continue
            try:
                if meta.stat().st_size > 4096 or raw_path.stat().st_size > MAX_INPUT_BYTES:
                    continue
                info = json.loads(meta.read_text(encoding="utf-8"))
                if info.get("version") != 1 or info.get("id") != trace_id:
                    continue
                created = info["created_at"]
                datetime.fromisoformat(created)
                name = _safe_name(info["name"])
                raw = raw_path.read_text(encoding="utf-8")
                self._records[trace_id] = _Record(trace_id, name, created, [raw],
                                                  len(raw.encode("utf-8")), bool(info.get("truncated")))
            except (OSError, ValueError, TypeError, KeyError):
                continue
        self._prune()

    def _path(self, trace_id: str, extension: str) -> Path:
        if not isinstance(trace_id, str) or not _ID.fullmatch(trace_id):
            raise KeyError("Unknown trace")
        return self.folder / f"trace-{trace_id}.{extension}"

    def _save_info(self, record: _Record, *, new: bool = False) -> None:
        info = {"version": 1, "id": record.id, "name": record.name,
                "created_at": record.created_at, "truncated": record.truncated}
        path = self._path(record.id, "json")
        if path.is_symlink():
            raise OSError("Refusing a symlink in the trace archive")
        with path.open("x" if new else "w", encoding="utf-8") as stream:
            json.dump(info, stream)

    def _new(self, name: str) -> _Record:
        record = _Record(uuid.uuid4().hex, _safe_name(name), datetime.now(timezone.utc).isoformat())
        self._path(record.id, "txt").touch(exist_ok=False)
        self._save_info(record, new=True)
        self._records[record.id] = record
        self._prune()
        return record

    def _prune(self) -> None:
        ordered = sorted(self._records.values(), key=lambda r: (r.created_at, r.id), reverse=True)
        for record in ordered[MAX_TRACES:]:
            self._records.pop(record.id)
            if self._active == record.id:
                self._active = None

    def _append(self, record: _Record, text: str) -> None:
        size = len(text.encode("utf-8"))
        if record.truncated or record.byte_count + size > MAX_INPUT_BYTES:
            if not record.truncated:
                record.truncated = True
                record.parsed = None
                self._save_info(record)
            return
        path = self._path(record.id, "txt")
        if path.is_symlink():
            raise OSError("Refusing a symlink in the trace archive")
        with path.open("a", encoding="utf-8") as stream:
            stream.write(text)
        record.chunks.append(text)
        record.byte_count += size
        record.parsed = None

    def _parsed(self, record: _Record) -> dict:
        if record.parsed is None:
            record.parsed = parse_trace("".join(record.chunks), record.truncated)
        return record.parsed

    def _metadata(self, record: _Record) -> dict:
        parsed = self._parsed(record)
        return {"id": record.id, "name": record.name, "created_at": record.created_at,
                **{k: deepcopy(v) for k, v in parsed.items() if k not in ("samples", "episodes", "recognized")}}

    def ingest(self, text: str) -> str | None:
        """Consume one debug MQTT payload (possibly multiple complete lines)."""
        if not isinstance(text, str):
            raise ValueError("Trace payload must be text")
        bounded, cut = _bounded(text)
        updated = None
        with self._lock:
            for original in bounded.splitlines():
                line = _line(original)
                if line.startswith("[BCAP] run summary:"):
                    record = self._new("barcode_trace_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".txt")
                    self._active = record.id
                elif self._active is None:
                    continue
                elif not (line.startswith("[BCAP]") or re.match(r"BC\d+ ", line)):
                    continue
                record = self._records[self._active]
                self._append(record, line + "\n")
                updated = record.id
                if line == "[BCAP] end":
                    self._active = None
            if cut and updated is not None:
                record = self._records[updated]
                record.truncated = True
                record.parsed = None
                self._save_info(record)
            return updated

    def import_text(self, name: str, text: str) -> dict:
        safe_name = _safe_name(name)
        if not isinstance(text, str):
            raise ValueError("Trace import must be text")
        bounded, truncated = _bounded(text)
        parsed = parse_trace(bounded, truncated)
        if parsed["summary"]["episodes"] is None:
            raise ValueError("File has no valid BCAP run summary")
        with self._lock:
            record = self._new(safe_name)
            self._append(record, bounded)
            record.truncated = truncated
            record.parsed = parsed
            self._save_info(record)
            return self._metadata(record)

    def ingest_binary(self, payload: bytes) -> dict:
        """Preserve partial/interleaved transfers; never declare loss as zero.

        IDs are meaningful within the connected car session, not globally
        unique across clock wraps/reboots. Imports on disk use independent UUIDs.
        Duplicate summary/end records are harmless; sample retransmissions and
        conflicting rows are checked by the existing text parser.
        """
        frame = decode_barcode(payload)
        key, text = frame["capture_id"], frame["text"]
        with self._lock:
            self._binary = {k:v for k,v in self._binary.items() if v in self._records}
            ident = self._binary.get(key)
            if ident is None:
                record = self._new("barcode_trace_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".txt")
                self._binary[key] = record.id
            else:
                record = self._records[ident]
            # Duplicated metadata must not create a second run or an error.
            if text+"\n" in record.chunks:
                return frame
            self._append(record, text+"\n")
            return frame

    def list(self) -> list[dict]:
        with self._lock:
            return [self._metadata(r) for r in sorted(self._records.values(), key=lambda r: (r.created_at, r.id), reverse=True)]

    def get(self, trace_id: str) -> dict:
        with self._lock:
            self._path(trace_id, "txt")  # Validate before looking up supplied IDs.
            record = self._records[trace_id]
            return {**self._metadata(record), "samples": deepcopy(self._parsed(record)["samples"]),
                    "episodes": deepcopy(self._parsed(record)["episodes"])}

    def download(self, trace_id: str) -> tuple[str, str]:
        with self._lock:
            self._path(trace_id, "txt")
            record = self._records[trace_id]
            # Same-second captures and repeated imports cannot overwrite a download.
            name = record.name[:-4] + "_" + record.id[:8] + ".txt"
            return name, "".join(record.chunks)
