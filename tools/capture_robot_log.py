#!/usr/bin/env python3
"""Capture USB console bytes on the laptop; never send commands to the robot.

Replay needs only Python's standard library. Live capture/listing needs pyserial.
Host arrival timestamps are not MCU task timings. Opening a serial port can
toggle control lines on some systems even though DTR/RTS are requested low.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

READ_BYTES = 4096
READ_TIMEOUT_SECONDS = 0.25


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


class ArrivalCSV:
    """Bound line storage even if a faulty device never sends a newline."""

    def __init__(self, output):
        self.writer = csv.writer(output)
        self.writer.writerow(("host_received_utc", "host_elapsed_s", "record_kind", "text"))
        self.pending = bytearray()
        self.fragmented = False
        self.last_utc = ""
        self.last_elapsed = 0.0

    def _write(self, data, kind):
        if kind in ("line", "fragment_end"):
            data = data[:-1]  # The terminating LF is represented by record_kind.
            if data.endswith(b"\r"):
                data = data[:-1]
        self.writer.writerow((self.last_utc, f"{self.last_elapsed:.6f}", kind,
                              data.decode("utf-8", errors="replace")))

    def feed(self, data, received_utc, elapsed):
        self.last_utc, self.last_elapsed = received_utc, elapsed
        self.pending.extend(data)
        while self.pending:
            newline = self.pending.find(b"\n", 0, READ_BYTES)
            if newline >= 0:
                self._write(bytes(self.pending[:newline + 1]),
                            "fragment_end" if self.fragmented else "line")
                del self.pending[:newline + 1]
                self.fragmented = False
            elif len(self.pending) >= READ_BYTES:
                self._write(bytes(self.pending[:READ_BYTES]), "fragment")
                del self.pending[:READ_BYTES]
                self.fragmented = True
            else:
                break

    def finish(self):
        if self.pending:
            self._write(bytes(self.pending),
                        "fragment_partial" if self.fragmented else "partial")
            self.pending.clear()


def atomic_metadata(output, metadata):
    """Replace JSON only after a complete temporary file has been written."""
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=output,
                                         prefix=".metadata-", suffix=".tmp", delete=False) as file:
            temporary = Path(file.name)
            json.dump(metadata, file, indent=2)
            file.write("\n")
            file.flush()
            os.fsync(file.fileno())
        temporary.replace(Path(output) / "metadata.json")
    finally:
        if temporary is not None:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass  # Never mask the original write/replace failure.


class Recording:
    """Reserve output files before opening hardware, then own their cleanup."""

    def __init__(self, output, *, source, source_name, firmware="unspecified",
                 test="unspecified", notes="", duration=None, line_csv=False,
                 baud=None, clock=time.monotonic, utc=utc_now):
        if source not in ("hardware", "replay"):
            raise ValueError("source must be hardware or replay")
        if duration is not None and (not math.isfinite(duration) or duration <= 0):
            raise ValueError("duration must be finite and positive")
        self.output, self.clock, self.utc = Path(output), clock, utc
        self.raw = self.arrivals_file = self.arrivals = None
        self.finished = False
        # mkdir is the reservation, not an earlier exists() check. A race or
        # unusable parent fails here before main is allowed to open a port.
        self.output.mkdir(parents=True, exist_ok=False)
        self.started = clock()
        self.metadata = {
            "format_version": 1,
            "source": source,
            "source_name": str(source_name),
            "evidence_label": ("REPLAY_NOT_HARDWARE" if source == "replay"
                               else "SERIAL_CAPTURE_NOT_A_PASS_VERDICT"),
            "firmware_identifier_supplied_by_user": firmware,
            "test_label": test,
            "notes": notes,
            "started_utc": utc(),
            "ended_utc": None,
            "elapsed_seconds": None,
            "requested_duration_seconds": duration,
            "requested_baud_rate": baud if source == "hardware" else None,
            "serial_note": ("No commands sent; DTR/RTS requested low before opening. "
                            "OS/driver may still toggle control lines." if source == "hardware" else None),
            "status": "running",
            "bytes_received": 0,
            "read_chunks": 0,
            "files": ["console.log", "metadata.json"] + (["hostarrival.csv"] if line_csv else []),
            "timestamp_note": "UTC/elapsed are laptop arrival times, not MCU task timings.",
            "raw_note": "console.log preserves bytes; CSV replaces invalid UTF-8 for display.",
        }
        try:
            atomic_metadata(self.output, self.metadata)
            self.raw = (self.output / "console.log").open("xb")
            self.raw.flush()
            if line_csv:
                self.arrivals_file = (self.output / "hostarrival.csv").open("x", encoding="utf-8", newline="")
                self.arrivals = ArrivalCSV(self.arrivals_file)
                self.arrivals_file.flush()
        except BaseException as error:
            self.finish("error", f"Output preparation failed: {error}")
            raise

    def finish(self, status, error=None):
        if self.finished:
            return self.metadata
        self.metadata["status"] = status
        if error is not None:
            self.metadata["error"] = error
        try:
            if self.arrivals is not None:
                self.arrivals.finish()
        except Exception as failure:
            self.metadata.update(status="error", error=f"CSV finalization failed: {failure}")
        for file in (self.arrivals_file, self.raw):
            if file is not None:
                try:
                    file.close()
                except Exception as failure:
                    self.metadata.update(status="error", error=f"Log close failed: {failure}")
        self.metadata["ended_utc"] = self.utc()
        self.metadata["elapsed_seconds"] = max(0.0, self.clock() - self.started)
        try:
            atomic_metadata(self.output, self.metadata)
        except Exception as failure:
            # The prior valid running JSON is intentionally retained. The raw
            # log remains available; a failed finalization is not success.
            self.metadata.update(status="error", error=f"Metadata finalization failed: {failure}")
        self.finished = True
        return self.metadata


def capture(stream, output, *, source, source_name, firmware="unspecified",
            test="unspecified", notes="", duration=None, line_csv=False,
            baud=None, clock=time.monotonic, utc=utc_now, _recording=None):
    """Read bounded bytes. The caller owns stream; Recording owns output files.

    Empty replay reads mean EOF; empty live reads mean timeout. Main supplies a
    prepared recording so output is validated before the hardware port opens.
    """
    recording = _recording or Recording(output, source=source, source_name=source_name,
        firmware=firmware, test=test, notes=notes, duration=duration,
        line_csv=line_csv, baud=baud, clock=clock, utc=utc)
    reading_started = recording.clock()
    status, error = "complete", None
    try:
        while True:
            if duration is not None and recording.clock() - reading_started >= duration:
                break
            data = stream.read(READ_BYTES)
            if not isinstance(data, bytes):
                raise TypeError("binary input must return bytes")
            if not data:
                if source == "replay":
                    break
                continue
            recording.raw.write(data)
            recording.raw.flush()
            recording.metadata["bytes_received"] += len(data)
            recording.metadata["read_chunks"] += 1
            if recording.arrivals is not None:
                recording.arrivals.feed(data, recording.utc(), recording.clock() - recording.started)
                recording.arrivals_file.flush()
    except KeyboardInterrupt:
        status = "interrupted"
    except Exception as failure:
        status, error = "error", f"{type(failure).__name__}: {failure}"
    return recording.finish(status, error)


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return number


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="explicit serial device; never sends robot commands")
    source.add_argument("--replay", type=Path, help="read saved bytes; always labelled NOT HARDWARE")
    source.add_argument("--list-ports", action="store_true", help="list ports without opening them")
    parser.add_argument("--output", type=Path, help="NEW recording directory; existing directories refused")
    parser.add_argument("--baud", type=positive_int, default=115200)
    parser.add_argument("--duration", type=positive_float,
                        help="live seconds (up to one 0.25s read timeout extra); otherwise Ctrl+C")
    parser.add_argument("--firmware", default="unspecified", help="your build/profile identifier; not automatically verified")
    parser.add_argument("--test", default="unspecified", help="short test name")
    parser.add_argument("--notes", default="", help="conditions/observations; do not put passwords here")
    parser.add_argument("--line-csv", action="store_true", help="also write hostarrival.csv; NOT MCU timing")
    args = parser.parse_args(argv)
    if not args.list_ports and args.output is None:
        parser.error("--output is required for capture/replay")
    if args.replay is not None and args.duration is not None:
        parser.error("--duration is live-only; replay reads the whole file")
    if args.output is not None and (args.output.exists() or args.output.is_symlink()):
        parser.error("--output already exists; choose a new directory (nothing overwritten)")
    return args


def serial_module():
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("Live capture needs pyserial: python3 -m pip install pyserial. "
                           "--replay works without it.") from error
    return serial


def open_serial(port, baud):
    serial = serial_module()
    device = serial.Serial(port=None, baudrate=baud, timeout=READ_TIMEOUT_SECONDS,
                           write_timeout=READ_TIMEOUT_SECONDS)
    try:
        # Avoid an intentional reset pulse. Some OS/drivers may still toggle
        # lines while opening; this tool cannot guarantee reset-free opening.
        device.dtr = False
        device.rts = False
        device.port = port
        device.open()
    except BaseException:
        device.close()
        raise
    return device


def main(argv=None):
    args = parse_args(argv)
    stream = None
    recording = None
    try:
        if args.list_ports:
            serial_module()
            from serial.tools import list_ports
            for port in list_ports.comports():
                print(f"{port.device}\t{port.description}")
            return 0
        source = "replay" if args.replay is not None else "hardware"
        source_name = args.replay if source == "replay" else args.port
        if source == "replay":
            stream = args.replay.open("rb")  # Missing replay input touches no hardware/output.
        recording = Recording(args.output, source=source, source_name=source_name,
                              firmware=args.firmware, test=args.test, notes=args.notes,
                              duration=args.duration, line_csv=args.line_csv, baud=args.baud)
        if source == "hardware":
            stream = open_serial(args.port, args.baud)
        print("REPLAY — NOT HARDWARE EVIDENCE" if source == "replay" else
              "Read-only serial capture. Ctrl+C stops. Opening may toggle USB control lines.")
        result = capture(stream, args.output, source=source, source_name=source_name,
                         firmware=args.firmware, test=args.test, notes=args.notes,
                         duration=args.duration, line_csv=args.line_csv, baud=args.baud,
                         _recording=recording)
        print(f"Saved {result['bytes_received']} bytes to {args.output}; status={result['status']}.")
        if result["status"] == "error":
            print(result["error"], file=sys.stderr)
            return 1
        return 130 if result["status"] == "interrupted" else 0
    except KeyboardInterrupt:
        if recording is not None:
            recording.finish("interrupted", "Stopped before reading serial data.")
        print("Stopped before reading serial data.", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        if recording is not None:
            recording.finish("error", f"Input open/preparation failed: {error}")
        print(f"Capture failed: {error}", file=sys.stderr)
        return 1
    finally:
        if recording is not None and not recording.finished:
            recording.finish("error", "Recording ended before capture completed.")
        if stream is not None:
            try:
                stream.close()
            except OSError as error:
                print(f"Warning while closing input: {error}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
