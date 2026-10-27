#!/usr/bin/env python3
"""Live, human-readable view of the car's MQTT traffic (Buddy 1 protocol v2).

The car sends compact binary messages (see app_program/subsystem1/MQTT_PROTOCOL.md).
This laptop-side viewer decodes them as they arrive. It changes nothing on the
car and needs no extra packages: pipe mosquitto_sub's "topic hex" lines into it.

    mosquitto_sub -h 172.20.10.2 -t '#' -F '%t %x' | python3 tools/buddy1_live.py

By default repeated heartbeats and near-identical speed reports are folded so
the screen stays readable; use --all to print every message.

After every run the car also sends its barcode-sensor trace as text on
car/debug/barcode. The viewer shows the summary/episode lines and saves the
whole trace to barcode_trace_<date>_<time>.txt (see --trace-dir).
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from buddy1_mqtt_codec import decode  # noqa: E402  (same folder, standard library only)

SPEED_STEP_MMS = 10  # fold speed reports that changed less than this


def _faults(section):
    if not section:
        return "unknown"
    return ", ".join(section["active"]) or "none"


def _snapshot(d):
    parts = []
    for name in ("state", "motion", "barcode", "hump", "heading", "obstacle", "faults"):
        s = d[name]
        if not s["valid"]:
            parts.append(f"{name}=n/a")
        elif name == "state":
            parts.append(f"state={s['state']}")
        elif name == "motion":
            parts.append(f"speed L={s['left_mms']} R={s['right_mms']} mm/s")
        elif name == "barcode":
            parts.append(f"barcode={s['letter']}->{s['action']}")
        elif name == "hump":
            parts.append(f"humps={s['count']} max={s['height_mm']} mm")
        elif name == "heading":
            parts.append(f"heading={s['heading_deg']} deg")
        elif name == "obstacle":
            parts.append(f"obstacle {s['distance_mm']} mm away, {s['width_mm']} mm wide, go {s['route']}")
        elif name == "faults":
            parts.append(f"faults={_faults(s)}")
    return "SNAPSHOT  " + " | ".join(parts)


def describe(topic, d):
    """One readable line for a decoded message."""
    name = topic.rsplit("/", 1)[-1]
    if topic == "car/cmd/snapshot":
        return "SNAPSHOT REQUESTED (laptop -> car)"
    if topic == "car/cmd/barcode_map":
        return ("LETTER LIST SENT (laptop -> car)  forward={forward} left={left} "
                "right={right} uturn={uturn}").format(**d)
    if topic == "car/reply/barcode_map":
        return f"LETTER LIST REPLY  {d['result'].upper()}"
    if topic == "car/reply/snapshot":
        return _snapshot(d)
    if name == "heartbeat" or topic == "car/status":
        return f"HEARTBEAT  alive, faults: {_faults(d['faults'])}"
    if name == "state":
        return f"STATE     {d['state']}"
    if name == "motion":
        return f"SPEED     left {d['left_mms']} mm/s, right {d['right_mms']} mm/s"
    if name == "barcode":
        return f"BARCODE   {d['letter']} -> {d['action']}"
    if name == "faults":
        return f"FAULTS    {_faults(d)}"
    if name == "hump":
        return f"HUMP      count {d['count']}, highest {d['height_mm']} mm"
    if name == "heading":
        return f"HEADING   {d['heading_deg']} deg"
    if name == "obstacle":
        return f"OBSTACLE  {d['distance_mm']} mm away, {d['width_mm']} mm wide, route {d['route']}"
    return str(d)


class Folder:
    """Hide repeats that carry no new information (unless --all)."""

    def __init__(self, show_all):
        self.show_all = show_all
        self.last_heartbeat = None
        self.heartbeats = 0
        self.last_speed = None

    def keep(self, topic, d):
        if self.show_all:
            return True
        name = topic.rsplit("/", 1)[-1]
        if name == "heartbeat" or topic == "car/status":
            self.heartbeats += 1
            key = _faults(d["faults"])
            changed = key != self.last_heartbeat
            self.last_heartbeat = key
            return changed or self.heartbeats % 10 == 1
        if name == "motion" and topic.startswith("car/telemetry/"):
            speed = (d["left_mms"], d["right_mms"])
            if self.last_speed is not None and all(
                    abs(a - b) < SPEED_STEP_MMS for a, b in zip(speed, self.last_speed)):
                return False
            self.last_speed = speed
        return True


class TraceFile:
    """Saves the car's barcode-sensor trace (car/debug/barcode) to a file."""

    def __init__(self, folder):
        self.folder = Path(folder)
        self.handle = None
        self.path = None

    def line(self, text, out):
        if text.startswith("[BCAP] run summary"):
            self.close()
            self.path = self.folder / time.strftime("barcode_trace_%Y%m%d_%H%M%S.txt")
            self.handle = open(self.path, "w", encoding="ascii", errors="replace")
            print(f"--- saving barcode trace to {self.path}", file=out, flush=True)
        if self.handle:
            self.handle.write(text + "\n")
            self.handle.flush()
        if text == "[BCAP] end":
            if self.path:
                print(f"--- barcode trace saved: {self.path}", file=out, flush=True)
            self.close()

    def close(self):
        if self.handle:
            self.handle.close()
        self.handle = None


def handle(line, folder, out, trace=None):
    line = line.strip()
    if not line:
        return
    topic, _, hexpart = line.partition(" ")
    stamp = time.strftime("%H:%M:%S")
    if topic.startswith("car/debug/"):
        try:
            text = bytes.fromhex(hexpart.strip()).decode("ascii", "replace")
        except ValueError:
            text = hexpart.strip()
        if trace is not None:
            trace.line(text, out)
        # Sample lines are long and many: show headers, keep samples in the file.
        if not text.startswith("BC") or trace is None:
            print(f"{stamp}  DEBUG  {text}", file=out, flush=True)
        return
    try:
        payload = bytes.fromhex(hexpart.strip())
        d = decode(topic, payload)
    except ValueError as error:
        print(f"{stamp}  ?? {topic} {hexpart.strip()}  (not decoded: {error})", file=out, flush=True)
        return
    if folder.keep(topic, d):
        print(f"{stamp}  {describe(topic, d)}", file=out, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--all", action="store_true",
                        help="print every message, including repeated heartbeats and speeds")
    parser.add_argument("--trace-dir", default=".",
                        help="folder for barcode_trace_*.txt files (default: current folder)")
    args = parser.parse_args()
    folder = Folder(args.all)
    trace = TraceFile(args.trace_dir)
    try:
        for line in sys.stdin:
            handle(line, folder, sys.stdout, trace)
    except KeyboardInterrupt:
        pass
    finally:
        trace.close()


if __name__ == "__main__":
    main()
