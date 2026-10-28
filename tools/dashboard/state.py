"""Thread-safe, bounded display state derived from original MQTT payload bytes."""
from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import threading
import time
import math

from buddy1_mqtt_codec import NAMES, WIDTHS, VERSION, decode, encode_map
from .ir_protocol import IRBuffer, IR_TOPIC, decode_ir, encode_ir
from .control_protocol import TOPIC as CONTROL_TOPIC, decode_control, encode_control
from .debug_history import DebugHistory

HEARTBEAT_TOPIC = "mtk3smp-pico/heartbeat"
COMMAND_TIMEOUT = 8.0
CAR_TIMEOUT = 15.0
MOTION_TIMEOUT = 3.0
ACTIVE_STATES = {"line_following", "turning_left", "turning_right", "uturn",
                 "avoiding_obstacle"}


def timestamp():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class DashboardState:
    def __init__(self, traces, mode="live", clock=time.monotonic,
                 heartbeat_topic=HEARTBEAT_TOPIC):
        self.lock = threading.RLock()
        self.clock, self.mode, self.traces = clock, mode, traces
        self.heartbeat_topic = heartbeat_topic
        self.connection = dict(status="disconnected", host="127.0.0.1", port=1883,
                               error=None)
        self.last_car = self.last_heartbeat = None
        self.sections = {name: dict(valid=False, data={}, received_at=None,
                                    source="unavailable", mono=None)
                         for name in NAMES}
        self.commands = {"map": self._idle(), "snapshot": self._idle(),
                         "map_lock": "unknown"}
        self.events = deque(maxlen=160)
        self.history = deque(maxlen=120)
        self.counters = dict(received=0, malformed=0, bytes_received=0)
        self.event_id = 0
        self.transport = None
        self.demo_scenario = "ready"
        self.demo_map = dict(forward="A", left="Z", right="Y", uturn="C")
        self._demo_pending = None
        self.ir = IRBuffer(clock, mode)
        self.debug = DebugHistory(clock, mode)
        self._last_control_event = None
        self._demo_control_last = -1
        self._demo_encoders = [0,0]
        self._demo_epoch = self.clock()
        self._demo_sequence = 0

    @staticmethod
    def _idle():
        return dict(status="idle", detail="No request sent", letters=None)

    def add_event(self, buddy, level, message):
        with self.lock:
            self.event_id += 1
            self.events.appendleft(dict(id=self.event_id, time=timestamp(),
                                        buddy=buddy, level=level, message=str(message)[:1000]))

    def set_connection(self, status, host=None, port=None, error=None):
        with self.lock:
            changed = status != self.connection["status"] or error != self.connection["error"]
            self.connection.update(status=status, error=error)
            if host is not None:
                self.connection["host"] = host
            if port is not None:
                self.connection["port"] = port
            if status != "connected":
                for key in ("map", "snapshot"):
                    if self.commands[key]["status"] == "pending":
                        self.commands[key].update(status="error", detail="Connection lost; command outcome unknown")
            if changed:
                self.debug.connection(status)
                self.add_event(1, "warning" if error else "info",
                               f"Broker {status}" + (f": {error}" if error else ""))

    def _update(self, name, data, source="telemetry", valid=True):
        if source == "telemetry":
            self.debug.report(name,data,valid)
        previous = self.sections[name]
        changed = previous["data"] != data or previous["valid"] != valid
        self.sections[name] = dict(valid=valid, data=data if valid else {},
                                   received_at=timestamp(), source=source,
                                   mono=self.clock())
        if changed and name in ("state", "barcode", "faults") and valid:
            buddy = 3 if name == "barcode" else (2 if name == "state" else "all")
            if name == "state":
                message = "Robot state: " + data["state"].replace("_", " ")
            elif name == "barcode":
                message = f"Barcode {data['letter']} → {data['action']}"
            else:
                message = "Faults: " + (", ".join(data["active"]).replace("_", " ") or "none reported")
            self.add_event(buddy, "warning" if name == "faults" and data.get("active") else "info",
                           message)
        # Observed active navigation is evidence the first start occurred. No
        # reboot ID exists; a reconnect or a STOP must never clear this latch.
        if name == "state" and valid and data.get("state") in ACTIVE_STATES:
            self.commands["map_lock"] = "locked"

    def _reply(self, kind, status, detail):
        pending = self.commands[kind]["status"] == "pending"
        if pending:
            self.commands[kind].update(status=status, detail=detail)
        self.add_event(1, "info" if status == "accepted" else "warning",
                       ("Reply: " if pending else "Unsolicited/late reply: ") + detail)

    def ingest(self, topic, payload, retained=False):
        with self.lock:
            if retained:
                return  # retained old packets must not establish liveness/ack
            payload = bytes(payload)
            if len(payload) > 65536:
                self.counters["malformed"] += 1
                self.add_event(1, "warning", "Oversized MQTT payload ignored")
                return
            if topic == IR_TOPIC:
                try:
                    frame = decode_ir(payload)
                except ValueError as exc:
                    self.counters["malformed"] += 1
                    self.add_event(3, "warning", str(exc))
                    return
                if self.ir.add(frame):
                    self.last_car = self.clock()
                    self.counters["received"] += 1
                    self.counters["bytes_received"] += len(payload)
                    self.debug.receive(len(payload))
                return
            if topic == CONTROL_TOPIC:
                try:
                    frame = decode_control(payload)
                except ValueError as exc:
                    self.counters["malformed"] += 1
                    self.add_event(3,"warning",str(exc))
                    return
                if self.debug.controller.add(frame):
                    self.last_car = self.clock()
                    self.counters["received"] += 1
                    self.counters["bytes_received"] += len(payload)
                    self.debug.receive(len(payload))
                    d = frame["data"]
                    key = (d["drive_state"],d["reason"],d["pending"])
                    if key != self._last_control_event:
                        from .control_protocol import DRIVE_STATES, REASONS, ACTIONS
                        self.add_event("controller","warning" if d["reason"] else "info",
                                       f"Controller {DRIVE_STATES[d['drive_state']]}: {REASONS[d['reason']]} · pending {ACTIONS[d['pending']]}")
                        self._last_control_event = key
                return
            if topic == "car/debug/barcode":
                try:
                    text = payload.decode("ascii")
                    self.traces.ingest(text)
                    self.last_car = self.clock()
                    self.counters["received"] += 1
                    self.counters["bytes_received"] += len(payload)
                    self.debug.receive(len(payload))
                    if text.startswith("[BCAP] run summary") or text.strip() == "[BCAP] end":
                        self.add_event(3, "info", text)
                except (ValueError, OSError, UnicodeError) as exc:
                    self.add_event(3, "warning", f"Trace ignored: {exc}")
                return
            is_heartbeat = topic in ("car/status", self.heartbeat_topic)
            if not is_heartbeat and topic not in {"car/reply/snapshot", "car/reply/barcode_map"} and not (
                    topic.startswith("car/telemetry/") and topic.rsplit("/", 1)[-1] in NAMES):
                return
            try:
                decoded = decode("car/status" if is_heartbeat else topic, payload)
            except (ValueError, TypeError) as exc:
                self.counters["malformed"] += 1
                self.add_event(1, "warning", f"Rejected {topic}: {exc}")
                return
            self.last_car = self.clock()
            self.counters["received"] += 1
            self.counters["bytes_received"] += len(payload)
            self.debug.receive(len(payload))
            self.history.appendleft(dict(time=timestamp(), topic=topic,
                                          hex=payload.hex(), decoded=decoded))
            if is_heartbeat:
                self.last_heartbeat = self.clock()
                if decoded["faults_valid"]:
                    self._update("faults", decoded["faults"], source="heartbeat")
            elif topic == "car/reply/barcode_map":
                result = decoded["result"]
                if result == "busy":
                    self.commands["map_lock"] = "locked"
                detail = {"ok": "OK reply received. Protocol has no request ID or map readback; use one command sender.",
                          "busy": "Configuration closed until robot reboot.",
                          "invalid": "Robot rejected the letter map.",
                          "internal": "Robot reported an internal error."}[result]
                self._reply("map", "accepted" if decoded["ok"] else "rejected", detail)
            elif topic == "car/reply/snapshot":
                for name, value in decoded.items():
                    self._update(name, {k: v for k, v in value.items() if k != "valid"},
                                 source="snapshot", valid=value["valid"])
                self._reply("snapshot", "accepted", "Snapshot received. Values are cached; sensor sample ages are unknown.")
            else:
                self._update(topic.rsplit("/", 1)[-1], decoded)

    def _expire(self):
        for kind in ("map", "snapshot"):
            command = self.commands[kind]
            if command["status"] == "pending" and self.clock() - command["sent_mono"] >= COMMAND_TIMEOUT:
                command.update(status="timeout", detail="No reply; application outcome unknown. Not automatically resent.")
                self.add_event(1, "warning", kind + ": " + command["detail"])

    def request(self, kind, letters=None):
        if kind not in ("map", "snapshot"):
            raise ValueError("Unsupported command")
        if kind == "map":
            if not isinstance(letters, dict) or set(letters) != {"forward", "left", "right", "uturn"}:
                raise ValueError("Supply forward, left, right and uturn letters")
            if any(not isinstance(v, str) for v in letters.values()):
                raise ValueError("Letters must be strings")
            payload = encode_map(*(letters[key] for key in ("forward", "left", "right", "uturn")))
        else:
            payload = bytes([VERSION])
        with self.lock:
            self._expire()
            if self.connection["status"] != "connected" or self.last_car is None or self.clock() - self.last_car > CAR_TIMEOUT:
                raise ValueError("Connect and wait for recent robot traffic first")
            if any(self.commands[k]["status"] == "pending" for k in ("map", "snapshot")):
                raise ValueError("Wait for the current command reply")
            # The protocol has no boot ID. A previously observed lock is shown
            # to the operator, but an explicit retry after a physical reboot
            # must reach the firmware, which remains the authoritative gate.
            self.commands[kind] = dict(status="pending", detail="Waiting for robot reply",
                                       letters=deepcopy(letters), sent_mono=self.clock())
            self.add_event(1, "info", f"{kind} request sent; waiting for reply")
            if self.mode == "demo":
                self._demo_pending = (self.clock() + 0.6, kind, letters)
                return
        try:
            self.transport.publish("car/cmd/barcode_map" if kind == "map" else "car/cmd/snapshot", payload)
        except (OSError, RuntimeError, ValueError) as exc:
            with self.lock:
                if self.commands[kind]["status"] == "pending":
                    self.commands[kind].update(status="error", detail=f"Publish failed: {exc}")
            raise ValueError(str(exc)) from exc

    def snapshot(self):
        with self.lock:
            self._expire()
            now = self.clock()
            age = None if self.last_car is None else max(0, now - self.last_car)
            online = self.connection["status"] == "connected" and age is not None and age <= CAR_TIMEOUT
            sections = {}
            for name, section in self.sections.items():
                item = {k: deepcopy(v) for k, v in section.items() if k != "mono"}
                item["age_s"] = None if section["mono"] is None else round(max(0, now - section["mono"]), 2)
                item["stale"] = not online or (name == "motion" and item["age_s"] is not None and item["age_s"] > MOTION_TIMEOUT)
                item["freshness"] = ("unavailable" if section["mono"] is None else "invalid" if not item["valid"]
                                     else "cached" if item["source"] == "snapshot" else "live" if name == "motion"
                                     else "last_report")
                if item["source"] == "snapshot":
                    item["stale"] = True
                sections[name] = item
            return dict(mode=self.mode, connection=deepcopy(self.connection),
                        car=dict(status="online" if online else "unknown" if age is None else "stale",
                                 age_s=round(age, 2) if age is not None else None,
                                 heartbeat_age_s=None if self.last_heartbeat is None else round(now - self.last_heartbeat, 2)),
                        sections=sections, commands=deepcopy(self.commands),
                        events=list(self.events), counters=dict(self.counters),
                        traces=self.traces.list(), diagnostics={"raw_messages": list(self.history),
                            "ir": self.ir.snapshot(self.connection["status"] == "connected", False),
                            "controller": self.debug.controller.snapshot(self.connection["status"] == "connected",False)},
                        demo_scenario=self.demo_scenario)

    def demo_select(self, scenario):
        if self.mode != "demo":
            raise ValueError("Demo scenarios are available only with --demo")
        if scenario not in {"ready", "following", "line_lost", "wifi_loss", "fault", "barcode"}:
            raise ValueError("Unknown demo scenario")
        with self.lock:
            self.demo_scenario = scenario
            if scenario == "ready":
                # Explicit simulation reboot, never applied to a real connection.
                self.commands = {"map": self._idle(), "snapshot": self._idle(), "map_lock": "unknown"}
                self._demo_pending = None
                self._demo_epoch = self.clock()
                self._demo_sequence = 0
                self.ir = IRBuffer(self.clock, self.mode)
                self.debug = DebugHistory(self.clock,self.mode)
                self._last_control_event = None
                self._demo_control_last = -1
                self._demo_encoders = [0,0]
            self.add_event("all", "info", "SIMULATION: " + scenario.replace("_", " "))
            if scenario == "wifi_loss":
                self.set_connection("disconnected", error="Simulated Wi-Fi/broker loss")
            else:
                self.set_connection("connected", host="simulation", port=1883)
                self.demo_tick()
                self.demo_ir_tick()

    def sensor_snapshot(self):
        with self.lock:
            return self.ir.snapshot(self.connection["status"] == "connected")

    def debug_snapshot(self):
        with self.lock:
            return self.debug.snapshot(self.connection["status"] == "connected")

    def demo_ir_tick(self):
        with self.lock:
            if self.mode != "demo" or self.demo_scenario == "wifi_loss":
                return
            t = self.clock() - self._demo_epoch
            active = self.demo_scenario in {"following", "barcode"}
            left = int(max(0, math.sin(t * 1.2)) * 900) if active else 0
            right = int(max(0, -math.sin(t * 1.2)) * 900) if active else 0
            phase = t % 7.5
            barcode = 970 if active and 1 < phase < 6 and int(phase * 5) % 4 in (0, 1) else 15
            darkness = [left, right, barcode]
            raw = [180 + int(value * 3.45) for value in darkness]
            surfaces = [2 if value >= 600 else 1 if value <= 400 else 0 for value in darkness]
            payload = encode_ir(int(t * 1e6) & 0xffffffff, self._demo_sequence,
                                raw, darkness, surfaces)
            self._demo_sequence = (self._demo_sequence + 1) & 255
            self.ingest(IR_TOPIC, payload)
            if t-self._demo_control_last >= 0.2:
                self._demo_control_last = t
                following = self.demo_scenario in {"following","barcode"}
                fault = self.demo_scenario == "fault"
                v = dict(timestamp_us=int(t*1e6)&0xffffffff,motion_timestamp_us=int(t*1e6)&0xffffffff,
                         sequence=int(t*5)&255,motion_available=1,motion_valid=1,braked=int(not following),
                         motion_state=5 if fault else 1 if following else 0,motion_fault=1 if fault else 0,
                         drive_state=6 if fault else 1 if following else 5 if self.demo_scenario=="line_lost" else 0,
                         reason=2 if fault else 4 if self.demo_scenario=="line_lost" else 0,
                         line_error=round(math.sin(t)*60) if following else 0,line_flags=3,
                         accepted_calibration=7,sample_gap_us=10000,run_max_gap_us=11000,
                         setpoint_left=40 if following else 0,setpoint_right=40 if following else 0,
                         measured_left=round(40+math.sin(t*2)*6) if following else 0,
                         measured_right=round(40-math.sin(t*2)*6) if following else 0,
                         duty_left=18 if following else 0,duty_right=19 if following else 0,
                         pending=1 if self.demo_scenario=="barcode" else 0)
                for c in ("left","right","barcode"):
                    v.update({c+"_white":180,c+"_black":3600,c+"_white_limit":400,c+"_black_limit":600})
                for i,c in enumerate(("left","right")):
                    # Illustrative generated counts, not a calibration model.
                    self._demo_encoders[i]=((self._demo_encoders[i]+round(v["measured_"+c]*2.4)+524288)%1048576)-524288
                    v["encoder_"+c]=self._demo_encoders[i]
                self.ingest(CONTROL_TOPIC,encode_control(**v))

    def demo_tick(self):
        with self.lock:
            if self.mode != "demo" or self.demo_scenario == "wifi_loss":
                return
            scenario = self.demo_scenario
            state = 1 if scenario in {"following", "barcode"} else 7 if scenario == "fault" else 0
            speed = 40 if state == 1 else 0
            bits = speed // 2 | (speed // 2 << 9)
            self.ingest("car/telemetry/motion", bytes([VERSION]) + bits.to_bytes(3, "little"))
            self.ingest("car/telemetry/state", bytes([VERSION, state]))
            self.ingest("car/status", bytes([VERSION, 3 | (1 << 2 if scenario == "fault" else 0)]))
            if scenario == "barcode":
                self.ingest("car/telemetry/barcode", bytes([VERSION, 25 | 2 << 5]))
            if self._demo_pending and self.clock() >= self._demo_pending[0]:
                _, kind, letters = self._demo_pending
                self._demo_pending = None
                if kind == "map":
                    busy = self.commands["map_lock"] == "locked"
                    if not busy:
                        self.demo_map = letters
                    self.ingest("car/reply/barcode_map", bytes([VERSION, 2 if busy else 0]))
                else:
                    value, offset = 0, 7
                    for i, (name, width) in enumerate(zip(NAMES, WIDTHS)):
                        section = self.sections[name]
                        raw = 0
                        if section["valid"] and name in {"motion", "state", "faults", "barcode"}:
                            value |= 1 << i
                            raw = {"motion": bits, "state": state,
                                   "faults": 1 if scenario == "fault" else 0,
                                   "barcode": 25 | 2 << 5}[name]
                        value |= raw << offset
                        offset += width
                    self.ingest("car/reply/snapshot", bytes([VERSION]) + value.to_bytes(15, "little"))
