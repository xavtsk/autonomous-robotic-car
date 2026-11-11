"""Thread-safe, bounded display state derived from original MQTT payload bytes."""
from collections import deque
from copy import deepcopy
from contextlib import nullcontext
from datetime import datetime, timezone
import threading
import time
import math

from buddy1_mqtt_codec import NAMES, WIDTHS, VERSION, SNAPSHOT_BYTES, decode, encode_map
from .ir_protocol import IRBuffer, IR_TOPIC, decode_ir, encode_ir
from .control_protocol import TOPIC as CONTROL_TOPIC, decode_control, encode_control
from .debug_history import DebugHistory
from .route import RouteTracker
from .run_control_protocol import (COMMAND_TOPIC as RUN_COMMAND_TOPIC, REPLY_TOPIC as RUN_REPLY_TOPIC,
    STATUS_TOPIC as RUN_STATUS_TOPIC, encode_request, decode_reply, encode_reply, decode_status, encode_status)

POSE_TOPIC = "car/telemetry/pose"
TERRAIN_TOPIC = "car/telemetry/terrain"
LIVE_SECTIONS = ("motion", "terrain")  # periodic 5 Hz streams: stale after MOTION_TIMEOUT

HEARTBEAT_TOPIC = "mtk3smp-pico/heartbeat"
COMMAND_TIMEOUT = 8.0
CAR_TIMEOUT = 15.0
MOTION_TIMEOUT = 3.0
ACTIVE_STATES = {"line_following", "turning_left", "turning_right", "uturn",
                 "avoiding_obstacle"}
RUN_PENDING = {"sending", "sent", "queued", "executed"}


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
                         for name in NAMES + ("terrain",)}
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
        self.route = RouteTracker(clock)
        self._demo_pose_t0 = None
        self._demo_barcode_last = None
        self._run_request = None
        self._run_id = 0
        self._run_lease = None
        self._run_publish_lock = threading.Lock()
        self._run_sample = None
        self._run_sample_mono = None
        self._run_sample_serial = 0
        self._demo_run_pending = None

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
                self._run_sample = None
                self._run_lease = None
                if self._run_request and self._run_request["status"] in RUN_PENDING:
                    self._run_request.update(status="error", detail="Connection lost; outcome unknown. Not automatically resent.")
                self._demo_run_pending = None
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
            if topic == RUN_REPLY_TOPIC:
                try:
                    reply = decode_reply(payload)
                except ValueError as exc:
                    self.counters["malformed"] += 1
                    self.add_event("controller", "warning", str(exc))
                    return
                self.last_car = self.clock()
                self.counters["received"] += 1
                self.counters["bytes_received"] += len(payload)
                self.debug.receive(len(payload))
                self._run_reply(reply)
                return
            if topic == RUN_STATUS_TOPIC:
                try:
                    lease = decode_status(payload)
                except ValueError as exc:
                    self.counters["malformed"] += 1
                    self.add_event("controller", "warning", str(exc))
                    return
                prior_epoch = (self._run_lease["epoch"] if self._run_lease else
                               self._run_request["epoch"] if self._run_request else None)
                if prior_epoch != lease["epoch"]:
                    if prior_epoch is not None:
                        # A reconnect token cannot make a previous connection's
                        # controller preview current, even if received recently.
                        self._run_sample = None
                        self._run_sample_mono = None
                    self._run_id = lease["last_id"]
                    if (self._run_request and self._run_request["status"] in RUN_PENDING
                            and self._run_request["action"] != "stop"):
                        self._run_request.update(status="error", detail="Robot command connection changed; outcome unknown. Not resent.")
                elif 0 < ((lease["last_id"] - self._run_id) & 0xffffffff) < 0x80000000:
                    self._run_id = lease["last_id"]
                self._run_lease = dict(lease, received_mono=self.clock())
                self.last_car = self.clock()
                self.counters["received"] += 1
                self.counters["bytes_received"] += len(payload)
                self.debug.receive(len(payload))
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
                    self._run_sample = d
                    self._run_sample_mono = self.clock()
                    self._run_sample_serial += 1
                    self._observe_run()
                    key = (d["drive_state"],d["reason"],d["pending"])
                    if key != self._last_control_event:
                        from .control_protocol import DRIVE_STATES, REASONS, ACTIONS
                        self.add_event("controller","warning" if d["reason"] else "info",
                                       f"Controller {DRIVE_STATES[d['drive_state']]}: {REASONS[d['reason']]} · pending {ACTIONS[d['pending']]}")
                        self._last_control_event = key
                return
            if topic == "car/debug/barcode":
                try:
                    frame = self.traces.ingest_binary(payload)
                    text = frame["text"]
                    self.last_car = self.clock()
                    self.counters["received"] += 1
                    self.counters["bytes_received"] += len(payload)
                    self.debug.receive(len(payload))
                    if text.startswith("[BCAP] run summary") or text.strip() == "[BCAP] end":
                        self.add_event(3, "info", text)
                except (ValueError, OSError, UnicodeError) as exc:
                    self.counters["malformed"] += 1
                    self.add_event(3, "warning", f"Trace ignored: {exc}")
                return
            if topic == POSE_TOPIC:
                try:
                    decoded = decode(topic, payload)
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
                self.route.pose(decoded["distance_cm_mod"], decoded["heading_deg"], timestamp())
                return
            if topic == TERRAIN_TOPIC:
                try:
                    decoded = decode(topic, payload)
                except (ValueError, TypeError) as exc:
                    self.counters["malformed"] += 1
                    self.add_event(4, "warning", f"Rejected {topic}: {exc}")
                    return
                self.last_car = self.clock()
                self.counters["received"] += 1
                self.counters["bytes_received"] += len(payload)
                self.debug.receive(len(payload))
                self.history.appendleft(dict(time=timestamp(), topic=topic,
                                              hex=payload.hex(), decoded=decoded))
                previous = self.sections["terrain"]["data"]
                if decoded["state"] == "impact" and previous.get("state") != "impact":
                    self.add_event(4, "warning", "Buddy 4: sudden impact detected")
                if decoded["peak_valid"] and decoded["peak_mm"] != previous.get("peak_mm"):
                    self.add_event(4, "info", f"Buddy 4: highest hump this run {decoded['peak_mm']} mm")
                self._update("terrain", decoded)
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
                name = topic.rsplit("/", 1)[-1]
                self._update(name, decoded)
                self.route.telemetry(name, decoded, timestamp())

    def _expire(self):
        request = self._run_request
        if request and request["status"] in RUN_PENDING and self.clock() - request["sent_mono"] >= COMMAND_TIMEOUT:
            request.update(status="timeout", detail="No confirmed controller outcome; not automatically resent. STOP remains available.")
            self.add_event("controller", "warning", request["action"] + ": " + request["detail"])
        for kind in ("map", "snapshot"):
            command = self.commands[kind]
            if command["status"] == "pending" and self.clock() - command["sent_mono"] >= COMMAND_TIMEOUT:
                command.update(status="timeout", detail="No reply; application outcome unknown. Not automatically resent.")
                self.add_event(1, "warning", kind + ": " + command["detail"])

    def _run_view(self):
        from .control_protocol import DRIVE_STATES, REASONS
        connected = self.connection["status"] == "connected"
        d = self._run_sample
        fresh = bool(connected and d is not None and self.clock() - self._run_sample_mono <= 1.0)
        running = bool(fresh and 1 <= d["drive_state"] <= 4)
        stopped = bool(fresh and d["motion_available"] and d["motion_valid"] and d["braked"]
                       and d["drive_state"] in (0, 5, 6))
        pending = bool(self._run_request and self._run_request["status"] in RUN_PENDING)
        lease = self._run_lease
        capable = bool(lease and self.clock() - lease["received_mono"] <= 2.0 and lease["available"])
        start = reset = ""
        if not connected:
            start = reset = "Connect to the MQTT broker first."
        elif not fresh:
            start = reset = "Wait for fresh controller telemetry."
        elif not capable:
            start = reset = "Wait for fresh firmware control availability."
        elif pending:
            start = reset = "A control request is pending; STOP remains available."
        else:
            if d["drive_state"] == 6:
                start = "Reset the controller fault before starting."
            elif d["drive_state"] not in (0, 5):
                start = "The controller is already active."
            elif d["collecting"] or d["accepted_calibration"] != 7 or d["readiness"]:
                start = "Controller sensors/calibration are not ready."
            elif not (stopped and d["motion_state"] == 0 and d["motion_fault"] == 0 and not d["busy"]):
                start = "Wait for valid, idle, braked motor feedback."
            elif not lease["can_start"]:
                start = "Firmware is not ready to start; check sensor/IMU readiness."
            elif (self._run_request and self._run_request["action"] == "stop"
                  and self._run_request["status"] != "observed"):
                start = "STOP outcome is unconfirmed; request STOP again and wait for confirmation."
            if d["drive_state"] != 6:
                reset = "Reset is available only for a controller fault."
            elif not lease["can_reset"]:
                reset = "Firmware is not ready to reset the fault."
        stop = "" if connected else "Connect to the MQTT broker first."
        request = None
        if self._run_request:
            request = {k: deepcopy(v) for k, v in self._run_request.items()
                       if k not in ("sent_mono", "sample_serial", "epoch")}
            request["age_s"] = round(max(0, self.clock() - self._run_request["sent_mono"]), 2)
        return dict(can_start=not start, can_stop=not stop, can_reset=not reset,
                    reason=start or "Ready to request START; firmware still checks readiness.",
                    reasons=dict(start=start, stop=stop, reset=reset), pending=pending,
                    last_request=request, observed=dict(fresh=fresh, running=running, stopped=stopped,
                        state=DRIVE_STATES[d["drive_state"]] if d else None,
                        reason=REASONS[d["reason"]] if d else None))

    def _observe_run(self):
        request = self._run_request
        if not request or request["status"] != "executed" or self._run_sample_serial <= request["sample_serial"]:
            return
        view = self._run_view()["observed"]
        action = request["action"]
        matches = (action == "start" and view["running"] or
                   action == "stop" and view["stopped"] or
                   action == "reset" and view["stopped"] and view["state"] == "off")
        if matches:
            request.update(status="observed", detail=("Controller running observed." if action == "start" else
                "Controller stopped/braked observed." if action == "stop" else "Fault reset and stopped controller observed; START is a separate action."))
            self.add_event("controller", "info", action + ": " + request["detail"])

    def _run_reply(self, reply):
        self._expire()
        request = self._run_request
        if (not request or request["status"] not in RUN_PENDING or
                reply["id"] != request["id"] or reply["action"] != request["action"] or
                (request["action"] != "stop" and reply["epoch"] != request["epoch"])):
            self.add_event("controller", "warning", "Unmatched or late control reply ignored.")
            return
        if reply["result"] in ("queued", "executed"):
            # A delayed queue reply cannot downgrade a later execution reply.
            result = "executed" if request["status"] == "executed" else reply["result"]
            request.update(status=result, detail="Firmware " + result + " the request; waiting for observed controller state.")
            self._observe_run()
        else:
            request.update(status=reply["result"], detail="Firmware " + reply["result"] + " request: " + reply["reason"].replace("_", " ") + ".")
        self.add_event("controller", "info" if reply["result"] in ("queued", "executed") else "warning", request["detail"])

    def request_control(self, action):
        if not isinstance(action, str) or action not in ("start", "stop", "reset"):
            raise ValueError("Action must be start, stop or reset")
        with self.lock:
            self._expire()
            view = self._run_view()
            if not view["can_" + action]:
                raise ValueError(view["reasons"][action])
            if self._run_request and self._run_request["status"] in RUN_PENDING:
                self.add_event("controller", "warning", "Pending " + self._run_request["action"] + " superseded by STOP; its outcome may be unknown.")
            self._run_id = (self._run_id + 1) & 0xffffffff or 1
            ident = self._run_id
            # A dashboard-only reconnect may clear fresh status while an older
            # START publisher is still in flight. Keep its epoch for STOP so
            # firmware advances that connection's request-ID watermark.
            lease = self._run_lease or dict(epoch=self._run_request["epoch"] if self._run_request else 0,
                                           robot_time_us=0)
            payload = encode_request(ident, action, lease["epoch"], lease["robot_time_us"])
            self._run_request = dict(id=ident, action=action, status="sending", detail="Sending one non-retained MQTT request.",
                sent_at=timestamp(), sent_mono=self.clock(), sample_serial=self._run_sample_serial, epoch=lease["epoch"])
            if self.mode == "demo":
                self._run_request.update(status="sent", detail="SIMULATION: sent; waiting for firmware queue reply.")
                self._demo_run_pending = (self.clock() + 0.2, ident, action, "reply")
                return
        # STOP bypasses a slow publisher. Firmware's newer-ID watermark rejects
        # an older START arriving after STOP; unsent superseded requests skip.
        try:
            with (nullcontext() if action == "stop" else self._run_publish_lock):
                with self.lock:
                    if self._run_request["id"] != ident or self._run_request["status"] != "sending":
                        return
                self.transport.publish(RUN_COMMAND_TOPIC, payload)
            with self.lock:
                if self._run_request["id"] == ident and self._run_request["status"] == "sending":
                    self._run_request.update(status="sent", detail="MQTT sent; waiting for firmware queue reply. This does not confirm movement.")
        except (OSError, RuntimeError, ValueError) as exc:
            with self.lock:
                if self._run_request["id"] == ident and self._run_request["status"] in RUN_PENDING:
                    self._run_request.update(status="error", detail="Publish failed or timed out; outcome unknown. Not automatically resent.")
            raise ValueError(str(exc)) from exc

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
                item["stale"] = not online or (name in LIVE_SECTIONS and item["age_s"] is not None and item["age_s"] > MOTION_TIMEOUT)
                item["freshness"] = ("unavailable" if section["mono"] is None else "invalid" if not item["valid"]
                                     else "cached" if item["source"] == "snapshot" else "live" if name in LIVE_SECTIONS
                                     else "last_report")
                if item["source"] == "snapshot":
                    item["stale"] = True
                sections[name] = item
            return dict(mode=self.mode, connection=deepcopy(self.connection),
                        car=dict(status="online" if online else "unknown" if age is None else "stale",
                                 age_s=round(age, 2) if age is not None else None,
                                 heartbeat_age_s=None if self.last_heartbeat is None else round(now - self.last_heartbeat, 2)),
                        sections=sections, commands=deepcopy(self.commands),
                        control=self._run_view(),
                        events=list(self.events), counters=dict(self.counters),
                        traces=self.traces.list(), diagnostics={"raw_messages": list(self.history),
                            "ir": self.ir.snapshot(self.connection["status"] == "connected", False),
                            "controller": self.debug.controller.snapshot(self.connection["status"] == "connected",False)},
                        demo_scenario=self.demo_scenario, route=self.route.snapshot())

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
                self.route = RouteTracker(self.clock)
                self._demo_pose_t0 = None
                self._demo_barcode_last = None
                self._demo_peak = None
                self._run_request = None
                self._run_sample = None
                self._run_lease = None
                self._run_id = 0
                self._demo_run_pending = None
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
            if self._demo_run_pending and self.clock() >= self._demo_run_pending[0]:
                _, ident, action, phase = self._demo_run_pending
                self._demo_run_pending = None
                if self._run_request and self._run_request["id"] == ident and self._run_request["status"] in RUN_PENDING:
                    if phase == "reply":
                        self.ingest(RUN_REPLY_TOPIC, encode_reply(ident, action, "queued"))
                        self._demo_run_pending = (self.clock() + 0.2, ident, action, "observed")
                    else:
                        self.ingest(RUN_REPLY_TOPIC, encode_reply(ident, action, "executed"))
                        self.demo_scenario = "following" if action == "start" else "fault" if action == "stop" and self.demo_scenario == "fault" else "ready"
                        self._demo_control_last = -1
                        self.demo_tick()
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
                self._demo_pose(t, active)
                self._demo_terrain(t, active)
                following = self.demo_scenario in {"following","barcode"}
                fault = self.demo_scenario == "fault"
                self.ingest(RUN_STATUS_TOPIC, encode_status(epoch=1, robot_time_us=int(t*1e6)&0xffffffff,
                    last_id=self._run_id, flags=1 | (8 if following else 4 if fault else 2)))
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

    def _demo_terrain(self, t, active):
        """Simulated Buddy 4: a gentle 30 mm hump every 15 s while following."""
        phase = t % 15.0
        on = active and 5 < phase < 8
        pitch = round(80 * math.sin((phase - 5) / 3 * 2 * math.pi)) if on else 0
        roll = round(15 * math.sin(t * 0.7)) if active else 0
        state = (5 if pitch > 40 else 6 if pitch < -40 else 2) if active else 1
        climb = round(30 * math.sin((phase - 5) / 3 * math.pi)) if on else 0
        if on and phase > 7.8:
            self._demo_peak = 30
        peak = getattr(self, "_demo_peak", None) if active else None
        body = (1 | (pitch & 0x7ff) << 1 | (roll & 0x7ff) << 12 | state << 23 |
                int(on) << 26 | climb << 27 | int(peak is not None) << 36 | (peak or 0) << 37)
        self.ingest(TERRAIN_TOPIC, bytes([VERSION]) + body.to_bytes(6, "little"))

    def _demo_pose(self, t, active):
        """Simulated pose on an oval: 60 cm straights, 15 cm radius ends, 4 cm/s."""
        if not active:
            self._demo_pose_t0 = None
            return
        if self._demo_pose_t0 is None:
            self._demo_pose_t0 = t
        straight, radius = 60.0, 15.0
        arc = math.pi * radius
        lap = 2 * (straight + arc)
        d = (t - self._demo_pose_t0) * 4.0
        s = d % lap
        if s < straight:
            heading = 0.0
        elif s < straight + arc:
            heading = (s - straight) / arc * 180.0
        elif s < 2 * straight + arc:
            heading = 180.0
        else:
            heading = 180.0 + (s - 2 * straight - arc) / arc * 180.0
        bits = (int(round(d)) % 4096) | ((int(round(heading)) % 360) << 12)
        self.ingest(POSE_TOPIC, bytes([VERSION]) + bits.to_bytes(3, "little"))

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
            if scenario == "barcode" and (self._demo_barcode_last is None or
                                          self.clock() - self._demo_barcode_last >= 12.0):
                # One simulated decode every 12 s: each real report is one event.
                self._demo_barcode_last = self.clock()
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
                    self.ingest("car/reply/snapshot", bytes([VERSION]) + value.to_bytes(SNAPSHOT_BYTES-1, "little"))
