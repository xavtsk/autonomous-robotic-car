"""Observed runs and route reconstruction from controller state and encoder pose.

The robot sends distance since START (cm, modulo 4096) and heading since START
(degrees, + = anticlockwise) every 200 ms while a run is active, a (0, 0) pose
at START and a final pose when the run ends. This module unwraps the distance,
integrates x/y and places event markers (barcode, hump, obstacle, turns) at the
latest pose when each event arrives. It is a sketch of the route: wheel slip
during spins and humps bends it. Nothing here is sent to the car.
"""
from collections import deque
import math
from uuid import uuid4

from buddy1_mqtt_codec import unwrap_distance

MAX_RUNS = 5
MAX_POINTS = 6000          # 20 minutes at 5 Hz per run
MAX_MARKERS = 300
MAX_SENT_POINTS = 1500     # decimated for the browser
# A (0, 0) pose that would need an impossible jump is a new START whose
# "stopped" report was lost. Top speed 0.884 m/s covers ~18 cm per 200 ms
# sample; a normal pass through a multiple of 40.96 m at heading 0 is a 1-step.
RESTART_JUMP_CM = 60
STOP_CONFIRM_S = 1.0  # allow queued poses to arrive before a fallback stop
HUMP_DRAIN_S = 2.0    # allow a late count/peak report from the stopped run
TURN_STATES = {"turning_left": "Left turn", "turning_right": "Right turn",
               "uturn": "U-turn", "avoiding_obstacle": "Avoiding obstacle"}
ACTIVE_STATES = {"line_following", *TURN_STATES}
END_STATES = {"stopped", "complete", "error"}


def _heading_step(previous, current):
    """Signed change in degrees, in -180..+180."""
    return ((current - previous + 180.0) % 360.0) - 180.0


class RouteTracker:
    def __init__(self, clock, record_run=None):
        self.clock = clock
        self.record_run = record_run
        self.runs = deque(maxlen=MAX_RUNS)
        self.current = None
        self.ended = True
        self.counter = 0
        self.idle_since = None
        self.ended_mono = None
        self.awaiting_hump_boundary = False

    def _new_run(self, received_at, *, inferred=False):
        self.counter += 1
        self.current = dict(id=self.counter,
                            log=dict(id=uuid4().hex, started_at=received_at, ended_at=None,
                                     hump_count=None, peak_mm=None),
                            joined_mid_run=False, distance_cm=None, start_cm=None,
                            heading_deg=None, x_cm=0.0, y_cm=0.0, path_cm=0.0,
                            points=[], markers=[], dropped_points=0)
        self.runs.appendleft(self.current)
        self.ended = False
        self.idle_since = None
        self.last_activity = self.clock()
        self.awaiting_hump_boundary = inferred
        self._save_run()
        return self.current

    def _save_run(self):
        if self.record_run is not None:
            self.record_run(self.current["log"])

    @property
    def current_log_id(self):
        return self.current["log"]["id"] if self.current is not None else None

    @staticmethod
    def _first_pose(run, wrapped_cm, heading):
        # A nonzero first pose leaves the earlier route unknown.
        start_cm = unwrap_distance(0, wrapped_cm)
        run.update(joined_mid_run=(wrapped_cm != 0 or heading != 0),
                   distance_cm=start_cm, start_cm=start_cm, heading_deg=heading)
        run["points"].append(dict(x=0.0, y=0.0, d=0))

    def pose(self, wrapped_cm, heading, received_at):
        run = self.current
        if run is None or self.ended:
            run = self._new_run(received_at)
        self.last_activity = self.clock()
        if wrapped_cm == 0 and heading == 0:
            self.awaiting_hump_boundary = False
        if not run["points"]:
            self._first_pose(run, wrapped_cm, heading)
            return
        distance = unwrap_distance(run["distance_cm"], wrapped_cm)
        if wrapped_cm == 0 and heading == 0 and abs(distance - run["distance_cm"]) > RESTART_JUMP_CM:
            self.end(received_at)
            self._first_pose(self._new_run(received_at), wrapped_cm, heading)
            return
        step = distance - run["distance_cm"]
        # Average heading over the step; heading 0 = START direction, drawn up.
        middle = math.radians(run["heading_deg"] + _heading_step(run["heading_deg"], heading) / 2.0)
        run["x_cm"] += -step * math.sin(middle)
        run["y_cm"] += step * math.cos(middle)
        run["path_cm"] += abs(step)
        run["distance_cm"], run["heading_deg"] = distance, heading
        if len(run["points"]) < MAX_POINTS:
            run["points"].append(dict(x=round(run["x_cm"], 1), y=round(run["y_cm"], 1),
                                      d=distance - run["start_cm"]))
        else:
            run["dropped_points"] += 1

    def end(self, received_at):
        if self.current is not None and not self.ended:
            self.current["log"]["ended_at"] = received_at
            self.ended = True
            self.ended_mono = self.clock()
            self._save_run()

    def interrupt(self):
        # Keep the observed results when the link drops without an observed STOP.
        self.current = None
        self.ended = True
        self.idle_since = None

    def controller_status(self, running, received_at):
        if running:
            if self.ended:
                # Status can overtake the ordinary telemetry FIFO. Wait for
                # a start pose, active state or reset terrain report before
                # assigning a positive hump measurement to this run.
                self._new_run(received_at, inferred=True)
            self.idle_since = None
            return
        if self.ended:
            return
        now = self.clock()
        if self.idle_since is None:
            self.idle_since = now
        elif now - max(self.idle_since, self.last_activity) >= STOP_CONFIRM_S:
            self.marker("end", "Run stopped", received_at)
            self.end(received_at)

    def _update_humps(self, count, peak_mm):
        log = self.current["log"]
        if log["peak_mm"] is not None and (peak_mm is None or peak_mm < log["peak_mm"]):
            peak_mm = log["peak_mm"]
        if (count, peak_mm) == (log["hump_count"], log["peak_mm"]):
            return False
        log.update(hump_count=count, peak_mm=peak_mm)
        self._save_run()
        return True

    def _accept_humps(self):
        return self.current is not None and (
            not self.ended or self.clock() - self.ended_mono <= HUMP_DRAIN_S)

    def terrain(self, data):
        if not self._accept_humps():
            return
        if self.awaiting_hump_boundary:
            if data["peak_valid"]:
                return
            self.awaiting_hump_boundary = False
        log = self.current["log"]
        if data["peak_valid"]:
            # Live terrain can prove a completed hump even if the count packet was lost.
            self._update_humps(log["hump_count"] or None, data["peak_mm"])
        elif (not self.ended and data["tilt_valid"] and data["state"] not in ("unknown", "impact")
              and log["hump_count"] is None and log["peak_mm"] is None):
            self._update_humps(0, None)

    def marker(self, kind, label, received_at):
        """Place an event at the latest pose. Ignored when no run is active."""
        run = self.current
        if run is None or self.ended or not run["points"] or len(run["markers"]) >= MAX_MARKERS:
            return
        run["markers"].append(dict(kind=kind, label=label, time=received_at,
                                   x=round(run["x_cm"], 1), y=round(run["y_cm"], 1),
                                   d=run["distance_cm"] - run["start_cm"]))

    def telemetry(self, name, data, received_at):
        """Turn decoded Buddy 1 telemetry into route markers or run boundaries."""
        if name == "barcode":
            self.marker("barcode", f"Barcode {data['letter']} → {data['action']}", received_at)
        elif name == "hump" and self._accept_humps() and not self.awaiting_hump_boundary:
            # The firmware can retry this cumulative report after STOP.
            count, height = data["count"], data["height_mm"]
            log = self.current["log"]
            if count >= (log["hump_count"] or 0) and (count or log["peak_mm"] is None):
                if self._update_humps(count, height if count else None) and count:
                    self.marker("hump", f"Hump {count} · peak {height} mm", received_at)
        elif name == "obstacle":
            self.marker("obstacle", f"Obstacle {data['width_mm']} mm wide at {data['distance_mm']} mm · "
                                    f"route {data['route']}", received_at)
        elif name == "state":
            state = data["state"]
            if state in ACTIVE_STATES:
                if self.ended:
                    self._new_run(received_at)
                self.last_activity = self.clock()
                self.awaiting_hump_boundary = False
            if state in TURN_STATES:
                self.marker("turn", TURN_STATES[state], received_at)
            elif state in END_STATES:
                self.marker("end", "Run " + state, received_at)
                self.end(received_at)

    def snapshot(self):
        runs = []
        for run in self.runs:
            points = run["points"]
            log = run["log"]
            stride = max(1, math.ceil(len(points) / MAX_SENT_POINTS))
            sent = points[::stride]
            if points and sent[-1] is not points[-1]:
                sent.append(points[-1])
            runs.append(dict(id=run["id"], started_at=log["started_at"], ended_at=log["ended_at"],
                             active=run is self.current and not self.ended,
                             joined_mid_run=run["joined_mid_run"],
                             distance_m=round((run["distance_cm"] - run["start_cm"]) / 100.0, 2) if points else None,
                             path_m=round(run["path_cm"] / 100.0, 2) if points else None,
                             heading_deg=run["heading_deg"], points=sent,
                             point_count=len(points), dropped_points=run["dropped_points"],
                             markers=list(run["markers"])))
        return dict(runs=runs)
