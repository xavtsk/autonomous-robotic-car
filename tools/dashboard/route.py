"""Route reconstruction from car/telemetry/pose (encoder dead reckoning).

The robot sends distance since START (cm, modulo 4096) and heading since START
(degrees, + = anticlockwise) every 200 ms while a run is active, a (0, 0) pose
at START and a final pose when the run ends. This module unwraps the distance,
integrates x/y and places event markers (barcode, hump, obstacle, turns) at the
latest pose when each event arrives. It is a sketch of the route: wheel slip
during spins and humps bends it. Nothing here is sent to the car.
"""
from collections import deque
import math

from buddy1_mqtt_codec import POSE_DISTANCE_MOD, unwrap_distance

MAX_RUNS = 5
MAX_POINTS = 6000          # 20 minutes at 5 Hz per run
MAX_MARKERS = 300
MAX_SENT_POINTS = 1500     # decimated for the browser
# A (0, 0) pose that would need an impossible jump is a new START whose
# "stopped" report was lost. Top speed 0.884 m/s covers ~18 cm per 200 ms
# sample; a normal pass through a multiple of 40.96 m at heading 0 is a 1-step.
RESTART_JUMP_CM = 60
TURN_STATES = {"turning_left": "Left turn", "turning_right": "Right turn",
               "uturn": "U-turn", "avoiding_obstacle": "Avoiding obstacle"}
END_STATES = {"stopped", "complete", "error"}


def _heading_step(previous, current):
    """Signed change in degrees, in -180..+180."""
    return ((current - previous + 180.0) % 360.0) - 180.0


class RouteTracker:
    def __init__(self, clock):
        self.clock = clock
        self.runs = deque(maxlen=MAX_RUNS)
        self.current = None
        self.ended = True
        self.counter = 0

    def _new_run(self, wrapped_cm, heading, received_at):
        joined = not (wrapped_cm == 0 and heading == 0)
        # Joining mid-run: the earlier path is unknown, so the route starts here
        # and only distances from now on are meaningful.
        start_cm = wrapped_cm if wrapped_cm < POSE_DISTANCE_MOD // 2 else wrapped_cm - POSE_DISTANCE_MOD
        self.counter += 1
        self.current = dict(id=self.counter, started_at=received_at, ended_at=None,
                            joined_mid_run=joined, distance_cm=start_cm, start_cm=start_cm,
                            heading_deg=heading, x_cm=0.0, y_cm=0.0, path_cm=0.0,
                            points=[dict(x=0.0, y=0.0, d=0)], markers=[], dropped_points=0)
        self.runs.appendleft(self.current)
        self.ended = False

    def pose(self, wrapped_cm, heading, received_at):
        run = self.current
        if run is None or self.ended:
            self._new_run(wrapped_cm, heading, received_at)
            return
        distance = unwrap_distance(run["distance_cm"], wrapped_cm)
        if wrapped_cm == 0 and heading == 0 and abs(distance - run["distance_cm"]) > RESTART_JUMP_CM:
            self.end(received_at)
            self._new_run(wrapped_cm, heading, received_at)
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
            self.current["ended_at"] = received_at
            self.ended = True

    def marker(self, kind, label, received_at):
        """Place an event at the latest pose. Ignored when no run is active."""
        run = self.current
        if run is None or self.ended or len(run["markers"]) >= MAX_MARKERS:
            return
        run["markers"].append(dict(kind=kind, label=label, time=received_at,
                                   x=round(run["x_cm"], 1), y=round(run["y_cm"], 1),
                                   d=run["distance_cm"] - run["start_cm"]))

    def telemetry(self, name, data, received_at):
        """Turn decoded Buddy 1 telemetry into route markers or run boundaries."""
        if name == "barcode":
            self.marker("barcode", f"Barcode {data['letter']} → {data['action']}", received_at)
        elif name == "hump":
            self.marker("hump", f"Hump {data['count']} · peak {data['height_mm']} mm", received_at)
        elif name == "obstacle":
            self.marker("obstacle", f"Obstacle {data['width_mm']} mm wide at {data['distance_mm']} mm · "
                                    f"route {data['route']}", received_at)
        elif name == "state":
            state = data["state"]
            if state in TURN_STATES:
                self.marker("turn", TURN_STATES[state], received_at)
            elif state in END_STATES:
                self.marker("end", "Run " + state, received_at)
                self.end(received_at)

    def snapshot(self):
        runs = []
        for run in self.runs:
            points = run["points"]
            stride = max(1, math.ceil(len(points) / MAX_SENT_POINTS))
            sent = points[::stride]
            if sent[-1] is not points[-1]:
                sent.append(points[-1])
            runs.append(dict(id=run["id"], started_at=run["started_at"], ended_at=run["ended_at"],
                             active=run is self.current and not self.ended,
                             joined_mid_run=run["joined_mid_run"],
                             distance_m=round((run["distance_cm"] - run["start_cm"]) / 100.0, 2),
                             path_m=round(run["path_cm"] / 100.0, 2),
                             heading_deg=run["heading_deg"], points=sent,
                             point_count=len(points), dropped_points=run["dropped_points"],
                             markers=list(run["markers"])))
        return dict(runs=runs)
