"""Bounded, laptop-side history. Receive time is not MCU measurement time."""
from collections import deque
from .ir_protocol import IRBuffer


class DebugHistory:
    def __init__(self, clock, mode):
        self.clock, self.mode, self.origin = clock, mode, clock()
        self.controller = IRBuffer(clock,mode)
        self.controller.samples = deque(maxlen=300)  # one minute at 5 Hz
        self.telemetry = {k:deque(maxlen=300) for k in ("motion","heading","hump","obstacle")}
        self.traffic = deque(maxlen=120)
        self.connections = deque(maxlen=60)

    def receive(self, size):
        second = int(self.clock()-self.origin)
        if not self.traffic or self.traffic[-1]["second"] != second:
            self.traffic.append(dict(second=second,messages=0,bytes=0))
        self.traffic[-1]["messages"] += 1
        self.traffic[-1]["bytes"] += size

    def connection(self, status):
        self.connections.append(dict(t_ms=round((self.clock()-self.origin)*1000),status=status))

    def report(self, name, data, valid):
        if name in self.telemetry:
            self.telemetry[name].append(dict(t_ms=round((self.clock()-self.origin)*1000),
                                              data=data if valid else None))

    def snapshot(self, connected):
        now = self.clock()-self.origin
        buckets = {p["second"]:p for p in self.traffic}
        # Only completed seconds: a partial current bucket is not a per-second rate.
        rates = [dict(t_ms=s*1000,messages=buckets.get(s,{}).get("messages",0),
                      bytes=buckets.get(s,{}).get("bytes",0))
                 for s in range(max(0,int(now)-60),int(now))]
        return dict(mode=self.mode,now_ms=round(now*1000),
                    controller=self.controller.snapshot(connected),
                    telemetry={k:list(v) for k,v in self.telemetry.items()},
                    rates=rates,connections=list(self.connections))
