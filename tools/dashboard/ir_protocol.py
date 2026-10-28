"""IR debug v1: clock32 + raw36 + normalized30 + validity6 + surface6 + seq8.

The 118 body bits follow one version byte; two padding bits are zero.
This observation topic is separate from Buddy 1 telemetry protocol v2.
"""
from collections import deque

IR_TOPIC = "car/debug/ir"
IR_VERSION = 1
CHANNELS = ("left", "right", "barcode")
SURFACES = ("unknown", "white", "black")


def decode_ir(payload):
    payload = bytes(payload)
    if len(payload) != 16 or payload[0] != IR_VERSION:
        raise ValueError("IR preview requires a 16-byte version-1 packet")
    bits = int.from_bytes(payload[1:], "little")
    if bits >> 118:
        raise ValueError("Nonzero IR preview padding")
    raw_mask, norm_mask = bits >> 98 & 7, bits >> 101 & 7
    if norm_mask & ~raw_mask:
        raise ValueError("Normalized IR validity requires raw validity")
    sensors = {}
    for i, name in enumerate(CHANNELS):
        raw, darkness = bits >> (32 + 12 * i) & 4095, bits >> (68 + 10 * i) & 1023
        surface = bits >> (104 + 2 * i) & 3
        raw_valid, valid = bool(raw_mask & (1 << i)), bool(norm_mask & (1 << i))
        if surface > 2 or darkness > 1000 or (not raw_valid and raw) or (not valid and (darkness or surface)):
            raise ValueError("Invalid or noncanonical IR preview field")
        sensors[name] = dict(raw=raw if raw_valid else None,
                             darkness=darkness if valid else None,
                             raw_valid=raw_valid, valid=valid,
                             surface=SURFACES[surface] if valid else "unavailable")
    return dict(timestamp_us=bits & 0xffffffff, sequence=bits >> 110 & 255,
                sensors=sensors)


def encode_ir(timestamp_us, sequence, raw, darkness, surfaces=(1, 1, 1), raw_valid=7, normalised_valid=7):
    """Synthetic packet builder for demo/tests, never a robot command."""
    if len(raw) != 3 or len(darkness) != 3 or len(surfaces) != 3:
        raise ValueError("Expected three IR channels")
    if not 0 <= timestamp_us <= 0xffffffff or not 0 <= sequence <= 255 or not 0 <= raw_valid <= 7 or not 0 <= normalised_valid <= 7:
        raise ValueError("IR field out of range")
    bits = timestamp_us | raw_valid << 98 | normalised_valid << 101 | sequence << 110
    for i in range(3):
        if raw_valid & (1 << i):
            if not 0 <= raw[i] <= 4095:
                raise ValueError("IR raw value out of range")
            bits |= raw[i] << (32 + 12 * i)
        if normalised_valid & (1 << i):
            if not 0 <= darkness[i] <= 1000 or not 0 <= surfaces[i] <= 2:
                raise ValueError("IR normalized field out of range")
            bits |= darkness[i] << (68 + 10 * i) | surfaces[i] << (104 + 2 * i)
    packet = bytes([IR_VERSION]) + bits.to_bytes(15, "little")
    decode_ir(packet)
    return packet


class IRBuffer:
    """Latest minute at 20 Hz. Caller holds DashboardState.lock."""
    def __init__(self, clock, mode="live"):
        self.clock, self.mode = clock, mode
        self.samples = deque(maxlen=1200)
        self.last_received = self.last_clock = self.last_sequence = None
        self.elapsed_us = 0
        self.received = self.skipped = self.restarts = 0

    def add(self, frame):
        timestamp, sequence = frame["timestamp_us"], frame["sequence"]
        gap = False
        if self.last_clock is not None:
            delta = (timestamp - self.last_clock) & 0xffffffff
            if delta == 0 and sequence == self.last_sequence:
                return False
            if delta >= 0x80000000:
                self.samples.clear()
                self.elapsed_us = 0
                self.restarts += 1
                gap = True
            else:
                self.elapsed_us += delta
                gap = delta > 250000
                if delta < 12800000:
                    self.skipped += max(0, ((sequence - self.last_sequence) & 255) - 1)
        self.last_received = self.clock()
        self.last_clock, self.last_sequence = timestamp, sequence
        self.received += 1
        self.samples.append(dict(t_ms=self.elapsed_us / 1000, gap_before=gap, **frame))
        return True

    def snapshot(self, connected, include_samples=True):
        age = None if self.last_received is None else max(0, self.clock() - self.last_received)
        result = dict(source="simulation" if self.mode == "demo" else "wireless",
                      age_s=None if age is None else round(age, 2),
                      stale=not connected or age is None or age > 1,
                      received=self.received, skipped_previews=self.skipped,
                      clock_restarts=self.restarts,
                      latest=self.samples[-1] if self.samples else None)
        if include_samples:
            result["samples"] = list(self.samples)
        return result
