# Buddy 5 data representation and sharing

Tribunal 1: justify range/unit/resolution before assigning bits; validity is
separate from zero; binary shared data; synchronise actual shared updates.

## Encoded task snapshot (same-firmware internal format)

Least-significant bit first; fields follow in this order. **182 meaningful bits,
23 bytes, six zero padding bits**. Exact re-encoding rejects noncanonical padding
and invalid values. Decoding leaves the destination unchanged on failure.
This internal format is not a new MQTT packet/version.

| Field | Range / unit | Bits |
|---|---|---:|
| Forward measurement source time | wrapping uint32 microseconds | 32 |
| Completed scan source time | wrapping uint32 microseconds | 32 |
| Scan identity | 1–65535, zero = no scan; wraps skipping zero | 16 |
| Forward distance | valid 30–4000 mm, 1 mm; zero when invalid | 12 |
| Forward validity | false/true | 1 |
| Confirmed no-echo/invalid-return failure | false/true after 3 consecutive failed measurements | 1 |
| Scan completed | false/true | 1 |
| Profile valid | false/true | 1 |
| Closest distance | 30–4000 mm when valid | 12 |
| Conservative lateral width | 0–4095 mm | 12 |
| Left clearance | 0–4000 mm | 12 |
| Right clearance | 0–4000 mm | 12 |
| Estimated forward extent | 0–4000 mm | 12 |
| Left/right lateral edge, each | −4000..4000 mm; encoded +4000 | 13 each |

The conservative width includes angular/beam allowance; do not label it an exact
physical width measurement. The 4000 mm sensing limit follows the HC-SR04 range;
30 mm is this application's deliberately narrower minimum, not the datasheet's
minimum. Invalid/stale values do not become clearance.

## Other retained values

- Scan sample: 12 distance bits, 5 angle-index bits (30–150 degrees in 5 degree
  steps), one validity bit = **18 bits / 3 bytes**, upper six padding bits zero.
  Capacity 16 samples (48 bytes); the current coarse+fine schedule uses at most 12.
- Scan request ID: one 16-bit mailbox slot, with separate protected cancel/busy
  flags; no queue of old route requests.
- Controller diagnostic status: 3 state + 3 fault + 2 route + 4 leg + 3
  leg-3 step + 1 side-swapped bits = **one 16-bit word** (13 Nov 2026: leg,
  step and swap added for telemetry). Only the controller writes; telemetry
  readers copy under the mutex.
- Servo angle for telemetry: one byte, last commanded 1..180 degrees (0 = not
  positioned). Written only by the S5 task, read whole (single aligned byte).
- Private FSM state/fault/leg/issued use 3/3/4/1 bits; configuration distances
  use 12-bit fields and speed 9 bits. Bypass flags added 13 Nov 2026 (reverse
  checked/running, aim done/running, bend, tape tail, extra drive) are 1 bit
  each; the aim turn is a 7-bit field (-40..40 degrees, 81 values).
- Bypass state added 17 Nov 2026 (one owner, the controller task): heading vs
  the passing heading 8 bits (-90..90), pending side turn 4 bits (0..10 deg),
  side turns made 2 bits (0..3), hand-back tries 2 bits (0..2), consecutive
  blocked readings 2 bits (0..2), sideways travel after turning away 9 bits
  (0..511 mm, saturating), distance from the line at the last side peek 12 bits
  (0..4095 mm). Checked by `testing/change_requirement_1/check_widths.py`.
- Bend measure from the controller (14 Nov 2026): whole percent -100..100 in an
  `int8_t` (201 values, 8 bits), latched when the obstacle is detected. The
  controller's private averaging filter keeps 1/10000 steps in an `int16_t` so
  the slow average does not stall; it was a `float` before. Full-width microsecond timers preserve
  wrapping-clock semantics. Full-precision odometry is retained for calculations.
- Public decoded views and temporary geometry arithmetic are not wire structs.
  No raw C struct/bit-field layout crosses the task boundary.

S5 sampling is single-owner. The shared packet, request and status are protected
by the same priority-inheritance mutex. Controller reads are bounded/nonblocking;
no sampling, scanning delay or motor command occurs while holding it. The task
checks fresh stopped feedback itself before scanning, independently of the
controller's request. PIO MMIO uses per-SM control and atomic set/clear aliases;
no encoder FIFO reader is added.

Buddy 1 owns the global fault word. The controller updates only NO_ECHO using its
existing changed-mask API; it never writes a copied global word back. Buddy 5's
MQTT data (obstacle, avoidance and scan topics, protocol v3) uses the same
12/13-bit distance widths as this snapshot; see Buddy 1's data representation.

There was no older Buddy 5 firmware. The before/after in
[`docs/CHANGE_REQUIREMENT_1_EVIDENCE.md`](../../docs/CHANGE_REQUIREMENT_1_EVIDENCE.md)
compares these packed formats with the same data in the decoded-view structs,
measured by the Arm compiler. Runtime stack/heap/timing are recorded separately.
Physical width, clearance margins, maximum depth, scan latency and recovery
accuracy still need real-car validation and Week 13 demonstration evidence.
