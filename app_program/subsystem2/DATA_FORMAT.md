# Buddy 2 — Data Representation (draft for team agreement)

Status: **Proposed, implemented with these values** in `motion_pack.c` /
`motion_pack.h` (the steps and widths are `#define`s there, so they can change
if the team agrees other values). Ranges marked *to agree* still need a team
decision.

**Fault bits are implemented.** Buddy 2's faults are 5 individual bits
(`MOTION_FAULT_BIT()`, `motion_get_fault_bits()`), carried in the status word.
Where these 5 bits sit in the team's shared fault mask, and who owns the
mask, is still a team decision.

Scope: data that leaves Buddy 2 (telemetry, reports, fault bits) or comes in
from the Vehicle Controller (commands), packed into words with shifts and
masks; and Buddy 2's internal application data (states, flags, faults,
timers, counters, stored encoder counts), stored in C bit-fields of the width
their range needs ([Internal fields](#internal-fields)). Control values stay
at full precision (floats), as agreed in Tribunal 1. If packing ever limits
control, the field gets more bits (as done for speed below).

## Basis for the ranges

Measured on the robot (calibration, 21 Oct 2026) unless marked.

| Quantity | Value | Source |
| --- | --- | --- |
| Wheel diameter | 65.4 mm | measured, `WHEEL_DIAMETER_M` |
| Wheel circumference | π × 65.4 mm = 205.5 mm | |
| Encoder counts per wheel turn | 2534 (L 2533, R 2538) | calibration, `ENCODER_COUNTS_PER_REV` |
| Distance per count | 0.0811 mm | 205.5 / 2534 |
| Highest wheel speed | 884 mm/s (full power, wheels lifted) | calibration, `MOTOR_MAX_SPEED_MPS` |
| Highest commanded speed | 796 mm/s (90 % of 884 mm/s) | `SPEED_LIMIT_FRACTION` |
| Speed step of one count per 10 ms period | 8.1 mm/s | 0.0811 mm / 10 ms |
| Longest run | **30 m** (*to agree*) | track length + margin |

## Encoding rules

- **Direction bit + magnitude** (sign-magnitude) for every signed value, as
  decided in Tribunal 1: bit = 0 forward / left, 1 = backward / right. Kept
  because on-the-spot turns, `moveBackward()` and obstacle-recovery reversing
  all need it.
- Values are **rounded** to the nearest step and **saturate** at the largest
  magnitude; they never wrap. A saturated value means "at least this much".
- Unsigned fields count from 0. Unused bits are 0 and ignored by the reader.
- Pack and unpack shared messages with **shifts and masks**, not C bit-fields
  (bit-field layout depends on the compiler; the laptop decoder must match
  exactly). Internal data that never leaves the firmware uses C bit-fields.
- Validity is a **separate bit**, never a special value of the field.

## Fields

| Field | Direction | Range | Resolution | Encoding | Bits | Why this width |
| --- | --- | --- | --- | --- | --- | --- |
| Wheel speed, measured (L and R) | out | −1022 … +1022 mm/s | 2 mm/s | dir + 9-bit magnitude | 10 each | Covers the measured 884 mm/s. 2 mm/s is a quarter of the 8.1 mm/s count step, so finer steps carry only noise. 8 bits (510 mm/s) clipped the real robot |
| Wheel speed, command (L and R) | in | −1022 … +1022 mm/s | 2 mm/s | dir + 9-bit magnitude | 10 each | Same format as measured speed; Buddy 2 still caps at 796 mm/s |
| Robot distance travelled | out | −32.767 … +32.767 m | 1 mm | dir + 15-bit magnitude | 16 | Covers a 30 m run; 1 mm is finer than wheel slip errors |
| Drive distance, command | in | −8.191 … +8.191 m | 1 mm | dir + 13-bit magnitude | 14 | Longest single straight move (*to agree*); dir 1 = backward |
| Turn angle, command | in | −511 … +511 ° | 1 ° | dir + 9-bit magnitude | 10 | U-turn is 180 °; full spin 360 °; dir 1 = right |
| Move speed, command (drive / turn) | in | 0 … 1022 mm/s | 2 mm/s | unsigned | 9 | Direction is in the distance / angle |
| Heading | out | 0 … 359 ° | 1 ° | unsigned, wraps at 360 | 9 | 1 ° is ten times finer than the ±10 ° turn tolerance |
| Motion state | out | 6 states | — | code (table below) | 3 | 6 values |
| Motion fault bits (status) | out | 5 faults | — | 1 bit per fault, several at once | 5 | One bit per fault, as the requirement asks; position in the team's shared mask *to agree* |
| Fault code (report) | out | 6 values | — | code (table below) | 3 | The one fault that ended a command; with several bits set it is the lowest |
| Command ID | in / out | 1 … 255, 0 = none | — | unsigned, wraps 255 → 1 | 8 | The Vehicle Controller has one command in flight; it only checks equality |
| Command type | in | 7 types | — | code (table below) | 3 | 7 values |
| Manoeuvre | in | L / R / S / U | — | code | 2 | 4 values |
| Report outcome | out | 4 values | — | code (table below) | 2 | 4 values |
| Report lost flag | out | 0 / 1 | — | flag | 1 | Earlier reports dropped (queue full) |
| Stopped flag | out | 0 / 1 | — | flag | 1 | `motion_is_stopped()` |
| Busy flag | out | 0 / 1 | — | flag | 1 | Calibration running, requests rejected |
| Estimates valid | out | 0 / 1 | — | flag | 1 | 0 when the encoder PIO is not running |

### Internal fields

Stored inside Buddy 2 as C bit-fields of the width their range needs.
"Before" is the type used before this change.

| Field | Where | Range | Before | Bits | Representation |
| --- | --- | --- | --- | --- | --- |
| Statechart state | `motion_controller.c` `g_ctl.state` | 6 states | 32 (enum) | 3 | code (table below) |
| Latched fault bits | `g_ctl.fault_bits` | 5 faults | 32 (enum code) | 5 | 1 bit per fault, OR-ed in, cleared by `motion_reset()` |
| Command ID | `g_ctl.cmd_id` | 0 … 255 | 8 | 8 | unsigned, 255 → 1 |
| Busy (calibration) | `g_ctl.b_busy` | 0 / 1 | 8 (bool) | 1 | flag |
| Move kind | `g_ctl.b_mv_turn` | DRIVE / TURN | 32 (enum) | 1 | 1 = TURN |
| Move direction | `g_ctl.mv_back` | forward / backward per wheel | 2 × 32 (int ±1) | 2 | 1 bit per wheel, 1 = backward |
| Stored encoder counts: move start, stall reference, estimator's last count, encoder's last FIFO value | `wheel_count_t`, `wheel_ctrl_t`, `wheel_estimate_t`, `encoder_t` | ±524 287 counts = ±42.5 m | 32 (int32) | 20 | two's complement; subtract with `wheel_encoder_delta()`, correct across the wrap |
| Stall timer | `wheel_ctrl_t.stall_ms` | 0 … 650 ms (fault at 600) | 32 | 10 | unsigned, held at 1023 |
| Wrong-direction timer | `wheel_ctrl_t.wrong_ms` | 0 … 350 ms (fault at 300) | 32 | 9 | unsigned, held at 511 |
| Glitch counter copy | `wheel_ctrl_t.glitch_seen` | difference per period < 256 | 32 | 8 | low bits, difference mod 256 |
| Glitch bucket | `wheel_ctrl_t.glitch_bucket` | 0 … 50 | 32 | 6 | unsigned |
| Wheels-still timer | `g_still_ms` | 0 … 60 049 ms | 32 | 16 | `uint16_t` |
| Calibration requested / running / done | `motion.c` `g_flags` | 0 / 1 each | 3 × 8 (bool) | 3 × 1 | flags |
| Report lost | `g_flags.b_report_lost` | 0 / 1 | 8 (bool) | 1 | flag |
| Reports queued, peak | `g_flags.report_queued`, `report_peak` | 0 … 8 | 2 × 32 | 2 × 4 | unsigned |
| Encoder pin, state machine, inverted | `wheel_encoder.c` `encoder_t` | GPIO 0 … 29, SM 0 … 3, 0 / 1 | 32, 32, 8 | 5, 2, 1 | unsigned, flag |
| Motor pins, inverted | `motor_control.c` `motor_t` | GPIO 0 … 29 ×2, 0 / 1 | 32, 32, 8 | 5, 5, 1 | unsigned, flag |

`g_ctl` holds 20 bits in one word and `g_flags` 12 bits in one word.

Kept at full width, on purpose:

| Value | Width | Reason |
| --- | --- | --- |
| Speeds, accelerations, distances, setpoints, PID gains and terms | float (32) | Control precision kept, as agreed in Tribunal 1 |
| Timestamps (µs) | 32 | The width of the RP2040 timer; the wrap-safe time comparisons need all 32 bits |
| The PIO's own count (Y register) | 32 | Hardware register; only its low 20 bits are read and kept |
| `motion_status_t`, `motion_report_t` | — | Unpacked copies made for a caller; the stored and shared forms are the packed words |

### Safe access to packed data

- `g_ctl`, the controller's per-wheel fields and `g_flags` are changed and
  read only with the Buddy 2 mutex held (`TA_INHERIT`), so a read-modify-write
  of a packed word never interleaves with another one.
- The encoder and motor configuration fields and the estimator's last count
  are written only by the motion task (single writer).
- The encoder's last FIFO value can be updated by any task that reads a
  count, so it sits in a word of its own (zero-width bit-field) and never
  shares a read-modify-write with the inversion flag.

### Encoders: no ISR

The Week 6 design and Tribunal 1 assumed GPIO interrupts for the encoders.
The implementation decodes them in the PIO instead (README, *Encoder
decoding*), so there is no ISR and no ISR/task sharing. A count is one 32-bit
FIFO read, so a reader never sees a torn value, and counts are never reset:
moves and estimates use differences, so there is no copy-and-reset race.
20 bits cover a 30 m run (30 000 / 0.0811 = 370 k counts = 19 bits +
direction), and `wheel_encoder_delta()` stays correct when a count wraps.

## Codes

| Motion state | Code | | Fault code | Code | Fault bit |
| --- | --- | --- | --- | --- | --- |
| IDLE | 0 | | NONE | 0 | — |
| VELOCITY | 1 | | STALL | 1 | bit 0 |
| DRIVE | 2 | | STALE_COMMAND | 2 | bit 1 |
| TURN | 3 | | ENCODER | 3 | bit 2 |
| COMPLETE | 4 | | DIRECTION | 4 | bit 3 |
| FAULT | 5 | | TIMEOUT | 5 | bit 4 |

| Command type | Code | | Outcome | Code |
| --- | --- | --- | --- | --- |
| VELOCITY | 0 | | NONE | 0 |
| DRIVE | 1 | | DONE | 1 |
| TURN | 2 | | CANCELLED | 2 |
| MANOEUVRE | 3 | | FAULT | 3 |
| STOP | 4 | | | |
| CANCEL | 5 | | | |
| RESET | 6 | | | |

Manoeuvre codes: LEFT 0, RIGHT 1, STRAIGHT 2, U_TURN 3 (as `manoeuvre_t`).

The fault **bits** above are Buddy 2's part of the shared fault mask
(`MOTION_FAULT_BIT(f)` = bit f − 1). Only the motion task sets them and only
`motion_reset()` clears them, both with the Buddy 2 mutex held. A fault on
each wheel in the same period latches both bits. Where the 5 bits sit in the
team's mask, and who owns it, is a team decision.

## Packed messages

### Status / telemetry — 64 bits (was 28 bytes as floats and ints)

`motion_get_status_packed()`; unpack with `motion_unpack_status()`.

| Bits | Field | Width |
| --- | --- | --- |
| 0–2 | Motion state | 3 |
| 3–7 | Fault bits (bit 3 = stall … bit 7 = timeout) | 5 |
| 8–15 | Current command ID | 8 |
| 16–25 | Left wheel speed (bit 25 = direction) | 10 |
| 26–35 | Right wheel speed (bit 35 = direction) | 10 |
| 36–51 | Distance travelled (bit 51 = direction) | 16 |
| 52–60 | Heading | 9 |
| 61 | Stopped | 1 |
| 62 | Busy | 1 |
| 63 | Estimates valid | 1 |

All 64 bits are used.

Written only by Buddy 2; a reader gets a complete copy taken under the Buddy 2
mutex, so it never sees a half-updated word.

### Report — 16 bits (was 16 bytes)

| Bits | Field | Width |
| --- | --- | --- |
| 0–7 | Command ID | 8 |
| 8–9 | Outcome | 2 |
| 10–12 | Fault code | 3 |
| 13 | Lost (earlier reports dropped) | 1 |
| 14–15 | Reserved (0) | 2 |

Stored packed in the report queue; `motion_get_report()` unpacks it. Queue
memory: 8 × (4-byte kernel header + 4-byte rounded message) = 64 bytes (was
160).

### Command — 32 bits (Vehicle Controller → Buddy 2)

| Bits | VELOCITY | DRIVE | TURN | MANOEUVRE | CANCEL | STOP / RESET |
| --- | --- | --- | --- | --- | --- | --- |
| 0–2 | type 0 | type 1 | type 2 | type 3 | type 5 | type 4 / 6 |
| 3–… | left speed (10), right speed (10) | distance (14), speed (9) | angle (10), speed (9) | manoeuvre (2) | command ID (8) | — |
| Used bits | 23 | 26 | 22 | 5 | 11 | 3 |

Build a word with `motion_cmd_velocity()`, `motion_cmd_drive()`,
`motion_cmd_turn()`, `motion_cmd_manoeuvre()`, `motion_cmd_stop()`,
`motion_cmd_cancel()` or `motion_cmd_reset()`, and run it with
`motion_request_packed()`. An unknown type (7) is rejected. The existing
functions (`moveForward()`, `turnLeft()`, `stop()`, …) stay.

## To agree with the team

1. Longest run (30 m proposed) and longest single drive (8.191 m proposed).
2. Speed: 2 mm/s with a 9-bit magnitude (implemented, covers the measured
   884 mm/s) or 4 mm/s with an 8-bit magnitude (1 bit less per speed).
3. The shared fault mask: bit positions for every buddy and its owner
   (Buddy 2's 5 bits are implemented; their position in the mask is not).
4. Telemetry rate for the status word (Buddy 1) and byte order on MQTT.
5. Whether commands use the packed 32-bit word or stay as function calls.
