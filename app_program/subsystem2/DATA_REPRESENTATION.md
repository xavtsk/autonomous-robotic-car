# Buddy 2 Tribunal 1 data representation

## Superseded representation-only baseline

The material below records the **23 October pre-pull implementation**, not the
current PIO driver or calibration layout. On 25 October the integration adopted
Buddy 2 `a29f973` plus the feedback/lease additions in `71193e2` without altering
its control algorithms. The current authoritative formats are [DATA_FORMAT.md](DATA_FORMAT.md):
20-bit modular encoder counts, 64-bit status (speed up to +/-1022 mm/s), 16-bit
reports in an eight-entry queue and 32-bit commands. Calibration includes an
abort flag and is no longer the old 28-byte struct. The legacy 4-byte reporting
adapter remains only for diagnostics/Buddy 1 compatibility; it invalidates
speeds outside +/-510 mm/s. See [current integration review](../../docs/INTEGRATION_2026-10-25.md).

## Historical detail (not current RAM or interrupt evidence)

This is a representation-only update. PID equations/gains, filtering, ramps,
PWM frequency/pins, quadrature lookup, calibration calculations, fault
thresholds, stop/reset rules, motion states and the 10 ms schedule are unchanged.
No new RTOS objects or command queues were added. All public motion requests
still take the same units and arguments. Rebuild all modules together because
the calibration-result C layout is smaller.

## Requirement trace

Reviewed against these workspace documents, not unrelated lecture files:

- `Change Requirement 1.docx`: minimum bits throughout application data;
  race-safe sharing; binary messages; RAM/message/correctness/timing evidence.
- `AAI07_ImmTri1.pdf`, pages 12-14 and 18: preserve wheel direction; pack
  shared motion results, retain internal control precision, derive encoder
  widths from maximum run, and retain Buddy 2 as the only motor owner.
- `AAI07_Week_6_Design_Review.pdf`, pages 3-7 and 10: encoder/PWM allocation,
  Motion Control task, motion statechart and A2/A3 encoder/slip assumptions.
- `Project reference files/Project Write-up.pdf`, page 6, and
  `Project Briefing.pdf`, page 5: reusable motion APIs, encoders, PID,
  straight correction, calibration and repeatable distance/turns.
- `Project reference files/briefing transcript.md`, motion-control section:
  independently regulate the two unequal motors and retain accurate turns.

The representation requirement does not prove physical accuracy. Actual motor
calibration, slip and encoder interrupt-load assumptions remain unvalidated.

## Stored fields

| Field and owner | Valid values / unit | Required bits | Representation |
|---|---|---:|---|
| Controller state | IDLE, VELOCITY, DRIVE, TURN, COMPLETE, FAULT | 3 | `g_bits.state` |
| Primary latched fault reason | none, stall, stale command, encoder, direction, timeout | 3 | `g_bits.fault`; no reason removed |
| Controller busy | false/true | 1 | `g_bits.busy` |
| Finite-move kind | drive/turn | 1 | `g_bits.turning` |
| Commanded wheel direction, each wheel | +1 / -1 | 1 each | reverse bits decoded to the original mathematical sign |
| Stall elapsed time, each wheel | 0..649 ms | 10 | bit-field; threshold remains 600 ms |
| Wrong-direction elapsed time, each wheel | 0..349 ms | 9 | bit-field; threshold remains 300 ms |
| Wheels-still elapsed time | 0..60049 ms | 16 | `uint16_t`; threshold and original overshoot preserved |
| Calibration requested/running/done | false/true each | 1 each | one mutex-protected byte |
| Snapshot-ready marker | false/true | 1 | same calibration byte; reporting only |
| Calibration result conditions | eight independent booleans | 1 each | one byte, same named members; six floats unchanged |
| Motor GPIO identifiers | fixed GP8..GP11 | 4 each | private configuration bit-fields |
| Motor polarity inversion | false/true | 1 each | motion-task-owned bit-field |
| Encoder GPIO identifiers | fixed GP4/5/16/17 | 5 each | private configuration bit-fields |
| Encoder inversion | false/true | 1 each | task-written, ISR-read configuration |
| Encoder A/B phase | 00, 01, 10, 11 | 2 each | separate ISR-owned byte object |
| Encoder IRQ core | 0/1 | 1 | initialisation-owned field |
| Shared reported wheel speed | -510..510 mm/s, 2 mm/s resolution | 8 magnitude + 1 reverse per wheel | explicit snapshot masks; same reporting convention as Buddy 1 |
| Shared motion state / primary reason | same six values each | 3 each | explicit snapshot masks |
| Shared busy / speed validity | false/true each | 1 each | explicit snapshot masks |

Timer proofs use `motion_process()`'s existing 2..50 ms dt clamp. A timer
below 600 or 300 ms can receive one final 50 ms increment before the existing
comparison, so allocating only for the threshold would be unsafe. The still
timer may similarly reach 59999 + 50 = 60049. Compile-time checks guard the
current thresholds; re-audit these widths if the dt clamp changes.

Function arguments, result enums and wheel IDs are decoded calculation/ABI
values, not new retained packet fields. Four manoeuvres require two logical
bits and five request-result codes three; no unused queue was introduced just
to store these values. CPU registers/stack alignment may occupy more space.

## Four-byte copied reporting snapshot

`motion_read_snapshot()` copies the two speed estimates, state, primary reason
and busy flag under the existing priority-inheritance motion mutex, releases
the mutex, then builds the packet. It does not call `motion_poll()`, consume
COMPLETE, command motors or change a PID target. During calibration/startup,
or for nonfinite/out-of-report-range estimates, speed validity is false.

| Bit positions, little-endian within four explicit bytes | Meaning |
|---|---|
| 0..7 / 8 | left magnitude in 2 mm/s steps / reverse |
| 9..16 / 17 | right magnitude in 2 mm/s steps / reverse |
| 18..20 | motion state |
| 21..23 | complete original primary fault reason |
| 24 / 25 | busy / speed-valid |
| 26..31 | zero byte-rounding padding, rejected if nonzero |

The format uses 26 meaningful bits, rounded to four bytes. It is an internal
copied-result contract, not a raw C bit-field layout. MQTT still uses Buddy 1's
separate versioned format; topics and payload lengths are unchanged.

Only reporting is quantised. Integer mm/s rounds to the nearest 2 mm/s,
ties away from zero, with at most 1 mm/s error. Conversion from the original
floating-point estimate first truncates to whole mm/s, as the previous
controller telemetry path did; combined reporting error is less than 2 mm/s.
The +/-510 mm/s reporting range is a protocol choice, not proof of the car's
maximum physical speed. An out-of-range value is invalid, not wrapped/clipped.
No reduced-precision value is fed back into PID, steering or finite moves.

## Fields deliberately not narrowed without a valid bound

| Field | Current representation | Reason / remaining evidence |
|---|---|---|
| Cumulative encoder count, last count and movement/stall references | signed 32-bit, aligned | Maximum run time/distance is not specified. Do not invent a shorter run or silently wrap. |
| Cumulative invalid-transition count and last seen count | unsigned 32-bit, aligned | No maximum lifetime/invalid-edge budget has been agreed; retain current modulo-difference behaviour. |
| Leaky glitch-bucket arithmetic | signed 32-bit | Current code adds an unsigned cumulative delta cast to signed, then caps/decrements. Packing the accumulator before that computation would truncate bursts. Its large-delta behaviour needs a bound/audit before narrowing; no algorithm correction is included here. |
| Microsecond clock stamps/deadlines | unsigned 32-bit | Existing hardware-counter wrap domain is 0..2^32-1; wrap-safe comparisons unchanged. |
| PID gains/integral/setpoint, estimated speed/acceleration, geometry, calibration and distance/angle/speed requests | 32-bit float | Preserve the existing control precision and accepted finite inputs. The documents do not define a narrower range/resolution that reproduces these calculations. |
| RTOS IDs and MMIO values | existing kernel/hardware types | Not compact application payload fields; do not change the platform ABI. |

Encoder bound to fill in once the team defines the maximum run:

`maximum absolute count = ceil(maximum wheel speed * maximum run seconds * counts per revolution / (pi * wheel diameter)) + margin`.

For a signed field, use `1 + ceil(log2(maximum absolute count + 1))` bits.
Current setup values 0.30 m/s, 2000 counts/revolution and 0.061 m diameter imply
about 3131 counts/s, but are not physical calibration evidence. Neither the
maximum run nor the margin is confirmed. Keeping 32-bit counts is a documented
open width decision, not a claim that ISR-updated values are exempt from the
requirement. Similarly, the float and glitch-bucket bounds remain review items
if stricter minimum-width acceptance is required. Do not claim complete
minimum-bit sign-off for these unresolved fields.

## Sharing and fault reporting

- The existing motion mutex covers control flags, calibration flags and copies
  of calibration results. `volatile` is not used as a substitute for locking.
- ISR A/B state and task-written inversion are separate nested storage objects.
  Arm disassembly confirms a state-byte write at offset 2 and an inversion
  halfword write at offset 0. Neither packed update overwrites the other.
- Counts/glitch totals stay aligned 32-bit, with the ISR as the sole writer.
  Existing individual getters do not promise an instantaneous two-wheel pair.
  This update adds no new count reset or multiwriter counter.
- The controller's reporting adapter sets/clears only Buddy 1's existing stall
  and encoder mask bits. Wi-Fi/IMU/no-echo bits are preserved by the central
  masked update. Packing/network calls never run from the encoder ISR.
- All five original fault reasons remain available through `motion_get_fault()`
  and the three-bit snapshot reason. Stale command, wrong direction and timeout
  are not silently relabelled as stall/encoder. The current common five-bit
  MQTT fault schema has no dedicated bits for them; extending that schema or
  adding a diagnostic topic still needs team agreement. The mirror represents
  the existing primary latched reason, not a newly invented all-fault detector.
- The controller continues sending full-precision authorised motion requests.
  Quantising command speeds would change steering/control and is deliberately
  outside this representation-only update. A future copied command format must
  preserve the required precision before replacing this interface.

Measured sizes, regression evidence and remaining physical checks are in
`testing/subsystem2/EVIDENCE.md`.
