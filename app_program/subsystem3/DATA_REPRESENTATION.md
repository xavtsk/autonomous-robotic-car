# Buddy 3: Tribunal 1 data representation

## 27 October observation extension

The internal shared status is now **146 bytes** (60 navigation + 22 feedback +
64 metadata). Six run-timing fields add exactly 192 bits to the previous 317-bit
metadata: start timestamp, samples, maximum gap, above-target and above-safety
counts, and latest safety-gap timestamp. Metadata uses 509 bits with three zero
padding bits. Timestamps retain the full uint32 microsecond wrap domain; counters
saturate at UINT32_MAX rather than wrap. No smaller maximum run length has been
agreed. These are diagnostics, not newly quantised control values. The 11-byte
control packet, navigation codec and external MQTT v2 protocol are unchanged.
The earlier sizes below are retained as historical measurements.

## Current integration, 25 October 2026

The teammate's `71193e2` navigation API adds readiness/lifecycle, persistent
faults, accepted calibration, diagnostic steering, rearming and exact capture
distance/epoch information. All are retained by the extended codec: **479 bits
in 60 bytes**, one zero padding bit. Raw frames remain 266 bits/34 bytes;
barcodes remain 38 bits/5 bytes. Numerical steering/calibration floats and
capture-distance binary64 doubles are exact, not newly quantised.

The single `s3_integration` owner stores an explicit **122-byte status packet**
(60 navigation + 22 feedback + 40 metadata) and **11-byte pending command**
(3-bit kind, 2-bit sensor, 26 three-bit map entries). Status metadata has 317
used bits and three zero padding bits; the command has 83 used bits/five zero
padding bits. Enums/ranges are validated before packing. Snapshot/STOP/pending
flags are protected by the existing mutex. ADC acquisition, controller calls,
telemetry and printing remain outside that lock.

The former separate controller mailbox/task is replaced, not duplicated.
Stationary bench shared raw/navigation/last-barcode values also use these
codecs. Decoded working views stay private and wide for validation/calculation;
private boolean/small-mask state remains compact. Host short-enum sizes are
128/60 bytes for decoded/packed navigation, 52/34 raw frame, 8/5 barcode,
488 bytes for the new private navigation context, and 18 bytes per calibration
collector. These host sizes are not a complete target RAM or stack measurement.

Current regression compares against `71193e2`, not the pre-pull core. See
[integration review](../../docs/INTEGRATION_2026-10-25.md) and
[controller ownership](../controller/README.md). Correct-original-line identity,
physical calibration and deadline measurements remain open.

## Historical 23 October representation-only design

The older 234-bit/30-byte mailbox and context figures below describe the
superseded navigation API; they are retained as history, not current sizes or
current shared-task architecture. New readiness/odometry data must not be dropped
merely to preserve an earlier packet size.

Scope: minimum-bit storage and the existing Buddy 3 → Vehicle Controller →
Buddy 2 handoff. Sensing, calibration, hysteresis, line/steering calculations,
Code 39 decoding, event order and movement decisions are unchanged. Buddy 2
remains the sole motor-output owner. Two IR sensors detect the line; the third
reads barcodes.

This implements the representation concerns in **Change Requirement 1** and
**AAI07_ImmTri1.pdf** (Buddy 3 raw precision/validity, shared-data ownership and
boundary/timing validation). It does not claim the whole robot is ready for the
physical demo. Buddy 4/5 integration and correct-original-line recovery remain
separate work.

## Values and widths

| Value | Existing valid range/meaning | Stored bits | Reason |
|---|---|---:|---|
| ADC count / white and black references | 0..4095 | 12 each | All ADC detail retained; never just black/white |
| Normalised darkness / classifier thresholds | 0..1000 | 10 each | Integer resolution unchanged |
| Surface | UNKNOWN, WHITE, BLACK | 2 | UNKNOWN cannot be represented by black/white alone |
| Sensor enable/valid and other conditions | false/true | 1 each | Validity remains separate from numeric zero |
| Enabled/seen/fault sensor masks | three sensors | 3 each | Independent channel information retained |
| Events | barcode, junction, lost, reacquired, sensor fault | 5 | All events may coexist |
| Signed line error | -100..100 | 8 | Packet stores error + 100; no rounding |
| Barcode status | five existing outcomes | 3 | Invalid codes rejected |
| Barcode letter | none or A..Z | 5 | Internal copied format: 0 none, 1..26 letters |
| Barcode orientation | forward/reversed scan | 1 | Not wheel direction |
| Development map action | none/left/right/straight/U-turn | 3 | All 26 entries retained in 10 bytes |
| Barcode run count | 0..29 | 5 | Existing overflow guard preserved |
| Calibration sample count per label | 0..65535 | 16 | Same cap and rejection behaviour |
| Calibration sum | 0..268365825 | 28 | 65535 × 4095; no overflow or precision loss |
| Calibration minimum/maximum | 0..4095 | 12 each | Exact extrema retained |
| ADC channel selector | 0..2 | 2 each | Same pins and channel order |
| Processing mode | TRACK/SUPPRESSED | 1 | Same transitions |
| Configured line/sample-age and barcode frame intervals | validated below 2^31 microseconds | 31 each | Full accepted configuration range, not only defaults |
| Barcode gap/narrow/run configuration | at most 214748364 microseconds | 28 each | Existing quiet ≥ 10 and run × quiet < 2^31; gap/narrow ≤ run |
| Barcode quiet-zone modules | 10..50 | 6 | All accepted settings retained |
| Steering request / gains and limits | existing binary32 values | 32 each | No agreed quantisation change; copy exact float bits, preserve control behaviour |
| Timestamps / measured elapsed values | uint32 wrap domain | 32 each | Keep wrap handling, large gaps and rejected-input diagnostics |
| Driver error / benchmark counters | full signed error code / existing uint32 count | 32 each | No justified smaller error set or run-length bound |

Public inputs and decoded calculation views deliberately stay wide. Validate
4096 as invalid **before** putting an ADC value into 12 bits: truncating first
would turn it into a false healthy zero. The retained configuration is packed
only after the same original validators succeed. Calibration arithmetic still
uses sufficiently wide intermediates.

Full-width working values are not a claim that every field can never be made
smaller. A later numerical-resolution or maximum-run agreement would need its
own validation and may change behaviour; it is not silently invented here.

## Copied formats

`navigation_codec.c/.h` explicitly writes least-significant bit first. It does
not copy a C struct or compiler-dependent bit-field layout as a packet.

- Navigation result: **234 bits in 30 bytes**, previously a 72-byte decoded
  struct. Includes exact steering floats, all event/validity fields, sensor
  detail and timestamps. Six final padding bits must be zero.
- Pending barcode: **38 bits in 5 bytes**, previously an 8-byte decoded result.
  Last two padding bits must be zero.
- Existing stationary bench raw-frame snapshot: **266 bits in 34 bytes**,
  previously 52 bytes. All channel timestamps, 12-bit counts, signed driver
  errors and flags survive. Six padding bits must be zero. This does not add
  a normal-mode raw queue.

These are internal interfaces within one compiled firmware, not new MQTT
topics. Their letter coding deliberately differs from Buddy 1's A=0 MQTT
coding; the controller uses decoded letters at that boundary. Both sides use
the same codec. Invalid ranges, enum codes and padding are rejected without
modifying the caller's destination.

## Ownership and workflow

1. Sampling task acquires the same ADC frame and runs the same algorithms.
   Runtime context/mode access uses the existing navigation mutex.
2. Controller submission validates and packs its task-private result before
   taking the mailbox mutex. The mailbox stores 30 bytes, five sticky events
   and a separately preserved pending barcode.
3. The controller copies/clears pending work under that mutex, then decodes
   its own copy after unlocking. It still forwards exact steering requests,
   remembers the mapped action until a junction and calls Buddy 2 as before.
4. The optional stationary bench likewise encodes before its snapshot lock
   and decodes the reporter's private copy afterward. ADC and console work
   stay outside that lock. Diagnostic output is unchanged.

Private retained contexts use GCC/Clang packed bit-fields; target sizes are
measured rather than inferred by adding logical bits. They must never be
modified concurrently without their existing owner/lock. No new task, queue,
timer, heap allocation or mutex was introduced. Redundant wide global config
storage was removed; startup uses a temporary validated input and retains the
compact copy.

## IR fault reporting and compatibility

The controller now mirrors Buddy 3's current nonzero three-channel fault mask
into **Buddy 1 fault bit 5**. The original five common fault meanings remain
unchanged. Updates own only the IR bit, under Buddy 1's existing mutex; they
cannot clear Wi-Fi, encoder or other faults. Motor stop occurs before network
fault reporting on the existing sensor-fault path.

The aggregate bit describes the latest IR condition, not fault history. If a
fault event is followed by a healthy sample before consumption, the sticky
event still forces the existing latched controller FAULT/stop; the current IR
bit may be clear. A clear bit cannot restart the robot.

Buddy 1 and the laptop codec now use **MQTT protocol version 2** together.
Snapshot and heartbeat byte lengths are unchanged. Version 1 is rejected;
upgrade both endpoints. See `../subsystem1/MQTT_PROTOCOL.md`.

## Evidence and remaining checks

- Portable behaviour, packet boundaries and bench diagnostic regression:
  `testing/subsystem3/EVIDENCE.md`.
- Actual controller-task tests: `testing/integration/controller_test.mk`.
- Full firmware profiles and measured RAM/code trade-off:
  `testing/subsystem3/BUILD_AND_RAM.md`.

Actual-track calibration, moving barcode success, real sampling jitter/WCET,
task stack margin and concurrent hardware load remain unmeasured. The local
start input is still not wired into the application. No automatic start,
navigation rewrite, Buddy 4/5 behaviour or original-line identity logic was
added by this representation update.
