# Optional wireless IR preview, version 1

`IR_DEBUG=1 S3_MOTION=1 MQTT=1` enables observation of the existing three-channel
scan; default is off. It does not change motor/PID, line following, calibration,
barcode decoding, start/stop or mapping decisions. Buddy 1 telemetry protocol v3
and its caches/commands remain unchanged.

Topic: `car/debug/ir`. Exactly **16 binary bytes**, QoS 0, retain 0. Byte 0 is
version 1. Following fields use least-significant-bit-first packing in the
remaining 15 bytes; offsets below are relative to that body, not byte 0.

| Field | Body bit offset | Width | Meaning/range |
|---|---:|---:|---|
| Source clock | 0 | 32 | MCU microseconds, unsigned wrapping clock |
| Raw readings | 32, 44, 56 | 12 each | ADC 0–4095 |
| Normalized darkness | 68, 78, 88 | 10 each | 0–1000, not 1023 |
| Raw validity mask | 98 | 3 | Bit 0 left, 1 right, 2 barcode |
| Normalized validity mask | 101 | 3 | Same order; must be a subset of raw validity |
| Surface classifications | 104, 106, 108 | 2 each | 0 unknown, 1 white, 2 black; 3 reserved |
| Preview sequence | 110 | 8 | Increments for generated previews, wraps at 256 |
| Padding | 118 | 2 | Must be zero |

All repeated fields use **left GP27, right GP26, barcode GP28** order. This is
not the internal `s3_sensor_id_t` enumeration order. Raw-invalid readings must be
encoded as zero. Normalized-invalid readings/classifications must be zero.
Decoders expose invalid values as unavailable, never as measured zero. Raw can
be valid while normalization is unavailable. Normalized validity/classification
is not an independent calibration-acceptance flag.

The C codec uses explicit shifts/masks, not C struct layout. 12/10/2-bit fields
are the minimum widths for their agreed numeric ranges. Sequence/clock widths
define wrap behaviour and are transport metadata. No ADC precision is reduced;
the preview selects at most one existing scan every 50 ms. All raw samples used
by the actual algorithms are unchanged.

## Ownership, load and loss

- The existing `s3-motion` sampling owner builds the frame after its ordinary
  processing, at most 20 times per second. No extra ADC read, task, kernel object
  or peripheral owner is added.
- One 16-byte latest-only mailbox shares the existing status mutex. The network
  owner polls it with `TMO_POL`, and drops samples older than 100 ms. Packing is
  outside the mutex; bounded copying occurs inside it.
- The existing Wi-Fi/MQTT owner remains the only network caller. Normal outbox
  messages have first priority, BCAP recorded upload second, the optional
  controller snapshot third and live IR last.
  A failed preview publish is discarded, never retried as stale history. The
  existing stable diagnostic transmit buffer and one-inflight guard are reused.
- Maximum IR application payload is 320 bytes/s; this excludes topic/MQTT/TCP/Wi-Fi
  overhead. A rate cap is not a guarantee of delivery or deadline compliance.
- Sequence gaps include mailbox replacement, publish rejection, disconnection
  and receiver loss. The dashboard labels these skipped/not observed, not solely
  network loss. Gaps above 250 ms break plotted curves. Missing samples are never
  interpolated into the exported data.
- The laptop holds at most 1200 frames, marks data stale after one second or a
  broker disconnection, and preserves last-known values. It unwraps ordinary
  MCU clock rollover. A backward/discontinuous clock clears the plotted window;
  this is a source-clock discontinuity, not proof of a detected physical reboot.

This best-effort stream cannot show every narrow barcode edge. Use the existing
higher-rate BCAP upload for detailed decoding analysis. On-car control timing,
stack use and Wi-Fi behaviour with the stream enabled remain hardware checks.

## Controller/buddy debug snapshot, version 1

28 October integration update: reason 10 is terrain/IMU fault; 11–15 remain
reserved. Firmware, independent laptop decoder and the UI agree on this value.
The packet remains 70 bytes, including the existing 4-bit reason field. Live
previews remain optional (`IR_DEBUG=1`), latest-only and below ordinary telemetry.

The same optional build also publishes `car/debug/controller`, exactly **70
bytes**, QoS 0, retain 0, capped at **5 Hz**. Byte 0 is version 1; the body
contains 551 least-significant-bit-first bits and one zero padding bit. The
ordered field contract is `CONTROL_DEBUG_FIELDS` in `control_debug.h`, with an
independently written laptop schema in `tools/dashboard/control_protocol.py`.
No C struct or float is put on the wire.

| Field group | Representation / validity |
|---|---|
| Sampling/source clocks; preview sequence | Two wrapping 32-bit microsecond clocks, one 8-bit sequence |
| Motion availability, validity, brake acknowledgement, calibration busy | One bit each; busy/contended/stale feedback must not be plotted as zero |
| Motion state/fault | 3 bits each, every existing code preserved |
| Ramped PID setpoints and measured speed, both wheels | Four signed 13-bit two's-complement integers, 1 mm/s resolution; diagnostic envelope ±4095 mm/s |
| Last signed motor duty, both wheels | Signed 8-bit integers, −100..100 percent (1% resolution) |
| Encoder snapshots, both wheels | Signed 20-bit two's-complement counts, existing −524288..524287 modular range |
| Controller state/reason/pending action | 3/4/3 bits; existing enum codes, no new decisions |
| Line error and observation flags | Signed 8-bit −100..100 plus 5 bits: valid, present, junction candidate, touching, rearming |
| Last non-NONE barcode outcome | 3 bits; retained for observation, not a new decode result |
| Accepted calibration / readiness blockers | Existing 3-bit internal sensor mask and 5-bit readiness mask |
| Barcode/junction/line-lost event counts | Three 16-bit saturating counters since boot, updated at the original sampling cadence |
| Last sampling gap / run maximum gap | Two 32-bit microsecond values; zero run maximum means none recorded yet in the UI |
| Active sensor calibration references | Left/right/barcode: raw white12/black12, normalized white-limit10/black-limit10 |
| Current/last collection | Internal sensor ID2, collecting1, raw min/max12 for each surface, sample-count16 each |

The internal calibration sensor IDs/mask bits are **barcode 0, left 1, right 2**;
this differs from the IR preview channel order. The browser labels them by name.
Normalized limits are 0..1000. Accepted flags are separate from configured
references, which can otherwise be development defaults. White/black observed
collection ranges are distinct from active averaged calibration references.

`motion_try_debug_snapshot()` tries the existing Buddy 2 mutex with `TMO_POL`.
It only reads the ramped PID setpoint, speed estimate, motor-duty cache, encoder
cache and state. It never reads a PIO FIFO, consumes a motion report, advances
the statechart, resets a PID or writes a motor. The sampling owner then packs
the snapshot outside the shared status mutex. Motion data older than 50 ms,
busy calibration or values outside the diagnostic envelope are unavailable.
The packet's state/availability metadata remains distinct from speed validity.

A separate 70-byte latest-only mailbox uses the existing status mutex. The
network reader never waits, drops controller snapshots older than 400 ms and
never retries rejected previews. Together the two diagnostic streams cap
application payload at **670 bytes/s** (320 IR + 350 controller), excluding
all protocol overhead and ordinary telemetry. Normal telemetry and recorded
barcode uploads still have priority. No task, queue or kernel object was added.

The controller reason is sampled, not an exhaustive event log. In particular,
reason 0 means OK; it cannot distinguish a fresh boot from an operator stop.
Event counts preserve brief barcode/junction/lost edges between snapshots but
do not give their exact timestamps. The dashboard uses source-clock plots for
controller data and separately labels laptop receive-time telemetry histories.
Cached snapshots are never inserted as new measurements. MCU RAM/stack/queue
metrics, PID component terms, stop latency, pitch/roll and paired scan-angle
samples are not supplied by this packet.

## Verification commands

```sh
make -C testing/integration -f controller_test.mk test
make -C testing/subsystem1 test
python3 -m unittest discover -s testing/dashboard -v
node --check tools/dashboard/static/sensor_graphs.js
```

Tests include independent C/Python wire vectors, physical channel ordering,
valid zeroes, reserved bits, wrap, stale/contended mailbox behaviour, normal
telemetry/BCAP priority, rejected publish and bounded laptop history. Isolated
MQTT tests use a disposable broker and do not communicate with the car.

## Local verification, 28 October 2026

- Dashboard: 41 tests passed, including C/Python byte equivalence and isolated
  real-MQTT sensor delivery. JavaScript syntax and Python compilation checks pass.
- Controller suite, including the diagnostic-enabled production sampling runtime
  and course simulation: passed under AddressSanitizer/UndefinedBehaviorSanitizer
  at `-O2`. Apple Clang 21 crashes compiling the new codec with sanitizers at
  `-O1`; this is a compiler failure, not a passing test. The codec also passes
  both sanitizers at `-O0`. The full test harness cannot link at `-O0` because
  otherwise-unreferenced reporting callbacks require additional fakes.
- Buddy 1 IR-enabled network/outbox suite: passed under both sanitizers. Buddy 2
  and Buddy 3 host regression suites also pass. Buddy 2 adds read-only accessors;
  its equations/state transitions, `speed_pid.c`, shared `s3_drive.c` and Buddy 3
  algorithm files are unchanged. Repeated diagnostic reads during idle and a
  turn preserve motor output and feedback; contended reads return immediately.
- Pico W diagnostic course firmware: built without warnings. Linker totals are
  text 400032, data 152, BSS 102544 bytes. These are whole-image totals, not the
  feature's incremental cost or runtime free heap/stack measurements.
- The same course build with `IR_DEBUG=0` also builds without warnings: text
  396080, data 152, BSS 102192 bytes. Enabling both diagnostic streams therefore
  adds 3952 bytes of linked text and 352 bytes of static BSS in these profiles.
  This is not a measurement of transient MQTT buffers, free heap or stack margin.
- UF2 SHA-256: `85b58d76b5db1e50194cc3a9e9a63460ed1fcb946f8bbbbb1c774575e855ebe9`.
- Browser checks: generated three-channel waveforms, raw/normalized switching,
  pause while reception continues, and stale last-known values on disconnection.

Firmware was **not flashed** in this verification. Physical sensor correspondence,
radio load, motion deadlines and stop latency with the preview enabled remain
on-car validation, not established by the simulation or host tests.
