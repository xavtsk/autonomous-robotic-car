# Change Requirement 1: Minimum-Bit Data Representation, Week 13 evidence

## GitHub submission after the demo, 27 November 2026

| Submission item | Evidence |
| --- | --- |
| Field ranges, required bits and implementation | The [submission table](compliance/CR1_SUBMISSION_WIDTHS.md) verifies 167 selected fields and 19 layout totals. The [broader inventory at its original revision](https://github.com/xavtsk/autonomous-robotic-car/blob/a3de92a3465749f962540bcb75c275959e1a9ef0/docs/compliance/CR1_FIELD_INVENTORY.md) records which declarations had no verified width or ownership decision at the time of review. |
| RAM before and after | The integrated Arm builds below measure the effect of the final drive change. Earlier tables identify their own source and build profile. |
| Message sizes before and after | The two new fields are private to the drive controller. MQTT and inter-task payloads keep their existing layouts. The [course-test comparison](#course-test-updates-after-8bc969c) gives the measured sizes of messages changed earlier. |
| Correctness and performance | The Buddy 3 host suite and controller integration suite passed. All 25 simulated course cases produced identical output before and after the drive change. The team has dismantled the car after its demo. The final revision has no on-car interval or recovery measurements. |

### Line recovery fields and matched firmware builds

During line recovery the sampling task stores which way the car is sweeping
and whether it is on the return sweep. Each has two values. The drive state
now stores each in one unsigned bit alongside its existing search-active
flag. The host course comparison includes left and right turns with both
undershoot and overshoot. [Drive implementation](../app_program/controller/s3_drive.c) ·
[Software test commands](BUILD_AND_TEST.md)

| Arm measurement | Before (`5b7b542`) | Submission cleanup |
| --- | ---: | ---: |
| Two drive fields | 8 bits each | 1 bit each |
| Whole drive state object | 10,840 B | 10,840 B |
| Linked static RAM | 134,204 B | 134,204 B |
| GNU `size` text | 470,296 B | 470,312 B |

Both fresh builds used Arm GNU 14.3.1, the same pinned SDK, public example
network settings and the [24 November course profile](compliance/evidence/COURSE_FIELD_LAYOUT.json)
with IR and IMU diagnostics output enabled (`IR_DEBUG=1 IMU_DEBUG=1 DIAGNOSTICS=0`).
The only firmware source changes were in `s3_drive.c` and `s3_drive.h`.
Alignment kept the drive allocation unchanged. Each image used 192 B for
vectors, 696 B for data and 133,316 B for BSS.

| Integrated UF2 | SHA-256 |
| --- | --- |
| Before | `8d37796eb96feb9b4ed25b6bdaf3ae2b47fc12bd4279fcb754c5dbd26df06af0` |
| Submission cleanup | `2134337c27634e50be9fb14230c0b71808b270e2acb3d231d8c59e319a745d9b` |

### Recorded barcode regression

Replaying the saved sensor waveforms against the deployed baseline
(`4ad2cc4`) and the submission code gave identical output in all 1,008
cases. Two of 3,110 assertions failed in both versions. With two seconds of
synthetic leading white and no trailing extension, one Z recording ended
before the decoder returned a letter. One normal-speed recording returned A
where the fixture expected silence. The audit retains both failures.
[Replay results](compliance/evidence/post-demo-barcode-replay-20261127.json) ·
[Original recordings and expectations](../testing/subsystem3/fixtures/barcode/README.md)

## Course-test updates after 8bc969c

The terrain and obstacle changes add a roll history, scan progress fields
and an optional face-width interval. The [current width table](compliance/CR1_SUBMISSION_WIDTHS.md)
checks the packed fields and selected source declarations. These entries
supplement the earlier evidence below.

| Field | Range and resolution | Representation and ownership |
| --- | --- | --- |
| Terrain odometry pivot flag | False or true | 1 bit. Buddy 4 sets it in its task-local view of the motor feedback. |
| Terrain monitor roll-history flag | False or true | 1 bit. The terrain monitor's owner updates it with the history. |
| Terrain monitor motion stability count | 0..3 samples, including reset and saturation | 2 bits in the terrain monitor. |
| Two prior right-wheel roll samples | −90..+90 degrees, floating-point resolution | Two 32-bit floating-point values in the terrain monitor. The median and hump thresholds use the existing float precision. Integer angles would change comparisons near the thresholds. |
| Obstacle scan count | 0..16 samples, including the full buffer | 5 bits in the scan state. Buddy 5 owns it. |
| Routing sweep complete | False or true | 1 bit in the scan state. The code derives the routing count from the completed sweep and its nearest coarse bearing. It returns 0 before completion, 9 when the fine sweep is clipped at 30° or 150°, and 12 otherwise. |
| Face-width probe index | 0..4, including the end of the probe list | 3 bits in the scan state. |
| Face-width lower and upper bounds, Buddy 5 snapshot | 0..4095 mm in whole millimetres. Both zero means unavailable. | 12 bits each. Buddy 5 publishes the encoded snapshot under its existing mutex. |
| Face-width lower and upper bounds, MQTT scan v4 | Lower 0..4095 mm, upper 1..4095 mm in whole millimetres | 12 bits each. Scan v3 carries the original fields when the width interval is unavailable. |
| Face-width bounds in the C working and codec input views | 16-bit inputs may contain 0..65535. Codecs accept bounded intervals only. | Two `uint16_t` values per view. Validate before packing so invalid inputs can be rejected without truncation. |

| Encoded application message | At `8bc969c` | Current | Change |
| --- | ---: | ---: | ---: |
| Buddy 5 inter-task obstacle snapshot | 182 bits / 23 B | 206 bits / 26 B | +24 bits / +3 B |
| MQTT scan without face-width bounds | 95 body bits / 13 B | 95 body bits / 13 B | 0 B |
| MQTT scan with face-width bounds | 95 body bits / 13 B for the previous scan layout | 119 body bits / 16 B | +24 bits / +3 B |

MQTT byte counts include the version byte and byte-alignment padding. The
extended scan fits the existing MQTT payload buffer. Telemetry uses the extra
width readings to report the measured interval. The bypass uses the padded
routing estimate.
The laptop estimates turning rate from received wheel speeds, so that new
graph and log add no firmware fields or MQTT bytes.

### Matched course firmware measurements, 24 November 2026

All three builds used Arm GNU 14.3.1, the same Pico SDK revision, the public
example Wi-Fi settings, and the integrated course profile with both IR and
IMU debug enabled. Static RAM includes the vector table, data and BSS.

| Source | Static RAM | GNU `size` text |
| --- | ---: | ---: |
| Last push, `8bc969c` | 134,192 B | 467,812 B |
| Course changes, before this cleanup | 134,204 B | 470,048 B |
| Course changes, after this cleanup | 134,204 B | 470,296 B |

The cleanup shrank the terrain task's local monitor from 232 to 224 bytes
with IMU debug enabled, and from 216 to 208 bytes with it disabled. The
task keeps its existing stack allocation. The scan object stays at 60 bytes
after alignment. The linked firmware uses the same static RAM as the
pre-cleanup course build and 248 more text bytes.
[Arm layouts, build settings and image hashes](compliance/evidence/COURSE_FIELD_LAYOUT.json)

The Buddy 1, 4 and 5 Linux test suites passed on both the pre-cleanup course
code and the candidate. All 42 emitted MQTT vector lines matched byte for
byte. The hump fixtures and simulated bypass routes produced the same
results. The optional simulation with turns 10% shorter than commanded
still reports its known miss. The only log difference was the Python
runner's elapsed time. The scan fixtures now also check the derived routing
count at each coarse bearing. This comparison covered simulated behaviour
and message encoding. It included no physical runs.

## Phase 3 submission results

The final packing batch covers the drive flags and terrain reporting cache.
The sampling task owns both records during a run. Startup initializes them
before it starts sampling. The terrain cache still updates only after the
associated report is accepted. See the [current width table](compliance/CR1_SUBMISSION_WIDTHS.md)
for named fields, ranges and codec totals.

| Retained fields | Valid range | Required bits | Representation before → after |
| --- | --- | ---: | --- |
| Drive `b_has_external_hold`, `b_is_configured`, `b_waits_for_stop`, `b_is_advancing`, `b_has_touched_line`, `b_is_searching` | False or true | 1 each | Byte-sized Booleans → unsigned one-bit fields |
| Drive `b_has_obstacle_snapshot`, `b_is_bypassing` | False or true when Buddy 5 is enabled | 1 each | Byte-sized Booleans → unsigned one-bit fields |
| Drive `reset_result` | 0..3, including idle and failure | 2 | Two-bit field moved into the same group |
| Terrain `sent_height` | 0..500 mm in whole millimetres | 9 | `uint16_t` → unsigned nine-bit field after existing validation |
| Terrain `sent_fault` → `b_sent_fault` | 0..1 | 1 | `int` → unsigned one-bit field |
| Terrain impact latch `b_has_unreported_impact` | False or true | 1 | Separate Boolean → unsigned one-bit field in the cache |

Wrapping microsecond timestamps retain 32 bits. Run tokens and the saturating
hump count retain 16 bits. Public validation inputs and floating-point sensor
and control arithmetic keep their existing types. The dashboard still uses
version 3 for requests and status and version 4 for replies.

### Matched firmware RAM

The two linked images use Arm GNU 14.3.1, the pinned Pico SDK and the same
production profile with public example network settings. “Before” is the
Phase 2 closeout immediately before this batch. Static RAM is `.data + .bss`.
It excludes live stack usage.

| Arm measurement | Before | Phase 3 | Change |
| --- | ---: | ---: | ---: |
| Static RAM | 129,496 B | 129,480 B | −16 B |
| `.data` | 696 B | 696 B | 0 B |
| `.bss` | 128,800 B | 128,784 B | −16 B |
| Drive state object | 10,840 B | 10,832 B | −8 B |
| Terrain cache and separate impact latch | 61 B | 56 B | −5 B |
| `.text` | 465,756 B | 465,748 B | −8 B |

The object measurements exclude padding between objects. The linked totals
include that padding. Build inputs, commands and hashes are in the
[measurement receipt](compliance/evidence/phase3-submission-20261122.json).

### Matched message sizes

An Arm compile probe measured the existing C message types and encoded sizes
with each image's production flags. MQTT sizes count application bytes.

| Message | Before | Phase 3 |
| --- | ---: | ---: |
| Buddy 1 outbox slot | 18 B | 18 B |
| Buddy 1 command slot | 5 B | 5 B |
| Buddy 1 maximum telemetry payload | 17 B | 17 B |
| Motion snapshot | 4 B | 4 B |
| Navigation result | 60 B | 60 B |
| Terrain event | 12 B | 12 B |
| Obstacle snapshot | 23 B | 23 B |
| Dashboard request | 14 B | 14 B |
| Dashboard status | 14 B | 14 B |
| Dashboard reply | 10 B | 10 B |
| Controller debug packet | 70 B | 70 B |

### Correctness and timing evidence

One offline Linux comparison matched all 16 behaviour and wire traces
(80,959 rows) and 1,008 barcode replays against `4ad2cc4`. The runner still
reported the same two inherited replay expectation failures. The width
checker passed for 155 selected fields and 18 codec totals. All 14 width
and timing-analysis unit tests passed.

The baseline and candidate timing-counter fixtures both passed nominal
period, threshold, rollover and saturation checks. The integrated IR and
motion periods remain 5 ms and 10 ms. The Arm terrain polling function grew
by 20 code bytes. These checks cover software timing logic and code size.
The comparison recorded no on-car cycle times or end-to-end timings.
[Verification receipt](compliance/evidence/phase3-submission-20261122.json) ·
[On-car timing check](PHYSICAL_VALIDATION.md#three-minute-timing-and-reconnection-capture)

## Earlier Week 13 evidence

> Historical record: this body describes its original revision/session. Use the [current documentation index](README.md) and [baseline status](CURRENT_STATUS.md) for later changes and evidence.

Team AAI07. Historical measurements below identify their original snapshots;
they are not evidence that every test was executed on the current revision.
The latest compliance cleanup starts from `f684e0c` and keeps the drive,
barcode, terrain and avoidance decisions and tuning unchanged.
[How to regenerate](#how-to-regenerate) is at the end.

## 19 November compliance cleanup

The same-profile Arm builds measure **129720 B before** and **129696 B after**
the completed cleanup: **24 B less static RAM**. Text grows from
465756 B to 466260 B; `.data` stays 688 B. Both use 200 Hz IR sampling,
0.10 m/s cruise, Wi-Fi/MQTT, S3 motion, S4, S5, and IR debug; IMU debug and
diagnostics are off. These figures include the same local Wi-Fi settings.
The expanded barcode capture object is **32736 B**, not the historical
17336 B. Static RAM is `.data + .bss`, not stack high-water or free heap.
The representation-only intermediate image used 129688 B (32 B less); the
subsequent module renaming changes sorted link order and adds 8 B of alignment
padding. The final figure above includes that padding.

| Data changed | Valid values | Required bits | Current storage |
|---|---|---:|---|
| Remote reply action | 1..3 | 2 | Bits 0..1 of reply byte 1 |
| Remote reply result | 0..3 | 2 | Bits 2..3 of reply byte 1 |
| Remote reply reason | 0..7 | 3 | Bits 4..6 of reply byte 1 |
| Buddy 4 event queue head | 0..7 | 3 | Protected bit-field |
| Buddy 4 event queue count | 0..8 | 4 | Protected bit-field; full state fits |
| Queue, mailbox and barcode workspace flags | false/true | 1 each | Boolean bit-fields |
| Remote mailbox head/count | 0..5 / 0..6 | 3 each | Protected bit-fields |
| Buddy 5 relative heading / aim | -90..90 / -40..40 degrees | 8 / 7 | Unsigned fields with explicit signed decoding |

The remote reply is now **10 B instead of 12 B**. Its reply-only version 4
preserves IDs, epochs and result meanings. Commands and status remain version
3, and the updated dashboard accepts both old and new replies. Restart the
dashboard before using the new firmware; no automatic flashing is performed
by these checks. See [the protocol](REMOTE_CONTROL_PROTOCOL.md).

The width check covers **143 declared fields and 18 totals**, with selected
source bindings. It now includes the terrain packet's two safety bits and
checks the actual 95-bit packing sequence. It does not prove that every field
in every application structure has been inventoried.

The normal target build passes. Naming and formatting were separately checked
against saved intermediate Arm images: loadable flash contents were identical
before and after the naming and formatting passes. Final module/function
renames also produce identical flash contents when linked in the original
object order. The normal final build has a different file order and addresses.
These comparisons do not claim that representation changes themselves produce
identical machine code. See the
[verification record](../testing/change_requirement_1/logs/2026-11-19_compliance_checks.json).

On 19 November, Windows Application Control allowed the host executables to
compile and link but blocked execution on that machine. The final host and
replay results appear in the [submission summary](#github-submission-after-the-demo-27-november-2026).

The change requirement asks for four things in the repository:

| # | Required evidence | Where |
|---|---|---|
| 1 | Data representation table (field, valid range, bits, how stored) | [Section 1](#1-data-representation-table) + per-buddy tables |
| 2 | RAM usage before and after | [Section 2](#2-ram-usage-before-and-after) |
| 3 | Message size before and after | [Section 3](#3-message-size-before-and-after) |
| 4 | No loss of function, no unacceptable timing cost | [Section 4](#4-correctness-and-performance) |

## 1. Data representation table

**Rule used by every buddy.** First fix the valid range and resolution, then
count the distinct values, then use `ceil(log2(values))` bits. Example: servo
angle 0..180 degrees in 1 degree steps = 181 values = **8 bits**; pitch
-90.0..+90.0 in 0.1 degree steps = 1801 values = **11 bits** (two's complement).
Validity is always a separate bit: zero never means "invalid".

**Automated check.** [`check_widths.py`](../testing/change_requirement_1/check_widths.py)
recomputes the minimum width for an explicit field inventory. It compares
topic totals with the laptop codec and selected declarations/packing calls
with C source. Its result covers the fields listed for that revision.
Historical result before the latest cleanup: **103 fields; 17 totals matched**
([log](../testing/change_requirement_1/logs/2026-11-19_widths.txt); 86 fields on 14 Nov, 7 added
for the 17 Nov bypass state, 10 on 19 Nov: Buddy 4 turn rate + valid and
lifted/overturned, 7 for the new `car/telemetry/scan_sample` topic).

**Added on 19 Nov 2026** (readings that were USB-only, now on Wi-Fi):

| Field | Range | Bits | Why this width |
|---|---|---:|---|
| Buddy 4 compass turn rate (+ valid bit) | -360..+360 deg/s, 1 deg/s | 10 + 1 | 721 values = 10 bits; same width as Buddy 4's own packet |
| Buddy 4 lifted / overturned | none, lifted, overturned | 2 | 3 values = 2 bits |
| Buddy 5 raw reading: angle | 30..150 deg, 5 deg steps | 5 | 25 values = 5 bits |
| Buddy 5 raw reading: distance (+ valid bit) | 0..4000 mm (0 = no reading) | 12 + 1 | 4001 values = 12 bits |
| Buddy 5 raw reading: index / count | 0..15 / 1..16 | 4 + 4 | 16 values each |
| Buddy 5 raw reading: scan id, side peek | low 8 bits of the id; flag | 8 + 1 | wrapping id, as on `car/telemetry/scan` |
Wrapping counters/clocks (microsecond timestamps, scan ids, distance since
START modulo 4096 cm) are sized by how far they can move between two reads,
not by a range; they are marked as such in the log.

**Per-buddy tables** (range, unit, resolution, bits, validity rule, sharing):

| Buddy / module | Table |
|---|---|
| Buddy 1, every MQTT field and the shared fault bitmask | [`subsystem1/DATA_REPRESENTATION.md`](../app_program/subsystem1/DATA_REPRESENTATION.md), wire layout in [`MQTT_PROTOCOL.md`](../app_program/subsystem1/MQTT_PROTOCOL.md) |
| Buddy 2, motion and encoder data | [`subsystem2/DATA_REPRESENTATION.md`](../app_program/subsystem2/DATA_REPRESENTATION.md) |
| Buddy 3, line, junction and barcode data | [`subsystem3/DATA_REPRESENTATION.md`](../app_program/subsystem3/DATA_REPRESENTATION.md) |
| Buddy 4, IMU/terrain 12-byte packet | [`subsystem4/DATA_FORMAT.md`](../app_program/subsystem4/DATA_FORMAT.md) |
| Buddy 5, obstacle snapshot, scan samples, status word | [`subsystem5/DATA_FORMAT.md`](../app_program/subsystem5/DATA_FORMAT.md) |

**Fault bitmask.** Stall, encoder, IMU, no-echo, Wi-Fi loss and IR sensor fault
are six bits of one shared mask (`BUDDY1_FAULT_*`), not separate variables.
Buddy 1 owns the word; other modules change only their own bits through its
mutex-protected masked-update API, so there is no unsynchronised
read-modify-write (tribunal risk R2).

**Changed on 14 Nov 2026 to close the last gaps** (both found by this review):

| Field | Before | After | Why this width |
|---|---|---|---|
| Controller bend measure shared with Buddy 5 | `float` (32 bits) | `int8_t` percent | -100..100 % = 201 values = 8 bits |
| Controller bend filter state (private arithmetic) | `float` | `int16_t`, 1/10000 steps | -10000..10000; kept finer so the slow average does not stall (internal precision, as agreed at the tribunal) |
| Buddy 5 aim turn (private) | `int8_t` | 7-bit bit-field | -40..40 degrees = 81 values = 7 bits |

In the historical 14 November comparison, replaying 20000 random line-follower
sequences through the old float filter and the new integer filter gave the
same decision in 19972 runs; the 28 differences were all within 0.01
of the 20 % threshold, and the largest difference was 0.004
([log](../testing/change_requirement_1/logs/2026-11-14_bend_filter.txt)).

## 2. RAM usage before and after

Static RAM = `.data + .bss` from `arm-none-eabi-size` on the linked Arm image.
This is not stack high-water or free heap; those are not claimed.

### 2.1 Whole-firmware before/after (measured when each change was made)

Each row compares two builds with identical options, one before and one after
that buddy's change, so feature work done later does not blur the result.

| Change | Static RAM before | After | Saved | Source |
|---|---:|---:|---:|---|
| Buddy 1: text/JSON MQTT to binary (IR_DEBUG=0 image) | 103596 B | 103444 B | **152 B** | [Buddy 1 protocol v3](BUDDY1_PROTOCOL_V3_2026-10-28.md) |
| Buddy 2: packed motion/encoder state (TEST=1 image) | 83348 B | 83288 B | **60 B** | [Buddy 2 evidence](../testing/subsystem2/EVIDENCE.md) |
| Buddy 3: packed navigation/barcode data (TEST=1 image) | 83288 B | 83052 B | **236 B** | [Buddy 3 build and RAM](../testing/subsystem3/BUILD_AND_RAM.md) |

Code size grows a little in each case (encode/decode code): +2152 B, +800 B,
+4720 B of flash. That cost is recorded, not hidden.

**Buddy 1 message buffers** (the largest saving, allocated at start-up):
4-slot telemetry outbox + 2-slot command buffer **2736 B to 120 B** (104 B until 19 Nov 2026, when the snapshot grew by 3 B to carry distance/heading)
([log](../testing/change_requirement_1/logs/2026-11-14_sizes_buddy1.txt)).

### 2.2 Buddy 4, Buddy 5 and controller storage, before/after

Buddy 4 and Buddy 5 were written after the change requirement was given, so
there is no older firmware to build. "Before" is therefore the same data in
ordinary C types: for Buddy 4 and 5 these are the real decoded-view structs
the code still uses for arithmetic (`terrain_result_t`, `s5_observation_t`).
Sizes are measured by the Arm compiler with the firmware's own flags
([`struct_sizes.c`](../testing/change_requirement_1/struct_sizes.c)):

| Data | Before | After | Saved |
|---|---:|---:|---:|
| Buddy 4 shared terrain measurement | 32 B | 12 B | 20 B (62%) |
| Buddy 4 event queue, 8 slots | 256 B | 96 B | 160 B (62%) |
| Buddy 5 shared task snapshot | 32 B | 23 B | 9 B (28%) |
| Buddy 5 scan sample buffer, 16 samples | 64 B | 48 B | 16 B (25%) |
| Controller to telemetry, Buddy 5 status | 6 B | 2 B | 4 B (66%) |
| Controller bend detector (14 Nov fix) | 12 B | 8 B | 4 B (33%) |

([log](../testing/change_requirement_1/logs/2026-11-14_ram_target.txt))

### 2.3 Historical firmware before the 200 Hz buffer expansion

| Image | Flash (text) | .data | .bss | Static RAM |
|---|---:|---:|---:|---:|
| Demo build, `..._s4_s5` | 460596 B | 496 B | 111048 B | 111544 B |
| Demo build with dashboard previews, `..._s4_s5_ir_debug` | 464996 B | 496 B | 111488 B | 111984 B |

Measured 19 Nov 2026 ([log](../testing/change_requirement_1/logs/2026-11-19_ram_target.txt));
14 Nov: 111408 B / 111848 B. Most of the 136 B since then are the 17 Nov bypass
state, the two 16-reading copies that let raw scan readings reach the
dashboard (Buddy 5 keeps the last scan's 48 B of readings; the controller sends
them one by one from its own copy), and the snapshot cache and outbox slots
growing by 3 B each for distance/heading since START.

The RP2040 has 264 KB of RAM. The large items are kernel task stacks, lwIP/Wi-Fi
buffers and, in this historical build, a 17336 B barcode capture buffer.
The current capture object is 32736 B; see the current comparison above.
Per-buddy object totals are in the historical log; most application state is
instantiated by the vehicle controller module, so it appears under that row.

## 3. Message size before and after

### 3.1 MQTT payloads (application bytes; topic, MQTT and TCP headers excluded)

The first six topics existed before the change: "before" is the exact text the
old Buddy 1 code published. Topics marked * were added after the change; their
"before" is the same values written as JSON, i.e. what a text design would send.

| Topic | Before (text) | After (binary, incl. 1 version byte) |
|---|---:|---:|
| `car/telemetry/motion` | 33 B | 4 B |
| `car/telemetry/barcode` | 30 B | 2 B |
| `car/telemetry/hump` | 26 B | 5 B |
| `car/telemetry/heading` | 19 B | 3 B |
| `car/telemetry/obstacle` | 49 B | 5 B |
| `car/telemetry/state` | 26 B | 2 B |
| `car/telemetry/faults` * | 59 B | 2 B |
| `car/reply/snapshot` * | 279 B | 17 B |
| `car/reply/barcode_map` * | 15 B | 2 B |
| `car/status` * | 39 B | 2 B |
| `car/telemetry/pose` * | 38 B | 4 B |
| `car/telemetry/terrain` * | 168 B | 9 B |
| `car/telemetry/avoidance` * | 124 B | 6 B |
| `car/telemetry/scan` * | 152 B | 13 B |
| `car/telemetry/scan_sample` * (one reading) | 87 B | 6 B |
| **One message of each** | **1144 B** | **82 B** |

19 Nov 2026 ([log](../testing/change_requirement_1/logs/2026-11-19_sizes_buddy1.txt)):
terrain grew from 7 B to 9 B (turn rate, lifted/overturned), the snapshot
reply from 14 B to 17 B (distance and heading since START: 1 validity bit +
12 + 9 bits, the same widths as the live pose) and `car/telemetry/scan_sample`
is new. A 12-reading scan is 12 × 6 = 72 B
in binary; the same readings as text would be about 1 kB.

### 3.2 Messages between tasks

| Message | Before | After | Source |
|---|---:|---:|---|
| Buddy 1 outbox slot (one queued telemetry message) | 452 B | 18 B | Buddy 1 log |
| Buddy 1 command slot | 450 B | 5 B | Buddy 1 log |
| Buddy 1 shared snapshot cache | 52 B | 17 B | Buddy 1 log |
| Buddy 2 motion report copy | 12 B | 4 B | [Buddy 2 log](../testing/change_requirement_1/logs/2026-11-14_sizes_buddy2.txt) |
| Buddy 3 navigation copy | 128 B | 60 B | [Buddy 3 log](../testing/change_requirement_1/logs/2026-11-14_sizes_buddy3.txt) |
| Buddy 3 pending barcode | 8 B | 5 B | Buddy 3 log |
| Buddy 4 queued terrain event | 32 B | 12 B | Section 2.2 |
| Buddy 5 shared snapshot | 32 B | 23 B | Section 2.2 |
| Controller Buddy 5 status | 6 B | 2 B | Section 2.2 |

## 4. Correctness and performance

### 4.1 Historical desk-test evidence

The following logs describe the revisions on which they were recorded. They
are not a claim that native tests were executed for the later compliance cleanup.

| Suite | What it proves | Log |
|---|---|---|
| Buddy 1 host tests | Every topic packs and unpacks exactly over its full range (e.g. every range 30..4000 mm, every edge -4000..4000 mm); out-of-range values and bad padding are rejected | [log](../testing/change_requirement_1/logs/2026-11-19_tests_buddy1.txt) |
| Laptop decoder tests | The dashboard decodes the same bytes the car sends | [log](../testing/change_requirement_1/logs/2026-11-19_tests_laptop_codec.txt) |
| Buddy 2 to 5 host tests | Packed shared data round-trips; behaviour tests (PID, line following, barcode, hump, obstacle bypass) still pass with the packed formats | `..._tests_buddy2..5.txt` |
| Integration tests | All five buddies through the real controller, including the shared status packet boundaries and run timing | [log](../testing/change_requirement_1/logs/2026-11-19_tests_integration.txt) |
| Dashboard tests | The laptop shows every decoded field | [log](../testing/change_requirement_1/logs/2026-11-19_tests_dashboard.txt) |
| Width check, bend filter check | Sections 1 above | `..._widths.txt`, `..._bend_filter.txt` |

**Earlier on-car demonstrations:** the packed telemetry is what the team uses: the
dashboard shows motion, barcode, terrain (pitch, roll, hump height, run peak,
turn rate, lifted/overturned), obstacle scan, raw scan readings and avoidance
live over Wi-Fi, and the 10 mission requirements
except the U-turn were reported demonstrated at that earlier snapshot. This is
not a physical validation claim for the new compliance firmware.

### 4.2 Timing evidence

The integrated firmware configures the motion PID at 10 ms and IR sampling at
5 ms. The host fixtures exercised nominal gaps, missed-cycle detection,
counter saturation and clock rollover. The checker flags recorded start gaps
of at least 20 ms for motion and 10 ms for IR.

The team completed the demo before dismantling the car. There is no matched
three-minute hardware capture for the post-demo firmware. Its actual task
intervals under Wi-Fi load and its hotspot recovery time are unmeasured.
The [archived procedure](PHYSICAL_VALIDATION.md) describes the original test conditions.

## How to regenerate

From the repository root (the Arm toolchain and `config/wifi_secrets.h` are
needed only for the target RAM part):

```
sh testing/change_requirement_1/run_all.sh
```

The runner creates a fresh directory under `build/change-requirement-1/`
with per-check logs and a `results.tsv` summary. The final paired builds used
the same public example network settings.

## Interpreting the measurements

Static RAM figures come from the linked image. The Buddy 4 and 5 storage
comparisons use the same values represented by ordinary C types as their
“before” case. Buddy 3 retains the full 12-bit ADC reading for calibration
and steering, and packs the derived results for transport.
