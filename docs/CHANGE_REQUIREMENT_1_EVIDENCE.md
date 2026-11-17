# Change Requirement 1: Minimum-Bit Data Representation, Week 13 evidence

Team AAI07. Measured 14 Nov 2026 on branch `buddy5-revert-B` unless a row says
otherwise. Every number below comes from a script or test in this repository;
[how to regenerate](#how-to-regenerate) is at the end.

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
lists 93 packed fields (all MQTT payloads, the Buddy 4 and Buddy 5 internal
packets, the Buddy 5 scan sample, the controller's Buddy 5 status word and
Buddy 5's private avoidance fields). For each field it recomputes the minimum
width from the range and fails if the code uses a different width. It also
checks each topic's field widths add up to what the laptop decoder uses.
Result: **all 93 fields use the minimum width; all 16 totals match**
([log](../testing/change_requirement_1/logs/2026-11-17_widths.txt); 86 fields on 14 Nov, the 7 new
ones are the 17 Nov bypass state).
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

The car makes the same bend decisions as before: replaying 20000 random
line-follower sequences through the old float filter and the new integer filter
gave the same decision in 19972 runs; the 28 differences were all within 0.01
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
4-slot telemetry outbox + 2-slot command buffer **2736 B to 104 B**
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

### 2.3 Current firmware (for the record)

| Image | Flash (text) | .data | .bss | Static RAM |
|---|---:|---:|---:|---:|
| Demo build, `..._s4_s5` | 453948 B | 488 B | 110920 B | 111408 B |
| Demo build with dashboard previews, `..._s4_s5_ir_debug` | 458396 B | 488 B | 111360 B | 111848 B |

The RP2040 has 264 KB of RAM. The large items are kernel task stacks, lwIP/Wi-Fi
buffers and the fixed 17336 B barcode capture buffer (scans uploaded to the
laptop). Per-buddy object totals are in the log; most application state is
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
| `car/reply/snapshot` * | 234 B | 14 B |
| `car/reply/barcode_map` * | 15 B | 2 B |
| `car/status` * | 39 B | 2 B |
| `car/telemetry/pose` * | 38 B | 4 B |
| `car/telemetry/terrain` * | 122 B | 7 B |
| `car/telemetry/avoidance` * | 124 B | 6 B |
| `car/telemetry/scan` * | 152 B | 13 B |
| **One message of each** | **966 B** | **71 B** |

### 3.2 Messages between tasks

| Message | Before | After | Source |
|---|---:|---:|---|
| Buddy 1 outbox slot (one queued telemetry message) | 452 B | 15 B | Buddy 1 log |
| Buddy 1 command slot | 450 B | 5 B | Buddy 1 log |
| Buddy 1 shared snapshot cache | 52 B | 14 B | Buddy 1 log |
| Buddy 2 motion report copy | 12 B | 4 B | [Buddy 2 log](../testing/change_requirement_1/logs/2026-11-14_sizes_buddy2.txt) |
| Buddy 3 navigation copy | 128 B | 60 B | [Buddy 3 log](../testing/change_requirement_1/logs/2026-11-14_sizes_buddy3.txt) |
| Buddy 3 pending barcode | 8 B | 5 B | Buddy 3 log |
| Buddy 4 queued terrain event | 32 B | 12 B | Section 2.2 |
| Buddy 5 shared snapshot | 32 B | 23 B | Section 2.2 |
| Controller Buddy 5 status | 6 B | 2 B | Section 2.2 |

## 4. Correctness and performance

### 4.1 Nothing lost (desk tests, 14 Nov 2026, all PASS)

| Suite | What it proves | Log |
|---|---|---|
| Buddy 1 host tests | Every topic packs and unpacks exactly over its full range (e.g. every range 30..4000 mm, every edge -4000..4000 mm); out-of-range values and bad padding are rejected | [log](../testing/change_requirement_1/logs/2026-11-14_tests_buddy1.txt) |
| Laptop decoder tests | The dashboard decodes the same bytes the car sends | [log](../testing/change_requirement_1/logs/2026-11-14_tests_laptop_codec.txt) |
| Buddy 2 to 5 host tests | Packed shared data round-trips; behaviour tests (PID, line following, barcode, hump, obstacle bypass) still pass with the packed formats | `..._tests_buddy2..5.txt` |
| Integration tests | All five buddies through the real controller, including the shared status packet boundaries and run timing | [log](../testing/change_requirement_1/logs/2026-11-14_tests_integration.txt) |
| Dashboard tests | The laptop shows every decoded field | [log](../testing/change_requirement_1/logs/2026-11-14_tests_dashboard.txt) |
| Width check, bend filter check | Sections 1 above | `..._widths.txt`, `..._bend_filter.txt` |

**On the car** the packed telemetry is what the team uses every run: the
dashboard shows motion, barcode, terrain (pitch, roll, hump height, run peak),
obstacle scan and avoidance live over Wi-Fi, and the 10 mission requirements
except the U-turn have been demonstrated with this firmware.

### 4.2 Timing on the real car (to be recorded by the team)

Question: do the packing work and the network (including a Wi-Fi drop and
reconnection) make the 10 ms control loops late? (Tribunal test plan, Buddy 1
and Buddy 2, "heavy telemetry or forced Wi-Fi reconnection".)

The `DIAGNOSTICS=1` firmware measures, on the Pico itself, the gap between
starts of the motion PID task and the IR sampling task (both 10 ms). A gap of
20 ms or more would mean a missed cycle.

1. Build and flash the diagnostics image (same as the demo build plus `DIAGNOSTICS=1`):
   ```
   make -C build_make CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 TEST=0 IR_BENCH=0 S3_MOTION=1 IR_DEBUG=1 DIAGNOSTICS=1 MOTION_SMOKE=0 S4=1 S5=1 S5_HARDWARE_CONFIRMED=1 -j8
   cp -X build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_s3_motion_diag_s4_s5_ir_debug.uf2 /Volumes/RPI-RP2/
   ```
2. Start the broker and the dashboard as usual and check "Car online".
3. Keep the USB cable in and record 3 minutes of the console
   (`pip install -r tools/requirements-robot-log.txt` once):
   ```
   python3 tools/capture_robot_log.py --port /dev/cu.usbmodemMTK3PICO1 --output build/robot-logs/cr1-timing --firmware diag_ir_debug --test cr1-timing --duration 180
   ```
   During the recording: after 1 minute turn the phone hotspot off for
   10 seconds, turn it back on and wait until the dashboard shows the car online again.
4. Check the result:
   ```
   python3 testing/change_requirement_1/timing_check.py build/robot-logs/cr1-timing/console.log
   ```
5. Paste the table here and add a dashboard screenshot from the same run.

| Source | Samples | Min gap (ms) | Max gap (ms) | Max execution (ms) | Result |
|---|---:|---:|---:|---:|---|
| motion | _to fill_ | | | | |
| ir | _to fill_ | | | | |

Known risk carried from Tribunal 1: the kernel's Wi-Fi driver task runs at a
higher priority than motion control. This test is what shows whether that
matters in practice.

## How to regenerate

From the repository root (the Arm toolchain and `config/wifi_secrets.h` are
needed only for the target RAM part):

```
sh testing/change_requirement_1/run_all.sh
```

It writes one log per check to `testing/change_requirement_1/logs/` and prints
PASS/FAIL per step. Static RAM depends slightly on the private Wi-Fi settings
compiled in; the logs above were built with the public example settings.

## Not claimed

- Static RAM is not stack high-water, free heap or CPU load.
- Host tests do not replace the on-car timing test in 4.2.
- Buddy 4/5 "before" sizes are the same data in ordinary C types, not an
  older firmware (none existed).
- Buddy 3's private raw ADC readings keep full precision on purpose; only
  processed results are packed (tribunal decision for Buddy 3).
