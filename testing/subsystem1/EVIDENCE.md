# Buddy 1 Tribunal 1 implementation evidence

## Current protocol v3 — 28 October 2026

Current format, combined firmware sizes and reproducible tests are recorded in
[`docs/BUDDY1_PROTOCOL_V3_2026-10-28.md`](../../docs/BUDDY1_PROTOCOL_V3_2026-10-28.md).
Height is now 9 bits; obstacle distance/width 12 bits each; snapshot 14 bytes;
outbox slot 15 bytes. Recorded wireless barcode traces are binary, while local
text exports remain supported. No hardware timing/resource sign-off is claimed.
The v1/v2 measurements below are historical baselines, not the current format.

## Protocol v2 IR-fault addendum dated 2026-10-23

The Buddy 3 integration adds aggregate IR fault at bit 5 and leaves bits 0-4
unchanged. Fault bodies now use six bits, snapshot bodies 117 bits and heartbeat
bodies eight bits. The snapshot remains 16 bytes, faults/status two bytes, queue
slots 17 bytes and the cached snapshot 16 bytes. Robot, commands, fallback
heartbeat and laptop decoder now all use version 2; version 1 is rejected.

Current host verification passes 40 C checks and five independent Python tests.
All 64 fault combinations round-trip through live faults, snapshot and heartbeat;
eight concurrent producers preserve all six fault bits alongside heading and
obstacle values. The snapshot's first padding bit moved from 116 to 117 and the
new IR bit occupies heartbeat bit 7; boundary/padding cases are explicitly tested.
The aggregate IR bit does not replace Buddy 3's channel-specific mask or become
the controller's authoritative safety state. Hardware timing/physical fault
validation remains pending. The controller reporting adapter is integrated by
the current Buddy 3 work, separate from this codec change.

Current raw host results are in `PROTOCOL_V2_TEST_RESULTS.txt`. The original
`HOST_TEST_RESULTS.txt` remains the historical v1 run referenced below.

The original measurements and test counts below are retained as **historical
protocol-v1 evidence from 2026-10-22**, not measurements of the full current
Buddy 1/2/3 image. Use the new combined integration evidence for current linker
RAM and timing claims. Do not add these historical savings to later measurements.

## Original protocol-v1 baseline and measurements

Measured on 2026-10-22 in the `integration/buddies-1-3` working tree. The baseline
is the pre-change JSON implementation, including the existing uncommitted
integration work, not the older scaffolding at Git HEAD. No commits, hardware
flashing or broker connections were made during these checks.

## Requirement and design trace

| Source | Implementation / evidence | Remaining work |
|---|---|---|
| Change Requirement 1: binary telemetry/commands | explicit codec, raw MQTT receive bytes, explicit lengths, laptop decoder | physical broker/laptop exchange |
| Minimum bits for valid range | `DATA_REPRESENTATION.md`: heading 9, barcode 5, state 3, faults 5, wheel direction | Buddy 4/5 physical ranges; Buddy 2 confirmation of proposed speed contract |
| Safe packed sharing | packed cache/map under priority-inheritance mutex; copied queue slots; concurrent-producer test | real kernel/interrupt timing and fault injection |
| Week 6 FR8; Tribunal B: telemetry/snapshot | existing reporting APIs; 16-byte packed snapshot; readable topics | missing sensing fields and producers |
| Week 6 FR11: pre-run mapping | 20-bit map; same mutex closes configuration before controller begin | on-car before/after-run test |
| Tribunal B: fault mask | five agreed bits; masked updates; radio link hook | other owners connect actual faults/controller logic |
| Tribunal B: bounded outbox | four copied slots; `TMO_POL`; overflow preserves cache | burst load, drop tolerance, high-water observation |
| Week 6 NFR2/NFR7: control unaffected by networking | producer never waits for MQTT; drain budget four | kernel radio priority 4 still above motion 14; jitter/recovery NOT proven |

## Message and allocation measurements

Command: `make -C testing/subsystem1 sizes`. The baseline structs are preserved
in `size_report.c`. Its assertions were also compiled by `arm-none-eabi-gcc`
for Cortex-M0+. The host size target uses short enums to match that Arm ABI;
ordinary desktop enum layout gives a different baseline snapshot size. Wire
correctness tests also run with ordinary host enums to verify independence.

| Item | Before | After | Saving |
|---|---:|---:|---:|
| Outgoing slot | 452 B | 17 B | 435 B |
| Incoming command slot | 450 B | 5 B | 445 B |
| Four-slot outgoing RTOS buffer | 1,824 B | 96 B | 1,728 B |
| Two-slot command RTOS buffer | 912 B | 24 B | 888 B |
| Combined requested buffer storage | 2,736 B | 120 B | 2,616 B (95.6%) |
| Shared snapshot cache | 52 B | 16 B | 36 B |
| Network RX staging array | 128 B | 16 B | 112 B |
| Network per-poll message object | 452 B | 84 B | 368 B of object size on stack |

Buffer sizes include per-message kernel headers and four-byte alignment, as
implemented in `kernel/tkernel/messagebuf.c`. They exclude allocator and kernel
object overhead. Object counts, task stack reservations and lwIP allocations
are unchanged. Smaller stack objects do not prove lower maximum stack usage.

| Example MQTT application payload | JSON bytes | Binary bytes including version |
|---|---:|---:|
| motion: 120 and -118 mm/s | 33 | 4 |
| barcode: W, left | 30 | 2 |
| hump: count 2, height 45 mm | 26 | 5 |
| heading: 359 degrees | 19 | 3 |
| obstacle: 180 mm distance, 140 mm width, left | 49 | 6 |
| state: line_following | 26 | 2 |

JSON lengths vary with values. Figures exclude topics and MQTT/network overhead.
The 16-byte snapshot includes all seven sections and their validity.

## Full firmware static RAM

Both builds: Arm GNU tooling, the same local Pico SDK, SMP=1, USB CDC,
WIFI=cyw43, NET=lwip, MQTT=1, TEST=1. A temporary copy of the public
`wifi_secrets.example.h` supplied compile-only settings through the include path.
No private credentials were read or added. Generated network images do not
contain the user's real Wi-Fi settings.

Measured with `arm-none-eabi-size` on the saved pre-change ELF and revised ELF:

| Section / budget | Before | After |
|---|---:|---:|
| text | 362,728 B | 366,944 B |
| data | 180 B | 196 B |
| bss | 83,304 B | 83,152 B |
| static RAM (`data + bss`) | 83,484 B | 83,348 B |

Static RAM decreases by 136 bytes. Code/flash grows; the revised profile also
contains more tests. RTOS buffers are allocated inside the kernel heap already
reserved in BSS: the 2,616-byte allocation saving gives more space inside that
heap. Do not add it again as a linker static-RAM reduction. Runtime free-heap
and stack high-water measurements remain pending.

## Completed checks

- 37 C checks pass on host substitutes using the real Buddy 1 code. The same
  checks compile into `TEST=1`; they have not been run on a Pico in this session.
- All 1,021 signed speed inputs, 360 headings, eight states and all barcode
  body codes are checked against expected interpretation.
- Four concurrent host threads update separate cache/fault fields repeatedly;
  final fields and neighbouring fault bits remain intact.
- Real MQTT service callbacks/bridge run against lwIP test substitutes. Binary
  fragments, zero bytes, exact lengths, oversized topics/messages, truncated
  input, publishing, heartbeat and Wi-Fi bit preservation pass.
- Four Python tests independently decode C-generated packets, validate known
  values, reject malformed data and distinguish valid zero from invalid.
- Host address and undefined-behaviour checks pass.
- Full Wi-Fi/lwIP/MQTT production and test profiles compile and link.
- Arm target compilation confirms the layout assertions.

Reproduce with `make -C testing/subsystem1 test`, `sizes` and `sanitize`.
Raw generated results: `HOST_TEST_RESULTS.txt` and `SIZE_RESULTS.txt`.

## Still required for Week 13

Buddy 1's existing supported fields have been converted. Final subsystem ranges,
missing real fault adapters, actual Wi-Fi/broker recovery, stack/heap high-water
use and control-task timing remain open. Host threads do not prove interrupt or
kernel timing. No hardware success is claimed. See `README.md` for physical tests.
