# Subsystem 5 — test log

**Buddy 5:** Ultrasonic scanning and obstacle avoidance. Owner: TBD.

See `testing/README.md`. No subsystem tests have been implemented or run.
The test-build hook reports SKIP.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|


## 27 October 2026 — implementation on xavier/development

Base commit `6225b04` plus uncommitted Buddy 5 implementation. Pins confirmed by
Xavier: HC-SR04 ECHO GP0, TRIG GP1, servo signal GP12. No flash, motor actuation,
physical scan, broker connection, commit or push performed.

| ID | Check | Result / evidence |
|---|---|---|
| S5-H01 | Codec/range/scan/profile/planner/recovery boundary and fault cases | Host PASS; `logs/2026-10-27_host.log` |
| S5-H02 | Production S3 + S5 + real Buddy 2 controller/PID bypass handoff | Host PASS with mocked hardware; same log |
| S5-H03 | Production PIO/PWM drivers and resource isolation | Fake-register PASS; same log; not a physical pulse measurement |
| S5-H04 | Production sensing task, scan mailbox, cancellation, no-echo | Host PASS; same log |
| S5-H05 | Hardware activation guard | Host PASS; same log |
| S5-H06 | Telemetry profile/state and owned no-echo mask | Host PASS; same log |
| S5-P01 | Known-distance and servo-angle validation | NOT RUN |
| S5-P02 | Known obstacle/profile/clearance validation | NOT RUN |
| S5-P03 | Stopped ACK, bypass and original-line recovery | NOT RUN |
| S5-P04 | Integrated faults, networking and timing | NOT RUN |
| S5-W13 | Tribunal implementation demonstrated on robot | NOT RUN |

Host sanitizer/build/regression details are in `docs/BUDDY5_IMPLEMENTATION_2026-10-27.md`.
Earlier placeholder entries above remain historical; they do not describe this
implementation. Current records are not evidence that the physical robot passes.

## 28 October 2026 — merge with main

Fetched main `19d02fe`, fast-forwarded xavier/development and reapplied local
Buddy 5. Resolved both shared-controller conflicts preserving main's turn search
and barcode capture. Seven Buddy 5 executables, the controller/capture/course
suite and Buddy 1 tests passed normally and with ASan/UBSan. The added S5-enabled
clear-course simulation checks new left/right undershoot/overshoot recovery.
Offline enabled, MQTT enabled and offline unconfigured firmware builds passed
with zero compiler warnings. Final profile remains unconfigured; no flash or
physical test. Logs: `logs/2026-10-28_*`; details:
`docs/BUDDY5_MAIN_MERGE_2026-10-28.md`. Week 13 demonstration remains outstanding.

## 28 October 2026 — snapshot contention fix

Review reproduced a latched fault when one nonblocking S5 snapshot read returned
false despite a fresh clear range. The read API now distinguishes BUSY from
unavailable/corrupt data. The sole controller may reuse a packed snapshot only
for BUSY and within the existing 250 ms limits; source timestamps never advance
on reuse. Explicit invalid/no-echo observations still fault immediately.

Added regression cases for first-cycle contention after START, intermittent and
prolonged contention, an already-old source timestamp, no cache, corrupt/unavailable
data, active avoidance and STOP. The complete seven-move bypass also runs with
intermittent BUSY reads, with and without S4 arbitration. Production task tests
verify actual mutex timeout/error classification and malformed-packet handling.
Results are in `logs/2026-10-28_contention-*`. No physical test/flash was performed.
Speed settings were not changed; the separate simulator-speed mismatch remains.
