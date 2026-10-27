# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-B01 | Combined Buddy 1/2/3 firmware builds in all relevant profiles | I1–I3 compile/link compatibility | auto/build | Build normal, `TEST=1`, isolated `IR_BENCH=1`, and Wi-Fi/lwIP/MQTT profiles | Every profile compiles and links; bench remains isolated |
| INT-H01 | Latest steering reaches Buddy 2 without an outdated backlog | I1, I2 | manual car bench | Calibrate, start locally, vary the line under the mounted sensors and observe wheel response | Controller applies newest valid proposal; loss stops the motors |
| INT-H02 | Barcode action is consumed at a junction | I1–I3 | manual track | Configure map, pass one known barcode, reach a junction, observe manoeuvre and MQTT telemetry | One mapped manoeuvre runs, tracking resumes, barcode/state are reported |
| INT-B02 | Packed Buddy 3/controller/Buddy 2 handoff | Tribunal 1 minimum-bit, I1–I3 | host with fake RTOS/motor boundaries | Run actual controller-task tests and Buddy 2 adapter/regression tests | Exact steering and original control trace retained; barcode waits for junction; fault bits remain independently owned |
| INT-B03 | Optional diagnostic observation preserves normal firmware | Hardware-test preparation, I1–I3 | host/build | Test observer and startup failure paths; compile normal/diagnostic/test/IR-bench profiles; compare disabled loadable bytes | No motion requests or completion consumption; disabled image unchanged; diagnostic mode is separately named and guarded |
| INT-B04 | Operator v2 calibration, held STOP, recovery and map locking | Local control, single motor owner, pre-run configuration freeze | host/build | Complete all sensor collections/apply; exercise STOP held at boot/countdown and both-button release orders; reset FAULT; exercise refused/accepted/duplicate/restart map claims | Calibration works without bypass; held STOP inhibits START; reset remains available; earlier successful run retains map freeze |
| INT-B05 | Wi-Fi no-IP recovery | Buddy 1 radio/network owner | host | Run policy and production-task virtual-clock tests for JOIN/NOIP, errors, link loss, backoff and UP | Bounded deadline/backoff; radio/DHCP service continues; no owner-lock violation |
| INT-B06 | Retained MQTT publish and pacing | Buddy 1 outbox to lwIP | host | Inject temporary publish rejection, delayed callbacks and reconnect | Exact packet/FIFO retained with bounded retry/pacing; permanent errors explicit; no busy loop |
| INT-B07 | One-request reset acknowledgement | Controller to Buddy 2 | host | Clear real/fake motion fault, publish next feedback; inject unsafe feedback, cancellation and timeout | One final acknowledgement only; fresh safe feedback required; STOP wins; no automatic START |
| INT-B08 | Run-scoped timing observation | Controller status/packed format | host | Inject normal/jitter/safety gaps, START/halt/STOP, wrap and saturation | 10 ms target respected in labels; 30 ms violations recorded; ending interval retained; idle statistics frozen; 146-byte packet exact |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-17 | INT-B01 | `integration/buddies-1-3` working tree | Codex | PASS | Local build output | Normal, test, IR bench, and full Wi-Fi/lwIP/MQTT firmware compiled and linked. MQTT used temporary non-secret compile-only settings, removed afterward. No flashing or hardware runtime claim. |
| 2026-10-23 | INT-B01 | `integration/buddies-1-3` working tree | Codex | PASS | `testing/subsystem3/BUILD_AND_RAM.md` | TEST=0, TEST=1 and isolated IR bench compile/link after packed handoff and coordinated protocol-v2 update. Public example settings only; no flashing. |
| 2026-10-23 | INT-B02 | `integration/buddies-1-3` working tree | Codex | PASS (host) | `controller_test.mk`, `testing/subsystem2/EVIDENCE.md`, `testing/subsystem3/EVIDENCE.md` | Actual controller task tested with fake RTOS/motor boundaries, including sanitizers; original Buddy 2 production files unchanged. Does not close physical INT-H01/H02. |
| 2026-10-24 | INT-B03 | `integration/buddies-1-3` working tree | Codex | PASS (host/build) | `testing/diagnostics/EVIDENCE.md` | Optional diagnostics and startup/failure tests pass with sanitizers; four full profiles build; disabled TEST=1 loadable image byte-identical to saved baseline. No hardware ports, flashing or robot movement. |
| 2026-10-25 | INT-B01 | `71193e2` plus local integration | Codex | PASS (build) | `docs/INTEGRATION_2026-10-25.md` | Combined offline diagnostics, full network compile-only, 1/3-channel IR bench, offline self-tests and isolated smoke images build/link. Network uses public placeholders and is labelled COMPILE_ONLY; no flashing. |
| 2026-10-25 | INT-B02 | `71193e2` plus local integration | Codex | PASS (host) | `testing/integration/controller_test.mk`, `docs/INTEGRATION_2026-10-25.md` | Real single-owner runtime, facade and full 60/122/11-byte codecs tested with sanitizer/fake hardware boundaries; STOP precedence, calibration gate, map freeze, owned faults and safe telemetry range checked. Current teammate motion feedback/drive tests pass; Buddy 3 navigation and stationary bench traces match imported revision. INT-H01/H02 still open. |

Physical tests INT-H01 and INT-H02 remain pending.

## 26 October — reviewed operator version 2

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | INT-B04 | 71193e2 plus local integration and v2 patch | Codex | PASS (host/sanitizer) | controller_test.mk, operator_test.c, map_freeze_test.c, runtime_test.c; /tmp/robot-v2-final-integration-tests.log | Actual runtime calibration, held STOP, fault recovery and real Buddy 1 map storage/facade/runtime pass; independent operator timestamp-wrap/randomized probes also pass. Host boundaries replace hardware. |
| 2026-10-26 | INT-B02 | Same working tree | Codex | PASS (host/sanitizer) | /tmp/robot-v2-buddy1-tests.log, /tmp/robot-v2-motion-tests.log, /tmp/robot-v2-sensors-tests.log | Buddy 1/2/3 suites run in a patched source snapshot before application. Buddy 2 and Buddy 3 subsystem C/header hashes unchanged after application. No new physical evidence. |
| 2026-10-26 | INT-B01 | Same working tree | Codex | PASS (offline course build) | /tmp/robot-v2-final-course-build.log; docs/COURSE_QUICKSTART_2026-10-26.md | USB CDC, S3_MOTION=1, Wi-Fi/network/MQTT/test/bench/diagnostics/smoke disabled; no compiler warnings. text=120508, data=92, bss=24244 bytes (not total runtime RAM usage). No flash or Wi-Fi runtime test. |

The current image is for an initial short-straight trial, not a complete Y/Z
course lap. Physical INT-H01/H02 and measured software-STOP latency remain open.

## 27 October — software robustness fixes

The paragraph immediately above describes the 26 October offline build. The
new combined Wi-Fi image is documented in `docs/SOFTWARE_FIXES_2026-10-27.md`;
physical course success is still not claimed.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-27 | INT-B05 | 71193e2 + existing WIP + robustness changes | Codex | PASS (host/sanitizer) | logs/2026-10-27_wifi.txt | Policy and production radio task use virtual clock/fake driver; stuck JOIN/NOIP, backoff cap/reset, DOWN recovery, failed join/leave, clock wrap, ownership and continued polling. No hotspot accessed. |
| 2026-10-27 | INT-B06 | Same working tree | Codex | PASS (host/sanitizer) | logs/2026-10-27_buddy1.txt | Retains binary bytes/QoS/FIFO after temporary rejection and reconnect; pacing, delayed completion, permanent error and clock wrap. Existing 41 binary checks + laptop codec tests pass. Bounded outbox overflow/ambiguous delivery duplicates remain possible. |
| 2026-10-27 | INT-B07 | Same working tree | Codex | PASS (host/sanitizer) | logs/2026-10-27_integration.txt; logs/2026-10-27_buddy3.txt | Real Buddy 2 controller/PID with fake motors verifies single-request reset. Actual integration runtime verifies deferred ACK, blocked ordinary commands, STOP cancellation and timeout. Production Buddy 2 files unchanged. |
| 2026-10-27 | INT-B08 | Same working tree | Codex | PASS (host/sanitizer) | logs/2026-10-27_integration.txt | Target/safety boundaries, wrapping timestamps, saturating counters, per-START reset, fault-causing gap, freeze after halt, malformed/roundtrip 146-byte packed status. Instrumentation is not proof of timing bounds. |
| 2026-10-27 | INT-B01 | Same working tree | Codex | PASS (six builds) | logs/2026-10-27_network_build.txt and matching offline/ir_bench/smoke/selftest/network_diag build logs | Normal combined Wi-Fi and offline course, three-channel IR bench, smoke, offline self-test and combined Wi-Fi diagnostics all compile/link with zero compiler warnings. Final build is normal combined Wi-Fi course. No flash. |

Physical line-following, barcode/crossing, Wi-Fi recovery and measured STOP/timing
tests remain open. Unrelated changes and earlier results were preserved.
