# Subsystem 2 — test log

**Buddy 2:** Motion control using motors and wheel encoders. Owner: 2503508.

See `testing/README.md` and [how to run the Buddy 2 tests](README.md). Unit
tests S2-U01 to S2-U22 are automated in
`app_program/subsystem2/subsystem2_test.c` (`make TEST=1`). S2-U23 to S2-U30
are manual hardware tests: the robot is measured by hand.

- S2-U01 to S2-U10 and S2-U13 to S2-U20 never move the robot. S2-U05 runs only
  while the robot is **not** calibrated; S2-U06 to S2-U22 run only once the
  CALIBRATE values in `motion.c` are set. The others print SKIP.
- S2-U11, S2-U12, S2-U21 and S2-U22 drive the robot. Set `S2_TEST_MOVE` to 1 in
  `subsystem2_test.c`, put the robot on the floor with 30 cm clear ahead and
  room to turn, then reset the Pico. They drive forward, turn left, turn right
  and drive back, ending on the start mark.
- Acceptance criteria for the manual accuracy tests come from S2-U11 / S2-U12:
  **±20 mm per drive, ±10° per turn**.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | Motion task starts in a safe state | Statechart IDLE | Automated | Wait 100 ms after start-up, then read `motion_poll()` and `motion_get_fault()` | State IDLE, fault NONE |
| S2-U02 | Encoders stay steady while the motors are braked | Encoder feedback | Automated | Read `enc_l_count()` / `enc_r_count()`, wait 200 ms, read again (robot untouched) | Both counts unchanged |
| S2-U03 | Negative distances are rejected | Brief API (`moveForward`, `moveBackward`) | Automated | `moveForward(-0.1)` and `moveBackward(-0.1)` | Both return MOTION_ERR_BAD_ARG |
| S2-U04 | Invalid angles are rejected | Brief API (`turnLeft`, `turnRight`) | Automated | `turnLeft(NaN)` and `turnRight(-5)` | Both return MOTION_ERR_BAD_ARG |
| S2-U05 | Moves are refused until the robot is calibrated | Safety: no motion without encoder scale | Automated (uncalibrated build only) | With `ENCODER_COUNTS_PER_REV` = 0: `moveForward(0.1)` and `motion_request_velocity(0, 0)` | Both return MOTION_ERR_NOT_CALIBRATED |
| S2-U06 | Completion is reported exactly once | Statechart COMMAND COMPLETE -> IDLE | Automated | `motion_request_drive(0, 0.2)` (zero length), then `motion_poll()` twice | Request OK; first poll COMPLETE, second poll IDLE |
| S2-U07 | Zero / negative speeds are rejected | `motion_request_drive`, `motion_request_turn` | Automated | `motion_request_drive(0.1, 0)` and `motion_request_turn(90, -0.1)` | Both return MOTION_ERR_BAD_ARG |
| S2-U08 | VELOCITY blocks other moves; `stop()` returns to IDLE | Statechart VELOCITY CONTROL, `stop()` | Automated | `motion_request_velocity(0, 0)`, poll, `moveForward(0.1)`, `stop()`, wait 20 ms, poll | Velocity OK; state VELOCITY; move returns MOTION_ERR_BUSY; then IDLE |
| S2-U09 | A stale velocity command raises a fault | Statechart MOTION FAULT, 250 ms command timeout, fault bitmask | Automated | `motion_request_velocity(0, 0)`, wait 400 ms without re-sending, poll, read the fault and fault bits, then `moveForward(0.1)` | State FAULT, fault STALE_COMMAND, fault bits = STALE_COMMAND bit only; move returns MOTION_ERR_FAULT |
| S2-U10 | `motion_reset()` clears a fault once the wheels are still | Statechart MOTION FAULT -> IDLE | Automated | After S2-U09, wait 300 ms, `motion_reset()`, poll | Reset returns true; state IDLE, fault NONE, fault bits 0 |
| S2-U11 | Straight drive reaches its distance | Straight manoeuvre, encoder distance | Automated, robot moves (`S2_TEST_MOVE` = 1) | `moveForward(0.20)`, poll until COMPLETE (10 s limit), compare `robot_distance_m()` before and after | COMPLETE; distance 0.20 m ± 0.02 m |
| S2-U12 | Left turn on the spot reaches its angle | Left turn, encoder heading | Automated, robot moves (`S2_TEST_MOVE` = 1) | `turnLeft(90)`, poll until COMPLETE (10 s limit), compare `robot_heading_deg()` before and after | COMPLETE; heading change 90° ± 10° |
| S2-U13 | A DONE report carries the command ID, once | Report queue, command IDs | Automated | `motion_request_drive(0, 0.2)`, read `motion_command_id()`, `motion_get_report()` twice | One DONE report with that ID; no second report |
| S2-U14 | Cancel: an old ID is rejected, the running command is cancelled | `motion_cancel()` | Automated | Start VELOCITY (new ID), `motion_cancel(old ID)`, `motion_cancel(current ID)`, wait 20 ms | Old ID: MOTION_ERR_OBSOLETE; current: OK, one CANCELLED report with that ID |
| S2-U15 | A fault report carries the ID; `stop()` keeps the fault | Fault reporting | Automated | Let a VELOCITY command go stale, read the report, `stop()`, `motion_cancel(id)` | FAULT report with the ID and STALE_COMMAND; cancel returns MOTION_ERR_FAULT; state stays FAULT |
| S2-U16 | Stopped acknowledgement after a reset | `motion_is_stopped()` | Automated | After S2-U15, wait 300 ms, `motion_reset()`, `motion_is_stopped()` | Both true |
| S2-U17 | Two reports queue up and are read in order | Report queue | Automated | VELOCITY then `stop()` (cancelled), wait 20 ms, zero-length drive (done), read two reports | CANCELLED then DONE, with their IDs; queue peak ≥ 2 |
| S2-U18 | A full queue never blocks; the next report is marked lost | Bounded report queue (8) | Automated | Queue 9 reports without reading, read them all, queue two more | 8 read; next report has `b_lost` set, the one after does not |
| S2-U19 | Packing: values, limits, sizes | Minimum-bit status (64), report (16), command (32) words | Automated | Pack/unpack examples, saturation, no negative zero, two faults at once, out-of-range commands | Round trip within one step; both fault bits survive; invalid input rejected |
| S2-U20 | Packed command and status word end to end | `motion_request_packed()`, `motion_get_status_packed()` | Automated | VELOCITY and CANCEL as packed words, an unknown type (7), read the status word | OK, CANCELLED report, MOTION_ERR_BAD_ARG, state IDLE in bits 0–2 |
| S2-U21 | Right turn on the spot reaches its angle | Right turn, encoder heading | Automated, robot moves (`S2_TEST_MOVE` = 1) | `turnRight(90)` after S2-U12, poll until COMPLETE | COMPLETE; heading change −90° ± 10° |
| S2-U22 | Straight drive backward reaches its distance | Backward manoeuvre (both wheels backward) | Automated, robot moves (`S2_TEST_MOVE` = 1) | `moveBackward(0.20)` after S2-U21, poll until COMPLETE | COMPLETE; distance −0.20 m ± 0.02 m; robot back on its start mark |
| S2-U23 | Calibration | Encoder scale, top speed, motor and encoder directions | Manual (`S2_TEST_CALIBRATE` = 1) | Turn each wheel 10 turns by hand, keep still 3 s, lift the robot; motors run at full power | `VALID`; left/right counts per turn within 5 %; values copied into `motion.c` |
| S2-U24 | Wheel-speed PID step response | Speed PID (Kp, Ki), feedforward, ramp | Manual, wheels lifted (step-test build) | Steps 0→0.2 and 0.2→0.4 m/s on both wheels for each gain set; rise, overshoot, settling, steady-state error printed per wheel | Steady-state error < 8 mm/s (one count per period); chosen gains justified by the table |
| S2-U25 | Straight-drive accuracy | Drive distance, straight-line correction | Manual (evaluation build) | 0.2 m and 0.5 m at 0.1/0.2/0.3 m/s, 0.5 m backward; 5 runs each; measure from a start mark | Each drive within ±20 mm; record mean, SD and heading change |
| S2-U26 | Turn accuracy | On-the-spot turns | Manual (evaluation build) | Left 90°, right 90°, U-turn 180°, left 360°; 5 runs each; measure the angle | Each turn within ±10° |
| S2-U27 | Stop during a drive | `stop()` priority, braking | Manual (evaluation build) | 0.5 m drive at 0.2 m/s, `stop()` after 1.5 s; 3 runs; measure from the start mark | Stops at once; robot rests within a few cm of where `stop()` was called |
| S2-U28 | Stall detection (wheel held) | STALL fault, 600 ms no progress | Manual (evaluation build) | 1 m drive; hold one wheel completely still once it moves | STALL fault ~0.6 s after the wheel stops; motors braked |
| S2-U29 | Encoder disconnection | STALL fault (no encoder progress) | Manual (evaluation build) | Unplug one encoder cable, start a 0.3 m drive | STALL fault ~0.6 s after the start; motors braked |
| S2-U30 | Drive timeout | TIMEOUT fault (move deadline = 3 × expected time + 2 s) | Manual (timeout-test build: deadline cut to 0.5 × expected + 0 s) | 0.5 m drive at 0.2 m/s on the floor, nothing in the way (expected 2.5 s, deadline 1.25 s); 3 runs. A real deadline cannot be reached on purpose: blocked wheels give STALL first, slipping wheels still count to the distance | TIMEOUT fault at ~1250 ms, well short of 0.5 m; motors braked; no STALL |

The PID step test (S2-U24), the evaluation tests (S2-U25 to S2-U29) and the
timeout test (S2-U30) used test-only firmware: the subsystem 2 code above plus
a scratch routine that runs the steps and prints the results. For S2-U30 the
only other change is the deadline (`TIMEOUT_FACTOR` 0.5, `TIMEOUT_MARGIN_S` 0
in `motion_controller.c`); the timeout check itself is unchanged. That routine is not part of the
repository; its output is in `logs/`.

## Results

† Run on the Buddy 2 working copy before it was committed; the commit that
adds this code is the one to quote. ‡ Kp 3.2, Ki 5 (final gains) and the
code-review fixes (stop priority, calibration abort, single encoder reader,
one report per command, timeout limit, packed-input checks).

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S2-U23 | † | 2503508 | FAIL | `logs/2026-10-21_S2-U23_calibration_fail_step1_stuck.png` | Step 1 never ended: the RP2040 microsecond timer was never started by the kernel. Fixed: `time_start()` in `motion_init()` |
| 2026-10-21 | S2-U23 | † | 2503508 | FAIL | `logs/2026-10-21_S2-U23_calibration_fail_top_speed_0.png` | Top speed 0: the PWM block was left in reset (`USE_PTMR` = 0). Fixed: reset released in `motor_control_init()` |
| 2026-10-21 | S2-U23 | † | 2503508 | PASS | `logs/2026-10-21_S2-U23_calibration_pass.png` | VALID: 2534 counts/turn (L 2533, R 2538), top speed 0.884 m/s, no inversions |
| 2026-10-21 | S2-U01–U04, U06–U10, U13–U20 | † | 2503508 | PASS | `logs/2026-10-21_unit_tests.png` | 17 passed, 0 failed; S2-U05 SKIP (calibrated) |
| 2026-10-21 | S2-U11, S2-U12 | † | 2503508 | PASS | Observed on the floor (no saved log) | Encoder checks passed, but the left turn measured ~85°: `TRACK_WIDTH_M` changed from 0.110 to 0.116; turns then measured ~90° |
| 2026-10-21 | S2-U01–U04, U06–U10, U13–U20 | † | 2503508 | PASS | `logs/2026-10-21_unit_tests_after_min_bit.png` | After the minimum-bit changes (fault bitmask, packed state, 20-bit counts): 17 passed, 0 failed |
| 2026-10-21 | S2-U01–U04, U06–U22 | † | 2503508 | PASS | `logs/2026-10-21_drive_tests.png` | First run with S2-U21/U22: 21 passed, 0 failed; turns measured ~90° |
| 2026-10-22 | S2-U24 | † | 2503508 | PASS | `logs/2026-10-22_S2-U24_pid_round1.png`, `logs/2026-10-22_S2-U24_pid_results.csv` | Round 1 (Kp 0.8–3.2, Ki 0–20): Ki = 0 never settles (13–24 mm/s short); overshoot rises with Ki; Kp 1.6 / Ki 5 best (3 %, 160 ms) |
| 2026-10-22 | S2-U24 | † | 2503508 | PASS | `logs/2026-10-22_S2-U24_pid_round2.png`, `logs/2026-10-22_S2-U24_pid_results.csv` | Round 2 (Kp 1.6–4.8 × Ki 5/10): Ki 5 better at every Kp; Kp 3.2 / Ki 5 chosen. Left wheel showed 7–8 mm/s error and > 2 s settling: later traced to a loose left motor wire |
| 2026-10-22 | S2-U25 | † ‡ | 2503508 | PASS | `logs/2026-10-22_S2-U25_eval_*.png`, `logs/2026-10-22_S2-U25-U26_motion_eval_cases3-13.txt` | 0.2 m and 0.5 m at 0.1–0.3 m/s and 0.5 m backward: every mean within ±3.3 mm, SD ≤ 1.2 mm, heading change 0 to −0.5°. 1.0 m: encoder 1001 mm (5 runs), not measured (no space) |
| 2026-10-22 | S2-U26 | † ‡ | 2503508 | PASS | `logs/2026-10-22_S2-U25-U26_motion_eval_cases3-13.txt` | 90° left/right, 180°, 360° (runs 1–3): measured means within 3.3°, encoder within 0.4–0.6°; angles read to about ±5° |
| 2026-10-22 | S2-U26 | † ‡ | 2503508 | FAIL | `logs/2026-10-22_S2-U25-U26_motion_eval_cases3-13.txt` | 360° runs 4–5 (and the square, later dropped): STALL at 619 ms, only the right wheel moving. Cause: left motor ground wire came loose (hardware). Stall detection worked as designed. Fixed by reconnecting the wire |
| 2026-10-22 | S2-U26 | † ‡ | 2503508 | PASS | `logs/2026-10-22_S2-U26-U29_eval_rerun.png` | 360° runs 4–5 re-run after the wire fix: encoder 360.6° / 361.0° |
| 2026-10-22 | S2-U27 | † ‡ | 2503508 | PASS | `logs/2026-10-22_S2-U26-U29_eval_rerun.png` | `stop()` at 281–282 mm; robot rests 10–11 mm later by encoder; heading within ±0.6°. Measured totals ~11 mm shorter than the encoder after this abrupt stop |
| 2026-10-22 | S2-U28 | † ‡ | 2503508 | Invalid run | `logs/2026-10-22_S2-U26-U29_eval_rerun.png` | No STALL: the wheel was only slowed, not stopped (1001 mm in 5399 ms, same as a free drive). The speed controller held the speed, as intended. Repeated below |
| 2026-10-22 | S2-U28 | † ‡ | 2503508 | PASS | `logs/2026-10-22_S2-U28_stall_pass.png` | Wheel held still: STALL at 819 ms (~0.6 s after it stopped), motors braked |
| 2026-10-22 | S2-U29 | † ‡ | 2503508 | PASS | `logs/2026-10-22_S2-U26-U29_eval_rerun.png` | Encoder unplugged: STALL at 619 ms, motors braked |
| 2026-10-22 | S2-U01–U04, U06–U22 | † ‡ | 2503508 | PASS | `logs/2026-10-22_drive_tests_after_review.png` | Final code: 21 passed, 0 failed; S2-U05 SKIP |
| 2026-10-22 | S2-U30 | † ‡ | 2503508 | PASS | `logs/2026-10-22_S2-U30_timeout_pass.png` | Timeout-test build, 3 runs: "drive/turn timed out" (TIMEOUT) at 1279 / 1259 / 1279 ms after 143 / 134 / 138 mm (encoder), heading change −2.1° to +1.0°; motors braked, no STALL |

For S2-U25 to S2-U29, `logs/2026-10-22_S2-U25-U29_motion_eval_3runs.csv` has
the three runs per case chosen at random (fixed seed) for the accuracy
evaluation.
