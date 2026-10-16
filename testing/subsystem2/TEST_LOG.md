# Subsystem 2 — test log

**Buddy 2:** Motion control using motors and wheel encoders. Owner: 2503508.

See `testing/README.md`. Unit tests S2-U01 to S2-U12 are automated in
`app_program/subsystem2/subsystem2_test.c` (`make TEST=1`); none have been
run yet.

- S2-U01 to S2-U10 never move the robot. S2-U05 runs only while the robot is
  **not** calibrated; S2-U06 to S2-U12 run only once the CALIBRATE values in
  `motion.c` are set. The others print SKIP.
- S2-U11 and S2-U12 drive the robot. Set `S2_TEST_MOVE` to 1 in
  `subsystem2_test.c`, put the robot on the floor with 30 cm clear ahead and
  room to turn, then reset the Pico.

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
| S2-U09 | A stale velocity command raises a fault | Statechart MOTION FAULT, 250 ms command timeout | Automated | `motion_request_velocity(0, 0)`, wait 400 ms without re-sending, poll, then `moveForward(0.1)` | State FAULT, fault STALE_COMMAND; move returns MOTION_ERR_FAULT |
| S2-U10 | `motion_reset()` clears a fault once the wheels are still | Statechart MOTION FAULT -> IDLE | Automated | After S2-U09, wait 300 ms, `motion_reset()`, poll | Reset returns true; state IDLE, fault NONE |
| S2-U11 | Straight drive reaches its distance | Straight manoeuvre, encoder distance | Automated, robot moves (`S2_TEST_MOVE` = 1) | `moveForward(0.20)`, poll until COMPLETE (10 s limit), compare `robot_distance_m()` before and after | COMPLETE; distance 0.20 m ± 0.02 m |
| S2-U12 | Left turn on the spot reaches its angle | Left turn, encoder heading | Automated, robot moves (`S2_TEST_MOVE` = 1) | `turnLeft(90)`, poll until COMPLETE (10 s limit), compare `robot_heading_deg()` before and after | COMPLETE; heading change 90° ± 10° |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
