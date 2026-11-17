# Buddy 5 testing

Run `make -C testing/subsystem5 test sanitize` from the repository root.
Tests use C11 warnings-as-errors; sanitizer runs use AddressSanitizer and UBSan.
They do not access a sensor, serial port, broker or robot.

| Test | Actual production logic exercised | Replaced boundary |
|---|---|---|
| host_test | Packed codecs, range conversion, coarse/fine scan, profile, plan, recovery, finite-state controller | Synthetic measurements and motion completions |
| controller_test | S5 + S3 controller/navigation + real Buddy 2 controller/PID/estimator | Encoder counts, PWM and sensor observations |
| terrain_controller_test | S4 arbitration + S5 bypass + S3 + real Buddy 2 motion controller | IMU/ultrasonic/IR observations, encoder counts, PWM; verifies impact cancellation/braking |
| course_clear_test | Upstream course simulator with S5 arbitration enabled, including left/right turn undershoot and overshoot recovery | Simulated wheels/IR and a fresh clear ultrasonic reading |
| hardware_test | Actual PIO ultrasonic and PWM servo drivers | Register/GPIO/PWM access |
| runtime_test | Actual S5 sensing task, mailbox/cancellation, packed publication | RTOS, clock, echo and servo hardware |
| activation_gate_test | Same task/lifecycle compiled with hardware unconfirmed | No hardware activation allowed |
| telemetry_test | Actual vehicle-controller telemetry adapter | Buddy 1 report API captures |

Passing those is software evidence only. The existing Buddies 1–4, controller,
startup and course simulation suites are also regression checks.

## Required physical records (NOT RUN by this implementation)

1. Verify GP0/GP1 signal roles and level protection; GP12 servo signal, supply,
   common ground, pulse centre/orientation and safe angle limits. Scope trigger
   pulse, echo width and servo PWM if available; check shared PIO/PWM operation.
2. Measure known distances across the useful range; include tilted/poorly
   reflecting targets, no echo, stuck-high/disconnected input and timed recovery.
3. While stationary, verify coarse/fine angles, servo settling, bounded retries,
   scan duration and profiles against measured obstacle positions/widths.
4. Measure robot footprint, sensor-to-axle distance and obstacle depth; update
   the configuration. Confirm clearance and braking margins at operating speed.
5. Record stop request → actual stopped ACK → first scan movement, proving
   detailed scanning did not begin while the car was moving.
6. Repeat LEFT and RIGHT bypasses, both sides blocked, blocked second leg,
   invalid/stale range during a drive, and local STOP at every phase.
7. Recover the original line, including slip/offset and failed recovery cases.
   Test nearby lines/junctions: a white-only frame must not establish recovery.
   Confirm no replayed barcode and correct retained-command/junction association.
8. Run with network load/reconnect and simultaneous motor/Wi-Fi/no-echo faults.
   Save fault-mask values, motion/sensor gaps, STOP timing and relevant telemetry.
9. Save the tested commit (or patch hash), build flags, geometry/calibration,
   measured results, state-transition log and video. Demonstrate Tribunal 1
   changes in Week 13; host tests are not a substitute.

Acceptance tolerances must be agreed from actual course geometry; the initial
configuration constants are provisional engineering settings. Do not label any
of the above PASS merely because the build or a mocked test succeeds.
