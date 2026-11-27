# Buddy 5 testing

> Current guide: [documentation index](../../docs/README.md), [baseline status](../../docs/CURRENT_STATUS.md), and [build/test commands](../../docs/BUILD_AND_TEST.md). Dated results below retain their original evidence limits.

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

## Physical measurement scope

The dated test log distinguishes host, register-level and physical cases.
Host fixtures provide echoes, servo angles and motion feedback. They cover
scan ordering, clearance decisions, return to the original line and response
to stale inputs. Physical echo accuracy, servo settling, clearance margins,
wheel slip and stopping latency require instrumented observations. The
submission contains no such measurements for the post-demo revision.
[Dated obstacle tests](TEST_LOG.md).

The hardware procedure covered GP0/GP1 signal levels and echo pulses, GP12
servo orientation, stationary scan timing and known obstacle dimensions.
The bypass cases checked that stopped acknowledgement preceded scanning,
tested both turn directions and blocked legs, and required recovery onto the
original line without a repeated barcode. Simultaneous Wi-Fi, motor and echo
faults formed the load case for STOP timing.
