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
| bypass_course_test | Five configurations of the production line follower, drive controller, motion controller and obstacle bypass | Simulated wheels, IR, sonar and obstacle geometry |
| hardware_test | Actual PIO ultrasonic and PWM servo drivers | Register/GPIO/PWM access |
| runtime_test | Actual S5 sensing task, mailbox/cancellation, packed publication | RTOS, clock, echo and servo hardware |
| activation_gate_test | Same task/lifecycle compiled with hardware unconfirmed | No hardware activation allowed |
| telemetry_test | Actual vehicle-controller telemetry adapter | Buddy 1 report API captures |

Passing those is software evidence only. The existing Buddies 1–4, controller,
startup and course simulation suites are also regression checks.

## Bypass simulation

| Configuration | Successful missions | Failed missions |
| --- | ---: | --- |
| Standard | 16/17 | Tight right bend |
| Sonar half-angle 10° | 16/17 | Tight right bend |
| Sonar half-angle 20° | 16/17 | Tight right bend |
| Turns 10% long | 17/17 | None |
| Turns 10% short | 15/17 | Tight right bend and left wall |
| **Total** | **80/85** | **Five misses across two distinct layouts** |

On the tight bend, the line curves right at a 20 cm radius, starting 10 cm
after a 20 cm box. The final obstacle fault is `NO_LINE`. In the left-wall
case with short turns, the car regains the line but its outline overlaps an
obstacle sample. The model represents any such overlap by −1 mm, irrespective
of penetration depth.

Run from the repository root on a host with Make, GCC and Python 3:

```sh
make -C testing/subsystem5 bypass-regression
make -C testing/subsystem5 bypass-strict
```

Both commands run the same five profiles and write `build/buddy5/bypass-results.json`
and per-profile logs. The strict target returns a non-zero status if any mission
fails. The regression target compares each named case with its reference,
including minimum clearance, line reacquisition and failure reasons. An improved
case is accepted. A new miss or collision, lower clearance, missing or duplicate
case, process crash, timeout or invalid telemetry fails the check.

The reference is the 85 cases of course firmware `8131e1a` observed with
instrumentation confined to the simulator's result reporting. The original
console measurements matched in all 85 cases. Clearances are recorded before
display rounding, with a 10⁻⁹ mm numerical tolerance for host maths-library
differences. Any change from positive clearance to collision counts as a new
failure regardless of that tolerance. [Baseline values](bypass-baseline-8131e1a.csv)

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
