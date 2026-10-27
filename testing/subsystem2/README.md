# Buddy 2 — Motion Control Using Motors and Wheel Encoders

Test cases and results are in [TEST_LOG.md](TEST_LOG.md); saved console output,
screenshots and data are in `logs/`.

## Current Mac/Linux software checks (25 October integration)

`make -C testing/subsystem2 test adapter regression sizes` runs current packed
boundaries, the teammate's production feedback and Buddy 2/3 drive tests, team
adapters and comparisons against imported `71193e2`. These mock hardware and
do not replace the teammate's saved physical evidence or new tests on this car.
The old GPIO-IRQ regression harness is historical; default tests no longer use it.
See [integration review](../../docs/INTEGRATION_2026-10-25.md).

## Unit tests (S2-U01 to S2-U22)

Automated in `app_program/subsystem2/subsystem2_test.c`.

```sh
make -C build_make PICO_SDK_PATH=... TEST=1 -j8   # -> mtk3pico_smp1_usb_cdc_wifi_test.uf2
```

Flash the image, open the USB console and reset the Pico. Each check prints
`[TEST] S2-Uxx PASS|FAIL`; the last line is the subsystem 2 summary.

| Setting in `subsystem2_test.c` | Effect |
|---|---|
| `S2_TEST_MOVE` = 0 (default) | Tests that never move the robot; S2-U11, U12, U21, U22 print SKIP |
| `S2_TEST_MOVE` = 1 | Also drives: forward 20 cm, left 90°, right 90°, back 20 cm. Robot on the floor, 30 cm clear |
| `S2_TEST_CALIBRATE` = 1 | Runs calibration (S2-U23) instead of the tests |

Leave both at 0 when committing.

## Calibration (S2-U23)

Build with `S2_TEST_CALIBRATE` = 1 and follow the `[CAL]` prompts: turn each
wheel exactly 10 turns forward by hand, keep both still for 3 s, then lift
the robot (each motor runs at full power). Copy the printed `#define` lines
into the CALIBRATE block of `motion.c` when the result is `VALID`.

## Manual hardware tests (S2-U24 to S2-U30)

These are measured by hand; the steps and acceptance criteria are in
`TEST_LOG.md`. Mark the robot's front edge before each move, measure the
distance with a tape measure (mm) or the angle with a protractor, and put the
robot back on the start mark. Record each run in the CSV and save the console
output in `logs/`.
