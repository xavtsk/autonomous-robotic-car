# Line following: "tape between the sensors" profile — 27 Oct 2026

For Buddy 3 (and the team). Not yet run on the car.

## Why

On 26 Oct the car could not follow the line. The two front sensors are further
apart than the 18 mm tape, so they can never both sit on opposite edges as the
original Buddy 3 design needs. Centred on the tape, both read white, and the
original detector calls that "no line" and halts after 0.1 s (run 5:
`L=0 R=4 present=0` -> `line lost`).

## What the car does now (integration build only)

| Sensors (L, R) | Meaning | Action |
|---|---|---|
| white, white | tape between the sensors (centred) | straight |
| black, white | tape moved left | steer left |
| white, black | tape moved right | steer right |
| black, black for 30 ms | crossing | barcode action, or straight across if none |

* Steering error and junction rule are Buddy 3's, unchanged (`error = (R - L)/10`,
  both >= 600 for 30 ms). Steering gain raised for the car from 0.06 to **0.10**
  (`S3_COURSE_STEERING_KP`): the course curves measured about **14 cm radius**
  (27 Oct, circle fitted to the tape centre in a photo with a ruler), and with
  0.06 the tightest turn is about 13 cm. With 0.10 a full error stops the inner
  wheel (0 m/s is not a stall) and runs the outer at 0.20 m/s. Base speed
  0.10 m/s and the 0.06 m/s minimum wheel speed are unchanged.
* No "line lost" while following: the car always starts on the line, and the
  tiles' curves are expected to be within the car's tightest steering turn
  (see limits). GP21 is the stop if it ever leaves the line.
* A crossing with no barcode before it, or a STRAIGHT action, is driven
  straight across (was: halt "no direction"). Needed for a full lap of the
  practice course (counter-clockwise, barcodes on the car's left).
* LEFT/RIGHT: the car spins about the axle, which is behind the sensors. It now
  first drives on so the axle is over the crossing, then turns
  (`S3_SENSOR_TO_AXLE_MM`, **must be measured**; 0 = not yet).
* After LEFT/RIGHT/U-TURN a front sensor must have crossed the new line in the
  last 0.3 s of the spin (0.5 s accepted a wrong turn when the sensors are only
  40 mm ahead of the axle; 0.3 s rejects it and still accepts every correct
  turn in the simulator); otherwise the car halts with "line lost" instead of
  driving off beside the line.
* Barcode -> crossing limit 20 s (was 5 s; Y is about 1 m before crossing 2).

## Where

| File | Change |
|---|---|
| `subsystem3/line_detection.{h,c}` | new `between` config flag (default **false** = original); `touching` output |
| `controller/s3_drive.{h,c}` | `s3_drive_options_t` (defaults = original behaviour), straight across, advance-then-turn, after-turn check |
| `controller/course_profile.h` | the car's settings: `S3_LINE_BETWEEN`, `S3_CROSS_STRAIGHT`, `S3_SENSOR_TO_AXLE_MM`, 20 s, 0.5 s |
| `controller/s3_integration.c` | applies the profile |
| `testing/integration/course_test.c` | detector + options unit checks |
| `testing/integration/course_sim.c` | simulator: real Buddy 3 navigation + s3_drive + Buddy 2 controller/PID on modelled wheels and tape |

Buddy 3's own build and tests use the defaults and behave exactly as before
(all `testing/subsystem3` suites pass unchanged).

## Simulator results (sensor gap 22 mm, sensors 60 mm ahead of the axle)

Straight (incl. 2 mm off-centre start), 20 and 15 cm radius U-curves both ways,
S-bend, crossing straight across, LEFT/RIGHT/U-TURN at a crossing: all pass.
LEFT/RIGHT without the advance end beside the line and are halted by the
after-turn check, as intended.

Limits found (wheel model constants are guesses):

* With Buddy 3's gain 0.06 the tightest curve is about **13 cm radius** (12 cm
  and below: the tape escapes and the car drives off). With the course gain
  0.10 the simulator follows **10 cm** curves; straight-line weave grows by
  about 1 mm. Tiles: ~14 cm.
* The advance distance tolerates about +/-15 mm error.

Run it: `make -C testing/integration -f controller_test.mk test`
(or `build/controller/course_sim <gap_mm> <axle_to_sensor_mm> [steering_gain]`).

## To measure before car tests

1. Sensor head centre-to-centre (expected > 18 mm).
2. ~~Tile curve radius~~ done: about 14 cm.
3. Driven-wheel axle centre to the front sensor heads (along the car) -> `S3_SENSOR_TO_AXLE_MM`.
