# Buddy 5 — ultrasonic scanning, profiling, avoidance and recovery

45-degree route revision: 11 November 2026.
**Software implementation and simulated tests; no physical success claimed.**
Tribunal 1 changes still require implementation AND demonstration in Week 13.

## Hardware and activation

User-confirmed wiring: **HC-SR04 ECHO → GP0, TRIG → GP1; servo signal → GP12**.
Use USB CDC with `CRASH_REPORT=0`; the build rejects the UART conflict.
The servo description PH2.0-6P identifies a connector, not a calibrated angle/
pulse range. Confirm the actual servo supply, common ground, orientation,
1000–2000 us nominal mapping and safe 30–150 degree travel before use.
The HC-SR04 uses a 5 V supply; verify the ECHO path to GP0 is conditioned for
Pico's 3.3 V GPIO. Do not assume the connector supplies that protection.
References: [HC-SR04 manufacturer datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf),
[Pico datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf).

`S5=0` remains the default. To compile the integrated code with hardware activation
still disabled:

```sh
make -C build_make PICO_SDK_PATH=/Users/xavtsk/pico/pico-sdk \
  CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 \
  S3_MOTION=1 S4=1 S4_BENCH=0 S5=1 S5_HARDWARE_CONFIRMED=0 \
  TEST=0 IR_BENCH=0 DIAGNOSTICS=0 MOTION_SMOKE=0 -j8
```

The resulting `_s5_UNCONFIGURED` image refuses subsystem startup and cannot arm
navigation. Once hardware/geometry settings have been checked, explicitly set
`S5_HARDWARE_CONFIRMED=1`. That is configuration, not evidence of physical testing.
For network builds also set `WIFI=cyw43 NET=lwip MQTT=1` and use the actual local
Wi-Fi configuration. The hardware-enabled compact 30 cm Wi-Fi candidate was
built and flashed on 11 November 2026 at 18:58 +08. Post-flash telemetry and
valid ultrasonic readings were received; obstacle-avoidance validation remains
pending.

Normal local START still requires the existing IR readiness and braked-motion
checks, plus a fresh valid ultrasonic reading. GP21 STOP remains the controller
stop path. Enabling S5 does not automatically start the motors.

## Ownership and execution

- Buddy 5's priority-20/core-0 task owns ultrasonic acquisition and PWM6A servo.
- Echo capture uses PIO0 SM2 and words 24–31 at a 1 MHz counter. The original
  encoder SM0/SM1 and words 0–23 are preserved. The PIO assembly source is
  `ultrasonic_echo.pio`; jump addresses in the C image are relocated by 24.
- Wait for Buddy 2's actual valid publication before enabling SM2. No GPIO ISR
  is installed. A trigger is a nominal 12 us pulse; echo waiting is asynchronous.
- The existing `s3_drive` controller is still the ONLY motion requester and
  report consumer. The pure S5 state machine returns proposals, and the
  controller authorises the corresponding Buddy 2 commands.
- Source-timed observations cross the task boundary as a 25-byte packet under
  one priority-inheritance mutex. Scan requests use one identified token;
  cancellation takes precedence. No new unbounded queues or application heap.
- Buddy 3 remains the IR owner. Its normal action generation is suppressed during
  a detour, while health, timing, odometry and line observations stay active.

Flow: monitor → request STOP → fresh stopped/cancel acknowledgement → coarse
scan → fine scan/profile → choose route → bounded bypass → line recovery →
original-heading/stopped/healthy-line confirmation → normal following.

Coarse angles are 30/45/60/90/120/135/150 degrees. Fine scans refine both
near-obstacle to farther-return boundaries at 5 degree intervals, with no
duplicate coarse samples (at most 17 angles). Each angle settles for 250 ms;
the median of three valid readings is retained, including during side peeks.
Measurements remain at least 70 ms apart. Three consecutive invalid readings
fail the scan. Echo timeout is 25 ms; the whole scan remains limited to 10 s.
Neither no echo nor a failed scan is converted into open space.

The profile reports nearest return, near/far forward bounds, left/right
clearance and a lateral obstacle envelope over an assumed +/-15-degree bearing
uncertainty. It no longer shifts returns inward before estimating width.
Only the contiguous near-front component containing the centre return defines
this obstacle. A nearer foreign object or a missing farther-return bracket
makes the edge unknown; it is not silently discarded. Other observed objects
still reduce route clearance. This prevents a distant wall from automatically
becoming part of the obstacle's width. This is not an exact measurement of an
obstacle's physical width or proof that every surface reflects ultrasound.

## Route and recovery limits — physical tuning required

Configuration is in `s5_config.h`. Dimensions use the user's approximate
11 November measurements; margins and sensor uncertainty still need testing:

| Setting | Initial value |
|---|---:|
| Obstacle detection / moving emergency distance | 300 / 80 mm |
| Absolute profile obstacle cutoff | 500 mm, also checked against the centre-front slab |
| Robot half width / additional margin | 63 / 30 mm |
| Axle to front / rear / sonar | 125 / 55 / 75 mm |
| Available floor from the line, left / right | 450 / 450 mm |
| Maximum side-search travel | 500 mm |
| Maximum lateral detour | 700 mm |
| Recovery corridor half-width | 100 mm |
| Front IR sensor-to-axle offset | 135 mm, inherited from the course setting; verify mounting |
| Drive/turn wheel speed | 80 mm/s |
| Complete avoidance deadline | 120 s |

The axle extents total 180 mm, larger than the user's separate overall-length
estimate of 165 mm; the larger extents are retained. The turn envelope also
includes sensing heads protruding beyond the body.

Both sides are evaluated against their floor allowance, sonar clearance and
the room ahead. The route with the greater minimum reserve wins, with shorter
offset breaking ties. There is no left-first setting. The complete diagonal
is checked against the near corner using the rectangle's transverse separating
axis. Initial and final turns use exact swept-corner extents, rather than
requiring the whole circumscribed circle to finish turning before the obstacle.
For one side, the near-corner condition is `front_from_axle - edge >=
ceil(sqrt(2)*half_width) + 2*margin`. The initial pivot and floor limits are
checked separately. A too-close or blocked route still stops.

Detection is now 300 mm, the upper end of the requested 200--300 mm range.
Some wide obstacles can still be too close at 200 mm; lowering the trigger
alone does not make their diagonal path fit.

The route turns out 45 degrees, drives a diagonal of `ceil(offset * sqrt(2))`,
turns 45 degrees back to parallel, passes the obstacle, then returns through
the matching diagonal and restores the original heading. Side observations
begin immediately after the outbound turn to parallel; there is no blind
approach to the old far-front estimate. If the selected
heading is blocked before translation, the other prevalidated route can be
tried through two separately checked 45-degree turns, once.

A front-facing scan cannot see the hidden rear edge. While parallel, stop and
peek 60 degrees toward the obstacle. An occupied reading causes a 25 mm step,
bounded by `max_depth_mm`. Require two distinct fresh clear scans before the
final clearance move. The full opposite obstacle edge, beam uncertainty,
75 mm sonar offset and inward-turn corner clearance determine that move; encoder
travel must confirm it before the inward turn. This also handles a short
obstacle already behind the first side ray using a conservative rear bound.
It assumes a static, continuously reflecting obstacle; holes, acoustic misses
and wheel slip need physical validation. It does not claim exact hidden depth.

A clear side ray can also pass through empty space before the obstacle. Early
clear readings therefore cause another bounded step until actual encoder
travel puts the sonar beyond the observed far-front bound. Only then may two
fresh clear scans establish the rear bound. This gate is recomputed for the
chosen side and increased if the outbound diagonal travelled short.

For the measured car, the body needs 36 mm beyond the proven rear bound for
the inward turn and diagonal return, replacing the old 179 mm circle allowance.
This is **not** a 36 mm total move after the first clear echo: the forward-looking
60-degree sensor still needs its full-width/beam look-ahead allowance and
75 mm axle offset. Actual outbound undertravel increases the corner allowance.
The return diagonal also advances forward by approximately the lateral offset;
unknown remaining straight-track length cannot be guaranteed from sonar width.

The 450 mm floor allowance is an external course assumption. Ultrasound cannot
see a flat mat edge. Some wide or uncertain profiles will still be rejected
even when the route looks usable. Do not increase floor limits or shrink the
uncalibrated acoustic uncertainty merely to make a route pass.

Every move requires a new valid centre reading after the previous phase;
ongoing movement, including turns, stops on stale/invalid range or emergency distance.
If neither side has enough observed clearance, the current policy is STOP/NO_PATH.
It does not reverse blindly into unobserved space or repeatedly retry an unsafe
route. A reverse-and-reattempt policy requires verified rear clearance first.

Recovery requires sustained actual tape evidence (30 ms) near the predicted
original corridor, rejecting junction candidates. The corridor uses the
45-degree lateral projection of actual wheel travel and the IR offset; return
distance is based on the achieved outbound travel. With the current wide-spaced
IR mounting, both-white tracking alone is never accepted as recovery evidence.
If the return leg does not see tape, a bounded +20/−40/+20 degree search at the
predicted corridor restores the original heading. Failure to confirm tape
stops the robot. Before handing back, require fresh stopped feedback and a
healthy, non-junction line observation. This geometric/IR gate still needs real
original-line validation: another nearby line, wheel slip or a curved detour can
invalidate its assumptions. It is not a map-based proof of line identity.

A previously decoded barcode is retained; its pending timeout is paused during
the detour. Confirm course placement so a bypass does not skip its intended
junction. Unexpected barcode/obstacle overlap is not physically validated.

STOP cancels a scan and the detour. Motion faults, report loss, sensor faults,
odometry epoch changes and timing faults keep the existing stop/latch path.
A reset never automatically restarts navigation. An obstacle detected during a
barcode manoeuvre stops it rather than starting competing motion.

## Telemetry and diagnostics

The controller uses Buddy 1's existing protocol-v2 obstacle message (closest
range, conservative width, route), avoiding-obstacle state, and only the owned
NO_ECHO fault bit. Other buddies' flags are preserved by Buddy 1's protected
changed-mask update. Invalid profiles invalidate the cached obstacle section.
The full clearance/profile details and source timestamps are available through
`subsystem5_snapshot()`. The controller uses `subsystem5_read_snapshot()` to
separate a busy snapshot mutex from unavailable/corrupt data. Only BUSY may reuse
its owner-local 25-byte packed cache, for at most 250 ms since a successful read.
Original source timestamps stay unchanged, so moving range data must also pass
its 250 ms source-age check. A sonar publication up to 30 ms newer than the
controller's ADC-frame timestamp is allowed to avoid a concurrent-publication
false fault; larger future skew is rejected. A fresh invalid/no-echo observation replaces
the old cache and still stops motion; unavailable/corrupt data never falls back.
STOP/fault clears the cache, and START requires a successful fresh snapshot read.
No queue, task, shared lock or sensor-validity threshold was added. USB `[S5]` lines show FSM state/fault, route, front range,
validity, no-echo, scan ID, width and both clearances. Numeric state/fault meanings
are in `obstacle_avoidance.h`. Existing MQTT best-effort/drop behaviour and the
network-priority/timing issue have not been solved by adding Buddy 5.

## Validation and deliverables

The four code deliverables are implemented across `obstacle_scan`,
`obstacle_profile`, `avoidance_planner`/`obstacle_avoidance`, and `line_recovery`.
Drivers and controller wiring connect them into the integrated application.

```sh
make -C testing/subsystem5 test sanitize
```

See `testing/subsystem5/README.md` and `TEST_LOG.md` for test scope and open
physical evidence. Software tests/builds do not demonstrate the Week 13 robot.

11 November compact revision: strict host/controller/terrain/runtime,
profile edge/median and geometric body-sweep test builds compile.
Windows Application Control blocks execution
of the new host binaries, so no passing runtime result is claimed for this
revision. The integrated compact Pico firmware was flashed on 11 November 2026
at 18:58 +08; its obstacle manoeuvre has not yet been validated physically.
Servo-settling times now start after the actual
servo command, and stopped-feedback age uses the clock after the feedback read.
An independent mathematical check covered 216 return geometries and 75,972
sampled poses without intersecting margin-expanded obstacles; it does not
execute the production C state machine or model acoustic/encoder errors.
