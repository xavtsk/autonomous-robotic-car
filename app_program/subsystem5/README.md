# Buddy 5 — ultrasonic scanning, profiling, avoidance and recovery

Implemented on `xavier/development` over `6225b04`, 27 October 2026.
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
Wi-Fi configuration. The compile-only image made during this session uses public
placeholder credentials and is not a deployment image. Nothing is auto-flashed.

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
- Source-timed observations cross the task boundary as a 23-byte packet under
  one priority-inheritance mutex. Scan requests use one identified token;
  cancellation takes precedence. No new unbounded queues or application heap.
- Buddy 3 remains the IR owner. Its normal action generation is suppressed during
  a detour, while health, timing, odometry and line observations stay active.

Flow: monitor → request STOP → fresh stopped/cancel acknowledgement → coarse
scan → fine scan/profile → choose route → bounded bypass → line recovery →
original-heading/stopped/healthy-line confirmation → normal following.

Coarse angles are 30/60/90/120/150 degrees; fine scans use 5 degree steps in a
bounded ±15 degree sector around the nearest coarse return. Each angle settles
250 ms, measurements are at least 70 ms apart, and invalid returns get at most
three attempts. Echo timeout is 25 ms; the whole scan is limited to 10 s.
Neither no echo nor a failed scan is converted into open space.

The profile reports nearest return, estimated forward extent, left/right
clearance and a conservative lateral obstacle envelope. Angular/beam uncertainty
inflates the observed lateral extent. This is not an exact measurement of an
obstacle's physical width or proof that every surface reflects ultrasound.

## Route and recovery limits — physical tuning required

Initial configuration is in `s5_config.h`, NOT a set of agreed physical tolerances:

| Setting | Initial value |
|---|---:|
| Obstacle detection / moving emergency distance | 350 / 150 mm |
| Profile obstacle cutoff | 700 mm |
| Robot half width / additional margin | 90 / 70 mm |
| Assumed maximum obstacle depth | 500 mm |
| Maximum lateral detour | 700 mm |
| Recovery corridor half-width | 100 mm |
| Sensor-to-axle offset | 0 mm: MUST measure and update |
| Drive/turn wheel speed | 80 mm/s |
| Complete avoidance deadline | 60 s |

A front-facing scan cannot see the hidden rear edge. **The configured maximum
depth and robot footprint must bound the actual demonstration obstacles.**
The route is turn out, move sideways, turn parallel, pass the bounded obstacle,
turn toward the original corridor, return, then restore the original heading.
Every move requires a new valid centre reading after the previous phase;
ongoing movement, including turns, stops on stale/invalid range or emergency distance.
If neither side has enough observed clearance, the current policy is STOP/NO_PATH.
It does not reverse blindly into unobserved space or repeatedly retry an unsafe
route. A reverse-and-reattempt policy requires verified rear clearance first.

Recovery requires sustained actual tape evidence (30 ms) near the predicted
original corridor, rejecting junction candidates. With the current wide-spaced
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
`subsystem5_snapshot()`. USB `[S5]` lines show FSM state/fault, route, front range,
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
