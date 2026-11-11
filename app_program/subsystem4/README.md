# Buddy 4: GY-511 / LSM303DLHC

Implemented: I2C sensor configuration/readout, signed acceleration and magnetic
samples, level/still offset calibration, tilt, compass-derived turn rate,
encoder-assisted hump height, highest completed valid hump per run, motion/impact
classification, protected snapshots and bounded event queue.

## Wiring and mounting

Grove 2: black GND, red 3.3V, white GP2 SDA, yellow GP3 SCL. I2C1 only.
Board flat, pins upwards; sensor +X left and +Y rearwards. Vehicle axes are
(-sensor Y, sensor X, sensor Z). At rest expect vehicle Z approximately +1g.
A failed level calibration can indicate incorrect Z orientation.

## Isolated bench build

Use the existing Make toolchain and initialized Pico SDK:

```sh
make -C build_make S4_BENCH=1 S3_MOTION=0 MOTION_SMOKE=0 DIAGNOSTICS=0 TEST=0 IR_BENCH=0 WIFI=none NET=none MQTT=0 PICO_SDK_PATH="C:/Users/aiken/.pico-sdk/sdk/2.3.0" -j8
```

The image suffix is `_s4_bench`. Other application tasks are disabled. Keep the
car level/still for 100 accepted samples (at least two seconds). USB reports once
per second; pitch_x10 and roll_x10 are tenths of a degree. No encoder data exists
in bench mode, so height and moving/stationary classification remain unavailable.
Compass turn rate remains invalid until measured magnetic calibration is supplied.
No firmware has been flashed or physically verified by this change.

## Processing and integration

`terrain_monitor_process()` is portable and receives sensor samples plus coherent
odometry. The runtime explicitly selects the **right-wheel-only** obstacle used
on this car: the left wheel stays on the flat surface while the right wheel
crosses the bump. `S4_HUMP_GEOMETRY` selects this profile and
`S4_WHEEL_TRACK_M` supplies the wheel centre-to-centre spacing (currently 0.116 m,
matching the Buddy 2 configuration).

For this geometry, the estimated right-wheel elevation relative to the left is
`track_m * sin(roll) * cos(pitch)`. Positive roll means right side up. A 10 mm
right-wheel rise with 116 mm spacing produces approximately 4.95 degrees of roll
when pitch is zero. The detector requires forward wheel travel and a rise followed
by return to level before counting a hump. Live height is available during the
crossing; the completed peak is retained afterward. A stationary hand tilt is not
a completed hump. The pitch-based terrain label can remain "moving" while the
separate `on_hump` flag and live height report a right-wheel crossing.

The portable monitor defaults to `TERRAIN_HUMP_LONGITUDINAL`, which retains the
existing distance-times-sine-of-pitch method for a hump crossed by both wheels.
`terrain_monitor_set_hump_geometry()` selects the geometry explicitly; a new run
retains that selection. Do not use the right-wheel formula for a general slope,
both-wheel hump, left-wheel-only obstacle, or a lifted chassis.

Both estimates require physical checks against measured obstacles. Wheel spacing,
mounting, initial level calibration, tyre compliance, acceleration and filtering
affect accuracy. Invalid inputs, reverse travel, excessive dynamics and timeouts
discard an incomplete crossing. A successful sensor/telemetry check does not by
itself validate measured height.

Runtime `S4=1` reads Buddy 2's timestamped feedback and starts in the integrated
`S3_MOTION=1` profile. `controller/terrain_integration.c` runs inside the existing
sampling/controller task. It is the sole S4 event consumer; S4 never calls motors.

START requires a fresh calibrated IMU sample. After the normal navigation/motion
checks pass, the controller requests a new run token. Navigation continues sampling
while motors are held stopped until S4 publishes a sample for that token (maximum
500 ms). The run clears hump count/peak but retains stationary calibration.
`subsystem4_read()` copies the packed sample, 16-bit run token and saturated 16-bit
completed-hump count under one mutex. A reset request alone is not acknowledgement.

While driving, a sample older than 150 ms, sensor fault, loss of calibration,
unexpected run token, event queue overflow or impact candidate latches a controller
fault and brakes through Buddy 2. This also interrupts Buddy 5 scanning/avoidance.
STOP takes priority during reset waiting. Recovery requires healthy IMU data and
an explicit controller RESET, followed by START. No automatic restart occurs.
These are conservative integration policies, not instructor-approved numerical
thresholds. Impact candidates need physical false-positive testing. Tilt, climbing,
descending and turn-rate observations do not automatically change motor commands.

Buddy 1 receives completed-hump count and highest valid peak in millimetres on its
existing binary v2 hump topic. Only the IMU bit is updated in the shared fault mask;
other subsystem fault bits are preserved. Peak telemetry is unavailable until a
valid completed hump exists for the run, and a valid run maximum survives transient
sensor faults. Cumulative counts do not depend on successful event delivery.
If Buddy 1's outbox is full, the latest current-run count/peak remains pending
and is retried without waiting for queue space, including after STOP. It is
marked sent only when queued successfully; a new run supersedes an unqueued
previous-run report. Successful enqueue is not a broker delivery acknowledgement.
USB `[S4]` logs expose run ID, terrain state, validity flags, pitch/roll in tenths
of a degree, turn rate, count, peak and impact. The existing heading packet remains
unavailable: compass-derived turn rate is not an absolute heading measurement.

`subsystem4_set_magnetic_calibration()` accepts measured vehicle-axis hard-iron
bias and diagonal soft-iron scale. Identity coefficients must not be used as a
claim that calibration was done. Compass turn rate is an estimate, not gyro data;
motor magnetic interference must be measured. Stationary offset calibration does
not calibrate the compass or prove acceleration scale accuracy.

Thresholds in the processing modules are provisional engineering defaults. Impact
is a candidate event, not proof of collision. The controller response above is enabled only with S4=1. S4 remains off by
default unless explicitly requested.

## Validation status

Run `make -C testing/subsystem4 test sanitize`. The suite exercises portable sensor
processing, minimum-width packing, the production task/transport reset handshake,
and the production controller runtime with simulated hardware. The Buddy 5 suite
also tests an IMU impact during an active bypass. Firmware and software evidence
is recorded in `docs/BUDDY4_INTEGRATION_2026-10-28.md`.

No physical IMU, hump-height, impact, full-load timing or terrain validation was
performed by this integration. The terrain analysis report and measured accuracy
remain physical deliverables. Tribunal 1 changes must be demonstrated in Week 13.
