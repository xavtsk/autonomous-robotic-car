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
odometry. Height integrates distance times sine of pitch; this is a provisional
wheel-path estimate, not a guaranteed physical hump-height measurement. Vehicle
wheelbase, rear/top mounting, acceleration, wheel slip and filtering cause error.
Test on measured humps before declaring accuracy. Reject gaps, reverse travel,
large roll, obvious dynamics and implausible distance changes. Only a completed
ascent/descent with plausible return to level contributes to the run maximum.

Runtime `S4=1` in the normal profile reads Buddy 2's timestamped feedback. S4 is
not yet started in the S3_MOTION branch. That integration requires the existing
controller owner to call `subsystem4_reset_run(false)` at each accepted run start,
consume `subsystem4_event()` and publish snapshots through Buddy 1. Reset requests
are applied asynchronously by the sensing task. Readers must check flags and age.
A previous valid run peak survives transient read failures until reset.

`subsystem4_set_magnetic_calibration()` accepts measured vehicle-axis hard-iron
bias and diagonal soft-iron scale. Identity coefficients must not be used as a
claim that calibration was done. Compass turn rate is an estimate, not gyro data;
motor magnetic interference must be measured. Stationary offset calibration does
not calibrate the compass or prove acceleration scale accuracy.

Thresholds in the processing modules are provisional engineering defaults. Impact
is a candidate event, not proof of collision. No automatic safety/motor response
is wired yet. S4 is off in existing profiles unless explicitly requested.

## Validation status

All Buddy 4 source modules and bench app_main compiled to ARM objects with GCC.
Host tests compiled with -Wall -Wextra -Werror; Windows Application Control
blocked their execution. MSYS Make failed to start (access denied), so full
firmware linking/UF2 generation remains unverified. No hardware tests passed yet.
Run testing/subsystem4/host_test.c on an environment permitting test executables.
On-target TEST=1 includes S4-U01 through S4-U03, without a hardware pass claim.
