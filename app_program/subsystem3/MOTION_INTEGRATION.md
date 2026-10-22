# Buddy 2 + Buddy 3 integration

Updated against `origin/Buddy2` commit `a29f973` (22 October 2026). Its PIO
encoders, calibrated constants, PID settings, packed formats and report queue are
preserved. These upstream changes are included in the working tree; no branch
merge commit, flash or push was performed. The remaining local motion changes
are the feedback/lease contract described below.

## What is connected

`S3_MOTION=1 IR_BENCH=0 TEST=0` starts only Buddy 2 and Buddy 3 application tasks.
It initialises motor outputs braked, enables all three sensor inputs and stays
**disarmed**. The default build remains unchanged; `IR_BENCH=1` still starts only
the stationary sensor diagnostic. Conflicting test/bench/integration flags fail
at build time. No firmware was flashed or motor operated during development.

- `app_program/controller/s3_drive` passes valid active steering (m/s) to `motion_request_velocity()`.
- `motion_get_feedback()` provides a mutex-protected distance/source timestamp,
  validity, reset epoch, current motion state/fault, command ID, report-loss count
  and stopped acknowledgement.
  Reads do not refresh measurement timestamps or consume completion events.
- The temporary controller alone drains `motion_get_report()` every cycle, even
  while inactive. It matches completion to the accepted command ID and never uses
  `motion_poll()`. Unexpected cancellation, queue loss and wrong current IDs fault
  the run; an old completion cannot finish a newer manoeuvre.
- A mapped barcode arms one direction for the next persistent junction candidate.
  Two pending codes, an unmapped letter or an unassigned junction halt the run.
- Before manoeuvring, it requests stop and waits for a **newer** motion update
  reporting IDLE, applied brakes and at least 100 ms of encoder-observed
  stillness (`motion_controller_is_stopped`). A short invalid steering interval
  also waits for this acknowledgement before resuming. Physical stopping remains
  dependent on working encoders and requires hardware validation.
- Navigation capture is suppressed during a manoeuvre; sensor/freshness faults
  still stop it. On completion, tracking is restored with fresh framing/rearming.
  The car remains braked until the front line is reacquired. No blind line search
  or automatic driving out of an unknown junction is implemented.
- Persistent initial/subsequent line loss halts and requires explicit restart.
  An instantaneous invalid steering proposal brakes immediately; a short contrast
  interruption may recover before the persistent-loss event.
- Sensor faults, motion faults, stale feedback, rejected motion requests and
  manoeuvre/acknowledgement timeouts latch the adapter FAULT state. Stop takes
  precedence, pending directions are discarded and valid readings do not restart.

Development timeouts: pending direction 5 s, brake acknowledgement 500 ms,
manoeuvre 10 s, post-manoeuvre line reacquisition 1 s. Distance rearming remains
30 ms stable clear line + 20 mm net displacement. These policies/tunings need
team and track validation. Sensor roles remain barcode GP26, left GP27, right
GP28, which differs from the Week 6 L/C/R assignment. Check GP28's VBAT jumper.

## Small Buddy 2 changes

`motion.h/c` adds a non-consuming feedback snapshot and a navigation-owner
heartbeat. The motion task publishes distance and its original measurement time
under its existing mutex. Feedback is unavailable before its first update, after
initialisation/recalibration until another update, during calibration or after
encoder initialisation failure. Epoch changes on initialisation/recalibration.
Signed centre distance accumulates modular 20-bit encoder deltas, so a counter
wrap does not look like a large displacement and incorrectly rearm a barcode.
Distance starts at zero in each epoch. A monotonically increasing report-loss
counter exposes dropped reports immediately, without waiting for another report.
Existing scalar getters, movement APIs and packed telemetry format are retained.
The shared `monotonic_clock.h` starts the hardware clock for both motion and the
stationary bench without resetting a running counter or changing the kernel tick.

The navigation lease is disabled by default. While the adapter is running it
refreshes the lease after each healthy processing cycle. The motion task latches
STALE_COMMAND and brakes if the lease age exceeds 30 ms, also during DRIVE/TURN
where the old velocity-only timeout did not apply. This catches a blocked/dead
integration task while the motion task is still running; it is not a hardware
watchdog for a stopped kernel. Motion calibration requests are rejected while the
lease is armed. The latest Buddy 2 PID, motor settings and 10 ms motion period are
unchanged relative to `a29f973`. These limited changes should be reviewed by Buddy 2 before merging.

A positive `motion_is_calibrated()` means Buddy 2 has non-zero configured encoder
scaling. It does not prove the currently configured motor/encoder values were
measured on this car. Physical calibration remains necessary.

## Explicit control API

Include `app_program/controller/s3_integration.h` from a future command/UI task. It exposes a bounded,
thread-safe control queue and copied status; only the sampling owner touches the
navigation context. `s3_integration_submit()` returning true means queued, not
executed. Observe `command_sequence` changing and `command_ok` in
`s3_integration_status()` before sending the next dependent command. The queue
holds one pending command. STOP pre-empts/cancels queued work; normal commands
are rejected while STOP is pending. The status is a per-cycle snapshot.

Per physical sensor (BARCODE, LEFT, RIGHT), while disarmed:

1. Hold actual white material under that sensor at its final height. Submit
   `S3_CONTROL_WHITE` with that sensor ID. Wait until collection starts and
   `collecting` returns false (one second). Inspect white count/minimum/maximum.
2. Repeat with black and `S3_CONTROL_BLACK`.
3. Submit `S3_CONTROL_APPLY_CALIBRATION`. Check command success and that sensor's
   acceptance bit. Rejection retains the previous accepted calibration. Each
   collection replaces only its chosen label, leaving the other label intact.
4. Repeat for all three sensors. References are RAM-only and must be accepted
   again after reboot. Provisional 150/280 values never authorise a start.

Optional `S3_CONTROL_MAPPING` carries a full validated 26-entry table while
inactive. `S3_CONTROL_START` requires all three accepted calibrations, fresh valid
samples/odometry, Buddy 2 IDLE with acknowledged stillness and no fault. `S3_CONTROL_STOP`
ends a healthy run. `S3_CONTROL_RESET` attempts both fault resets but never starts
motion. Buddy 2 may reset before the next copied feedback reflects IDLE, so retry
a reset after another status update if required; persistent faults still reject.
A separate START is always needed after reset or a halted run.

Example request from an application task (not a USB terminal command):

```c
s3_control_request_t request = { .kind = S3_CONTROL_WHITE,
                                 .sensor = S3_SENSOR_BARCODE };
bool queued = s3_integration_submit(&request);
/* Poll copied status; after collecting and accepting all three calibrations: */
request.kind = S3_CONTROL_START;
/* Only submit START when the operator actually requests a run. */
```

The USB console is output-only in this repository: it currently discards received
characters. This update does not add a USB command parser, buttons or MQTT.
The application control API is wired through the owner; a user-facing
command source must call it before the integrated firmware can be operated without
a debugger/custom application task. Sending text to the existing USB terminal
will not start it. This deliberately avoids an automatic start or calibration bypass.

## Scheduling and ownership

The integration uses two S3 tasks (sampler/controller priority 16 and reporter
priority 17), one snapshot/command mutex and one 10 ms cyclic handler. They replace,
not supplement, the bench tasks in this mutually exclusive profile. Buddy 2 keeps
its priority-14 motion task/mutex/10 ms cyclic handler. ADC reads, navigation,
motion calls and console printing occur outside the snapshot lock. STOP is
processed before acquisition; a stalled ADC/control task is covered by the lease.

The integrated profile is the sole motion-request and completion consumer. Do not
run Buddy 2 calibration/movement tests or another controller concurrently. Future
obstacle/IMU/network arbitration belongs to one controller. See the explicit
[replacement contract](../controller/README.md). No Buddy 1 draft was imported. The default profile still builds all existing subsystems normally.

The shared kernel tick and priorities are unchanged. Measured sampling gaps and
counts above Week 6's 5 ms target are printed. A 10 ms target does not satisfy the
5 ms design target, and no physical stop-latency guarantee is asserted.

## Build and tests

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test-buddy3.ps1
powershell -ExecutionPolicy Bypass -File .\tools\build-s3-motion.ps1
```

Output: `build_make/mtk3pico_smp1_usb_cdc_s3_motion.uf2` (motor-capable after an
explicit accepted start). The separate stationary build script remains
`tools/build-ir-bench.ps1`; its default output ends `_ir_bench.uf2` (GP26 only).
Use `-Sensors 3` for `_ir_bench_3sensor.uf2` (GP26/GP27/GP28). Both are stationary,
never call navigation START and never initialise motion. Neither script flashes.

Host tests compile with warnings as errors. `test_drive.c` links the real Buddy 2
controller/PID/estimator and real Buddy 3 core/adapter, mocking only hardware and
the public request wrappers. It covers follow, all four manoeuvre mappings,
brake acknowledgement, one-shot completion, reacquisition, rejected commands,
line loss, direction expiry, unmapped letters, sensor/reset faults, explicit
restart and stopped-owner watchdog behaviour. `test_motion_feedback.c` additionally
compiles production `motion.c` with a mocked kernel/hardware boundary to verify
snapshot timestamps/epochs, calibration exclusion and the turn watchdog.
Existing A-Z/fixture, acquisition, calibration and navigation tests are retained.

Hardware testing, actual-track calibration, motor/encoder calibration, junction
geometry, safe speeds, moving barcode performance, obstacle avoidance, MQTT and
full-team scheduling remain outstanding. Software integration is not validated
vehicle navigation.
