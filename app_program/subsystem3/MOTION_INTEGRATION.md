# Buddy 2 and Buddy 3 motion integration

The integrated profile (`S3_MOTION=1 IR_BENCH=0 TEST=0`) boots disarmed.
Buddy 3 observes the line and decodes barcodes. A single controller consumes
its events and Buddy 2's encoder feedback. Buddy 2 supplies the wheel PID and
writes motor outputs. Buddy 1 handles map and telemetry messages. `S4=1` and
`S5=1` add terrain and obstacle inputs to the same controller.
[Controller contract](../controller/README.md)

## Motion requests and feedback

- `app_program/controller/s3_drive` passes valid active steering (m/s) to `motion_request_velocity()`.
- `motion_get_feedback()` provides a mutex-protected distance/source timestamp,
  validity, reset epoch, current motion state/fault, command ID, report-loss count
  and stopped acknowledgement.
  Reads do not refresh measurement timestamps or consume completion events.
- The controller alone drains `motion_get_report()` every cycle, even
  while inactive. It matches completion to the accepted command ID and never uses
  `motion_poll()`. Unexpected cancellation, queue loss and wrong current IDs fault
  the run; an old completion cannot finish a newer manoeuvre.
- A mapped barcode supplies the action for the next junction. The frozen
  pre-run map and the configured course options govern crossing and turning.
  An unmapped letter halts the run.
- Before a manoeuvre the controller requests braking and checks newer motion
  feedback for IDLE, applied brakes and encoder-observed stillness.
- During a turn, manoeuvre suppression discards partial barcode captures.
  The course configuration allows a bounded line search after a turn. The
  drive controller can reverse the sweep once. Losing the line after that
  halts the drive. Reacquisition indicates line presence. A nearby parallel
  marking can also satisfy the sensor test.
- Sensor faults, stale feedback, motion faults, rejected requests and
  timeouts can latch a fault. Reset and start are separate operator actions.

The wired sensor roles were confirmed on 27 October 2026: front left GP27
(Grove 6), front right GP26 (Grove 5), and offset barcode GP28 (Grove 7).
The default barcode rearm requires 30 ms of stable clearance and 20 mm of
net travel. [Sensor interface and rearming](INTEGRATION.md)

## Buddy 2 feedback and motion lease

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
lease is armed. The motion task is configured for a 10 ms period.

A positive `motion_is_calibrated()` means Buddy 2 has non-zero configured encoder
scaling. It does not prove the currently configured motor/encoder values were
measured on this car. The [Buddy 2 measurements](../subsystem2/DATA_FORMAT.md) give the calibration values and their basis.

## Explicit control API

The [integration API](../controller/s3_integration.h) exposes a bounded,
thread-safe control queue and copied status; only the sampling owner touches the
navigation context. `s3_integration_submit()` returning true means queued, not
executed. Observe `command_sequence` changing and `b_did_command_succeed` in
`s3_integration_status()` before sending the next dependent command. The queue
holds one pending command. STOP pre-empts/cancels queued work; normal commands
are rejected while STOP is pending. The status is a per-cycle snapshot.

Per physical sensor (BARCODE, LEFT, RIGHT), while disarmed:

1. Hold actual white material under that sensor at its final height. Submit
   `S3_CONTROL_WHITE` with that sensor ID. Wait until collection starts and
   `b_is_collecting` returns false (one second). Inspect white count/minimum/maximum.
2. Repeat with black and `S3_CONTROL_BLACK`.
3. Submit `S3_CONTROL_APPLY_CALIBRATION`. Check command success and that sensor's
   acceptance bit. Rejection retains the previous accepted calibration. Each
   collection replaces only its chosen label, leaving the other label intact.
4. Repeat for all three sensors. Button/API recalibration is RAM-only; reboot
   restores the built-in measured ranges, when enabled. Persist new measurements
   by updating `controller/builtin_calibration.h` and rebuilding. Provisional
   150/280 core defaults never authorise a start.

Optional `S3_CONTROL_MAPPING` carries a full validated 26-entry table while
inactive. `S3_CONTROL_START` requires all three accepted calibrations, fresh valid
samples/odometry, Buddy 2 IDLE with acknowledged stillness and no fault. `S3_CONTROL_STOP`
ends a healthy run. `S3_CONTROL_RESET` attempts both fault resets but never starts
motion. Buddy 2 may reset before the next copied feedback reflects IDLE, so retry
a reset after another status update if required; persistent faults still reject.
A separate START is required after reset or a halted run. GP20/GP21 and
the [dashboard command interface](../../docs/REMOTE_CONTROL_PROTOCOL.md)
submit requests to this owner. USB serial provides diagnostic output.

## Scheduling and ownership

The integrated sampler/controller (priority 16) targets 5 ms, the motion task
(priority 14) targets 10 ms, and the reporter (priority 17) publishes status.
The stationary bench uses a separate profile and a 10 ms wake period. Each
uses one sampling owner. ADC reads, navigation, motion calls and console
printing occur outside the snapshot mutex. The sampler processes STOP before
acquisition. The motion lease covers loss of the sampling task while the
motion task remains operational.

The diagnostics include maximum sampling gap and counts above the configured
period. The submission has no post-demo on-car timing or braking measurements.

## Build and tests

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test-buddy3.ps1
powershell -ExecutionPolicy Bypass -File .\tools\build-s3-motion.ps1
```

Output: `build_make/mtk3pico_smp1_usb_cdc_s3_motion.uf2` (motor-capable after an
explicit accepted start). The separate stationary build script remains
`tools/build-ir-bench.ps1`; its default output ends `_ir_bench.uf2` (barcode sensor only, GP28).
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

The [submission evidence](../../docs/CHANGE_REQUIREMENT_1_EVIDENCE.md) records
the integrated host results, course simulation, saved-waveform comparison
and Arm builds. The team completed the demo before dismantling the car.
