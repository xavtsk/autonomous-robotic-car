# Buddy 3: core software implemented, hardware validation pending

Two front sensors provide line observations; the offset sensor provides barcode
transitions. Algorithms are portable C with no GPIO, RTOS or Buddy 2 dependency.
The portable core remains independent. An opt-in `S3_MOTION=1` controller in
`app_program/controller/` now
connects it to Buddy 2, with explicit start/calibration, pending directions,
manoeuvres and braking. See [motion integration](MOTION_INTEGRATION.md). The
stationary `IR_BENCH` remains separate. Full-team arbitration is not implemented.

| Module | Responsibility |
|---|---|
| `ir_acquisition` | Named channels, enabled mask, timestamped raw samples and per-channel validity; injectable ADC/clock callbacks |
| `ir_sensor` | Single-owner kernel ADC adapter; configures only enabled GP26/27/28 inputs |
| `ir_reflectance` | Per-sensor normalisation to 0..1000 darkness and hysteresis, either raw polarity |
| `ir_calibration` | Explicit labelled white/black collection, range/average summaries, validated commit |
| `line_detection` | Signed line error, persistence, loss/reacquisition and junction candidates |
| `line_controller` | Limited proportional left/right speed proposals in m/s |
| `barcode_decoder` | Bounded Code 39 single-letter framing and direction detection |
| `line_navigation` | Readiness/run lifecycle, distance rearming, pre-run mapping, latched stop requests and manoeuvre suppression |
| `ir_bench_config` | Explicit sensor enablement and provisional per-sensor references |
| `subsystem3` | Sampling/reporting tasks and protected diagnostic snapshot |

## Development configuration

Roles follow the car's wiring (Week 6 L/C/R layout): left=GP26/ADC0,
barcode=GP27/ADC1, right=GP28/ADC2, set once by `S3_ADC_CHANNEL_*` in
`ir_acquisition.h`. Only
barcode is enabled in the default bench. Build with `tools/build-ir-bench.ps1
-Sensors 3` (or Make `IR_CHANNELS=3`) for three connected sensors. GP28's Robo Pico battery-sensing jumper must be checked before connecting
that input. Do not read GP29, which belongs to the Pico W radio.

White=150 and black=280 are development references for every channel, based on
approximate user-reported paper readings. They are **not measured track
calibration**. Each channel has independent references and classifier state.
Defaults do not authorise navigation: all three sensors need explicitly accepted
calibration, fresh input and odometry before an explicit start. The bench never
calls start. Calibration never adapts automatically while passing a junction or barcode.

The default line error is `(right_darkness-left_darkness)/10`, in -100..+100.
Negative means line left. This assumes the sensors straddle opposite line edges:
a centred line can give equal moderate darkness. A sum below 200 is treated as
insufficient line contrast, not as proof of a surface or junction shape. Both
sensors at darkness >=600 immediately inhibit steering; persistence of 30 ms
emits a junction candidate once. A 100 ms clear period rearms it. Loss persists
100 ms and reacquisition 30 ms. Forward proposals are withheld immediately when
signal is insufficient, during faults, and during pending/latched junctions.

Proportional steering starts at base=0.10 m/s, gain=0.06 m/s per full-scale error,
maximum=0.20 m/s; each output clamps to 0..maximum. These are unvalidated tuning
values. Buddy 2 retains ownership of wheel-speed PID. The stationary diagnostic
only prints these proposals.

## Scheduling and diagnostics

`IR_BENCH=1` bypasses other application subsystem startup and tests. `ir-sample`
runs at priority 16 on core 0, exclusively owns the ADC and processes one frame
per wake. A cyclic handler targets 10 ms; queued wake-ups are discarded rather
than replayed. Actual per-channel and frame timestamps come from the hardware
microsecond counter. No kernel tick settings were changed.

`ir-report` runs at priority 17 on core 0 and reports every 200 ms. It copies a
snapshot under a priority-inheritance mutex and prints after unlocking. Sampling
reads/processes outside the lock; only snapshot publication and counters are
protected. Cumulative event counts preserve visibility between reports; the last
barcode/action is diagnostic history, never a pending turn. Counters wrap at
32 bits. Console/kernel activity can still delay sampling, so 10 ms is a target,
not a deadline guarantee. Latest/max gaps, counts exceeding the Week 6 5 ms target and timing rejects are
displayed. This 10 ms bench does not meet the proposed 5 ms schedule.

Active faults latch a persistent stop request; recovery requires 100 ms of healthy
inputs, explicit reset to inactive and a separate restart. Diagnostic previews
are separate from actionable steering. See the handoff for the full contract.

Default maximum gap is 30 ms. A bad gap, stale/failed input or unknown barcode
classification resets affected detection state. Barcode elements must have
sufficient samples relative to observed gaps. No moving scan speed is certified.

`IR_BENCH=0 S3_MOTION=0` leaves both Buddy 3 lifecycle hooks as no-ops: no new ADC access,
tasks or navigation startup. The normal firmware otherwise retains Buddy 2's
existing startup behaviour. `IR_BENCH=1 TEST=1` is rejected by Make.

See [bench setup and tests](../../testing/subsystem3/README.md),
[test evidence](../../testing/subsystem3/TEST_LOG.md) and the
[public interface handoff](INTEGRATION.md). The general target `TEST=1` hook
reports SKIP because host tests do not establish physical performance.
