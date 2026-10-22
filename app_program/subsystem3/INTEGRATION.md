# Buddy 3 team-interface handoff

Buddy 2 + Buddy 3 now have an opt-in integrated runtime (`S3_MOTION=1`). It uses
the existing motion task/PID, a coherent feedback snapshot and a bounded controller
slice in `app_program/controller/s3_drive`. See [integrated operation](MOTION_INTEGRATION.md) for builds,
control requests, tests and limitations. Boot is disarmed; nothing starts moving
without explicit accepted sensor calibration and a start request.

The portable navigation core below still calls no motor APIs. The temporary controller
under `app_program/controller/` owns those calls. Normal/default and stationary `IR_BENCH` profiles retain
their previous startup boundaries. This does not integrate Buddy 1/4/5, MQTT or
obstacle avoidance, and does not establish full Week 6 or hardware compliance.

## Ownership and input contract

Include `line_navigation.h`. New public types use `s3_`/`S3_`; existing
`ir_reflectance` types are retained. One owner serialises processing and all
control calls. The context is not internally locked. Other tasks must deliver
requests to that owner through `s3_integration_submit()` in the integrated profile.

`s3_navigation_process(&nav, &frame, &odometry, &out)` replaces the previous
three-argument call. The odometry pointer may be NULL for diagnostics. It must
be supplied on every active processing call; NULL does not reuse old input.

- Sensors: raw 0..4095, enabled/valid flags, ADC errors and channel timestamps.
  The frame timestamp marks completion of sequential acquisition, not simultaneous
  measurements. Disabled channels remain unavailable. A floating disconnected
  wire can still produce plausible counts: validity does not certify wiring.
- Odometry: `s3_odometry_sample_t` contains signed centre displacement in metres
  (`double centre_m`), source timestamp, validity and counter-reset `epoch`.
  The producer increments/changes the epoch whenever its counter resets. Supply
  signed centre displacement, not absolute accumulated travel or either wheel alone.
  The portable core does not call motion getters or consume `motion_poll()`.
  The controller obtains `motion_get_feedback()` and forwards its odometry here;
  it alone drains ID-tagged motion reports, including cancellations. See the
  [future-controller contract](../controller/README.md).
- All timestamps use the same unsigned 32-bit microsecond clock. Subtraction
  handles wrap; intervals must be below half the clock range. Future/stale samples,
  non-finite distances and invalid flags are unusable. Repeated odometry snapshots
  are allowed while their original source timestamp remains fresh; do not refresh
  a timestamp without a new source measurement.
- The default maximum frame/channel gap and sample/odometry age is 30 ms,
  configurable with `max_sample_age_us`. Do not retimestamp stale inputs to pass it.

## Readiness, calibration and lifecycle

`init` starts `S3_RUN_INACTIVE`. Numerical calibration defaults (150/280) support
raw/normalised diagnostics but **do not count as accepted calibration**. There is
no navigation bypass and no automatic start.

For each sensor, explicitly collect known white/black samples with
`s3_calibration_begin` and `s3_calibration_add`. `s3_calibration_summary` reports
count, minimum, maximum and average without changing the active references. Each
label holds up to 65535 samples. `s3_navigation_apply_calibration` accepts at least
ten per label with strictly non-overlapping ranges. Missing samples, overlap and
equal references reject the candidate and retain the previous calibration and its
acceptance status. Accepted means are truncated to integer references, existing
hysteresis limits are retained, and affected detection state is reset. New samples
are required after acceptance. Calibration changes are allowed only while inactive.
There is no silent adaptation, serial calibration command or flash persistence.

`s3_navigation_readiness(&nav, now_us)` returns these bit flags:

| Flag / value | Blocker |
|---|---|
| `S3_READY_CHANNELS` / 1 | All three sensors are not enabled |
| `S3_READY_CALIBRATION` / 2 | At least one sensor lacks explicitly accepted calibration |
| `S3_READY_SENSORS` / 4 | Missing, invalid or stale sensor/frame input |
| `S3_READY_ODOMETRY` / 8 | Missing, invalid or stale odometry |
| `S3_READY_FAULT` / 16 | Fault is still latched |

Typical owner sequence (all return values must be checked):

1. Initialise once with validated configuration; enable only physically connected
   channels. Collect and apply calibration for all three sensors while inactive.
2. Optionally apply the full action table using `s3_navigation_set_mapping`.
3. Process fresh sensor and odometry input. Inspect readiness.
4. Call `s3_navigation_start(&nav, now_us)`. It rechecks readiness at call time,
   enters active only when every blocker is clear and resets detection state.
5. Process every frame and consume its output immediately. Starting does not itself
   provide a steering request; wait for fresh active output and line persistence.
6. `s3_navigation_end_run` returns a healthy context to inactive. It cannot clear a
   fault. Discard old outputs on any control change; outputs are per-call snapshots.

Only active, healthy TRACK processing emits actionable navigation events/steering.
`steering_preview` remains a diagnostic calculation; never send it to motors.
Inactive/faulted TRACK processing may decode diagnostic letters without actions.
`diagnostic`, `barcode_status`, calibration mask, readiness, lifecycle, rearming,
odometry availability and latched faults make the distinction explicit.

## Output, mapping and manoeuvre contract

Line error is -100..+100, negative means line left. Speeds are **metres per second**.
Both-black is a junction candidate, not a junction shape. Invalid steering means no
forward request, not an issued motor stop. The portable core never queues or executes turns; `s3_drive` owns that policy.

| Event | Meaning |
|---|---|
| `S3_EVENT_BARCODE` | Valid framed A-Z letter; inspect direction and action, including NONE |
| `S3_EVENT_JUNCTION` | Sustained both-black candidate |
| `S3_EVENT_LINE_LOST` | Persistent initial absence or subsequent line loss, once per loss episode |
| `S3_EVENT_LINE_REACQUIRED` | Stable acquisition after startup, loss or detector reset |
| `S3_EVENT_SENSOR_FAULT` | One-shot transition into fault-latched state; inspect latched reasons |

Line detection continues independently during barcode capture. Genuine front-line
loss is not masked by barcode activity. Ordinary line loss and malformed barcodes
are not critical faults, but line loss immediately inhibits forward steering.

`s3_navigation_set_mapping(&nav, map)` takes exactly 26 A-Z action entries. It is
inactive-only, validates every entry before copying and preserves the old table on
any rejection. NONE is valid. Defaults: A=left, B=right, C=straight, D=U-turn;
E-Z unmapped. A valid unmapped letter still emits an active barcode event with
NONE. Updates require neither recompilation nor context reinitialisation. This is
Buddy 3's FR11 receiving API; MQTT, parsing and remote validation remain external.

Lifecycle and processing mode are separate. `S3_MODE_SUPPRESSED` withholds steering
and navigation transitions, and disables barcode capture; raw/classified readings
and fault detection continue. Mode changes discard partial captures and reset line
persistence. Resuming TRACK during an active run anchors rearming at the latest
odometry distance and requires clearance plus entirely fresh barcode framing.
Nothing suppressed is queued or replayed. A new start always discards diagnostic
captures, so inactive results cannot become active commands.

## Distance rearming and latched faults

Timestamp-based element decoding is preserved; odometry does **not** determine bar
widths. Capture start/end distances are recorded when available; the end is the
sample where decoding completes (including trailing quiet-zone confirmation) or
capture is rejected. Distance validity flags/epoch prevent presenting mixed counter
epochs as one measurement. These fields remain until another capture or reset.

After success or rejection of an active capture, a new capture is blocked until:

- Both front sensors give a valid, present, non-junction line and the barcode sensor
  is WHITE continuously for `rearm_stable_us` (default 30 ms).
- Absolute **net** centre displacement from completion/rejection is at least
  `rearm_distance_m` (default 0.020 m), in either direction and in the same epoch.

These are provisional tuning values. Oscillating within the clearance distance
cannot accumulate progress. A full new quiet zone is required after rearming.
Diagnostic scans do not enforce distance rearming, and cannot generate actions.

During an active run, invalid enabled sensors, excessive frame/channel gaps,
unusable odometry or epoch changes latch a fault before outputs are made actionable.
`latched_faults` is a bitmask (SENSOR=1, TIMING=2, ODOMETRY=4, ODOMETRY_EPOCH=8);
additional causes accumulate while faulted, without repeated fault events.
`fault_mask` separately reports currently invalid enabled sensor channels.

`stop_requested` remains true on every faulted output, steering stays invalid and
navigation actions stop. Good readings alone do not resume operation. The owner
must process healthy required inputs continuously for `fault_clear_us` (100 ms
by default), then call `s3_navigation_reset_fault(&nav, now_us)`. Freshness is
rechecked at the call; merely waiting without samples cannot clear a fault.
A changed epoch becomes the new observed baseline while faulted, but interrupts
the healthy interval. Reset returns to inactive and a separate start is required.
Do not use context reinitialisation as a fault-reset shortcut.

The integrated `s3_drive` adapter consumes `stop_requested` and calls Buddy 2's
`stop()`. The portable core itself calls no brake function. If processing
itself stops, no new output can be generated: the opt-in motion-owner lease expires after 30 ms without a healthy integration
heartbeat, including during turns. The kernel/motion task must still be running. A false process return is an API/configuration failure, not a
valid output; the consumer must reject it. No physical stopping latency is claimed.

## Barcode assumptions and limits

Provisional Code 39 frame: quiet zone, `*`, inter-character white gap, one A-Z
letter, gap, `*`, quiet zone. Both scan directions are accepted; `reversed` describes
the ordering of the scanned bars, not the car's travel direction. No checksum,
extended alphabet or multi-letter payload is supported.

The 29-run buffer is bounded. Each character must contain six narrow and three
wide elements, wide/narrow mean ratio 1.8..3.5, each element within 35% of its
group mean. Inter-character gaps must be 0.5..1.5 local narrow units. Both quiet
zones require ten narrow modules by default. Narrow means must meet the configured
minimum (30 ms default) and be at least three times the largest observed sample
gap. Maximum run/frame duration defaults are 1 s/10 s; the streaming frame timeout
includes trailing quiet-zone confirmation. Invalid framing, timeout, input faults,
overflow or inadequate timing discard the frame. A new frame is required after a
result. Batch fixtures supply their own measured sampling gap and durations.

Tests include supplied A/Z PDF bar widths, separately extracted from the decoder's
table. Their PDF margins are not adequate for our quiet-zone requirement: fixtures
add explicit white time, and physical prints need sufficient blank material around
the bars. Relative character widths help with different scan rates; they do not
prove tolerance to acceleration within a character or reliable decoding while moving.


## Week 6 traceability and outstanding team decisions

Source: `Project/AAI07_Week_6_Design_Review.pdf`, pages 2-8 and assumption A4.
The submitted PDF is unchanged.

| Week 6 item | Status in this update | Limit / next owner |
|---|---|---|
| Sensor acquisition, calibration and line/barcode algorithms | Preserved and extended | Actual-track calibration and moving tests remain open (A4) |
| L/C/R on GP26/27/28 | Changed proposal | Current roles are offset barcode=GP26, front left=GP27, front right=GP28; team must agree mounting/roles; check GP28 VBAT jumper |
| CALIBRATE then TRACK/start gate | Implemented within Buddy 3 | All three accepted calibrations and fresh odometry required; calibration UI/storage not supplied |
| Encoder-distance capture input and REARM | Implemented within Buddy 3 | Distance metadata and net-clearance gate; element widths still use time; tune 30 ms/20 mm physically |
| Line-loss / manoeuvre suppression | Implemented within Buddy 3 | Independent front-line events; controller still owns recovery and turns |
| FR11 pre-run mapping | Receiving interface implemented within Buddy 3 | MQTT transport, parsing and remote validation await Buddy 1/controller integration |
| SENSOR FAULT / explicit reset | Implemented within Buddy 3 | Persistent request only; adapter now forwards stop and adds a motion-owner lease; physical latency unmeasured |
| NFR6 safe braking within control period | Awaiting integration | Adapter calls stop; detection, command acceptance and physical stop latency unmeasured |
| NFR3 Line+Barcode 5 ms | Awaiting scheduling/integration | Bench target remains 10 ms; unchanged kernel tick; diagnostics count actual gaps over 5 ms |
| NFR2 priorities / full-system concurrency | Awaiting integration | Bench sampler 16, reporter 17; existing kernel Wi-Fi priority 4; validate full-system priority/timing with team |
| NFR4 C conventions | Applied to touched code | Explicit braces, four-space indentation, bounded storage and namespaced APIs; no repository-wide Barr compliance claim |
| Shared Vehicle Controller, network non-blocking behaviour | Awaiting integration | Two-buddy controller slice only; no MQTT/obstacle/full-team controller |

Default normal firmware does not acquire new sensors or start navigation.
The separate S3_MOTION profile starts sensing and motion tasks but stays disarmed. The bench keeps
one priority-16/core-0 sampler and a priority-17 reporter with a short protected
snapshot; neither calls a motor API. Its measured timing counters will only become
hardware evidence after running it. Passing synthetic 5 ms/10 ms traces does not
certify scan speed, acceleration tolerance, junction recognition or timing compliance.
