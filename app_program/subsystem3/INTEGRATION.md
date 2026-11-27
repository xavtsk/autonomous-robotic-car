# Buddy 3 team-interface handoff

In the integrated `S3_MOTION=1` profile, the controller passes Buddy 2's
timestamped encoder feedback to Buddy 3, consumes navigation events, and
requests motion. Buddy 2 owns the motor outputs. Buddy 1 supplies the pre-run
map and telemetry. The same controller coordinates Buddy 4 terrain and
Buddy 5 avoidance when enabled. The car boots disarmed. The stationary
`IR_BENCH` profile runs sensing and diagnostics independently.
[Integrated operation](MOTION_INTEGRATION.md)

## Ownership and input contract

Include [line_navigation.h](line_navigation.h). One owner serialises processing and all
control calls. The context is not internally locked. Other tasks must deliver
requests to that owner through `s3_integration_submit()` in the integrated profile.

`line_navigation_process(&nav, &frame, &odometry, &out)` replaces the previous
three-argument call. The odometry pointer may be NULL for diagnostics. It must
be supplied on every active processing call; NULL does not reuse old input.

- Sensors: raw 0..4095, enabled/valid flags, ADC errors and channel timestamps.
  The frame timestamp marks completion of sequential acquisition, not simultaneous
  measurements. Disabled channels remain unavailable. A floating disconnected
  wire can still produce plausible counts: validity does not certify wiring.
- Odometry: `line_navigation_odometry_t` contains signed centre displacement in metres
  (`float64_t centre_m`), source timestamp, validity and counter-reset `epoch`.
  The producer increments/changes the epoch whenever its counter resets. Supply
  signed centre displacement, not absolute accumulated travel or either wheel alone.
  The portable core does not call motion getters or consume `motion_poll()`.
  The controller obtains `motion_get_feedback()` and forwards its odometry here;
  it alone drains ID-tagged motion reports, including cancellations. See the
  [controller contract](../controller/README.md).
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
`ir_calibration_begin` and `ir_calibration_add`. `ir_calibration_summary` reports
count, minimum, maximum and average without changing the active references. Each
label holds up to 65535 samples. `line_navigation_set_calibration` accepts at least
ten per label with strictly non-overlapping ranges. Missing samples, overlap and
equal references reject the candidate and retain the previous calibration and its
acceptance status. Accepted means are truncated to integer references, existing
hysteresis limits are retained, and affected detection state is reset. New samples
are required after acceptance. Calibration changes are allowed only while inactive.
There is no silent adaptation, serial calibration command or flash persistence.

`line_navigation_readiness(&nav, now_us)` returns these bit flags:

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
2. Optionally apply the full action table using `line_navigation_set_mapping`.
3. Process fresh sensor and odometry input. Inspect readiness.
4. Call `line_navigation_start(&nav, now_us)`. It rechecks readiness at call time,
   enters active only when every blocker is clear and resets detection state.
5. Process every frame and consume its output immediately. Starting does not itself
   provide a steering request; wait for fresh active output and line persistence.
6. `line_navigation_end_run` returns a healthy context to inactive. It cannot clear a
   fault. Discard old outputs on any control change; outputs are per-call snapshots.

Only active, healthy TRACK processing emits actionable navigation events/steering.
`steering_preview` remains a diagnostic calculation; never send it to motors.
Inactive/faulted TRACK processing may decode diagnostic letters without actions.
`b_is_diagnostic`, `barcode_status`, calibration mask, readiness, lifecycle, rearming,
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

`line_navigation_set_mapping(&nav, map)` takes exactly 26 A-Z action entries. It is
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

`b_is_stop_requested` remains true on every faulted output, steering stays invalid and
navigation actions stop. Good readings alone do not resume operation. The owner
must process healthy required inputs continuously for `fault_clear_us` (100 ms
by default), then call `line_navigation_reset_fault(&nav, now_us)`. Freshness is
rechecked at the call; merely waiting without samples cannot clear a fault.
A changed epoch becomes the new observed baseline while faulted, but interrupts
the healthy interval. Reset returns to inactive and a separate start is required.
Do not use context reinitialisation as a fault-reset shortcut.

The integrated `s3_drive` adapter consumes `b_is_stop_requested` and calls Buddy 2's
`stop()`. The portable core itself calls no brake function. If processing
itself stops, no new output can be generated: the opt-in motion-owner lease expires after 30 ms without a healthy integration
heartbeat, including during turns. The kernel/motion task must still be running. A false process return is an API/configuration failure, not a
valid output; the consumer must reject it. No physical stopping latency is claimed.

## Barcode assumptions and limits

```text
Waiting history -> triggered recording -> bounded fit work
                                                |
                                                v
Navigation <- accepted decode <- quiet gate <- candidate
```

Provisional Code 39 frame: quiet zone, `*`, inter-character white gap, one A-Z
letter, gap, `*`, quiet zone. Both scan directions are accepted; `reversed` describes
the ordering of the scanned bars, not the car's travel direction. No checksum,
extended alphabet or multi-letter payload is supported.

The 29-run buffer is bounded. Each character must contain six narrow and three
wide elements, wide/narrow mean ratio 1.8..3.5, each element within 35% of its
group mean. Inter-character gaps must be 0.5..1.5 local narrow units. Both quiet
zones require ten narrow modules by default. Narrow means must meet the configured
minimum (25 ms default) and be at least three times the largest observed sample
gap. Maximum run/frame duration defaults are 1 s/15 s; the streaming frame timeout
includes trailing quiet-zone confirmation. Invalid framing, timeout, input faults,
overflow or inadequate timing discard the frame. A new frame is required after a
result. Batch fixtures supply their own measured sampling gap and durations.

Tests include supplied A/Z PDF bar widths, separately extracted from the decoder's
table. Their PDF margins are not adequate for our quiet-zone requirement: fixtures
add explicit white time, and physical prints need sufficient blank material around
the bars. Relative character widths help with different scan rates; they do not
prove tolerance to acceleration within a character or reliable decoding while moving.


## Week 6 design and submitted implementation

Source: `Project/AAI07_Week_6_Design_Review.pdf`, pages 2-8 and assumption A4.
The submitted PDF is unchanged.

| Week 6 item | Implementation | Evidence or limit |
|---|---|---|
| Sensor acquisition, calibration and line/barcode algorithms | Implemented | [Recorded calibration](../../testing/subsystem3/IR_CALIBRATION_2026-10-27.md) and [recorded-waveform replay](../../docs/CHANGE_REQUIREMENT_1_EVIDENCE.md). Maximum moving scan speed is unmeasured. |
| L/C/R on GP26/27/28 | Confirmed different wiring, 2026-10-27 | Barcode=Grove 7 GP28, front left=Grove 6 GP27, front right=Grove 5 GP26; check GP28 VBAT jumper |
| CALIBRATE then TRACK/start gate | Connected to the operator panel | All three channels need accepted calibration and fresh odometry. GP20 recalibration updates RAM until reboot. |
| Encoder-distance capture input and REARM | Implemented within Buddy 3 | The default clearance is 30 ms of stable line and white space plus 20 mm net travel. Element widths use time. |
| Line-loss / manoeuvre suppression | Implemented within Buddy 3 | Independent front-line events; controller still owns recovery and turns |
| FR11 pre-run mapping | Connected in the integrated profile | Buddy 1 validates four distinct letters. The controller freezes and copies the map at explicit start. |
| SENSOR FAULT / explicit reset | Implemented within Buddy 3 | Persistent request only; adapter now forwards stop and adds a motion-owner lease; physical latency unmeasured |
| NFR6 safe braking within control period | Stop request and acknowledgement connected | The host suite exercises the braking path. Physical stop latency on the submission revision is unmeasured. |
| NFR3 Line+Barcode 5 ms | Integrated sampler configured for 5 ms | The stationary bench uses 10 ms. Diagnostics record actual gaps. No post-demo hardware timing capture exists. |
| NFR2 priorities / full-system concurrency | Sampling owner at priority 16, reporter at 17, kernel Wi-Fi at 4 | [Task ownership](../../TEAM.md) records the configured priorities. Actual scheduling latency under load is unmeasured for the submission. |
| NFR4 C conventions | Applied to touched code | Explicit braces, four-space indentation, bounded storage and namespaced APIs; no repository-wide Barr compliance claim |
| Shared Vehicle Controller and network reporting | One control owner coordinates motion, navigation and enabled terrain/avoidance tasks | Buddy 1 publishes through a bounded outbox. Control and braking operate independently of MQTT delivery. |

Default normal firmware does not acquire new sensors or start navigation.
The separate S3_MOTION profile starts sensing and motion tasks but stays disarmed. The bench keeps
one priority-16/core-0 sampler and a priority-17 reporter with a short protected
snapshot. Both bench tasks run without motor control. The
[submission evidence](../../docs/CURRENT_STATUS.md) records the physical demo and
the scope of the post-demo software verification.
