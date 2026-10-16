# Buddy 3 team-interface handoff

Status: core algorithms implemented and host-tested on `buddy3`, based on Buddy 2.
No shared Vehicle Controller, Buddy 1 merge or motor integration is included.

## Consumer contract

Include `line_navigation.h`. All new public types/enums use `s3_`/`S3_`; the
existing `ir_reflectance` calibration/classifier types are retained. There is no
copy of Buddy 1's controller types. One owner must access the navigation context,
its mode and calibration. Other tasks must send changes through that owner during
future integration; the context is not internally locked.

```c
s3_navigation_config_t cfg = s3_navigation_defaults();
/* Enable only physically connected sensors. This change is explicit. */
cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
s3_navigation_t nav;
if (!s3_navigation_init(&nav, &cfg)) { /* configuration error */ }
/* For each acquired frame, in the owning task: */
s3_navigation_output_t out;
if (s3_navigation_process(&nav, &frame, &out)) {
    /* Consume out.events on EVERY call, not just console reporting intervals.
       out.steering is a proposal; only the future controller may apply it. */
}
```

Inputs: timestamped raw 0..4095 counts, enabled/valid flags and ADC error codes.
Times are unsigned 32-bit microseconds, with wrap-aware subtraction; configured
intervals must stay below half the counter range. Use the same clock for channel
and frame timestamps. Frames mark the end of sequential acquisition; these are
not simultaneous ADC measurements. Disabled channels are unavailable, not faults.
A disconnected enabled analogue wire may still produce plausible counts: software
validity does not certify wiring health.

Outputs: per-sensor darkness 0..1000 and state, line validity/presence/error,
junction candidate, left/right speed proposal in **metres per second**, events,
fault mask and decoded letter/direction. Negative line error means line left.
Never use a steering request when `valid` is false; zero fields are not an issued
stop command. The future controller must decide whether to brake, recover or turn.
Any enabled sensor fault blocks a valid forward proposal.

| Event | Meaning |
|---|---|
| `S3_EVENT_BARCODE` | Complete framed letter; inspect `barcode` and `action` |
| `S3_EVENT_JUNCTION` | Sustained both-black candidate, not a classified junction shape |
| `S3_EVENT_LINE_LOST` | Previously acquired line has remained absent/ambiguous for the configured period |
| `S3_EVENT_LINE_REACQUIRED` | Stable line acquired after startup, loss or detector reset |
| `S3_EVENT_SENSOR_FAULT` | Newly faulted enabled channel(s); inspect `fault_mask` for current state |

The action table is `cfg.map[letter-'A']`. Defaults are A=left, B=right,
C=straight, D=U-turn; E..Z are unmapped (`S3_NAV_NONE`). Unmapped letters still
emit a barcode event. Replace the table before context initialisation. Reinitialise
only while the consumer is stopped; this deliberately resets detection state.
Buddy 3 never stores a pending turn or executes the table's actions.

Call `s3_navigation_set_mode(&nav, S3_MODE_SUPPRESSED)` during manoeuvres. It
suppresses navigation events and steering, while raw/classified diagnostics and
sensor-fault reporting continue. Decoded letters can still appear in diagnostic
outputs, but are not queued/replayed. Switching modes resets line/barcode state;
returning to `S3_MODE_TRACK` requires fresh persistence/framing.

## Explicit calibration

Create one `s3_calibration_collector_t` per physical sensor; call
`s3_calibration_begin`, then `s3_calibration_add` only for valid samples labelled
WHITE or BLACK by an explicit calibration session. Hold each known material at
its final mounting height. `s3_calibration_summary(&collector.white)` (or black)
returns availability, count, minimum, maximum and average without changing active
calibration. Each label accepts at most 65535 samples.

`s3_navigation_apply_calibration` requires at least ten samples per label and
strictly separated ranges; equal references, missing samples or overlap reject
the candidate and preserve the previous calibration. Accepted integer references
use truncated averages and preserve configured hysteresis limits. Applying them
resets classifier/detection state. This API does not provide an interactive serial
calibration command or persistent flash storage; startup references are configured
in `ir_bench_config.h`. No silent adaptation is performed.

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

## Remaining team/hardware work

Confirm Code 39 format and action meanings with the team/brief. Check final sensor
placement, GP28 battery-sensing jumper and all AO connections; calibrate on actual
track materials. Tune line thresholds, steering gains/speed, measure sampling gaps
and scan performance while moving, and validate junction handling. The future
Vehicle Controller must arbitrate motion requests, stopping, recovery and turns,
and adapt this interface to Buddy 2's lifecycle/watchdog. Host tests cannot validate
any of those physical behaviours.
