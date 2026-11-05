# Buddy 3 stationary diagnostics and software tests

## 5 November 2026: raw-signal architecture integrated, experimental image built

The architecture branch now uses one reader in navigation:
`barcode_signal.c` preserves raw 12-bit contrast and extracts edges,
`barcode_pattern.c` classifies black/white widths independently, and
`barcode_frame.c` validates timing, geometry, markers and actual quiet margins.
The legacy decoder remains available for standalone comparison tests but does
not emit navigation commands in this image. ELF symbols confirm the new signal
reader is linked and the old streaming/recording entry points are absent.

Input polarity comes from calibration. A three-sample median rejects isolated
spikes; a paper-noise estimate freezes when capture starts. Local prominence
finds peaks/valleys without pruning to fifteen bars. A local shallow-dip rule
merges plateau fluctuations independently of any letter or expected bar count.
Every consecutive fifteen-bar window is eligible for exact marker validation;
earlier retained marks still interrupt its leading quiet. Drifted approach
history is discarded conservatively, never converted into extra white time.

Frame width ratios use an uncertainty interval based on the observed sample
gap, rather than treating optical width estimates as exact printed dimensions.
The allowed interval must overlap 1.8..3.5; exact patterns and the separate
colour-class separation checks still apply. Quiet requires ten estimated local
modules plus boundary/sample allowance. A valid pattern waits for real trailing
white; invalid samples or timing faults cancel it. Mapping, physical START/STOP,
rearm and controller fault handling retain their existing ownership.

Validation performed:

- 17,727 pure pattern checks and 2,005 frame-validation checks passed.
- 2,539 raw-stream scenarios passed, including 1,560 randomized A-Z scans with
  jitter, faint bars, noise and broad-bar dips: no wrong letters.
- 540 recorded raw-waveform checks passed across timestamp-rounding extremes,
  ADC polarities and explicitly labelled padding. Refusal is permitted for
  ambiguous/noisy recordings; the suite also requires named positive cases.
- Navigation/core and fresh drive integration passed. Eleven controller
  integration executables passed the final stack-change run; Windows
  Application Control blocked four (`runtime_ir_debug`, `map_freeze`, `course`,
  `bcap_wire`), plus mapping and motion-feedback reruns. Those blocked checks
  are incomplete, not passes. All fifteen controller executables passed the
  earlier run before the task-stack allocation change.
- Wi-Fi/MQTT firmware build and startup/portable-object profile checks passed.
  No new firmware has been flashed or physically validated.

The latest six complete small-A recordings do **not** establish a higher
success rate for this architecture. Four retain a preliminary low-contrast
feature and fail leading quiet: 18:54:45, 19:00:21, 19:01:47 and 19:04:13.
For example, 18:54:45 has about 467 ms clear interval but needs about 962 ms;
19:00:21 has 439 ms but needs 915 ms. The raw traces cannot determine whether
those features are ink, paper variation or motor interference. The 18:57:01
and 19:04:59 clips pass width/leading checks and need roughly another 42 ms
and 28 ms of trailing white. They decode with explicitly synthetic tail
samples. None of these six unchanged clips emits a complete result in the
current replay. Several older recordings that the legacy padded reader could
recover still refuse. This is an experimental candidate, not a demonstrated
replacement with better physical reliability.

Run `tools/test-buddy3.ps1`, then
`python testing/subsystem3/test_barcode_signal_replay.py`. POSIX equivalents are
`make -C testing/subsystem3 test drive feedback replay-signal`. The old
`test_barcode_replay.py` still tests the unchanged legacy reader; its positive
results are not results for the new architecture. Historical exact trace/bench
comparisons intentionally detect changed behaviour and are not a functional
equivalence release gate for this rewrite.

Before a physical trial, use a straight path on uniform white paper, sufficient
clear approach/exit space, and let the full BCAP upload finish. A stationary
raw comparison with motors off/on can distinguish paper/optical effects from
motor interference. The remaining signal assumptions and task timing need
physical measurement; do not interpret synthetic padding as that measurement.

## Earlier checkpoint: separate signal, pattern and frame validation

The deployed baseline is preserved on `main` at `46725c7`. Development of its
replacement starts on `codex/barcode-signal-architecture`. The first stage is
`app_program/subsystem3/barcode_pattern.c`: a pure duration-pattern classifier,
independent of capture, ADC normalization, peak extraction and navigation.
The live reader still uses `barcode_decoder.c`; the new module is not linked
into the firmware yet.

Each nine-element symbol classifies black bars and white spaces independently.
Each colour's cutoff is the greater of its min/max midpoint and 1.5 times its
minimum width. This follows the separate-colour approach used by
[ZXing-C++](https://github.com/zxing-cpp/zxing-cpp/blob/master/core/src/oned/ODRowReader.h)
and described in [US5352878A, Figure 14](https://patents.google.com/patent/US5352878A/en).
It avoids fitting an optical bias or assuming that narrow black and white
durations must be equal. The implementation is independent and does not copy
ZXing-C++'s image-specific spread guards.

Only exact Code 39 patterns are recognized. Application frames require exactly
29 positive widths matching `*A-Z*`, including both markers, in one orientation.
The closest narrow/wide classes must differ by more than twice the supplied
maximum sample interval. That separation guard is experimental, not a Code 39
standard tolerance or a guarantee against noisy waveforms.

Run the new stage alone with `make -C testing/subsystem3 pattern`, or run it
alongside the existing host tests with `tools/test-buddy3.ps1` on Windows.
`test_barcode_pattern.c` covers 17,727 checks: all 44 symbols, A-Z in both
directions, scales, width ratios, arithmetic boundaries, malformed patterns,
and fixed measured widths from A/Z/F recordings. Of 15,600 distorted synthetic
frames, 15,495 classify correctly and 105 refuse; none produces a wrong letter.
The fixed widths from the 18:22, 18:54:45 and 19:04:59 A rejections now classify
as A. Those are pattern results only, not successful whole-capture replays or
new physical scans.

The remaining architecture work is deliberately separated into these layers:

1. Preserve raw ADC contrast and timestamps through capture. A new raw-input
   API must explicitly declare its 0..4095 range and polarity; the existing
   darkness API's 0..1000 contract must not silently change. Keep storage bounded
   and account for the packed representation required by CR1.
2. Extract edges and locate candidate start markers within the recording, so a
   small preliminary bump need not invalidate every subsequent frame. Searching
   must not discard real ink and then credit that ink as white quiet time.
3. Validate actual leading/trailing quiet, inter-character spaces, sample
   freshness, duration and contrast before accepting a pattern. These checks
   are absent from the pure pattern API; even implausible nonzero gap durations
   are intentionally not rejected there. An OK pattern cannot drive the car.
4. Select the new reader explicitly in integration, preserve command mapping
   and fault behaviour, replay complete captures including negative controls,
   then build and physically validate it. Avoid concurrent readers where the
   first result hides which architecture emitted the command.

Six complete recordings from the latest ten-trial set are retained with their
original hardware outcomes in `fixtures/barcode`. The two failures with an
extra preliminary region remain unresolved by this pattern-only stage. Firmware
build, flash and new physical success-rate measurements are still outstanding.

## 5 November 2026: recover faint bars without selecting a known letter

Four complete small-A captures lost narrow bars before width validation. Their
weak peaks were near 10..25 normalized units, while broad bars reached hundreds.
Lowering prominence alone admitted plateau wiggles that displaced real bars.
In the two 16:04 captures, a weak leading shoulder also shortened one measured
white gap enough to fail the existing width checks.

The primary learned-prominence extractor is retained. Both its complete
candidate and the bounded recovery candidate are checked when paper is near zero
(0..5) and frozen prominence is at most 15. A primary count of 29 does not prevent
recovery when those widths fail validation. Two valid interpretations must agree
on letter and orientation; disagreement rejects, while agreement retains the
more conservative trailing-quiet requirement. Recovery groups readings above paper+5 into exactly
15 complete ink regions, bridging dips of at most 30 ms. It neither prunes to
15 regions nor chooses regions using the expected letter. Weak regions use the
half-height core connected to their maximum; strong bars keep outer crossings.
Marker, payload-width, correction, sampling and quiet-zone rules are unchanged.

At recording start, a contiguous low-band paper prefix can provide the noise
estimate before faint bars or a rising shoulder contaminate it. The prefix must
last at least `quiet_modules * min_narrow_us` (300 ms with current settings).
Otherwise the earlier running estimate is retained. This does not skip noisy
samples to manufacture a clean interval; alternating paper readings remain
subject to the measured noise gate.

Eligible recordings use the 5-unit band for live quiet/turn monitoring as well,
so a long sequence of faint bars cannot look like blank paper. The pre-trigger
ring separately accounts for continuous low-band white before its oldest entry:
a faint mark that has fallen out of the ring must still interrupt leading quiet.
No additional record storage, global calibration change, sampling-rate change or
speed change is required. The profile remains 100 Hz and 0.04 m/s.

The four raw-ADC recordings decode A at midpoint reconstructed timing with
explicit synthetic quiet padding. This is software evidence, not four hardware
successes: three saved streams end before the reader finishes waiting for white.
The quantized darkness/4 export can lose enough detail to reject a trace that
its raw ADC readings recover. Timing, quantization and reversed-waveform variants
are checked for wrong letters, without claiming universal noise robustness.

The evening hardware captures include a successful A and Z, plus three further
A rejections. Raw replay now recovers the 18:24 and 18:28 failures: the earlier
noise estimate and the count-only recovery decision had blocked the correct
interpretation. The 18:22 raw trace remains a rejection because one narrow white
gap is about 2 ms below its strict minimum. Its quantized preview can pass; that
does not justify relaxing the raw width check. Fixture documentation preserves
each original hardware result and labels all synthetic quiet padding.

Coverage includes 781 synthetic faint-bar/noisy-paper checks across A-Z and both directions,
the existing 3000 jittered-frame/no-wrong-letter checks, and 552 saved-waveform
replays. Run `tools/test-buddy3.ps1`, followed by
`python testing/subsystem3/test_barcode_replay.py`. These are software checks;
the later physical results below remain the deployment reliability evidence.

The evening Wi-Fi firmware build and startup-profile check passed, as did all
15 controller integration executables. These host/build results do not establish
successful physical operation of the revised decoder.

The evening image was copied to the Pico's RPI-RP2 bootloader at approximately
18:51 SGT. UF2 SHA256:
`DF4B0BC62A5C797B3D9DC2D40235A2DAB1F11C4871D0729593AAC3174BE2AFD8`.
Fresh post-reboot controller telemetry was braked, drive state 0, with no motion
fault and a 10.31 ms sample gap. The B=forward, D=left, Y=right, C=U-turn map
received a fresh OK reply at 18:51:49 SGT. A and Z are therefore unmapped and
halt on a successful decode. The map requires four distinct letters rather than
blank direction slots, and must be resent after reboot. No remote START was sent.

The user's next ten small-A trials (18:53:51 to 19:04:59 SGT) produced five A
decodes and five rejections. Four failed waveforms are complete: two contain a
tiny preliminary bump counted as a sixteenth region; two retain all fifteen
bars but the same corrected payload white gap is about 46 ms, below its roughly
51 ms minimum. The fifth failure uploaded only six samples, so its cause is
unknown. Five of the eleven post-flash exports, including an earlier success,
were incomplete; starting a new run clears the unfinished capture upload.
All summaries report zero timing errors and overflow. This baseline is useful
for comparison, but is not a reliably solved small-barcode reader.

## 3 November 2026: retain the outer edges of uneven bars

The 21:27:37 shifted F run still reported zero decodes and one rejection, with
no timing error or overflow. Unlike the earlier F captures, all 15 real bars
survived extrema trimming. A wide closing-marker bar dipped below its local
midpoint, rebounded, then reached white. Choosing the crossing nearest the
weighted estimate ended that bar at the internal dip (about 220 ms), making
the adjacent white gap too wide (about 365 ms).

The new reader chooses the first rising and last falling midpoint crossing
within each retained pair of extrema. It keeps the complete merged bump,
mirrors on scan reversal, and does not select edges using a known letter.
The weighted estimate remains only a fallback when no crossing exists.
This changes no thresholds, marker tolerances, width corrections or actual
quiet-zone requirements. The earlier experimental contrast-normalised and
shallow-first extrema cleanup changes remain unshipped.

The shifted F fixture now passes both strict markers and the F payload in
offline analysis. Its saved stream alone still lacks enough measured quiet
to emit a result: explicit synthetic leading/trailing margins are required in
the regression. The capture starts 169 ms after the run began and ends at
the previous reader's rejection, so omitted white must not be treated as
observed samples. Earlier poor-contrast F captures still reject.

The new Wi-Fi build was flashed at approximately 21:39 SGT, SHA256
`F3E1510A9D6F17C6DD61142ACFB550AD736607DCE9D7F473BC6AC353101CCFB7`.
Post-reboot Wi-Fi controller telemetry was fresh, braked and fault-free.
At the user's request, the runtime map is now B=forward, A=left, Y=right,
C=U-turn. The first map request timed out; an identical retry received an OK
reply at 21:40:40 SGT. A now queues LEFT for a junction, rather than halting
as an unmapped letter. This runtime map must be supplied again after reboot.
Physical barcode/turn validation of this firmware is still pending.
It retains 100 Hz, 0.04 m/s and the wider-gap settings below. Saved-waveform
coverage is now 120 checks. New synthetic regressions cover 104 broad dips
inside wide bars and 624 brief spikes in white gaps across A-Z and both scan
directions; the latter must never emit a wrong letter. Existing quiet,
grey-shelf, scratch and 3000 jittered-frame checks remain in the suite.
The complete host suite, 120 saved-waveform checks, controller integration
suite and Wi-Fi firmware profile check passed for this build. The new wide-bar
dip regression fails against the earlier crossing rule and passes this fix.

## 3 November 2026: wider printed gaps

The wider-gap Wi-Fi image keeps 100 Hz sampling, 0.04 m/s cruise, start rise 50
and minimum prominence 15. It increases the post-trigger recording deadline
from 10 to 15 seconds. At 0.04 m/s a 19 mm wide element takes 475 ms; a
nominal 2:1 frame plus trailing quiet can take about 11.4 seconds. The
one-second individual-run limit and 30 ms sampling safety limit are unchanged.

The letter may contain at most one narrow white gap up to 150% of its narrow
mean, instead of 135%. This addresses the measured Z payload's approximately
149.4% gap. All sentinel widths, black bars, wide gaps, lower bounds and other
narrow gaps keep their original checks. Edge interpolation also uses bounded
32-bit arithmetic with the same integer rounding, removing expensive 64-bit
divisions; this is not a measured guarantee of the sampling deadline.

`test_barcode_wide.c` covers 468 synthetic scans: A-Z, both directions,
8/14/19 mm wide elements and 2:1/2.5:1/3:1 ratios, plus malformed widths,
truncation, real quiet and the longer deadline. These ideal geometries do not
prove that the user's different papers are scaled copies. The recorder,
varying-correction and 108 existing saved-waveform checks also pass. Full host
and integration runners encountered Windows Application Control launch
failures; those incomplete runs are not test passes.

The complete 21:00:17 capture of the user-confirmed F paper still rejects.
Its first episode has jagged thick bars and a faint bar peaking near 42/1000.
The current extrema trimming drops that real bar in favour of a plateau wiggle.
Even diagnostic cleanup that retains it leaves a distorted closing star that
fails strict width validation. A broad contrast-normalised cleanup was not
adopted because it produced a wrong letter in the recorder's 3000-frame
width-jitter/noise stress test.
The bounded gap/deadline changes do not establish a fix for this F waveform.
Use at least 10 cm clear white before the first bar and a straight path.
The user cannot rotate the paper because its edge catches the sensor;
comparison scans instead use a shifted parallel path in the same orientation.

This image was flashed at approximately 21:17 SGT on 3 November, SHA256
`E71CD50732805B4EA8F0A6056E7902A998F184323E2A914094877B049FC386C1`.
Fresh post-reboot wireless telemetry showed the car braked, drive off, no
motion fault and a 10.3 ms latest sample interval. The B/Z/Y/C direction map
was resent and acknowledged, leaving A and F unmapped stop actions.
The first complete F test after flashing (21:19:05) reported zero decodes,
one rejection, and no timing error or overflow. Its middle symbol matches
reverse F, but the marker checks still fail. The previously faint closing bar
is stronger (~588 versus 42); a different opening bar is now faint (~70) and
is discarded by current trimming. Diagnostic shallow-pair cleanup restores
the opening marker but still cannot validate the closing marker. The approach
to the first bar is only about 34 mm of logged travel. Three later attempts
were interrupted by paper catching the sensor, as confirmed by the user;
they are not counted as complete barcode tests. Physical A regression tests
of this image are still pending.
The earlier 100 Hz/start-50 image
produced 21/22 observed A successes, including the last 20 consecutively;
those results do not validate the wider-gap changes on hardware.

## 3 November 2026: earlier recording with real pre-trigger history

The integrated motion profile is back to 10 ms samples (100 Hz) and a 10 ms
kernel tick. This also restores motion/pose telemetry's original 200 ms cadence.
Cruise remains 0.04 m/s and minimum peak prominence 15; the recording-start
rise is now 50 rather than 150 normalised units above the paper reference.

The decoder retains up to 1.5 seconds of real waiting readings in a bounded
320-entry ring. When a stronger bar triggers, those readings are copied in
chronological order so a faint opening star or gradual edge is preserved.
Their measured duration is subtracted from the prior quiet time: no synthetic
white is added and no interval is credited twice. Invalid samples, timing gaps
and reset discard the history. A failed scan cannot re-trigger on the same
50..75-contrast patch without first returning below the start threshold.

The decoder retains its 2048-entry frame buffer. Dashboard captures now retain
the same 1.5-second pre-trigger window, with the original 400 ms quiet ending;
their 2400-entry pool covers about 24 seconds at 100 Hz. The existing width,
quiet-zone, frame-duration and 30 ms safety checks are retained. Host coverage
includes faint opening stars, ring/timestamp wrap, insufficient quiet, invalid
input/history reset, noise, 100/200 Hz shapes and recorded waveform regression.
Fresh physical A runs are still required: the earlier failed captures omitted
their opening readings and cannot demonstrate recovery of those missing bars.

The sections below retain the earlier bench/integration history.

25 October team integration: `S3_MOTION=1` now also starts Buddy 1 and mirrors
telemetry through a taskless facade, while preserving this runtime's single
motion/report owner. Mac/Linux: `make -C testing/subsystem3 test codec drive
feedback regression sizes`; `make sanitize` adds memory/undefined-behaviour
checks. [Current review](../../docs/INTEGRATION_2026-10-25.md) supersedes older
two-buddy-only and representation-size descriptions below.

The core software is implemented and host-tested. The new firmware has **not**
been flashed or validated on the car. Use the isolated bench UF2 for stationary
checks; the normal firmware retains Buddy 2's existing motion startup.

The [27 October wiring/calibration correction](IR_CALIBRATION_2026-10-27.md)
records measured raw ranges, the excluded wrong-material capture, passing host
tests/builds, and the user's decision to defer flashing. Saved calibration and
sensor roles are corrected in source; the Pico still runs its older image.

## Buddy 2 integration

The separate `S3_MOTION=1` profile connects steering, distance feedback,
manoeuvres, fault braking and a motion-owner watchdog. It boots disarmed.
See [operation and control API](../../app_program/subsystem3/MOTION_INTEGRATION.md).
Build with `tools/build-s3-motion.ps1`; never confuse its `_s3_motion.uf2` output
with the stationary `_ir_bench.uf2`. No motor-capable image was flashed here.
The same host test command also runs the real Buddy 2/Buddy 3 integration tests
and production motion-feedback tests with mocked kernel/hardware boundaries.

## Repeatable host tests

From the repository root in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test-buddy3.ps1
```

Requires MSYS2 UCRT64 GCC at `C:\msys64` (override with `-MsysRoot`). Compiles
portable C with `-std=c11 -Wall -Wextra -Werror -pedantic`, runs the existing
reflectance suite and the new acquisition/calibration/line/barcode/navigation
suite. It does not access the Pico. Generated executables are ignored under
`build/host-tests`.

Coverage includes per-channel identity/enabling/errors, independent calibration,
both polarities and hysteresis; centred/left/right traces, loss/reacquisition,
junction persistence/rearming, steering direction/limits/faults; all A-Z frames
forward and reversed through timestamped samples and run fixtures, timestamp
wrap, width variation, malformed/truncated frames, quiet zones, duplicate
suppression, timing failures and overflow; mappings, unmapped letters, manoeuvre
suppression, simulated calibration -> start -> follow -> barcode -> rearm,
initial/subsequent line loss and explicit fault reset/restart. `test_navigation.c`
adds provisional/disabled/stale/missing-input start gates, atomic inactive-only
mapping/calibration updates, net forward/reverse rearming and oscillation,
malformed capture recovery, odometry epoch changes, persistent stop requests,
no suppressed replay and synthetic 5 ms/10 ms traces with jitter. The existing
acquisition, calibration, line, steering, A-Z and independent A/Z tests remain.
These are synthetic tests, not measured physical detection performance.

The Windows script also runs `test_barcode_record.c` and
`test_barcode_varying.c`, and builds a stdin replay utility. To run the saved
3 November waveform regressions afterwards:

```powershell
python .\testing\subsystem3\test_barcode_replay.py
```

On Mac/Linux, use `make -C testing/subsystem3 replay`. The
[fixture notes](fixtures/barcode/README.md) document adaptive peak detection,
sentinel-only width correction, original run outcomes, quantised timing and
the explicitly synthetic quiet margins used in replay. Replay success does
not replace a new hardware run.

`barcode_fixtures.h` contains independent bar widths from all four pages of the
supplied `Project/Barcode Sample.pdf` (A,A,Z,Z). Its source hash and extraction
method are recorded in the header. Tests add explicit quiet zones; they do not
claim the PDF's existing margins meet the decoder's requirement. The optional
fixture-check script below requires PyMuPDF and the original course PDF:

```powershell
python .\tools\check-barcode-fixtures.py 'A:\OneDrive - Singapore Institute Of Technology\SIT\Year 2 Tri 1\Embedded Systems Programming\Project\Barcode Sample.pdf'
```

## Build the stationary firmware

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build-ir-bench.ps1
```

Output: `build_make/mtk3pico_smp1_usb_cdc_ir_bench.uf2` (barcode GP28 only).
For all three physically connected sensors:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build-ir-bench.ps1 -Sensors 3
```

Output: `build_make/mtk3pico_smp1_usb_cdc_ir_bench_3sensor.uf2`. It enables
barcode GP28, left GP27 and right GP26 and reports each channel separately.
Its startup still cannot initialise motion. The build scripts automatically run
`tools/check-buddy3-profile.ps1` against the compiled object symbols. A channel
being enabled does not prove its wiring works.
The script builds only; it never flashes. It uses MSYS2, local Arm GNU Toolchain
14.3.Rel1 in `build/tooling/arm`, and Pico SDK 2.3.0 with TinyUSB in
`build/tooling/pico-sdk`. These dependencies are ignored by Git and must be
installed separately in another checkout. With those tools on PATH:

```sh
make -C build_make PICO_SDK_PATH=../build/tooling/pico-sdk IR_BENCH=1 WIFI=none TEST=0 -j8
```

Normal regression build (also requires the SDK's CYW43 submodule):

```sh
make -C build_make PICO_SDK_PATH=../build/tooling/pico-sdk IR_BENCH=0 TEST=0 -j8
```

Make tracks profile changes to invalidate stale objects. The bench bypasses all
other application startup and rejects `TEST=1`. Only the bench starts the two
Buddy 3 tasks. The kernel USB console still runs. No motion request is issued.

## Wiring and enablement

Disconnect USB and battery before changing wiring. For initial stationary tests,
leave the battery disconnected and use USB power. Wiring confirmed 2026-10-27:

| Physical sensor | Grove port | Signal connected to AO |
|---|---|---|
| Front-left | 6 | GP27 / ADC1 |
| Front-right | 5 | GP26 / ADC0 |
| Barcode | 7 | GP28 / ADC2 (yellow wire, confirmed by board label) |

For each sensor, red 3V3 connects to VCC and black connects to GND. Leave DO
and unused signal wires disconnected. Follow printed GPIO labels rather than
assuming cable colours. Grove 6 also exposes GP26: use GP27 there so it does
not share the right sensor's analogue input. Check GP28's VBAT-sensing jumper.

`app_program/subsystem3/ir_bench_config.h` defaults to mask=1: barcode=GP28 enabled,
left=GP27 and right=GP26 disabled/unavailable. After both front AO signals are
connected correctly, build with `-Sensors 3`. **Check GP28's
battery-sensing jumper before connecting that input.** Left and right are the
car's left/right when facing forward. Earlier logs use superseded firmware role
labels; identify those observations by GPIO when comparing them with this wiring.

Each sensor has its own measured reference macros: left white/black=144/1501,
right=182/3068, barcode=156/305. These use the 27 October mounted-sensor raw
collections; front sensors were tested on path material and barcode on printed
paper. See [ranges, exclusions and verification](IR_CALIBRATION_2026-10-27.md).
Do not substitute black path or desk readings for printed barcode black. Keep height and
lighting fixed while comparing surfaces. The classifier normalises to darkness
0..1000 and supports either polarity; WHITE at <=400, BLACK at >=600, retaining
state between boundaries. Startup is UNKNOWN until a boundary is reached.

Explicit calibration collection and validation APIs are described in the
[handoff](../../app_program/subsystem3/INTEGRATION.md). They report count,
minimum, maximum and average, reject missing/overlapping ranges and preserve
previous references on failure. There is no automatic threshold adaptation.

## Manual flashing and reading, when ready

1. Keep battery disconnected, ensure the Pico pins are correctly aligned and
   undamaged, and use a USB data cable. If mounted, set the Robo Pico switch ON.
2. Hold the Pico's BOOTSEL button while connecting USB to the computer; release
   it when `RPI-RP2` appears.
3. Copy **the isolated bench UF2** to that drive. This replaces the Pico firmware;
   the drive disappears when it restarts.
4. Read output with the command below; add `-Port COMx` if several ports exist.
   Close any other programme using that serial port. Ctrl+C exits the reader.

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\read-ir.ps1
```

Illustrative output, not a captured hardware result:

```text
[S3 BENCH] enabled=1 cal=MEASURED_2026-10-27 status=0 samples=20 gap=10000us max=10020us target=10ms
  run=INACTIVE readiness=0xf accepted_cal=0x0 odometry=0 rearming=0 stop_requested=0 faults=0x0
  readiness bits: 1=channels 2=calibration 4=sensors 8=odometry 16=fault; gaps>5ms=19 diagnostic_letters=0 actionable_steering=0
  barcode GP28 valid=1 raw=156/4095 dark=0/1000 WHITE white=156 black=305 adc_error=0
  left GP27 DISABLED/unavailable
  right GP26 DISABLED/unavailable
  line valid=0 present=0 error=0 junction_candidate=0 preview_valid=0 L=0 R=0 mm/s DIAGNOSTIC PREVIEW ONLY
```

Reports also include cumulative barcode/junction/lost/reacquired/fault event
counts, diagnostic-letter count, last decoded letter/direction/action, ADC errors, barcode rejects and
timing/overflow counts. Speed proposals are formatted in mm/s for the console;
the public API uses m/s. The bench stays inactive and has no odometry adapter,
calibration UI or automatic start: diagnostic letters/previews never become actions.
Measured numerical references do not set the bench's accepted-calibration mask.
No motor moves are requested. A missing front sensor
pair correctly means no valid steering proposal in the default mode.

Sampling targets 10 ms on core 0 at priority 16; reporting runs separately at
priority 17 every 200 ms. Actual timestamps/gaps are used. Console or kernel
activity can delay sampling. Counts of measured gaps over 5 ms expose the Week 6
scheduling mismatch; they do not fix it. The kernel tick/priorities are unchanged. Frames reset on inadequate timing; this is not a
claim of guaranteed barcode operating speed. ADC errors invalidate samples, but
a disconnected floating wire can still look valid: software cannot certify wire
health from ADC counts alone.

## Physical acceptance still outstanding

At final height and lighting, collect repeated white/black ranges for each sensor
using actual track materials. Confirm steering sign from mounted left/right
channels; tune the line thresholds and gains. Check printed barcode quiet zones,
then measure decoding under actual movement and sampling jitter. Confirm real
junction behaviour and action mapping with the team. Vehicle Controller/Buddy 2
integration comes after these checks. See [test evidence](TEST_LOG.md).

## Week 6 scope

The [handoff traceability table](../../app_program/subsystem3/INTEGRATION.md#week-6-traceability-and-outstanding-team-decisions)
distinguishes preserved algorithms, changed sensor roles, behaviour implemented
within Buddy 3 and work awaiting integration. Outstanding decisions are sensor
roles, 5 ms scheduling, distance tuning, mapping delivery, fault-to-brake handling
and full-system priority/timing validation. None is certified by host tests.
