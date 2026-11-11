# 3 November 2026 barcode replay fixtures

These are unmodified BCAP exports, not synthetic sensor readings. Filenames
identify the local capture time. Source exports:

| Fixture | Original export | Recorded outcome |
| --- | --- | --- |
| `20261103-152854-successful-z.txt` | Barcodetxt / `barcode_trace_20261103_152854_0fd144b7.txt` | Z, reversed |
| `20261103-171918-normal-speed.txt` | dashboard / `trace-f3e43a3290184c3e9b5ce1945cae27e4.txt` | rejected at 0.04 m/s |
| `20261103-173819-half-speed-1.txt` | dashboard / `trace-297177659b4940138d36d64ab23887c8.txt` | rejected at 0.02 m/s |
| `20261103-173958-half-speed-2.txt` | dashboard / `trace-f9b154b7cbf742918749b74ae9d4debf.txt` | rejected at 0.02 m/s |
| `20261103-181733-adaptive-a.txt` | dashboard / `trace-7da9d18abc33474c8656e898a5c03c20.txt` | A test rejected with adaptive reader, minimum prominence 32 |
| `20261103-182659-varying-a.txt` | dashboard / `trace-9ea50ecc66ed45a281c7fd1d50a5b902.txt` | A test rejected with minimum prominence 24 and shared width correction |
| `20261103-191949-speed003-a.txt` | dashboard / `trace-457142746b524d959113e3c59153670e.txt` | A rejected at 0.03 m/s, minimum prominence 24 |
| `20261103-192042-speed003-a.txt` | dashboard / `trace-c08b3193efd24f31a405002f87064418.txt` | A rejected at 0.03 m/s, minimum prominence 24 |
| `20261103-192137-speed003-a.txt` | dashboard / `trace-9a2c535eb4004f2b8667390e104fd56c.txt` | A rejected at 0.03 m/s, minimum prominence 24 |

The capture contains only about 20 pre-trigger readings, not the entire
leading quiet zone. Each `dt` was floored to milliseconds, and darkness is
quantised in units of four. The failed runs stop recording at the old reader's
rejection, so they may omit white needed by the new reader.

`test_barcode_replay.py` reconstructs intervals as `dt_ms * 1000 + 500` us,
and separately checks the 0/999 us extremes. It labels every use of synthetic
leading/trailing white. With 2 seconds added before and 1 second after, the
normal-speed waveform, second half-speed waveform, 18:17/18:26 A waveforms,
and all three 19:19..19:21 waveforms at 0.03 m/s
decode as A; the first half-speed waveform remains rejected. The old Z still
decodes as Z with added leading white. None of the
nine clips alone proves a complete decode when the missing leading quiet is
not supplied. A padded replay is a conditional algorithm check, not evidence
that the car decoded the recorded run.

The new reader learns peak prominence from quiet-paper sample differences
(four times their running mean, bounded to 15..100 darkness units), freezing it
for each recording. The current start trigger is 50 units, with up to 1.5 seconds
of real decoder pre-trigger history to retain faint opening bars before a
stronger bar triggers. Its stricter quiet band uses the learned prominence,
and the first strong bar cannot raise the paper reference. These older exports
contain only their original 20-sample history; the new decoder cannot recover
readings that were never saved. Synthetic faint-opening tests and new hardware
runs are needed to validate that change.

The 18:17 A run's final bar peaked at 28 darkness units. The original adaptive
floor of 32 missed it and yielded only 27 widths. Lowering the floor to 24
allows all 29 widths to be measured. The new fixture fails against the
32-unit reader even with added white margins. The 24-unit reader must decode
A with those margins while still refusing the clip without leading quiet.

The three 19:19..19:21 runs reached about 0.03 m/s but all rejected A at the
24-unit floor. Faint marks peaked around 20 normalised darkness units, below
that floor despite several readings across the marks. The user-selected
15-unit floor recovers A in all three midpoint replays with explicit white
margins. These fixtures also check the interval-rounding extremes and refuse
the clips when the missing leading quiet is not supplied. These captures were
recorded at 0.03 m/s; the user subsequently requested 0.04 m/s for the new
15-unit firmware, keeping 100 Hz sampling. These 0.03 m/s replays do not prove
decoding at the restored speed; a new physical run is still needed.

If uncorrected widths fail, both known star patterns first determine a shared
black/white width correction. The fit minimises the worst relative sentinel
width error on a bounded 129-point grid in each direction. If no shared
sentinel fit exists, a bounded fallback fits each star independently. It
requires the same star orientation, no reversal of bias sign, and a change
no larger than half the smaller sentinel narrow mean. Each endpoint is bounded
to three quarters of its local narrow mean. A shared fit whose payload widths
or pattern fail is a terminal rejection, not a reason to try another model.

The fallback holds the correction constant within each star and interpolates
by measured time between them. Alternating rising/falling edges move by half
the local bias; adjacent widths share the same shifted edge. The first and
last biases also adjust their respective quiet zones. The payload never
chooses the endpoint fits. The existing 29-run, ratio, tolerance, gap, sampling,
duration, letter and quiet-zone rules must still pass. The candidate is cached
once and released only after actual trailing white is observed; a new visible
mark cancels it. Speed, sampling rate and dashboard action mapping are unchanged.

The 18:26 run had all 29 measured widths but no shared sentinel fit. Its first
star needed roughly 103..160 ms of correction and its last roughly 54..80 ms
in midpoint replay. The bounded fallback fits approximately 132/67 ms and
recovers A with supplied margins. This models changing edge distortion; it
does not establish its physical cause or prove a successful car run.

Validation on 3 November: all synthetic recorder tests passed, including all
letters/directions, positive/negative width bias, invalid payload, inconsistent
stars, faint peaks, noise, interrupted quiet and 3,000 jittered frames without
a wrong letter. `test_barcode_varying.c` also checks all letters/signs/scan and
slope directions, payload-independent fits, invalid payloads, inconsistent
stars, bounded drift, unchanged failure inputs and corrected quiet margins.
The nine fixtures pass 108 replay checks at minimum prominence 15. An earlier comparison
of 189 complete, nonempty historical episodes using the same padded midpoint
replay at minimum prominence 24 produced identical decoded-letter results with shared-only and varying correction (three Z and
one Y). These replay counts are not the original run-success counts.

Hardware detection and runtime latency still require a new car test with the
updated Wi-Fi firmware. The firmware build itself is not hardware proof.

## Larger papers, preserved diagnostic captures

`20261103-203855-wide-z-rejected.txt` is the complete export
`trace-88c811341ada4a42be89046a6f087630.txt` (763 samples). Offline reconstruction
indicates Z; the measured payload has one narrow white gap approximately
149.4% of its narrow mean after sentinel correction. The final recorded sample
is invalid and exceeds the 30 ms interval bound. Offline replay that excludes
that sample and supplies trailing white is conditional, not a hardware decode.

`20261103-210017-wide-f-rejected.txt` is the complete export
`trace-8ba88aed9b904c8ebfee20f06c6d1d15.txt` (1098 samples in two episodes).
The user confirmed F. The first episode suggests reverse *F* but the actual
firmware reported zero decodes and one rejection. Weak-bar/plateau segmentation
and the distorted closing marker remain unresolved. Its roughly one-second
leading white may also be too short; the first episode's trailing cutoff must
not be interpreted as the physical end of the white margin. These files are
retained for diagnosis and are not added to the older single-episode padded
success expectations in `test_barcode_replay.py`.

`20261103-211905-wide-f-repeat-rejected.txt` is the complete post-flash export
`trace-b47324a0626b4471a6a0c907642481e8.txt` (1143 samples, two episodes).
The same-orientation F repeat reported zero decodes and one rejection, with
no timing error or overflow. Its middle nine widths match reverse F; neither
marker passes current extraction/validation. The old faint closing bar is now
strong, while an opening bar at roughly 70 darkness units is discarded in
favour of a wiggle in a wide bar. Diagnostic shallow-pair cleanup restores the
opening marker but leaves the closing marker invalid. These are approximate
offline measurements from raw-normalised samples and midpoint reconstructed
intervals, not a successful firmware decode. The three later captures at
21:19:52, 21:20:46 and 21:21:37 were interrupted, confirmed by the user.

`20261103-212737-wide-f-shifted-rejected.txt` is the complete export
`trace-559145a050e548a2b4b38c750f8e0dfd.txt` (963 samples, one episode), after
shifting the sensor path on the same F paper. Hardware still reported zero
decodes and one rejection, without timing error or overflow. All 15 real bars
survive extraction. A sag and rebound inside a wide marker bar expose the
old nearest-midpoint-crossing error, which shortens that bar and inflates its
following gap. First-rising/last-falling crossings recover strict *F* widths
without relaxing marker checks. `test_barcode_replay.py` includes this fixture
and now runs 120 checks: no-padding and missing-leading cases still refuse;
the midpoint replay with explicit 2 s leading and 1 s trailing white decodes
F. Those margins are synthetic. The saved capture starts 169 ms into the run
and ends on rejection; it does not establish the actual total white margins
or a successful hardware decode with the new reader.

## 5 November: small A contrast failure

`20261105-153638-small-a-faint-rejected.txt` is the complete 537-sample export
`trace-fe29a712d437462eb736a9ab4f4289b2.txt`. The user's four runs were Z/A/Z/A:
both Z summaries report successful decodes and both A summaries report a
rejection, without decoder timing errors or overflows. Only this last upload
is complete. The first A also has a later motor-stall report, but its partial
capture cannot establish that fault as the cause of the rejection.

Current-source raw-normalised and quantised replays extract only 23 widths
(12 bars), rather than the required 29 widths (15 bars). Refusal occurs
before marker, payload-width and quiet-zone validation. Three faint bumps at
approximately 1.059, 1.338 and 2.050 seconds within this clip peak at raw ADC
256, 251 and 260: normalised 12, 10 and 13 using white/black 215/3550. They
are recorded, but fall below the frozen peak-prominence floor of 15. Six
other narrow peaks are only 18..23. These measurements identify a contrast /
peak-detection problem; the waveform alone does not establish its optical
cause. The older nearest-crossing rule produces the same 23 widths, so this
failure is not introduced by the outer-edge selection change.

Diagnostic-only minimum prominence 12 or 10 does not produce a valid A:
the adaptive threshold and extra/missed extrema still prevent validation.
Those initial diagnostics did not establish a fix. Its recorded hardware result
remains a rejection; later recovery checks below are offline software evidence.

The user also confirmed these three complete captures as the same small A:

| Fixture time | Samples | Original hardware result |
|---|---:|---|
| `20261105-160054-small-a-faint-rejected.txt` | 568 | rejected |
| `20261105-160403-small-a-faint-rejected.txt` | 522 | rejected |
| `20261105-160438-small-a-faint-rejected.txt` | 522 | rejected |

The bounded faint-region recovery retains 15 bars and measures weak peaks around
their connected half-height core. All four raw-normalized captures recover A at
midpoint reconstructed timing with explicit trailing white padding. Normalization
uses the recorded firmware calibration, white=215 and black=3550. Their raw ADC
counts retain information that the exported darkness/4 stream discards: the last
quantized capture can still reject. One captured stream (16:00:54) decodes without
padding; the other three end before the new reader finalizes trailing quiet.

The replay suite checks raw and quantized inputs, three interval reconstructions,
both waveform directions, and four explicit padding choices. The observed raw
scan direction must recover at midpoint timing with padding; uncertain variants
may reject but cannot report another letter. These checks neither invent physical
quiet-zone evidence nor establish a successful run of the new firmware.

## 5 November evening: repeat A failures and successful Z control

The user confirmed the following four scans as the small A and the last as
the larger Z. All five uploads contain one complete episode, with no reported
timing error, overflow or unstored episode. The names retain the observed
hardware result; a later successful replay does not change that result.

| Fixture | Samples | Hardware result | Dashboard trace ID |
|---|---:|---|---|
| `20261105-182025-small-a-decoded.txt` | 533 | A decoded | `385d66f5e07e475e996e0446cbbedf17` |
| `20261105-182223-small-a-rejected.txt` | 531 | rejected | `594e98326151409bb6ec535db4d7372f` |
| `20261105-182416-small-a-rejected.txt` | 519 | rejected | `09aed14d2fdf475db0ebd103ed19fade` |
| `20261105-182831-small-a-rejected.txt` | 507 | rejected | `eb6313e5a49147ae883ff67d857a710b` |
| `20261105-183024-wide-z-decoded.txt` | 990 | Z decoded | `8f2f9e42b9bd4b39bff5c4c928d3fc7c` |

These fixtures exercise two further failure paths: rising or faint barcode
features contaminate an estimate intended for quiet-paper noise, and a primary
extraction can return 29 widths while retaining a wide-bar wiggle in place of
a real narrow bar. A count of 29 is therefore not itself a valid barcode.

The later correction recovers the 18:24 and 18:28 raw-normalized A captures in
offline replay. The 18:22 raw capture still refuses: one narrow payload white gap
remains about 2 ms below the strict minimum. Its quantized representation can
pass, which illustrates that lossy previews are not interchangeable with raw ADC
measurements. The successful A and Z controls must remain successful.

At midpoint reconstructed timing in the observed direction, replay requires
those recoveries and the retained 18:22 raw rejection with explicit 2-second
leading and 1-second trailing synthetic white margins. Quantized inputs, both
scan directions, three timing reconstructions and all four margin choices must
never produce a different letter. Lossy or timing-uncertain variants may refuse.
These are software regression requirements, not claims that the rejected
hardware runs decoded or that the added quiet intervals were physically observed.

## 10 November: replacement sensor and dense wide-bar ripple

These five files are byte-for-byte copies of complete dashboard exports after
the faster BCAP upload firmware was installed. The user tested small A and
wide W. Names and summaries preserve the observed hardware result, including
the rejected W; subsequent replay does not change that historical result.
All five summaries have zero timing errors, overflow and unstored episodes.

| Fixture | Samples | Hardware result | Dashboard trace ID |
|---|---:|---|---|
| `20261110-165431-small-a-decoded.txt` | 569 | A decoded | `bd4b228fc872427c9b382f257ea93560` |
| `20261110-165500-small-a-decoded.txt` | 592 | A decoded | `55f0cabab3fc4610a53cbabfcd04c8d3` |
| `20261110-165546-wide-w-rejected.txt` | 935 | rejected | `53e4457179014defa99607d08cf32fd4` |
| `20261110-165615-wide-w-decoded.txt` | 972 | W decoded | `d05107c66f92450c965401ea9d78c5d1` |
| `20261110-165652-wide-w-decoded.txt` | 976 | W decoded | `d4156f0daefc45c281b1423a78c6228c` |

Reconstruction uses the recorded white/black calibration, 215/3550 ADC counts.
The raw readings retain finer detail than the separate darkness/4 preview.
Neither representation retains the discarded fraction of each millisecond
interval; 0, 500 and 999 microseconds are alternative reconstructions, not
measured timing. Reversing a saved waveform is likewise a software variant,
not another observed physical scan. The 972-sample W includes one 20 ms
interval; the remaining intervals in these five files are at most 12 ms.

The failed W contains more local extrema than the former 63-entry workspace
could retain before its existing cleanup. A dense-ripple synthetic regression
exercises 79 extrema across an otherwise unchanged Code 39 frame, both scan
directions and full/quantized levels. It also checks malformed markers, a
missing bar, insufficient/interrupted quiet, and a 139-extremum input that
must still refuse within bounded storage. These tests retain strict Code 39
validation and are not a measure of physical reliability.

At midpoint reconstructed timing using raw ADC, the two A captures and the
reverse W decode without adding samples. The forward W hardware success has
clipped leading history in its saved export: replay requires 100 ms of explicit
synthetic leading white, even with the earlier firmware. The rejected forward
W needs the larger workspace plus that same leading margin to form a valid
candidate. With no added trailing samples it must remain in the quiet-wait
state and emit no letter; adding one second of synthetic trailing white permits
W. The older 63-entry workspace still rejects that same padded failed capture.

The replay suite runs all five recordings at raw and darkness/4 precision,
0/500/999 microsecond interval reconstructions, both waveform directions and
four explicit margin choices. Every interpretation may refuse except the
documented midpoint requirements; none may emit a different letter/orientation.
In particular, the failed run is not reported as an unpadded software success
or a successful hardware scan. New physical runs are required to validate it.

## 11 November: raised sensor, small and large Z

The sensing face was approximately 5–6 mm above the paper. The batch contained
21 runs: two small-Z successes among 11 small scans, and one large-Z success
among ten large scans. Six large scans ended in an acquisition-age fault and
two in a stale motor-command fault. Their live controller records, rather than
the separate barcode timing-error counter, establish those faults.

These four fixtures are exact, complete dashboard exports:

| Fixture | Samples | Hardware result | Dashboard trace ID |
|---|---:|---|---|
| `20261111-163031-small-z-rejected.txt` | 538 | rejected | `1460b4a27dc34ed2aeb159bd8bc28c5d` |
| `20261111-163440-small-z-decoded.txt` | 578 | reverse Z decoded | `10fe48fa40da4fcfb7c9aab2f708bf5d` |
| `20261111-163608-large-z-motion-fault.txt` | 950 | stale motor command; no barcode result | `28e29c93027b4bc9964dbab38a75e486` |
| `20261111-164410-large-z-motion-fault.txt` | 947 | stale motor command; no barcode result | `dc3ba114a57a4b199766372809e0cf37` |

The failed small scan lacks distinct ink regions even with the existing faint
recovery. It must continue to refuse at midpoint raw reconstruction, including
with added margins. The successful small scan decodes unchanged. Both large
exports contain valid Z geometry after the existing sentinel correction, but
their saved quiet context is insufficient for standalone replay. At midpoint
raw timing, one second of synthetic leading white reaches the quiet-wait state;
three additional seconds of synthetic trailing white permit Z (forward at
16:36:08, reverse at 16:44:10). Neither result proves that the physical scan had
those margins or would have succeeded without its controller fault.

The 96 new replay variants use raw ADC calibration 215/3550, three timing
reconstructions, both waveform directions and four explicit margin choices.
The arithmetic optimization must preserve accepted letters and rejection
boundaries; it does not loosen contrast, width, quiet or watchdog rules.

## 11 November after 17:00: large Z and interrupted trailing quiet

These five fixtures are byte-for-byte copies of complete dashboard exports
after the arithmetic timing change. The four `no-result` names preserve the
observed hardware outcome: neither a decode nor a rejection was reported
before the user stopped the car. They must not be relabelled as successful
physical scans if a later offline replay recovers Z.

| Fixture | Samples | Hardware result | Dashboard trace ID |
|---|---:|---|---|
| `20261111-170538-large-z-decoded.txt` | 998 | forward Z decoded; unmapped stop | `7b3d6327398544808aeb00fc76c46cd8` |
| `20261111-170621-large-z-no-result.txt` | 1193 | no result; recorder still active until stop | `3bce988b503c4746b8f414fba02cd526` |
| `20261111-170913-large-z-no-result.txt` | 1118 | no result; recorder still active until stop | `5ff1ff7b75f443b7a3294ab6c916dfd0` |
| `20261111-170958-large-z-no-result.txt` | 1139 | no result; recorder still active until stop | `c0c1a1c165b1438eadc92203a45ab58e` |
| `20261111-171039-large-z-no-result.txt` | 1165 | no result; recorder still active until stop | `4d0a90a202d7431394a59b9de8206e3b` |

All five summaries report zero timing errors, overflows and unstored capture
opportunities.
The four no-result waveforms contain reverse Z geometry followed by approximately
2.7--3.5 seconds of observed paper. Small paper fluctuations repeatedly reset
the former trailing-quiet timer. This differs from the older fixtures whose
physical trailing margin was clipped by a motor/control fault.

The 120 added replay variants retain raw ADC normalization with white/black
215/3550, all three interval reconstructions, both waveform directions and
the explicit margin choices `(0, 0)`, `(1000, 0)`, `(0, 3000)` and
`(1000, 3000)` in milliseconds. At midpoint timing, the four no-result
captures must recover one reverse Z with **only** one second of synthetic
leading history; their unmodified recorded trailing paper must suffice.
Without the synthetic lead they may still refuse because the saved pre-trigger
history clips the leading margin. No variant may invent another letter or
orientation, duplicate a decode, report a timing error, or overflow.

The forward success remains a positive control with explicit leading/trailing
padding; its shorter stored tail need not include the extra samples required
to finish deferred decoder work. Padding is always reported as synthetic, and
the original 888 replay checks retain their leading/trailing margin requirements.
The full suite contains 1008 variants. Fresh physical tests remain necessary
to establish whether the firmware changes resolve these failures on the car.

### Explicit latency accounting for the bounded worker

The former synchronous decoder could publish on the final sample retained by
the capture. After moving that work across subsequent updates, EOF can arrive
while the worker is still processing. The suite does not drain work at a frozen
timestamp or append unreported samples. Only these ten midpoint, observed-direction
variants with zero synthetic trailing time may instead end in clean processing:

| Fixture | Representation | Synthetic leading ms | Existing positive counterpart |
|---|---|---|---|
| `20261103-152854-successful-z.txt` | darkness/4 preview | 2000 | same lead plus 1000 ms synthetic trailing white |
| `20261110-165431-small-a-decoded.txt` | raw ADC | 0 or 100 | same lead plus 1000 ms synthetic trailing white |
| `20261110-165500-small-a-decoded.txt` | raw ADC | 0 or 100 | same lead plus 1000 ms synthetic trailing white |
| `20261110-165615-wide-w-decoded.txt` | raw ADC | 100 | same lead plus 1000 ms synthetic trailing white |
| `20261110-165652-wide-w-decoded.txt` | raw ADC | 0 or 100 | same lead plus 1000 ms synthetic trailing white |
| `20261111-163440-small-z-decoded.txt` | raw ADC | 0 or 1000 | same lead plus 3000 ms synthetic trailing white |

Clean processing means `state=3` and zero decoded, rejected, timing-error and
overflow events. Each explicitly padded positive counterpart still must emit
exactly the expected letter and orientation. These EOF outcomes are neither
software decode successes nor evidence that a new physical run succeeded.

Three existing insufficient-trailing-margin checks likewise permit processing
(`state=3`) as well as quiet-wait (`state=2`), while still requiring zero events:
the rejected 16:55:46 W with 100 ms synthetic leading white, and the 16:36:08 /
16:44:10 large-Z fault captures with 1000 ms synthetic leading white. None may
emit a letter without its previously required explicit trailing margin. Missing
bars, missing leading quiet, wrong letters/orientations and all other prior
rejection requirements remain unchanged.
