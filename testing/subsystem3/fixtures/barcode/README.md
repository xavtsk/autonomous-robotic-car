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
This fixture is preserved for diagnosis, not added to the current replay
success expectations. Its observed hardware rejection and offline analyses
must not be described as a successful A decode.
