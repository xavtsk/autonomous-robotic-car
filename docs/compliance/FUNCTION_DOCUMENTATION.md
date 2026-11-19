# Function documentation coverage

The first contract batch covers seven files: the estimator (now
[`motion_estimation.c`](../../app_program/subsystem2/motion_estimation.c) and
[`motion_estimation.h`](../../app_program/subsystem2/motion_estimation.h)),
`integration_codec.c/.h`, `line_controller.c/.h` and `run_timing.h`. It documents
31 definitions and 17 public prototypes from their actual implementations.
The [batch manifest](FUNCTION_DOCS_BATCH1.json) binds every file to its preceding
Git version. Its original `estimate_motion.c/.h` paths are historical evidence
and remain pinned to the documentation commit. All preprocessing tokens and
existing comments were preserved by that comment-only batch.

Contracts include estimator units and noncoherent live reads; codec validation,
unchanged outputs on rejection and copied floating representations; steering
acceptance/clamping; and strict timing thresholds with saturating counters.
No namespace, signature, visibility, numeric operation or synchronization changed.
Clang 18.1.1 recognizes all 48 comments with no documentation diagnostics.
Doxygen output has not yet been generated or reviewed, so this is not closure of
BARR 2.2.h across the project.

```sh
python tools/compliance/verify_format_batch.py --manifest docs/compliance/FUNCTION_DOCS_BATCH1.json --candidate 4f5dd616198455d9784e7a60f3bd1edc873fa89e
```

This command checks the bounded historical batch despite later source changes.

## Observed remaining workload

`tools/compliance/barr_functions.py` reuses the compiler contexts and six profile
variants from the field inventory. The current collection covers 247 scoped
paths after upstream-boundary correction and addition of two fixtures. Its 1,238
translation-unit observations contain 45 units with diagnostics. It records:

| Observation | Count |
| --- | ---: |
| Definition variants, including macro-renamed test variants | 1,757 |
| Distinct source definition sites | 1,749 |
| Variants observed in an error-free translation unit | 1,487 |
| Definition variants with recognized function comments | 98 |
| Error-free variants still without recognized function comments | 1,407 |
| Files containing observed definitions | 148 |

These are observed counts, not a certified denominator. Recovery ASTs account
for 270 variants; unobserved conditional combinations remain open. A recognized
comment is not proof of a correct contract. Module `@file` comments are excluded
from the function-comment count. Mixed imported bodies are excluded through the
shared region selectors. The local report is
`build/compliance-review-20261120/barr-functions.json`; regenerate it after source
changes using current target and host compile databases:

```sh
python tools/compliance/barr_functions.py --compile-db <target.json> --compile-db <host.json> --output <function-report.json>
```

## Next review batches

1. Finish pure codecs and calculation modules with their headers: navigation,
   telemetry, motion packing, PID, reflectance/calibration, tilt and obstacle
   profile calculations. Document exact rejection, sentinel, units and mutation
   rules before touching their statements or representations.
2. Review owner/state-machine modules in smaller groups: motion and controller,
   sampling/runtime, barcode work scheduling, terrain owner and obstacle owner.
   Their task/ISR ownership, deferred requests, reset transitions and bounded
   work need actual call-site review. A generic signature summary is insufficient.
3. Document test harnesses and adapter callbacks from their actual fake/ABI
   behavior. Keep negative-test inputs and assertion predicates intact. External
   SDK, kernel and MQTT callback names are exact pending boundary cases, not
   candidates for blanket private/static conversion.

The largest remaining application groups observed are `motion.c` (61
undocumented variants), `ctrl_motion.c` (43), `s3_integration.c` (41),
`bc_decoder.c` and `buddy4.c` (30 each), `buddy5.c` (29), and `s3_drive.c` (28).
Line movements and conditional definitions can change those counts.

The [separate module rename](MOTION_MODULE_RENAME.md) aligns the estimator's
filenames with its ten existing `motion_estimation_*` functions (BARR 6.1.i).
It preserves the public API names and uses revision-specific source lookup in
the shared runners. No API symbol rename is included in the documentation batch.

Prototype review candidates include `usermain`, the two MQTT debug-take callbacks
in `s3_integration.c`, and `pass_extra_test` under `S5_HOST_TEST`. A canonical
declaration first seen in a C file is only a candidate, not proof of a violation
in every profile. Check module headers and external contracts before adding a
declaration or changing visibility. Public-before-private body order (4.3.b)
remains a separate review: moving definitions may affect optimizer decisions,
link layout and stack evidence even when C operations remain the same.
