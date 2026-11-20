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
Doxygen output was not generated as part of this historical batch. The
[generated reference workflow](GENERATED_REFERENCE.md) now records generation
and compiler-definition comparison separately. This is not closure of BARR
2.2.h across the project.

```sh
python tools/compliance/verify_comment_batch.py --manifest docs/compliance/FUNCTION_DOCS_BATCH1.json --candidate 4f5dd616198455d9784e7a60f3bd1edc873fa89e
```

This command checks the bounded historical batch despite later source changes.

The [controller debug contract batch](CONTROL_DEBUG_CONTRACTS.json) separately
documents 13 definitions and six public prototypes at `07ad498`. Its contracts
distinguish pack rejection with unchanged output, unpack rejection after fields
were written, latched field errors and a non-latching final relationship check.
Inputs retain their full validation widths. Clang reports no documentation
diagnostics and attaches every parameter contract. Preprocessing fingerprints
and the order of all existing comments match the prior sources. This is a
comments-only source proof; no new runtime behavior or full-project coverage is
claimed. Both batches are rechecked at their immutable revisions in Linux CI.

```sh
python tools/compliance/verify_comment_batch.py --manifest docs/compliance/CONTROL_DEBUG_CONTRACTS.json --candidate 07ad498a49428b7362c0e5bcc010f66f1bb650d8
```

Two later source-preserving batches add [motion/PID contracts](MOTION_FUNCTION_CONTRACTS.md)
for 31 definitions and 17 prototypes, and [IR contracts](IR_FUNCTION_CONTRACTS.json)
for ten definitions and nine prototypes. The IR contracts distinguish overall
scan success from individual ADC errors, full output clearing on configuration
rejection, calibration rejection with unchanged active settings, and retained
surface hysteresis. Each batch has exact preprocessing fingerprints, preserved
old comments and zero Clang documentation diagnostics in its recorded contexts.
Their immutable commits are checked separately by the audit workflow.

The [navigation and telemetry codec contracts](CODEC_FUNCTION_CONTRACTS.md)
add 28 definitions and 14 prototypes at `0ba528b`. They distinguish telemetry
encoding that may leave partial output on rejection from decoders that commit
only validated outputs. The current version-3 snapshot's pose fields and exact
rounding/encoding ranges are documented without changing packet bytes.

The [tilt, IMU calibration and obstacle profile contracts](CALCULATION_FUNCTION_CONTRACTS.md)
add nine definitions and nine prototypes at `4ce9b14`. Their 34 recorded compiler
contexts include both IMU debug layouts. Contracts retain signed-zero and angle
wrapping behavior, calibration readiness latching, and the exact order in which
sample/profile outputs can change before rejection. These are comments-only
proofs, with generated-reference evidence recorded separately.

The [sensor and terrain contracts](SENSOR_TERRAIN_FUNCTION_CONTRACTS.md) add ten
definitions and seven prototypes at `4dd87b3`. They document partial device
writes and output mutation, optional magnetic validity, crossing completion and
rearming, preserved historical peaks and classification precedence. Twenty-four
recorded compiler contexts have unchanged interfaces and no documentation
warnings. None of these bounded batches establishes whole-project completion.

## Earlier observed workload snapshot

`tools/compliance/barr_functions.py` reuses the compiler contexts and six profile
variants from the field inventory. This earlier collection covered 247 scoped
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

These historical observed counts are not the current source denominator.
Later ownership corrections, contracts and new fixtures changed the scope;
the current-revision CI function inventory supersedes this snapshot when
available. Recovery ASTs in this snapshot account
for 270 variants; unobserved conditional combinations remain open. A recognized
comment is not proof of a correct contract. Module `@file` comments are excluded
from the function-comment count. Mixed imported bodies are excluded through the
shared region selectors. The local report is
`build/compliance-review-20261120/barr-functions.json`; regenerate it after source
changes using current target and host compile databases:

```sh
python tools/compliance/barr_functions.py --compile-db <target.json> --compile-db <host.json> --output <function-report.json>
```

## Retained operator contracts

The interrupted operator documentation batch was published in `903057d`.
Its [source-bound receipt](OPERATOR_FUNCTION_CONTRACTS.json) covers 23 definitions
and eight prototypes in `op_logic.c`/`.h`, including STOP precedence, startup
button arming, callback lifetime, rejected requests and timeout behaviour.
The existing comment-batch verifier confirms unchanged C tokens, preprocessing
directives and original comment order against `a3de92a`. The recorded six parser
contexts have no documentation diagnostics. The later four-job Linux run for
`903057d` also has identical operator traces; physical validation is separate.

```sh
python tools/compliance/verify_comment_batch.py --manifest docs/compliance/OPERATOR_FUNCTION_CONTRACTS.json --candidate 903057d35ecc1c8551f58c357ae1517c58bd16f5
```

## Next review batches

The Phase 2 [mechanical source receipt](evidence/phase2-mechanical.json) also
records contracts for three inline helpers and six remote-control prototypes.
The review covers full-range little-endian values, the half-range sequence
comparison, invalid input, STOP precedence, lock failures and disabled stubs.
The corresponding target consumers compile without documentation diagnostics;
paired Arm object/relocation/stack comparisons are unchanged. This is bounded
interface documentation, not completion of the controller's function contracts.

1. Reconcile the completed codec/calculation/sensor batches above with the
   current compiler inventory. Keep any remaining pure helpers and public
   prototypes visible rather than repeating already completed batches or
   assuming a whole directory is documented.
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
