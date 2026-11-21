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

The local Phase 2 MQTT interface follow-up documents 17 definitions and eleven
public prototypes in `mqtt_bridge.c` and `mqtt_utk.c/.h`. Contracts describe the
actual queue selection, failure/output effects, ownership and weak defaults.
Six target parser contexts have no documentation diagnostics. Unlike the
earlier comments-only batches, it also renames three private helpers and uses
an equivalent fixed-width type; the
[grouped source receipt](evidence/phase2-mechanical.json) records token changes,
canonical-type compatibility and six matching Arm object/stack comparisons.
No generated-reference output, Linux runtime check or whole-project coverage is
claimed for this local follow-up.

The subsequent Wi-Fi owner/policy batch covers all eleven definitions in
`cyw43_utk.c` and `wifi_retry_policy.h` across eight core/network/scan contexts.
Contracts explain request acceptance versus connection, lock ownership,
scan completion, backoff, clock wrap and status-copy coherence limits.
The same [grouped receipt](evidence/phase2-mechanical.json) records unchanged
target object contents and stack usage. There are no documentation diagnostics
in the edited files; two imported SDK warnings remain recorded. This is bounded
compiler coverage, not a new generated-reference or runtime validation result.

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

## Local obstacle acquisition and recovery contracts

The [grouped Phase 2 receipt](evidence/phase2-mechanical.json) includes
`obstacle_contract_followup`: eight files covering `scan_obstacle`,
`ultrasonic_sensor`, `scan_servo` and `line_recovery`. All 22 definition variants
and 15 public prototypes have reviewed contracts, including APP_S5=0 stubs.
They describe actual caller ownership, range/rounding, timeout precedence,
failure output writes, partial hardware setup and latched line confirmation.
No new validation or behavior change is implied by these descriptions.

The comment-only verifier preserves all C tokens/directives and original
comments. Six Arm object/relocation/stack comparisons match, six documentation
contexts have no diagnostics, and nine existing host-test translation units
compile. Runtime and physical validation are not claimed. These bounded results
do not refresh the historical whole-project inventory above or close rule 2.2.h.

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

The later working-tree review added contracts to all 43 definitions in
[`ctrl_motion.c`](../../app_program/subsystem2/ctrl_motion.c). These cover
the nonzero command ID, request rejection and stop priority, fault ordering,
encoder units, brake ownership and completion acknowledgement. The motion
controller produced matching allocated Arm object sections, relocations and
compiler stack estimates before and after these comment edits. The figures
above describe the earlier compiler inventory.

The same review documented all 28 definitions in
[`s3_drive.c`](../../app_program/controller/s3_drive.c), covering barcode
actions at junctions, braking acknowledgement, motion-report ownership,
obstacle arbitration and fault/reset precedence. Clang 18.1.1 attached
function comments to all 28 definitions with avoidance enabled, all 23
active with avoidance disabled, and all 43 motion-controller definitions.
Both navigation configurations retained their Arm sections, relocations
and compiler stack estimates. [Source review and compiler evidence](evidence/phase2-mechanical.json)

The [separate module rename](MOTION_MODULE_RENAME.md) aligns the estimator's
filenames with its ten existing `motion_estimation_*` functions (BARR 6.1.i).
It preserves the public API names and uses revision-specific source lookup in
the shared runners. No API symbol rename is included in the documentation batch.

The kernel declares `usermain` in `kernel/knlinc/kernel.h`. Its application
implementation and host startup fixture now share the matching declaration in
[app_main.h](../../app_program/app_main.h). The IR and control debug callbacks
see the existing [MQTT declarations](../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h)
before either definition, including when barcode capture is disabled. The
obstacle header declares `pass_extra_test` only for `S5_HOST_TEST`. The host
fixture retains a matching declaration for runs against the older headers.
The [grouped evidence](evidence/phase2-mechanical.json) records eight matched
Arm compilation contexts for this prototype review. Source ordering and
whole-project contract coverage remain open.

The owner-task review covers every definition of the terrain and ultrasonic
tasks and their disabled implementations, the motion owner, both barcode
decoder profiles and the sampling/reporting integration module. The compiler
associated a function contract with each active definition in 14 Arm compilation
contexts. These contracts describe queue overflow, STOP/reset precedence,
sensor freshness, caller outputs on failure and task ownership where applicable.
The per-profile counts appear under `phase2_owner_review` in the
[comparison receipt](evidence/phase2-mechanical.json).

### Function coverage on 20 November

The source review identified 1,761 owned function definitions, including nine
weak callbacks. Each has a contract on its definition or declaration. All
300 wholly owned source and header files have a file-level documentation
marker. The four mixed-ownership files contain team edits to directives or
calls and contribute no team-owned function definitions.

Clang associated documentation with all 786 distinct host-test definitions
in 52 compiler configurations and all 672 production definitions in 61 Arm
translation units. These counts establish rule 2.2.h documentation coverage.
The generated reference and the accuracy of every individual contract still
need separate review. [Coverage and source bindings](evidence/phase2-mechanical.json)

### Earlier interface checkpoint

The interface checkpoint added 159 contracts covering diagnostics, course
and navigation simulation, decoder fixtures and existing test helpers.
In 52 Linux host-test compiler configurations, Clang attached documentation
to 432 of 786 distinct fixture definitions. All 672 production definitions
had compiler-associated documentation in the 61 inspected Arm translation
units. The host count includes the fixture headers selected by each test.
Subsequent fixture and conditional-source review produced the totals above.
[Configuration and source hashes](evidence/phase2-mechanical.json)

In the production compiler profile, Clang attached documentation to all 672
unique owned function definitions in 64 source and header files across 61
translation units. This includes application functions, header-defined
functions and network integration. The production count excludes host-only
functions and code disabled in that profile. Separate diagnostics builds
exercise the startup and motion smoke-test code. The counts and source
bindings are under `phase2_runtime_and_header_review` in the
[comparison receipt](evidence/phase2-mechanical.json).
