# Current status and preservation baseline

> Archived checkpoint through Phase 3. See [current status](../CURRENT_STATUS.md) for the release and validation summary.

## Phase 3 closeout for submission

Phase 3 source changes and desk verification are complete for the agreed
submission scope. Packed drive flags and terrain report state reduced linked
static RAM from 129,496 B to 129,480 B with the same production profile.
All 11 measured message sizes and the exercised wire bytes matched.

One final comparison matched the fixed baseline across 16 behaviour and
wire traces and 1,008 barcode replays, including the two inherited replay
expectation failures. The scoped width and timing-counter checks passed.
Live timing and physical course validation remain pending.
[Phase 3 results](../CHANGE_REQUIREMENT_1_EVIDENCE.md#phase-3-submission-results) ·
[Current scoped width table](../compliance/CR1_SUBMISSION_WIDTHS.md)

## Phase 2 closeout for submission

Phase 2 is complete for the agreed coursework submission scope on
`codex/compliance-preserve-behaviour`. The submission uses the existing
source cleanup and final comparison against the fixed baseline
`4ad2cc42595722cedb47102e4343298339040d40`.

The 22 November source review clarified conversion bounds, operating
assumptions and function contracts. Disabled subsystem implementations
and host fixtures now use the corresponding pointer and Boolean names.
Telemetry packet layout dispatch and terrain startup cleanup removed seven
of the 74 over-depth conditionals in 150 matched compiler contexts. The
exception register groups the remaining 67 by path and preservation evidence.
[Source review](../compliance/evidence/phase2-mechanical.json#phase2_layout_dispatch_and_source_contract_review) ·
[Final source batch](../compliance/evidence/phase2-mechanical.json#phase2_final_source_review) ·
[Conditional nesting register](../compliance/BARR_EXCEPTIONS.md#remaining-conditional-nesting)

The final baseline comparison matched 80,959 rows across 16 semantic and
wire traces. All 1,008 barcode replay cases matched. The comparison runner
returned failure for the same two inherited replay expectations. Earlier
expectations and their recorded outcomes are preserved. The scoped style
check covered 59 files. Three reported alignment differences were corrected
with unchanged C tokens and passed the scoped recheck.
[Final comparison](../compliance/evidence/phase2-mechanical.json#phase2_end_of_phase_verification_20261122)

The Arm object comparisons used the immediately preceding Phase 2 snapshot.
The telemetry codec and the two checked terrain profiles compiled with
different allocated instructions. The terrain stack estimates matched. The
telemetry encoder estimate decreased from 48 to 40 bytes. On-car timing and
course validation remain pending. The candidate has not been flashed.
[Arm comparison](../compliance/evidence/phase2-mechanical.json#phase2_end_of_phase_verification_20261122)

### Remaining BARR-C findings

Rule 8.2.b still has 67 over-depth conditionals. The ledger also records
incomplete reviews of cast justifications (1.6.a), comment clarity (2.2.a),
contract detail (2.2.d) and assumptions (2.2.g). These findings may affect
marks if the rubric assesses the rules individually. Full BARR-C compliance
remains unverified. Revisit individual refactors, compiler-output comparisons
or formal exception approvals if the rubric or lecturer requires them.
[Rule ledger](../compliance/BARR_RULE_MATRIX.md) ·
[Exception register](../compliance/BARR_EXCEPTIONS.md)

### Earlier Phase 2 checkpoints

Each checkpoint below records its own verification scope.

The cleanup branch has reviewed source ordering, integer domains, bit masks,
identifier names and fixture loop bounds. The expanded compiler review covers
150 contexts and 127 of 130 owned direct C entry units. The three remaining
entries are superseded or unsupported fixtures. The signedness review found
no mixed signed/unsigned arithmetic or comparisons in the recorded scope.
Retained signed bitwise sites are listed in the
[exception register](../compliance/BARR_EXCEPTIONS.md).

The source-section review now ties its 29 remaining observations to six
pending exceptions. The active loop-bound review completed its targeted
checks. The separate historical IRQ harness still fails to compile against
the current API and retains its original bounds.

The assignment review found zero unaligned operators in 1,038 written blocks
across 150 supported compiler contexts. The last Buddy 2 edits preserved all 22
console assertion locations. An expanded compiler review reached 23 additional
entry units. Fixture corrections passed 17 paired output comparisons against
the preceding checkpoint, including two recorded barcode inputs. All 14 Arm
layout symbols in the size probe matched. Three superseded or unsupported
fixtures are outside the direct compiler review.

The earlier sampling, post-turn recovery, reference decoder and obstacle
changes passed focused Linux subsystem and integration suites. Paired
Arm comparisons matched allocated instructions, data and stack estimates
for those applied changes. The nesting review identifies 74 branches
over two levels, primarily in obstacle avoidance and barcode decoding.
Trials that changed generated code are recorded in the exception register.

New comments document the sensor and wire ranges, MQTT callback ownership,
motion limits and behaviour of the capture buffers. Cast comments, sentence
clarity, complex contracts and assumptions still require source-wide review.
The course comments describe the 5 ms integrated IR period, normalized
reflectance thresholds, turn reacquisition and assumed tape geometry.

The fit-score and bounded-work barcode fixtures now link the decoder
separately. Matched Linux runs reproduced all 432 fit boundary cases,
120,446 random fits and 156 work-quota scan cases. The terrain transport,
sonar task, built-in calibration, diagnostics and map-lock fixtures also
compile the production modules separately. Their paired Linux checks
matched, and the compared Arm profiles matched with the probes disabled.

The lexical audit reports four unresolved directives. The superseded
wheel-adapter fixture still directly includes C implementation. Three macro
names mirror imported SDK or kernel names. The
[current scope review](../compliance/evidence/phase2-mechanical.json#phase2_remaining_mandatory_checkpoints)
records the exact source locations and the latest compiler branch-depth
count. The bounded comment scans reviewed 5,319 owned comments and found
no ordinary fragments. Full review of Doxygen and prose remains pending.

The barcode phase-boundary, retained-motion, GP20/GP21 and sampling runtime
fixtures now build their firmware owners separately. Their observed Linux
runs matched all 854, 802, 8,173 and 2,443 output lines per build,
respectively. The sampling fixture also matched its plain, MQTT, integrated
IR diagnostic and terrain slope test outputs. Its probes are disabled in
the compared Arm firmware profiles. [Sampling runtime evidence](../compliance/evidence/phase2-mechanical.json#phase2_separate_sampling_runtime_fixture)

The latest clock cleanup removed 21 redundant casts in sensor, motion,
avoidance, diagnostics and network timing. The 21 compared Arm objects
matched. Focused Linux subsystem, integration and diagnostics checks passed.
[Sensor and motion clock evidence](../compliance/evidence/phase2-mechanical.json#phase2_sensor_and_motion_lease_elapsed_casts) ·
[Network clock evidence](../compliance/evidence/phase2-mechanical.json#phase2_wifi_retry_elapsed_casts)

Strict Arm C99 syntax checks passed for 59 of 61 recorded production
commands. Imported kernel and Pico SDK headers caused the remaining two
failures. Nine selected alternate contexts passed. The existing production
toolchain mode remains in use.
[Production C99 evidence](../compliance/evidence/phase2-mechanical.json#phase2_c99_recorded_production_profiles) ·
[Alternate C99 evidence](../compliance/evidence/phase2-mechanical.json#phase2_c99_selected_conditional_profiles)

The normal host runner now supplies the fixed baseline's existing member
aliases to the motion, capture and operator fixtures. Motion storage,
barcode boundaries, capture storage and operator observations all passed
against both subjects, with exact standard output for the scoped checks.
[Host runner evidence](../compliance/evidence/phase2-mechanical.json#phase2_regular_host_fixture_compatibility)

The local Phase 2 comparison on 21 November 2026 covered 16 semantic
and wire traces with 80,959 rows. All 1,008 recorded barcode cases matched
the fixed baseline, including its two failed expectations.

All nine Linux suites passed after the public type rename. The paired Arm
check matched allocated sections, relocations and stack estimates in
60 production objects and eight conditional profiles. The separate
precedence checkpoint covered all 61 production objects and five selected
alternate profiles. The scoped Change Requirement 1 width checker also
passed after its field names were synchronized with the C declarations.

The operator and timing API cleanup matched all 17 comparisons and seven
additional Arm checks. Four affected Linux suites passed after the subsequent
host fixture naming changes.

The sampling calibration helper matched four Arm profiles. The compiler
review confirmed the declared order of 299 bit-fields in 85 records across
113 compilations. The next checkpoint kept all 17 comparisons identical
after renaming scaled capture and tilt fields. The four previously observed
host branches over two levels of nesting were resolved and their suites
passed. The remaining 99 observed branches are in firmware.

The controller and navigation codec APIs matched all 17 comparisons after
their module prefixes were applied. Five affected Linux suites and 11 Arm
checks passed, including the stationary IR bench and Buddy 4 self-test
profiles. The static-linkage trials for the operator labels and command
kind helper are recorded as pending under BARR-P17.

The equality and byte-type cleanup standardized 1,423 host fixture
comparisons and kept device self-test diagnostic locations intact. All nine
Linux suites, the packet builder and the scoped representation checks passed.
All 17 baseline comparisons matched, including the two original replay
expectation failures.

The declaration-order and precedence review covered 113 compile
configurations and passed six affected Linux suites and 11 paired Arm checks.
All 17 baseline comparisons matched on that snapshot.

The capture guard and hump API cleanup matched 21 paired Arm checks,
including the separate unsigned-literal review. The latest MQTT fixture
changes kept all 17 baseline comparisons identical and passed four affected
Linux suites and the fixture tooling tests. The eight remaining pointer
prefix sites are host platform interfaces recorded under BARR-P20.

The header dependency and fixture naming cleanup passed 19 paired Arm
checks and kept all 17 baseline comparisons matched. The global-prefix
review leaves seven platform names pending under BARR-P21.

The terrain, motion packet, barcode, IR, and peripheral API naming
checkpoints passed 53 paired Arm checks. The public type and function
checkpoints passed another 44 paired Arm checks and all nine Linux suites
on each snapshot. All 17 baseline comparisons matched after each of these
checkpoints. The exception register identifies retained course and platform
entry points and the names used by on-device failure diagnostics.

The control-block spacing review separated 2,994 adjacent boundaries
without changing C tokens. A repeat scan across 120 configurations found
82 retained sites, all in the three on-device self-tests whose failure
locations must be preserved. The 300-file style gate and 24 paired Arm
checks passed.

The declaration cleanup moved 146 locals to their first existing assignment.
All 42 paired Arm checks and nine Linux suites passed, and all 17 baseline
comparisons matched. A separate warning audit found zero uninitialized-value
diagnostics in 73 Arm configurations. The numeric expression review matched
13 paired Arm checks after documenting existing conversions in six modules.

The source-section cleanup moved 104 private helpers in 19 host fixtures.
The Pico build now enables GCC initialization warnings in application and
team network sources. The combined-prefix cleanup covered 101 names. The
last full host snapshot passed all nine Linux suites and all 17 baseline
comparisons, with the two original replay expectation failures reproduced.
Seventy-five paired Arm checks also matched after normalizing renamed symbols.

The extended identifier review included device self-tests, the stationary
IR bench, the IMU preview and the synchronous barcode reference. Only the
two short RTOS time fields remain, pending under BARR-P31. The latest
bounded host and paired Arm checks matched their respective snapshots.

The subsequent changes added const qualifiers to 107 local variables,
clarified controller predicates and private codec helpers, and aligned
function arguments with their documentation. The 122-configuration audit
found that all 422 parameter-annotated declarations use the documented
names. The corresponding Linux suites and baseline comparisons matched.

Another 52 declarations now sit closer to first use, with 29 matching Arm
checks. The cleanup also named barcode, packet and retry test bounds,
reordered the operator panel source sections, and moved the terrain
startup failure case first. The line-following function is down from 110
to 95 body lines in its extraction comparison. The latest length audit
found 71 owned function bodies over 100 lines across 122 configurations,
including one added by line wrapping in the telemetry decoder.

The enum, narrow-integer and bit-field batches made existing arithmetic
domains explicit. The target scan reports 43 remaining mixed declared-type
sites across 3,504 expressions, down from 447 before these batches. The
Arm comparisons and Linux suites passed for the applied changes, and all
17 baseline comparisons matched. The replay comparison still reproduces
the two baseline expectation failures.

The line-following and motion modules now put public functions before their
private implementations. The 122-configuration section audit found no
ordering candidates in these two files and 234 elsewhere. The corresponding
Arm comparisons, affected Linux suites and baseline comparisons matched.

An earlier ledger snapshot recorded 97 verified rules, nine inapplicable
rules, 36 pending deviations and 25 unreviewed rules out of 167 entries.
The accompanying 304-file lexical
audit reports 18 violations, all in host fixtures. The full linked release
build and supervised track checks remain open.
[Comparison receipt](../compliance/evidence/phase2-mechanical.json) · [Rule ledger](../compliance/BARR_RULE_MATRIX.md)

The earlier whole-project integrity check recorded 270 files, 9,253
declarations and 178 open representation items before the later Phase 2
source edits. These counts describe that snapshot. The current submission
uses the scoped source bindings and evidence linked above.

> Housekeeping checkpoint, 20 November 2026: the latest retained full-matrix receipt
> is [revision a3de92a](../compliance/evidence/checkpoint-a3de92a-linux.json).
> Its affected checks pass in six host modes and twelve target profiles; the
> two original replay expectation failures remain. The detailed 55af85c summary
> below is the previous checkpoint and has not yet been consolidated for the
> final candidate. Subsequent working-tree edits have separate bounded checks
> in the [cleanup record](../REPOSITORY_CLEANUP.md), not a new firmware execution claim.
> Phase 2 source revision `903057d35ecc1c8551f58c357ae1517c58bd16f5` is published.
> Its [four-job Linux run](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44699456784)
> has matching semantic/replay outputs and a successful production build.
> CI remains red for the two unchanged baseline replay expectations. The
> production UF2 matches the previous a3de92a cleanup build byte for byte;
> remaining BARR work and physical validation are still pending.
> Subsequent local Phase 2 batches are described in the
> [cleanup record](../REPOSITORY_CLEANUP.md), including host-interface type cleanup
> and obstacle acquisition/recovery contracts.
> The latest local Phase 2 snapshot has Linux GCC results for all five
> subsystems, integration, diagnostics and Wi-Fi recovery. Sixteen semantic
> and wire comparisons match the fixed baseline across 80,959 output rows.
> All 1,008 barcode replay cases match, including the two existing failed
> expectations. The final two source-formatting edits have matching compiler
> tokens and Arm objects. Full-course validation and a linked firmware image
> for this working tree remain pending. [Local source-review evidence](../compliance/evidence/phase2-mechanical.json)
>
> The subsequent host-fixture and control-statement review reduced the lexical
> findings to 22. The final source still matches all 16 semantic and wire
> traces and all 1,008 replay cases. All 300 wholly owned files pass the
> formatter. The latest Arm evidence covers 13 object comparisons.
> [Phase 2 results and open items](../REPOSITORY_CLEANUP.md#phase-2-host-fixtures-and-control-statements)
>
> The subsequent owner review documents five runtime modules in 14 Arm
> configurations. Nine Linux suites passed. The 80,959 trace rows and 1,008
> barcode cases still matched the fixed baseline. The rule ledger at that
> checkpoint had 43 verified rules, three inapplicable rules and one pending
> deviation. [Owner review](../REPOSITORY_CLEANUP.md#phase-2-owner-review)

The cleanup baseline is `4ad2cc42595722cedb47102e4343298339040d40`.
The local firmware `build/compliance-review-20261119/compliance-wifi.uf2` has
SHA-256 `690d145ba1a5c3b9771e0af191f01632697306d41274cc9a113a8d00fb1cbffa`.
The similarly named `baseline.uf2` in that directory belongs to an earlier
comparison revision and is not this deployed baseline.

The [sanitized deployment receipt](../compliance/evidence/baseline-deployment-20261119.json),
copied from local `build/compliance-review-20261119/flash-receipt.json`, records:

- Copy completion on 19 November 2026 at 14:30 UTC and subsequent boot verification.
- Online telemetry verification at 14:31:47 UTC: 818 received messages and zero
  malformed messages at that snapshot.
- An accepted mapping of A forward, F left, Y right and Z U-turn. The protocol
  has no map request ID or readback; acceptance is an OK acknowledgement.
- Fresh control status reporting stopped, not running, state `off`, reason `OK`.

This is firmware transfer, boot and online communication evidence. It does not
prove every mapped manoeuvre, calibration accuracy, obstacle avoidance, bounded
physical timing or a completed course. No subsequent cleanup build inherits
those claims without its own validation.

## Cleanup status

The source work aims to preserve baseline behavior while reviewing BARR-C 2018
and Change Requirement 1 across all owned C/header files, tests and listed custom
adapters. Imported kernel/SDK code remains outside the edit scope. External ABI
names and types inside owned adapters require explicit boundary records.

The [BARR ledger](../compliance/BARR_RULE_MATRIX.md) records all chapters and nested
subclauses. Its initial entries are unreviewed; lexical/AST findings and missing
manual evidence are reported separately. Specific pending cases are listed in
[BARR_EXCEPTIONS.md](../compliance/BARR_EXCEPTIONS.md), with no approval inferred.
The [CR1 inventory](../compliance/CR1_FIELD_INVENTORY.md) records representation
coverage and unresolved domains rather than assuming every stored field is minimal.

The [Linux regression workflow](../../.github/workflows/compliance.yml) is the route
for executed C evidence. The initial run exposed an unchanged barcode fixture
expectation failure in both revisions; see the
[baseline failure record](../compliance/evidence/BASELINE_FAILURES.md). Matching
semantic outputs and passing exercised C checks are separate from a passing
complete replay gate. Later candidates require their own exact run and artifacts.
Local Windows compile/link checks must remain distinct because Application
Control has blocked generated native test programs in this workspace.

The most recently examined complete production image is cleanup revision
`55af85ce084fa0fe1fe4f8fe6444cd78e5acf007`, built with public example network
settings in [Linux run 44604115125](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44604115125).
The [checkpoint receipt](../compliance/evidence/checkpoint-55af85c-linux.json)
records all six host modes, twelve target profiles and downloaded trace hashes.
It has not been flashed. The physical baseline above remains the rollback image.

| Work or evidence | Current result |
| --- | --- |
| Linux regression infrastructure | Implemented: pinned actions/tools, separate baseline/candidate trees, GCC/Clang and ASan/UBSan, retained failure logs. |
| Compact debug packet builder | Executed boundary tests and exact packet traces match; matched production static RAM decreases by 128 bytes. |
| C99 aliases and assertions | Original assertion conditions retained; exercised C99 checks pass. One primitive `float` spelling is retained under pending BARR-P12. |
| Repaired production executable | Entire flash payload is byte-identical to the pre-alias debug-builder image; [matched evidence](../compliance/evidence/alias-repaired-target.json). |
| Semantic comparison at `302100e` | Motion, terrain, obstacle, controller/MQTT, debug packets, navigation and all 1008 barcode replay comparisons match. Additional fixtures compare 802 motion-storage, 854 barcode-work, 776 capture-storage and 2093 fragmented-receive trace rows. All six host modes execute these four characterization fixtures successfully, including sanitizers. |
| Operator observations at `b4ad5d0` | All six Linux compiler/sanitizer modes pass. Exact 8066 state rows and 8173 total output lines match the baseline, including console bytes; [source-bound proof](../compliance/evidence/operator-linux.json). |
| Complete regression gate | **Failed:** two original barcode fixture expectations also fail on the fixed baseline. [Investigation](../compliance/evidence/BASELINE_FAILURES.md) preserves those expectations. The two stale formatter mocks exposed at `302100e` were corrected and now pass; their historical failures remain recorded. |
| Historical test contexts | Three obsolete harnesses fail to compile on both fixed baseline and candidate against the current APIs. [Exact commands and diagnostics](../compliance/evidence/historical-fixture-compilation.json) are retained; no replacement API or weakened assertion hides these gaps. |
| BARR formatting/module comments | Historical token-preserving batch `6326fcb` covered 239 files; the later provenance audit restored 15 mistakenly included imported files. Production-byte and repaired differential results remain in the [historical record](../compliance/evidence/FORMAT_CI_44579973340.md), including its earlier harness failure. |
| Motion private storage | Six Linux host modes pass, exact 802-row traces match; linked production static RAM decreases by 32 bytes versus the preceding image. [Evidence](../compliance/evidence/motion-storage-candidate.json). Physical timing remains pending. |
| Estimator module rename | Production flash payload and UF2 are byte-identical to the motion candidate, with link order preserved. [Evidence](../compliance/evidence/estimator-rename-linux.json). |
| MQTT private storage | All six Linux host modes and four MQTT profiles pass; linked static RAM decreases by 24 bytes while text increases by 40 bytes. Packet bytes and retry/receive traces match. [Evidence](../compliance/evidence/mqtt-state-linux.json). |
| Conditional-compilation collector | At `55af85c`, all 14 host/target receipts pass with zero collector/receipt errors and no missing profiles. The official union is **incomplete: 398/440 explicit arms observed, 42 unobserved** (artifact `14569062961`). Fixture-path binding and all seven added host recipes are now exercised; additional fixture/baseline profiles remain work in progress. |
| Header pairs and function contracts | Twenty standalone test pairs, controller-debug and motion/IR contracts, fake guards and the shared navigation header pass their bounded Linux audit gates at `302100e`; 177 tooling tests pass without skips. Project-wide naming and documentation remain open. |
| Barcode phase extraction | First four helpers, three pruning helpers and three edge helpers pass six host modes, all 854 detailed work rows and all 12 target builds. At `55af85c`, all 1008 replay outputs still match with two original expectations failing. [Edge evidence](../compliance/evidence/barcode-edge-extraction.json) preserves the differing instruction/table bytes and separate timing limit; barcode compiler stack estimates remain unchanged. |
| Capture private storage | Five recorder-owned fields compacted without changing serializers or function bodies. All six Linux host modes and the exact 776-row differential trace pass. Linked `g_bcap` decreases from 32736 to 32728 bytes; production static RAM decreases by 8 bytes versus `3fda4d5`, with all compiler stack rows unchanged. [Evidence](../compliance/evidence/bcap-storage-candidate.json). |
| Imported template preservation | Fifteen unchanged imported files restored to fixed-baseline text; [provenance correction](../compliance/TEMPLATE_BOUNDARY_CORRECTION.md). Actual team network modifications remain in scope. |
| Generated documentation | Doxygen 1.9.8 actually generated HTML/XML at `55af85c`, with source preservation verified. **Incomplete:** 4101 warning-log lines, 980 compiler recovery variants, 1512 undocumented variants and 27 missing observed definitions. An additional obsolete `HTML_TIMESTAMP` configuration warning is retained in the generation log. Artifact `14569583700` contains the reference and diagnostics. |
| Data inventory binding | 270 owned/mixed files and 9,250 declarations recorded, with consumer/header dependencies checked. Integrity passes; 192 representation items (162 gaps and 30 unresolved) and the same three historical fixtures remain open. Part of the count reduction corrects exposed-workspace domain assumptions; it is not claimed as RAM savings. |
| Latest private integration batch | Executed at `55af85c`: all six host integration/operator modes pass; 2398 runtime rows and 8066 operator rows match exactly. Linked production static RAM decreases by 8 bytes and text increases by 32 bytes versus `b4ad5d0`; `sensor_sample_task` compiler stack estimate decreases from 488 to 456 bytes, the sole change among 1441 matched rows. [Bounded evidence](../compliance/evidence/integration-private-storage.json) preserves earlier object-only measurements separately. |
| Added host recipes | Fit-error, incremental decoder, bench lifecycle, three size reports and zero-axle course simulation pass all 84 build/execution checks across the six host modes. Existing assertions remain unchanged; no new baseline failures appeared. |
| Complete BARR / CR1 closure | **Incomplete:** remaining field gaps, function contracts, statement/naming rules, profile coverage and pending exceptions remain in their ledgers. |
| New candidate physical behavior | **Not verified:** no cleanup image has been flashed or driven in this implementation session. |

The [alias/assertion batch record](../compliance/ALIAS_ASSERT_BATCH.md) and
[compiler investigation](../compliance/evidence/ALIAS_TARGET_INVESTIGATION.md)
distinguish source equivalence, executed traces and target-byte evidence.
The later branch commits and working tree are implementation work in progress;
they do not inherit the exact-image or execution result of `55af85c`. In
particular, the later tilt filename batch `99b07b7` awaits its own Linux gate.

The inspected `55af85c` production image has 465,884 text bytes, 696 data bytes
and 128,800 BSS bytes (129,496 bytes of static RAM). Relative to the preceding
`b4ad5d0` production image, text increases by 32 bytes and static RAM decreases
by 8 bytes. Relative to the fixed baseline's matched CI profile, text decreases
by 392 bytes and static RAM by 200 bytes. Production artifact `14570590232` has
flash SHA-256 `37a68de18dbbe440935264d845a6acf03b253c43c8501bb598ee5f2ba9cbd392`;
the downloaded ELF, UF2, map and flash hashes were independently verified.
These are actual linker allocations, not a sum of information bits.
Compiler stack estimates and any increases remain in the target manifests;
physical task high-water measurements are still outstanding.

Remaining rollout evidence includes matched final-profile artifacts, complete
software gates, supervised course/barcode/hump/avoidance/control checks, and the
three-minute timing/reconnection session with a ten-second hotspot interruption.
Task timing must be judged against each task's own period. No measured high-water
stack or full-course result is inferred from compiler `.su` estimates. Flashing,
motor operation and merging to `main` remain separate user-directed actions.

Use [BUILD_AND_TEST.md](../BUILD_AND_TEST.md) for commands and the
[documentation index](../README.md) for current interfaces. Dated reports and logs
remain historical. Their earlier "not flashed" wording describes the time of
that record and is superseded only by the later receipt for the matching hash.
