# Current status and preservation baseline

> Housekeeping checkpoint, 20 November 2026: the latest retained full-matrix receipt
> is [revision a3de92a](compliance/evidence/checkpoint-a3de92a-linux.json).
> Its affected checks pass in six host modes and twelve target profiles; the
> two original replay expectation failures remain. The detailed 55af85c summary
> below is the previous checkpoint and has not yet been consolidated for the
> final candidate. Subsequent working-tree edits have separate bounded checks
> in the [cleanup record](REPOSITORY_CLEANUP.md), not a new firmware execution claim.
> Phase 2 source revision `903057d35ecc1c8551f58c357ae1517c58bd16f5` is published.
> Its [four-job Linux run](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44699456784)
> has matching semantic/replay outputs and a successful production build.
> CI remains red for the two unchanged baseline replay expectations. The
> production UF2 matches the previous a3de92a cleanup build byte for byte;
> remaining BARR work and physical validation are still pending.

The cleanup baseline is `4ad2cc42595722cedb47102e4343298339040d40`.
The local firmware `build/compliance-review-20261119/compliance-wifi.uf2` has
SHA-256 `690d145ba1a5c3b9771e0af191f01632697306d41274cc9a113a8d00fb1cbffa`.
The similarly named `baseline.uf2` in that directory belongs to an earlier
comparison revision and is not this deployed baseline.

The [sanitized deployment receipt](compliance/evidence/baseline-deployment-20261119.json),
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

The [BARR ledger](compliance/BARR_RULE_MATRIX.md) records all chapters and nested
subclauses. Its initial entries are unreviewed; lexical/AST findings and missing
manual evidence are reported separately. Specific pending cases are listed in
[BARR_EXCEPTIONS.md](compliance/BARR_EXCEPTIONS.md), with no approval inferred.
The [CR1 inventory](compliance/CR1_FIELD_INVENTORY.md) records representation
coverage and unresolved domains rather than assuming every stored field is minimal.

The [Linux regression workflow](../.github/workflows/compliance.yml) is the route
for executed C evidence. The initial run exposed an unchanged barcode fixture
expectation failure in both revisions; see the
[baseline failure record](compliance/evidence/BASELINE_FAILURES.md). Matching
semantic outputs and passing exercised C checks are separate from a passing
complete replay gate. Later candidates require their own exact run and artifacts.
Local Windows compile/link checks must remain distinct because Application
Control has blocked generated native test programs in this workspace.

The most recently examined complete production image is cleanup revision
`55af85ce084fa0fe1fe4f8fe6444cd78e5acf007`, built with public example network
settings in [Linux run 44604115125](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44604115125).
The [checkpoint receipt](compliance/evidence/checkpoint-55af85c-linux.json)
records all six host modes, twelve target profiles and downloaded trace hashes.
It has not been flashed. The physical baseline above remains the rollback image.

| Work or evidence | Current result |
| --- | --- |
| Linux regression infrastructure | Implemented: pinned actions/tools, separate baseline/candidate trees, GCC/Clang and ASan/UBSan, retained failure logs. |
| Compact debug packet builder | Executed boundary tests and exact packet traces match; matched production static RAM decreases by 128 bytes. |
| C99 aliases and assertions | Original assertion conditions retained; exercised C99 checks pass. One primitive `float` spelling is retained under pending BARR-P12. |
| Repaired production executable | Entire flash payload is byte-identical to the pre-alias debug-builder image; [matched evidence](compliance/evidence/alias-repaired-target.json). |
| Semantic comparison at `302100e` | Motion, terrain, obstacle, controller/MQTT, debug packets, navigation and all 1008 barcode replay comparisons match. Additional fixtures compare 802 motion-storage, 854 barcode-work, 776 capture-storage and 2093 fragmented-receive trace rows. All six host modes execute these four characterization fixtures successfully, including sanitizers. |
| Operator observations at `b4ad5d0` | All six Linux compiler/sanitizer modes pass. Exact 8066 state rows and 8173 total output lines match the baseline, including console bytes; [source-bound proof](compliance/evidence/operator-linux.json). |
| Complete regression gate | **Failed:** two original barcode fixture expectations also fail on the fixed baseline. [Investigation](compliance/evidence/BASELINE_FAILURES.md) preserves those expectations. The two stale formatter mocks exposed at `302100e` were corrected and now pass; their historical failures remain recorded. |
| Historical test contexts | Three obsolete harnesses fail to compile on both fixed baseline and candidate against the current APIs. [Exact commands and diagnostics](compliance/evidence/historical-fixture-compilation.json) are retained; no replacement API or weakened assertion hides these gaps. |
| BARR formatting/module comments | Historical token-preserving batch `6326fcb` covered 239 files; the later provenance audit restored 15 mistakenly included imported files. Production-byte and repaired differential results remain in the [historical record](compliance/evidence/FORMAT_CI_44579973340.md), including its earlier harness failure. |
| Motion private storage | Six Linux host modes pass, exact 802-row traces match; linked production static RAM decreases by 32 bytes versus the preceding image. [Evidence](compliance/evidence/motion-storage-candidate.json). Physical timing remains pending. |
| Estimator module rename | Production flash payload and UF2 are byte-identical to the motion candidate, with link order preserved. [Evidence](compliance/evidence/estimator-rename-linux.json). |
| MQTT private storage | All six Linux host modes and four MQTT profiles pass; linked static RAM decreases by 24 bytes while text increases by 40 bytes. Packet bytes and retry/receive traces match. [Evidence](compliance/evidence/mqtt-state-linux.json). |
| Conditional-compilation collector | At `55af85c`, all 14 host/target receipts pass with zero collector/receipt errors and no missing profiles. The official union is **incomplete: 398/440 explicit arms observed, 42 unobserved** (artifact `14569062961`). Fixture-path binding and all seven added host recipes are now exercised; additional fixture/baseline profiles remain work in progress. |
| Header pairs and function contracts | Twenty standalone test pairs, controller-debug and motion/IR contracts, fake guards and the shared navigation header pass their bounded Linux audit gates at `302100e`; 177 tooling tests pass without skips. Project-wide naming and documentation remain open. |
| Barcode phase extraction | First four helpers, three pruning helpers and three edge helpers pass six host modes, all 854 detailed work rows and all 12 target builds. At `55af85c`, all 1008 replay outputs still match with two original expectations failing. [Edge evidence](compliance/evidence/barcode-edge-extraction.json) preserves the differing instruction/table bytes and separate timing limit; barcode compiler stack estimates remain unchanged. |
| Capture private storage | Five recorder-owned fields compacted without changing serializers or function bodies. All six Linux host modes and the exact 776-row differential trace pass. Linked `g_bcap` decreases from 32736 to 32728 bytes; production static RAM decreases by 8 bytes versus `3fda4d5`, with all compiler stack rows unchanged. [Evidence](compliance/evidence/bcap-storage-candidate.json). |
| Imported template preservation | Fifteen unchanged imported files restored to fixed-baseline text; [provenance correction](compliance/TEMPLATE_BOUNDARY_CORRECTION.md). Actual team network modifications remain in scope. |
| Generated documentation | Doxygen 1.9.8 actually generated HTML/XML at `55af85c`, with source preservation verified. **Incomplete:** 4101 warning-log lines, 980 compiler recovery variants, 1512 undocumented variants and 27 missing observed definitions. An additional obsolete `HTML_TIMESTAMP` configuration warning is retained in the generation log. Artifact `14569583700` contains the reference and diagnostics. |
| Data inventory binding | 270 owned/mixed files and 9,250 declarations recorded, with consumer/header dependencies checked. Integrity passes; 192 representation items (162 gaps and 30 unresolved) and the same three historical fixtures remain open. Part of the count reduction corrects exposed-workspace domain assumptions; it is not claimed as RAM savings. |
| Latest private integration batch | Executed at `55af85c`: all six host integration/operator modes pass; 2398 runtime rows and 8066 operator rows match exactly. Linked production static RAM decreases by 8 bytes and text increases by 32 bytes versus `b4ad5d0`; `sensor_sample_task` compiler stack estimate decreases from 488 to 456 bytes, the sole change among 1441 matched rows. [Bounded evidence](compliance/evidence/integration-private-storage.json) preserves earlier object-only measurements separately. |
| Added host recipes | Fit-error, incremental decoder, bench lifecycle, three size reports and zero-axle course simulation pass all 84 build/execution checks across the six host modes. Existing assertions remain unchanged; no new baseline failures appeared. |
| Complete BARR / CR1 closure | **Incomplete:** remaining field gaps, function contracts, statement/naming rules, profile coverage and pending exceptions remain in their ledgers. |
| New candidate physical behavior | **Not verified:** no cleanup image has been flashed or driven in this implementation session. |

The [alias/assertion batch record](compliance/ALIAS_ASSERT_BATCH.md) and
[compiler investigation](compliance/evidence/ALIAS_TARGET_INVESTIGATION.md)
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

Use [BUILD_AND_TEST.md](BUILD_AND_TEST.md) for commands and the
[documentation index](README.md) for current interfaces. Dated reports and logs
remain historical. Their earlier "not flashed" wording describes the time of
that record and is superseded only by the later receipt for the matching hash.
