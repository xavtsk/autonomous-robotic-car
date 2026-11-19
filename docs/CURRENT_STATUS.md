# Current status and preservation baseline

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
`d26a9808f00add85ee3b6701b415cdec21444e29`, built with public example network
settings in [Linux run 44587222244](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44587222244).
It has not been flashed. The physical baseline above remains the rollback image.

| Work or evidence | Current result |
| --- | --- |
| Linux regression infrastructure | Implemented: pinned actions/tools, separate baseline/candidate trees, GCC/Clang and ASan/UBSan, retained failure logs. |
| Compact debug packet builder | Executed boundary tests and exact packet traces match; matched production static RAM decreases by 128 bytes. |
| C99 aliases and assertions | Original assertion conditions retained; exercised C99 checks pass. One primitive `float` spelling is retained under pending BARR-P12. |
| Repaired production executable | Entire flash payload is byte-identical to the pre-alias debug-builder image; [matched evidence](compliance/evidence/alias-repaired-target.json). |
| Semantic comparison at `d26a980` | Motion, terrain, obstacle, controller/MQTT, debug packets, navigation and all 1008 barcode replay comparisons match. Additional fixtures compare 802 motion-storage, 854 barcode-work, 776 capture-storage and 2093 fragmented-receive trace rows. |
| Complete regression gate | **Failed:** two original barcode fixture expectations also fail on the fixed baseline. [Investigation](compliance/evidence/BASELINE_FAILURES.md) preserves those expectations. |
| BARR formatting/module comments | Token-preserving batch `6326fcb` covers 239 files. Production bytes match exactly and the repaired differential rerun matches; [record](compliance/evidence/FORMAT_CI_44579973340.md) retains the earlier harness failure. |
| Motion private storage | Six Linux host modes pass, exact 802-row traces match; linked production static RAM decreases by 32 bytes versus the preceding image. [Evidence](compliance/evidence/motion-storage-candidate.json). Physical timing remains pending. |
| Estimator module rename | Production flash payload and UF2 are byte-identical to the motion candidate, with link order preserved. [Evidence](compliance/evidence/estimator-rename-linux.json). |
| MQTT private storage | All six Linux host modes and four MQTT profiles pass; linked static RAM decreases by 24 bytes while text increases by 40 bytes. Packet bytes and retry/receive traces match. [Evidence](compliance/evidence/mqtt-state-linux.json). |
| Conditional-compilation collector | Repaired parser executes successfully in all 12 target jobs. Per-profile branch receipts are retained; their union still requires closure review. Collector success alone is not full branch coverage. |
| Header pairs and function contracts | Twenty standalone test pairs and 19 controller-debug contracts have bounded source/compiler proofs; their exact historical commits are checked by CI. Project-wide naming and documentation remain open. |
| Barcode phase extraction | Four helpers preserve the original dispatcher guards and per-update budget; inverse-token and local compile checks pass. Extracted-candidate Linux traces and target matrix are pending. |
| Complete BARR / CR1 closure | **Incomplete:** remaining field gaps, function contracts, statement/naming rules, profile coverage and pending exceptions remain in their ledgers. |
| New candidate physical behavior | **Not verified:** no cleanup image has been flashed or driven in this implementation session. |

The [alias/assertion batch record](compliance/ALIAS_ASSERT_BATCH.md) and
[compiler investigation](compliance/evidence/ALIAS_TARGET_INVESTIGATION.md)
distinguish source equivalence, executed traces and target-byte evidence.
The later branch commits and working tree are implementation work in progress;
they do not inherit the exact-image or execution result of `d26a980`.

The inspected `d26a980` production image has 466,036 text bytes, 696 data bytes
and 128,816 BSS bytes (129,512 bytes of static RAM). Relative to the fixed
baseline's matched CI profile, text decreases by 240 bytes and static RAM by
184 bytes. These are actual linker allocations, not a sum of information bits.
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
