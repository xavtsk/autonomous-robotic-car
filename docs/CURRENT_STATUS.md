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

The [alias/assertion batch record](compliance/ALIAS_ASSERT_BATCH.md) describes
the first C99 type/assertion cleanup and its limited source/compile-only evidence.

Use [BUILD_AND_TEST.md](BUILD_AND_TEST.md) for commands and the
[documentation index](README.md) for current interfaces. Dated reports and logs
remain historical. Their earlier "not flashed" wording describes the time of
that record and is superseded only by the later receipt for the matching hash.
