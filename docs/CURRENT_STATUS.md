# Current status and preservation baseline

## Phase 4 release candidate

The coursework release candidate is prepared on
`codex/compliance-preserve-behaviour`. The [release receipt](compliance/evidence/phase4-release-20261122.json)
binds the working-tree source snapshot, Arm GNU 14.3.1, the pinned Pico SDK,
the build profiles, the firmware hashes and the executed checks.

| Verified local image | Profile | Use |
| --- | --- | --- |
| `candidate-production.uf2` | Integrated, `DIAGNOSTICS=0` | Coursework review candidate |
| `candidate-diagnostics.uf2` | Integrated, `DIAGNOSTICS=1` | Candidate for supervised timing measurements |
| `baseline-diagnostics.uf2` | Baseline `4ad2cc4`, `DIAGNOSTICS=1` | Matched timing reference |

All three UF2 files are in `build/release-candidate-20261122/firmware/`
with their ELF, map and hash manifest. Each UF2's flash bytes were checked
against its ELF. All three use public example Wi-Fi settings. An on-car
rebuild uses the private test network settings and gets its own image hash.
[Build profiles](BUILD_AND_TEST.md#target-builds)

### Executed verification

- The production build inputs match the Phase 3 inputs. The existing comparison
  matched 80,959 rows across 16 behaviour and packet traces, plus all 1,008
  barcode replays against the fixed baseline. The two inherited replay
  expectation failures remain recorded. [Baseline comparison](compliance/evidence/phase3-submission-20261122.json)
- The final GCC runner passed 48 of 53 steps. Both affected AddressSanitizer and
  UndefinedBehaviorSanitizer suites passed, including the terrain-to-drive
  fixture. The runner reported the inherited barcode replay failure and
  dashboard environment failures. The offline image lacked Node and some CI
  packages, and its source archive omitted dashboard assets.
- A targeted follow-up on Windows passed 13 frontend tests and 10 dashboard
  service tests, including the isolated MQTT broker test. The original GCC
  runner exit remains recorded. [Verification details](compliance/evidence/phase4-release-20261122.json)

## Phase 3 closeout for submission

The matched Phase 3 builds measured 129,496 B before and 129,480 B after
packing the drive flags and terrain cache. All 11 measured message sizes
matched. The current table covers 155 selected widths and 18 codec totals.
The timing-counter checks passed for the baseline and candidate.
[Phase 3 results](CHANGE_REQUIREMENT_1_EVIDENCE.md#phase-3-submission-results) ·
[Current scoped width table](compliance/CR1_SUBMISSION_WIDTHS.md)

## Phase 2 closeout for submission

### Remaining BARR-C findings

The agreed submission scope retains 67 over-depth conditionals under rule
8.2.b. Cast justifications, comment clarity, contract detail and assumptions
have incomplete review evidence. The ledger keeps the pending findings
visible. Full BARR-C compliance remains unverified.
[Rule ledger](compliance/BARR_RULE_MATRIX.md) · [Exception register](compliance/BARR_EXCEPTIONS.md)

## Deployed baseline and rollback

The rollback image for `4ad2cc42595722cedb47102e4343298339040d40` is
`build/compliance-review-20261119/compliance-wifi.uf2`. Its verified SHA-256 is
`690d145ba1a5c3b9771e0af191f01632697306d41274cc9a113a8d00fb1cbffa`.

The 19 November deployment record confirms boot, online telemetry and a
stopped controller. The accepted mapping was A forward, F left, Y right and
Z U-turn. These observations apply to the deployed baseline.
[Baseline deployment receipt](compliance/evidence/baseline-deployment-20261119.json)

## Outstanding physical validation

The candidate awaits supervised course and on-car timing verification,
including the hotspot recovery capture. Use 5 ms for integrated IR sampling
and 10 ms for motion control. Stationary IR bench sampling uses 10 ms in its
separate profile. [Supervised validation procedure](PHYSICAL_VALIDATION.md)

Earlier test results and source review checkpoints are preserved in the
[cleanup history](history/2026-11-22-compliance-checkpoints.md).
