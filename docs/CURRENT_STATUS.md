# Submission results and verification history

## Final GitHub submission, 27 November 2026

The team completed the demonstration and dismantled the car. The record does
not identify the demo firmware revision. The final software checks used
fresh Arm firmware builds, host regressions and saved barcode waveforms.

| Check | Result |
| --- | --- |
| Field widths and RAM | The checker passed for 167 selected fields and 19 encoded layouts. The integrated builds used 134,204 B of static RAM both before and after the final drive change. [CR1 measurements](CHANGE_REQUIREMENT_1_EVIDENCE.md) |
| Controller and navigation | The Buddy 3 host suite and controller integration suite passed. All 25 simulated course cases produced identical output before and after the drive change. [Build and test commands](BUILD_AND_TEST.md) |
| Recorded barcodes | All 1,008 replay outputs matched the deployed baseline. Both versions failed the same two fixture expectations. [Replay results](compliance/evidence/post-demo-barcode-replay-20261127.json) |
| BARR-C | The source checker reported four host-test violations. One wheel-adapter fixture includes a C source file and three macro names in test doubles mirror platform definitions. The register explains these cases and the extent of manual review. [Exception register](compliance/BARR_EXCEPTIONS.md) |
| Hardware coverage | Physical course performance, task intervals and hotspot recovery were unmeasured on the final firmware. |

## Phase 4 release candidate, 22 November 2026

The coursework release candidate was prepared on
`codex/compliance-preserve-behaviour`. The [release receipt](compliance/evidence/phase4-release-20261122.json)
binds the working-tree source snapshot, Arm GNU 14.3.1, the pinned Pico SDK,
the build profiles, the firmware hashes and the executed checks.

| Verified local image | Profile | Use |
| --- | --- | --- |
| `candidate-production.uf2` | Integrated, `DIAGNOSTICS=0` | Coursework review candidate |
| `candidate-diagnostics.uf2` | Integrated, `DIAGNOSTICS=1` | Candidate for supervised timing measurements |
| `baseline-diagnostics.uf2` | Baseline `4ad2cc4`, `DIAGNOSTICS=1` | Matched timing reference |

The release process placed the UF2, ELF, map and hash manifest under
`build/release-candidate-20261122/firmware/` in the original local checkout.
It checked each UF2 against its ELF. All three images used the public
example Wi-Fi settings.
[Build profiles](BUILD_AND_TEST.md#target-builds)

### Executed verification

- The production build inputs matched the Phase 3 inputs. The comparison
  matched 80,959 rows across 16 behaviour and packet traces, plus all 1,008
  barcode replays against the fixed baseline. Both revisions failed the same
  two replay expectations. [Baseline comparison](compliance/evidence/phase3-submission-20261122.json)
- The final GCC runner passed 48 of 53 steps. Both affected AddressSanitizer and
  UndefinedBehaviorSanitizer suites passed, including the terrain-to-drive
  fixture. The runner reported the inherited barcode replay failure and
  dashboard environment failures. The offline image lacked Node and some CI
  packages, and its source archive omitted dashboard assets.
- A follow-up on Windows passed 13 frontend tests and 10 dashboard service
  tests, including the isolated MQTT broker test.
  [Verification details](compliance/evidence/phase4-release-20261122.json)

## Phase 3 closeout for submission

The matched Phase 3 builds measured 129,496 B before and 129,480 B after
packing the drive flags and terrain cache. All 11 measured message sizes
matched. The Phase 3 table covered 155 selected widths and 18 codec totals.
The timing-counter checks passed for the baseline and candidate.
[Phase 3 results](CHANGE_REQUIREMENT_1_EVIDENCE.md#phase-3-submission-results) ·
[Current scoped width table](compliance/CR1_SUBMISSION_WIDTHS.md)

## Phase 2 closeout for submission

### BARR-C findings at closeout

The Phase 2 review recorded 67 conditionals nested beyond the limit in rule
8.2.b. The reviewers documented partial coverage of cast justifications,
comment clarity, function contracts and assumptions. The register gives the
rule, affected code and disposition for each recorded case.
[Rule ledger](compliance/BARR_RULE_MATRIX.md) · [Exception register](compliance/BARR_EXCEPTIONS.md)

## Deployed baseline and rollback

The 19 November deployment used `4ad2cc42595722cedb47102e4343298339040d40`.
The original checkout held the rollback image at
`build/compliance-review-20261119/compliance-wifi.uf2`. Its verified SHA-256 is
`690d145ba1a5c3b9771e0af191f01632697306d41274cc9a113a8d00fb1cbffa`.

The 19 November deployment record confirms boot, online telemetry and a
stopped controller. The accepted mapping was A forward, F left, Y right and
Z U-turn. These observations apply to the deployed baseline.
[Baseline deployment receipt](compliance/evidence/baseline-deployment-20261119.json)

## Hardware timing

Integrated IR sampling is configured for 5 ms and motor control for 10 ms.
The stationary IR bench uses 10 ms. The archived protocol defines how those
intervals and hotspot recovery would be measured. [Hardware comparison method](PHYSICAL_VALIDATION.md)

Earlier test results and source review checkpoints are preserved in the
[cleanup history](history/2026-11-22-compliance-checkpoints.md).
