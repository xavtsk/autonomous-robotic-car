# Submission results and verification history

## Mission requirements

The course brief lists ten mission requirements. The team completed the live
demo, then dismantled the car. The [demo record](../testing/system/TEST_LOG.md)
contains no completion time, run-by-run score or firmware revision. The table
identifies which results can be reproduced from the repository.

| Course requirement | Implementation | Result and evidence |
| --- | --- | --- |
| 1. Follow the line autonomously | [Line controller](../app_program/subsystem3/line_controller.c) | **Measured on the car:** the two mounted front sensors had distinct white and black ranges. **Simulation:** the [course tests](../testing/integration/course_sim_main.c) exercise following, line loss and recovery. Track completion reliability was not measured on the submitted revision. [Calibration](../testing/subsystem3/IR_CALIBRATION_2026-10-27.md) |
| 2. Detect and decode barcodes | [Code 39 decoder](../app_program/subsystem3/bc_decoder.c) | **Recorded-signal replay:** all 1,008 variants matched the deployed decoder, and all 3,110 reviewed assertions passed. Several variants reuse each capture. Two historical expectations were corrected after measuring the white margin at each recording's end. [A and Z measurements](../testing/subsystem3/fixtures/barcode/README.md#submission-review-of-two-recorded-endpoints) · [Replay receipt](compliance/evidence/reviewed-barcode-replay-20261127.json) |
| 3. Execute left, right, straight and U-turn commands | [Vehicle drive controller](../app_program/controller/s3_drive.c) and [motion API](../app_program/subsystem2/motion.h) | **Simulation:** the [drive tests](../testing/subsystem3/test_drive_main.c) check all four actions and completion handling. **Measured on the car:** mean errors on the 0.2 m and 0.5 m trials were within 3.3 mm. The turn and braking trials have separate results. [Motion measurements](../testing/subsystem2/TEST_LOG.md#results) |
| 4. Detect humps | [Hump detector](../app_program/subsystem4/hump_detection.c) | **Simulation:** each of two 10 mm right-wheel profiles produced one completed crossing. A pivot and an isolated spike produced no extra crossing. [Terrain analysis](../app_program/subsystem4/TERRAIN_ANALYSIS.md) |
| 5. Measure and report the highest hump | [Terrain processing](../app_program/subsystem4/terr_data.c) and [controller reporting](../app_program/controller/terr_link.c) | **Simulation:** completed 10 mm and 15 mm rises retained the 15 mm maximum after an interrupted 20 mm rise. Physical peak-height error is unmeasured. [Terrain tests](../testing/subsystem4/README.md) |
| 6. Detect obstacles on or near the line | [Ultrasonic driver](../app_program/subsystem5/ultrasonic_sensor.c) | **Simulation:** register-level tests distinguish a completed pulse, no pulse and a timeout with ECHO still high. The last condition can report 4,000 mm even for a stuck signal. [Driver tests](../testing/subsystem5/hardware_main.c) |
| 7. Profile obstacle shape and position | [Scanner](../app_program/subsystem5/scan_obstacle.c) and [profile estimator](../app_program/subsystem5/profile_obstacle.c) | **Simulation:** coarse and fine returns produce a routing envelope and, when the readings support it, bounds on the front-face width. Physical width error is unmeasured. [Scan tests](../testing/subsystem5/README.md) |
| 8. Bypass obstacles without collision | [Avoidance planner](../app_program/subsystem5/avoidance_planner.c) | **Simulation:** one short-turn case returned to the line after contact. The simulator reports −1 mm to flag a collision and does not measure penetration depth. Front sensing leaves the space behind the car unobserved during automatic reverse. [Bypass results](../app_program/subsystem5/README.md) |
| 9. Reacquire the original line and continue | [Line recovery](../app_program/subsystem5/line_recovery.c) | **Simulation:** 16/17 standard bypasses and 80/85 across all five configurations met all mission conditions. A tight bend repeatedly ended in `NO_LINE`. The total includes the collision in row 8. [Bypass test procedure](../testing/subsystem5/README.md) |
| 10. Report telemetry over Wi-Fi | [MQTT protocol](../app_program/subsystem1/MQTT_PROTOCOL.md) and [controller adapter](../app_program/controller/vehicle_controller.c) | **Measured on the car:** the deployed baseline delivered 818 packets with zero malformed while stopped. This establishes a live data path. The record does not cover a moving course run or hotspot recovery. [Deployment receipt](compliance/evidence/baseline-deployment-20261119.json) |

### One command from decoding to telemetry

In the [drive fixture](../testing/subsystem3/test_drive_main.c), a synthetic A
uses that test's left-turn mapping. At the junction the controller brakes,
requests one turn from Buddy 2, consumes the matching completion and resumes
following once the front sensors reacquire the line. The [adapter
fixture](../testing/integration/team_adapter_main.c) separately verifies that
Buddy 1 receives barcode events and signed wheel speeds, retries a full outbox,
and publishes zero on halt. These tests check the interfaces at each handoff.
The deployed car used its own configurable letter mapping.

The [combined terrain and avoidance test](../testing/subsystem5/s5_ctrl_main.c)
also injects an impact during a detour and checks that the controller cancels
the bypass and brakes both simulated motors. The measured braking distance in
the separate physical stop trials was 10–12 mm. [Motion test log](../testing/subsystem2/TEST_LOG.md#results)

### Whole-repo verification

Two clean Ubuntu 24.04 checkouts of the review snapshot ran the same 22
checks, with a local MQTT broker and no attached hardware. The default command
exited 0. A separate `--strict` run exited 1 for the five simulated bypass
misses. All 22 comparisons to the submission criteria and baselines passed.
The BARR source check independently reported its four existing host-fixture
violations. The only skip was an optional dashboard test for a local hardware
capture. [Repeat the checks](BUILD_AND_TEST.md#check-the-submitted-system)

The [matched Arm builds](CHANGE_REQUIREMENT_1_EVIDENCE.md#integrated-build-after-the-submission-test-changes)
produced byte-identical UF2 images. Physical hump-height accuracy, course
completion time and integrated timing under load remain unmeasured for this image.

## GitHub submission at `8131e1a`, 27 November 2026

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
