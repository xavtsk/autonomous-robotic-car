# Subsystem 3 — test log

**Buddy 3:** IR line following and barcode decoding. Owner: TBD.

Buddy 3 core software is implemented and software-tested. Actual track and
moving-car validation remain pending. The current stationary UF2 has not been
flashed. The general target `TEST=1` hook reports SKIP; the host suite is separate.

## Wiring and measured calibration correction, 2026-10-27

Local `main` based on `6da2977`: left=Grove 6 GP27, right=Grove 5 GP26,
barcode=Grove 7 GP28. Raw button collections used the installed firmware's GPIO
numbers to translate its incorrect logical labels. The user confirmed that one
barcode pair used black path material; that pair is excluded from saved defaults.

| Check | Result | Evidence / limitation |
|---|---|---|
| Per-sensor raw white/black collection | PASS | At least 3 valid pairs per sensor, 101 samples per surface; combined ranges do not overlap |
| Physical ADC routing and barcode isolation | PASS | `test_mapping.c`, independent ADC0/1/2 inputs and line-sign checks |
| Buddy 3 and integration host suites | PASS | Saved-calibration endpoints, bench agreement, operator controls/rejection, navigation, motion feedback and course simulation |
| GP28-only bench, all-three bench, offline integrated firmware | PASS | ARM compile/link and profile-symbol checks; final UF2 structure validated |
| Flash and power-cycle verification | NOT RUN | RPI-RP2 did not appear; user requested finishing the source changes without flashing |

See [raw ranges, exclusions, build hash and verification details](IR_CALIBRATION_2026-10-27.md).
No driving test or firmware flash was performed.

## Updated Buddy 2 integration, 2026-10-22

Working tree based on `buddy3` `9be6b6e`, incorporating Buddy 2 `a29f973`
without a commit/merge commit. Upstream motor settings, PIO encoders, packed
formats and existing test evidence are preserved. No hardware was connected,
flashed or exercised for this update. Upstream photos/logs are teammate evidence,
not new measurements by this integration check.

| Check | Result | Evidence / limitation |
|---|---|---|
| Reflectance, acquisition, calibration, line, barcode and navigation | PASS | Existing strict host suites retained |
| Latest B2 controller/PID/estimator + temporary controller | PASS | `test_drive.c`: matching DONE IDs, old DONE ignored, unexpected cancellation, report-loss flag/counter, bounded inactive drain, brake/stillness wait, all previous scenarios |
| Production motion front end + mocked kernel/PIO/PWM | PASS | `test_motion_feedback.c`: original timestamps, epochs, calibration abort/exclusion, 30 ms turn watchdog, source validity, actual packed report queue overflow, old report acknowledgement, ID wrap, forward/reverse 20-bit counter wrap, clock startup idempotence |
| Normal Wi-Fi / integrated / GP26-only bench / three-sensor bench | PASS | `build/host-tests/build-{normal,s3,ir1,ir3}-updated.log`; no compiler warnings/errors |
| Strict ARM checks | PASS | 27 source/profile combinations, `-Wall -Wextra -Werror -fsyntax-only`; integrated and bench application sources plus touched motion front end/controller |
| Startup and dependency boundaries | PASS | `tools/check-buddy3-profile.ps1`: bench starts S3 only, normal S3 hooks inactive, integrated starts S2/S3 and uses reports, portable core has no motion/RTOS dependencies |
| UF2 structure | PASS | Magic/family/block numbers, unique flash addresses and sizes; `build/host-tests/updated-integration-verification.json` |
| New physical wiring, moving navigation, scheduling and concurrent RTOS commands | NOT VERIFIED | Host mocks and compiled startup checks cannot establish these |

Repeat: `powershell -ExecutionPolicy Bypass -File tools/test-buddy3.ps1`.
Build: `tools/build-s3-motion.ps1`, `tools/build-ir-bench.ps1`, or
`tools/build-ir-bench.ps1 -Sensors 3` from PowerShell. Builds do not flash.

| Image | Bytes / blocks | SHA-256 |
|---|---|---|
| `mtk3pico_smp1_usb_cdc_s3_motion.uf2` | 202240 / 395 | `fe2baa2c37f163ab0027892152f0f47bac4b7d21408da91387b66c6d06b1aba4` |
| `mtk3pico_smp1_usb_cdc_ir_bench.uf2` | 202752 / 396 | `64355f4a7b1596753ee0b61e5f5e92d045634b2e645e6ef9df5efa46a5ea30c5` |
| `mtk3pico_smp1_usb_cdc_ir_bench_3sensor.uf2` | 202752 / 396 | `05dabbddb388433c30dcbc2e57b9c5a6c53b3bd50b06c6ace548953d91da13a8` |

Images are under `build_make/` and ignored by Git. The integrated image is
motor-capable only after explicit calibration/start through the application API;
no USB command parser exists. Both bench images are stationary. The temporary
controller moved to `app_program/controller/`; Buddy 1's draft was not imported.
The 10 ms sampling target and 30 ms freshness/lease limit remain provisional,
and no measured 5 ms compliance is claimed.

Earlier records below describe their dated snapshots and different image hashes.

## Buddy 2 integration acceptance, 2026-10-18

Uncommitted `buddy3` working tree; earlier Week 6 alignment changes preserved.
The new opt-in profile is motor-capable after explicit calibration/start, but no
firmware was flashed and no hardware/motor test was performed.

| Check | Result | Evidence / limit |
|---|---|---|
| Existing reflectance/core/navigation suites | PASS | Warning-clean host tests retained |
| Real B2 controller/PID/estimator + S3 bridge | PASS | `test_drive.c`; mocked encoders/PWM/public wrappers, all four mappings, brake-before-turn, one-shot completion, reacquisition, unmapped/expired directions, stop/cancel, line/sensor/motion faults, rejection and explicit restart |
| Production `motion.c` feedback/lease | PASS | `test_motion_feedback.c` with mocked kernel/hardware; source timestamps preserved across reads, epoch on reinitialisation, calibration exclusion, watchdog brakes TURN |
| Integrated firmware | PASS | `build/host-tests/s3-motion-build.log`, S3_MOTION=1 IR_BENCH=0 WIFI=none TEST=0 |
| Normal Wi-Fi firmware | PASS | `normal-motion-regression.log`; integration off, S3 lifecycle remains no-op |
| Stationary bench firmware | PASS | `bench-motion-regression.log`; only S3 startup, no motion API references in its S3 objects |
| ARM strict warnings | PASS | 15 S3/application/touched B2 C sources, Wall/Wextra/Werror syntax checks; all three firmware build logs warning-free |
| Startup boundaries | PASS | Integrated app_main starts only S2/S3; bench starts only S3; host init/default tests issue no velocity/manoeuvre requests |
| UF2 integrity | PASS | Both images: block numbering, RP2040 family, magic and unique in-range addresses |
| RTOS control queue and concurrent timing | BUILD/STATIC ONLY | Queue/startup inspected; scheduling, concurrent commands and actual stop latency need on-device validation |
| Hardware / navigation / 5 ms compliance | NOT VERIFIED | No flash, actual-track run, obstacle integration or measured timing compliance |

The host command remains `powershell -ExecutionPolicy Bypass -File tools/test-buddy3.ps1`.
The integration build command is `powershell -ExecutionPolicy Bypass -File tools/build-s3-motion.ps1`.

Integrated, disarmed at boot: `build_make/mtk3pico_smp1_usb_cdc_s3_motion.uf2`, 349 blocks / 178688 bytes.
SHA-256: `f0f6f00fc9b204826ba8e481f67c7334da6d52e4288d73399b3b0b110aebb1e3`.

Stationary bench: `build_make/mtk3pico_smp1_usb_cdc_ir_bench.uf2`, 350 blocks / 179200 bytes.
SHA-256: `4f75dedd8ab13fc8e476f88418011cc5cbbaba5db5522ddb975f8a1685aaa698`.

Machine-readable evidence: `build/host-tests/s3-motion-verification.json`.
The records below describe earlier snapshots and hashes. See
[MOTION_INTEGRATION.md](../../app_program/subsystem3/MOTION_INTEGRATION.md)
for the explicit application control API and remaining user-interface/hardware work.

## Week 6 alignment acceptance, 2026-10-18

Checkout: `buddy3`, base commit `477230c` plus this uncommitted update. No flash,
commit or push performed. This extends the portable core; it does not implement
MQTT, the shared Vehicle Controller or motion integration.

| Check | Result | Evidence and limits |
|---|---|---|
| Host tests, warnings as errors | PASS | `tools/test-buddy3.ps1`; existing acquisition/calibration/line/steering/A-Z/PDF fixture checks plus `test_navigation.c` lifecycle suite |
| Lifecycle/configuration | PASS | Explicit synthetic per-sensor calibration, provisional/disabled/stale/missing-input start blocks, inactive-only atomic mapping/calibration updates |
| Rearming | PASS | Forward/reverse net clearance, insufficient displacement, oscillation, white/line persistence, configurable thresholds, malformed capture and epoch reset |
| Fault/recovery | PASS | Invalid sensor/ADC, stale channel/odometry, missing/invalid/non-finite distance, excessive gaps, epoch change; persistent stop, failed premature/stale reset, healthy interval, explicit restart |
| Line/mode/consumer | PASS | Initial and later line loss, loss during capture, junction, suppressed partial capture/no replay, unmapped/mapped letters, diagnostic scans without odometry |
| Timing traces | PASS | Synthetic 5 ms and 10 ms inputs, jitter, excessive gaps and wrapping clock; not measured physical scan speed |
| Independent A/Z fixture extraction | PASS | Four PDF page arrays and original source hash checked again |
| Isolated bench firmware | PASS | `build/host-tests/bench-alignment.log`; IR_BENCH=1, WIFI=none, TEST=0; no compiler warnings |
| Normal firmware | PASS | `build/host-tests/normal-alignment.log`; IR_BENCH=0, TEST=0, default CYW43 Wi-Fi; no compiler warnings |
| Strict ARM warning checks | PASS | Eleven S3/application C sources with Wall/Wextra/Werror syntax checks |
| Startup/symbol isolation | PASS | Bench app_main references only S3 init/start; S3 objects reference neither motion/motor APIs nor automatic `s3_navigation_start`; normal S3 hooks return zero without calls |
| Stationary UF2 integrity | PASS | 341 blocks, 174592 bytes; magic, RP2040 family, numbering and unique in-range addresses checked |
| New firmware on hardware | NOT RUN | No flashing, actual-track calibration, moving scan or braking test |
| Week 6 5 ms / concurrent timing | NOT VERIFIED | Bench target remains 10 ms; actual gaps and counts over 5 ms will be printed; shared kernel settings unchanged |

Current stationary UF2: `build_make/mtk3pico_smp1_usb_cdc_ir_bench.uf2`.
SHA-256: `33642b247b008914979da1465f21834e0674de189996766f30e4e81347bf20f3`.
Local machine-readable record: `build/host-tests/buddy3-alignment-verification.json`.
Symbol records: `bench-alignment-symbols.txt`, `normal-alignment-symbols.txt` in
the same ignored folder. Historical hashes below refer to earlier firmware.

See [Week 6 traceability](../../app_program/subsystem3/INTEGRATION.md#week-6-traceability-and-outstanding-team-decisions)
for preserved/changed/implemented/awaiting-integration items. Actual sensor roles,
5 ms scheduling, distance tuning, mapping delivery, fault-to-brake handling and
full-system priority/timing validation still require the team and hardware.

## Core implementation acceptance, 2026-10-16

Checkout: `buddy3`, HEAD `816d7f0` plus uncommitted working changes. Existing
Buddy 2 implementation and prior working changes were retained. No commit,
push, flash, motor command or Vehicle Controller integration was performed.

| Check | Result | Evidence and limits |
|---|---|---|
| Portable C suites | PASS | `powershell -ExecutionPolicy Bypass -File tools/test-buddy3.ps1`; C11, Wall/Wextra/Werror/pedantic; simulated inputs only |
| Independent A/Z fixtures | PASS | `tools/check-barcode-fixtures.py` reproduced all four original PDF bar-width arrays and verified source SHA-256 |
| Normal default Wi-Fi firmware | PASS | `IR_BENCH=0 TEST=0`, default SMP=1/USB/CYW43; local `build/host-tests/normal-build.log` |
| Isolated bench firmware | PASS | `IR_BENCH=1 WIFI=none TEST=0`; local `build/host-tests/bench-build-final.log` |
| ARM warning checks | PASS | All ten S3 C files plus app_main compiled with Wall/Wextra/Werror syntax checks; no warnings/errors in either build log |
| Startup isolation | PASS | Bench app_main object references only S3 init/start; S3 objects have no motion/motor dependency. Normal S3 init/start disassemble to return-zero only |
| UF2 integrity | PASS | 320 blocks, 163840 bytes; magic, RP2040 family, numbering and unique flash addresses checked |
| Hardware operation of this implementation | NOT RUN | No new UF2 flashed; no moving/three-sensor test conducted |

Bench UF2: `build_make/mtk3pico_smp1_usb_cdc_ir_bench.uf2`.
SHA-256: `bcc3c710940ab30b777b57a84264bf808a3cf6443d3955dd432b23974a60f3c4`.
Machine-readable local evidence: `build/host-tests/buddy3-verification.json`.
Build products/logs are ignored by Git; the source tests and fixture arrays are
part of the handoff. Local normal-build dependency setup initially lacked CYW43;
the SDK's pinned submodule `055d64274b014dd7b1c2fc94d26e8a18face7124` was installed
and the default build then passed.

The host tests cover named ADC channels, disabled inputs and ADC failures;
independent/reversed calibration and rejected candidates; hysteresis and fault
recovery; line direction, saturation, persistence and junction rearming; all A-Z
frames forward/backwards including sampled traces and counter wrap; supplied
A/Z fixtures, varied widths, malformed framing, quiet zones, incomplete frames,
duplicate suppression, overflow and excessive gaps; replaceable mappings,
unmapped results and manoeuvre suppression; and simulated navigation consumers
through follow/barcode/junction and loss/sensor-fault recovery. This demonstrates
software behaviour against those fixtures, not real navigation accuracy.

The overflow guard is exercised directly with an overfull producer state; excess
run counts and malformed extra elements are rejected by the batch/frame checks.
No unbounded run buffer is used. Barcode fixture tests explicitly add quiet zones;
the source PDF's left margin is shorter than ten narrow modules.

## Earlier diagnostic work (historical)

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-H01 | One sensor produces repeatable contrast | Grove 6 / GP26 analogue input | Manual USB bench diagnostic | Record five seconds each over white and black at fixed height; repeat three times | Valid counts and clearly separated, repeatable ranges |
| S3-H02 | Sensor behaves at intended mounting height | IR mounting/calibration | Repeat S3-H01 on the assembled car | Record height and ambient light with the readings | Contrast remains usable at the actual height |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-16 | S3-B01 | 816d7f0 + Buddy 3 working changes | Codex | PASS | Local `build/ir-bench-build.log` | ARM 14.3.Rel1, SDK 2.3.0: SMP=1 USB CDC, WIFI=none, TEST=0, IR_BENCH=1 compiled and linked; hardware not tested |
| 2026-10-16 | S3-B02 | Same working changes | Codex | PASS | Local `build/normal-build.log` | Same USB/no-WiFi profile with IR_BENCH=0 compiled and linked; startup object references all five subsystem hooks; subsystem 3 creates no task |
| 2026-10-16 | S3-B03 | Same working changes | Codex | PASS | ARM compiler and symbol inspection | Changed application C files pass -Wall -Wextra -Werror syntax check; bench startup object references only subsystem3 init/start, not motor startup or tests |
| 2026-10-16 | S3-B04 | Same working changes | Codex | PASS | UF2 binary validation | 290 blocks / 148480 bytes; correct magic, RP2040 family ID, block indices and unique in-range flash addresses |

The above are historical build/static checks, not measured sensor or motor performance.

## Hardware observations, 2026-10-16

- Before correcting the signal wire, COM3 captures showed white batch means
  32..38 and black 33..38 (black mean of 41 reports: 35.44). A replacement
  sensor gave white 34..39 (36.20 average) and black 32..38 (35.44 average).
  These were readings from the wrong/unconnected input, not usable calibration.
- The user's board photo confirmed Grove 6 yellow=GP27 and white=GP26. Earlier
  setup instructions had these reversed. Instructions now specify white to AO,
  yellow disconnected, red to VCC and black to GND; DO stays disconnected.
- After correcting the wire, the user reported approximately white=280 and
  black=3800. These were user-reported spot readings, not a logged calibration;
  the surface and height for that pair were not confirmed.
- At approximately 5 mm the user later reported white paper=150, black
  paper=280, black desk=1300. Actual track material is unavailable. Reference
  values 150/280 are used ONLY as development placeholders, not measured ranges
  or proof that the assessed track can be classified.
- S3-H01 is PARTIAL (response observed after wiring correction; fixed-height
  three-repeat ranges pending). S3-H02 is PENDING for actual track materials.
  The new classifier firmware has not been flashed or tested on hardware.

## Configurable classifier checks

- Portable C tests: PASS with `-std=c11 -Wall -Wextra -Werror -pedantic`.
  Covers unknown startup, exact switching boundaries, state retention in the
  hysteresis band, both polarities, saturation, invalid arguments/configuration,
  and reset/recovery. These are synthetic software checks.
- Updated `IR_BENCH=1 WIFI=none TEST=0` firmware: compiled and linked successfully.
  Build log: local ignored `build/ir-classifier-build.log`.
- Normal `IR_BENCH=0 WIFI=none TEST=0` build also passed, then the bench image
  was rebuilt. Log: `build/ir-classifier-normal-build.log`. No compiler warnings
  or errors were found in either log. The new UF2 has not been flashed.
