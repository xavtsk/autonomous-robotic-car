# Subsystem 3 — test log

**Buddy 3:** IR line following and barcode decoding. Owner: TBD.

Buddy 3 core software is implemented and software-tested. Actual track and
moving-car validation remain pending. The current stationary UF2 has not been
flashed. The general target `TEST=1` hook reports SKIP; the host suite is separate.

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
