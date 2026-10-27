# Buddy 3 stationary diagnostics and software tests

25 October team integration: `S3_MOTION=1` now also starts Buddy 1 and mirrors
telemetry through a taskless facade, while preserving this runtime's single
motion/report owner. Mac/Linux: `make -C testing/subsystem3 test codec drive
feedback regression sizes`; `make sanitize` adds memory/undefined-behaviour
checks. [Current review](../../docs/INTEGRATION_2026-10-25.md) supersedes older
two-buddy-only and representation-size descriptions below.

The core software is implemented and host-tested. The new firmware has **not**
been flashed or validated on the car. Use the isolated bench UF2 for stationary
checks; the normal firmware retains Buddy 2's existing motion startup.

## Buddy 2 integration

The separate `S3_MOTION=1` profile connects steering, distance feedback,
manoeuvres, fault braking and a motion-owner watchdog. It boots disarmed.
See [operation and control API](../../app_program/subsystem3/MOTION_INTEGRATION.md).
Build with `tools/build-s3-motion.ps1`; never confuse its `_s3_motion.uf2` output
with the stationary `_ir_bench.uf2`. No motor-capable image was flashed here.
The same host test command also runs the real Buddy 2/Buddy 3 integration tests
and production motion-feedback tests with mocked kernel/hardware boundaries.

## Repeatable host tests

From the repository root in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test-buddy3.ps1
```

Requires MSYS2 UCRT64 GCC at `C:\msys64` (override with `-MsysRoot`). Compiles
portable C with `-std=c11 -Wall -Wextra -Werror -pedantic`, runs the existing
reflectance suite and the new acquisition/calibration/line/barcode/navigation
suite. It does not access the Pico. Generated executables are ignored under
`build/host-tests`.

Coverage includes per-channel identity/enabling/errors, independent calibration,
both polarities and hysteresis; centred/left/right traces, loss/reacquisition,
junction persistence/rearming, steering direction/limits/faults; all A-Z frames
forward and reversed through timestamped samples and run fixtures, timestamp
wrap, width variation, malformed/truncated frames, quiet zones, duplicate
suppression, timing failures and overflow; mappings, unmapped letters, manoeuvre
suppression, simulated calibration -> start -> follow -> barcode -> rearm,
initial/subsequent line loss and explicit fault reset/restart. `test_navigation.c`
adds provisional/disabled/stale/missing-input start gates, atomic inactive-only
mapping/calibration updates, net forward/reverse rearming and oscillation,
malformed capture recovery, odometry epoch changes, persistent stop requests,
no suppressed replay and synthetic 5 ms/10 ms traces with jitter. The existing
acquisition, calibration, line, steering, A-Z and independent A/Z tests remain.
These are synthetic tests, not measured physical detection performance.

`barcode_fixtures.h` contains independent bar widths from all four pages of the
supplied `Project/Barcode Sample.pdf` (A,A,Z,Z). Its source hash and extraction
method are recorded in the header. Tests add explicit quiet zones; they do not
claim the PDF's existing margins meet the decoder's requirement. The optional
fixture-check script below requires PyMuPDF and the original course PDF:

```powershell
python .\tools\check-barcode-fixtures.py 'A:\OneDrive - Singapore Institute Of Technology\SIT\Year 2 Tri 1\Embedded Systems Programming\Project\Barcode Sample.pdf'
```

## Build the stationary firmware

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build-ir-bench.ps1
```

Output: `build_make/mtk3pico_smp1_usb_cdc_ir_bench.uf2` (GP26 only).
For all three physically connected sensors:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build-ir-bench.ps1 -Sensors 3
```

Output: `build_make/mtk3pico_smp1_usb_cdc_ir_bench_3sensor.uf2`. It enables
barcode GP26, left GP27 and right GP28 and reports each channel separately.
Its startup still cannot initialise motion. The build scripts automatically run
`tools/check-buddy3-profile.ps1` against the compiled object symbols. A channel
being enabled does not prove its wiring works.
The script builds only; it never flashes. It uses MSYS2, local Arm GNU Toolchain
14.3.Rel1 in `build/tooling/arm`, and Pico SDK 2.3.0 with TinyUSB in
`build/tooling/pico-sdk`. These dependencies are ignored by Git and must be
installed separately in another checkout. With those tools on PATH:

```sh
make -C build_make PICO_SDK_PATH=../build/tooling/pico-sdk IR_BENCH=1 WIFI=none TEST=0 -j8
```

Normal regression build (also requires the SDK's CYW43 submodule):

```sh
make -C build_make PICO_SDK_PATH=../build/tooling/pico-sdk IR_BENCH=0 TEST=0 -j8
```

Make tracks profile changes to invalidate stale objects. The bench bypasses all
other application startup and rejects `TEST=1`. Only the bench starts the two
Buddy 3 tasks. The kernel USB console still runs. No motion request is issued.

## Wiring and enablement

Disconnect USB and battery before changing wiring. For initial stationary tests,
leave the battery disconnected and use USB power. Current offset/barcode sensor:

| Grove 6 cable | Sensor pin |
|---|---|
| White: GP26 / ADC0 | AO |
| Red: 3V3 | VCC |
| Black: GND | GND |
| Yellow: GP27 | Disconnected for the GP26-only test |

Leave sensor DO disconnected. This wiring follows the user's board photo:
earlier yellow-to-GP26 advice was wrong. Follow printed labels, not sensor
orientation. GP26 also appears at another Grove port; never attach two analogue
outputs to the same input.

`app_program/subsystem3/ir_bench_config.h` defaults to mask=1: barcode=GP26 enabled,
left=GP27 and right=GP28 disabled/unavailable. After both front AO signals are
connected correctly, build with `-Sensors 3`. **Check GP28's
battery-sensing jumper before connecting that input.** These pin reservations do
not claim that the physical three-sensor wiring has been verified. Default left
and right are the car's left/right when facing forward.

Each sensor has its own reference macros. White=150, black=280 are provisional,
based on approximate paper readings near 5 mm, not measured track calibration.
Do not substitute the black-desk reading for the assessed track. Keep height and
lighting fixed while comparing surfaces. The classifier normalises to darkness
0..1000 and supports either polarity; WHITE at <=400, BLACK at >=600, retaining
state between boundaries. Startup is UNKNOWN until a boundary is reached.

Explicit calibration collection and validation APIs are described in the
[handoff](../../app_program/subsystem3/INTEGRATION.md). They report count,
minimum, maximum and average, reject missing/overlapping ranges and preserve
previous references on failure. There is no automatic threshold adaptation.

## Manual flashing and reading, when ready

1. Keep battery disconnected, ensure the Pico pins are correctly aligned and
   undamaged, and use a USB data cable. If mounted, set the Robo Pico switch ON.
2. Hold the Pico's BOOTSEL button while connecting USB to the computer; release
   it when `RPI-RP2` appears.
3. Copy **the isolated bench UF2** to that drive. This replaces the Pico firmware;
   the drive disappears when it restarts.
4. Read output with the command below; add `-Port COMx` if several ports exist.
   Close any other programme using that serial port. Ctrl+C exits the reader.

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\read-ir.ps1
```

Illustrative output, not a captured hardware result:

```text
[S3 BENCH] enabled=1 cal=PROVISIONAL status=0 samples=20 gap=10000us max=10020us target=10ms
  run=INACTIVE readiness=0xf accepted_cal=0x0 odometry=0 rearming=0 stop_requested=0 faults=0x0
  readiness bits: 1=channels 2=calibration 4=sensors 8=odometry 16=fault; gaps>5ms=19 diagnostic_letters=0 actionable_steering=0
  barcode GP26 valid=1 raw=150/4095 dark=0/1000 WHITE white=150 black=280 adc_error=0
  left GP27 DISABLED/unavailable
  right GP28 DISABLED/unavailable
  line valid=0 present=0 error=0 junction_candidate=0 preview_valid=0 L=0 R=0 mm/s DIAGNOSTIC PREVIEW ONLY
```

Reports also include cumulative barcode/junction/lost/reacquired/fault event
counts, diagnostic-letter count, last decoded letter/direction/action, ADC errors, barcode rejects and
timing/overflow counts. Speed proposals are formatted in mm/s for the console;
the public API uses m/s. The bench stays inactive and has no odometry adapter,
calibration UI or automatic start: diagnostic letters/previews never become actions.
The printed accepted-calibration mask differs from numerical provisional references.
No motor moves are requested. A missing front sensor
pair correctly means no valid steering proposal in the default mode.

Sampling targets 10 ms on core 0 at priority 16; reporting runs separately at
priority 17 every 200 ms. Actual timestamps/gaps are used. Console or kernel
activity can delay sampling. Counts of measured gaps over 5 ms expose the Week 6
scheduling mismatch; they do not fix it. The kernel tick/priorities are unchanged. Frames reset on inadequate timing; this is not a
claim of guaranteed barcode operating speed. ADC errors invalidate samples, but
a disconnected floating wire can still look valid: software cannot certify wire
health from ADC counts alone.

## Physical acceptance still outstanding

At final height and lighting, collect repeated white/black ranges for each sensor
using actual track materials. Confirm steering sign from mounted left/right
channels; tune the line thresholds and gains. Check printed barcode quiet zones,
then measure decoding under actual movement and sampling jitter. Confirm real
junction behaviour and action mapping with the team. Vehicle Controller/Buddy 2
integration comes after these checks. See [test evidence](TEST_LOG.md).

## Week 6 scope

The [handoff traceability table](../../app_program/subsystem3/INTEGRATION.md#week-6-traceability-and-outstanding-team-decisions)
distinguishes preserved algorithms, changed sensor roles, behaviour implemented
within Buddy 3 and work awaiting integration. Outstanding decisions are sensor
roles, 5 ms scheduling, distance tuning, mapping delivery, fault-to-brake handling
and full-system priority/timing validation. None is certified by host tests.
