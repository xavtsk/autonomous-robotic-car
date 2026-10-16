# Buddy 3 stationary diagnostics and software tests

The core software is implemented and host-tested. The new firmware has **not**
been flashed or validated on the car. Use the isolated bench UF2 for stationary
checks; the normal firmware retains Buddy 2's existing motion startup.

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
suppression, simulated follow -> barcode -> junction and loss/fault recovery.
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

Output: `build_make/mtk3pico_smp1_usb_cdc_ir_bench.uf2`.
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

`app_program/subsystem3/ir_bench_config.h` keeps mask=1: barcode=GP26 enabled,
left=GP27 and right=GP28 disabled/unavailable. After both front AO signals are
connected correctly, explicitly change the mask to 7 and rebuild. **Check GP28's
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
  barcode GP26 valid=1 raw=150/4095 dark=0/1000 WHITE white=150 black=280 adc_error=0
  left GP27 DISABLED/unavailable
  right GP28 DISABLED/unavailable
  line valid=0 present=0 error=0 junction_candidate=0 steer_valid=0 L=0 R=0 mm/s PROPOSAL ONLY
```

Reports also include cumulative barcode/junction/lost/reacquired/fault event
counts, last decoded letter/direction/action, ADC errors, barcode rejects and
timing/overflow counts. Speed proposals are formatted in mm/s for the console;
the public API uses m/s. No motor moves are requested. A missing front sensor
pair correctly means no valid steering proposal in the default mode.

Sampling targets 10 ms on core 0 at priority 16; reporting runs separately at
priority 17 every 200 ms. Actual timestamps/gaps are used. Console or kernel
activity can delay sampling. Frames reset on inadequate timing; this is not a
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
