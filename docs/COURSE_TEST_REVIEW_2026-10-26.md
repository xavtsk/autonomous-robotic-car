# Course-test review — 26 October 2026

## Later update — version 2 applied and built

The review below is retained as the earlier audit. Its missing operator input
and zero-minimum calibration blockers are now resolved by the reviewed version 2
patch. Host/sanitizer tests and the offline course build pass; no physical course
run is claimed. Follow [the current flashing/calibration/short-straight guide](COURSE_QUICKSTART_2026-10-26.md).
Offline Y/Z mapping, full-lap routing, turn geometry, finish policy and physical
timing/accuracy still require the staged work described there.

## Verdict

The current integration branch has the main Buddy 1/2/3 software components,
but **is not yet operator-ready for a full autonomous course run**. Do not
present flashing the combined image as sufficient to start driving.

Review target: `integration/buddies-1-3`, HEAD `71193e2`, with the existing
uncommitted implementation preserved. No application source, firmware, Git
history, robot or network settings were changed by this review. Temporary host
test binaries and image previews were created outside the tracked sources.

User update on this date: **both wheels have successfully moved and stopped
with the isolated motor smoke-test firmware**. This supersedes yesterday's
handoff uncertainty. Do not repeat that test unless new symptoms require it.
It is not yet evidence of accurate distance/turns or autonomous line following.

## Course and direction

Source photo: `/Users/zhaobao/Downloads/IMG_4863.HEIC`.
It shows a left-hand rectangular route connected through two crossings to a
central straight and two small right-hand loops. There are barcode strips
beside the left straight and west/left of the central straight. No assigned
start, finish or readable barcode letters were established from the photo.

The user permits any start provided barcodes are on the robot's right.
Recommended **first test position**: on the left straight below the left-hand
barcode, facing toward the top of the photo. The barcode paper is then on the
car's right. The right-offset barcode sensor must face downward and pass over
the bars, not merely point sideways toward them.

A possible later lap is left straight upward -> upper crossing RIGHT -> central
straight downward -> lower crossing RIGHT -> left/bottom curve -> starting
straight. Both photographed barcode strips are then on the car's right.
This is a proposal, not an implemented or verified route. Do not take the small
right loops initially: they can return to a crossing without a new barcode.

For that rectangular lap, both crossings need a right turn. **Identify the
actual letters first.** The current Buddy 1 map stores one distinct letter per
action and cannot map two different letters to RIGHT. If the papers contain
different letters, that lap requires a mapping/interface decision or a different
agreed test arrangement. Do not silently replace the course papers or widen
the protocol merely to make the proposed route work.

For the first moving trial use only a short straight; the next trial is one
barcode plus one crossing. A complete lap is not the first acceptance test.

## Sensor roles — physical right is not ADC2 automatically

| Function | Pin | Intended physical placement |
|---|---|---|
| Barcode | GP26 / ADC0 | Offset to the car's right, facing down over barcode strip |
| Left line input | GP27 / ADC1 | Front-left relative to the tracking line |
| Right line input | GP28 / ADC2 | Front-right relative to the tracking line |

These are logical functions, not left-to-right numbering of all three sensors.
Do not swap GP26 with GP28 simply because the barcode sensor is rightmost.
Verify actual connections and GP28 board jumper against the existing board/
sensor documentation with all power disconnected before changing wiring.
Use the team's verified supply arrangement; do not infer safe voltages from
the sensor's displayed black/white labels.

Both front sensors must see useful contrast around the tracking-line edges.
Both fully white means no line; both fully black is treated as a possible
junction and inhibits ordinary steering. Mounting/spacing must therefore be
validated on the actual line, not just by testing one sensor over black paper.

## Verified blockers

### 1. Operator controls are not connected in the combined firmware

The `S3_MOTION=1` profile boots DISARMED. APIs for calibration, START, STOP and
RESET exist, but no real UI calls the calibration API and no production input
calls the facade's start/stop functions. USB console input is discarded in
`lib/libtm/sysdepend/pico_rp2040/usb/usb_tinyusb_glue.c:62`.

GP20/GP21 operate only `MOTION_SMOKE=1`. They do not start/stop the full course
firmware. Do not tell the user to type enum names or invented terminal commands.
See `app_program/controller/README.md:25` and `vehicle_controller.c:55`.

### 2. Actual runtime calibration cannot currently be accepted

`app_program/controller/s3_integration.c:113,117` resets the selected white or
black calibration range to `{0}`. Its minimum therefore starts at zero.
`app_program/subsystem3/ir_calibration.c:22` only lowers the minimum, so with
unsigned ADC samples both minima remain zero. Acceptance at lines33–34 requires
non-overlapping white/black ranges, which is then impossible.

The proper collector initializer starts minima at4095, but this runtime path
does not use that initialization for the selected collection range. A fix must
initialise the selected range correctly **without discarding the other surface's
already collected data**, then add end-to-end acceptance/rejection/recollection
tests. Do not remove the contrast/non-overlap validation or bypass calibration.

This was reproduced using the actual runtime `execute()` and actual calibration
module, with 20 white samples150 and20 black samples280:

```text
Actual runtime: white range [0,150], black range [0,280]; APPLY rejected
Same samples with correct initialisation: accepted; source files unchanged
```

Temporary reproducer at review time:
`/private/tmp/course-review-20261026.xgt1z7/calibration_repro.c`.
It passed ASan/UBSan while demonstrating the rejected calibration.

The existing integration host tests still pass, but their runtime test collects
one white sample then stops. It does not complete white+black collection/apply
and a successful calibrated START. Earlier passing tests did not cover this
complete real-runtime workflow. This finding corrects that readiness assumption.

### 3. There is no explicit finish recogniser

The controller supports following, braking, manoeuvre, reacquisition, halt and
fault, but no successful end-of-course rule. Line loss halts; that is not proof
of a completed lap. A first lap can use deliberate operator STOP once a usable
local STOP input exists. Automatic finish requires an agreed marker/count/route
condition and validation, not merely an unused telemetry COMPLETE enum.

### 4. Course timing/mapping/turn geometry remain unvalidated

- A decoded action expires after **5 seconds** before its next junction. At
  nominal0.10m/s that is approximately0.50m of travel without delays. Measure the
  real post-decode distance/time, including curves, before changing this policy.
- Any junction with no pending action halts, even if an operator expects the
  car to go straight. Two barcodes before one junction also halt.
- Full barcode decoding completes after the trailing white margin, not the
  first bar. Finish reading before a junction starts suppressing navigation.
- Left/right are fixed90-degree pivots at the detected position. The controller
  does not advance to the crossing centre to compensate for sensors ahead of
  the axle. Validate one crossing; do not assume the new heading places sensors
  over the outgoing line.
- On manoeuvre completion it brakes and waits up to1second for valid line.
  There is no blind driving/search in reacquisition. A stable line is not proof
  of route identity if several lines are possible.

## What can be flashed/tested now: stationary three-sensor check

This stage checks wiring/contrast only and cannot move the robot. It is useful
while the combined-firmware blockers are being resolved. Existing smoke success
does not remove the need to check all three sensor roles.

### 1. Build the isolated sensor image

In Terminal:

```sh
cd "/Users/zhaobao/Documents/SIT/SIT-Y2-Tri1/Embedded System/Project/autonomous-robotic-car"
export PICO_SDK_PATH="/Users/zhaobao/pico/pico-sdk"
make -C build_make CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 TEST=0 IR_BENCH=1 IR_CHANNELS=3 S3_MOTION=0 DIAGNOSTICS=0 MOTION_SMOKE=0 -j8
```

Expected output:
`build_make/mtk3pico_smp1_usb_cdc_ir_bench_3sensor.uf2`.
Do not flash an old GP26-only image or the motor smoke image for this check.
Building changes generated files, not application logic. This review did not
rebuild that firmware; these are commands for the deliberate next test.

### 2. Prepare and enter BOOTSEL

- Keep the robot stationary, wheels supported and motor power switched off.
- Close an existing serial viewer first: Control+A, release, then lowercase k,
  then y if asked. Only one viewer/logger should open the serial port.
- Do not unplug/rewire sensors while powered. Use the team's verified power
  arrangement for logic/sensors; do not improvise a new power connection.
- Disconnect the green Pico W's micro-USB cable from the laptop, hold BOOTSEL
  on that green Pico W, reconnect the data cable to the green Pico's micro-USB
  socket, then release BOOTSEL when `RPI-RP2` appears.

### 3. Copy this exact UF2

Verify the bootloader mount and firmware exist before copying:

```sh
ls -ld "/Volumes/RPI-RP2"
ls -l "build_make/mtk3pico_smp1_usb_cdc_ir_bench_3sensor.uf2"
cp -X "build_make/mtk3pico_smp1_usb_cdc_ir_bench_3sensor.uf2" "/Volumes/RPI-RP2/"
```

The Pico normally reboots and the drive disappears. The UF2 replaces the
currently installed smoke firmware; copying does not preserve both programs.

### 4. Open the actual serial port

```sh
ls /dev/cu.usbmodem*
screen /dev/cu.usbmodemMTK3PICO1 115200
```

Use the actual port from the first command if it differs. If no port appears,
check normal boot/data cable/enumeration before retrying the old port name.

### 5. Check each sensor independently

Expected: `[S3 BENCH] enabled=7` and barcodeGP26, leftGP27 and rightGP28 all
available. The label `PROVISIONAL` is expected; this image does not perform
accepted operational calibration.

- Put real course white under only the right-offset barcode sensor, then
  black. Confirm GP26 raw changes. Record both readings at final sensor height.
- Repeat independently for front-leftGP27 and front-rightGP28.
- With the front pair in intended line-following position, record both raw/
  darkness values and how they change when the line shifts left/right.
- Preserve invalid/error data; a healthy-looking black label with placeholder
  thresholds is not calibration evidence. No source constant should be tuned
  from a single unexplained sample.
- Do not expect motor start buttons or autonomous navigation in this image.

## Full sequence after the blockers are fixed — not executable yet

1. **Operator interface + calibration fix/tests:** connect real controls to the
   existing single owner; GP20 START and GP21 STOP are reasonable local controls.
   Calibration can use a bounded USB interface while parked, with explicit
   prompts/acknowledgements. A UI is a new scoped implementation task, not
   currently implemented terminal syntax. Retain no-motion-on-boot/STOP priority.
2. **Build/flash the corrected course image:** initially offline combined
   `S3_MOTION=1`, not smoke/bench/compile-only. Do not disable readiness gates.
3. **Calibrate while disarmed:** for BARCODE, LEFT, RIGHT collect one second of
   actual white then black at final mounting height, APPLY and check success.
   All three acceptance bits must be present; calibration currently resides in
   RAM and is lost if logic power is interrupted. Keep the MCU powered when
   removing the laptop data tether, or recalibrate after reboot.
4. **Agree mapping:** identify both printed letters and required turns. Current
   default is A=forward, B=left, C=right, D=U-turn. Map freezes at explicit START.
   Right-side sensor location does not reverse LEFT and RIGHT commands.
5. **Short straight-only test:** no junction/finish claim, low-speed baseline,
   accessible physical switch and working local STOP; confirm steering correction
   and safe stop on deliberate line loss. Do not drag the laptop cable behind
   the car. Preserve the current0.10m/s baseline unless measurement justifies
   changing it; no speed or PID setting was altered by this review.
6. **One barcode:** ensure offset aligns; verify its decoded letter exactly once.
   Current decoder expects Code39 star + one uppercase letter + star with white
   margins (at least10narrow modules each end), either scan direction. Narrow
   bars must take at least30ms and at least3×the largest measured sampling gap;
   at0.10m/s nominal3mm is only a mathematical floor, not a validated safe width.
7. **One crossing:** ensure the decoded action is held until the crossing,
   stop is acknowledged before turning, completion matches the command ID,
   and sensors reacquire the outgoing line. Check axle/sensor offset and actual
   post-decode travel time against the5second pending-command limit.
8. **One full agreed lap:** only after both crossing behaviours work and barcode
   mapping/route are compatible. Start below the left barcode facing up-photo,
   keep both strips on the right along the chosen route, and deliberately STOP
   on return to a marked finish location. No automatic lap counting exists.
9. **Buddy 1 network evidence:** after mechanics/navigation work, build the full
   network-enabled profile with validated real local configuration; confirm
   binary telemetry, snapshot, heartbeat and map before run, then measure timing
   under publishing/reconnection. Do not require live MQTT delivery for stopping.

Local ignored `config/wifi_secrets.h` now exists; its contents were not read and
its connection settings have not been validated. Do not assume networking is
ready from file existence, and do not expose secrets in logs. Existing
`_COMPILE_ONLY` images remain unsuitable for deployment. Buddy 1 telemetry v2
still marks wheel speeds above +/-510mm/s unavailable rather than clipping.

## Evidence and non-claims

Current host facade/codec/runtime tests were rerun successfully on26October;
log `/tmp/course-integration-review-20261026.log`. The additional calibration
reproducer demonstrates an uncovered functional failure despite those passes.
No flashing, robot movement, network connection, PID retuning, automatic
finish behaviour or full-course success was performed/claimed by this review.
Keep the successful raised-wheel test as user-confirmed evidence, separate from
the remaining sensing/calibration/navigation tests.
