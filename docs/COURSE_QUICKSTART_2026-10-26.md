# First course trial — Buddies 1, 2 and 3

> Historical 26 October instructions below predate the evening wiring and
> built-in calibration changes. Current mapping (confirmed 27 October) is GP27
> left line, GP28 barcode, GP26 right line; built-in calibration must be rechecked after
> moving sensors. Follow the team's verified power/BOOTSEL arrangement, not an
> assumption that the switch can remain off. See the latest
> [27 October software update and retest checklist](SOFTWARE_FIXES_2026-10-27.md).
> The old firmware hash below identifies an older build, not the current image.

## Current result

Version 2 of `calibration_fix_and_buttons_v2.patch` is applied to the
`integration/buddies-1-3` working tree. The offline course firmware has built
successfully with no compiler warnings. Nothing was flashed, committed or pushed.
Existing unrelated changes are preserved.

Built image: `build_make/mtk3pico_smp1_usb_cdc_s3_motion.uf2` (241664 bytes).
SHA-256: `959f9e592dccc05a95b478da133b83e124b85e814a744e31002b118c1cb876e8`.
The reviewed temporary build and final repository build produced identical UF2
bytes. This identifies the tested build; it does not prove physical performance.

Verified before application: Buddy 1, Buddy 2, Buddy 3 and integration host suites,
including address/undefined-behaviour sanitizers. Integration tests were rerun
after application. Calibration completes through the actual integration runtime;
overlapping surfaces are still rejected. Held STOP blocks START, fault recovery
offers RESET, and a refused restart cannot unlock an earlier run's map.
Additional independent operator probes covered timestamp wrap and randomized
button sequences. These are simulated tests, not measurements on this car.

Buddy 2 and Buddy 3 subsystem C/header files are byte-for-byte unchanged by this
patch. The calibration correction and operator interface live in the controller;
Buddy 1's map-freeze helpers and associated tests also change. No PID, steering,
turn geometry, sampling period or navigation timeout was retuned.

**Today's first target: follow a short straight, then stop using GP21.**
This offline image is NOT a complete Y/Z barcode-course solution. It cannot
receive a new letter map over Wi-Fi. The combined controller's default map is
A = forward, B = left, C = right, D = U-turn; Y or Z will halt as unmapped.
Use a stretch before any barcode, crossing or bend for the first trial.

## 1. Flash the correct file

1. Keep the robot supported with both wheels clear of the surface. Disconnect
   the power bank and switch the car off before flashing. Do not change wiring.
2. Close any existing serial viewer: press **Control+A**, release both keys,
   then press lowercase **k**, then **y** if asked. This is Control, not Command.
3. Disconnect the laptop cable from the green Pico W's micro-USB socket.
4. Hold **BOOTSEL on the green Pico W**. While holding it, reconnect the laptop
   data cable to that same micro-USB socket. Release BOOTSEL when the Mac shows
   the `RPI-RP2` drive.
5. In Terminal, paste this one command:

```sh
cp -X "/Users/zhaobao/Documents/SIT/SIT-Y2-Tri1/Embedded System/Project/autonomous-robotic-car/build_make/mtk3pico_smp1_usb_cdc_s3_motion.uf2" "/Volumes/RPI-RP2/"
```

The drive normally disappears when flashing finishes and the Pico reboots.
Do not copy the old IR bench, motor smoke or any `_COMPILE_ONLY` image instead.
If the copy fails or the drive remains mounted, stop and share the exact output.

## 2. Open the instructions

Keep the Pico's USB data cable connected to the laptop. In Terminal:

```sh
ls /dev/cu.usbmodem*
```

If it lists `/dev/cu.usbmodemMTK3PICO1`, run:

```sh
screen /dev/cu.usbmodemMTK3PICO1 115200
```

If it lists a different device name, use that name instead. Open only one viewer
or logger on the port. Instructions beginning with `[OP]` repeat, so connecting
after boot does not lose the calibration prompt. Terminal text input does not
control the car; use the physical GP20/GP21 buttons.

## 3. Calibrate the three sensors on the actual mat

Use the team's already verified sensor/power arrangement and normal mounted
sensor height. Reconnect the normal car supply with the car stationary and
the power switch accessible. Calibration requests do not start the motors.
The two front sensors follow the line; the separate right-offset sensor reads
the barcode. Physical rightmost does not automatically mean GP28.

For each printed step, put **only the sensor named in that prompt** over the
specified surface, keep it still, briefly press and release **GP20**, then wait
for its one-second collection to finish before repositioning:

| Step | Named sensor | Surface |
|---|---|---|
| 1 | Front-left line, GP27 | Plain white mat |
| 2 | Front-left line, GP27 | Centre of black tape |
| 3 | Front-right line, GP28 | Plain white mat |
| 4 | Front-right line, GP28 | Centre of black tape |
| 5 | Right-offset barcode, GP26 | Plain white barcode background |
| 6 | Right-offset barcode, GP26 | Black barcode bar, or matching black material |

Wait for **All 3 sensors calibrated** and **READY**.
If a sensor is REJECTED, redo that sensor's white/black pair as prompted;
check positioning, height and actual contrast. Do not bypass calibration.
Do not press GP20 again until the next prompt is understood: after calibration,
a short GP20 press requests START.

Calibration is held in RAM. Rebooting or losing Pico power requires all six
steps again. If using standalone power, verify that it keeps the Pico running
when USB is removed; do not assume a lit sensor proves the Pico stayed alive.

## 4. Check the STOP interlock before driving

At READY, hold **GP21**, briefly press/release GP20, and keep GP21 held.
There must be no START/countdown or wheel motion. Release both buttons.
A fresh GP20 press is required to start afterwards.

GP21 is a software STOP, not a guaranteed emergency power cutoff. If it does
not respond as expected, use the physical power switch and do not continue.
Real STOP latency still needs measurement.

## 5. First short straight

1. Choose a clear, short straight **before** the next barcode, junction or bend.
   On the photographed left straight, face toward the top of the photo so the
   barcode strip is on the robot's right. The downward-facing GP26 sensor must
   actually pass over the strip for later barcode trials.
2. Put the front line sensors around the line at their intended mounted height.
   Both seeing only white means line lost; both fully black may mean junction.
   Do not compensate for poor mounting by changing thresholds blindly.
3. Clear hands/objects from wheels. Keep GP21 and the physical cutoff reachable.
   Use the team's verified standalone power for an untethered trial. If doing
   a very short USB-connected trial, the cable must remain slack and clear of
   the wheels; stop before it pulls the car. Do not attempt a tethered lap.
4. Briefly press/release GP20 once. Read the **3–2–1 countdown**, then RUNNING.
5. Let it follow only a short straight, then press **GP21 before the barcode
   or crossing**. Confirm both wheels stop. Do not let a successful start turn
   into an unplanned full-course attempt.
6. If it halts/faults first, save the exact `[OP] Run ended` line and nearby
   status output; record whether it stayed on the line and which wheels moved.
   Do not repeatedly restart without understanding the reported reason.

After STOP or a normal halt, a short GP20 press requests another run. Holding
GP20 for at least two seconds and releasing it at READY starts recalibration.
During FAULT, either press length requests RESET instead; fix the cause first.
Rejected RESET prints advice and does not automatically move the car.

## 6. Next stage: one barcode and one crossing

Only after the straight and physical STOP work:

- Build/flash the real Wi-Fi/MQTT course profile with the team's actual settings,
  and send an agreed valid map **before its first accepted START**. This guide
  deliberately does not flash a Wi-Fi image or publish commands.
- Configure the first real letter, for example Y = RIGHT, then test just its
  one crossing and deliberately stop before the next barcode. The map remains
  locked after a successful START until reboot, including after operator STOP.
- Measure whether the decoded action reaches the crossing within the unchanged
  five-second limit. Observe the pivot position and outgoing-line reacquisition;
  neither has been validated on this course by host tests.
- The one-distinct-letter-per-action interface cannot make Y and Z both RIGHT.
  Agree the intended course/demo mapping before claiming a full lap. There is
  also no automatic finish recogniser; deliberate operator STOP ends this trial.

Do not alter motor/PID logic, barcode timeout, mapping rules or turn geometry
just to force a first run. Record the result and choose the next change from
actual evidence.

## Rebuild and evidence

Rebuild this exact offline profile only if source changes:

```sh
cd "/Users/zhaobao/Documents/SIT/SIT-Y2-Tri1/Embedded System/Project/autonomous-robotic-car"
make -C build_make PICO_SDK_PATH=/Users/zhaobao/pico/pico-sdk CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 TEST=0 IR_BENCH=0 S3_MOTION=1 DIAGNOSTICS=0 MOTION_SMOKE=0 -j8
```

Record: firmware filename/hash, sensor white/black ranges and acceptance, starting
position/direction, whether both wheels tracked correctly, STOP result, and
exact halt/fault reason. A short video plus saved serial output is useful.
Keep physical line-following and barcode/junction tests open until actually run.
