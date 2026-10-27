# One-shot Buddy 2 movement check

This is a separate `MOTION_SMOKE=1` firmware, not an autonomous navigation
program. Normal builds default to zero. Only Buddy 2 is started; no Vehicle
Controller, IR processing, Wi-Fi, MQTT, IMU or obstacle task is started.
The test calls the existing motion API; PID, calibration, motor polarity,
encoder decoding, thresholds and motion state logic are not changed.

## Limits and physical preparation

- The request is forward 0.10 m at 0.05 m/s, with a 3 s software time limit.
  The 25 October integration imports the teammate's measured geometry,
  counts/revolution, PID and PWM-reset fix. They have NOT been revalidated on
  this assembled car. This is not a guarantee of 10 cm or low physical speed.
- FIRST test with both wheels raised on a stable support. Check that both
  rotate in the physical forward direction and stop. If they disagree, stop
  and investigate; do not put the car on the floor.
- For a later floor attempt, use a clear floor area (at least 1 m ahead),
  never a tabletop. Keep leads free and avoid pulling the laptop cable.
- GP20 is the START button; GP21 is the software STOP button. Do not use
  M1A/M1B/M2A/M2B hardware motor-test buttons during this firmware.
- Keep the board's physical power switch reachable. GP21 and the timeout
  require the software, scheduler and motion task to run; they are NOT an
  independent emergency shutdown or hardware watchdog.
- Keep the team's known-working power arrangement; do not rewire powered
  hardware. Turn all power off before repositioning or changing wiring.

## Build and run

```
make -C build_make CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 TEST=0 IR_BENCH=0 S3_MOTION=0 DIAGNOSTICS=0 MOTION_SMOKE=1 -j8
```

Image: `build_make/mtk3pico_smp1_usb_cdc_motion_smoke.uf2`.
It waits at boot; no motion starts merely from flashing or opening USB.
Enter BOOTSEL while powering up, copy this exact image, then open the USB
serial console at `/dev/cu.usbmodemMTK3PICO1` (verify the actual port).

Release BOTH programmable buttons. After `[SMOKE] READY`, press GP20 once.
There is a 3 s countdown during which GP21 cancels. During the move, GP21,
the kernel-monotonic 3 s deadline, Buddy 2 completion/fault or an unexpected
state ends the attempt through `stop()`. Completion consumes the original
one-shot Buddy 2 result. No output is printed while the move is active.
Each boot permits at most one attempt; there is no automatic retry/reset.

Record the full `[SMOKE]` output, actual direction and whether both wheels
stop. An encoder completion is not a ruler-based distance measurement.
Boundary/timing issues already seen in diagnostics remain open.

## Software verification

```
make -C testing/diagnostics -f firmware_test.mk smoke
make -C testing/diagnostics -f firmware_test.mk smoke CFLAGS='-std=c11 -Wall -Wextra -Werror -pedantic -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer'
```

The real runner is tested against fake buttons, clock and motion calls for
explicit/debounced start, held-at-boot/no press, simultaneous stop priority,
countdown cancellation, running stop, timeout, completion, fault, request
rejection, clock wrap/failure and delay/GPIO/priority failure. These do not
prove hardware stop latency, actual button wiring, calibration or distance.
No image was flashed by the preparation tools.
