# Buddy 4 software and integration tests

Portable Make workflow: `make -C testing/subsystem4 test sanitize`. Runs processing/
packing tests, production controller runtime tests and production task/transport
tests with fake hardware. ASan/UBSan are enabled by the sanitize target.

The Buddy 5 suite additionally runs the real S4 arbitration alongside the S5
bypass and Buddy 2 controller, including impact-triggered cancellation.


From PowerShell in the repository root:

```powershell
.\tools\test-buddy4.ps1
```

No car or IMU is needed. Uses the installed MSYS2 UCRT64 GCC at
`C:\msys64\ucrt64\bin\gcc.exe`; override with `-MsysRoot` if necessary.
The script compiles the actual six portable Buddy 4 C modules with mocked I2C
registers, then executes assertions in `host_test.c`. It does not flash firmware,
run motors or change security settings. Assertions remain enabled.

Results appear on screen and in `build/host-tests/buddy4/latest.log`.
Successful execution ends with `Execution PASS: all assertions completed.`
Compilation PASS alone is not an executed test pass. `-CompileOnly` allows
compilation checks where execution is unavailable. A failed assertion identifies
the source line in the output; keep the complete log for debugging.

Coverage: sensor identity/configuration, axis mapping, signed data, bus failures,
saturation, magnetometer order/scaling, tilt, heading wrap, calibration, stale
samples, synthetic hump integration, packed fields and new-run reset.

The Make workflow validates task/transport and controller logic with fake RTOS,
I2C and telemetry boundaries. It does not measure physical I2C timing, calibration,
height accuracy, impact sensitivity, RTOS latency or live Wi-Fi delivery.

If Windows Application Control blocks the executable in your terminal, retain
that error and use a development environment where locally compiled C programs
are permitted, or ask the device administrator for the supported approval path.
Do not disable Application Control to run the tests.

## Physical handoff still required

- Verify GP2/GP3 I2C1 wiring and mounted axes; keep the car level/still through
  startup calibration. Confirm START is rejected before calibration completes.
- Run known-height humps, repeat runs and verify count and highest peak reset.
- Check stationary, accelerating, turning, climbing and descending classifications.
- Calibrate the magnetometer and test motor interference before claiming turn-rate accuracy.
- Test IMU disconnect/staleness, impact false positives, STOP during start/reset and
  fault RESET. Confirm an IMU fault also brakes during Buddy 5 avoidance.
- Verify MQTT hump payload, preservation of simultaneous fault bits and source
  timing under full network/motor load. Retain logs/video for Week 13 and the
  terrain analysis report. No physical pass is claimed by these software tests.
