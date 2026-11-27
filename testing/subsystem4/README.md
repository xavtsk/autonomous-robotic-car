# Buddy 4 software and integration tests

> Current guide: [documentation index](../../docs/README.md), [baseline status](../../docs/CURRENT_STATUS.md), and [build/test commands](../../docs/BUILD_AND_TEST.md). Dated results below retain their original evidence limits.

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
registers, then executes assertions in `imu_host_main.c`. It does not flash firmware,
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

The controller integration test also injects rejected hump reports (`E_TMOUT`):
unchanged values must retry, newer cumulative count/height replaces the pending
value, and `E_OK` stops duplicate retries. It checks retry after operator STOP
and excludes a previous run's pending values during the next START/reset.
The fake reports distinguish enqueue attempts from accepted reports; these
checks do not establish MQTT delivery to the dashboard.

If Windows Application Control blocks the executable in your terminal, retain
that error and use a development environment where locally compiled C programs
are permitted, or ask the device administrator for the supported approval path.
Do not disable Application Control to run the tests.

## Physical measurement scope

The host tests use a configured 116 mm wheel spacing. The right-wheel crossing
case models one wheel passing over a 10 mm obstacle with the other on level
ground and checks that one crossing produces one reported hump. The model
does not measure the mounted sensor's height error, magnetic interference,
physical impact sensitivity, or task latency under combined motor and network
load. See the [dated test log](TEST_LOG.md) for the checks performed.

The hardware procedure specified GP2/GP3 I2C wiring, mounted axes and level
startup calibration. Its cases included repeated known-height crossings,
START/reset clearing the previous peak, left-wheel-only bumps, stationary
tilt, motor interference, stale readings and braking on an IMU fault during
avoidance. These cases describe the intended measurement conditions.
