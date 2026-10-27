# Buddy 4 simulated tests

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

These tests do not validate the RTOS task, physical I2C timing, on-car calibration,
real height accuracy, impact sensitivity, or controller/WiFi integration.

If Windows Application Control blocks the executable in your terminal, retain
that error and use a development environment where locally compiled C programs
are permitted, or ask the device administrator for the supported approval path.
Do not disable Application Control to run the tests.
