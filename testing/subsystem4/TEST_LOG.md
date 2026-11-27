# Buddy 4 test record

These dated entries cover IMU-based motion and terrain monitoring. The host and controller results are reported separately from target execution.

## 27 November 2026 — submission verification

The PowerShell host test and all three executables from the Make test target
passed with GCC. The processing test checked tilt, synthetic hump heights and
packet encoding. The controller and transport tests exercised terrain reporting,
faults and reset handling with simulated inputs.

The [terrain analysis report](../../app_program/subsystem4/TERRAIN_ANALYSIS.md)
gives the test inputs, expected heights and limits of the physical evidence.

## 2026-10-27 implementation checks

- ARM module compilation: successful, not a linked firmware or hardware result.
- Host test compilation: successful; execution blocked by Windows Application Control.
- Full Make build: blocked at MSYS startup, access denied.
- S4-U01: known +30 degree nose-up pitch; no target run recorded.
- S4-U02: negative packed pitch round-trip; no target run recorded.
- S4-U03: reject 600 mm height outside 500 mm range; no run recorded for this session.
- Host suite additionally covers identity, I2C failures, signed axes, saturation,
  mag byte order, calibration, sample gaps, synthetic hump integration and reset.
- Hardware: sensor ID, axes, disconnect/reconnect, known-height accuracy,
  magnetic calibration/interference, impact false positives, timing and stack use were not measured in this session.

## 28 October 2026 — controller integration

Implemented acknowledged per-run reset, cumulative hump count/peak telemetry,
IMU-only fault updates, bounded event consumption and latched safety response in
the existing controller. Internal packet fields now use minimum widths for the
declared ranges (83 bits in 11 bytes). No extra task or mutex was added.

Processing, production task/transport and controller-runtime tests pass normally
and with ASan/UBSan. Combined S4/S5/motion tests and existing controller regression
also pass. See `logs/2026-10-28_*` and
`docs/BUDDY4_INTEGRATION_2026-10-28.md` for the measured results and limits of this session.
No IMU/car test, flash or Week 13 demonstration was performed in this session.
