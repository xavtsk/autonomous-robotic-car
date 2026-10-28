# Subsystem 4 â€” test log

**Buddy 4:** IMU-based motion and terrain monitoring. Owner: TBD.

See `testing/README.md`. Software test cases are implemented; execution remains
pending. Compilation results are recorded separately from runtime tests.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|

## 2026-10-27 implementation checks

- ARM module compilation: successful, not a linked firmware or hardware result.
- Host test compilation: successful; execution blocked by Windows Application Control.
- Full Make build: blocked at MSYS startup, access denied.
- S4-U01: known +30 degree nose-up pitch; on-target execution pending.
- S4-U02: negative packed pitch round-trip; on-target execution pending.
- S4-U03: reject 600 mm height outside 500 mm range; execution pending.
- Host suite additionally covers identity, I2C failures, signed axes, saturation,
  mag byte order, calibration, sample gaps, synthetic hump integration and reset.
- Hardware: sensor ID, axes, disconnect/reconnect, known-height accuracy,
  magnetic calibration/interference, impact false positives, timing and stack pending.

## 28 October 2026 — controller integration

Implemented acknowledged per-run reset, cumulative hump count/peak telemetry,
IMU-only fault updates, bounded event consumption and latched safety response in
the existing controller. Internal packet fields now use minimum widths for the
declared ranges (83 bits in 11 bytes). No extra task or mutex was added.

Processing, production task/transport and controller-runtime tests pass normally
and with ASan/UBSan. Combined S4/S5/motion tests and existing controller regression
also pass. See `logs/2026-10-28_*` and
`docs/BUDDY4_INTEGRATION_2026-10-28.md` for results and physical work still required.
No IMU/car test, flash or Week 13 demonstration was performed in this session.
