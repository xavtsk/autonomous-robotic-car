# Buddy 4 integration — 28 October 2026

Buddy 4 is now connected to the existing single controller alongside Buddies 1–3
and 5 on `xavier/development`, based on main `19d02fe`. The implementation and
test records are included with this document. No firmware was flashed, and no
physical test was performed.

## Requirement basis and policy decisions

The Week 6 Design Review defines FR5 (detect humps and record/report the highest
peak per run), FR8 (operational telemetry), FR10 (safe stop for critical faults),
and independent IMU processing without direct motor ownership. The supplied
`AAI07_PostTri1.docx` retains these responsibilities, calls for agreed event/fault
interfaces, minimum-width data and synchronised fault-mask updates, and leaves
thresholds and accuracy to validation. Source files are in
`/Users/xavtsk/Documents/SIT Y2T1/Embedded Systems/`.

The 150 ms freshness limit follows the existing S4 snapshot policy. The 500 ms
run-reset timeout and stop-on-impact/event-loss behaviour are conservative
integration decisions, not claimed Tribunal-approved numeric requirements.
The existing impact detector (>2g or <0.3g) remains a candidate detector; real
false positives, bumps, acceleration and motor interference need testing.
No automatic speed change on slope or compass-based steering has been introduced.

## Integrated behaviour

- The existing S3 sampling/controller task is the only S4 event consumer and
  motor/report owner. S4 still owns only its IMU and protected data.
- START requires calibrated, fresh, nonfaulted IMU data. Once other START checks
  pass, S4 receives a run generation. Sampling continues while motion is held
  stopped until the matching post-reset sample arrives. The controller also
  waits for fresh stopped motion feedback. No movement occurs merely because
  the reset request was accepted. STOP takes priority; timeout faults the run.
- Reset clears hump count/peak/history and retains calibration. A cumulative
  16-bit count and peak are read atomically with the run generation. The count
  saturates rather than wrapping. Snapshot reads are unavailable during reset.
- Stale/faulted data, lost calibration, unexpected generation, event overflow
  or impact candidates latch `S3_DRIVE_TERRAIN_FAULT` (reason 10). The normal
  controller halt path brakes Buddy 2 and interrupts Buddy 5 scanning/avoidance.
  Healthy data alone cannot restart the car: explicit RESET and START are needed.
- Buddy 1 receives completed-hump count and maximum valid height through the
  unchanged binary v2 hump packet. The highest completed peak remains a run
  statistic during transient read faults. Before a valid hump or after reset,
  hump telemetry is marked unavailable. Only the protected IMU fault bit is
  updated; simultaneous Wi-Fi, motor and ultrasonic faults are retained.
- USB `[S4]` reports show terrain state, validity, tilt, turn rate, count/peak
  and run ID. Turn rate remains unavailable until measured magnetic calibration
  is supplied. It is not sent as absolute heading: the existing heading topic
  remains invalid because S4 does not export that measurement.

## Representation and resources

The internal S4 packet now uses 11-bit signed pitch/roll, 10-bit signed turn rate,
9-bit height, 3-bit terrain state, 5 validity bits, 32-bit timestamp and two event
bits. Total: 83 meaningful bits in 11 bytes, with five zero padding bits. This
saves four meaningful bits compared with the earlier format, not another whole
byte. Run/count metadata are two protected 16-bit values. The old packet layout
must not be mixed with this revision; MQTT v2 itself is unchanged.

The S4 task remains priority 18, core 0, 4096-byte stack, with its existing mutex
and eight-event queue. The controller adapter adds no task, mutex or motor owner.
The controller reason still fits its existing four-bit field. Runtime stack
headroom and worst-case timing remain unmeasured.

## Software validation

- `make -C testing/subsystem4 test sanitize`: portable processing and signed
  packing across all tenths-of-degree angle values; production S4 task/transport
  reset acknowledgement, event overflow/impact delivery, saturation and wrap;
  production controller START/STOP/reset, stale/event faults and hump telemetry.
- `make -C testing/subsystem5 test sanitize`: existing avoidance suite plus a
  combined S4/S5/Buddy 2 test that injects an impact during an active detour,
  verifies interruption and checks motor braking. Eight executables total.
- `make -C testing/integration -f controller_test.mk test sanitize`: existing
  S4-disabled controller, calibration, mapping, barcode capture and course tests.
- Integrated offline and Wi-Fi/MQTT builds with all application subsystems enabled;
  the network build uses public example credentials for compile checking only.
  Final selected local profile remains `_s5_UNCONFIGURED` (hardware gate zero).

See `testing/subsystem4/logs/2026-10-28_*` for logs and exact build sizes/hashes.
Software checks are not proof of real sensor or robot behaviour.

## Physical handoff

Keep the board level/still through startup calibration; normal S4 calibration
also requires valid calibrated Buddy 2 feedback with braked wheels. Wiring remains
GY-511/LSM303DLHC on GP2 SDA / GP3 SCL, I2C1, with the documented mounted axes.
Test known hump heights, repeated run resets, terrain/impact classification,
magnetic calibration/interference, disconnection and faults during ordinary motion
and Buddy 5 avoidance. Measure timing and verify live MQTT with simultaneous faults.
Buddy 5 hardware/geometry activation is still separately gated.

The terrain analysis report, physical accuracy and full-load evidence remain
outstanding. Tribunal 1 changes MUST be implemented and demonstrated in Week 13;
this document records implementation/software evidence, not completion of that demo.
