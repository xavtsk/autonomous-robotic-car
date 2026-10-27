# Buddies 1-3 controller integration

The `S3_MOTION=1` profile starts Buddy 1, Buddy 2 and the teammate's
`s3_integration`/`s3_drive` controller. It boots disarmed. Default normal
builds leave navigation disabled; the stationary IR and one-shot movement
profiles are separate and mutually exclusive.

```text
IR -> Buddy 3 navigation -> s3_drive -> Buddy 2 motion/PWM
                          ^       <- timestamped feedback + ID reports
Buddy 1 pre-run map ------|
                          -> vehicle_controller facade -> Buddy 1 binary v2
```

There is ONE sampling/navigation/motion-request/report-consumer owner.
`vehicle_controller.c` is a taskless facade, not the old priority-15 task.
It never consumes completion or writes a motor. Buddy 2 remains the sole
motor-output writer. Buddies 4/5 have not been integrated.

## Control and data ownership

- Use `s3_integration_submit()` for calibration, mapping, START, STOP and RESET.
  One pending request is allowed; STOP overrides it. Acceptance is not execution.
  Inspect `command_sequence` and `command_ok` in copied status.
- RESET completes asynchronously: one request resets Buddy 2, then waits up to
  500 ms for fresh post-reset stopped/braked/healthy feedback and navigation
  readiness. Only then does `command_sequence` advance with the result. Normal
  commands are blocked during this wait; STOP pre-empts it and never restarts.
- All three physical sensors need accepted white/black calibration. Fresh
  odometry, acknowledged stillness and healthy sensors are required for START.
  No provisional calibration bypass or automatic start has been added.
- Operator input (S3_MOTION only): `operator_panel.c` polls GP20/GP21 and
  `operator_logic.c` walks through calibration (LEFT, RIGHT, BARCODE; white
  then black, 1 s each), then GP20 = START after a 3 s countdown, hold GP20
  2 s = recalibrate, GP20 after a FAULT = RESET, GP21 = STOP at any time.
  It only calls `s3_integration_submit()`/`s3_integration_status()`, so the
  single owner is unchanged. USB text stays output-only. Buttons held at
  power-up are ignored. Software stop only; the power switch is the cutoff.
- The facade's `begin()`/`request_stop()` only queue requests. Its legacy
  external navigation submission returns E_NOSPT, preventing a second producer.
- Buddy 1 validates four distinct A-Z mapping letters. On START the owner
  atomically freezes/copies this map and installs its private 26-entry expansion.
  Default A=forward, B=left, C=right, D=U-turn. The map stays frozen until reboot.
- The drive policy remembers a barcode action until a junction, requests braking,
  waits for a newer stopped acknowledgement, then matches manoeuvre completion
  by command ID. Reacquisition proves a stable line, not its route identity.
- Shared command/status storage is explicitly packed into 11/146 bytes under
  the existing priority-inheritance mutex. Pack/unpack runs on private copies
  where possible; no ADC, motor, network or console call occurs under that lock.
- Buddy 1 v2 still supports +/-510 mm/s. New Buddy 2 status supports +/-1022
  mm/s. Out-of-range/nonfinite/busy/invalid motion telemetry is invalidated,
  never clipped or silently reinterpreted. Agree a versioned wider wire format
  before relying on high-speed telemetry; this does not change PID inputs.
- Shared telemetry fault ownership remains separate from controller safety:
  the facade updates only stall, encoder and IR bits. Unknown motion status
  cannot clear a previously known motor fault. Other modules' bits survive.
- Motion telemetry is sampled every 20 owner cycles (~5 Hz at 10 ms).
  Network publishing is best-effort through Buddy 1's bounded outbox and never
  the path that makes a safety/motion decision.

## 27 October robustness and timing update

Wi-Fi now has a 20 s initial join-to-IP deadline and capped 5/10/20/30 s retry
backoff. MQTT retains one fixed pending packet on temporary publish failure,
allows one application request in flight and paces sends (20 ms) and retries
(100 ms). Existing bounded application queues can still overflow; reconnect
retries can duplicate delivery. Binary MQTT protocol v2 is unchanged.

The 10 ms sampling period is unchanged. `above10ms` includes ordinary jitter,
not a count of missed deadlines. Separate RUN timing resets at accepted START,
includes the gap that caused a halt/fault, then freezes while inactive: start
timestamp, sample count, maximum gap, above-target count, count above the existing
30 ms safety threshold, and timestamp of its latest violation. Boot maximum is
reported separately. All six 32-bit diagnostic fields are explicitly packed
(metadata 509 bits/64 bytes); counters saturate. This instrumentation does not
fix or attribute long scheduling delays.

See [implementation and validation](../../docs/SOFTWARE_FIXES_2026-10-27.md).

See [operation/calibration API](../subsystem3/MOTION_INTEGRATION.md) and
[25 October review](../../docs/INTEGRATION_2026-10-25.md) for verification,
source revisions and remaining on-car checks.
