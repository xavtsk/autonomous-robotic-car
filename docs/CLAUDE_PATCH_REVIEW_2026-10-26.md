# Review of the proposed calibration/buttons patch — 26 October 2026

## Later update — version 2 approved and applied

The findings below concern version 1 and are retained as historical evidence.
Version 2 fixes the held-STOP, map-freeze ownership and fault/recalibration
problems. Its actual-runtime regression tests and sanitizers pass, alongside
independent operator probes. It is applied to the integration working tree;
the offline course image builds without warnings. No flash, commit or push was
performed. See [the current course quickstart](COURSE_QUICKSTART_2026-10-26.md).

## Verdict

**Request changes before applying or flashing.** The calibration initializer
correction is sound, but independent host tests reproduce a STOP/start safety
issue, a mapping-freeze regression and an operator recovery dead-end.

Reviewed material: the complete diff pasted by the user, labelled
`calibration_fix_and_buttons.patch`. The proposed application source changes
were NOT applied to the actual integration branch. Temporary reconstructions
and compiled probes were used; this Markdown report is the only new repo file
created by this review. No firmware was flashed or physical motor test run.

## P1 — A held STOP does not inhibit START

`operator_panel.c` passes `stop_edge` to `operator_step`, not the held GP21
condition. After the first edge is handled, GP21 remains down but no longer
produces a stop input. A later GP20 release starts a new countdown and submits
START even though GP21 is still held.

Reproduced using the exact pasted `operator_logic.c/.h`, its button decoder and
a fake controller port that reports normal calibration/command outcomes:

```text
Six calibrations -> READY -> press and hold GP21 -> press/release GP20
Starting in 3... 2... 1...
RUNNING. GP21 = STOP.
START=1 STOP=1 physical_stop_down=1 state=FOLLOW
```

Second reproduced case: hold GP21 continuously from power-on. GP21 never arms,
while GP20 independently arms. Six calibration steps and START remain possible:
`START=1 STOP=0 stop_press_edges=0 physical_stop_down=1 state=FOLLOW`.

Required correction:

- A held STOP must inhibit starting/countdown regardless of whether an edge
  was previously observed or the button was held during power-on.
- Require both buttons released stably, then a fresh GP20 press after STOP.
- Do not merely pass a held level and print/send STOP every10ms indefinitely;
  retain bounded output and a safe latch/retry/acknowledgement policy.
- Add held-STOP, held-at-boot STOP and simultaneous/release-order tests through
  the real production runtime, not only a fake command sink.

## P2 — Failed START can reopen a previously frozen run map

New `buddy1_configuration_reopen()` unconditionally clears the freeze latch.
Neither `prepare_start()` nor `cancel_start()` records whether the rejected
attempt newly acquired the latch or inherited it from an earlier accepted run.

Confirmed with actual map storage/codec, a temporary facade containing the
pasted changes, real `s3_drive` and real navigation lifecycle/mapping gates:

1. Successful START -> STOP -> restart refused because wheels are unbraked.
   Cancellation clears the ORIGINAL freeze, allowing a new mapping update.
2. Duplicate START during FOLLOW. Readiness passes, but installing the map is
   correctly refused by active navigation. The patched error path reopens
   Buddy1 configuration while navigation stays active with its old private map.
   A new Buddy1 map is accepted, so reported actions can disagree with navigation.

Required correction:

- Reject ineligible START states before freezing or installing any map.
- Record freeze ownership atomically (`newly_closed` or equivalent token).
- Roll back only a latch newly acquired by that failed first-start transaction.
  Preserve the documented freeze after an earlier successful run, including STOP.
- Test initial rejected START, successful retry, rejected later restart,
  duplicate START during FOLLOW and map-install failure. Exercise real map
  helpers together; existing supplied tests stub away their interaction.

## P2 — Recalibration in FAULT leaves no operator recovery path

`ready()` advertises long-GP20 recalibration in FAULT and enters CAL_PROMPT.
The production runtime rejects WHITE/BLACK while FAULT. Calibration phases
have no RESET route; GP21 does not leave CAL_PROMPT. The user gets repeated
"Sampling refused (state=FAULT)" and cannot reset through the offered UI.

Reproduced: FAULT -> long GP20 -> four refused collection attempts -> GP21 ->
another GP20; `RESET=0`, still CAL_PROMPT and FAULT.

Required correction: either refuse long-press recalibration while faulted and
stay at the reset prompt, or perform an explicit acknowledged safe RESET before
entering calibration. A reset refusal must not trap the operator.

## What is sound / what was not established

- The selected calibration range minimum4095 fixes the zero-minimum bug and
  preserves the other surface's data. Keep overlap/too-few-samples rejection.
- Requests still route through the single existing controller; no direct motor
  writes or PID changes are added in the pasted source.
- The existing USB producer uses a bounded ring and drops output when full:
  it does not wait for the laptop. However `tm_printf` masks local interrupts
  during formatting in this SMP build, and USB service priority3 outranks motion.
  Host logic tests do not prove actual unplugged STOP latency or network timing.
- Reducing status output from200ms to1000ms is reasonable; it is not timing
  validation. The new operator task's priority19/2048-byte stack and kernel
  object use should also be documented/budgeted; priority19 overlaps the future
  Buddy4 band in TEAM.md. New bounded operator fields need minimum-bit review.
- The claimed eight firmware builds and the original sandbox test outputs were
  not independently reproduced as a complete matrix in this review. Passing
  those builds would not invalidate the reproduced behavioural defects.

## Y/Z course mapping is separate

This patch's "map fix" is failed-start freeze rollback, NOT support for multiple
letters per action. Buddy1's current wire/API still has one distinct letter for
each of forward/left/right/U-turn. If the selected course lap needs BOTH Y andZ
to mean RIGHT, sending a WiFi map alone cannot express that. Agree the required
route and letter meanings, then address the interface explicitly. Do not claim
the course is ready merely because Y/Z can be included in some four-letter map.

## Reproduction artifacts (temporary, may later be removed)

Exact pasted operator logic plus independent button probe:

```text
/private/tmp/operator-review.kdLPea/operator_logic.c
/private/tmp/operator-review.kdLPea/operator_logic.h
/private/tmp/operator-review.kdLPea/probe.c
/private/tmp/operator-review.kdLPea/probe
```

Compiled with strict C11 warnings and AddressSanitizer/UndefinedBehaviorSanitizer.
No sanitizer error; the assertion-backed reproductions demonstrate logic defects.
The controller port is fake; production source confirms FAULT rejection.

Map rollback reproduction:

```text
/private/tmp/map-rollback-review.6j6cFn/repro.c
/private/tmp/map-rollback-review.6j6cFn/vehicle_controller_patched.c
/private/tmp/map-rollback-review.6j6cFn/repro
```

Uses actual map codec/storage, facade, drive and navigation code. RTOS mutex/
message operations, telemetry and motor boundaries are fake; readiness is seeded
to reach the tested START states. No physical ADC, stop-time or movement claim.
