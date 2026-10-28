# Buddy 5 implementation — 27 October 2026

Implemented on local `xavier/development`, based on `6225b04` (Buddies 1–4).
At the end of that implementation session, changes were uncommitted and had not
been pushed. They are now included with the Buddy 4/5 integration commit. No robot
was flashed or moved.

Update, 28 October: `xavier/development` now includes `origin/main` at `19d02fe`.
The Buddy 5 changes were reapplied and the two shared-controller conflicts were
resolved while retaining main's barcode diagnostics, calibration/course settings
and turn-recovery search. See [merge verification](BUDDY5_MAIN_MERGE_2026-10-28.md).

Buddy 4 was subsequently integrated on 28 October; see
[Buddy 4 integration](BUDDY4_INTEGRATION_2026-10-28.md). Statements below describe
the original Buddy 5 implementation session.

## Result

Buddy 5's four code deliverables now have implementations: bounded coarse/fine
scanning, obstacle profiling, avoidance planning and line recovery. The existing
controller authorises the proposed moves through Buddy 2, waits for matching
stop/completion reports, and resumes Buddy 3 after recovery. There is no second
motor requester/report consumer. Buddy 1 receives the existing binary obstacle
payload, avoidance state and protected no-echo fault updates.

User-confirmed wiring: HC-SR04 ECHO GP0, TRIG GP1, servo GP12. PIO0 SM2/words 24–31
capture echo pulses; encoder SM0/SM1/words 0–23 are preserved. PWM6A drives the
servo independently of motor slices 4/5. USB console is required; a build guard
rejects the GP0/GP1 UART conflict.

## Activation and physical limits

`S5=0` remains default. `S5=1 S3_MOTION=1` compiles the integration; hardware
activation requires the separate `S5_HARDWARE_CONFIRMED=1` setting. The final
selected local image is `_s5_UNCONFIGURED` with that flag zero: it refuses
application startup/arming. Enabled-path images were compiled in `/private/tmp`
for verification only, with the network image using public placeholder settings.
No compiled image has been deployed.

Before activation, validate ECHO level protection, servo supply/pulse limits/
orientation, robot footprint, sensor-to-axle offset, clearance/stopping margins
and maximum obstacle depth. The initial settings are provisional. A front scan
cannot measure a hidden rear edge. The implemented bypass assumes a bounded
obstacle depth and a recoverable original corridor. It stops when neither side
has verified clearance instead of reversing into unobserved space.

Recovery needs sustained actual tape near the predicted corridor, not just both
IR sensors seeing white. A bounded angular search returns to the original heading
before handoff. Nearby lines, curved paths, slip and barcode/junction placement
still require physical verification; the code does not claim map-based identity.

## Shared integration changes

- Extended the existing `s3_drive` owner under `APP_S5`; normal S5-disabled behaviour
  remains covered by the original controller/course tests.
- Added sensing lifecycle, readiness, cancellation, compact status and telemetry.
- Started optional Buddy 4 and Buddy 5 tasks in the integrated profile after all
  objects are created. Buddy 4 event-to-controller/telemetry integration is still
  separate work; its subsystem source was not changed.
- Enabled linker section garbage collection for offline builds as already used
  in network builds. This fixes otherwise-unreachable newlib stdio/syscall
  references pulled in by the imported Buddy 4 libm code. No syscall shim, heap
  implementation or newlib errno replacement was added.
- Recorded GPIO/PWM/PIO ownership, task/object allocations and interfaces in TEAM.md.

Buddy 1–4 subsystem source files are unchanged. Existing network priority and
Buddy 1/Buddy 2 speed-range issues remain outside this change.

## Verification completed

All six Buddy 5 host executables pass normally and with AddressSanitizer/UBSan:
codec/core, real motion-controller integration, production hardware drivers with
fake registers, production sensing task with fake RTOS/hardware, disabled
activation gate, and the production telemetry adapter. Tests include invalid
and stale ranges during straight moves AND turns, scan refusal without stopped
feedback, bounded retries/timeouts, blocked routes, STOP/cancel, report loss,
line confirmation, invalid packed data and preservation of other fault bits.

The existing Buddy 1, Buddy 2/3 feedback/navigation, integration/course simulation,
Buddy 4 and diagnostic/startup regression checks pass. All three integrated builds
pass with zero compiler warnings: offline enabled-path, Wi-Fi enabled-path and
offline unconfigured. PIO words match the official assembler after relocation.

See [test logs](../testing/subsystem5/logs/2026-10-27_host.log) and
[build summary/hashes](../testing/subsystem5/logs/2026-10-27_build-summary.txt).
Full local compiler logs are retained under ignored `build/buddy5/validation`.
These are software/compile results, not measurements of scheduling, stack usage,
servo angles, ultrasonic accuracy or physical robot motion.

## Handoff

- [Build/configuration and operating assumptions](../app_program/subsystem5/README.md)
- [Bit widths, packet layout and synchronization](../app_program/subsystem5/DATA_FORMAT.md)
- [Physical test checklist](../testing/subsystem5/README.md)
- [Implementation versus physical evidence log](../testing/subsystem5/TEST_LOG.md)

Tribunal 1 changes MUST still be implemented and demonstrated in Week 13.
Physical distance/clearance/scan timing, safe bypass, original-line recovery and
full-load fault/network behaviour remain NOT RUN in this implementation session.
