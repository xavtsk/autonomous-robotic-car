# Main + Buddy 5 integration — 28 October 2026

`xavier/development` was fast-forwarded from `6225b04` to fetched `origin/main`
`19d02fefd3ef614224cd8485920c4d5978ae5c92`. The local Buddy 5 implementation was
then reapplied. At that point it was uncommitted and nothing had been pushed or
flashed. The resolved changes are now included with the Buddy 4/5 integration commit.

The original tracked and untracked local changes are retained in Git stash
`338f82f22188f899b660fdd9f27cac470e8cab4c`, named
`Backup Buddy 5 before main integration 2026-10-28`. Do not apply it again onto
this already integrated working tree.

## Resolution

- `s3_drive.h`: retained both main's line-search state and Buddy 5 avoidance state.
- `s3_integration.c`: retained barcode-capture support and Buddy 5 API/status reporting.
- Reviewed the automatically merged `s3_drive.c`: Buddy 5 range checks still run
  before normal driving, turns and the new line-search branch; active avoidance
  stays under the existing controller's sole motion/report ownership.
- Kept main's calibration, speed, 135 mm line-sensor-to-axle setting, 40-second
  pending-barcode window, turn search, USB/MQTT capture and viewer updates.
  These line-sensor settings do not establish Buddy 5 ultrasonic/servo geometry.

## Verification

All passed normally and with AddressSanitizer/UBSan:

- Buddy 5 suite, now seven executables. Added `course_clear_test.c`, reusing the
  upstream course simulator with S5 enabled and a simulated clear ultrasonic
  reading. It covers ordinary course operation and left/right undershoot and
  overshoot recovery alongside the existing obstacle-present controller tests.
- Shared controller suite, including barcode capture, MQTT capture runtime,
  mapping, calibration, course settings and course simulation.
- Buddy 1 suite, including the updated real MQTT callbacks/bridge.

Integrated offline and Wi-Fi/MQTT builds with S3, S4 and S5 enabled passed.
Enabled-path builds are compile-only; the network build used the repository's
public example Wi-Fi configuration. The final selected profile remains offline
`S5_HARDWARE_CONFIRMED=0` (`_s5_UNCONFIGURED`), which cannot arm navigation.
See `testing/subsystem5/logs/2026-10-28_*` for results and build summaries.

No physical barcode, turn, scan, avoidance or recovery test was performed.
The Tribunal 1 Week 13 implementation and demonstration requirement remains;
these software checks do not establish demonstration evidence.
