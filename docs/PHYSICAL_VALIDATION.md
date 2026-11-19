# Supervised validation of the cleanup candidate

This is the current validation procedure. No cleanup firmware has been flashed
or driven by this implementation work. Flashing, motor operation and the hotspot
interruption are separate user-directed steps. The
[current status](CURRENT_STATUS.md) identifies the deployed rollback image and
the latest software-verified revision; later source changes do not inherit its
results.

## Match the two images and the test conditions

Keep the deployed `4ad2cc4` UF2 and its hash available for rollback. Prepare a
diagnostics image from that same revision and a candidate diagnostics image using
the identical Arm 14.3.1 compiler, pinned SDK, profile options and private network
settings. Use the explicit integrated profile in [BUILD_AND_TEST.md](BUILD_AND_TEST.md)
with `DIAGNOSTICS=1` for both. Record each build's exact command, revision, ELF,
UF2 and hashes in separate directories. CI images use public example network
settings and are build evidence; they are not ready for the real hotspot.

Keep sensor mounting, calibration, battery condition, track, obstacle, barcode
prints, mappings and requested speeds the same between the two images. Record
any unavoidable difference. Keep both wheels level during the existing startup
calibration. Verify stopped state and fresh telemetry before a supervised run.

| Item | Preserved setting or contract |
| --- | --- |
| Integrated IR/navigation update | 5 ms |
| Motion control | 10 ms |
| Stationary barcode bench | 10 ms; separate image |
| Buddy 1 application telemetry | [Protocol version 3](../app_program/subsystem1/MQTT_PROTOCOL.md) |
| Dashboard control requests/status | [Version 3](REMOTE_CONTROL_PROTOCOL.md) |
| Dashboard control replies | Version 4 |
| Last accepted baseline map | A forward, F left, Y right, Z U-turn; see the deployment receipt, not an assumed boot default |

## Save complete recordings

Use the actual serial port reported by the logger, close competing serial
monitors, and choose a new output directory for every attempt. On this laptop,
the existing dashboard environment supplies the Python interpreter:

```powershell
./build/dashboard-venv/Scripts/python.exe tools/capture_robot_log.py --list-ports
./build/dashboard-venv/Scripts/python.exe tools/capture_robot_log.py --port ACTUAL_PORT --output build/robot-logs/candidate-timing-01 --firmware EXACT_REVISION_AND_UF2_HASH --test cleanup-timing --duration 180 --line-csv
```

The logger receives bytes and does not submit driving commands. Its firmware
label is a user annotation, not automatic verification of what is flashed.
Copy the [session notes template](../testing/diagnostics/SESSION_NOTES_TEMPLATE.md)
into the created directory and record the actual workload, actions, timestamps
and interruptions. Retain `console.log`, `metadata.json`, optional arrival CSV,
dashboard run exports and video. Keep failed and interrupted attempts too.
Host arrival timestamps include buffering; they are not MCU timing.

## Compare the course behavior

Run each case on the matched baseline and candidate, recording failures as well
as successes. Keep the original barcode and map expectations unchanged.

| Case | Evidence to compare |
| --- | --- |
| Ordinary track following and junctions | State order, steering/motor requests, faults and independent course outcome. |
| Small and large barcodes, both directions | Actual letter, decision timestamp, work/capture outcome and physical action; retain rejected or unfinished captures. |
| Mapped junction actions | Forward, left, right and U-turn under the same pre-run map; preserve unmapped-stop behavior. |
| Hump crossing | Completion count, live/peak height, safety state and independently measured hump geometry. |
| Obstacle avoidance | Detection range, selected side, scan/profile, clearance and track return on the same obstacle. |
| GP20/GP21 controls | Countdown, STOP precedence, held-button inhibition, reset and restart state. |
| Dashboard controls | Request/reply sequence, acceptance versus execution, STOP/reset precedence and stale/replayed requests where supervised. |

The pre-run map freezes on START and clears only on reboot. A live map
acknowledgement does not demonstrate that a manoeuvre was physically completed.
Use the existing control paths; do not bypass readiness checks to make a test run.

## Three-minute timing and reconnection capture

Repeat the same supervised workload for each diagnostics image. Record for
180 seconds. At approximately 60 seconds, disable the test hotspot for ten
seconds, then restore it. Record the actual off/on times and the observed
Wi-Fi, MQTT and dashboard recovery times. Continue recording to the end. If the
capture drops out or the planned workload/interruption did not occur, retain it
as incomplete and repeat in a new directory.

Evaluate both captures using their recorded periods explicitly:

```powershell
./build/dashboard-venv/Scripts/python.exe testing/change_requirement_1/timing_check.py build/robot-logs/candidate-timing-01/console.log --motion-period-ms 10 --ir-period-ms 5
```

Repeat with the baseline capture path. The existing checker fails a recorded
motion start gap at or above 20 ms and an IR gap at or above 10 ms; missing or
malformed source evidence also fails. It retains the worst valid summary across
the capture. This is a missed-cycle screen, not proof of every deadline, WCET,
network recovery or the physical workload. Review execution intervals and
baseline/candidate differences separately. Other tasks need observations against
their own configured periods; the motion/IR checker does not cover them.

Any new functional difference or timing regression blocks deployment of the
affected change. Actual task stack high-water values remain separately required;
compiler `.su` estimates are not measurements on the car. Record unavailable
measurements as pending. Software traces, an uploaded UF2, a boot heartbeat and a
completed course are distinct evidence levels.
