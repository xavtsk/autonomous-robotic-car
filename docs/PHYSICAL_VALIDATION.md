# Archived on-car comparison procedure

The team prepared this protocol to compare a cleanup build with the deployed
firmware. The car was dismantled after the demonstration. The final submission
contains software regression results and Arm build measurements. No matched
on-car capture was collected for the final revision. [Submission results](CURRENT_STATUS.md)

## Firmware and test conditions

The protocol specified two diagnostic images built with Arm GNU 14.3.1, the
pinned SDK, the same private network configuration and the integrated profile
with `DIAGNOSTICS=1`. One image represented deployed revision `4ad2cc4`,
the other the cleanup revision. The deployed UF2 and its hash identified the
rollback image. Each build record called for the command, revision, ELF, UF2
and image hashes. The CI reference images use public example network settings.
[Build profiles](BUILD_AND_TEST.md)

The protocol called for identical sensor mounting, calibration, battery
condition, track, obstacle, barcode prints, mappings and requested speeds.
The starting conditions specified level wheels during calibration, a stopped
controller and fresh telemetry.

| Item | Preserved setting or contract |
| --- | --- |
| Integrated IR/navigation update | 5 ms |
| Motion control | 10 ms |
| Stationary barcode bench | 10 ms in its own firmware profile |
| Buddy 1 application telemetry | [Protocol version 3](../app_program/subsystem1/MQTT_PROTOCOL.md) |
| Dashboard control requests/status | [Version 3](REMOTE_CONTROL_PROTOCOL.md) |
| Dashboard control replies | Version 4 |
| Last accepted baseline map | A forward, F left, Y right, Z U-turn. [Deployment receipt](compliance/evidence/baseline-deployment-20261119.json) |

## Serial recording method

The reference commands list the available serial ports and capture one session
in its own directory. They assume the project's local dashboard environment.

```powershell
./build/dashboard-venv/Scripts/python.exe tools/capture_robot_log.py --list-ports
./build/dashboard-venv/Scripts/python.exe tools/capture_robot_log.py --port ACTUAL_PORT --output build/robot-logs/candidate-timing-01 --firmware EXACT_REVISION_AND_UF2_HASH --test cleanup-timing --duration 180 --line-csv
```

The logger reads serial data. The operator supplies the firmware label. A
session record pairs `console.log` and `metadata.json` with the workload, actions
and interruptions. Optional arrival timestamps, dashboard exports and video
add context. Host arrival times include serial buffering. The MCU diagnostic
timestamps supply the timing measurements. [Session notes template](../testing/diagnostics/SESSION_NOTES_TEMPLATE.md)

## Course behaviour

The protocol specified the same cases, barcode expectations and mappings for
both images.

| Case | Evidence to compare |
| --- | --- |
| Ordinary track following and junctions | State order, steering/motor requests, faults and independent course outcome. |
| Small and large barcodes, both directions | Actual letter, decision timestamp, physical action, rejected frames and incomplete captures. |
| Mapped junction actions | Forward, left, right and U-turn under the same pre-run map, plus the stop for an unmapped letter. |
| Hump crossing | Completion count, live/peak height, safety state and independently measured hump geometry. |
| Obstacle avoidance | Detection range, selected side, scan/profile, clearance and track return on the same obstacle. |
| GP20/GP21 controls | Countdown, STOP precedence, held-button inhibition, reset and restart state. |
| Dashboard controls | Request/reply sequence, acceptance versus execution, STOP/reset precedence and stale/replayed requests where supervised. |

The pre-run map freezes on START and clears on reboot. The protocol distinguished
acceptance of the map from physical completion of each manoeuvre and retained
the controller's normal readiness checks.

## Three-minute timing and reconnection capture

The protocol specified a 180-second capture per image. At roughly 60 seconds
the operator would interrupt the hotspot for ten seconds, restore it and
record recovery times for Wi-Fi, MQTT and the dashboard. A dropped recording
or a missed interruption made that session inconclusive.

The analysis command takes the configured periods of the image under test.

```powershell
./build/dashboard-venv/Scripts/python.exe testing/change_requirement_1/timing_check.py build/robot-logs/candidate-timing-01/console.log --motion-period-ms 10 --ir-period-ms 5
```

The checker reports failure for a motion start gap of at least 20 ms, an IR
gap of at least 10 ms, or missing or malformed timing evidence. It retains
the worst valid summary across the capture. These thresholds screen for a
missed cycle and permit gaps longer than the nominal 10 ms and 5 ms periods.
The firmware logs and session observations supply the separate evidence for
execution times, physical workload and reconnection. Compiler `.su` files
provide static stack estimates. Peak stack use on the submitted firmware was
unmeasured on hardware.
