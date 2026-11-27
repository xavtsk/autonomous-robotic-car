# Autonomous Robotic Car

INF2004 Embedded Systems Programming project for Raspberry Pi Pico W and Cytron
Robo Pico, implemented in C on micro T-Kernel 3.0 by five subsystem teams.

The team completed the physical demo and dismantled the car. The
[mission results](docs/CURRENT_STATUS.md#mission-requirements) link each of the ten
course requirements to the implementation, test result and measurement conditions.
The obstacle simulation completed 80 of 85 bypasses, including 16 of 17 in
the standard configuration. The [build and test guide](docs/BUILD_AND_TEST.md)
gives Linux prerequisites for the whole-repo check. From a fresh checkout run:

```sh
python3 tools/verify_submission.py
```

The results distinguish actual mission outcomes from changes since the submitted
baseline. [Submission results](docs/CURRENT_STATUS.md) ·
[Change Requirement 1 evidence](docs/CHANGE_REQUIREMENT_1_EVIDENCE.md) ·
[BARR-C review](docs/compliance/BARR_RULE_MATRIX.md)

## Coursework deliverables and evidence

| Buddy | Implementation | Where to check the result |
| --- | --- | --- |
| 1 · Communication | [Communication API](app_program/subsystem1/buddy1.h) and [MQTT telemetry and command protocol](app_program/subsystem1/MQTT_PROTOCOL.md) | [Protocol tests and measured message sizes](testing/subsystem1/EVIDENCE.md) |
| 2 · Motion control | [Motion-control library](app_program/subsystem2/motion.h) | [PID tuning and measured drive, turn and stopping accuracy](testing/subsystem2/TEST_LOG.md#results) |
| 3 · Line and barcode navigation | [Line-following module](app_program/subsystem3/line_controller.c), [barcode decoder](app_program/subsystem3/bc_decoder.c) and [navigation command interface](app_program/subsystem3/line_navigation.h) | [Mounted IR calibration](testing/subsystem3/IR_CALIBRATION_2026-10-27.md), [recorded waveforms](testing/subsystem3/fixtures/barcode/README.md) and [baseline replay comparison](docs/CHANGE_REQUIREMENT_1_EVIDENCE.md#recorded-barcode-regression) |
| 4 · Terrain monitoring | [IMU processing and height estimation](app_program/subsystem4/README.md#processing-and-integration) and [hump detection](app_program/subsystem4/hump_detection.c) | [Terrain analysis report](app_program/subsystem4/TERRAIN_ANALYSIS.md) and [controller test results](testing/subsystem4/TEST_LOG.md) |
| 5 · Obstacle handling | [Ultrasonic scan](app_program/subsystem5/scan_obstacle.c), [obstacle profile](app_program/subsystem5/profile_obstacle.c), [avoidance](app_program/subsystem5/avoid_obstacle.c) and [line recovery](app_program/subsystem5/line_recovery.c) | [Scan and bypass tests with simulated inputs](testing/subsystem5/README.md) |

The controller coordinates the enabled subsystems. Buddy 2 owns motor writes.
The integrated sampling and navigation code authorises motion and consumes
completion reports. Default builds leave
navigation disabled. The explicit integrated profile boots disarmed and requires
the readiness, calibration and START checks. [Controller interfaces](app_program/controller/README.md) ·
[Pin, peripheral and task ownership](TEAM.md)

## Build and verification

From the repository root, run these two Python 3 commands to verify the
Change Requirement 1 field table and exercise its width and timing checks.
They run without a car or Pico SDK.

```sh
python testing/change_requirement_1/check_widths.py
python -m unittest discover -s testing/change_requirement_1 -p "test_*.py"
```

The project uses Make. Building requires an Arm GCC toolchain, a host C++
compiler and a Pico SDK checkout with its required submodules. Keep network
credentials in the ignored `config/wifi_secrets.h`.

```sh
make -C build_make PICO_SDK_PATH=/absolute/path/to/pico-sdk -j8
```

This command builds the default profile. The build guide gives the integrated
profile and commands for reproducing the software checks. The Linux workflow
runs C regressions against the selected baseline and candidate revisions.
[Build and test commands](docs/BUILD_AND_TEST.md) ·
[Linux workflow](.github/workflows/compliance.yml) · [Documentation index](docs/README.md)

## Repository map

| Path | Contents |
| --- | --- |
| `app_program/` | Application, controller, subsystem drivers and device test hooks. |
| `build_make/` | Tracked Makefiles and local firmware build outputs. |
| `config/` | Platform configuration and the untracked private Wi-Fi configuration. |
| `kernel/`, `include/` | Imported RTOS and interface foundations. |
| `device/`, `lib/` | Platform code and team-owned adapters. |
| `testing/` | Host regressions, device test records, compliance inventory and logs. |
| `tools/` | Build, dashboard, capture, analysis and compliance tooling. |
| `docs/` | Current guides, standards evidence and preserved historical records. |

## Template provenance

The RTOS foundation and repository layout derive from
[sirfonzie/INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate)
at commit `c3b09544cb1fd6dac6ad4b8f523be0af59b44482`.
Imported kernel, board and SDK sources retain their licences, including
micro T-Kernel's T-License 2.2. The
[upstream README](docs/UPSTREAM_README.md) and
[migration record](docs/TEMPLATE_MIGRATION.md) are historical references.
