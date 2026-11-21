# Autonomous Robotic Car

INF2004 Embedded Systems Programming project for Raspberry Pi Pico W and Cytron
Robo Pico, implemented in C on micro T-Kernel 3.0 by five subsystem teams.

Start with [release status and evidence](docs/CURRENT_STATUS.md),
[build and test commands](docs/BUILD_AND_TEST.md), or the
[documentation index](docs/README.md).

The coursework candidate uses `4ad2cc4` as its fixed behaviour baseline.
The [BARR-C ledger](docs/compliance/BARR_RULE_MATRIX.md) records reviewed rules
and pending findings. The [Change Requirement 1 results](docs/CHANGE_REQUIREMENT_1_EVIDENCE.md#phase-3-submission-results)
record field widths, matched Arm RAM and message sizes, and regression scope.
On-car timing and full-course validation remain pending.

## Subsystems and ownership

| Buddy | Source folder | Responsibility |
| --- | --- | --- |
| 1 | [subsystem1](app_program/subsystem1/README.md) | MQTT telemetry, commands and bounded communication queues. |
| 2 | [subsystem2](app_program/subsystem2/README.md) | Motor outputs, encoders, motion estimation and PID. |
| 3 | [subsystem3](app_program/subsystem3/README.md) | IR acquisition, line navigation and barcode decoding. |
| 4 | [subsystem4](app_program/subsystem4/README.md) | IMU sensing and terrain/motion monitoring. |
| 5 | [subsystem5](app_program/subsystem5/README.md) | Ultrasonic scanning, obstacle avoidance and recovery. |

The [controller](app_program/controller/README.md) coordinates the enabled
subsystems. Buddy 2 owns motor writes; the integrated sampling/navigation owner
authorizes motion and consumes completion reports. Default builds leave
navigation disabled. The explicit integrated profile boots disarmed and requires
the existing readiness/calibration and START checks. See [TEAM.md](TEAM.md) for
pin, peripheral and task ownership.

## Build and verification

The project uses Make. Building requires an Arm GCC toolchain, a host C++
compiler and a Pico SDK checkout with its required submodules. Keep network
credentials in the ignored `config/wifi_secrets.h`.

```sh
make -C build_make PICO_SDK_PATH=/absolute/path/to/pico-sdk -j8
```

That command selects defaults, not the deployed integrated profile. Use the
[explicit profile and check commands](docs/BUILD_AND_TEST.md) for reproducible
comparisons. The [Linux workflow](.github/workflows/compliance.yml) executes C
regressions and records results for baseline and candidate revisions. Consult
the actual run and artifacts before reporting a pass. Local compile/link success
is separate from executed tests, and host tests are separate from hardware proof.

For controlled hardware sessions, use the
[session guide](testing/diagnostics/ROBOT_TEST_SESSION.md) and record evidence in
[testing](testing/README.md). Building, checking or editing documentation does not
flash firmware or start the car.

## Repository map

| Path | Contents |
| --- | --- |
| `app_program/` | Application, controller, subsystem drivers and device test hooks. |
| `build_make/` | Tracked Makefiles and generated firmware; do not delete this directory. |
| `config/` | Platform configuration and the untracked private Wi-Fi configuration. |
| `kernel/`, `include/` | Imported RTOS and interface foundations. |
| `device/`, `lib/` | Platform code; owned adapter boundaries are explicitly inventoried. |
| `testing/` | Host regressions, device test records, compliance inventory and logs. |
| `tools/` | Build, dashboard, capture, analysis and compliance tooling. |
| `docs/` | Current guides, standards evidence and preserved historical records. |

Keep changes scoped to their owner, preserve upstream notices, and retain old
test results. Update current interface guides alongside code changes. Historical
records describe their dated revision and must not be rewritten as new results.
Use `make -C build_make clean` when cleaning target artifacts.

## Template provenance

The RTOS foundation and repository layout derive from
[sirfonzie/INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate)
at commit `c3b09544cb1fd6dac6ad4b8f523be0af59b44482`.
Imported kernel, board and SDK sources retain their licenses, including
micro T-Kernel's T-License 2.2. The
[upstream README](docs/UPSTREAM_README.md) and
[migration record](docs/TEMPLATE_MIGRATION.md) are historical references.
