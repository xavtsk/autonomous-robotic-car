# Autonomous Robotic Car

Latest software update: [27 October robustness fixes and hardware retest checklist](docs/SOFTWARE_FIXES_2026-10-27.md).
This supersedes older packet sizes/timing labels; no full-course success is claimed.

INF2004 university project for Raspberry Pi Pico W + Cytron Robo Pico, developed
by five Buddies in C with micro T-Kernel 3.0.

This repository uses the structure and RTOS foundation from
[sirfonzie/INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate).
The original car module scaffolding is preserved inside the subsystem folders.
This integration branch combines Buddy 1's telemetry/MQTT service, Buddy 2's
motion implementation and Buddy 3's sensor/navigation implementation. A minimal
Vehicle Controller uses the teammate's single `s3_drive`/`s3_integration` owner
and reports state through Buddy 1. Navigation requires the explicit
`S3_MOTION=1` build and accepted calibration/local start controls. The default
profile does not run navigation. Real-track validation is still pending.
See [25 October integration review](docs/INTEGRATION_2026-10-25.md) for the imported
revisions, compatibility decisions, current tests and remaining hardware gaps.

For stationary sensor diagnostics, use the dedicated `IR_BENCH=1` build:
[Buddy 3 setup](testing/subsystem3/README.md). It starts only Buddy 3's sampling
and reporting tasks, with GP26 enabled by default, and skips motor startup.
See the [team-interface handoff](app_program/subsystem3/INTEGRATION.md).

For the first hardware session, start with the
[test-session guide](testing/diagnostics/ROBOT_TEST_SESSION.md). It covers safe
firmware selection, laptop USB log capture and a fill-in notes template.
`DIAGNOSTICS=1 TEST=0 IR_BENCH=0` enables optional low-rate motion/IR timing and
encoder observations in a separately named `_diag` image. It does not start
driving or change motion algorithms, periods or existing motion/sensor/network
priorities. The reused initial task becomes the optional reporter at priority 23. Default
`DIAGNOSTICS=0` builds retain no diagnostic state or code.

## Subsystem ownership

| Buddy | Folder | Responsibility |
|---|---|---|
| 1 | `app_program/subsystem1/` | WiFi communication, command and telemetry |
| 2 | `app_program/subsystem2/` | Motors, encoders, motion estimation and PID |
| 3 | `app_program/subsystem3/` | IR line following and barcode decoding |
| 4 | `app_program/subsystem4/` | IMU-based motion and terrain monitoring |
| 5 | `app_program/subsystem5/` | Ultrasonic scanning and obstacle avoidance |

Record student owners, pins, peripherals, task priorities and agreed interfaces
in [TEAM.md](TEAM.md) before implementing each subsystem.

## Repository structure

```text
app_program/
  app_main.c             # usermain(): initialize all subsystems, then start them
  app.h                  # Shared RTOS definitions and lifecycle declarations
  controller/            # Robot-level FSM and cross-subsystem coordination
  system_config.h        # Original shared configuration placeholder
  subsystem1/            # Buddy 1 sources, headers and lifecycle hooks
  subsystem2/            # Buddy 2 sources, headers and lifecycle hooks
  subsystem3/            # Buddy 3 sources, headers and lifecycle hooks
  subsystem4/            # Buddy 4 sources, headers and lifecycle hooks
  subsystem5/            # Buddy 5 sources, headers and lifecycle hooks
build_make/              # Makefiles, build rules and ignored generated firmware
config/                  # Kernel, console and platform configuration
include/                 # Kernel and board-support headers
kernel/                  # micro T-Kernel implementation
device/                  # Kernel device drivers
lib/                     # Board support, USB, WiFi and supporting libraries
etc/linker/              # Linker scripts
tools/                   # Host build tools
testing/
  subsystem1/ ... subsystem5/
  integration/
  system/
videos/                  # Test/demo video index
docs/                    # Migration notes and original template instructions
```

Buddy source and header files now live together. For example,
`src/buddy2/motor_control.c` and `include/buddy2/motor_control.h` moved to
`app_program/subsystem2/`. Tests moved from `tests/buddyN/` to
`testing/subsystemN/`. See [migration notes](docs/TEMPLATE_MIGRATION.md).

## Build for Pico W

Requirements: Make, `arm-none-eabi-gcc`, a native C++ compiler, and the Pico SDK
with the TinyUSB and CYW43 submodules initialized. SDK 2.3.0 is used locally.
This template uses Make directly; there is no CMake project.

From the repository root:

```sh
make -C build_make PICO_SDK_PATH="$HOME/pico/pico-sdk" -j8
```

Default profile: dual-core micro T-Kernel, USB CDC console, built-in CYW43 driver,
application tests off. Output:

```text
build_make/mtk3pico_smp1_usb_cdc_wifi.uf2
```

The radio driver is platform infrastructure. Buddy 1 adds its application outbox,
snapshot, commands and MQTT bridge on top of that service. TCP/IP and MQTT are
not enabled by the default build and need a private `config/wifi_secrets.h`.
Never commit that file.

Buddy 1's Tribunal 1 update uses compact binary telemetry and commands.
See its [learning/read order](app_program/subsystem1/README.md),
[binary MQTT contract](app_program/subsystem1/MQTT_PROTOCOL.md), and
[tests and memory evidence](testing/subsystem1/EVIDENCE.md).

To compile the test hooks:

```sh
make -C build_make PICO_SDK_PATH="$HOME/pico/pico-sdk" TEST=1 -j8
```

Output: `build_make/mtk3pico_smp1_usb_cdc_wifi_test.uf2`. Each empty subsystem
test reports SKIP; this is not evidence that subsystem hardware works.
The Makefile tracks build profiles, so pass options on every build.

Hold BOOTSEL while connecting the Pico W, then copy the chosen UF2 onto
`RPI-RP2`. Open the USB serial port to view the startup report. Firmware has
not been flashed or verified on hardware as part of this structure migration.

## Development rules

- Work on personal branches and submit pull requests; coordinate shared files.
- Keep each Buddy's implementation and drivers inside its subsystem folder.
- `subsystemN_init()` creates kernel objects; `subsystemN_start()` starts tasks
  only after every subsystem has completed initialization.
- Buddy 2 owns motion hardware and processing. The Vehicle Controller is the only
  application component that requests motion. In `S3_MOTION=1`, Buddy 3's ADC,
  navigation processing and controller run in one 10 ms owner task. The
  `vehicle_controller` facade adds no second task or completion consumer.
- Use the RTOS/BSP APIs provided by the template. Standalone Pico SDK `main()`,
  runtime initialization and CMake code are not drop-in replacements here.
- Follow Barr C style for new application code; preserve upstream notices.
- Follow the template's peripheral and network ownership rules in `TEAM.md`.
- Record test evidence in `testing/` and video links in `videos/`.
- Do not delete `build_make/` to clean artifacts: it contains tracked Makefiles.
  Use the template's `make -C build_make clean` target.

## Template provenance

Imported from upstream commit
[c3b09544cb1fd6dac6ad4b8f523be0af59b44482](https://github.com/sirfonzie/INF2004_ProjectTemplate/commit/c3b09544cb1fd6dac6ad4b8f523be0af59b44482).
The kernel, board support and build infrastructure retain upstream copyright
and license notices, including micro T-Kernel's T-License 2.2 notices.
The original [template README](docs/UPSTREAM_README.md) is retained as a reference;
its LED/producer/consumer/monitor examples have been replaced here with empty
car subsystem hooks.
