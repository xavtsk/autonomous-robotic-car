# Build and test commands

Run commands from the repository root. This project uses Make and a Pico SDK
checkout with its required submodules. Use a native host compiler for host tests
and `arm-none-eabi-gcc` for firmware. Keep private Wi-Fi credentials in ignored
`config/wifi_secrets.h`; the example file is not a deployment configuration.

## Target builds

The default profile leaves integrated navigation disabled:

```sh
make -C build_make PICO_SDK_PATH=/absolute/path/to/pico-sdk -j8
```

The deployed `4ad2cc4` comparison profile is explicit below. This command builds
only. Its hardware confirmation setting records the already configured car; it
does not confirm a different robot's wiring or automatically flash/start anything.

```sh
make -C build_make PICO_SDK_PATH=/absolute/path/to/pico-sdk \
  SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=cyw43 NET=lwip MQTT=1 \
  S3_MOTION=1 S3_STRAIGHT_TEST=0 S4=1 S4_BENCH=0 \
  S5=1 S5_HARDWARE_CONFIRMED=1 IR_DEBUG=1 IMU_DEBUG=0 \
  TEST=0 IR_BENCH=0 DIAGNOSTICS=0 MOTION_SMOKE=0 -j8
```

In PowerShell, put the command on one line or use PowerShell's continuation
syntax; the backslashes above are POSIX shell continuations. Supply profile
options on every build. Compare the same options, compiler/SDK and source
revision when measuring RAM, flash or loadable bytes.

Bench, self-test, diagnostics and movement-smoke profiles are separate:
consult [IR bench](../testing/subsystem3/README.md),
[motion smoke](../testing/diagnostics/MOTION_SMOKE.md), and the
[hardware session guide](../testing/diagnostics/ROBOT_TEST_SESSION.md).
Do not substitute a test image for the integrated deployment profile.

## Executed host regressions

Use the [Linux GitHub Actions workflow](../.github/workflows/compliance.yml) for
the reproducible baseline/candidate compiler and sanitizer runs. These direct
commands are useful on a Linux machine that permits native test execution:

```sh
make -C testing/subsystem1 test
make -C testing/subsystem2 test
make -C testing/subsystem3 test
make -C testing/subsystem4 test
make -C testing/subsystem5 test
make -C testing/integration -f controller_test.mk test
make -C testing/diagnostics -f firmware_test.mk test smoke
make -C testing/network_wifi test
```

The workflow's driver records the expanded target list, actual invocations,
stdout/stderr and exit codes. Consult its artifacts for the exact tested scope.
Compiling a target without running it is not an executed test. If local Windows
Application Control blocks a program, preserve that result and use the authorized
Linux runner; do not infer success or keep trying alternate native launch paths.

## Compliance tooling

These Python commands inspect source or test the audit tools; they do not run a
generated C binary or touch the car:

```sh
python tools/compliance/barr_matrix.py --check
python -m unittest discover -s testing/compliance -p 'test_barr*.py'
python tools/compliance/barr_check.py
python testing/compliance/inventory.py --check
python tools/check_c_style.py --formatter /absolute/path/to/clang-format
python testing/change_requirement_1/check_widths.py
```

The BARR audit currently reports unresolved source findings and incomplete AST/
manual review. Its nonzero exit is a real incomplete/failing result, not an
installation problem to suppress. A clean format or width subset does not certify
the full standards. See the [rule matrix](compliance/BARR_RULE_MATRIX.md) for AST
input and function-documentation coverage, and the
[field inventory](compliance/CR1_FIELD_INVENTORY.md) for representation closure.

On Windows use `py -3` if `python` resolves to an unavailable Store alias.
Some dashboard Python tests compile and execute C helpers; run those in the
Linux suite, not under the local native-execution restriction.

## Evidence to retain

Record the exact source commit or working-tree digest, profile flags, toolchain
versions, test command, exit code and artifact hashes. Keep compile/link, host
execution, replay comparison, target layout/build, transfer, boot/telemetry and
physical validation as separate evidence levels. Flash and start actions are
separate from this build/check workflow. Preserve the deployed rollback artifact
identified in [CURRENT_STATUS.md](CURRENT_STATUS.md).
