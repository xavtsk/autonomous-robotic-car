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

On this Windows laptop, the pinned local compiler and SDK are under
`C:\Code\autonomous-robotic-car\build\tooling`. The corresponding explicit build
command is below; run it from the repository root. It does not flash anything.
Keep the resulting candidate separate from the saved baseline directory.

```powershell
$env:Path = "$PWD\build\tooling\arm\bin;C:\msys64\ucrt64\bin;C:\msys64\usr\bin;$env:Path"
make -C build_make PICO_SDK_PATH="$PWD/build/tooling/pico-sdk" SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=cyw43 NET=lwip MQTT=1 S3_MOTION=1 S3_STRAIGHT_TEST=0 S4=1 S4_BENCH=0 S5=1 S5_HARDWARE_CONFIRMED=1 IR_DEBUG=1 IMU_DEBUG=0 TEST=0 IR_BENCH=0 IR_CHANNELS=1 DIAGNOSTICS=0 MOTION_SMOKE=0 -j4
```

Verify `arm-none-eabi-gcc -dumpfullversion` is `14.3.1` and
`git -C build/tooling/pico-sdk rev-parse HEAD` is
`98a542c1a62fb549ffb5d66a3e5892b06276b670` before comparing images. A newly
installed toolchain or changed private hotspot configuration requires a new
manifest; do not compare its hash with a public-example CI image as though the
inputs were identical. Windows Application Control may also prevent local build
helpers from launching; retain the failure instead of treating it as a C result.

The operating periods are distinct and unchanged:

| Path | Period |
| --- | --- |
| Integrated IR acquisition/navigation | 5 ms (200 Hz) |
| Motion control | 10 ms (100 Hz) |
| Stationary IR bench | 10 ms (100 Hz) |

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

Each target artifact contains full ELF/UF2/map files, loadable flash bytes,
compiler stack reports, real compile commands, build logs and hashes. Newer
target runs additionally contain all symbol allocations and all stack changes,
including increases, removed functions and duplicate local symbol names.
Information-bit counts in CR1 tables are separate from these padded target
allocations. Compiler stack reports are not physical task high-water values.

The conditional-coverage collector preprocesses a disposable copy using the
recorded compiler definitions and public example network settings. Original
sources are hashed before and after observation. Missing arms, preprocessing
failures and host-only paths remain explicit; a successful collector invocation
does not mean every supported conditional branch has been covered.
The [matrix union](compliance/CONDITIONAL_COVERAGE.md) now requires every named
profile receipt and matches them to the complete current source inventory.
An unobserved arm or a failed/stale receipt keeps that gate incomplete.

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

The Windows host scripts accept `-CompileOnly` and create a unique directory for
every invocation under `build/host-tests/<suite>/<UTC-time>-<unique-suffix>`:

```powershell
./tools/test-ir-reflectance.ps1 -CompileOnly
./tools/test-buddy3.ps1 -CompileOnly
./tools/test-buddy4.ps1 -CompileOnly
```

Each native command has an argument-list receipt, stdout/stderr log, exit status
and result receipt. Successful compiler outputs include a hash. The summary
records `COMPILE_PASS_EXECUTION_NOT_RUN`, with a separate `SKIP_COMPILE_ONLY`
entry for every planned test execution. These are compile/link results only.
Buddy 3 also records its reflectance child-script invocation; that child prints
its own evidence directory. Prior directories and logs are retained. A compiler
or script failure remains a failure, including failures before any test runs.
Input preservation hashes cover explicit source arguments; complete transitive
source/configuration binding comes from the Linux compliance pipeline.

`testing/compliance/test_host_evidence.ps1` checks the logging helper with an
existing Python interpreter. It exercises argument quoting, stderr, nonzero
exit codes, compile-only skips, unique directories and a deliberately modified
disposable input. It does not execute a generated C binary. The retained
[Windows logging receipt](compliance/evidence/windows-host-logging.json) records
the interpreter used and the execution-policy limitation of the separate
Windows PowerShell 5 launch.

## Evidence to retain

Record the exact source commit or working-tree digest, profile flags, toolchain
versions, test command, exit code and artifact hashes. Keep compile/link, host
execution, replay comparison, target layout/build, transfer, boot/telemetry and
physical validation as separate evidence levels. Flash and start actions are
separate from this build/check workflow. Preserve the deployed rollback artifact
identified in [CURRENT_STATUS.md](CURRENT_STATUS.md).
