# Compliance cleanup and behavior checks

This cleanup starts from `f684e0c` on `codex/compliance-preserve-behaviour`.
It addresses the concrete findings in the 19 November review. The selected
automated checks do not certify every rule in the full BARR-C standard.

## Runtime changes

- Remote reply action/result/reason use 2/2/3 bits. Replies are version 4 and
  10 bytes; requests/status remain version 3. The dashboard accepts either
  reply version. Update/restart it before deploying the new firmware.
- Buddy 4 queue indices and flags, remote mailbox state and retained barcode
  flags use bounded bit-fields. Existing mutex ownership is retained.
- Buddy 5's signed heading/aim bit-fields use unsigned two's-complement
  storage and explicit accessors. Every formerly representable signed value
  has the same interpretation, including zero-initialized state.
- The internal unknown echo value is unsigned 2 instead of signed -1;
  healthy/no-echo remain 0/1. It is not a wire-format field.
- The motion fault helper evaluates its argument once. Debug-field validation
  uses helpers rather than macros returning from their enclosing function.

No speed, PID gain, steering policy, barcode threshold, quiet-zone rule,
sampling period, hump threshold, obstacle distance or mapping decision was
retuned. No Start/Stop command or flash operation is part of this cleanup.

## Mechanical changes

Application types now use lowercase `_t` names. Compiler-assisted refactoring
adds prefixes to pointer/global names and lengthens short local names; private
task entry points have `_task` suffixes. Public callers and test fixtures are
updated together. Every application source now includes a matching header,
including the on-device test modules. Module names are distinct in their first
eight characters, and the overlong function names have been shortened to at
most 31 characters. The barcode and debug APIs have documentation comments.

`.clang-format` specifies four-space indentation, separate braces, mandatory
control-statement braces, no tabs and an 80-column limit. The X-macro debug
schema is kept as one field per line between formatter markers; its field
order and widths are unchanged. Source notices are retained. The normal
application build's loadable flash bytes were identical before/after naming
and again before/after formatting. This proves those mechanical passes did
not change that build's executable contents. The final module/function renames
also produce identical flash contents when linked in the original object order.
The normal build sorts the renamed files differently, changing addresses and
adding 8 bytes of alignment padding. These comparisons are not a runtime test
of the separate representation changes.

Imported kernel/SDK interfaces and precision-sensitive arithmetic are retained.
Reducing floating-point precision to save more bits would change calculations,
so it is outside this behavior-preserving cleanup. This is not an exhaustive
function-by-function certification of the full standard, nor an assertion that
every application field has the minimum width. No unreviewed exception to a
BARR rule is represented as approved.

## Reproduce checks

From the repository root with Python and a compatible installed clang-format:

```text
python tools/check_c_style.py --formatter <path-to-clang-format>
python testing/change_requirement_1/check_widths.py
python -m unittest testing.change_requirement_1.test_timing_check testing.change_requirement_1.test_widths testing.change_requirement_1.test_c_style testing.dashboard.test_run_control testing.dashboard.test_barcode_protocol.BarcodeCapacityTests
```

The style check covers 129 application files: formatting, tabs, line widths,
module names and matching headers. It does not check every BARR rule. The width
check covers 143 declared fields and 18 totals, with selected actual C-source
bindings. Removing the terrain safety field or widening an audited queue
field makes its regression check fail.

The target firmware builds and 43 Python checks pass. All 45 host regression
programs compile/link with their configured warning checks; they have not been
executed here
because Windows Application Control blocks those native binaries. Run the
repository's C regression suites in an environment that permits them before
treating this as executed C-test evidence.

On a machine that permits native test execution, the existing suites are:

```text
make -C testing/subsystem1 test
make -C testing/subsystem2 test
make -C testing/subsystem3 test
make -C testing/subsystem4 test
make -C testing/subsystem5 test
make -C testing/integration -f controller_test.mk test
make -C testing/diagnostics -f firmware_test.mk test smoke
```

The final same-profile image uses 129696 bytes of static RAM, compared with
129720 bytes at the baseline (24 bytes less). It is available locally as
`build/compliance-review-20261119/compliance-wifi.uf2`; it has not been flashed.
The repository [verification record](../testing/change_requirement_1/logs/2026-11-19_compliance_checks.json)
identifies the baseline, exact profile, checks, hashes and validation limits.

See [Change Requirement evidence](CHANGE_REQUIREMENT_1_EVIDENCE.md) for the
matched RAM/message-size comparisons and the outstanding physical timing
capture. A synthetic log regression is not evidence about the real car.
