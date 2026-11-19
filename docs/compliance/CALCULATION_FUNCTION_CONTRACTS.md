# Tilt, calibration and obstacle profile contracts

This comments-only batch documents all nine definitions and nine public
prototypes in `subsystem4/tilt_estimation.c/.h`,
`subsystem4/imu_calibration.c/.h` and `subsystem5/profile_obstacle.c/.h`.
Contracts describe axes, units, precision, rejection conditions, pointer
ownership and output mutation from the existing implementation and callers.

The tilt contracts retain the exact norm/projection comparisons and the
different Z-axis policies for tilt and heading. They describe signed zero,
possible heading rounding to 360, and the single revolution correction in
`tilt_angle_delta`; arbitrary angles and nonfinite deltas are not sanitized.
Invalid tilt inputs leave outputs unchanged. No null checks are invented.

Calibration requires a caller-owned initialized state and 100 accepted feeds.
Once ready, it returns immediately without checking new input or stationarity.
Pending captures reset on invalid samples, excessive level deviation or axis
spread. The function does not inspect timestamps or overrun. A new note
clarifies that the older gap comment refers to the terrain monitor's caller
logic, while retaining that original comment unchanged.

The obstacle sample decoder writes its three outputs before the final range
check, so a false return can still mutate them. Profile building instead
clears non-null output before validating inputs and leaves it zero on failure.
Its contract identifies required angular coverage, the strict profile-range
threshold, table projection/truncation and rounded-up edge padding. Side-peek
merge replaces one clearance and preserves existing valid geometry; it does
not recompute or validate the complete shape.

The [manifest](CALCULATION_FUNCTION_CONTRACTS.json) binds these six files to
pre-batch revision `0ba528b193872e2b515d44cb2d33dbdbb8585f1e`. Preprocessing
tokens/directive semantics, function interfaces and every original comment
in order remain unchanged. Canonical clang-format 18.1.3 leaves all six
resulting files unchanged.

Libclang 18.1.1 reports zero warnings/errors in 34 source/header contexts,
before and after the batch, including `-Wdocumentation` and
`-Wdocumentation-unknown-command`. Contexts come from the actual production
and host compiler receipts in Linux run `44595295824`, with local repository,
SDK and compiler-header mapping. Both recorded `APP_IMU_DEBUG=0` and `=1`
contexts are included. The six files contain no functions conditionally
compiled by that flag; this does not imply the caller/debug profiles are
otherwise equivalent.

```sh
python tools/compliance/verify_comment_batch.py --manifest docs/compliance/CALCULATION_FUNCTION_CONTRACTS.json --candidate WORKTREE
```

Pin the published batch commit instead of `WORKTREE` for historical checks.
The verifier recomputes only the bounded source proof; it does not repeat the
compiler observations or execute C. Doxygen generation, repository-wide
coverage and Linux runtime/target evidence remain separate gates. This batch
does not close the whole BARR documentation rule or alter any algorithm.
