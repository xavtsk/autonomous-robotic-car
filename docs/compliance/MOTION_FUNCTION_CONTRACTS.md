# PID and motion-word function contracts

This comments-only batch covers `speed_pid.c/.h` and `motion_pack.c/.h` against
immutable revision `3fda4d517e5720c51da7c41033bc0b9c221b11b5`. The
[bound receipt](MOTION_FUNCTION_CONTRACTS.json) records 31 definitions, including
the inline fault-bit helper, and 17 public prototypes. Private functions have
local contracts; public definitions refer to the complete header contracts.
All parameters are documented in their own or canonical declaration comment.

The review followed callers in `ctrl_motion.c`, `motion.c`, `buddy2_test.c` and
the packing tests, plus the enum/structure definitions in `motion.h`. It records
these distinctions without changing the implementation:

- PID gains and elapsed time are used without finite/sign validation. The
  exact zero stop condition clears the integral; it is not an epsilon test.
  Controller instances require caller-owned synchronization.
- Status encoding saturates magnitudes and wraps headings. Command builders
  reject unrepresentable values using the existing floating rounding test.
  Nominal decoded ranges do not describe the precise accepted input boundary:
  positive sub-step move speed can be accepted when it rounds to one step.
- Raw status/report decoding does not validate every enum or heading value.
  Sign-magnitude decoding can preserve negative zero from noncanonical input.
- Command decoding validates before writing output, so failed decoding leaves
  the destination unchanged. Building or decoding a word does not submit motion.
- The inline fault-bit helper requires a valid fault code; it does not validate
  arbitrary shift counts. Status packing inherits that input requirement.

The existing comments remain byte-for-byte and in their original order.
Preprocessing-token fingerprints, including directive structure, are unchanged
for all four files. No function, declaration, expression, include or directive
was edited. Clang 18.1.1 observes all 48 documented definitions/prototypes with
zero diagnostics under `-Wdocumentation -Wdocumentation-unknown-command`, using
the recorded target source contexts. Doxygen generation and whole-repository
documentation coverage remain separate, unfinished checks.

```sh
python tools/compliance/verify_format_batch.py --manifest docs/compliance/MOTION_FUNCTION_CONTRACTS.json
```

This rechecks the four bound token/comment relationships, not the compiler
observations. Pin `--candidate <contract-batch-commit>` after publication when
later work changes these files. The pinned clang-format 18.1.3 wrapper also
reports no formatting changes for this batch.

This supplies bounded evidence for BARR 2.2.h and the documentation-content
review; it does not close those rules across the repository. Existing naming,
function ordering, floating-comparison and other findings remain open. No C
test or firmware was executed as part of this documentation verification.
