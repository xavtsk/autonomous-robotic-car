# Operator regression observations

The shared operator fixture now has an additional, read-only observer. Both the
fixed `4ad2cc42595722cedb47102e4343298339040d40` baseline and the candidate use
the same fixture, observer and compiler options. The production operator code is
selected from each subject checkout. No operator storage change is part of this
batch.

After each existing fixture period, the observer records:

- The simulated clock, operator phase, timestamps, sequence, countdown and flags.
- Both buttons' debounce timestamps and flags, raw inputs and callback counters.
- Controller state, reason, command sequence/result and calibration state.
- Calibration counts/extrema and exact binary32 average bits using `memcpy`.
- Every newly appended console byte, hex encoded in order.

The original fixture assertions and callback order remain unchanged. The
instrumentation test removes only the new observations and include, reverses the
subject-path substitution, and requires the original C token stream. It also
checks that instrumentation leaves the source file unchanged. Observer calls
occur after the existing `operator_step` call and do not read the clock through
callbacks. Pointer addresses are never printed; only callback presence is
observed.

The existing fixture covers GP21 at startup, countdown and both-button release
orders, power-up GP20, calibration, refused starts, stop/restart, fault reset,
recalibration, overlapping calibration ranges and stopping during sampling.
These observations supplement the existing assertions; they do not establish
all invalid-input domains for future packing.

`run_host.py` compiles and executes the instrumented fixture in GCC, Clang and
ASan/UBSan modes for both revisions. `differential.py` compares the complete
output bytes, including state rows, console rows and original fixture output.
Build commands, logs and outputs are retained in each unique CI artifact.

Linux execution is recorded for candidate
`b4ad5d09a9953cfd308bf4e2d19cd9ffcd4d0c60` in
[run 44597427198](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44597427198).
All six baseline/candidate GCC, Clang and ASan/UBSan modes compile and execute
the operator fixture successfully, with no sanitizer errors. The differential
artifact `14566786566` contains 8,066 semantic rows and 8,173 total output rows
per subject; the complete bytes match SHA-256
`5a759cfcded604288fab493d7361673ae767bf5868f163a115a0ff36a92c2a1f`.

All 12 target profiles build successfully. Production artifact `14565678331`
has 465,852 text bytes and 129,504 bytes of static RAM. Its flash payload is
byte-identical to the preceding `302100e` image, with SHA-256
`d4a0bb724368d25dad8c3dd9d44e548354ffc15b19cd57aec07a01bf9dc433c9`.
The [execution receipt](evidence/operator-linux.json) binds exact source and
artifact hashes, all six host results and the independently checked output.

The complete workflow remains failed: two original replay expectations remain,
the conditional union observes 376/421 arms with 45 unobserved and zero receipt
errors, and repository compliance audits remain open. The stale formatter mocks
from the preceding run are fixed and pass in this run.

The local Windows check remains compilation and linking only. The fixture's
existing 10 ms simulated period is unchanged; it does not measure the physical
integrated 5 ms IR sampling task, RTOS scheduling or hardware latency. This image
has not been flashed. Later source changes need their own matching execution.

Sources: [observer](../../tools/compliance/traces/op_trace.c),
[interface](../../tools/compliance/traces/op_trace.h),
[instrumentation](../../tools/compliance/instrument_harnesses.py),
[inverse check](../../testing/compliance/test_instrument_harnesses.py).
