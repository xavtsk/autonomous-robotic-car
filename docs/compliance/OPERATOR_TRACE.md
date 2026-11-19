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

The local Windows check is compilation and linking only. Linux execution and
matched baseline/candidate output must be recorded before using this fixture as
evidence for a subsequent operator-state refactor. The fixture's existing 10 ms
simulated period is unchanged; it does not measure the physical integrated
5 ms IR sampling task, RTOS scheduling or hardware latency.

Sources: [observer](../../tools/compliance/traces/op_trace.c),
[interface](../../tools/compliance/traces/op_trace.h),
[instrumentation](../../tools/compliance/instrument_harnesses.py),
[inverse check](../../testing/compliance/test_instrument_harnesses.py).
