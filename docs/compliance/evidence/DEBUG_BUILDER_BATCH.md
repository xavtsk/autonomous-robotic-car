# Compact controller debug scratch

The sampling task previously retained a `control_debug_sample_t` with fifty
32-bit fields, consuming 200 bytes before the separate 70-byte output packet.
The new `control_debug_builder_t` retains the explicit packet bytes and one
validity bit. Compiler allocation, alignment and stack effects must be measured
from the matched target build; information-bit counts alone are not RAM savings.

The public wide sample type, `control_debug_pack` and `control_debug_unpack`
remain available. Each builder setter checks the original wide input before
encoding. It enforces the same field widths and narrower semantic ranges, with
the motion-valid/available dependency checked before publication. A rejected
builder leaves its destination unchanged. Signed values retain the existing
two's-complement wire representation. Reassigning a field clears old bits.

The sampling owner retains sole access to the scratch. Publication still uses
the existing packet and mutex. The sample freshness test, sequence increment,
rate limiting, conversion order and short-circuit conditions remain in their
original order. The original scaling helper receives an addressable `int32_t`
temporary. Invalid motion readings clear the same six output quantities and
leave encoder output zero. No control command, sensor setting or wire version
changes in this batch.

Regression evidence required before acceptance:

- The shared debug trace compares acceptance, exact packet bytes and public
  output mutation for valid and invalid inputs against `4ad2cc4`.
- `control_builder_test.c` compares every field's boundary values and semantic
  limits with the wide packer, including failure destination preservation,
  repeated-field clearing, invalid selectors, wrong setter types and nulls.
- Instrumented runtime tests compare every produced controller-debug byte,
  STOP/reset decisions and queue/mutex observations against the baseline.
- GCC, Clang and ASan/UBSan execute on Linux. Native Windows checks compile and
  link only; they are never reported as execution.
- Matched Arm builds report allocation, flash size and compiler stack usage.
  Hardware timing and reconnection evidence remains outstanding.
