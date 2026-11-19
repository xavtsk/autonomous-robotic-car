# Private MQTT state representation

This batch changes only the retained discrete state in `mqtt_utk.c`. The
CYW43/lwIP task continues to own the state and callbacks. No volatile cross-task
flags, public callback signatures, topic strings, packet buffers or hook
validation inputs are narrowed. Serialization remains explicit.

| Retained quantity | Representation and preserved domain |
| --- | --- |
| Connection, truncation, retry and fairness flags | One-bit Boolean members, initialized to the original values. |
| Retry source | Two bits for none, application or capture. |
| Receive byte counts | Five bits for 0 through 17; the original capacity guard saturates accumulation before incrementing. |
| Capture packet length | Eight bits; the hook's wide return is rejected above 164 before narrowing. |
| Preview fairness counts | Two bits for 0 through 2; the existing saturation guards remain. |
| Publish interval | Two-bit encoding of exactly 0, 20 or 100 ms; decoded before the unchanged timestamp comparison. |

The complete pending application message stays a wide validation boundary: its
callback may provide invalid metadata that must still be rejected. Timestamps
remain full-width modular counters. Clock reads, update ordering, synchronous
completion handling, FIFO/retry precedence and fragmented-message admission are
unchanged. The state struct's padding is not counted as information bits.

Before application, [Linux run 44584191817](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44584191817)
executed the 2,093 receive boundary cases, all four debug configurations and the
existing network suites. Shared baseline/candidate packet traces matched. That
run precedes this representation change and is **characterization evidence**,
not evidence that the new candidate executed.

Local Arm 14.3.1 compilation with the recorded production flags measures
19 bytes of old discrete symbols versus an 8-byte private state object. This is
an 11-byte symbol sum reduction; final linker padding and whole-image RAM still
require matched CI builds. Compiler stack estimates change from 64 to 40 bytes
for `mqtt_utk_poll`, and from 40 to 32 bytes for `publish_preview`; other emitted
function estimates are unchanged. These are not physical stack measurements.
Commands, objects and `.su` files are retained under `build/mqtt-state-target/`.
All four layouts compile under C99 locally; no generated Windows C executable
was run.

The MinGW host bitfield ABI can allocate 16 bytes for the same fields, while
the disabled layout occupies 4 bytes and the measured production Arm layout
occupies 8. The allocation assertion is therefore specific to the Arm target;
field-domain assertions apply to every build. Compiler contexts and padded
type variants remain visible in the data inventory.

Candidate Linux differential/sanitizer tests, final target manifests and
physical timing/reconnection evidence are pending. No firmware has been flashed
as part of this batch.
