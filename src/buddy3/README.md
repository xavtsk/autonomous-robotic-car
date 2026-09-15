# Buddy 3 — IR Line Following and Barcode Decoding

Starter modules only; no functionality is implemented.
Matching headers are in `include/buddy3/`.

| Source / header basename | Purpose |
| --- | --- |
| `line_navigation` | Public line navigation subsystem interface. |
| `ir_sensor` | Acquisition and calibration for three IR sensors. |
| `line_detection` | Line position estimation and junction detection. |
| `line_controller` | Line-following control. |
| `barcode_decoder` | Barcode detection and decoding. |

Only `line_navigation.h` declares the proposed subsystem lifecycle API:
`line_navigation_init(void)` and `line_navigation_process(void)`.
These functions have no definitions yet and cannot be linked until implemented.
The remaining headers contain interface TODOs.

Agree on data types, units, errors, timing, and ownership before implementation.
Route cross-subsystem requests and observations through the future integration
layer in `src/main.c`; do not call other Buddies' modules directly.
Define micro T-Kernel task, queue, and synchronization contracts during integration.
Keep private implementation details within this source directory and follow the
Barr C Coding Standard as development proceeds.
