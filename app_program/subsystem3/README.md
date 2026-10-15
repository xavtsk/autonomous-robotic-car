# Buddy 3 — IR Line Following and Barcode Decoding

Starter modules only; no functionality is implemented.
Matching headers are in this folder.

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
layer in `app_program/app_main.c`; do not call other Buddies' modules directly.
Define micro T-Kernel task, queue, and synchronization contracts during integration.
Keep private implementation details within this source directory and follow the
Barr C Coding Standard as development proceeds.

## RTOS template lifecycle

This folder is now subsystem 3: IR line following and barcode decoding.
The original module files above remain placeholders. The separate
`subsystem3_init()` and `subsystem3_start()` hooks currently return `E_OK`
without creating objects, starting tasks, or configuring hardware.

Implement these hooks when this Buddy's module interfaces are ready. Keep
hardware drivers in this folder and register ownership/interfaces in `TEAM.md`.
Shared configuration is in `app_program/system_config.h`; the top-level
`include/` directory now belongs to the kernel and board support.

The test hook reports SKIP until real tests are written. Record evidence in
`testing/subsystem3/TEST_LOG.md`.
