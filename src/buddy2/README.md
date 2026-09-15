# Buddy 2 — Motion Control

Starter modules only; no functionality is implemented.
Matching headers are in `include/buddy2/`.

| Source / header basename | Purpose |
| --- | --- |
| `motion` | Public motion subsystem interface. |
| `motor_control` | Motor actuation interface. |
| `wheel_encoder` | Wheel encoder acquisition. |
| `motion_estimation` | Encoder-based speed and distance estimation. |
| `speed_pid` | Motor speed PID control. |
| `motion_controller` | Straight-line correction and encoder-based turning. |

Only `motion.h` declares the proposed subsystem lifecycle API:
`motion_init(void)` and `motion_process(void)`.
These functions have no definitions yet and cannot be linked until implemented.
The remaining headers contain interface TODOs.

Agree on data types, units, errors, timing, and ownership before implementation.
Route cross-subsystem requests and observations through the future integration
layer in `src/main.c`; do not call other Buddies' modules directly.
Define micro T-Kernel task, queue, and synchronization contracts during integration.
Keep private implementation details within this source directory and follow the
Barr C Coding Standard as development proceeds.
