# Buddy 5 — Ultrasonic Obstacle Scanning and Avoidance

Starter modules only; no functionality is implemented.
Matching headers are in `include/buddy5/`.

| Source / header basename | Purpose |
| --- | --- |
| `obstacle_avoidance` | Public obstacle avoidance subsystem interface. |
| `ultrasonic_sensor` | Ultrasonic range acquisition. |
| `scan_servo` | Scanning servo positioning. |
| `obstacle_scan` | Coarse and fine ultrasonic scanning. |
| `obstacle_profile` | Obstacle profile construction. |
| `avoidance_planner` | Obstacle avoidance planning. |
| `line_recovery` | Line recovery after obstacle avoidance. |

Only `obstacle_avoidance.h` declares the proposed subsystem lifecycle API:
`obstacle_avoidance_init(void)` and `obstacle_avoidance_process(void)`.
These functions have no definitions yet and cannot be linked until implemented.
The remaining headers contain interface TODOs.

Agree on data types, units, errors, timing, and ownership before implementation.
Route cross-subsystem requests and observations through the future integration
layer in `src/main.c`; do not call other Buddies' modules directly.
Define micro T-Kernel task, queue, and synchronization contracts during integration.
Keep private implementation details within this source directory and follow the
Barr C Coding Standard as development proceeds.
