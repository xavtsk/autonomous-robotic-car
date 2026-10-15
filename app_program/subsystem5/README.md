# Buddy 5 — Ultrasonic Obstacle Scanning and Avoidance

Starter modules only; no functionality is implemented.
Matching headers are in this folder.

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
layer in `app_program/app_main.c`; do not call other Buddies' modules directly.
Define micro T-Kernel task, queue, and synchronization contracts during integration.
Keep private implementation details within this source directory and follow the
Barr C Coding Standard as development proceeds.

## RTOS template lifecycle

This folder is now subsystem 5: Ultrasonic scanning and obstacle avoidance.
The original module files above remain placeholders. The separate
`subsystem5_init()` and `subsystem5_start()` hooks currently return `E_OK`
without creating objects, starting tasks, or configuring hardware.

Implement these hooks when this Buddy's module interfaces are ready. Keep
hardware drivers in this folder and register ownership/interfaces in `TEAM.md`.
Shared configuration is in `app_program/system_config.h`; the top-level
`include/` directory now belongs to the kernel and board support.

The test hook reports SKIP until real tests are written. Record evidence in
`testing/subsystem5/TEST_LOG.md`.
