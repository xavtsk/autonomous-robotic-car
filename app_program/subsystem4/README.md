# Buddy 4 — IMU Motion and Terrain Monitoring

Starter modules only; no functionality is implemented.
Matching headers are in this folder.

| Source / header basename | Purpose |
| --- | --- |
| `terrain_monitor` | Public terrain monitoring subsystem interface. |
| `imu_sensor` | IMU sample acquisition. |
| `imu_calibration` | IMU calibration. |
| `tilt_estimation` | IMU-based tilt estimation. |
| `hump_detection` | Hump detection and peak hump measurement. |
| `motion_events` | Collision detection and turn-rate/motion events. |

Only `terrain_monitor.h` declares the proposed subsystem lifecycle API:
`terrain_monitor_init(void)` and `terrain_monitor_process(void)`.
These functions have no definitions yet and cannot be linked until implemented.
The remaining headers contain interface TODOs.

Agree on data types, units, errors, timing, and ownership before implementation.
Route cross-subsystem requests and observations through the future integration
layer in `app_program/app_main.c`; do not call other Buddies' modules directly.
Define micro T-Kernel task, queue, and synchronization contracts during integration.
Keep private implementation details within this source directory and follow the
Barr C Coding Standard as development proceeds.

## RTOS template lifecycle

This folder is now subsystem 4: IMU-based motion and terrain monitoring.
The original module files above remain placeholders. The separate
`subsystem4_init()` and `subsystem4_start()` hooks currently return `E_OK`
without creating objects, starting tasks, or configuring hardware.

Implement these hooks when this Buddy's module interfaces are ready. Keep
hardware drivers in this folder and register ownership/interfaces in `TEAM.md`.
Shared configuration is in `app_program/system_config.h`; the top-level
`include/` directory now belongs to the kernel and board support.

The test hook reports SKIP until real tests are written. Record evidence in
`testing/subsystem4/TEST_LOG.md`.
