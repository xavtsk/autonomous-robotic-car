# Buddy 4 — IMU Motion and Terrain Monitoring

Starter modules only; no functionality is implemented.
Matching headers are in `include/buddy4/`.

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
layer in `src/main.c`; do not call other Buddies' modules directly.
Define micro T-Kernel task, queue, and synchronization contracts during integration.
Keep private implementation details within this source directory and follow the
Barr C Coding Standard as development proceeds.
