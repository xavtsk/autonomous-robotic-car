# Buddy 2 — Motion Control

Matching headers are in this folder.

| Source / header basename | Purpose |
| --- | --- |
| `motion` | Public motion subsystem interface. |
| `motor_control` | Motor actuation interface. |
| `wheel_encoder` | Wheel encoder acquisition. |
| `motion_estimation` | Encoder-based speed and distance estimation. |
| `speed_pid` | Motor speed PID control. |
| `motion_controller` | Straight-line correction and encoder-based turning. |
| `subsystem2` | RTOS lifecycle: mutex, 10 ms cyclic handler and the motion task. |

`motion.h` declares the subsystem lifecycle API: `motion_init(void)` sets up
the hardware and `motion_process(void)` runs one control period. Both are
called by the motion task in `subsystem2.c`.

Agree on data types, units, errors, timing, and ownership before implementation.
Route cross-subsystem requests and observations through the future integration
layer in `app_program/app_main.c`; do not call other Buddies' modules directly.
Define micro T-Kernel task, queue, and synchronization contracts during integration.
Keep private implementation details within this source directory and follow the
Barr C Coding Standard as development proceeds.

## How it runs

- `subsystem2_init()` creates one mutex and one cyclic handler (10 ms).
- `subsystem2_start()` starts the **motion** task (priority 14, core 0). The task
  sets up the PWM and the encoder interrupt, then runs `motion_process()` every
  time the cyclic handler wakes it.
- Only the motion task drives the motors. Requests from other tasks change the
  statechart under the mutex; `stop()` also wakes the task so it brakes at once.

## Interface (motion.h)

Requests return immediately; the Vehicle Controller follows progress with
`motion_poll()`. Units: metres, metres per second, degrees.

| Function | Statechart |
| --- | --- |
| `moveForward(distance)`, `moveBackward(distance)`, `motion_request_drive(m, speed)` | DRIVE |
| `turnLeft(angle)`, `turnRight(angle)`, `motion_request_turn(deg, speed)`, `motion_request_u_turn(speed)` | TURN |
| `motion_request_manoeuvre(MANOEUVRE_LEFT / _RIGHT / _STRAIGHT / _U_TURN)` | TURN / DRIVE |
| `motion_request_velocity(left, right)` (re-send within 250 ms) | VELOCITY CONTROL |
| `stop()` | any active state -> IDLE |
| `motion_reset()` | MOTION FAULT -> IDLE once the fault has cleared |
| `motion_poll()`, `motion_get_fault()` | status; COMPLETE is reported once |
| `wheel_speed_*_mps()`, `wheel_distance_*_m()`, `robot_speed_mps()`, `robot_distance_m()`, `robot_heading_deg()`, `enc_l_count()`, `enc_r_count()` | estimates (FR4), safe from any task |

Faults (motors braked, latched until `motion_reset()`): stall / no encoder
progress, stale velocity command, invalid encoder signal, wheel turning against
its command, drive/turn timeout.

## RTOS template lifecycle

This folder is subsystem 2: Motion control using motors and wheel encoders.
`subsystem2_init()` and `subsystem2_start()` create the kernel objects and
start the motion task described in *How it runs*.

Keep hardware drivers in this folder and register ownership/interfaces in
`TEAM.md`. Shared configuration is in `app_program/system_config.h`; the
top-level `include/` directory now belongs to the kernel and board support.

The test hook (`subsystem2_test.c`, `make TEST=1`) runs unit tests S2-U01 to
S2-U12. Record evidence in `testing/subsystem2/TEST_LOG.md`.
