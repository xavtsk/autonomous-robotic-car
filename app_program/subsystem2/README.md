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
| `motion_pack` | Minimum-bit packing of shared data: status (64 bits), reports (16 bits), commands (32 bits). See `DATA_FORMAT.md`. |

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

- `subsystem2_init()` creates one mutex, one message buffer (report queue)
  and one cyclic handler (10 ms).
- `subsystem2_start()` starts the **motion** task (priority 14, core 0). The task
  sets up the PWM and the encoder PIO, then runs `motion_process()` every
  time the cyclic handler wakes it.
- Only the motion task drives the motors. Requests from other tasks change the
  statechart under the mutex; `stop()` also wakes the task so it brakes at once.
- Every done / cancelled / fault report goes into a message buffer (bounded
  queue, 8 reports) created in `subsystem2_init()`. Motion Control never waits
  on it: if it is full, the report is dropped and the next one has `b_lost`
  set. The Vehicle Controller reads it with `motion_get_report()` or waits with
  `motion_wait_report()`.

## Interface (motion.h)

Requests return immediately; the Vehicle Controller follows progress with
`motion_poll()`. Units: metres, metres per second, degrees.

| Function | Statechart |
| --- | --- |
| `moveForward(distance)`, `moveBackward(distance)`, `motion_request_drive(m, speed)` | DRIVE |
| `turnLeft(angle)`, `turnRight(angle)`, `motion_request_turn(deg, speed)`, `motion_request_u_turn(speed)` | TURN |
| `motion_request_manoeuvre(MANOEUVRE_LEFT / _RIGHT / _STRAIGHT / _U_TURN)` | TURN / DRIVE |
| `motion_request_velocity(left, right)` (re-send within 250 ms) | VELOCITY CONTROL |
| `stop()` | any active state -> IDLE, report cancelled (priority path) |
| `motion_cancel(id)` | cancel only if `id` is the running command; otherwise `MOTION_ERR_OBSOLETE` |
| `motion_reset()` | MOTION FAULT -> IDLE once the fault has cleared |
| `motion_poll()`, `motion_get_fault()` | status; COMPLETE is reported once |
| `motion_command_id()` | ID of the last accepted command (read it after `MOTION_OK`) |
| `motion_get_report(&report)`, `motion_wait_report(&report, timeout_ms)` | done / cancelled / fault reports with their command IDs, oldest first, from a bounded queue (8) |
| `motion_report_peak()` | most reports ever waiting, to validate the queue length |
| `motion_get_status(&status)`, `motion_get_status_packed()` | snapshot of the shared data (telemetry), full precision or packed in 64 bits |
| `motion_request_packed(word)` with `motion_cmd_...()` | same requests as above, as a packed 32-bit command word |
| `motion_is_stopped()` | stopped acknowledgement: no command running, both wheels still |
| `wheel_speed_*_mps()`, `wheel_distance_*_m()`, `robot_speed_mps()`, `robot_distance_m()`, `robot_heading_deg()`, `enc_l_count()`, `enc_r_count()` | estimates (FR4), safe from any task |

Faults (motors braked, latched until `motion_reset()`): stall / no encoder
progress, stale velocity command, wheel turning against its command,
drive/turn timeout. A broken encoder wire shows up as a stall. A stop or a
fault in the same period as completion wins over the completion, and a stop
never clears a latched fault.

## Encoder decoding

The encoders are decoded in **PIO0**, one state machine per wheel (SM0 left on
GP16/GP17, SM1 right on GP4/GP5), with the quadrature program from
pico-examples. The PIO keeps the count; the CPU takes no interrupt per edge and
reads the latest count from the RX FIFO. Subsystem 2 therefore does not use the
shared IO_BANK0 GPIO interrupt. PIO1 is left to the CYW43 WiFi driver.
This replaces the "Wheel encoder GPIO ISR" of the Week 6 design (sections 2
and 4), following the instructor's review.

Edge rate (counts are quadrature edges, 4 per encoder line):

| | Value |
| --- | --- |
| Formula | edges/s = wheel speed ÷ (π × 0.061 m) × `ENCODER_COUNTS_PER_REV` |
| Worst case | motor at no load (wheels lifted): 117 RPM + 10 % = 129 RPM = 2.15 turns/s |
| Per wheel | 2.15 × 2000 = **4.3 k edges/s** (one every ~233 µs) |
| Both wheels | **8.6 k edges/s** (one every ~116 µs on average) |
| PIO limit | ≤ 10 cycles per loop at 125 MHz = 12.5 M edges/s per wheel (~2900× margin) |

With a GPIO interrupt per edge the same rate would mean ~8.6 k interrupts/s,
and any interrupt latency longer than the gap between two edges of one wheel
loses a step. The PIO samples every few cycles, so latency cannot lose steps.

## RTOS template lifecycle

This folder is subsystem 2: Motion control using motors and wheel encoders.
`subsystem2_init()` and `subsystem2_start()` create the kernel objects and
start the motion task described in *How it runs*.

Keep hardware drivers in this folder and register ownership/interfaces in
`TEAM.md`. Shared configuration is in `app_program/system_config.h`; the
top-level `include/` directory now belongs to the kernel and board support.

The test hook (`subsystem2_test.c`, `make TEST=1`) runs unit tests S2-U01 to
S2-U20. Record evidence in `testing/subsystem2/TEST_LOG.md`.
