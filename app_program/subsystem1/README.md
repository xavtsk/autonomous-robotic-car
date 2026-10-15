# Buddy 1 — WiFi Communication, Command and Telemetry

Starter modules only; no functionality is implemented.
Matching headers are in this folder.

| Source / header basename | Purpose |
| --- | --- |
| `communication` | Public communication subsystem interface. |
| `wifi_connection` | WiFi connectivity and connection recovery. |
| `mqtt_client` | MQTT session and messaging management. |
| `telemetry` | Telemetry publishing, heartbeat, and status reporting. |
| `command_receiver` | Command subscription and reception. |

Only `communication.h` declares the proposed subsystem lifecycle API:
`communication_init(void)` and `communication_process(void)`.
These functions have no definitions yet and cannot be linked until implemented.
The remaining headers contain interface TODOs.

Agree on data types, units, errors, timing, and ownership before implementation.
Route cross-subsystem requests and observations through the future integration
layer in `app_program/app_main.c`; do not call other Buddies' modules directly.
Define micro T-Kernel task, queue, and synchronization contracts during integration.
Keep private implementation details within this source directory and follow the
Barr C Coding Standard as development proceeds.

## RTOS template lifecycle

This folder is now subsystem 1: WiFi communication, command and telemetry.
The original module files above remain placeholders. The separate
`subsystem1_init()` and `subsystem1_start()` hooks currently return `E_OK`
without creating objects, starting tasks, or configuring hardware.

Implement these hooks when this Buddy's module interfaces are ready. Keep
hardware drivers in this folder and register ownership/interfaces in `TEAM.md`.
Shared configuration is in `app_program/system_config.h`; the top-level
`include/` directory now belongs to the kernel and board support.

The test hook reports SKIP until real tests are written. Record evidence in
`testing/subsystem1/TEST_LOG.md`.
