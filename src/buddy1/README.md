# Buddy 1 — WiFi Communication, Command and Telemetry

Starter modules only; no functionality is implemented.
Matching headers are in `include/buddy1/`.

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
layer in `src/main.c`; do not call other Buddies' modules directly.
Define micro T-Kernel task, queue, and synchronization contracts during integration.
Keep private implementation details within this source directory and follow the
Barr C Coding Standard as development proceeds.
