# Buddy 1 — WiFi Communication, Command and Telemetry

Implemented communications subsystem. Matching public headers live under
`include/buddy1/`; shared message vocabulary and frozen run configuration
live under `include/shared/` and `src/shared/`.

| Module | Responsibility |
| --- | --- |
| `communication` | Public façade and non-blocking coordination |
| `wifi_connection` | Pico W association, link monitoring and reconnect backoff |
| `mqtt_client` | MQTT connection, topics, bounded command buffering and QoS 1 replies |
| `telemetry` | JSON payloads, event FIFO, periodic latest values and snapshots |
| `command_receiver` | Pure parsing and validation of permitted MQTT commands |
| `telemetry_port_pico` | Pico clock and the future μT-Kernel locking seam |

The laptop configures the barcode mapping before the run and observes
telemetry. MQTT does not command motors or choose mission behavior. During an
autonomous run, navigation must continue if WiFi, the broker or the laptop is
lost.

`communication_process()` is non-blocking and is intended for the eventual
lowest-priority network task at roughly 10 ms intervals. The exact task and
delay APIs must come from the lecturer's μT-Kernel starter port rather than
being invented here.

Host-test instructions are in `tests/buddy1/README.md`. The complete wire
contract is in `docs/buddy1/MQTT_PROTOCOL.md`.
