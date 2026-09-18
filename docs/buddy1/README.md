# Buddy 1 documentation

- `MQTT_PROTOCOL.md` defines topics, QoS, JSON fields, the configuration
  transaction and autonomous-operation rules.
- `DESIGN_REVIEW_NOTES.md` records requirements, assumptions, risks and
  evidence for the Week 6 design review; rewrite it in your own words.
- `mosquitto-dev.conf` is an unauthenticated broker configuration for an
  isolated development hotspot only.

## Integration contract

1. The integrated application owns one `robot_config_t` and initializes it
   with `robot_config_init()`.
2. It passes that object to `communication_init()`.
3. A low-priority task calls `communication_process()` periodically.
4. Other buddies call `tele_report_*()`; those functions never perform
   network I/O.
5. The physical start flow advances the shared phase from `CONFIGURING` to
   `ARMED` and then `AUTONOMOUS`, freezing the accepted barcode map.

## Still awaiting integration

- Actual μT-Kernel task and telemetry locking adapter
- Team CMake/Pico SDK build configuration
- Physical Pico W, hotspot and broker test
- Real telemetry calls from Buddies 2–5
- Physical start/arm mechanism
