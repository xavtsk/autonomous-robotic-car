# Buddy 1 host tests

These tests cover the pure C99 parts of communications: JSON telemetry,
bounded queues, topic routing, barcode-map parsing, structured replies and
the shared configuration lifecycle. No Pico, WiFi or MQTT broker is needed.

Run from the repository root:

```bash
make -C tests/buddy1 test
make -C tests/buddy1 sanitize
make -C tests/buddy1 analyze
```

Hardware-facing WiFi/MQTT behavior still requires the Pico cross-build and a
physical broker test described in `docs/buddy1/README.md`.
