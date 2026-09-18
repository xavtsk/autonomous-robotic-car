# Shared mission-controller tests

The controller is pure C: it consumes typed events and produces typed
decisions without calling Pico, MQTT, GPIO or RTOS APIs.

Run from the repository root:

```bash
make -C tests/controller test
make -C tests/controller sanitize
make -C tests/controller analyze
```

Two characterisation groups intentionally record unresolved team decisions:
whether line loss may cancel a turn/hump, and whether Buddy 2 treats starting
line following as restoring normal speed. Resolve those contracts with the
team before changing their tests or controller behavior.
