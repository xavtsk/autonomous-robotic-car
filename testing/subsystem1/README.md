# Buddy 1 Tribunal 1 validation

> Current guide: [documentation index](../../docs/README.md), [baseline status](../../docs/CURRENT_STATUS.md), and [build/test commands](../../docs/BUILD_AND_TEST.md). Dated results below retain their original evidence limits.

## Host checks

```sh
make -C testing/subsystem1 test
make -C testing/subsystem1 sizes
make -C testing/subsystem1 sanitize
```

Run these from the repository root. `test` compiles the real codec, reporting,
command, bridge and MQTT service sources. Host substitutes model copied bounded
message buffers and mutexes; no hardware or broker is used. Python requires only
the standard library; override `PYTHON=/path/to/python3` if needed.

- `buddy1_test.c`: S1-B01 through B37 (40 checks including four FIFO entries).
- `host_main.c`: all 1,021 signed speed inputs, every heading/state code,
  all 256 barcode body codes, all 64 fault masks through live/snapshot/heartbeat,
  eight simultaneous producers, exact topic/length bridge,
  fragmented binary commands, zero bytes, oversized/short fragments, Wi-Fi bit
  preservation and heartbeat payloads. Versions 1/2 are explicitly rejected. The real `mqtt_utk.c` callbacks run.
- `test_laptop_codec.py`: independently decode the C-emitted packets on the
  laptop, plus malformed-packet, valid-zero, IR-bit/padding and all-fault tests.
- `size_report_main.c`: before/after slot, buffer and payload measurements. The host
  size target uses short enums to match this Arm toolchain's ABI; wire tests also
  run with ordinary host enums to verify layout independence.

`sanitize` checks the C test run with address and undefined-behaviour checks.
It does not model the full kernel or DMA/interrupt scheduling.

## Pico firmware build

Use your own SDK path and private Wi-Fi configuration as described in the root
README. Enable the networking profile explicitly:

```sh
make -C build_make PICO_SDK_PATH=/path/to/pico-sdk WIFI=cyw43 NET=lwip MQTT=1 TEST=0 -j8
make -C build_make PICO_SDK_PATH=/path/to/pico-sdk WIFI=cyw43 NET=lwip MQTT=1 TEST=1 -j8
```

`TEST=1` includes all subsystem tests, some of which may move the motors.
Use the appropriate stationary/safe test setup. Buddy 1's test also closes its
configuration map latch, so reboot into `TEST=0` before a normal setup/demo.
Compiling the test firmware is not evidence the tests ran on the Pico.

## Physical evidence

The host suite exercises binary messages, map locking, bounded queues and
reconnection logic. Hardware measurements of end-to-end recovery time, task
jitter under network load and stack or heap high-water use were outside these
tests. The [dated report](EVIDENCE.md) gives the measurements for each protocol
version. The [submission results](../../docs/CHANGE_REQUIREMENT_1_EVIDENCE.md)
record the final software comparison.

The Week 13 procedure compared decoded units and validity with known inputs,
tested pre-run map locking, and interrupted Wi-Fi and the broker separately.
It specified burst telemetry, dropped-report counts, concurrent faults and
motion intervals against the 10 ms period. These are the conditions for
interpreting any on-car measurements.
