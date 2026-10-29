# Buddy 1 Tribunal 1 validation

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

- `subsystem1_test.c`: S1-B01 through B37 (40 checks including four FIFO entries).
- `host_test.c`: all 1,021 signed speed inputs, every heading/state code,
  all 256 barcode body codes, all 64 fault masks through live/snapshot/heartbeat,
  eight simultaneous producers, exact topic/length bridge,
  fragmented binary commands, zero bytes, oversized/short fragments, Wi-Fi bit
  preservation and heartbeat payloads. Versions 1/2 are explicitly rejected. The real `mqtt_utk.c` callbacks run.
- `test_laptop_codec.py`: independently decode the C-emitted packets on the
  laptop, plus malformed-packet, valid-zero, IR-bit/padding and all-fault tests.
- `size_report.c`: before/after slot, buffer and payload measurements. The host
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

## Physical checks before Week 13

1. Update robot and laptop to protocol v3 together. Subscribe on the laptop, run each producer and verify decoded values, units,
   direction and validity against the robot's known inputs.
2. Send raw binary map and snapshot commands. Confirm map changes are accepted
   before first run and refused afterward; the robot remains autonomous.
3. Disconnect Wi-Fi and the broker separately. Confirm autonomous control keeps
   running, reconnect/heartbeat recovery works, and the Wi-Fi bit preserves any
   concurrent motor fault. Measure recovery time; it is not yet a proven target.
4. Generate burst traffic from multiple buddies. Verify full-buffer rejection,
   latest snapshot behaviour and dropped reports; agree event-loss tolerance.
5. Record motion task start times and execution time during maximum telemetry
   and reconnect activity. Compare with its 10 ms period and agreed jitter limit.
6. Measure actual stack use/remaining heap under the combined workload. Keep
   runtime buffer allocation distinct from `.data + .bss` linker accounting.
7. Physically validate the adopted Buddy 4/5 ranges and confirm the wheel-speed range against
   calibration. Update versioned layouts and repeat cross-decoder checks if changed.

Record evidence, not only a pass assertion. Build/host results and pending items
are tracked in [EVIDENCE.md](EVIDENCE.md).
