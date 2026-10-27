# Buddy 1 Wi-Fi commands and telemetry

Buddy 1 now uses compact binary application data for Tribunal 1 (protocol v2).
Robot and laptop must be updated together: v1 packets/commands are rejected.
The shared fault mask now includes aggregate IR fault at bit 5; Buddy 3 keeps
its separate per-sensor fault mask. Snapshot/heartbeat byte lengths are unchanged. Other buddies
continue to call the typed `buddy1_report_*()` functions with normal values.

```text
Reporting task -> validate and pack -> protected packed latest-value cache
                                  -> four-slot RTOS outbox -> network task -> MQTT -> laptop decoder
Laptop binary setup/request -> network callback -> two-slot command buffer -> command task
```

The cache remembers the latest report independently of whether a live message
fits the outbox. A snapshot request returns a protected copy. A full outbox
rejects the new live message immediately; the reporting task never waits for
MQTT. The cache mutex may cause a short wait, and the cache values are not a
simultaneous sample of every sensor.

## Files to learn in order

1. `telemetry_codec.h`: message kinds, fault bits and decoded data views.
2. `telemetry_codec.c`: masks/shifts, field validation, pack/unpack and maps.
3. `subsystem1.h/.c`: reporting API, packed cache, mutex and RTOS message buffers.
4. `command.c`: binary setup/snapshot commands and pre-run map freeze.
5. `mqtt_bridge.c`: topic IDs to readable topics, explicit byte lengths, binary
   heartbeat and Wi-Fi fault observation hooks.
6. `MQTT_PROTOCOL.md`: the robot/laptop contract and known limitations.
7. `DATA_REPRESENTATION.md`: fields, ranges, resolution, bit counts and owner decisions.
8. `subsystem1_test.c`: portable checks also compiled into `TEST=1` firmware.

The existing platform files `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c/.h`
preserve binary input, bound staging to 16 bytes and publish explicit lengths.
The small CYW43 hook reports Wi-Fi link transitions after releasing the radio
lock. The networking task continues to own lwIP and MQTT. The Vehicle Controller
freezes barcode configuration before requesting the first run.

## Independent testing

Run from the repository root:

```sh
make -C testing/subsystem1 test
make -C testing/subsystem1 sizes
make -C testing/subsystem1 sanitize
```

These run without a Pico, network or teammates' hardware. They use the actual
Buddy 1 implementation and MQTT service with small host substitutes for RTOS
and lwIP calls. They do not prove real kernel scheduling or radio timing.
See `testing/subsystem1/README.md` for target and hardware checks.

## Integration limits

- Motion telemetry uses the proposed ±510 mm/s range in 2 mm/s steps. Odd input
  rounds by at most 1 mm/s. Buddy 2 control calculations are unaffected.
- Buddy 4/5 ranges are provisional; their placeholder modules have no hardware
  evidence yet. Existing unsigned precision is preserved until owners agree.
- Wi-Fi, motor and IR fault reporting use their owner adapters. Buddy 4/5
  fault sources still need integration. A cleared aggregate IR bit does not
  identify individual channels or certify physical wiring.
- Network outage and full buffers can lose live reports; a snapshot is latest
  state, not a retry/event history. Safety decisions must not depend on MQTT.
- Command priority 22 does not lower the kernel radio task's priority 4. Motion
  jitter, reconnect recovery, real stack usage and full-load behaviour still
  require testing on the robot.
- A decoded configuration map is frozen before first run and stays frozen until
  reboot. The laptop can still request snapshots during autonomous operation.
