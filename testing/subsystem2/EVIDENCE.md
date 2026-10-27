# Buddy 2 minimum-bit change evidence

**Historical results below:** measured before the 25 October upstream PIO,
report-queue and calibration changes. They do not certify the current code.
Current repeatable checks are `make test adapter regression sizes` in this
folder and the [25 October review](../../docs/INTEGRATION_2026-10-25.md).
`regression` now exports `71193e2` and runs production feedback/report/stop/
calibration/lease assertions against both copies. The old IRQ harness is kept
as `legacy-test`, not presented as a passing check of the new implementation.

Measured 2026-10-23 on `integration/buddies-1-3`. Changes are local and
uncommitted; no flash, broker connection or hardware motion test was performed.
Unrelated existing integration/Buddy 1/Buddy 3 work was preserved.

## Scope and review

The original request was representation only. `speed_pid.c/.h`,
`motion_estimation.c/.h`, `subsystem2.c/.h` and `wheel_encoder.h` are unchanged.
Calibration constants/calculations, PID/ramp/filter maths, PWM setup and output
decision logic, encoder lookup/edge acknowledgement, command acceptance,
completion consumption, fault ordering/thresholds and stop/reset rules were
reviewed against the saved pre-change sources. Only storage, accesses/decoding
and the reporting adapter were revised. The controller's navigation decisions
were not changed.

`app_program/subsystem2/DATA_REPRESENTATION.md` maps the requirement, Week 6,
project brief/write-up/transcript and Tribunal B decisions to the implementation.
It explicitly lists unresolved encoder-run, float-range and glitch-delta bounds.
These are not silently assumed to comply merely because existing widths remain.

## Correctness checks

Run from the repository root:

```
make -C testing/subsystem2 regression
make -C testing/subsystem2 adapter
make -C testing/subsystem2 sanitize
make -C testing/subsystem1 test
make -C testing/subsystem1 sanitize
```

`regression` exports Buddy 2 at revision
`816d7f020260d4772103fbc51fa983ea0e2a2bea` into a fresh temporary folder. That
subsystem was identical to the integrated branch before this update. It never
switches branches or resets working files. It compiles the same host harness
against before/after sources, using the real motion API, controller, PID,
estimation, encoder interrupt routine and motor driver with fake registers/kernel.

| Check | Result / limit |
|---|---|
| Original non-moving unit tests | 9 pass, 0 fail on host; uncalibrated-build branch and physical drive/turn tests skipped |
| Full before/after traces | 3754 rows byte-identical: states, primary faults, counts, exact float bits, PWM levels and register-write checksum |
| Manoeuvres | forward, backward, left, right, straight and U-turn; one-shot completion preserved |
| Faults | all five original reasons; stop does not clear latched fault; reset behaviour preserved |
| Interrupt decoding | all 16 previous/new A/B combinations with each inversion; counts and glitch detection match |
| Timing-domain cases | original dt clamp, stale expiry across 32-bit clock wrap, long still timer and velocity updates |
| Packed results | 36756 combinations: 1021 signed speeds x 6 states x 6 fault reasons |
| Protocol agreement | both wheel speeds match Buddy 1's actual codec for all 1021 integer mm/s inputs |
| Calibration flags | all 256 combinations preserve meanings and float fields |
| Invalid input/reporting | boundary overflow, invalid enum codes, reserved bits, negative zero, nonfinite estimates and invalid-versus-valid-zero |
| Snapshot effects | does not consume COMPLETE or change control state; busy/calibration data has invalid speed |
| Controller fault adapter | preserves Wi-Fi bit, suppresses duplicates, handles full outbox/error, never invokes motor commands |
| Memory/undefined behaviour checks | host AddressSanitizer and UndefinedBehaviorSanitizer pass |
| Existing Buddy 1 regression | C/Python codec, cache/concurrency and MQTT-service host tests pass |

The before/after trace SHA-256 is
`86fd4a6c4cb0e4e7507234f2c2b377522965198a83760e3a17c8d43e001281ae`.
Raw traces regenerate in `build/buddy2/`. They are deterministic host simulations,
not proof of physical stopping, calibration, interrupt throughput or RTOS WCET.

## RAM comparison

Measured with `arm-none-eabi-size` on before/after full firmware and
`arm-none-eabi-nm -S` on subsystem objects. Both full comparison images use
SMP=1, USB CDC, Wi-Fi CYW43, lwIP, MQTT and TEST=1. Temporary compile-only
settings came from the public `config/wifi_secrets.example.h`, not private
network credentials. The before image already includes the previous Buddy 1
tribunal update and existing integration work.

| Object | Before bytes | After bytes |
|---|---:|---:|
| Two wheel controllers | 96 | 88 |
| State/fault/busy/move-kind and two direction values combined | 12 | 2 |
| Still timer | 4 | 2 |
| Calibration control flags, now also including reporting-ready | 3 | 1 |
| Stored calibration result | 36 | 28 |
| Two motor objects | 32 | 16 |
| Two encoder objects | 40 | 24 |
| IRQ core | 4 | 1 |

These are symbol sizes; alignment and the new one-byte controller reporting
cache mean they must not simply be summed as linker RAM savings.

| Full TEST=1 image | Before bytes | After bytes |
|---|---:|---:|
| text / flash code | 366944 | 367744 |
| data | 196 | 160 |
| bss | 83152 | 83128 |
| Static RAM, data + bss | 83348 | 83288 |

Static RAM decreases by **60 bytes**, while code grows by 800 bytes. No new
task, mutex, queue or heap allocation was added. Stack reservations are unchanged;
smaller local/copy objects do not prove lower stack high-water usage.

## Message size comparison

- The newly defined copied motion result uses 26 meaningful bits in **4 bytes**.
  An equivalent decoded view with two floats, two short enums and busy/valid
  booleans needs 12 bytes with this Arm ABI. This is a like-for-like layout
  comparison, **not an old 12-byte queue that existed in the firmware**.
- Previously the controller fetched the two float speeds separately; it now
  uses the four-byte protected copy for reporting. No Buddy 2 queue was added
  or removed, so no queue-allocation saving is claimed.
- Buddy 1's MQTT motion packet was already **4 bytes including its version**
  before this Buddy 2 change and remains 4 bytes. Its magnitude/direction
  representation is unchanged. No additional MQTT byte saving is claimed here.
- The existing copied calibration result changes from **36 to 28 bytes**.
- Full-precision command function arguments remain unchanged; claiming a
  smaller command buffer would be incorrect because there is no such buffer.

## Build and outstanding validation

Both full Wi-Fi/lwIP/MQTT production and TEST=1 profiles compile/link. Host
fault-adapter and original Buddy 1 tests also pass. No generated image was flashed.

Still required: define/validate maximum run and encoder/glitch bounds; confirm
shared-speed reporting range/resolution; decide whether to extend the common
fault schema for the other three primary reasons; physically test calibration,
distance/turns and stopping; measure maximum edge load, ISR duration, control
jitter/WCET under networking/reconnection, stack and free-heap high-water usage.
Do not claim complete Week 13 minimum-bit or performance sign-off yet.
