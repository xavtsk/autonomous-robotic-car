# First robot test session: Buddies 1-3

Goal: keep the working baseline, record what actually happened, and change one
setting at a time. Software tests and replay logs are not hardware evidence.
Do not add Buddy 4/5 behaviour during this session.

## Before you have the robot

- Ask your teammates for their working UF2/build revision, wiring, calibration
  values and startup procedure. Flashing replaces the installed program.
- Keep the current baseline: networking priority 4, motion priority 14, IR
  priority 16, motion and IR periods 10 ms. The imported 25 October Buddy 2
  driver uses 20-bit modular PIO encoder counts (not the previous 32-bit IRQ
  counters); preserve its declared run bound until physically reviewed. Network
  priority 21 and IR sampling 5 ms are proposals, not changes already applied.
- Run the logger's tests and a replay without opening any serial port:

```sh
python3 -m unittest discover -s testing/diagnostics -p test_capture_robot_log.py
python3 tools/capture_robot_log.py --replay testing/subsystem1/PROTOCOL_V2_TEST_RESULTS.txt --output build/robot-logs/replay-01 --firmware host-test-replay --test logger-check --line-csv
```

The input above is an existing host-test log, not a robot recording. Replay is
marked `REPLAY_NOT_HARDWARE`. Use a new output directory each time; the logger
refuses to overwrite a previous recording. Live capture needs `pyserial` in
the Python environment; replay and core logger tests do not.

For a local isolated environment on macOS/Linux:

```sh
python3 -m venv build/robot-tools
source build/robot-tools/bin/activate
python -m pip install -r tools/requirements-robot-log.txt
```

This environment is already prepared on this Mac. Activate it before the
`python` commands below. On Windows, the equivalent activation is
`.\build\robot-tools\Scripts\Activate.ps1` after creating the environment.

## 1. Connect and preserve the baseline

Use the team's verified wiring and [USB/BOOTSEL guide](../../docs/UPSTREAM_README.md).
Initially keep motor power disconnected or use the team's safe stationary setup.
Disconnect power before changing wires. For IR wiring, use the existing
[sensor setup](../subsystem3/README.md), including the GP28 jumper check.

Identify the actual USB serial port; close other serial monitors. To list ports:

```sh
python tools/capture_robot_log.py --list-ports
```

If possible, capture the installed working firmware before replacing it. A log
does not back up firmware: retain your teammate's known-good UF2 and build details.
Opening a serial port can affect control lines on some systems, so secure the
robot first. The logger sends no commands and does not deliberately reset it.

## 2. Select the correct firmware

With `PICO_SDK_PATH` set to your local SDK, build from the repository root:

| Purpose | Build options after `make -C build_make` |
|---|---|
| Stationary IR only | `CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 TEST=0 IR_BENCH=1 DIAGNOSTICS=0 -j8` |
| Buddies 1-3 diagnostics and networking, disarmed with calibration/start API | `CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 TEST=0 IR_BENCH=0 S3_MOTION=1 DIAGNOSTICS=1 -j8` |

See [current integration review](../../docs/INTEGRATION_2026-10-25.md). The PIO
backend cannot count invalid edges: `glitches_available=0` means its zero glitch
values are **not** evidence of clean wiring. The full integration still requires
a user-facing calibration/start adapter; the smoke buttons only operate the
isolated smoke profile. Never flash the network compile-only placeholder build.

Pass the complete options on each build and record the resulting UF2 name.
The normal network diagnostic image ends in `_wifi_lwip_mqtt_diag.uf2`;
the sensor-only image ends in `_ir_bench.uf2`.
`DIAGNOSTICS=1` is opt-in, normal-mode only: it observes task intervals and
prints low-rate summaries; it does not start autonomous driving. Normal mode
enables all three IR sensors. IR bench starts no motors/controller and enables
only the GP26 barcode sensor by default (`IR_BENCH_ENABLED_MASK=1`).

For Wi-Fi/MQTT, privately fill `config/wifi_secrets.h` from the example before
building. Do not commit credentials or put them in test notes. Compile-only
images using example credentials will not connect to your real network. The
default build has `NET=none MQTT=0` and is not a Buddy 1 network test.

## 3. Save a recording, not just a screenshot

Replace `ACTUAL_PORT` and `YOUR_BUILD_ID`; do not create the output directory first:

```sh
python tools/capture_robot_log.py --port ACTUAL_PORT --output build/robot-logs/stationary-01 --firmware YOUR_BUILD_ID --test stationary-network --duration 120 --line-csv
```

The logger saves exact bytes in `console.log`, session information in
`metadata.json`, and optional host-arrival lines in `hostarrival.csv`. The
firmware label is your annotation, not automatic verification of the installed
image. Copy [the notes template](SESSION_NOTES_TEMPLATE.md) into the session
folder and fill it in. Keep failures, disconnects and incomplete runs too.
`complete` means the capture ended normally, not that the robot passed a test.
`running`, `error` or `interrupted` metadata identifies incomplete capture;
retain its partial evidence and record what happened.

Raw recordings under `build/robot-logs/` are gitignored. Keep a separate backup;
copy redacted conclusions/evidence links into the appropriate `testing/` log.

## 4. Test separately, then together

1. **Buddy 1, stationary:** use a real MQTT subscriber and sender. Robot and
   laptop must both use protocol v2. Check heartbeat, snapshot, valid/invalid
   pre-run mapping, Wi-Fi loss and broker loss separately. The Python codec
   encodes/decodes bytes; it is not itself a live MQTT client. Send binary bytes,
   not printed hex text. See [the protocol](../../app_program/subsystem1/MQTT_PROTOCOL.md).
2. **Buddy 3, stationary first:** record white/black ranges at final mounting
   height on actual track material. GP26-only bench results do not validate the
   two line sensors. Later test line, junction and moving-barcode recognition.
3. **Buddy 2:** reuse the teammate's calibration procedure and record its full
   output. `S2_TEST_CALIBRATE` and `S2_TEST_MOVE` are currently 0. Selecting
   calibration in a `TEST=1` build eventually runs motors at full power: follow
   console prompts with wheels safely lifted. Movement tests require clear
   floor space. Do not enable these casually for a stationary network session.
4. **Combined run, only when safe:** verify line following, barcode remembered
   until junction, one manoeuvre and telemetry. The integrated controller waits
   for `vehicle_controller_begin()`; USB connection/BOOTSEL is not the driving
   start input. Agree the team's start adapter first; diagnostics adds none.

`TEST=1` runs other subsystem tests too; Buddy 1's test closes the mapping latch.
Reboot into `TEST=0` before normal pre-run configuration. Do not interpret SKIP
as PASS or successful compilation as a test executed on the Pico.

## 5. What to send back for tuning

Send the raw recording, completed notes and independent measurements/video.
Use a ruler/protractor for actual distance/angle; encoder estimates alone cannot
prove physical accuracy. Record counts per revolution, speed, count changes and
elapsed time, including reverse movement and resets. Agree maximum continuous
operating time since counter initialization, including multiple runs, before
reducing encoder width. One observed maximum is not a guaranteed bound.

MCU start-gap/execution summaries help investigate load. Execution intervals
include preemption/blocking, not only CPU processing; they are not a proof of
worst-case CPU execution time. Instrumentation/reporting also adds workload.
Laptop arrival times include USB buffering and cannot substitute for MCU timing.
Compare equivalent conditions before accepting a new priority/sampling setting.
Real stack/heap high-water measurements and moving-network timing remain open;
the optional logger does not automatically supply them.

### Reading the optional diagnostic lines

- `timing` / `interval`: cumulative samples and MCU start-gap/elapsed maxima
  for each source. `gap_valid=0` means fewer than two measured starts.
- `publish_skips`: summary copies deferred because the diagnostic mutex was
  busy, not missed control cycles. The next published summary retains those
  intervening measurements. `last_end_us` helps identify stale summaries.
- `counts`: signed raw counts and cumulative invalid-transition counts.
- `estimate`: whole-mm/s estimator readings, not independent physical speed.
  Numeric-valid flags do not certify calibration; invalid numeric zeros are
  not measurements of a stationary wheel.
- `snapshot_speed_valid`: the existing reporting snapshot currently marks
  speed unavailable while motion is busy. This update leaves that behaviour
  unchanged; the separate `estimate` line is used for diagnostic observation.

USB output may already have been dropped in the firmware before it reaches
the laptop. `console.log` preserves the bytes received, not a guarantee that
every printed byte or every event reached the computer.
