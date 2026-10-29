# Buddy 1–5 integration and protocol v3 handoff — 28 October 2026

Software steps 1–5 are implemented and host/build checked. No robot was flashed,
serial port opened or real robot broker contacted. Hardware step 6 remains open.
No new development commit or push was made.

## Branch and recoverable backup

- Branch: `integration/buddies-1-3`, fast-forwarded from `549580d` to the bundle's
  existing `feature/dashboard` commit `8d512da`. No merge commit was created.
- Before integration, all 45 dashboard WIP files were byte-compared with the
  bundle's `d155e71` snapshot: identical. They were not applied a second time.
- Recovery stash: `b8e38c17ac42a4d87102a0a2a6479cd7e4be84b6`, named
  `Before Buddy 1–5 bundle integration 2026-10-28`. **Do not apply it again**:
  its code is already in the bundle. It is retained only as recovery evidence.
- Changes after `8d512da` are uncommitted. Private Wi-Fi settings were preserved,
  not displayed or copied into the isolated validation checkout.

## Changes and boundaries

1. Controller diagnostics now accept/show terrain/IMU fault reason 10; 11–15 are
   rejected. Real-C vectors are checked independently by the Python decoder.
2. Coordinated **MQTT protocol v3**: peak height 9 bits (0–500 mm), obstacle
   distance 12 bits (30–4000 mm), obstacle width 12 bits (0–4095 mm). Count remains
   16 bits for Buddy 4's saturating range. Out-of-range data is rejected before
   cache update; no clipping, wrapping into a valid value or accuracy downgrade.
3. Robot and laptop reject protocol v1/v2 rather than guessing. Map, snapshot,
   heartbeat, simulator, route adapter, terminal viewer and dashboard use v3.
   Restart the Python service as well as refreshing the browser.
4. Wireless barcode recordings use binary BCAP v1 packets, not ASCII. USB text,
   old `.txt` imports, plots and readable downloads remain supported. Missing,
   duplicate and conflicting records are tested. Capture identity/episode/index
   prevent ordinary interleaved or incomplete transfers becoming invented data.
5. The telemetry adapter reuses one coherent Buddy 2 status read per control
   cycle, instead of the previous two reads plus a third every 20 cycles.
   Private divider/pose/status flags are compact. Already-invalid hump/obstacle
   caches are not repeatedly invalidated every 10 ms; failed invalidations retry.
6. Network barcode capture reads use `TMO_POL`, not a blocking mutex wait. The
   binary cursor advances only on a complete packet; staging is bounded to 47
   bytes normally / 70 with optional live diagnostics, rather than 192.

**Unchanged from the imported bundle:** Buddy 2 motor/PID code, Buddy 3 sensing,
line/barcode algorithms, `s3_drive.c` navigation/avoidance behaviour, calibration,
START/STOP policies, task priorities and control periods. No new RTOS task,
mutex, queue, incoming motor command or peripheral owner was added. Buddy 4/5
imports keep their existing hardware gates and physical-validation caveats.

## Message and allocation measurements

| Item | Bundle v2 | Current v3 |
|---|---:|---:|
| Hump body meaningful bits | 32 | 25 |
| Hump MQTT bytes, including version | 5 | 5 |
| Obstacle body meaningful bits | 34 | 26 |
| Obstacle MQTT bytes, including version | 6 | 5 |
| Snapshot meaningful body bits | 117 | 102 |
| Snapshot MQTT/cache bytes | 16 | 14 |
| Application outbox slot bytes | 17 | 15 |
| Four-slot outbox requested storage, including headers/alignment | 96 | 80 |
| Two-command buffer requested storage | 24 | 24 |
| Combined requested message-buffer storage | 120 | 104 |

The hump's seven saved bits become canonical byte padding: no extra whole-byte
RAM saving is claimed for that standalone packet. Physical queue capacity and
kernel allocation metadata are not application fields. Runtime allocation is
separate from linked static RAM, so do not simply call the table a heap peak.

## Matched target builds

Arm GNU Toolchain 15.3.Rel1 / GCC 15.3.1, Pico SDK at the existing local path.
Before = original `8d512da`; after = current source. Both matched all-five-buddy
builds used the public example network configuration, USB console, `CRASH_REPORT=0`,
`WIFI=cyw43 NET=lwip MQTT=1 TEST=0 S3_MOTION=1 S4=1 S5=1 S5_HARDWARE_CONFIRMED=0`.
The unsafe/unconfirmed hardware gate was **not** bypassed. These images compile
the code but cannot establish all-five-buddy runtime readiness.

| Profile | Version | Linked text/constants | Data | BSS | Data+BSS |
|---|---|---:|---:|---:|---:|
| `IR_DEBUG=0` | Before | 419016 | 672 | 102924 | 103596 |
| `IR_DEBUG=0` | After | 421168 | 672 | 102772 | 103444 |
| `IR_DEBUG=1` | Before | 422952 | 672 | 103276 | 103948 |
| `IR_DEBUG=1` | After | 425120 | 672 | 103140 | 103812 |

Thus static RAM decreases **152 bytes** normally / **136 bytes** with previews;
code/constants increase **2152 / 2168 bytes** for binary capture and validation.
The fixed barcode capture buffer remains 17336 bytes; it was not shrunk or
relocated. These are whole-image linker figures, **not** measured free heap,
stack headroom, CPU load, radio airtime or STOP latency.

Additional successful builds:

- Offline Buddy 1–3 profile: text 128736, data 92, BSS 41964.
- Main checkout, existing private network config, Buddy 1–3 + `IR_DEBUG=1`:
  text 403064, data 152, BSS 102424. **Do not compare this text size directly
  with the public-config/all-five profile.**
- All four profiles built without compiler warnings/errors. Compile-only
  validation images in temporary folders are not demo firmware.

## Software verification

All completed successfully:

- Buddy 1 real codec/cache/queue/command/MQTT tests, ordinary and live-debug
  builds; exhaustive 9-bit height codes, obstacle boundaries, padding, v2
  rejection, no invalid cache update, full bounded queues and reconnect retries.
- Controller calibration, mapping freeze, START/STOP, reset, capture, latest-only
  previews, course and turn/reacquisition simulator tests.
- Buddy 2 packing/motion-feedback/drive tests and Buddy 3 core/mapping/reflectance/
  navigation codec tests. Source comparison confirms no algorithm changes.
- Buddy 4 sensing/runtime/controller and Buddy 5 sensing/controller/combined-S4/
  hardware-register-fake/activation-gate/telemetry tests.
- AddressSanitizer and UBSan for Buddy 1, controller, Buddy 4 and Buddy 5 suites.
- **58 dashboard tests**, including independent real-C/Python packet equality,
  malformed/lost/duplicate/interleaved capture cases, readable export and an
  isolated loopback Mosquitto transfer. No real broker was contacted.
- Wi-Fi robustness policy/runtime tests, diagnostic/startup/motor-smoke host
  tests, **25 logger tests**, JavaScript syntax checks and `git diff --check`.
- Isolated browser demo checked overview/route, sensor graphs and Buddy debug
  rendering, with live v3 demo packets and zero malformed reports. Existing
  dashboard port 8765 was not stopped or replaced.
- Original bundle/new course-simulator output matches byte-for-byte. SHA-256:
  `a9ae1540abb40e6adca56f3c0d40912c7956934d535a347126ad051da7180fdb`.

Logs and reproducible size output: `testing/subsystem1/logs/2026-10-28-v3/`.
Host tests do not model real ADC timing, interrupts, RF loss, wheel slip or stops.

## Ready for your hardware step 6

The built **Buddy 1–3 wireless + live-graph** image in the real checkout is:

`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_s3_motion_ir_debug.uf2`

SHA-256: `2e8717081367e0ff75db1fd4f0bdf2ecccee4676a82cf7c40169bc9fa876312b`.
It was built using the existing private network configuration without displaying
or changing it. Nothing was copied to the Pico. It intentionally has `S4=0 S5=0`.
Do not use an `_UNCONFIGURED` or placeholder-credentials validation artifact.

Rebuild command from the repository root:

```sh
export PICO_SDK_PATH="/Users/zhaobao/pico/pico-sdk"
make -C build_make CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=cyw43 NET=lwip MQTT=1 TEST=0 IR_BENCH=0 S3_MOTION=1 S4=0 S5=0 DIAGNOSTICS=0 MOTION_SMOKE=0 IR_DEBUG=1 -j8
```

After your normal supervised flash/setup, restart the old dashboard Python
process (Control+C in its terminal, not merely browser reload), then:

```sh
python3 tools/robot_dashboard.py --port 8765
```

Use real mode, not `--demo`; connect to your existing broker. Secure/raise the
wheels for the first START/STOP check, follow the existing `[OP]` instructions
and keep the independent power cutoff accessible. Do not change wiring powered.

1. Stationary: heartbeat, v3 snapshot, map acceptance before START; live sensor
   values/validity and physical white/black calibration. Confirm old v2 packets
   are rejected rather than shown as correct measurements.
2. Supervised short run: GP21 software stop, line/barcode actions, route sketch,
   captured trace and readable download. Repeat START and verify map stays frozen.
3. Repeat comparable runs with `IR_DEBUG=0` and `IR_DEBUG=1`. Record USB `RUN timing`
   (`max_gap_us`, `above10ms`, `over30ms`), source timestamps/gaps and physical STOP
   response. Startup-only maxima are not run-only jitter evidence. `DIAGNOSTICS=1`
   can add motion/IR execution summaries in a separate validation build; account
   for its own instrumentation overhead.
4. Test hotspot/broker loss and reconnect separately; preserve failures/partial
   captures. Verify autonomy/stop behaviour does not depend on MQTT delivery.
5. Enable Buddy 4 and then Buddy 5 **only after** their physical setup and
   software gates are satisfied, using their integration guides. Do not blindly
   set `S5_HARDWARE_CONFIRMED=1` to get past a refusal. Revalidate full-load timing.
6. Actual task stack/heap minima and MCU outbox drop/high-water counts are **not
   currently exposed by this dashboard**. Obtain separate instrumented/debugger
   evidence before signing these off; laptop receive counts/linker sizes cannot
   substitute. The network task still has priority 4 above motion 14, unchanged.

Save exact firmware hash/build flags, logs and videos. Physical hump accuracy,
scan geometry/clearance, original-line recovery, stack/heap/queue headroom and
worst-case timing remain **open**, not inferred from software PASS results.
