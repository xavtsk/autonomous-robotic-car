# Teammate update integration — 25 October 2026

## Result and recovery point

Working branch: `integration/buddies-1-3`, advanced from `477230c` to
`71193e2` (`origin/buddy3`, includes `origin/Buddy2` at `a29f973`). The original
pull fetched these commits but had not advanced this working checkout. The
fast-forward is complete; local application/integration changes are reapplied
and adapted. No commit, push, Pico flash or motor operation was performed here.

The pre-integration tracked and untracked work is retained in stash
`cfde7c4c7d4631f184d7e76c305a69eca2fd57ab`. Do not blindly apply it again: it
contains the superseded motion/controller API. Use `git show` to recover a
particular old file if necessary. Its old untracked controller README collided
with the incoming tracked README; the current README now documents the merged
architecture, and the old copy remains in the stash's third parent.

## What was adopted and preserved

- Adopted teammate Buddy 2 motion, PID, calibration, PWM reset release, PIO
  encoder driver, packed commands/status/reports and saved physical test data.
  `motion.c/.h`, `motion_controller.c/.h`, `motor_control.c`, `wheel_encoder.c`,
  `motion_estimation.c` and `speed_pid.c` match imported HEAD. Only optional
  timing hooks were added to the motion task wrapper; they do not set targets.
- Adopted the new coherent source-timestamp/epoch feedback, eight-report queue,
  matching command IDs, stopped acknowledgement and opt-in 30 ms owner lease.
  These changes already exist in the teammate revision; they are not replaced
  by the old local representation-only motion code.
- Adopted Buddy 3 calibration/start/readiness gates, persistent faults,
  odometry-aware barcode rearming and the controller drive policy. Its pure
  navigation algorithm and `s3_drive.c` match upstream. Leaf storage packing
  and boolean/small-mask packing were retained without modifying calculations.
- Kept Buddy 1's bounded binary-v2 telemetry/command buffers, decoder and fault
  ownership. Added atomic freeze-and-copy of the mapping at explicit START.
- Kept isolated IR bench, USB diagnostics, laptop log capture and one-shot
  movement test. Movement tests do not start because of a build or USB attach.

## Single-owner integration decisions

The opt-in `S3_MOTION=1` profile starts Buddy 1, Buddy 2 and the two controller/
sampling/report tasks from the teammate. It stays disarmed. `vehicle_controller`
is now a facade, not a second task: it queues START/STOP and mirrors telemetry.
Only `s3_drive` requests ordinary motion and consumes completion reports;
only Buddy 2 writes motor outputs. `S3_MOTION=0` leaves navigation disabled.
`MOTION_SMOKE=1`, `IR_BENCH=1` and navigation builds are mutually exclusive.

Buddy 1's A=forward/B=left/C=right/D=U-turn default (or validated replacement)
is authoritative at team START. Exactly four distinct A-Z letters are accepted.
The sampling owner atomically freezes/copies the map; its navigation context is
never modified directly by the network task. Calibration/start controls still
need an operator-facing adapter; the existing USB console is output-only, and
GP20/GP21 start/stop only the isolated smoke image. Do not expect the normal
integrated image to start by pressing those buttons.

Shared runtime data uses explicit codecs under the existing mutex:

| Retained copied data | Size | Important preserved information |
|---|---:|---|
| Navigation output | 479 bits / 60 bytes | All new readiness/fault/lifecycle/diagnostic/capture fields; exact float and double bits |
| Runtime status | 122 bytes | Navigation 60 + feedback 22 + metadata 40; exact signed odometry, epoch, source time, report loss and calibration summaries |
| Pending operator control | 83 bits / 11 bytes | Kind 3, sensor 2, 26 map entries × 3; STOP precedence retained |
| Stationary bench raw frame / last barcode | 34 / 5 bytes | ADC detail/validity/signed errors/timestamps and barcode direction |

The old 30-byte navigation packet omitted newly added mandatory fields, so it
was extended rather than silently dropping them. Public/working views stay
wide to validate before narrowing. This is not a new MQTT wire version.

### Important compatibility limits

- Buddy 2 status covers +/-1022 mm/s; its measured configured top speed is
  0.884 m/s. Buddy 1 wire v2 covers only +/-510 mm/s. The facade and legacy
  diagnostic adapter mark motion unavailable above that range or during
  calibration/invalid feedback. No wrap, clipping or hidden protocol widening.
  Agree and implement a versioned wider telemetry format if that speed range
  is needed; until then high-speed motion telemetry is incomplete.
- PIO0 SM0/SM1 and instructions 0..23 belong to the encoder driver. The local
  Pico SDK radio allocator searches PIO1 first; the kernel radio service starts
  before application motion. No additional PIO user is introduced. The encoder
  driver refuses already-running PIO0 state machines. Preserve this allocation
  when integrating other modules; raw-register ownership is not SDK-claimed.
- PIO invalid-transition counts are unavailable. Diagnostics explicitly print
  `glitches_available=0`; zero counters cannot validate signal quality.
- The 20-bit encoder/run bound is the teammate's declared limit, not a new
  on-car measurement by this integration. Do not infer correctness of physical
  distance/speed or direction solely from passing software tests.

## Software verification

Reproducible from the repository root:

```sh
make -C testing/subsystem1 test sanitize
make -C testing/subsystem2 test adapter regression sizes sanitize
make -C testing/subsystem3 test codec drive feedback regression sizes sanitize
make -C testing/integration -f controller_test.mk test sanitize
make -C testing/diagnostics -f firmware_test.mk test smoke disabled
python3 -m unittest discover -s testing/diagnostics -p test_capture_robot_log.py
```

Current checks cover:

- Buddy 1 C checks, independent Python decoder, concurrent producers, queue
  limits, MQTT callbacks, owned fault bits and freeze/copy of the live map.
- Current motion pack boundaries (every integer mm/s in +/-1022), report IDs,
  rejected out-of-range/nonfinite commands, legacy speed invalidation;
  production feedback timestamps/epochs, modular count wrap, report loss,
  cancellation/calibration exclusion and watchdog behaviour.
- Real Buddy 2 controller/PID plus Buddy 3 bridge with fake hardware; exact
  regression against imported `71193e2` across 58,915 navigation/barcode trace
  rows and all 65,536 ADC inputs, plus stationary bench diagnostic parity.
- Full new runtime/status/control codecs, boundary/padding rejection, hostile
  packets and exact float/double preservation; no second motion owner;
  production runtime STOP priority, bounded pending request, calibration/start
  gates, collection/status publication and signed ADC errors outside locks.
- Isolated smoke cancellation/deadline/fault behaviour and diagnostics
  publication/observer boundaries. Sanitizers check memory/undefined behaviour,
  not real-time scheduling or physical safety latency.

The earlier IRQ-only Buddy 2 harness and pre-pull size reports are explicitly
historical. The default current tests do not pretend they apply to the PIO code.
The teammate's existing hardware logs are preserved with their original
provenance; they are not newly measured results on this assembled car.

## Firmware build checks

Using the local Arm compiler and `PICO_SDK_PATH`, with full profile flags:

| Profile | Build result |
|---|---|
| Combined Buddies 1-3, offline, diagnostics, disarmed | Passed |
| Combined Buddies 1-3, Wi-Fi/lwIP/MQTT, diagnostics | Compile/link passed with public host-test placeholders only; NOT deployment credentials |
| Stationary IR, GP26 only | Passed |
| Stationary IR, all three channels | Passed |
| Offline on-target self-test image (not executed) | Passed |
| Isolated one-shot motion image | Passed |

Network compile-only files have `_COMPILE_ONLY` in their filename. **Do not
flash them.** No `config/wifi_secrets.h` was created or real credentials read.
Before deployment configure the real local ignored secrets file and rebuild
the complete network profile. Final build objects are from the offline smoke
profile, so a future network-profile build invalidates the dummy-config objects.

The combined offline ELF has text/data/BSS 117064/92/24092 bytes. Network
compile-only has 384928/152/84024. These are linker figures, not heap-free or
task-stack measurements: runtime allocations and high-water marks still need
measuring. Console/codec observation overhead is included only when enabled.

## Remaining validation / next step

First repeat the isolated smoke test with wheels raised and the team's verified
power setup. The earlier failed attempt predated the imported PWM-reset fix;
the new build has not been flashed or tried here. Do not declare that fault
resolved physically until the wheels actually move and stop correctly.

Then validate actual encoder direction/scaling and physical stopping, all three
IR calibrations and moving barcode/junction handling. Connect an explicit
operator calibration/start path before attempting autonomous integration.
Measure the 10 ms control/sampling cadence, owner-lease behaviour under console/
network/reconnect load and heap/stack usage. Reconsider the 5 ms Week 6 target
only with recognition/timing evidence. Network priority remains 4, motion 14,
sampling 16; no priority or PID tuning was silently applied.

Buddy 4/5 integration, full fault arbitration and correct-original-line identity
remain out of scope. Stable-line reacquisition is not proof of original route.
