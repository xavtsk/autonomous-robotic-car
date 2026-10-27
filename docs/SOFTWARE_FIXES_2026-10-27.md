# Software robustness fixes — 27 October 2026

## Outcome and scope

Applied to the existing uncommitted `integration/buddies-1-3` working tree,
HEAD `71193e22a8e8ce08f71c1ec94ff928bd84f3123f`. No flash, commit, push,
branch change, robot motion or real network connection was performed.
Existing unrelated changes were preserved. No credentials were read or printed.

These changes address the software findings from the 26 October hardware
session. They do **not** fix sensor geometry, prove the 0.06 m/s wheel floor,
retune the PID or establish a successful course run.
Buddy 2 and Buddy 3 subsystem C/header hashes match their pre-change values.

## What changed

### 1. Wi-Fi can recover from a join without an IP

`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` now uses the tested
`wifi_retry_policy.h`:

- An accepted join gets **20 seconds total** to obtain an IP. Transitions
  between DOWN/JOIN/NOIP do not restart the deadline.
- On timeout, the existing radio owner calls the official driver leave API
  under its lock. Retry delays are **5, 10, 20, then 30 seconds**, capped.
- Rejected initial join requests, terminal failures and DOWN after UP recover
  too. UP cancels a pending retry and resets the backoff.
- Normal 2 ms radio polling and lwIP/DHCP servicing continue while waiting;
  there are no timeout/backoff sleeps or new task/heap allocations.
- A genuine failed bus/interface initialization is still reported and parked;
  repeatedly retrying a broken hardware interface is not a safe repair.

Defaults are compile-time-overridable microsecond constants. The 20-second value
is an initial test setting, **not** a proven hotspot-recovery guarantee.
Driver join/leave operations themselves are synchronous; measured motion/STOP
latency is still required.

### 2. MQTT retains a temporarily rejected publish

`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` retains the exact queued
topic, binary bytes, QoS and retain flag in **one fixed 84-byte packet slot**.
With its small timing/flag state this uses about 96 bytes of static state;
the prior 84-byte stack temporary is removed. No queue/pool was enlarged.

- Only one application publish is in flight.
- Sends are spaced at least **20 ms**; temporary ERR_MEM/ERR_CONN failures
  are retried no earlier than **100 ms** later.
- A delayed packet cannot be overtaken by later FIFO items.
- The slot survives a disconnect; installed lwIP clears its own requests
  without completion callbacks, so the in-flight flag is cleared and the same
  packet can be retried on reconnect.
- Repeated temporary failures log once per retained packet rather than every
  radio poll. Invalid packets/permanent publish errors are explicitly dropped.

This is bounded-memory best effort, **not a lossless or exactly-once channel**.
The existing four-slot application outbox can still fill under sustained
backpressure. An ambiguous disconnect/timeout can duplicate a delivery.
QoS 0 does not prove broker receipt. Heartbeat remains best effort.
External MQTT protocol v2 and its laptop decoder are unchanged.

### 3. One fault-reset request waits for fresh confirmation

`app_program/controller/s3_drive.c/.h` records a pending reset after Buddy 2
accepts it. It retains FAULT until a newly timestamped, valid, fresh, IDLE,
braked, fault-free motion publication arrives and existing navigation
health/readiness checks pass. It fails after **500 ms** if those conditions
do not become true.

`s3_integration.c` acknowledges the command only after that outcome:
`command_sequence` changes once and `command_ok` reports the final result.
Other ordinary requests are blocked during the wait. STOP pre-empts/cancels it;
there is no later false success and no automatic START.

This removes the old first-press false refusal caused by checking pre-reset
feedback. It does not bypass a real jam, unsafe motor state or unhealthy sensor.
The operator's existing one-second response deadline accommodates this wait.

### 4. Timing now distinguishes boot from actual driving

The sampling period remains **10 ms** and the navigation safety limit remains
**30 ms**. No priorities or timing-related safety thresholds were changed.

The USB status now shows `boot_max`, `above10ms` and `target_ms=10`.
An above-target interval can be ordinary jitter; it is **not** labelled a
missed deadline.

The separate `RUN timing` line shows:

- accepted-START timestamp;
- run sample count;
- maximum run gap;
- above-10-ms count;
- above-30-ms count;
- raw microsecond timestamp of the latest above-30-ms gap.

Run counters reset only at an **accepted START**, include the interval that
caused a halt/fault, then freeze while inactive. Startup delays cannot pollute
the run maximum. Counters saturate rather than wrap; timestamps use the existing
uint32 microsecond clock. This observes long delays; it does not attribute or
eliminate them.

The internal shared status grows from **122 to 146 bytes**, exactly 24 bytes
for six full-range diagnostic fields, encoded explicitly under the same mutex.
Metadata is 509 bits/64 bytes with three padding bits. Navigation remains
60 bytes, motion feedback 22 bytes, control requests 11 bytes. MQTT packets
are unaffected. The 32-bit fields preserve microsecond timestamp/gap ranges
and diagnostic counts where no shorter maximum run has been agreed.

## Verification

Host checks use fake hardware boundaries; none connects to the robot:

```sh
make -C testing/network_wifi test sanitize
make -C testing/subsystem1 test sanitize
make -C testing/subsystem2 test sanitize
make -C testing/subsystem3 test bench codec drive feedback sanitize
make -C testing/integration -f controller_test.mk test sanitize
make -C testing/diagnostics -f firmware_test.mk test sanitize
```

All passed, including AddressSanitizer/UndefinedBehaviorSanitizer. Integration
contains seven test programs, including new run-timing tests. Coverage includes:
stuck JOIN/NOIP, capped backoff, late UP, failed join/leave, continued polling,
clock wrap, MQTT temporary rejection/deferred completion/reconnect/FIFO bytes,
one-request reset with real Buddy 2 controller logic and fake motor boundaries,
stale/unbraked/unhealthy reset rejection, STOP cancellation, timeout, timing
boundaries, saturation and packed-status round trips/malformed inputs.

Saved host logs are in `testing/integration/logs/2026-10-27_*.txt`.
Results are appended to `testing/integration/TEST_LOG.md`; earlier hardware/
test records are retained, not rewritten.

## Build and handoff

The combined Wi-Fi course image is:

`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_s3_motion.uf2`

Rebuild from the repository root if sources change:

```sh
make -C build_make PICO_SDK_PATH=/Users/zhaobao/pico/pico-sdk CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 TEST=0 IR_BENCH=0 S3_MOTION=1 DIAGNOSTICS=0 MOTION_SMOKE=0 -j8
```

It uses the existing gitignored deployment configuration, not host-test fakes.
Do not flash an image marked COMPILE_ONLY. This session does not flash it.
Offline course, three-channel IR bench, motor-smoke and offline self-test profiles
are also checked for compatibility; selecting a build profile is not permission
to run that image's tests on the car.

All **six profiles** (combined Wi-Fi course, combined offline course,
three-sensor bench, motor smoke, offline self-test and combined Wi-Fi diagnostics)
compiled/linked with zero compiler warnings. The final selected build is the
normal combined Wi-Fi course, not the self-test or diagnostics image.

| Image | UF2 bytes | SHA-256 |
|---|---:|---|
| Combined Wi-Fi course | 778752 | `917b11b98d879b08443a4a97114be7078701988b814923efbf909ba2a4102514` |
| Combined offline course | 242176 | `accea04abf66e19747a0e08edf381a1b3a1ac88cdddb83cfc45cf79266817f53` |

Combined Wi-Fi ELF: text 389208, data 152, bss 84384 bytes. Offline course ELF:
text 120904, data 92, bss 24348 bytes. These are linker section sizes, not
measured runtime heap usage, peak stack or a complete physical RAM budget.
Build logs are saved alongside the host logs in `testing/integration/logs/`.

## Hardware checks still required

1. Resolve the front-sensor spacing/height with the team. Mapping now follows
   the wiring confirmed in the 27 October calibration session:
   **GP27 left line (Grove 6), GP28 barcode (Grove 7), GP26 right line (Grove 5)**.
2. Recalibrate after remounting. Built-in references reflect the old fixture;
   numeric table acceptance/READY is not proof the new mounting is calibrated.
3. Test Wi-Fi absent at boot, then enable the hotspot; test repeated loss/rejoin.
   Observe the new timeout/retry messages and verify MQTT actually recovers.
4. Verify telemetry and snapshot/map replies under load. Deliberately interrupt
   the broker and reconnect; duplicates are possible, unlimited buffering is not.
5. With the robot safely supported, verify one GP20 reset after a genuine
   cleared fault, no automatic motion, and GP21 cancellation/held interlock.
6. After physical alignment is good, perform a short line-following trial and
   save both `RUN timing` and the exact halt/fault reason. Measure GP21/physical
   stopping response with networking active. Do not deliberately jam a powered wheel.

Unchanged design limits: five-second barcode-to-junction pending action, turn/
reacquisition geometry, junction-without-barcode halt, one distinct letter per
action, Y/Z both-RIGHT restriction and no automatic course finish recognizer.
These need team decisions or physical evidence, not silent code changes.
