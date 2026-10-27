# Offline preparation evidence, 2026-10-24

Branch: local, uncommitted `integration/buddies-1-3`. No Pico/robot port was
opened, no robot commands were sent, no firmware was flashed, and nothing was
committed or pushed. All robot/RTOS devices below were substituted in host
tests. The downloaded `pyserial` dependency is installed only in the ignored
project environment `build/robot-tools`, not the system Python.

## What was added

- `tools/capture_robot_log.py`: read-only live USB capture, or explicitly
  labelled replay. Exact `console.log`, `metadata.json`, optional
  `hostarrival.csv`. New output directories only; prepared before live port
  open. Metadata is replaced atomically after a complete write. It sends no
  application commands; OS/driver control-line toggling is still possible.
- `app_program/diagnostics/robot_diagnostics.c/.h`: opt-in observation of motion
  and IR activations, plus raw encoder/glitch counts and non-consuming status/
  estimator observations. No movement/reset/`motion_poll()` calls.
- `DIAGNOSTICS=1 TEST=0 IR_BENCH=0`: separate `_diag` image. One additional
  mutex; existing initial task becomes a priority-23 reporter approximately
  once per second. No additional task or stack reservation. Existing motion,
  sensing and networking priorities/periods are unchanged.
- Session guide and fill-in notes template. Real-world results remain blank.

## Completed software checks

| Check | Result |
|---|---|
| Laptop logger unit tests | 25 PASS in system Python and isolated environment |
| Raw input cases | CRLF, NUL, invalid UTF-8, fragmented/partial/long lines preserved in the raw log; CSV buffering bounded |
| Recording failures | Existing paths, broken symlinks, unusable parent, path race and initial output failure cannot open a live port; serial-open/read failure retains error/partial evidence |
| Metadata failures | Failed JSON write or replacement preserves the prior valid JSON and raw bytes; capture reports failure, not success |
| Interruption / duration | Fake quiet/live timeout, monotonic duration and Ctrl+C finalization PASS |
| Serial boundary | Fake port verifies no writes, DTR/RTS low before opening, close on exit; no real port enumerated/opened |
| Replay smoke test | 4206-byte existing host-test log copied byte-for-byte; metadata explicitly `REPLAY_NOT_HARDWARE` |
| Timing module | First sample, unsigned timer wrap, min/max gaps, max elapsed interval, private extrema preserved across deferred publication, counter saturation and invalid source PASS |
| Observation safety | Polling producer locks; no console/Buddy 2 calls under diagnostic mutex; COMPLETE not consumed; no movement/reset symbols linked |
| Estimator conversion | Independent numeric validity; NaN/infinity/out-of-int32 values rejected before cast; no protocol-range clipping |
| Actual startup wrapper | Init precedes producers; original startup sequence preserved; priority 23 before periodic output; 1000 ms delay; init/priority failure falls back to sleep |
| Sanitizers / review | Diagnostic module/startup ASan + UBSan PASS; Clang analysis clean; independent read-only review completed |
| Existing Buddy 1/2/3/controller tests | PASS, including sanitizers; portable Buddy 2 (3754 rows) and Buddy 3 (53660 rows) before/after traces remain identical |
| Profile guards | DIAGNOSTICS with TEST=1, IR_BENCH=1, or an invalid option rejected before building |

Reproduce from the repository root:

```
python3 -m unittest discover -s testing/diagnostics -p test_capture_robot_log.py -v
make -C testing/diagnostics -f firmware_test.mk test sanitize disabled
```

The isolated environment uses `tools/requirements-robot-log.txt` (pyserial 3.5).
Replay and tests do not need that package or a network. Live USB compatibility
is not proven merely by installing/importing the package.

## Full firmware builds and observer cost

All used the existing Arm GNU 15.3.1/Pico SDK build, SMP=1, USB CDC and public
example Wi-Fi settings supplied through a temporary include directory. No
private credentials were read. These compile-only network images must be
rebuilt with the user's own private settings before a real network test.

| Profile | text | data | bss | Result |
|---|---:|---:|---:|---|
| Normal Wi-Fi/lwIP/MQTT, diagnostics off | 363624 B | 160 B | 82892 B | compile/link PASS |
| Normal Wi-Fi/lwIP/MQTT, diagnostics on | 365744 B | 160 B | 83024 B | compile/link PASS |
| Wi-Fi/lwIP/MQTT TEST=1, diagnostics off | 372464 B | 160 B | 82892 B | compile/link PASS |
| Isolated IR_BENCH=1, network/diagnostics off | 94176 B | 100 B | 23308 B | compile/link PASS |

Opt-in diagnostics adds **132 bytes of static RAM** and **2120 bytes of linked
code/read-only data** in this comparison, plus consumption of one kernel mutex
object slot. This is not total peak RAM or free heap. The initial task reuses
its existing 1024-byte stack; the diagnostic report's direct Arm frame is
120 bytes, excluding callees, formatting and interrupt frames. Real stack
margin remains unmeasured.

Disabled diagnostic module contains no retained code/data. A stronger check
also extracted the loadable binary from the saved pre-preparation TEST=1 image
and the revised `DIAGNOSTICS=0` image: they are byte-identical, SHA-256
`48093d047dc7075941fc3fc482609b5f8772cd31306daca330a67c284f45b925`.
No motion/PID/calibration/decoder algorithm was changed.

## How to interpret the observations

Each timing source has one private writer. It collects every activation then
tries to publish a complete summary with `TMO_POL`. A failed attempt increments
`publish_skips`; the next success includes all intervening extrema/counters.
The reporter copies only published data under the mutex and prints after
unlocking. Publication skips are not missed control cycles.

Start gaps and execution intervals use MCU microseconds. They include
preemption/blocking and exclude the final diagnostic recording overhead; they
are not CPU-only WCET or a measured CPU-utilisation percentage. Unsigned timer
subtraction assumes consecutive calls are less than 2^32 microseconds apart.
Counters saturate at UINT32_MAX. The inherited SMP console masks interrupts
within each print call; low-rate output still affects what is being measured.
Compare diagnostic-on/off runs before drawing timing conclusions.

Encoder/status reads are sequential, not an instantaneous multi-field snapshot.
The existing packed motion snapshot makes speed unavailable while busy; that
behaviour was not changed. Separate diagnostic estimator readings have numeric
validity only, not proof of calibration or physical speed. Physical measurements
and the existing calibration output remain necessary.

Laptop arrival timestamps include buffering and cannot measure the motion
period. Raw capture preserves received bytes, not bytes dropped in firmware's
USB buffer before transmission. A completed recording is not a PASS verdict.

## Still open for hardware work

Actual wiring/calibration, live serial operation, Wi-Fi/broker recovery, moving
barcode/navigation performance, maximum encoder-count bounds, motion timing
under network load and real stack/heap high-water remain unvalidated. The
local autonomous-start adapter and Buddy 4/5 integration are not added here.
Networking stays at priority 4 and IR sampling stays at 10 ms; proposed changes
need separate equivalent-condition tests. Do not label this preparation as
Week 13 hardware demonstration evidence.
