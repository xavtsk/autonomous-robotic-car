# Buddy 3 handoff: build and RAM evidence

**Historical same-profile measurements below.** The 25 October import changes
motion/PIO, navigation fields and runtime ownership. These old savings must not
be reused as a measurement of the new integration. Current build sizes,
60/122/11-byte copied formats and remaining runtime measurements are in
[the integration review](../../docs/INTEGRATION_2026-10-25.md).

Measured 2026-10-23 on the uncommitted local `integration/buddies-1-3` tree.
Baseline is the saved working tree immediately before this Buddy 3 update,
not an earlier Week 6 design or a clean branch that omits previous Buddy 1/2
work. The comparison includes the compact controller handoff and the new
Buddy 1 IR reporting/protocol-v2 changes.

## Same-profile comparison

Arm GNU 15.3.1, Cortex-M0+, existing makefile `-O2` options, SMP=1,
CONSOLE=usb_cdc, WIFI=cyw43, NET=lwip, MQTT=1, TEST=1, IR_BENCH=0.

| Linker section | Before | After | Difference |
|---|---:|---:|---:|
| text (code/read-only data) | 367744 B | 372464 B | +4720 B |
| data | 160 B | 160 B | 0 B |
| bss | 83128 B | 82892 B | -236 B |
| Static RAM (data + bss) | 83288 B | 83052 B | **236 B saved** |

Packing saves RAM and copied-message bytes, but requires encoding/decoding
code and processing. The code increase is recorded, not hidden by counting
only field bits. These are linked static sections, **not** total peak RAM,
available heap, stack high-water or task-timing measurements. Dynamic kernel
allocations, task stacks and networking runtime allocations must still be
budgeted and measured on the board. Task priorities/periods/stack settings and
RTOS object counts were not increased by this update.

Target symbol/structure checks:

| Item | Before | After |
|---|---:|---:|
| Controller shared mailbox | 100 B | 38 B |
| Retained navigation context | 412 B | 344 B |
| Duplicate wide global navigation config | 116 B | removed |
| Retained ADC-channel config | 4 B | 2 B |
| Calibration collector (two labels) | 24 B | 18 B |
| Classifier per sensor | 10 B | 7 B |
| Copied navigation packet | decoded struct 72 B | 30 B |
| Copied pending barcode | decoded struct 8 B | 5 B |
| Bench copied raw frame | decoded struct 52 B | 34 B |

Do not add these rows to calculate total saved RAM: some are nested within
others, some are not instantiated in every profile, and ABI padding matters.
Wide decoded/raw buffers remain where needed for validation, driver callbacks
and calculations. Bench mode retains extra private decode buffers so console
work stays outside the snapshot lock. Sampling uses 69 bytes of packet locals
plus normal compiler call-frame/spill overhead; init uses a temporary wide
config. Real stack margin is still pending, not inferred from these locals.

## Successful full-image builds

| Profile | text | data | bss | Result |
|---|---:|---:|---:|---|
| USB + Wi-Fi/lwIP/MQTT, TEST=1 | 372464 B | 160 B | 82892 B | compile + link PASS |
| USB + Wi-Fi/lwIP/MQTT, TEST=0 | 363624 B | 160 B | 82892 B | compile + link PASS |
| Isolated USB IR_BENCH=1, Wi-Fi/network/MQTT off | 94176 B | 100 B | 23308 B | compile + link PASS |

Builds used the existing public `config/wifi_secrets.example.h` through a
temporary include directory. No private Wi-Fi configuration was read, no
connection was made and nothing was flashed. There were no compiler/linker
warnings or errors in the captured build logs.

Reproduce with your ordinary local configuration:

```
make -C build_make CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 TEST=1 -j8
make -C build_make CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 TEST=0 -j8
make -C build_make CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 TEST=0 IR_BENCH=1 -j8
```

Do not override TARGET with a board nickname; use the makefile's existing
`_PICO_RP2040_` setting. Profile changes rebuild generated objects.

## Software verification

- Buddy 3 original core/reflectance tests PASS.
- Before/after portable algorithms: **53,660 identical decoded trace rows**.
- Actual bench wrapper: **58 identical console lines**, including startup/read
  failures, empty snapshot, ADC boundaries, driver errors and timestamp wrap.
- Packet tests exercise full ADC range, malformed input/padding, exact floats,
  event/fault combinations and failure non-mutation; ASan/UBSan PASS.
- Actual controller task with fake RTOS/motor boundaries: exact steering,
  pending barcode until junction, sticky events, turn completion, local
  start/stop, IR ownership and full-outbox cases PASS, including ASan/UBSan.
- Buddy 2 control regression: **3754 identical trace rows**; adapter tests and
  ASan/UBSan PASS. No Buddy 2 production file changed in this update.
- Buddy 1 C suite: **40 checks PASS**, plus 5 laptop-codec tests, all 64 fault
  masks and concurrent-producer/callback coverage; ASan/UBSan PASS.
- Clang static analysis of changed portable Buddy 3 codec/navigation/barcode/
  line/classifier modules reported no issues. Independent read-only review
  found no packing-induced algorithm change.

See `EVIDENCE.md` for hashes and reproducible Buddy 3 test commands. Controller
tests use `make -C testing/integration -f controller_test.mk test sanitize`.
Buddy 1/2 have their own test folders and historical evidence records.

These software checks do not demonstrate real RTOS scheduling/atomicity,
physical ADC timing, motor accuracy, moving-barcode reliability or correct
original-line recovery. The latter remains deferred to Buddy 5 integration.
