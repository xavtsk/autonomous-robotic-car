# IR wiring and calibration, 27 October 2026

Local `main` base: `6da2977c777714eba48e88c35c4a831844cba40b`.
Hardware work in this session was limited to stationary calibration collection.

## Wiring and measured references

| Physical sensor | Port / GPIO / ADC | White raw range | Black raw range | White / black reference | Valid pairs |
|---|---|---:|---:|---:|---:|
| Front-left | Grove 6 / GP27 / ADC1 | 115..174 | 1242..1761 | 144 / 1501 | 3 |
| Front-right | Grove 5 / GP26 / ADC0 | 146..218 | 2960..3176 | 182 / 3068 | 4 |
| Barcode | Grove 7 / GP28 / ADC2 | 134..179 | 275..335 | 156 / 305 | 3 |

The user confirmed the signal GPIO labels and positioned each surface at the
mounted height. Exact height in millimetres and ambient illuminance were not
measured. Front sensors used white background/black path material; barcode used
white paper/printed black bars. These defaults are specific to those conditions.

The installed firmware used different logical labels: LEFT=GP26, RIGHT=GP28,
BARCODE=GP27. Captures were translated by GPIO, not by those old role names.
Each collection reports 101 raw ADC samples over approximately one second.
These are firmware range summaries, not an individual-sample ADC recording.

- [All collection summaries, with inclusion decisions](logs/2026-10-27_ir_calibration.csv)
- [Captured operator range/acceptance messages](logs/2026-10-27_ir_calibration_console.txt)
- Full local serial recording: `build/ir-captures/raw-calibration-20261027T085618Z.log` (ignored).

Barcode round 2 reported black 1987..2024. The user confirmed that this used the
black path material instead of the printed barcode. The entire barcode pair
from that round is excluded and retained in the evidence. Rounds 1, 3 and 4
provide the three valid paper-barcode pairs. The right sensor was also collected
in round 4 to advance the existing walkthrough; its valid pair is included.

## Calculation and acceptance

For each surface, combine the minimum and maximum across valid collections,
then use `floor((minimum + maximum) / 2)` as its reference. This preserves the
existing built-in calibration mechanism: minimum, ten midpoint samples and
maximum are passed through the normal validator at boot. Its synthetic boot
collector count is not the physical measurement count above.

Every collection has at least ten samples, and every combined white/black pair
is non-overlapping. At the measured endpoints, the unchanged classifier gives:

| Sensor | Highest white darkness | Lowest black darkness |
|---|---:|---:|
| Left | 22 | 809 |
| Right | 12 | 962 |
| Barcode | 154 | 798 |

White must be at most 400; black must be at least 600. The left black range is
wider across placements, but all observed endpoints still meet these limits.
The bench uses the same references without changing its inactive lifecycle.
Button recalibration remains RAM-only; reboot restores the compiled ranges.

## Verification status

Software checks passed on Windows with MSYS2 UCRT64 GCC:

- `tools/test-buddy3.ps1`: reflectance, physical routing, navigation, drive and
  production motion-feedback suites (`build/host-tests/buddy3-final.log`).
- `make -C testing/integration -f controller_test.mk CC=gcc test`: runtime,
  button calibration/rejection, built-in endpoints/bench agreement, START/STOP,
  course controller and simulator (`build/host-tests/integration-final.log`).
- GP28-only bench, all-three bench and offline integrated ARM builds passed,
  including object-symbol startup/ownership checks, with no compiler warnings.
  Logs: `build/host-tests/{bench1,bench3}-final-build.log` and
  `build/host-tests/offline-final-build.log`.

The build check was updated to recognise main's existing Buddy 1 telemetry
mirror/operator panel while continuing to reject Buddy 4/5 and smoke startup.
The simulator build now enables MinGW math constants. Neither change alters
the firmware's steering or motor algorithms.

Build dependencies: Arm GNU 14.3.Rel1 (14.3.1), Pico SDK 2.3.0 at
`98a542c1a62fb549ffb5d66a3e5892b06276b670`, TinyUSB at
`86ad6e56c1700e85f1c5678607a762cfe3aa2f47`. Downloaded compiler archive SHA256:
`864c0c8815857d68a1bbba2e5e2782255bb922845c71c97636004a3d74f60986`.

Final firmware: `build_make/mtk3pico_smp1_usb_cdc_s3_motion.uf2`, 248320 bytes,
485 UF2 blocks. SHA256:
`0845e6560abbbc1329c3b2e54bbac24ea112327294cdab00cf8e0bfb6fdeab2b`.
UF2 magic, family, flags, payload sizes, block indices and flash addresses passed.
ELF sections: text=124052, data=92, bss=24388 bytes.

Build flags: `CONSOLE=usb_cdc S3_MOTION=1 IR_BENCH=0 WIFI=none NET=none MQTT=0
TEST=0 DIAGNOSTICS=0 MOTION_SMOKE=0`. Wi-Fi/MQTT are disabled for this image.

The corrected firmware was **not flashed**. The user could not get the RPI-RP2
drive to appear and requested finishing the source changes without flashing. Post-flash labels,
stationary white/black checks and power-cycle persistence remain unverified on
hardware. The existing Pico firmware is unchanged apart from the earlier
button calibration held in RAM; reboot restores that image's old defaults.

Raw calibration collection remained stopped with no commanded run.
Software/simulator results do not establish physical driving or moving-barcode
performance. The source changes and built UF2 are ready for a later flash.
