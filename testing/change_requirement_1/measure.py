#!/usr/bin/env python3
"""Change Requirement 1: RAM and storage measurements on the real target build.

1. Builds the firmware (same flags as the demo build, IR_DEBUG=0 and =1) and
   records linked RAM = .data + .bss from arm-none-eabi-size.
2. RAM per buddy: .data + .bss of each subsystem's object files from the
   IR_DEBUG=1 build (before the linker drops unused sections, so an upper bound).
3. Storage before/after (packed vs unpacked) measured by the TARGET compiler:
   compiles struct_sizes.c with the firmware's own compile flags and reads the
   size of each array symbol with arm-none-eabi-nm.

Needs: the Arm toolchain on PATH, PICO_SDK_PATH, config/wifi_secrets.h
(as for a normal build). Run from the repository root:
    python3 testing/change_requirement_1/measure.py
Prints Markdown tables; nothing in the source tree is changed.
"""
import os
import re
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build_make"
FLAGS = ("CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 TEST=0 IR_BENCH=0 S3_MOTION=1 "
         "DIAGNOSTICS=0 MOTION_SMOKE=0 S4=1 S5=1 S5_HARDWARE_CONFIRMED=1").split()
GROUPS = [("Buddy 1 Wi-Fi/MQTT", "subsystem1"), ("Buddy 2 motion", "subsystem2"),
          ("Buddy 3 line/barcode", "subsystem3"), ("Buddy 4 IMU/terrain", "subsystem4"),
          ("Buddy 5 obstacle", "subsystem5"), ("Vehicle controller", "controller")]


def run(cmd, **kw):
    return subprocess.run(cmd, check=True, text=True, capture_output=True, **kw).stdout


def build(ir_debug):
    run(["make", "-C", str(BUILD), *FLAGS, f"IR_DEBUG={ir_debug}", f"-j{os.cpu_count() or 4}"])
    name = "mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_s3_motion_s4_s5" + ("_ir_debug" if ir_debug else "")
    text, data, bss = run(["arm-none-eabi-size", str(BUILD / f"{name}.elf")]).splitlines()[1].split()[:3]
    return name, int(text), int(data), int(bss)


def object_ram(folder):
    objs = sorted((BUILD / "mtkernel_3" / "app_program" / folder).glob("*.o"))
    objs = [o for o in objs if not o.name.endswith("_test.o")]
    total = 0
    for line in run(["arm-none-eabi-size", *map(str, objs)]).splitlines()[1:]:
        f = line.split()
        total += int(f[1]) + int(f[2])
    return total, len(objs)


def compile_flags():
    """The firmware's own compile command for one app file, from `make -n`."""
    src = ROOT / "app_program" / "subsystem5" / "obstacle_avoidance.c"
    src.touch()
    out = run(["make", "-n", "-C", str(BUILD), *FLAGS, "IR_DEBUG=1"])
    line = next(l for l in out.splitlines() if "arm-none-eabi-gcc" in l and "obstacle_avoidance.c" in l)
    args = shlex.split(line)
    keep, skip = [], False
    for a in args[1:]:
        if skip:
            skip = False
            continue
        if a in ("-o", "-MF", "-MT"):
            skip = True
            continue
        if a.startswith(("-MF", "-MT")) or a in ("-MMD", "-MP", "-c") or a.endswith(".c"):
            continue
        keep.append(a)
    return keep


def struct_sizes():
    flags = compile_flags()
    here = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory() as tmp:
        obj = Path(tmp) / "struct_sizes.o"
        inc = [f"-I{ROOT / 'app_program' / d}" for d in ("subsystem4", "subsystem5")]
        r = subprocess.run(["arm-none-eabi-gcc", *flags, *inc, "-c", str(here / "struct_sizes.c"),
                            "-o", str(obj)], cwd=BUILD, text=True, capture_output=True)
        if r.returncode:
            sys.exit("struct_sizes.c did not compile:\n" + r.stderr)
        sizes = {}
        for line in run(["arm-none-eabi-nm", "-S", str(obj)]).splitlines():
            m = re.match(r"[0-9a-f]+ ([0-9a-f]+) \w size_(\w+)", line)
            if m:
                sizes[m.group(2)] = int(m.group(1), 16)
    return sizes


def main():
    print("## Linked firmware RAM (target build)\n")
    print("| Image | Code+const (text) | .data | .bss | RAM = .data + .bss |")
    print("|---|---:|---:|---:|---:|")
    for ir in (0, 1):
        name, text, data, bss = build(ir)
        print(f"| `{name}` | {text} | {data} | {bss} | {data + bss} |")
    print("\n## RAM per buddy (object files of the IR_DEBUG=1 build, .data + .bss)\n")
    print("| Buddy | Objects | .data + .bss bytes |")
    print("|---|---:|---:|")
    for label, folder in GROUPS:
        total, n = object_ram(folder)
        print(f"| {label} | {n} | {total} |")
    s = struct_sizes()
    rows = [
        ("Buddy 4 shared terrain measurement", "b4_unpacked_result", "b4_packed_packet"),
        ("Buddy 4 event queue, 8 slots", "b4_unpacked_queue8", "b4_packed_queue8"),
        ("Buddy 5 shared task snapshot", "b5_unpacked_observation", "b5_packed_observation"),
        ("Buddy 5 one scan sample", "b5_unpacked_sample", "b5_packed_sample"),
        ("Buddy 5 scan sample buffer, 16", "b5_unpacked_samples16", "b5_packed_samples16"),
        ("Controller -> telemetry Buddy 5 status", "ctrl_unpacked_s5_status", "ctrl_packed_s5_status"),
        ("Controller bend detector (14 Nov fix)", "ctrl_bend_before", "ctrl_bend_after"),
    ]
    print("\n## Storage before (unpacked C types) / after (packed), target compiler\n")
    print("| Data | Before bytes | After bytes | Saved |")
    print("|---|---:|---:|---:|")
    for label, before, after in rows:
        b, a = s[before], s[after]
        print(f"| {label} | {b} | {a} | {b - a} ({100 * (b - a) // b}%) |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
