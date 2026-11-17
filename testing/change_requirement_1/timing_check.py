#!/usr/bin/env python3
"""Change Requirement 1, timing: did the packed data / network work make the
10 ms control loops late?

Reads a USB console capture from the DIAGNOSTICS=1 firmware (see the evidence
document for how to record it) and checks the last cumulative `[DIAG] interval`
summary of each 10 ms source (motion PID, IR sampling).

PASS when, over the whole capture, the largest gap between two starts stays
below two periods (no control cycle was missed) for both sources.

    python3 testing/change_requirement_1/timing_check.py build/robot-logs/cr1-timing/console.log
"""
import re
import sys

PERIOD_US = 10000
LIMIT_US = 2 * PERIOD_US  # a gap of two periods or more = a missed cycle

LINE = re.compile(r"\[DIAG\] interval source=(\w+) gap_valid=(\d+) gap_min_us=(\d+) "
                  r"gap_max_us=(\d+) exec_max_us=(\d+)")
SAMPLES = re.compile(r"\[DIAG\] timing source=(\w+) samples=(\d+)")


def main(path):
    last, samples = {}, {}
    with open(path, "rb") as f:
        for raw in f:
            line = raw.decode("utf-8", "replace")
            m = LINE.search(line)
            if m:
                last[m.group(1)] = tuple(int(x) for x in m.groups()[1:])
            m = SAMPLES.search(line)
            if m:
                samples[m.group(1)] = int(m.group(2))
    if not last:
        print("No [DIAG] interval lines found: was the DIAGNOSTICS=1 image flashed?")
        return 2
    bad = 0
    print("| Source | Samples | Min gap (ms) | Max gap (ms) | Max execution (ms) | Result |")
    print("|---|---:|---:|---:|---:|---|")
    for source in ("motion", "ir"):
        if source not in last:
            print(f"| {source} | - | - | - | - | **missing** |")
            bad += 1
            continue
        valid, gmin, gmax, emax = last[source]
        ok = valid and gmax < LIMIT_US
        bad += not ok
        print(f"| {source} | {samples.get(source, '?')} | {gmin / 1000:.2f} | {gmax / 1000:.2f} | "
              f"{emax / 1000:.2f} | {'PASS' if ok else '**FAIL**'} |")
    print(f"\nPeriod {PERIOD_US / 1000:.0f} ms; a gap of {LIMIT_US / 1000:.0f} ms or more is a missed cycle.")
    print("RESULT:", "PASS" if not bad else "FAIL")
    return 1 if bad else 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1]))
