#!/usr/bin/env python3
"""Check diagnostic start gaps against each source's configured period.

Defaults come from the current firmware headers: motion PID and IR sampling
need not have the same period. For an older capture, pass both period overrides
from that image's build configuration. The log itself does not identify periods.

PASS means all recorded valid start gaps are below two configured periods. This
is a missed-cycle screening criterion, not proof that every deadline was met or
that the requested workload/reconnection experiment actually took place.
"""
import argparse
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ("motion", "ir")
PERIOD_DEFINITIONS = {
    "motion": ("app_program/subsystem2/buddy2.h", "S2_PERIOD_MS"),
    "ir": ("app_program/controller/run_timing.h", "S3_SAMPLE_PERIOD_MS"),
}
LINE = re.compile(r"\[DIAG\] interval source=(\w+) gap_valid=(\d+) gap_min_us=(\d+) "
                  r"gap_max_us=(\d+) exec_max_us=(\d+)")
SAMPLES = re.compile(r"\[DIAG\] timing source=(\w+) samples=(\d+)")


def source_periods(root=ROOT):
    """Read integer millisecond macros; fail closed on unsupported definitions."""
    periods = {}
    for source, (name, macro) in PERIOD_DEFINITIONS.items():
        text = (root / name).read_text(encoding="utf-8")
        match = re.search(
            rf"^\s*#define\s+{macro}\s+\(?\s*(\d+)[uUlL]*\s*\)?\s*(?:/\*[^\n]*\*/\s*)?$",
            text, re.MULTILINE)
        if not match or int(match[1]) <= 0:
            raise ValueError(f"Cannot determine positive {macro} from {name}")
        periods[source] = int(match[1]) * 1000
    return periods


def read_capture(path):
    """Keep all summaries so a reset cannot hide an earlier excessive gap."""
    intervals = {source: [] for source in SOURCES}
    samples = {}
    with open(path, "rb") as stream:
        for raw in stream:
            line = raw.decode("utf-8", "replace")
            match = LINE.search(line)
            if match and match[1] in intervals:
                intervals[match[1]].append(tuple(int(x) for x in match.groups()[1:]))
            match = SAMPLES.search(line)
            if match and match[1] in intervals:
                samples[match[1]] = max(samples.get(match[1], 0), int(match[2]))
    return intervals, samples


def main(path, periods_us=None):
    periods = source_periods() if periods_us is None else periods_us
    if set(periods) != set(SOURCES) or any(
            type(value) is not int or value <= 0 for value in periods.values()):
        raise ValueError("Provide a positive integer period in microseconds for each source")
    intervals, samples = read_capture(path)
    if not any(intervals.values()):
        print("No [DIAG] interval lines found: was the DIAGNOSTICS=1 image flashed?")
        return 2
    bad = 0
    print("| Source | Period (ms) | Samples | Min gap (ms) | Max gap (ms) | Max execution (ms) | Result |")
    print("|---|---:|---:|---:|---:|---:|---|")
    for source in SOURCES:
        period = periods[source]
        rows = intervals[source]
        valid_rows = [row for row in rows if row[0] == 1 and 0 < row[1] <= row[2]]
        malformed = any(row[0] not in (0, 1) or
                        (row[0] == 1 and not 0 < row[1] <= row[2]) for row in rows)
        if not valid_rows or samples.get(source, 0) < 2 or malformed:
            print(f"| {source} | {period / 1000:g} | {samples.get(source, '?')} | - | - | - | **insufficient/invalid evidence** |")
            bad += 1
            continue
        gmin = min(row[1] for row in valid_rows)
        gmax = max(row[2] for row in valid_rows)
        emax = max(row[3] for row in valid_rows)
        ok = gmax < 2 * period
        bad += not ok
        print(f"| {source} | {period / 1000:g} | {samples[source]} | {gmin / 1000:.3f} | "
              f"{gmax / 1000:.3f} | {emax / 1000:.3f} | {'PASS' if ok else '**FAIL**'} |")
    for source in SOURCES:
        print(f"{source}: period {periods[source] / 1000:g} ms; "
              f"a gap >= {2 * periods[source] / 1000:g} ms fails this check.")
    print("This screens recorded start gaps; execution times and the physical workload still require review.")
    print("RESULT:", "PASS" if not bad else "FAIL")
    return 1 if bad else 0


def positive_ms(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("period must be a positive integer number of milliseconds")
    return number * 1000


def cli(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    for source in SOURCES:
        parser.add_argument(f"--{source}-period-ms", type=positive_ms,
                            help=f"override {source} period for the captured firmware")
    args = parser.parse_args(argv)
    try:
        overrides = {source: getattr(args, f"{source}_period_ms") for source in SOURCES}
        periods = source_periods() if any(value is None for value in overrides.values()) else {}
        periods.update({source: value for source, value in overrides.items() if value is not None})
        print("Period source: explicit overrides where supplied; otherwise current repository headers.")
        print("For archived firmware, supply both --motion-period-ms and --ir-period-ms from its build.")
        return main(args.path, periods)
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    sys.exit(cli())
