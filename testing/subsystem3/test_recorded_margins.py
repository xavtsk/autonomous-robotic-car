"""Check the recorded A/Z endpoint evidence independently of the firmware fitter."""
from itertools import accumulate, pairwise
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from dashboard.traces import parse_trace


def measure_marks(times, darkness, level):
    """Rank marks and spaces in each printed character at one fixed threshold."""
    ink = [value >= level for value in darkness]
    assert not ink[0] and not ink[-1], "expected paper at both ends"
    edges = [times[i] for i in range(1, len(times)) if ink[i] != ink[i - 1]]
    widths = [b - a for a, b in pairwise(edges)]
    assert len(widths) == 29, f"threshold {level} yielded {len(widths)} elements"
    symbols = []
    for start in (0, 10, 20):  # nine elements per character; single gaps between them
        group = widths[start:start + 9]
        ordered = sorted(group)
        assert ordered[5] < ordered[6], "cannot distinguish the three wide elements"
        symbols.append("".join("w" if width >= ordered[6] else "n" for width in group))
    ten_modules = 10 * sum(sorted(widths[-9:])[:6]) / 6
    return symbols, times[-1] - edges[-1], ten_modules


def replay(executable, steps, values, lead_ms, trail_ms):
    data = "".join(f"{dt} {value}\n" for dt, value in zip(steps, values, strict=True))
    run = subprocess.run([str(executable), str(lead_ms), str(trail_ms)],
                         input=data, text=True, capture_output=True,
                         check=True, timeout=30)
    lines = run.stdout.splitlines()
    assert "timing=0 overflow=0" in lines[-1], run.stdout
    events = [dict(token.split("=") for token in line.split())
              for line in lines if line.startswith("letter=")]
    return events, lines[-1]


def main(executable):
    # Wide/narrow patterns taken from the supplied print artwork, transcribed in
    # test_core_main.c (pages 1 and 3). Reverse scanning reverses all 29 elements.
    star = "nwnnwnwnn"
    captures = (
        ("20261103-152854-successful-z.txt", "Z", "nwwnwnnnn", True),
        ("20261103-171918-normal-speed.txt", "A", "wnnnnwnnw", False),
    )
    fixtures = Path(__file__).parent / "fixtures" / "barcode"
    for name, letter, payload, reverse in captures:
        trace = parse_trace((fixtures / name).read_text())
        assert trace["complete"] and len(trace["episodes"]) == 1, name
        samples = trace["samples"]
        steps = [s["dt_ms"] * 1000 + 500 for s in samples]
        values = [s["darkness"] for s in samples]
        times = list(accumulate(steps))
        expected = [star, payload, star]
        if reverse:
            expected = [symbol[::-1] for symbol in reversed(expected)]
        widths, margins = [], []
        for level in (32, 40, 48, 56):
            symbols, tail, ten_modules = measure_marks(times, values, level)
            assert symbols == expected, (name, level, symbols)
            widths.append(ten_modules)
            margins.append(tail)
        assert min(margins) == max(margins), "tail changed across threshold sweep"
        print(f"{letter} ({len(samples)} samples) has {margins[0] / 1000:.1f} ms "
              f"of trailing paper. Ten narrow elements span "
              f"{min(widths) / 1000:.1f} to {max(widths) / 1000:.1f} ms.")

        observed, _ = replay(executable, steps, values, 0, 0)
        assert not observed, (name, "clipped leading margin", observed)
        observed, end = replay(executable, steps, values, 2000, 0)
        if reverse:
            # The recorded tail is close to ten modules; the firmware's
            # qualified quiet clock still lacks one part of a narrow element.
            assert min(widths) <= max(margins) <= max(widths), name
            assert not observed and "rejected=0" in end and "state=2 " in end, end
            observed, _ = replay(executable, steps, values, 2000, 100)
            earliest, latest = 2_000_000 + times[-1], 2_100_000 + times[-1]
        else:
            assert min(margins) > max(widths), name
            # Keep every bar but remove 0.4 s from the measured trailing paper.
            count = sum(t <= times[-1] - 400_000 for t in times)
            shortened, _ = replay(executable, steps[:count], values[:count], 2000, 0)
            assert not shortened, (name, "incomplete trailing margin", shortened)
            earliest, latest = 2_000_000 + times[count - 1], 2_000_000 + times[-1]

        assert len(observed) == 1, (name, observed)
        event = observed[0]
        assert (event["letter"], event["reversed"]) == (letter, str(int(reverse))), event
        assert earliest < int(event["timestamp_us"]) <= latest, event
    print("PASS: printed patterns, observed endpoints and quiet-margin controls")


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve())
