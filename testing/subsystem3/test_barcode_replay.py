"""Recorded-waveform regressions; synthetic margins are explicit, never hardware proof.

Run tools/test-buddy3.ps1 first, then:
  python testing/subsystem3/test_barcode_replay.py
Only the standard library and the repository's BCAP parser are needed.
"""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from dashboard.traces import parse_trace


def main():
    exe = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / "build/host-tests/replay_barcode_record.exe"
    cases = {
        "20261103-152854-successful-z.txt": "Z",
        "20261103-171918-normal-speed.txt": "A",
        "20261103-173819-half-speed-1.txt": "",
        "20261103-173958-half-speed-2.txt": "A",
        "20261103-181733-adaptive-a.txt": "A",
        "20261103-182659-varying-a.txt": "A",
        "20261103-191949-speed003-a.txt": "A",
        "20261103-192042-speed003-a.txt": "A",
        "20261103-192137-speed003-a.txt": "A",
        "20261103-212737-wide-f-shifted-rejected.txt": "F",
    }
    checks = 0
    for name, padded_letter in cases.items():
        trace = parse_trace((Path(__file__).parent / "fixtures/barcode" / name).read_text())
        assert trace["complete"] and len(trace["episodes"]) == 1, name
        # BCAP floors each interval to milliseconds. Its missing sub-ms part
        # is unknown: use the midpoint for regression, plus both extremes to
        # check that timing uncertainty does not produce a different letter.
        for fraction in (0, 500, 999):
            data = "".join(f"{s['dt_ms'] * 1000 + fraction} {s['darkness']}\n" for s in trace["samples"])
            for lead, trail in ((0, 0), (0, 1000), (2000, 0), (2000, 1000)):
                run = subprocess.run([str(exe), str(lead), str(trail)], input=data,
                                     text=True, capture_output=True, check=True)
                letters = "".join(line.split()[0].split("=")[1]
                                  for line in run.stdout.splitlines() if line.startswith("letter="))
                assert "timing=0 overflow=0" in run.stdout, (name, fraction, lead, trail, run.stdout)
                assert not letters or letters == padded_letter, (name, fraction, lead, trail, run.stdout)
                if lead == 0:
                    assert not letters, (name, "missing leading quiet must not be fabricated", run.stdout)
                if fraction == 500:
                    expected = padded_letter if lead and (trail or padded_letter == "Z") else ""
                    assert letters == expected, (name, lead, trail, run.stdout)
                checks += 1
    # These observed A/Z captures include their real leading margin. Raw ADC is
    # retained exactly; reconstruct the firmware's 215/3550 calibration rather
    # than treating the exported darkness/4 preview as full precision. Some
    # captures end before the new reader can confirm trailing quiet: appended
    # white below is explicitly synthetic and is never a hardware success.
    # The boolean additionally requires the lossy darkness/4 reconstruction
    # to recover at midpoint timing. For the remaining low-contrast captures,
    # only the full-precision raw reconstruction is a positive expectation.
    observed_cases = {
        "20261105-153638-small-a-faint-rejected.txt": ("A", True),
        "20261105-160054-small-a-faint-rejected.txt": ("A", True),
        "20261105-160403-small-a-faint-rejected.txt": ("A", True),
        "20261105-160438-small-a-faint-rejected.txt": ("A", False),
        "20261105-182025-small-a-decoded.txt": ("A", True),
        "20261105-182223-small-a-rejected.txt": ("A", True),
        "20261105-182416-small-a-rejected.txt": ("A", False),
        "20261105-182831-small-a-rejected.txt": ("A", False),
        "20261105-183024-wide-z-decoded.txt": ("Z", True),
    }
    # The raw 18:22 payload has a narrow white gap just below the strict
    # minimum. Quantization can move it across that boundary; keep the raw
    # refusal rather than loosening width rules to make this fixture pass.
    raw_rejections = {"20261105-182223-small-a-rejected.txt"}
    for name, (expected_letter, require_quantized) in observed_cases.items():
        trace = parse_trace((Path(__file__).parent / "fixtures/barcode" / name).read_text())
        assert trace["complete"] and len(trace["episodes"]) == 1, name
        samples = trace["samples"]
        for raw in (False, True):
            darkness = [max(0, min(1000, (s["raw"] - 215) * 1000 // 3335))
                        if raw else s["darkness"] for s in samples]
            for fraction in (0, 500, 999):
                intervals = [s["dt_ms"] * 1000 + fraction for s in samples]
                for reverse in (False, True):
                    values = list(reversed(darkness)) if reverse else darkness
                    steps = ([intervals[0]] + list(reversed(intervals[1:]))) if reverse else intervals
                    data = "".join(f"{dt} {dark}\n" for dt, dark in zip(steps, values))
                    for lead, trail in ((0, 0), (0, 1000), (2000, 0), (2000, 1000)):
                        out = subprocess.run([str(exe), str(lead), str(trail)], input=data,
                                             text=True, capture_output=True, check=True).stdout
                        found = [line for line in out.splitlines() if line.startswith("letter=")]
                        assert len(found) <= 1, (name, out)
                        assert all(line.startswith(f"letter={expected_letter} reversed={int(reverse)} ")
                                   for line in found), (name, raw, fraction, reverse, out)
                        assert "timing=0 overflow=0" in out, (name, out)
                        # Require recovery in the observed scan direction at
                        # midpoint timing. Reversed/quantized approximations
                        # may safely refuse, but may never invent a letter.
                        if fraction == 500 and not reverse and lead and trail:
                            if raw and name in raw_rejections:
                                assert not found, (name, raw, reverse, out)
                            elif raw or require_quantized:
                                assert len(found) == 1, (name, raw, reverse, out)
                        checks += 1
    print(f"PASS: {checks} recorded replay checks; padded results are conditional on synthetic quiet margins.")


if __name__ == "__main__":
    main()
