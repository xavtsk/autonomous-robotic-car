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
    print(f"PASS: {checks} recorded replay checks; padded results are conditional on synthetic quiet margins.")


if __name__ == "__main__":
    main()
