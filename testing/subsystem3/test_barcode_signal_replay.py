"""Raw-ADC streaming regressions; synthetic quiet is always explicit.

Build with tools/test-buddy3.ps1, then run this script. BCAP's low two flag
bits describe surface (white=1, black=2), NOT a boolean validity bit. Unknown
surface is conservatively supplied as invalid because the export does not
separately retain raw acquisition validity. Interval sub-milliseconds are also
unknown; test midpoint plus both rounding extremes without inventing samples.
"""
from pathlib import Path
import re
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from dashboard.traces import parse_trace

# Positive expectations use exact observed direction at midpoint timing.
# Other timing/polarity variants may refuse, but must never return a wrong code.
TAIL_RECOVERIES = {"153638", "160054", "160403", "182831", "185701", "190459"}
PADDED_RECOVERIES = TAIL_RECOVERIES | {"152854", "181733", "191949", "210017", "212737"}
QUIET_REFUSALS = {"190147", "190413"}


def main():
    exe = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / "build/host-tests/replay_barcode_signal.exe"
    checks = 0
    observed = {}
    for path in sorted((Path(__file__).parent / "fixtures/barcode").glob("*.txt")):
        trace = parse_trace(path.read_text())
        assert trace["complete"], path.name
        # Multi-episode files preserve their original separate episodes. Each
        # is independent evidence, never joined across an unrecorded interval.
        for episode in sorted({s["episode"] for s in trace["samples"]}):
            samples = [s for s in trace["samples"] if s["episode"] == episode]
            paper = int(statistics.median(s["raw"] for s in samples[:15]))
            expected = "F" if "-f-" in path.name else "Z" if "-z" in path.name else "A"
            stamp = path.name[9:15]
            for fraction in (0, 500, 999):
                for inverted in (False, True):
                    data = "".join(
                        f"{s['dt_ms'] * 1000 + fraction} "
                        f"{4095 - s['raw'] if inverted else s['raw']} "
                        f"{int(bool(s['flags'] & 3))}\n" for s in samples)
                    for lead, tail in ((0, 0), (0, 4000), (2000, 4000)):
                        output = subprocess.run(
                            [str(exe), str(lead), str(tail),
                             str(4095 - paper if inverted else paper), str(int(not inverted))],
                            input=data, text=True, capture_output=True, check=True).stdout
                        letters = re.findall(r"^letter=([A-Z]) ", output, re.MULTILINE)
                        assert len(letters) <= 1 and all(c == expected for c in letters), (
                            path.name, episode, fraction, inverted, lead, tail, output)
                        if episode == 1 and fraction == 500:
                            if stamp in QUIET_REFUSALS:
                                assert not letters, (path.name, "preliminary ink is not white", output)
                            if tail and ((not lead and stamp in TAIL_RECOVERIES) or
                                         (lead and stamp in PADDED_RECOVERIES)):
                                assert letters == [expected], (path.name, lead, tail, output)
                            if not lead and not tail and stamp == "153638":
                                assert letters == [expected], (path.name, output)
                            if not inverted:
                                observed[(path.name, episode, lead, tail)] = "".join(letters) or "refused/incomplete"
                        checks += 1
    print(f"PASS: {checks} raw waveform replay checks; no wrong letters.")
    print("Padding results are conditional algorithm evidence, never physical success counts.")
    for (name, episode, lead, tail), outcome in observed.items():
        if name[9:15] in {"185445", "185701", "190021", "190147", "190413", "190459"}:
            print(f"{name} ep={episode} synthetic_lead_ms={lead} synthetic_tail_ms={tail}: {outcome}")


if __name__ == "__main__":
    main()
