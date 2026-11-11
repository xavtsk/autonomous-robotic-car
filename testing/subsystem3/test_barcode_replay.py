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
    # The synchronous reader published on the final retained sample of these
    # captures. Its bounded worker can legitimately still be processing at EOF.
    # Permit only clean PROCESSING for these exact midpoint/observed variants;
    # their existing explicitly padded counterparts must still decode. Do not
    # invent extra timestamped samples or relax clipped leading/quiet margins.
    pending_eof_cases = {
        ("20261103-152854-successful-z.txt", False, 2000),
        ("20261110-165431-small-a-decoded.txt", True, 0),
        ("20261110-165431-small-a-decoded.txt", True, 100),
        ("20261110-165500-small-a-decoded.txt", True, 0),
        ("20261110-165500-small-a-decoded.txt", True, 100),
        ("20261110-165615-wide-w-decoded.txt", True, 100),
        ("20261110-165652-wide-w-decoded.txt", True, 0),
        ("20261110-165652-wide-w-decoded.txt", True, 100),
        ("20261111-163440-small-z-decoded.txt", True, 0),
        ("20261111-163440-small-z-decoded.txt", True, 1000),
    }

    def pending_at_eof(name, raw, fraction, reverse, lead, trail, out):
        return (fraction == 500 and not reverse and trail == 0 and
                (name, raw, lead) in pending_eof_cases and
                "decoded=0 rejected=0 timing=0 overflow=0 state=3 " in out)

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
                    assert (letters == expected or (expected and pending_at_eof(
                        name, False, fraction, False, lead, trail, run.stdout))), (name, lead, trail, run.stdout)
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
    # Replacement-sensor captures include four observed successes and one W
    # rejection. A complete BCAP upload does not guarantee all physical quiet
    # margins were retained. Keep the two clipped-leading W cases explicit:
    # the failed W also needs more observed trailing white before publication.
    replacement_cases = {
        "20261110-165431-small-a-decoded.txt": ("A", False, "unmodified"),
        "20261110-165500-small-a-decoded.txt": ("A", True, "unmodified"),
        "20261110-165546-wide-w-rejected.txt": ("W", False, "both_margins"),
        "20261110-165615-wide-w-decoded.txt": ("W", False, "leading_margin"),
        "20261110-165652-wide-w-decoded.txt": ("W", True, "unmodified"),
    }
    for name, (expected_letter, observed_reverse, margin_need) in replacement_cases.items():
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
                    for lead, trail in ((0, 0), (0, 1000), (100, 0), (100, 1000)):
                        out = subprocess.run([str(exe), str(lead), str(trail)], input=data,
                                             text=True, capture_output=True, check=True).stdout
                        found = [line for line in out.splitlines() if line.startswith("letter=")]
                        details = (name, raw, fraction, reverse, lead, trail, out)
                        assert len(found) <= 1, details
                        assert all(line.startswith(f"letter={expected_letter} reversed={int(observed_reverse ^ reverse)} ")
                                   for line in found), details
                        assert "timing=0 overflow=0" in out, details
                        if raw and fraction == 500 and not reverse:
                            if margin_need == "unmodified" or (lead and
                                    (trail or margin_need == "leading_margin")):
                                assert len(found) == 1 or pending_at_eof(
                                    name, raw, fraction, reverse, lead, trail, out), details
                            else:
                                assert not found, details
                            if margin_need == "both_margins" and lead and not trail:
                                # A valid candidate waits for real trailing
                                # quiet; expanding workspace must not bypass it.
                                assert any(f"decoded=0 rejected=0 timing=0 overflow=0 state={state} " in out
                                           for state in (2, 3)), details
                        checks += 1
    # Raised-sensor Z batch: the small failed scan lacks ink regions; adding
    # quiet must not invent them. The two large scans stopped with motor-command
    # faults on the car. Their exports clip quiet context: padded recovery only
    # checks intact geometry, and is NOT a newly successful physical scan.
    raised_cases = {
        "20261111-163031-small-z-rejected.txt": (False, "missing_bars"),
        "20261111-163440-small-z-decoded.txt": (True, "unmodified"),
        "20261111-163608-large-z-motion-fault.txt": (False, "both_margins"),
        "20261111-164410-large-z-motion-fault.txt": (True, "both_margins"),
    }
    for name, (observed_reverse, margin_need) in raised_cases.items():
        trace = parse_trace((Path(__file__).parent / "fixtures/barcode" / name).read_text())
        assert trace["complete"] and len(trace["episodes"]) == 1, name
        samples = trace["samples"]
        darkness = [max(0, min(1000, (s["raw"] - 215) * 1000 // 3335)) for s in samples]
        for fraction in (0, 500, 999):
            intervals = [s["dt_ms"] * 1000 + fraction for s in samples]
            for reverse in (False, True):
                values = list(reversed(darkness)) if reverse else darkness
                steps = ([intervals[0]] + list(reversed(intervals[1:]))) if reverse else intervals
                data = "".join(f"{dt} {dark}\n" for dt, dark in zip(steps, values))
                for lead, trail in ((0, 0), (1000, 0), (0, 3000), (1000, 3000)):
                    out = subprocess.run([str(exe), str(lead), str(trail)], input=data,
                                         text=True, capture_output=True, check=True).stdout
                    found = [line for line in out.splitlines() if line.startswith("letter=")]
                    details = (name, fraction, reverse, lead, trail, out)
                    assert len(found) <= 1, details
                    assert all(line.startswith(f"letter=Z reversed={int(observed_reverse ^ reverse)} ")
                               for line in found), details
                    assert "timing=0 overflow=0" in out, details
                    if fraction == 500 and not reverse:
                        expected = margin_need == "unmodified" or (margin_need == "both_margins" and lead and trail)
                        assert bool(found) == bool(expected) or (expected and pending_at_eof(
                            name, True, fraction, reverse, lead, trail, out)), details
                        if margin_need == "both_margins" and lead and not trail:
                            assert any(f"decoded=0 rejected=0 timing=0 overflow=0 state={state} " in out
                                       for state in (2, 3)), details
                    checks += 1
    # Four complete reverse-Z scans never finalized on the car: the recorder
    # stayed active despite 2.7--3.5 seconds of observed paper after the bars.
    # Preserve those hardware outcomes in the fixtures. These regressions add
    # leading history explicitly, but require the existing real tail to suffice
    # at midpoint timing; adding synthetic trailing white would hide the bug.
    # The forward success is a control, with its shorter saved tail allowed to
    # finish deferred work only in the explicitly padded variant.
    quiet_tail_cases = {
        "20261111-170538-large-z-decoded.txt": (False, False),
        "20261111-170621-large-z-no-result.txt": (True, True),
        "20261111-170913-large-z-no-result.txt": (True, True),
        "20261111-170958-large-z-no-result.txt": (True, True),
        "20261111-171039-large-z-no-result.txt": (True, True),
    }
    for name, (observed_reverse, require_real_tail) in quiet_tail_cases.items():
        trace = parse_trace((Path(__file__).parent / "fixtures/barcode" / name).read_text())
        assert trace["complete"] and len(trace["episodes"]) == 1, name
        assert trace["summary"]["decoded"] == int(not require_real_tail), name
        assert trace["summary"]["rejected"] == 0, name
        samples = trace["samples"]
        darkness = [max(0, min(1000, (s["raw"] - 215) * 1000 // 3335)) for s in samples]
        for fraction in (0, 500, 999):
            intervals = [s["dt_ms"] * 1000 + fraction for s in samples]
            for reverse in (False, True):
                values = list(reversed(darkness)) if reverse else darkness
                steps = ([intervals[0]] + list(reversed(intervals[1:]))) if reverse else intervals
                data = "".join(f"{dt} {dark}\n" for dt, dark in zip(steps, values))
                for lead, trail in ((0, 0), (1000, 0), (0, 3000), (1000, 3000)):
                    out = subprocess.run([str(exe), str(lead), str(trail)], input=data,
                                         text=True, capture_output=True, check=True).stdout
                    found = [line for line in out.splitlines() if line.startswith("letter=")]
                    details = (name, fraction, reverse, lead, trail, out)
                    assert len(found) <= 1, details
                    assert all(line.startswith(f"letter=Z reversed={int(observed_reverse ^ reverse)} ")
                               for line in found), details
                    assert "timing=0 overflow=0" in out, details
                    if fraction == 500 and not reverse and lead and (require_real_tail or trail):
                        assert len(found) == 1, details
                    checks += 1
    print(f"PASS: {checks} recorded replay checks; padded results are conditional on synthetic quiet margins.")


if __name__ == "__main__":
    main()
