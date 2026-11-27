"""Run every bypass profile and report mission and baseline verdicts separately."""

import argparse
import csv
import json
import math
from pathlib import Path
import subprocess


PROFILES = ("standard", "beam=10", "beam=20", "turn=1.1", "turn=0.9")
BASELINE = Path(__file__).with_name("bypass-baseline-8131e1a.csv")
# Allows host libm rounding; measured GCC/Windows versus GCC/Linux difference
# in clearance was below 2e-13 mm. Keep collision detection exact at zero.
CLEARANCE_EPSILON_MM = 1e-9


def bit(value):
    if value not in ("0", "1"):
        raise ValueError(f"invalid boolean {value!r}")
    return value == "1"


def parse_case(line):
    (_, name, passed, line_seen, at_goal, clearance, controller, obstacle, fault,
     bypassed, right_route, error) = line.split("\t")
    row = dict(name=name, passed=bit(passed), line_reacquired=bit(line_seen),
               reached_goal=bit(at_goal), min_clearance_mm=float(clearance),
               final_state=controller, obstacle_state=obstacle, fault=fault.strip(),
               bypassed=bit(bypassed), correct_side=bit(right_route),
               max_follow_error_mm=float(error))
    if not all(math.isfinite(row[k]) for k in ("min_clearance_mm", "max_follow_error_mm")):
        raise ValueError(f"{name}: non-finite measurement")
    failures = []
    if row["min_clearance_mm"] <= 0:
        failures.append("collision")
    if not row["bypassed"]:
        failures.append("no bypass")
    if not row["reached_goal"]:
        failures.append("goal missed")
    if controller != "FOLLOW":
        failures.append(f"{controller}/{obstacle}/{row['fault']}")
    if not row["correct_side"]:
        failures.append("wrong side")
    if row["max_follow_error_mm"] >= 15:
        failures.append("follow error")
    if row["passed"] != (not failures):
        raise ValueError(f"{name}: measurements disagree with the simulator verdict")
    row["failures"] = failures
    return row


def read_baseline():
    baseline = {profile: {} for profile in PROFILES}
    with BASELINE.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            profile = baseline[row["profile"]]
            name = row["case"]
            if name in profile:
                raise ValueError(f"duplicate baseline case {name}")
            profile[name] = (float(row["min_clearance_mm"]),
                             set(filter(None, row["failures"].split("|"))),
                             bit(row["line_reacquired"]))
    names = set(baseline["standard"])
    if len(names) != 17 or any(set(group) != names for group in baseline.values()):
        raise ValueError("baseline must contain the same 17 named cases in every profile")
    return baseline


def assess(output, exit_code, reference):
    cases = {}
    regressions = []
    summary = None
    for line in output.splitlines():
        if line.startswith("CASE\t"):
            row = parse_case(line)
            if row["name"] in cases:
                raise ValueError(f"duplicate case {row['name']}")
            cases[row["name"]] = row
        elif line.startswith("SUMMARY\t"):
            if summary is not None:
                raise ValueError("duplicate simulator summary")
            _, failed_count, telemetry_errors = line.split("\t")
            summary = (int(failed_count), int(telemetry_errors))

    if set(cases) != set(reference):
        raise ValueError(f"case list changed: missing={sorted(reference.keys() - cases.keys())}, "
                         f"unexpected={sorted(cases.keys() - reference.keys())}")
    failed = sum(not row["passed"] for row in cases.values())
    if summary is None or summary[0] != failed or summary[1] < 0:
        raise ValueError("missing or inconsistent simulator summary")
    if exit_code != int(bool(failed or summary[1])):
        raise ValueError(f"unexpected simulator exit {exit_code}")
    if summary[1]:
        regressions.append(f"{summary[1]} invalid telemetry updates")

    for name, row in cases.items():
        clearance, previous_failures, regained_line = reference[name]
        new_failures = set(row["failures"]) - previous_failures
        if new_failures:
            regressions.append(f"{name}: new failure(s): {', '.join(sorted(new_failures))}")
        if regained_line and not row["line_reacquired"]:
            regressions.append(f"{name}: lost line reacquisition")
        if row["min_clearance_mm"] < clearance - CLEARANCE_EPSILON_MM:
            regressions.append(f"{name}: clearance fell from {clearance:.12g} "
                               f"to {row['min_clearance_mm']:.12g} mm")
    return list(cases.values()), regressions, summary[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--strict", action="store_true", help="fail for any missed mission")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    baseline = read_baseline()
    report = dict(baseline_revision="8131e1a577ee47f59531da44b831ee0eaf4a9a4c",
                  complete=True, profiles={}, total_cases=0, passed_cases=0, regressions=[])
    for profile in PROFILES:
        command = [str(args.executable.resolve()), "--results"]
        if profile != "standard":
            command.append(profile)
        profile_result = dict(command=command, cases=[])
        report["profiles"][profile] = profile_result
        log = args.output.with_name(f"bypass-{profile}.log")
        profile_result["log"] = log.name
        try:
            # Stream each profile into its own log, including crashes and timeouts.
            with log.open("w", encoding="utf-8") as stream:
                process = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                         timeout=120, check=False)
            profile_result["exit_code"] = process.returncode
            rows, errors, telemetry = assess(log.read_text(encoding="utf-8"),
                                              process.returncode, baseline[profile])
            profile_result.update(cases=rows, telemetry_errors=telemetry)
            report["total_cases"] += len(rows)
            passed = sum(row["passed"] for row in rows)
            report["passed_cases"] += passed
            print(f"{profile}: {passed}/{len(rows)} successful bypasses")
            for row in rows:
                if not row["passed"]:
                    print(f"  MISS {row['name']}: {', '.join(row['failures'])}")
            report["regressions"].extend(f"{profile}: {error}" for error in errors)
        except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
            report["complete"] = False
            profile_result["execution_error"] = str(exc)
            report["regressions"].append(f"{profile}: {exc}")
    report["mission_passed"] = (report["complete"] and
                                report["passed_cases"] == report["total_cases"] and
                                not any(group["telemetry_errors"] for group in report["profiles"].values()))
    report["regression_passed"] = report["complete"] and not report["regressions"]
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    for error in report["regressions"]:
        print(f"REGRESSION {error}")
    print(f"Mission {report['passed_cases']}/{report['total_cases']}; "
          f"baseline comparison {'PASS' if report['regression_passed'] else 'FAIL'}; {args.output}")
    return int(not report["regression_passed"] or (args.strict and not report["mission_passed"]))


if __name__ == "__main__":
    raise SystemExit(main())
