#!/usr/bin/env python3
"""Verify the bounded estimator filename change without executing C code."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

try:
    from .format_owned import ROOT, TOKEN, fingerprint
except ImportError:
    from format_owned import ROOT, TOKEN, fingerprint

OLD = "estimate_motion"
NEW = "motion_estimation"


def digest(text):
    return hashlib.sha256(text.replace("\r\n", "\n").encode()).hexdigest()


def compare_source(before, after):
    """Allow filename text only in comments and the precise header include."""
    for occurrence in re.finditer(OLD, before):
        token = next((item for item in TOKEN.finditer(before)
                      if item.start() <= occurrence.start() < item.end()), None)
        line = before[before.rfind("\n", 0, occurrence.start()) + 1:]
        if token is None or (token.lastgroup != "comment" and not
                            re.match(r'\s*#\s*include\s+"estimate_motion\.h"', line)):
            raise ValueError("Old module spelling appears outside a filename comment/include")
    if before.replace(OLD, NEW) != after:
        raise ValueError("Unexpected source change beyond declared filename text")
    inverse = re.sub(r'(?m)(^[ \t]*#[ \t]*include[ \t]+)"motion_estimation\.h"',
                     r'\1"estimate_motion.h"', after)
    if fingerprint(before) != fingerprint(inverse):
        raise ValueError("C tokens differ after exact include inversion")
    return {"tokens_identical_after_include_inversion": True,
            "before_token_sha256": fingerprint(before),
            "inverse_token_sha256": fingerprint(inverse)}


def expected_app_order(paths, renamed):
    """Model the two existing wildcard groups and the one historical sort key."""
    root_sources = sorted(p for p in paths if p.startswith("app_program/") and
                          p.endswith(".c") and len(Path(p).parts) == 2)
    nested = [p for p in paths if p.startswith("app_program/") and
              p.endswith(".c") and len(Path(p).parts) == 3]
    key = lambda value: value.replace("/motion_estimation.c", "/estimate_motion.c") if renamed else value
    return root_sources + sorted(nested, key=key)


def git_source(revision, path):
    return subprocess.check_output(["git", "show", revision + ":" + path], cwd=ROOT).decode("utf-8").replace("\r\n", "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=ROOT / "docs/compliance/MOTION_MODULE_RENAME.json")
    parser.add_argument("--candidate", default="WORKTREE")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        results = []
        for row in manifest["files"]:
            before = git_source(manifest["before_commit"], row["before_path"])
            after = (ROOT / row["after_path"]).read_text(encoding="utf-8") if args.candidate == "WORKTREE" else git_source(args.candidate, row["after_path"])
            if digest(before) != row["before_lf_sha256"] or digest(after) != row["after_lf_sha256"]:
                raise ValueError("Recorded file content differs: " + row["after_path"])
            proof = compare_source(before, after) if Path(row["after_path"]).suffix in {".c", ".h"} else {}
            results.append({"path": row["after_path"], **proof})
        if args.candidate == "WORKTREE":
            from testing.compliance import inventory
            scope = inventory.read_json(ROOT / "testing/compliance/inventory_scope.json")
            paths = [p.relative_to(ROOT).as_posix() for p in inventory.owned_files(ROOT, scope)]
        else:
            paths = subprocess.check_output(["git", "ls-tree", "-r", "--name-only", args.candidate], cwd=ROOT, text=True).splitlines()
        before_paths = subprocess.check_output(["git", "ls-tree", "-r", "--name-only", manifest["before_commit"]], cwd=ROOT, text=True).splitlines()
        old_c = "app_program/subsystem2/estimate_motion.c"
        new_c = "app_program/subsystem2/motion_estimation.c"
        for name in (old_c, old_c.replace(".c", ".h")):
            if name in paths:
                raise ValueError("Legacy module file is still present: " + name)
        collision = [p for p in paths if Path(p).suffix in {".c", ".h"} and
                     Path(p).stem[:8] == "motion_e" and Path(p).stem != NEW]
        if collision:
            raise ValueError("First-eight-character module collision: " + str(collision))
        old_order = expected_app_order(before_paths, False)
        new_order = expected_app_order(paths, True)
        if [p.replace(new_c, old_c) for p in new_order] != old_order:
            raise ValueError("Application source/link order differs beyond the path rename")
        report = {"status": "PASS", "baseline": manifest["before_commit"], "candidate": args.candidate,
                  "files": results, "application_source_order_preserved": True,
                  "target_bytes_compared": False, "c_tests_executed": False}
        print(json.dumps(report, indent=2) if args.json else
              f"PASS: {len(results)} bound files; source includes invert exactly; application order preserved")
        return 0
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}) if args.json else f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
