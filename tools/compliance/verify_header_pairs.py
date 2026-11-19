#!/usr/bin/env python3
"""Verify a bounded no-export header batch without executing compiled C tests."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

try:
    from .format_owned import ROOT, fingerprint
    from .barr_check import without_comments
except ImportError:
    from format_owned import ROOT, fingerprint
    from barr_check import without_comments


def digest(text):
    return hashlib.sha256(text.replace("\r\n", "\n").encode()).hexdigest()


def compare_source(before, after, row):
    """The one new include is the only permitted source edit."""
    lines = after.splitlines(keepends=True)
    position = row["include_line"] - 1
    expected = '#include "' + Path(row["header_path"]).name + '"\n'
    if position < 0 or position >= len(lines) or lines[position] != expected:
        raise ValueError("Recorded own-header include is missing or changed")
    del lines[position]
    if "".join(lines) != before:
        raise ValueError("Source changed beyond the single own-header include")
    if fingerprint(before) != fingerprint("".join(lines)):
        raise ValueError("Source tokens changed after include inversion")
    return fingerprint(before)


def check_empty_header(header, guard):
    """No declaration, expression, include or additional macro may be hidden."""
    if not re.fullmatch(r"[A-Z][A-Z0-9_]*", guard):
        raise ValueError("Invalid guard identifier")
    actual = [line.strip() for line in without_comments(header).splitlines()
              if line.strip()]
    if actual != ["#ifndef " + guard, "#define " + guard, "#endif"]:
        raise ValueError("Header contains content beyond its inclusion guard")


def git_source(revision, path):
    return subprocess.check_output(["git", "show", revision + ":" + path],
                                   cwd=ROOT).decode().replace("\r\n", "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path,
                        default=ROOT / "docs/compliance/HEADER_PAIRS_BATCH1.json")
    parser.add_argument("--candidate", default="WORKTREE")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        results, guards = [], set()
        if manifest.get("schema_version") != 1 or not manifest.get("files"):
            raise ValueError("Unsupported or empty header-pair manifest")
        for row in manifest["files"]:
            before = git_source(manifest["before_commit"], row["path"])
            get = (lambda path: (ROOT / path).read_text(encoding="utf-8")) if args.candidate == "WORKTREE" else (lambda path: git_source(args.candidate, path))
            after, header = get(row["path"]), get(row["header_path"])
            if (digest(before), digest(after), digest(header)) != (
                    row["before_lf_sha256"], row["after_lf_sha256"], row["header_lf_sha256"]):
                raise ValueError("Recorded contents differ: " + row["path"])
            if Path(row["header_path"]) != Path(row["path"]).with_suffix(".h"):
                raise ValueError("Header is not the recorded source's same-root pair")
            if row["guard"] in guards:
                raise ValueError("Guard reused across new headers")
            guards.add(row["guard"])
            check_empty_header(header, row["guard"])
            tokens = compare_source(before, after, row)
            results.append({"path": row["path"], "source_tokens_sha256": tokens,
                            "include_inverts_exactly": True, "new_c_declarations": 0})
        report = {"status": "PASS", "baseline": manifest["before_commit"],
                  "candidate": args.candidate, "files": results,
                  "c_tests_executed": False, "compiler_proof_repeated": False}
        print(json.dumps(report, indent=2) if args.json else
              f"PASS: {len(results)} same-root headers; exact include inversion; no new declarations")
        return 0
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}) if args.json else
              f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
