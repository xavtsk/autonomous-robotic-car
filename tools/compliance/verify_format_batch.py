#!/usr/bin/env python3
"""Recheck a bounded formatting manifest against pinned Git source revisions.

Only recorded files are certified by this check. Files changed elsewhere in the
two revisions are reported separately. Passing proves the recorded source-token
and comment relationship, not runtime equivalence or full BARR-C compliance.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

try:
    from .format_owned import ROOT, comment_texts, documented_module, fingerprint
except ImportError:
    from format_owned import ROOT, comment_texts, documented_module, fingerprint


def lf_digest(text):
    return hashlib.sha256(text.replace("\r\n", "\n").encode()).hexdigest()


def compare(before, after, row):
    """Recompute evidence; never trust a recorded tokens_identical Boolean."""
    if lf_digest(before) != row["before_lf_sha256"]:
        raise ValueError("Baseline source differs from recorded batch")
    if lf_digest(after) != row["after_lf_sha256"]:
        raise ValueError("Candidate source differs from recorded batch")
    original = fingerprint(before)
    if original != row["token_sha256"] or fingerprint(after) != original:
        raise ValueError("Preprocessing tokens or directive semantics changed")
    remaining = comment_texts(after)
    for comment in comment_texts(before):
        if comment not in remaining:
            raise ValueError("Existing source comment or notice changed")
        remaining.remove(comment)
    if not documented_module(after):
        raise ValueError("Candidate module documentation is missing")
    return {"tokens_identical": True, "existing_comments_preserved": True,
            "token_sha256": original}


def git_source(revision, name):
    return subprocess.check_output(["git", "show", f"{revision}:{name}"], cwd=ROOT).decode("utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=ROOT / "docs/compliance/FORMAT_MANIFEST.json")
    parser.add_argument("--candidate", default="WORKTREE", help="Pin the batch commit after later source edits")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        if manifest.get("mode") != "apply" or not manifest.get("production_source_written"):
            raise ValueError("A dry-run manifest cannot certify an applied batch")
        results = []
        for row in manifest["files"]:
            name = row["path"]
            before = git_source(manifest["before_commit"], name)
            after = (ROOT / name).read_text(encoding="utf-8") if args.candidate == "WORKTREE" else git_source(args.candidate, name)
            results.append({"path": name, **compare(before, after, row)})
        command = ["git", "diff", "--name-only", manifest["before_commit"]]
        if args.candidate != "WORKTREE":
            command.append(args.candidate)
        changed = subprocess.check_output(command, cwd=ROOT, text=True).splitlines()
        recorded = {row["path"] for row in manifest["files"]}
        outside = [name for name in changed if name.endswith((".c", ".h")) and name not in recorded]
        report = {"status": "PASS", "meaning": "recorded formatting/comment batch only",
                  "baseline": manifest["before_commit"], "candidate": args.candidate,
                  "files": results, "other_changed_c_headers_not_certified": outside,
                  "excluded_paths": manifest.get("review_gaps", []), "runtime_tests_executed": False}
        print(json.dumps(report, indent=2) if args.json else
              f"PASS: {len(results)} recorded files preserve tokens/comments; {len(outside)} other C/header changes not certified")
        return 0
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}) if args.json else f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
