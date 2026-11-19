#!/usr/bin/env python3
"""Recompute a bounded documentation-only source proof from immutable revisions.

This verifies tokens, directive boundaries and retained comments. It does not
certify the accuracy of new prose or generated code/timing equivalence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

try:
    from .format_owned import ROOT, comment_texts, fingerprint
except ImportError:
    from format_owned import ROOT, comment_texts, fingerprint


def digest(text):
    return hashlib.sha256(text.replace("\r\n", "\n").encode()).hexdigest()


def compare(before, after, row):
    """Do not trust a manifest's tokens_identical or review-status fields."""
    if digest(before) != row["before_lf_sha256"]:
        raise ValueError("Baseline hash differs from this bounded batch")
    if digest(after) != row["after_lf_sha256"]:
        raise ValueError("Candidate hash differs from this bounded batch")
    token_hash = fingerprint(before)
    if fingerprint(after) != token_hash:
        raise ValueError("C tokens or preprocessing directive semantics changed")
    # New comments may be inserted, but old notices/contracts retain their order.
    remaining = iter(comment_texts(after))
    for comment in comment_texts(before):
        if not any(item == comment for item in remaining):
            raise ValueError("An original comment was removed, changed or reordered")
    return {"preprocessing_fingerprint": token_hash,
            "tokens_identical": True, "original_comments_preserved_in_order": True}


def read_revision(revision, path):
    return subprocess.check_output(["git", "show", revision + ":" + path],
                                   cwd=ROOT).decode("utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--candidate", required=True,
                        help="Use WORKTREE before publication; pin its commit afterward")
    args = parser.parse_args()
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        files = manifest["files"]
        paths = [row["path"] for row in files]
        if not paths or len(paths) != len(set(paths)):
            raise ValueError("A batch must contain nonempty, unique file entries")
        results = []
        for row in files:
            name = row["path"]
            if Path(name).is_absolute() or ".." in Path(name).parts:
                raise ValueError("Batch paths must stay relative to the repository")
            before = read_revision(manifest["before_commit"], name)
            after = ((ROOT / name).read_text(encoding="utf-8")
                     if args.candidate == "WORKTREE" else read_revision(args.candidate, name))
            results.append({"path": name, **compare(before, after, row)})
        report = {"status": "PASS", "before": manifest["before_commit"],
                  "candidate": args.candidate, "files": results,
                  "scope": "Recorded files only; other changes are not certified",
                  "runtime_tests_executed": False}
    except (KeyError, OSError, ValueError, subprocess.SubprocessError) as error:
        report = {"status": "FAIL", "error": str(error)}
    print(json.dumps(report, indent=2))
    return report["status"] != "PASS"


if __name__ == "__main__":
    sys.exit(main())
