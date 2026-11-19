#!/usr/bin/env python3
"""Verify only the bound tilt filename/include rename; never rename its API."""
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

OLD = "tilt_estimation"
NEW = "tilt"
OLD_C = "app_program/subsystem4/tilt_estimation.c"
NEW_C = "app_program/subsystem4/tilt.c"
ORDER_RECIPE = "build_make/mtkernel_3/app_program/subdir.mk"
SOURCE_MAP = {
    OLD_C: NEW_C,
    OLD_C[:-1] + "h": NEW_C[:-1] + "h",
    "app_program/subsystem4/terr_data.c": "app_program/subsystem4/terr_data.c",
    "app_program/subsystem4/buddy4_test.c": "app_program/subsystem4/buddy4_test.c",
    "testing/subsystem4/host_test.c": "testing/subsystem4/host_test.c",
}


def digest(text):
    return hashlib.sha256(text.replace("\r\n", "\n").encode()).hexdigest()


def filename_transform(text):
    """Permit exact filename mentions in comments or a direct header include."""
    tokens = list(TOKEN.finditer(text))
    for occurrence in re.finditer(r"\btilt_estimation\.[ch]\b", text):
        token = next((item for item in tokens
                      if item.start() <= occurrence.start() < item.end()), None)
        start = text.rfind("\n", 0, occurrence.start()) + 1
        end = text.find("\n", occurrence.start())
        line = text[start:] if end == -1 else text[start:end]
        include = re.fullmatch(r'[ \t]*#[ \t]*include[ \t]+"tilt_estimation\.h"[ \t]*', line)
        if token is None or (token.lastgroup != "comment" and not include):
            raise ValueError("Filename mention outside a comment or exact include")
    return text.replace(OLD + ".h", NEW + ".h").replace(OLD + ".c", NEW + ".c")


def compare_source(before, after):
    if filename_transform(before) != after:
        raise ValueError("Unexpected edit beyond precise filename mentions")
    inverse = re.sub(r'(?m)(^[ \t]*#[ \t]*include[ \t]+)"tilt\.h"',
                     r'\1"tilt_estimation.h"', after)
    original = fingerprint(before)
    if fingerprint(inverse) != original:
        raise ValueError("Code/directive tokens changed after include inversion")
    return {"tokens_identical_after_include_inversion": True,
            "preprocessing_fingerprint": original}


def application_order(paths):
    """Model the existing two wildcards and unchanged estimator sort key."""
    root_sources = sorted(path for path in paths if path.startswith("app_program/")
                          and path.endswith(".c") and len(Path(path).parts) == 2)
    nested = [path for path in paths if path.startswith("app_program/")
              and path.endswith(".c") and len(Path(path).parts) == 3]
    return root_sources + sorted(nested, key=lambda path: path.replace(
        "app_program/subsystem2/motion_estimation.c",
        "app_program/subsystem2/estimate_motion.c"))


def check_paths(before_paths, after_paths):
    if OLD_C not in before_paths or OLD_C[:-1] + "h" not in before_paths:
        raise ValueError("Baseline lacks the complete old module pair")
    if NEW_C in before_paths or NEW_C[:-1] + "h" in before_paths:
        raise ValueError("Baseline already contains a new-name module file")
    if OLD_C in after_paths or OLD_C[:-1] + "h" in after_paths:
        raise ValueError("Legacy source/header still present in candidate")
    if NEW_C not in after_paths or NEW_C[:-1] + "h" not in after_paths:
        raise ValueError("Candidate lacks the complete renamed module pair")
    collisions = [path for path in after_paths if Path(path).suffix in {".c", ".h"}
                  and Path(path).stem[:8] == NEW[:8]
                  and path not in {NEW_C, NEW_C[:-1] + "h"}]
    if collisions:
        raise ValueError("First-eight-character collision: " + str(collisions))
    before_order, after_order = application_order(before_paths), application_order(after_paths)
    if [OLD_C if path == NEW_C else path for path in after_order] != before_order:
        raise ValueError("Application/link source order changed beyond filename")


def git_source(revision, path):
    return subprocess.check_output(["git", "show", revision + ":" + path],
                                   cwd=ROOT).decode().replace("\r\n", "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path,
                        default=ROOT / "docs/compliance/TILT_MODULE_RENAME.json")
    parser.add_argument("--candidate", required=True)
    args = parser.parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        rows = manifest["files"]
        recorded = [(row["before_path"], row["after_path"]) for row in rows]
        if len(recorded) != len(SOURCE_MAP) or dict(recorded) != SOURCE_MAP:
            raise ValueError("Manifest does not describe the exact five-file source scope")
        results = []
        for row in rows:
            before = git_source(manifest["before_commit"], row["before_path"])
            after = ((ROOT / row["after_path"]).read_text(encoding="utf-8")
                     if args.candidate == "WORKTREE" else git_source(args.candidate, row["after_path"]))
            if digest(before) != row["before_lf_sha256"] or digest(after) != row["after_lf_sha256"]:
                raise ValueError("Recorded source contents differ: " + row["after_path"])
            results.append({"path": row["after_path"], **compare_source(before, after)})
        before_paths = subprocess.check_output(["git", "ls-tree", "-r", "--name-only",
                                               manifest["before_commit"]], cwd=ROOT, text=True).splitlines()
        before_recipe = git_source(manifest["before_commit"], ORDER_RECIPE)
        after_recipe = ((ROOT / ORDER_RECIPE).read_text(encoding="utf-8")
                        if args.candidate == "WORKTREE" else git_source(args.candidate, ORDER_RECIPE))
        if before_recipe != after_recipe:
            raise ValueError("Application source-order recipe changed during rename")
        if args.candidate == "WORKTREE":
            sys.path.insert(0, str(ROOT))
            from testing.compliance.inventory import repository_sources
            after_paths = [path for path in repository_sources(ROOT)
                           if (ROOT / path).is_file()]
        else:
            after_paths = subprocess.check_output(["git", "ls-tree", "-r", "--name-only",
                                                  args.candidate], cwd=ROOT, text=True).splitlines()
        check_paths(before_paths, after_paths)
        report = {"status": "PASS", "before": manifest["before_commit"],
                  "candidate": args.candidate, "files": results,
                  "application_order_model_identical": True,
                  "application_order_recipe_unchanged": True,
                  "public_identifier_tokens_unchanged": True,
                  "runtime_tests_executed": False, "target_bytes_compared": False,
                  "scope": "Five named C/header files and modeled source order; runner compatibility is separate"}
    except (KeyError, OSError, ValueError, subprocess.SubprocessError) as error:
        report = {"status": "FAIL", "error": str(error)}
    print(json.dumps(report, indent=2))
    return report["status"] != "PASS"


if __name__ == "__main__":
    sys.exit(main())
