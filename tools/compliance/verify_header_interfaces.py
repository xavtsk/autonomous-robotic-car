#!/usr/bin/env python3
"""Recheck bounded guard/interface edits; does not execute compiler or C tests."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

try:
    from .format_owned import ROOT, fingerprint
    from .verify_header_pairs import digest, git_source
except ImportError:
    from format_owned import ROOT, fingerprint
    from verify_header_pairs import digest, git_source

PROTOTYPE = "void test_navigation (void);"
NAV_GUARD = "TESTING_SUBSYSTEM3_TEST_NAVIGATION_H"


def compare_guard(before, after, guard):
    if not re.fullmatch(r"TESTING_[A-Z0-9_]+_H", guard):
        raise ValueError("Unexpected fake-header guard name")
    opening = f"#ifndef {guard}\n#define {guard}\n\n"
    closing = f"#endif /* {guard} */\n\n"
    if after.count(opening) != 1 or after.count(closing) != 1:
        raise ValueError("Guard delimiters are missing or duplicated")
    inverse = after.replace(opening, "", 1).replace(closing, "", 1)
    if inverse != before or fingerprint(inverse) != fingerprint(before):
        raise ValueError("Header content changed beyond its new guard")


def replace_exact(text, old, new):
    if text.count(old) != 1:
        raise ValueError("Interface edit anchor is missing or ambiguous")
    return text.replace(old, new, 1)


def compare_navigation(before, after, name, brief):
    if fingerprint(brief) != fingerprint(""):
        raise ValueError("Function documentation contains C tokens")
    if name == "testing/subsystem3/test_core.c":
        expected = replace_exact(before, '#include "test_core.h"\n',
                                 '#include "test_core.h"\n#include "test_navigation.h"\n')
        expected = replace_exact(expected, PROTOTYPE + "\n\n", "")
    elif name == "testing/subsystem3/test_navigation.c":
        expected = replace_exact(before, "#include <assert.h>\n",
                                 '#include "test_navigation.h"\n#include <assert.h>\n')
        expected = replace_exact(expected, "void test_navigation (void)\n",
                                 brief + "void test_navigation (void)\n")
    else:
        raise ValueError("Unexpected source in navigation interface batch")
    if expected != after:
        raise ValueError("Source changed beyond the existing interface move")


def check_navigation_header(header):
    expected = f"#ifndef {NAV_GUARD}\n#define {NAV_GUARD}\n{PROTOTYPE}\n#endif\n"
    if fingerprint(header) != fingerprint(expected):
        raise ValueError("Navigation header changed its one existing prototype")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--candidate", default="WORKTREE")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        if manifest.get("schema_version") != 1 or manifest.get("kind") not in {
                "guards", "navigation_interface"}:
            raise ValueError("Unsupported header-interface manifest")
        if not manifest.get("files"):
            raise ValueError("Empty header-interface manifest")
        if manifest["kind"] == "navigation_interface" and {
                row["path"] for row in manifest["files"]} != {
                "testing/subsystem3/test_core.c", "testing/subsystem3/test_navigation.c"}:
            raise ValueError("Navigation proof must include both defining and calling sources")
        get = (lambda name: (ROOT / name).read_text(encoding="utf-8")) if args.candidate == "WORKTREE" else (lambda name: git_source(args.candidate, name))
        guards = set()
        for row in manifest["files"]:
            before = git_source(manifest["before_commit"], row["path"])
            after = get(row["path"])
            if digest(before) != row["before_lf_sha256"] or digest(after) != row["after_lf_sha256"]:
                raise ValueError("Recorded file content differs: " + row["path"])
            if manifest["kind"] == "guards":
                if row["guard"] in guards:
                    raise ValueError("Guard is reused across fake headers")
                guards.add(row["guard"])
                compare_guard(before, after, row["guard"])
            else:
                compare_navigation(before, after, row["path"], manifest["source_brief"])
        if manifest["kind"] == "navigation_interface":
            if manifest["header_path"] != "testing/subsystem3/test_navigation.h":
                raise ValueError("Unexpected navigation header path")
            header = get(manifest["header_path"])
            if digest(header) != manifest["header_lf_sha256"]:
                raise ValueError("Recorded navigation header differs")
            check_navigation_header(header)
        report = {"status": "PASS", "kind": manifest["kind"],
                  "files": len(manifest["files"]), "candidate": args.candidate,
                  "compiler_observations_repeated": False, "c_tests_executed": False}
        print(json.dumps(report, indent=2) if args.json else
              f"PASS: {manifest['kind']}; {len(manifest['files'])} bounded existing-file edits")
        return 0
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}) if args.json else f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
