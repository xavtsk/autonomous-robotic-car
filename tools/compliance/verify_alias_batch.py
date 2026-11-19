#!/usr/bin/env python3
"""Verify the bounded C99/float-alias batch against its recorded Git baseline.

This compares tokens after reversing only the declared aliases, added includes
and compile-time assertion declarations. It does not execute C, certify broader
behavior or replace the target-image and differential checks. Use --candidate
with a recorded batch commit after later source changes have been made.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "docs/compliance/ALIAS_ASSERT_MANIFEST.json"
TOKEN = re.compile(r'(?P<comment>/\*[\s\S]*?\*/|//[^\n]*)|'
                   r'(?P<literal>"(?:\\[\s\S]|[^"\\])*"|\'(?:\\[\s\S]|[^\'\\])*\')|'
                   r'(?P<identifier>[A-Za-z_]\w*)|(?P<other>\S)', re.DOTALL)


def tokens(text):
    return [match[0] for match in TOKEN.finditer(text) if match.lastgroup != "comment"]


def token_digest(text):
    return hashlib.sha256(json.dumps(tokens(text), separators=(",", ":")).encode()).hexdigest()


def invert_aliases(text):
    include_ranges = [match.span() for match in re.finditer(r'(?m)^\s*#\s*include\s*[<"][^>"\n]+[>"]', text)]
    pieces, start = [], 0
    names = {"float32_t": "float", "float64_t": "double"}
    for match in TOKEN.finditer(text):
        if match.lastgroup == "identifier" and match[0] in names and not any(begin <= match.start() < end for begin, end in include_ranges):
            pieces.extend((text[start:match.start()], names[match[0]]))
            start = match.end()
    return "".join(pieces) + text[start:]


def assertions(text, spelling):
    """Read balanced assertion calls; ignore commas inside strings/comments."""
    items = list(TOKEN.finditer(text))
    result = []
    for index, token in enumerate(items):
        if token.lastgroup != "identifier" or token[0] != spelling:
            continue
        cursor = index + 1
        while items[cursor].lastgroup == "comment":
            cursor += 1
        if items[cursor][0] != "(":
            raise ValueError(f"Malformed {spelling}")
        opening = items[cursor].end()
        depth, comma = 1, None
        cursor += 1
        while cursor < len(items):
            item = items[cursor]
            if item.lastgroup not in {"comment", "literal"}:
                if item[0] == "(":
                    depth += 1
                elif item[0] == ")":
                    depth -= 1
                    if depth == 0:
                        break
                elif item[0] == "," and depth == 1:
                    if comma is not None:
                        raise ValueError("More than two assertion arguments")
                    comma = item
            cursor += 1
        if comma is None or cursor >= len(items):
            raise ValueError("Unbalanced assertion arguments")
        closing = items[cursor]
        cursor += 1
        while items[cursor].lastgroup == "comment":
            cursor += 1
        if items[cursor][0] != ";":
            raise ValueError("Assertion has no terminating semicolon")
        result.append({"start": token.start(), "end": items[cursor].end(),
                       "condition": text[opening:comma.start()],
                       "argument": text[comma.end():closing.start()].strip(),
                       "original": text[token.start():items[cursor].end()]})
    return result


def compare(before, after, row):
    inverse = invert_aliases(after)
    for include in row["inserted_includes"]:
        exact = include.strip()
        pattern = r"(?m)^" + re.escape(exact) + r"\r?\n"
        inverse, count = re.subn(pattern, "", inverse, count=1)
        if count != 1:
            raise ValueError(f"Required added include changed or missing: {exact}")
    original_assertions = assertions(before, "_Static_assert")
    new_assertions = assertions(inverse, "BUILD_ASSERT")
    if len(original_assertions) != len(row["assertions"]) or len(new_assertions) != len(original_assertions):
        raise ValueError("Assertion count changed")
    for original, current, recorded in reversed(list(zip(original_assertions, new_assertions, row["assertions"]))):
        if current["argument"] != recorded["identifier"]:
            raise ValueError("Assertion identity changed")
        if tokens(original["condition"]) != tokens(current["condition"]):
            raise ValueError("Compile-time assertion condition changed")
        if tokens(original["condition"]) != tokens(recorded["condition"]):
            raise ValueError("Recorded original assertion condition disagrees with Git baseline")
        inverse = inverse[:current["start"]] + original["original"] + inverse[current["end"]:]
    if tokens(before) != tokens(inverse):
        raise ValueError("Unexpected token change beyond declared aliases/assertions/includes")
    return {"baseline_token_sha256": token_digest(before),
            "inverse_candidate_token_sha256": token_digest(inverse),
            "tokens_identical": True}


def git_source(revision, name):
    result = subprocess.run(["git", "show", f"{revision}:{name}"], cwd=ROOT,
                            capture_output=True, check=True)
    return result.stdout.decode("utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    parser.add_argument("--candidate", default="WORKTREE")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        results = []
        for row in manifest["files"]:
            name = row["path"]
            before = git_source(manifest["pre_alias_commit"], name)
            after = (ROOT / name).read_text(encoding="utf-8") if args.candidate == "WORKTREE" else git_source(args.candidate, name)
            results.append({"path": name, **compare(before, after, row)})
        for row in manifest["new_headers"]:
            after = (ROOT / row["path"]).read_text(encoding="utf-8") if args.candidate == "WORKTREE" else git_source(args.candidate, row["path"])
            if token_digest(after) != row["token_sha256"]:
                raise ValueError(f"New header semantics differ from recorded batch: {row['path']}")
        report = {"status": "PASS", "meaning": "declared mechanical token transformations only",
                  "baseline": manifest["pre_alias_commit"], "candidate": args.candidate,
                  "files": results, "runtime_tests_executed": False}
        print(json.dumps(report, indent=2) if args.json else
              f"PASS: {len(results)} files reverse exactly to baseline tokens; no C binary executed")
        return 0
    except (OSError, ValueError, KeyError, IndexError, subprocess.CalledProcessError) as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}) if args.json else f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
