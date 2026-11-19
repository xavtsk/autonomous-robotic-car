#!/usr/bin/env python3
"""Plan an owned-source documentation/whitespace batch without editing by default.

Only --apply writes source, after every proposed file passes token and provenance
checks. Mixed imported files are excluded from whole-file formatting. Function
documentation and new braces are deliberately outside this bounded batch.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import textwrap

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from testing.compliance import inventory

PLAN = ROOT / "docs/compliance/FORMAT_PLAN.json"
TOKEN = re.compile(
    r'(?P<comment>/\*[\s\S]*?\*/|//[^\n]*)|'
    r'(?P<literal>(?:u8|[LuU])?"(?:\\[\s\S]|[^"\\])*"|(?:[LuU])?\'(?:\\[\s\S]|[^\'\\])*\')|'
    r'(?P<number>(?:\d|\.\d)(?:[eEpP][+-]|[A-Za-z0-9_.])*)|'
    r'(?P<identifier>[A-Za-z_]\w*)|'
    r'(?P<operator>%:%:|>>=|<<=|\.\.\.|->|\+\+|--|<<|>>|<=|>=|==|!=|&&|\|\||\*=|/=|%=|\+=|-=|&=|\^=|\|=|##|<:|:>|<%|%>|%:)|'
    r'(?P<other>\S)', re.DOTALL)


def fingerprint(text):
    """Bind C preprocessing tokens plus directive boundaries/macro adjacency."""
    # C99 phase 1 precedes line splicing. A trigraph assembled by removing
    # whitespace must not pass merely because its punctuation tokens match.
    trigraphs = dict(zip("=/'()!<>-", "#\\^[]|{}~"))
    phase_one = re.sub(r"\?\?([=/'()!<>-])", lambda item: trigraphs[item[1]], text)
    spliced = re.sub(r"\\\r?\n", "", phase_one)
    tokens = list(TOKEN.finditer(spliced))
    values = [(token.lastgroup, token[0]) for token in tokens if token.lastgroup != "comment"]
    masked = TOKEN.sub(lambda item: re.sub(r"[^\n]", " ", item[0])
                       if item.lastgroup in {"comment", "literal"} else item[0], spliced)
    directives = []
    for match in re.finditer(r"(?m)^[ \t]*(?:#|%:)([^\n]*)", masked):
        original = spliced[match.start():match.end()]
        directive_items = [item for item in TOKEN.finditer(original) if item.lastgroup != "comment"]
        directive_tokens = [item[0] for item in directive_items]
        function_like = (len(directive_items) > 3 and directive_tokens[1] == "define" and
                         directive_tokens[3] == "(" and
                         directive_items[2].end() == directive_items[3].start())
        # An angle header-name is one preprocessing token, including whitespace
        # inside <...>; the ordinary punctuation lexer alone cannot bind it.
        header_name = None
        if len(directive_items) > 2 and directive_tokens[1] == "include":
            remainder = original[directive_items[2].start():]
            if remainder.startswith("<") and ">" in remainder:
                header_name = remainder[:remainder.index(">") + 1]
        directives.append((directive_tokens, function_like, header_name))
    return hashlib.sha256(json.dumps([values, directives], separators=(",", ":")).encode()).hexdigest()


def comment_texts(text):
    return [re.sub(r"\s+", " ", match[0]).strip()
            for match in TOKEN.finditer(text) if match.lastgroup == "comment"]


def leading_brief(text):
    """Read an existing initial purpose/brief; never infer behavior from a name."""
    first = re.match(r"\s*(/\*[\s\S]*?\*/|//[^\n]*(?:\n\s*//[^\n]*)*)", text)
    if not first:
        return None
    content = re.sub(r"(?m)^\s*\* ?", "", first[1].strip().removeprefix("/**").removeprefix("/*").removesuffix("*/"))
    purpose = re.search(r"(?mi)^\s*(?:Purpose:|@brief)\s*(.+)", content)
    if purpose:
        parts = [purpose[1].strip()]
        for line in content[purpose.end():].splitlines()[1:]:
            line = line.strip()
            if not line or line.startswith("@") or re.match(r"[A-Za-z][A-Za-z ]*:", line):
                break
            parts.append(line)
        value = " ".join(parts)
    else:
        lines = [line.strip().removeprefix("//").strip() for line in content.splitlines()]
        lines = [line for line in lines if line and not re.match(r"(?:File:|Owner:|@file|Copyright|[-*]{3})", line, re.IGNORECASE)]
        if not lines or any(word in " ".join(lines).lower() for word in ("copyright", "license", "licence")):
            return None
        value = re.split(r"(?<=[.!?])\s+", " ".join(lines), maxsplit=1)[0]
    if len(value) < 12 or value.startswith(("#", "typedef", "if ")):
        return None
    return value if value.endswith((".", "!", "?")) else value + "."


def documented_module(text):
    return any(re.match(r"/\*\*|/\*!|///|//!", token[0]) and
               re.search(r"(?:@file|\\file)\b", token[0])
               for token in TOKEN.finditer(text) if token.lastgroup == "comment")


def add_module_and_eof(text, name, brief):
    bom = "\ufeff" if text.startswith("\ufeff") else ""
    output = text.removeprefix(bom).replace("\r\n", "\n")
    if brief and not documented_module(output):
        lines = textwrap.wrap(brief, width=70)
        block = "/** @file " + Path(name).name + "\n * @brief " + lines[0] + "\n"
        block += "".join(" * " + line + "\n" for line in lines[1:]) + " */\n"
        first = re.match(r"\s*/\*[\s\S]*?\*/", output)
        if first and re.search(r"copyright|licen[cs]e", first[0], re.IGNORECASE):
            output = output[:first.end()] + "\n\n" + block + output[first.end():].lstrip("\n")
        else:
            output = block + output
    comments = [token for token in TOKEN.finditer(output) if token.lastgroup == "comment"]
    last = comments[-1] if comments else None
    has_eof = last and not output[last.end():].strip() and re.search(r"\b(?:end of file|eof)\b", last[0], re.IGNORECASE)
    if not has_eof:
        output = output.rstrip() + "\n\n/* End of file. */\n\n"
    else:
        output = output.rstrip() + "\n\n"
    return bom + output


def format_text(text, path, formatter):
    # These overrides are intentional: braces and split string literals change
    # tokens and belong in separately proven batches. Preserve existing comments.
    style = "{BasedOnStyle: InheritParentConfig, InsertBraces: false, BreakStringLiterals: false, ReflowComments: false}"
    result = subprocess.run([formatter, "--style=" + style, "--assume-filename=" + str(path)],
                            input=text, capture_output=True, text=True, check=False)
    if result.returncode:
        raise ValueError(result.stderr.strip() or "clang-format failed")
    return result.stdout.rstrip() + "\n\n"


def propose(root, plan, scope, formatter=None):
    allowed = {path.relative_to(root).as_posix() for path in inventory.owned_files(root, scope)}
    rows, changes, gaps = [], [], []
    errors = inventory.repository_errors(root, scope)
    if errors:
        raise ValueError("; ".join(errors))
    if {row["path"] for row in plan["files"]} != allowed:
        raise ValueError("Formatting plan differs from current owned/mixed scope; regenerate/review it")
    for row in plan["files"]:
        name = row["path"]
        if not row.get("format_allowed") or name in scope.get("mixed_files", {}):
            gaps.append({"path": name, "reason": row.get("boundary", "Whole-file formatting not authorized")})
            continue
        path = root / name
        before_bytes = path.read_bytes()
        before = before_bytes.decode("utf-8")
        if hashlib.sha256(before_bytes).hexdigest() != row["before_sha256"]:
            raise ValueError("Source changed after formatting plan: " + name)
        after = add_module_and_eof(before, name, row.get("module_brief"))
        if not documented_module(after):
            gaps.append({"path": name, "reason": "Module purpose needs an individually written/reviewed brief"})
        if formatter:
            after = format_text(after, path, formatter)
        if fingerprint(before) != fingerprint(after):
            raise ValueError("Preprocessing tokens or macro/directive semantics changed: " + name)
        remaining = list(comment_texts(after))
        for old_comment in comment_texts(before):
            if old_comment not in remaining:
                raise ValueError("Existing comment/notice changed: " + name)
            remaining.remove(old_comment)
        evidence = {"path": name, "before_sha256": hashlib.sha256(before_bytes).hexdigest(),
                    "after_sha256": hashlib.sha256(after.encode()).hexdigest(),
                    "before_lf_sha256": hashlib.sha256(before.replace("\r\n", "\n").encode()).hexdigest(),
                    "after_lf_sha256": hashlib.sha256(after.replace("\r\n", "\n").encode()).hexdigest(),
                    "token_sha256": fingerprint(before), "tokens_identical": True,
                    "direct_line_file_macros": sorted(set(re.findall(r"\b__(?:LINE|FILE)__\b", before))),
                    "module_documented": documented_module(after)}
        rows.append(evidence)
        if before != after:
            changes.append((path, before_bytes, after.encode()))
    return rows, changes, gaps


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, default=PLAN)
    parser.add_argument("--formatter", default=shutil.which("clang-format"))
    parser.add_argument("--comments-only", action="store_true", help="Plan comments/EOF only; do not launch formatter")
    parser.add_argument("--apply", action="store_true", help="Write the fully validated proposal; never the default")
    parser.add_argument("--evidence-dir", type=Path, default=ROOT / "build/compliance-format")
    args = parser.parse_args(argv)
    try:
        if not args.comments_only and not args.formatter:
            raise ValueError("No formatter available; use --comments-only for a partial dry run")
        scope = inventory.read_json(ROOT / "testing/compliance/inventory_scope.json")
        plan = inventory.read_json(args.plan)
        version = None
        if not args.comments_only:
            probe = subprocess.run([args.formatter, "--version"], capture_output=True,
                                   text=True, check=False)
            version = probe.stdout.strip()
            if probe.returncode or not re.search(r"\b18\.1\.3\b", version):
                raise ValueError("This batch requires clang-format 18.1.3; observed: " + version)
        rows, changes, gaps = propose(ROOT, plan, scope, None if args.comments_only else args.formatter)
        report = {"schema_version": 1, "mode": "apply" if args.apply else "dry-run",
                  "before_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                  "formatter_executed": not args.comments_only, "files": rows,
                  "formatter_version": version,
                  "formatter_config_sha256": hashlib.sha256((ROOT / ".clang-format").read_bytes()).hexdigest(),
                  "formatter_overrides": {"InsertBraces": False, "BreakStringLiterals": False, "ReflowComments": False},
                  "changed_files": len(changes), "review_gaps": gaps,
                  "runtime_equivalence_certified": False, "production_source_written": args.apply}
        if args.apply:
            # Bind the recorded baseline before changing any file. Normalize
            # only checkout CRLF, which Git may convert under attributes.
            for row in rows:
                baseline = subprocess.check_output(
                    ["git", "show", report["before_commit"] + ":" + row["path"]], cwd=ROOT)
                baseline = baseline.decode("utf-8").replace("\r\n", "\n").encode()
                if hashlib.sha256(baseline).hexdigest() != row["before_lf_sha256"]:
                    raise ValueError("Uncommitted source is not in the recorded baseline: " + row["path"])
            destination = args.evidence_dir.resolve()
            if not destination.is_relative_to((ROOT / "build").resolve()):
                raise ValueError("Evidence directory must remain below repository build/")
            destination.mkdir(parents=True, exist_ok=True)
            for path, before, after in changes:
                if path.read_bytes() != before:
                    raise ValueError("Concurrent source change during preparation: " + str(path))
                backup = destination / "before" / path.relative_to(ROOT)
                backup.parent.mkdir(parents=True, exist_ok=True)
                if backup.exists():
                    raise ValueError("Refusing to overwrite existing before evidence: " + str(backup))
            for path, before, after in changes:
                backup = destination / "before" / path.relative_to(ROOT)
                backup.write_bytes(before)
                path.write_bytes(after)
            (destination / "manifest.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 2 if gaps or args.comments_only else 0
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print(json.dumps({"status": "INCOMPLETE", "error": str(error)}))
        return 2


if __name__ == "__main__":
    sys.exit(main())
