#!/usr/bin/env python3
"""Check application formatting and module filenames without editing files.

This enforces the repository's selected formatting rules, not every BARR rule.
Use an installed clang-format supporting the checked-in configuration.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def module_errors(paths):
    """Check application module names, matching headers and own-header includes."""
    errors = []
    prefixes = {}
    for path in paths:
        name = path.stem
        if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
            errors.append(f"{path}: module name must use lowercase words")
        previous = prefixes.setdefault(name[:8], name)
        if previous != name:
            errors.append(f"{path}: first eight characters collide with {previous}")
        if path.suffix == ".c":
            if not path.with_suffix(".h").is_file():
                errors.append(f"{path}: matching header is missing")
            own_header = rf'#\s*include\s+"{re.escape(name)}\.h"'
            if not re.search(own_header, path.read_text(encoding="utf-8")):
                errors.append(f"{path}: source must include its own header")
    return errors


def text_errors(path):
    errors = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if "\t" in line:
            errors.append(f"{path}:{number}: tab character")
        if len(line) > 80:
            errors.append(f"{path}:{number}: {len(line)} columns (maximum 80)")
    return errors


def canonical_format(text):
    """Apply the few BARR spacing rules not expressed by clang-format 18."""
    from tools.compliance.barr_check import GROUPED_POINTER_DECLARATOR, mask_noncode

    # clang-format pads '?' in consecutive arms of a conditional expression.
    # Preserve leading indentation, comments and every character of a literal.
    if "  ?" in text:
        code = mask_noncode(text)
        text = re.sub(r"(?<=\S) {2,}(?=\?)",
                      lambda gap: " " if code[gap.end()] == "?" else gap[0],
                      text)
    if "(*" in text:
        # Separate the pointer star in a grouped declaration. Explicit
        # dereferences such as (*callback)(arg) keep unary operator spacing.
        code = mask_noncode(text)
        for match in reversed(list(GROUPED_POINTER_DECLARATOR.finditer(code))):
            if text[match.start():match.end()] != match[0]:
                continue
            start, end = match.span("group")
            line_start = text.rfind("\n", 0, start) + 1
            line_end = text.find("\n", end)
            if line_end < 0:
                line_end = len(text)
            if line_end - line_start + len("( * ") - (end - start) > 80:
                start = match.start("space")
                text = text[:start] + " ( * " + text[end:]
            else:
                text = text[:start] + "( * " + text[end:]
    if "break;" in text and "switch" in text:
        lines = text.split("\n")
        blocks = [None]
        continued_directive = False
        for index, code in enumerate(mask_noncode(text).split("\n")):
            if continued_directive or code.lstrip().startswith("#"):
                continued_directive = code.rstrip().endswith("\\")
                continue
            label = re.match(r"^( *)(?:case\b|default\s*:)", code)
            if label:
                blocks[-1] = label[1]
            if blocks[-1] is not None and re.match(r"^ *break\s*;", code):
                lines[index] = blocks[-1] + lines[index].lstrip(" ")
            for brace in re.finditer(r"[{}]", code):
                if brace[0] == "{":
                    blocks.append(None)
                elif len(blocks) > 1:
                    blocks.pop()
        text = "\n".join(lines)
    return text.rstrip() + "\n\n"


def check(paths, formatter):
    if not paths:
        return 2, ["INCOMPLETE: no C/header files selected for checking."]
    errors = [error for path in paths for error in text_errors(path)]
    if not formatter:
        return 2, errors + ["INCOMPLETE: clang-format is unavailable; use --formatter."]
    for path in paths:
        result = subprocess.run([formatter, "--style=file", str(path)],
                                cwd=ROOT, capture_output=True, encoding="utf-8",
                                check=False)
        if result.returncode:
            errors.append(f"{path}: formatter failed: " + result.stderr)
            continue
        expected = canonical_format(result.stdout)
        current = path.read_text(encoding="utf-8")
        if current != expected:
            old, new = current.splitlines(), expected.splitlines()
            first = next((index+1 for index, pair in enumerate(zip(old, new))
                          if pair[0] != pair[1]), min(len(old), len(new))+1)
            errors.append(f"{path}:{first}: differs from the repository's BARR formatting")
    return (1 if errors else 0), errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formatter", default=shutil.which("clang-format"))
    parser.add_argument("--check", nargs="+", type=Path,
                        help="check only these C/header files")
    args = parser.parse_args(argv)
    # Whole-file formatting is applied only where the whole file is owned.
    # Mixed template files remain in the rule/field audits at owned-region scope.
    from testing.compliance import inventory
    scope = inventory.read_json(ROOT / "testing/compliance/inventory_scope.json")
    paths = args.check or sorted(path for path in inventory.owned_files(ROOT, scope)
                                if path.relative_to(ROOT).as_posix() not in scope.get("mixed_files", {}))
    if not paths:
        parser.error("No C/header files selected")
    try:
        status, errors = check(paths, args.formatter)
        if not args.check:
            # Existing module-name checks have application-only assumptions.
            # The full-scope checker resolves split adapter/header directories.
            modules = module_errors([path for path in paths if path.is_relative_to(ROOT / "app_program")])
            errors.extend(modules)
            if modules and status == 0:
                status = 1
    except (OSError, UnicodeError) as error:
        print(f"INCOMPLETE: {error}")
        return 2
    for error in errors:
        print(error)
    print(f"{'PASS' if status == 0 else 'INCOMPLETE' if status == 2 else 'FAIL'}: "
          f"{len(paths)} files; formatting, tabs, line widths"
          f"{' and module filenames' if not args.check else ''}.")
    return status


if __name__ == "__main__":
    sys.exit(main())
