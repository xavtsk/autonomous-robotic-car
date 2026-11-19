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


def check(paths, formatter):
    errors = [error for path in paths for error in text_errors(path)]
    if not formatter:
        return 2, errors + ["INCOMPLETE: clang-format is unavailable; use --formatter."]
    result = subprocess.run(
        [formatter, "--dry-run", "--Werror", "--style=file", *map(str, paths)],
        cwd=ROOT, capture_output=True, text=True, check=False)
    if result.returncode:
        errors.append(result.stdout + result.stderr)
    return (1 if errors else 0), errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formatter", default=shutil.which("clang-format"))
    parser.add_argument("--check", nargs="+", type=Path,
                        help="check only these C/header files")
    args = parser.parse_args(argv)
    paths = args.check or sorted(path for path in (ROOT / "app_program").rglob("*")
                                if path.suffix in (".c", ".h"))
    if not paths:
        parser.error("No C/header files selected")
    try:
        status, errors = check(paths, args.formatter)
        if not args.check:
            modules = module_errors(paths)
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
