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
    """BARR 3.3.c keeps an extra blank line after the end-of-file comment.

    clang-format 18 removes that final blank line. Normalize its proposed output
    to the selected project rule; do not suppress other formatting differences.
    """
    return text.rstrip() + "\n\n"


def check(paths, formatter):
    errors = [error for path in paths for error in text_errors(path)]
    if not formatter:
        return 2, errors + ["INCOMPLETE: clang-format is unavailable; use --formatter."]
    for path in paths:
        result = subprocess.run([formatter, "--style=file", str(path)],
                                cwd=ROOT, capture_output=True, text=True, check=False)
        if result.returncode:
            errors.append(f"{path}: formatter failed: " + result.stderr)
            continue
        expected = canonical_format(result.stdout)
        current = path.read_text(encoding="utf-8")
        if current != expected:
            old, new = current.splitlines(), expected.splitlines()
            first = next((index+1 for index, pair in enumerate(zip(old, new))
                          if pair[0] != pair[1]), min(len(old), len(new))+1)
            errors.append(f"{path}:{first}: differs from clang-format plus BARR end-of-file blank line")
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
