#!/usr/bin/env python3
"""Render or check the review matrix from its canonical JSON ledger."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]


def render(ledger):
    introduction = """# BARR-C 2018 rule matrix

The canonical ledger is [BARR_RULES.json](BARR_RULES.json). This matrix covers
chapters 1-8, including explicit nested subclauses. Example rows preserve the
standard's structure; they are not additional mandatory requirements. The number
of rows is a coverage count, never a count of compliant requirements.

Baseline for the review: `4ad2cc42595722cedb47102e4343298339040d40`.
The original PDF is the authority. The local extraction used during review is
`build/compliance-review-20261119/barr-c-2018.txt` (not a tracked deliverable).
Owned C/header scope comes from
[inventory_scope.json](../../testing/compliance/inventory_scope.json).
Imported source and externally fixed ABI boundaries must remain explicit.
For a mixed-ownership file, the lexical/AST audit limits findings to its recorded
owned region and separately checks that the surrounding imported body is intact.
The mixed file is never a whole-file formatting target.

## Reading status and strength

- **mandatory:** a direct prohibition or requirement.
- **qualified / conditional:** an obligation qualified by applicability,
  reasonable effort, necessity or feasibility; it still needs evidence.
- **preferred:** a stated preference or recommendation; do not change behavior
  merely to force it. **mixed** rows identify both a preference and an obligation.
- **permission / guidance / example:** permitted syntax, explanatory guidance
  or an illustration; do not count these as independently mandatory rules.
- **unreviewed:** no completed source-wide review. **verified:** named evidence
  establishes the applicable rule across the recorded scope/profiles.
- **not_applicable:** requires a concrete scope/applicability reason.
- **pending_deviation:** a specific unresolved conflict, not a waiver.
  **approved_deviation:** requires named project approval, date and rationale.

The original deviation procedure requires documented and approved exceptions.
See [pending exceptions](BARR_EXCEPTIONS.md). No exception in the initial review
is approved, and preserving behavior does not silently approve one.

## Enforcement boundaries

`python tools/compliance/barr_check.py` scans owned source without changing it.
It reports observed violations, review findings and missing evidence separately.
Exit 1 means violations; exit 2 means incomplete review or unavailable evidence.
A clean partial scan does not certify full BARR compliance. The existing
`tools/check_c_style.py` checks wholly owned C/header files; mixed imported
regions remain under the separate ownership-aware audit.

The lexical pass excludes strings and comments when recognizing C keywords,
checks filenames/includes/text/comment tokens and identifies missing module
documentation. The AST pass consumes Clang JSON and checks actual function and
variable declarations, public function prefixes, function documentation and
length, floating equality operand types and conditional-depth review candidates.
It does not implement every rule marked `ast-review`; that label identifies the
required enforcement method, not a claim that its implementation is complete.
Compiler/profile coverage, promotions, casts, ownership, ISR classification,
function organization, documentation meaning and other manual rows remain open.

The [header review](HEADER_CLOSURE_PLAN.md) records the direct header checks
for all 129 wholly owned sources and the enclosing guard checks for all 174
headers in scope. The [first 20 host pairs](HEADER_PAIRS_BATCH1.md) retain
the evidence for that earlier batch. The export and prototype review,
filename collisions and external callback constraints remain open.

For AST input, pass `--ast-manifest path/to/manifest.json`. The manifest has
`schema_version: 1`, a `source_sha256` mapping for **every owned C/header file**,
and `translation_units` entries with `source`, relative `ast` JSON path,
`profile`, `compiler` and exact `command`. Generate JSON using Clang's
`-Xclang -ast-dump=json -fsyntax-only` under the actual compile definitions.
Use native documentation comments; do not use `-fparse-all-comments` to make
ordinary comments appear documented. The checker rejects stale hashes and
reports missing units/ambiguous include provenance. Conditional profiles and
header-only definitions still require a coverage review. An observed denominator
is never represented as complete function documentation coverage.

## Documentation completion procedure

1. Inventory definitions from all supported translation units and profile
   branches, including private, inline, test and adapter functions.
2. Add Doxygen-compatible module and function comments covering purpose,
   parameters, units/ranges, outputs, mutation, ownership, concurrency and
   timing assumptions as applicable. Document exact retained behavior.
3. Generate Doxygen with warnings for undocumented members/parameters; compare
   its definition inventory with the compiler inventory. A warning-free run
   alone cannot prove all conditional branches were included.
4. Review meaning and complexity coverage under 2.2.a-g; mark 2.2.h verified
   only when all owned definitions and modules are accounted for.

Templates and abbreviation definitions are in [templates](templates/README.md)
and [ABBREVIATIONS.md](ABBREVIATIONS.md). Generate this matrix with
`python tools/compliance/barr_matrix.py --write`; check it with `--check`.

## Rule-by-rule ledger

| Rule | Strength | Enforcement needed | Requirement or explanatory item | Review status |
| --- | --- | --- | --- | --- |
"""
    return introduction + "".join(
        f"| {row['id']} | {row['strength']} | {row['enforcement']} | "
        f"{row['requirement'].replace('|', '/')} | {row['status']} |\n"
        for row in ledger["rules"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    ledger = json.loads((ROOT / "docs/compliance/BARR_RULES.json").read_text(encoding="utf-8"))
    target = ROOT / "docs/compliance/BARR_RULE_MATRIX.md"
    expected = render(ledger)
    if args.write:
        target.write_text(expected, encoding="utf-8", newline="\n")
        print(f"Wrote {target.relative_to(ROOT)}")
        return 0
    if not target.is_file() or target.read_text(encoding="utf-8") != expected:
        print("FAIL: BARR_RULE_MATRIX.md differs from BARR_RULES.json; regenerate it")
        return 1
    print("PASS: matrix matches ledger; this is consistency, not compliance")
    return 0


if __name__ == "__main__":
    sys.exit(main())
