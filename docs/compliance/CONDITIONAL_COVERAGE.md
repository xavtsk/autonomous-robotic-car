# Conditional-compilation evidence

The Linux matrix collects preprocessing observations from each of the twelve
target profiles and the GCC/Clang host configurations. A separate union job now
checks that every expected profile supplied a successful receipt for the exact
same revision and complete source-hash inventory. It independently reconstructs
the owned conditional arms and rejects missing, duplicate, stale or inconsistent
receipts. A successful individual collector is not complete matrix coverage.
Receipt directory names must also match the declared profile and revision.

Each receipt retains its actual compiler arguments, command outcomes, arm IDs
and observations. The collector operates on a disposable copy, hashes the
measured originals before and after, and never rewrites their contents or
modification times. Imported files are excluded; owned regions of mixed files
remain covered. The public network template is copied into the disposable
settings include, and that derivation is recorded. Private laptop settings are
never uploaded or substituted into CI.

Before inserting markers, the collector preprocesses the untouched copy and
records active macro definitions, including imported-header and command-line
definitions. Conditions or macro-selected includes depending on `__LINE__`,
explicit source line controls, or unsupported token-pasting expansion fail
closed. This prevents marker line shifts from silently selecting another arm.
The union requires matching successful pristine/instrumented commands and no
line-safety errors. Harmless reporting macros are not treated as conditions.

The union fails with an `INCOMPLETE` result if an owned arm remains unobserved.
Its `SUMMARY.md` lists the exact source line and directive, while `results.json`
retains all supporting receipts and profile associations. No missing arm is
automatically waived as unreachable. Supported alternate settings require a
recorded build or a reviewed scope decision. Preprocessor coverage does not
establish runtime path execution, task timing or physical validation.

```sh
python tools/compliance/coverage_union.py --receipts branch-receipts --output evidence-union
python -m unittest testing.compliance.test_coverage_union testing.compliance.test_conditional_coverage
```

The output directory must be new. The workflow keeps the complete build/test
artifacts and additionally uploads small per-profile branch receipts, which the
union job downloads into separate directories. Actions remain pinned to full
commit SHAs with read-only repository permissions. The new download action uses
the verified [v4.3.0 release commit](https://github.com/actions/download-artifact/commit/d3f86a106a0bac45b974a628896c90dbdf5c8093),
following [GitHub's secure-use guidance](https://docs.github.com/en/actions/reference/security/secure-use).

The audit also corrects a formatter conflict: BARR 3.3.c requires a blank line
after the end-of-file comment, but clang-format 18 removes that line. The check
now compares the formatter's output with the required final blank restored.
Missing/excess final blank lines and every other formatting difference still
fail. Whole-file formatting checks cover all wholly owned C/header files;
mixed imported files remain subject to the separate owned-region audits.
