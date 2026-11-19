# Standalone test header pairs: batch 2

Thirteen standalone host tests and trace wrappers now directly include a
guarded header with the same root name. The headers declare no additional
project API and contain no C declarations, variables, types or expressions.
Only one include line was added to each source. No assertions changed.

The [manifest](HEADER_PAIRS_BATCH2.json) binds every pair to pre-batch revision
`b4ad5d09a9953cfd308bf4e2d19cd9ffcd4d0c60`. The
[selection audit](STANDALONE_HEADER_PLAN.md) retains the exact compiler
contexts, declarations and excluded cases. In particular,
`barcode_work_test.c` is excluded because its translation unit also contains
production external definitions.

Before and after syntax checks passed in all 22 recorded contexts using local
GCC 15.2.0. Libclang 18.1.1 reported zero warnings/errors and found identical
function signatures and body tokens, with only `main` externally defined and
no repository-origin external variable definitions in each translation unit.
The two trace wrappers obtain `main` through the existing included host test;
this does not close the separate source-inclusion finding under 4.3.f.

Seventeen contexts retain identical raw expanded token streams. In the other
five, every changed token is the source-line argument of MinGW's `_assert`
call, increasing by exactly one. The manifest retains each changed token's
assertion-call context. Expanded tokens match with `__LINE__=0` for this
comparison only; ordinary syntax/AST checks keep the real line values.
No `__FILE__` tokens, assertion conditions or other expanded tokens changed.

All thirteen new headers pass the canonical clang-format 18.1.3 wrapper, and
their guards are unique against the current tracked and untracked C/header
scope. Exact include inversion restores each source's complete previous text.
No compiled C program was linked or executed locally. The parser uses local
MinGW headers; these observations are not a Linux ABI or exhaustive profile
claim. Four sources have Linux CI command receipts and nine use explicit
repository recipes, as distinguished in the audit.

This batch addresses 4.2.a, 4.2.b and 4.3.c only for the thirteen listed pairs.
It adds no exported content forbidden by 4.2.c. Filename/main requirements,
first-eight-character collisions, standard entry-point naming and remaining
real callback interfaces retain their separate pending reviews. There is no
blanket test-folder exemption or whole-rule closure.

```sh
python tools/compliance/verify_header_pairs.py --manifest docs/compliance/HEADER_PAIRS_BATCH2.json
python -m unittest testing.compliance.test_header_pairs testing.compliance.test_header_interfaces testing.compliance.test_barr_check
```

The bounded verifier and all 33 focused Python audit tests pass. The verifier
checks source/header contents and exact include inversion;
it does not repeat the compiler observations. Linux GCC/Clang/sanitizer suites
and semantic traces remain the runtime gate. The nine recipe-only sources
also require explicit compile coverage. Production source is outside this
batch.
