# Standalone test header pairs: batch 1

Twenty standalone test sources now directly include a guarded header with the
same root name. This addresses BARR 4.2.a, 4.2.b and 4.3.c for these pairs. The
headers accurately declare no additional project API, as permitted by 4.2.c.
They add no function prototypes, variables, types or executable expressions.
The standard C `main` entry point retains its name; this batch does not close
its separate 6.1.i boundary review or any source-inclusion findings under 4.3.f.

The [manifest](HEADER_PAIRS_BATCH1.json) names every source and header, binds
their contents to pre-batch revision `d26a980`, and records the compiler
contexts. Fresh libclang 18.1.1 parsing found only `main` as an external function
defined in each selected source; every other function defined there was static.
Definitions in included production sources are outside that statement.

Each source changes by exactly one own-header include. Removing that include
restores the complete previous text and preprocessing tokens. New header guards
were checked against tracked C/header contents and are unique to their paths.
GCC 15.2.0 syntax-only checks pass before and after the change in all 20 recorded
host contexts. No C program was linked or executed locally.

Added includes shift source and assertion diagnostic line numbers by one.
Preprocessed C token streams match when `__LINE__` is set to zero for that
comparison only; normal syntax checks retain the real macro and warning flags.
This is a bounded diagnostic-metadata normalization, not a claim of identical
raw preprocessor output. Production files and test assertions are unchanged.
Linux test execution and sanitizer results remain the required runtime gate.

```sh
python tools/compliance/verify_header_pairs.py --candidate df346fb6dced5376b9b9b5e32014d296829b2f24
python -m unittest testing.compliance.test_header_pairs testing.compliance.test_barr_check
```

The focused verifier and 27 audit trust-boundary tests pass locally. The command
above pins the published batch for historical verification, including after
the navigation test gains its separate shared interface. The verifier checks the bound source/header changes;
it does not rerun the compiler observations recorded in the manifest.

Shared CI fixtures must carry their new paired headers. In particular,
`testing/compliance/bcap_storage_test.c` now includes `bcap_storage_test.h`.
Its raw SHA-256 is
`0e2fb4b73f62ed45a1756fbae6fbdb51355eeb5eb795415762bb9d7ddc2f3cb3`.

The [remaining header plan](HEADER_CLOSURE_PLAN.md) separates actual module
interfaces, compiler-resolved split include trees, fixed external callbacks and
test naming collisions. It grants no testing-folder exception.
