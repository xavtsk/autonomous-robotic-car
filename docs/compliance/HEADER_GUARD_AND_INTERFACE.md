# Fake guards and the navigation test interface

These two bounded batches start from immutable revision
`3fda4d517e5720c51da7c41033bc0b9c221b11b5`. They change team-owned host test files
only. The [ownership correction](HEADER_CLOSURE_PLAN.md#current-ownership-correction)
withdraws earlier imported-adapter proposals; no restored imported file is part
of either batch. No assertions or production operations change.

The [guard receipt](FAKE_HEADER_GUARDS.json) covers eleven fake headers: seven
for network/Wi-Fi tests, three for subsystem 1 and one for subsystem 2. Each
change adds only a unique conventional `#ifndef`/`#define`/`#endif` guard.
Removing those lines restores the entire old file exactly. Guard names are
unique across tracked C/header files. Seventeen before/after GCC preprocessing
contexts have identical raw C-token fingerprints, with no line-macro
normalization. Normal syntax checks pass. Double-inclusion probes for all
eleven headers pass with GCC and Clang in their three family environments.
The subsystem-2 probe validates its fake header only; it does not claim that
the historical GPIO-IRQ regression executable now works.

The [navigation receipt](NAVIGATION_TEST_INTERFACE.json) moves the existing
`void test_navigation(void)` declaration from `test_core.c` to the new guarded
`test_navigation.h`, which both the defining and calling modules include.
It does not invent or rename a public operation. The header documents the
synthetic cases and assertion-failure behavior; the definition has a brief
reference to that contract.

Both C files were checked in normal and `S3_BASELINE_TRACE` configurations.
Clang observes identical function types and definition-token fingerprints;
GCC syntax checks pass. Expanded C tokens match after removing exactly the
one unchanged interface prototype from its old/new position and normalizing
`__LINE__` for shifted assertion diagnostics. The comparison does not suppress
other declarations, expressions or test assertions. The new header has zero
Clang documentation warnings.

```sh
python tools/compliance/verify_header_interfaces.py --manifest docs/compliance/FAKE_HEADER_GUARDS.json
python tools/compliance/verify_header_interfaces.py --manifest docs/compliance/NAVIGATION_TEST_INTERFACE.json
python -m unittest testing.compliance.test_header_interfaces
```

Both bounded verifiers and six adversarial verifier tests pass. The verifiers
recheck source transformations and hashes; they do not repeat the recorded
compiler observations. Pin `--candidate <batch-commit>` after publication when
later changes touch these files. No C executable was linked or run locally;
the next Linux runtime/sanitizer checks remain required for these two batches.

This supplies scoped evidence for BARR 4.2.a-c and 4.3.c. It does not close
those rules across the repository. The two observer headers remain separate:
the harness generator injects their bodies, and `mqtt_observer.h` contains
static storage. Adding guards alone would not resolve its 4.2.c.ii question.
Existing fake ABI names, first-eight-character collisions and private test
function documentation also remain open.
