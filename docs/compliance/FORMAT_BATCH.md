# Module documentation and whitespace batch

This batch is prepared in [FORMAT_PLAN.json](FORMAT_PLAN.json). Application is
held until the root review of the alias batch's target-image difference completes.
No production formatting or module-comment application has been performed by the
preparation step.

The current proposal has 239 files from the shared owned/mixed scope. Each has
a specific module brief taken from an existing purpose statement or individually
reviewed against its declarations, implementation and test calls. No function
parameter/return semantics are automatically invented; function documentation
remains a separate manual review.

Eleven files are excluded: the concurrently owned motion implementation/test,
the mixed kernel integration file, and eight configuration/ADC/I2C files with
upstream notices requiring a precise whole-file provenance decision. These
exclusions are not compliance waivers. Imported bodies and notices remain intact.

The proposal adds Doxygen-compatible `@file`/`@brief` comments where missing,
adds final EOF comments/blank lines, and uses clang-format 18.1.3. It preserves
all original comments. The formatting overlay intentionally sets `InsertBraces`,
`BreakStringLiterals` and `ReflowComments` to false: new braces, literal changes
and comment rewrites are outside this exact-token batch.

```sh
python tools/compliance/format_owned.py --comments-only
python tools/compliance/format_owned.py --formatter /path/to/clang-format-18
python -m unittest testing.compliance.test_format_plan testing.compliance.test_instrument_harnesses
```

Both tool invocations are read-only by default. The explicit `--apply` option
writes only after all proposals pass and preserves original bytes plus a manifest
under a new repository `build/` evidence directory. Stale source hashes, changed
scope, changed existing comments, imported-boundary drift or unexpected code
tokens stop the operation before source writing.

The fingerprint compares C preprocessing tokens after line splicing. It retains
multi-character operators, literal contents, preprocessor directive boundaries
and the distinction between function-like and object-like macros. Whitespace
insensitivity must never hide `++` versus `+ +`, a lost macro continuation or
`MACRO(x)` versus `MACRO (x)` at definition time. C99 trigraph conversion precedes
line splicing, and angle-header spelling is bound as one header-name token.

An applied manifest records its exact preceding commit, both raw and LF-normalized
source hashes, and every token fingerprint. Before writing, the tool checks the
baseline against Git. `verify_format_batch.py --manifest <applied-manifest>` then
recomputes the evidence from Git and the candidate sources, including preservation
of existing comments. Pin `--candidate <format-batch-commit>` after later changes.
Other changed C/header paths are reported separately and are not certified by this
bounded batch. Checkout CRLF normalization is recorded separately from source tokens.

The full 18.1.3 dry run observed identical fingerprints for every proposed file
and unchanged existing comments/notices. This is source evidence, not executed
car or C-test evidence. File/line metadata can move, including metadata produced
by imported macros. Compare the exact target profile's loadable contents and
report any `__FILE__`/`__LINE__` effects; self-test assertion locations can change
without changing assertion expressions.

`instrument_harnesses.py` now locates unique C-token sequences instead of literal
whitespace-sensitive strings in the runtime and MQTT harness copies. Comments and
string literals cannot manufacture a code match; absent, changed or duplicate
anchors still fail closed. Tests exercise formatting variation, token drift and
actual harness-copy instrumentation without altering the original test files.

The full standard still requires the [BARR ledger](BARR_RULE_MATRIX.md), including
all manual and pending rules. A successful token-preserving formatting batch is
not a full BARR-C certificate.
