# Remaining standalone test headers

The thirteen proposed pairs have now been applied in
[header batch 2](HEADER_PAIRS_BATCH2.md); filename changes remain pending.
The rest of this document retains the pre-application selection audit.

This proposal was recorded at `302100ecf10f088f26a5b6278a8efa4149cdc06b`.
No C/header source was changed for this audit. The
[source-bound manifest](evidence/STANDALONE_HEADER_PROPOSAL.json) records all
47 remaining owned test/trace sources without a colocated same-root header,
their exact observed signatures, compiler arguments, source/dependency hashes,
and exclusions. Imported files are excluded by the current scope manifest.

The audit parsed 138 contexts covering 45 sources. Thirteen sources have only
`main` as a repository-origin external function definition and no external
file-scope variable definitions in every observed context. These contexts have
zero parse errors and zero warnings. Four of the thirteen have actual Linux
CI command receipts; nine use their explicit repository build recipes. The
recipes establish how to compile a file, **not that CI executed that file**.

The local observations use libclang 18.1.1 with mapped command-line flags and
MinGW host headers. They do not claim a Linux ABI observation or exhaustive
coverage of unrecorded conditional configurations. Actual command receipts
come from run `44589867778` at `3fda4d5`; the manifest separates these receipts
from the current-checkout parse and from repository recipes.

## Proposed no-export pairs

Each source below can receive one same-root guarded header stating that it
exports no additional project API. The planned source change is exactly one
direct include. No `main` prototype or additional operation is invented.
Every existing assertion, function and expression stays unchanged.

| Source | Context evidence | Later filename proposal |
| --- | --- | --- |
| `testing/change_requirement_1/bend_filter_check_main.c` | `run_all.sh` bend-check recipe | `bend_filter_main` |
| `testing/compliance/barcode_replay_trace_main.c` | Differential replay recipe | `replay_trace_main` |
| `testing/compliance/packet_builder_main.c` | GCC, Clang and sanitizer CI commands | `builder_main` |
| `testing/compliance/debug_wire_trace_main.c` | GCC, Clang and sanitizer CI commands | `ctrl_trace_main` |
| `testing/subsystem1/size_report_main.c` | Subsystem 1 `sizes` recipe | `s1_sizes_main` |
| `testing/subsystem2/size_report_main.c` | Subsystem 2 `sizes` recipe | `s2_sizes_main` |
| `testing/subsystem3/replay_barcode_record_main.c` | GCC, Clang and sanitizer CI commands | `record_replay_main` |
| `testing/subsystem3/size_report_main.c` | Subsystem 3 `sizes` recipe | `s3_sizes_main` |
| `testing/subsystem3/quiet_zone_main.c` | `test-buddy3.ps1` recipe | `quiet_clock_main` |
| `testing/subsystem3/record_zone_main.c` | `test-buddy3.ps1` recipe | `quiet_record_main` |
| `testing/subsystem3/test_codec_main.c` | GCC, Clang and sanitizer CI commands | `codec_main` |
| `tools/compliance/traces/obstacle_trace_main.c` | Differential obstacle recipe | `obstacle_trace_main` |
| `tools/compliance/traces/terrain_trace_main.c` | Differential terrain recipes, debug on/off | `terrain_trace_main` |

The last two wrappers obtain their sole `main` definition from the included
subsystem host test. They have no additional external definitions, but their
source inclusion remains separately open under BARR 4.3.f. Header pairing does
not approve that practice. Exact `main` signatures and locations, including
the included source origins, are in the manifest's declaration registry.

The filename proposals are a **separate, unapplied batch**. Each contains
`main` for 4.1.d and has a unique first-eight-character prefix against the
observed owned modules for 4.1.b. Eight of the thirteen current names belong
to existing prefix-collision groups; these groups are listed in the manifest.
The later rename must update recipes, includes, test runners and current docs,
with per-revision compatibility for historical differential subjects. Neither
filename rule is closed by adding headers.

## Excluded from this batch

Thirty sources define real test callbacks/shared functions or external
variables. Their exact source declarations and canonical types are retained
for a later caller/interface review. The target-only `struct_sizes.c` is one
of these: it defines external `size_*` arrays and no `main`. No empty-interface
header is proposed for these sources.

`testing/compliance/decode_work_main.c` defines only `main` in its own text,
but its included `bc_decoder.c` adds eleven production external definitions.
It is separately classified as an included-production-interface case rather
than counted as a strictly standalone translation unit. Existing production
headers and inclusion behavior need review before selecting its paired header.

`testing/subsystem2/host_regression_main.c` fails its existing `legacy-test`
context: the call at line 375 passes three arguments to a four-argument
function. This audit does not repair or silently omit that error.
`testing/integration/controller_main.c` and
`testing/subsystem2/wheel_adapter_main.c` have no identified current
compiler command or repository build recipe and remain unobserved.

## Bounded application and checks

Before applying, bind the manifest to the chosen immutable checkpoint and
revalidate source/dependency hashes. Add only the direct include and guarded
header. The existing `verify_header_pairs.py` can verify exact include
inversion and reject any declaration hidden in the new header. Check guard
uniqueness against the complete current source scope.

Repeat the actual source contexts for before/after syntax and AST comparisons.
Compare expanded tokens and all existing function signatures. If an include
shifts assertion diagnostic line numbers, report that explicitly; any
`__LINE__` normalization is limited to that comparison and must not conceal
line-dependent behavior. Check `__FILE__` sensitivity separately before a
later rename. Compile coverage must explicitly include the recipe-only files.

The final gate is relevant Linux GCC/Clang/sanitizer execution plus unchanged
semantic traces, alongside the bounded header proof. Production source is
outside this header batch. Evidence closes only the listed paired scope under
4.2.a/4.3.c; documentation, naming, source inclusion and remaining real
interfaces keep their own unresolved status.

The raw local audit is retained at
`build/compliance-next-headers/actual-command-observations.json`; the tracked
manifest stores its hash and the exact arguments/signatures needed to review
the classification. No compiled C program was executed locally.
