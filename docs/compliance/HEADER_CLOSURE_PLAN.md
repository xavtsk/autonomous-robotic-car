# Header pairing and guard closure

## Current ownership correction

The subsequent [upstream/adoption review](evidence/RESTORED_TEMPLATE_BOUNDARIES.json)
establishes that all seven adapter implementations listed in the historical
table below are imported. Their baseline text has been restored with identical
preprocessing tokens, and they are preserved boundaries, **not header-edit
targets**. The prior adapter proposals are withdrawn. This also removes their
missing-pair diagnostics from the owned remediation scope; it does not approve
a deviation for any team-owned implementation.

The sys_arch include-resolution receipt remains valid evidence about the
recorded compiler context. It establishes neither ownership nor authorization
to alter the source or its imported header. The 268-file/55-candidate counts
below describe the earlier batch-1 snapshot and are historical after the
ownership correction. Current counts come from the scope/inventory manifest.
The host-interface section below records the completed pairing and guard
checks. Review of the exported declarations continues. In particular, changed
`mqtt_utk.c/.h` and `cyw43_utk.c` retain their explicit owned scope.

BARR 4.2.a requires one same-root header for each source; it does not require
the two files to occupy one directory. Rule 4.3.c separately requires direct
inclusion of that header. Rule 4.2.c limits the exported interface to what other
modules need, so a standalone test can have a guarded header declaring no
additional project API. Test code remains in scope. Neither `main` nor a fixed
SDK callback is renamed merely to satisfy an internal namespace convention.
Those exact ABI cases remain pending under [BARR-P08/P10](BARR_EXCEPTIONS.md).

After [batch 1](HEADER_PAIRS_BATCH1.md), the observed scope contains 268 C/header
files, including four mixed files whose imported bodies are excluded from
file-level edits. The lexical scan still finds 55 sources without a colocated
same-root header and 13 headers without a conventional guard. These are review
candidates, not 55 proven absent module interfaces. No rule is closed solely
because the lexical diagnostic becomes a review finding.

## Historical adapter review: proposals withdrawn

| Source | Observed interface and next change |
| --- | --- |
| `lib/liblwip/sysdepend/pico_rp2040/sys_arch.c` | Existing pair: `include/arch/sys_arch.h`. The actual Linux target command resolves `arch/sys_arch.h` to that file; see the [bound probe](evidence/SYS_ARCH_HEADER_PAIR.json). Add its direct include in a separate target batch; review existing `sys_now`, `lwip_utk_rand` and `lwip_utk_assert` declarations instead of creating another header. |
| `lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c` | Build a same-root header from existing `tm_usb_*` declarations. Preserve kernel types and the current boundary between kernel and SDK types. Do not edit an imported caller merely to make it use the new header. |
| `lib/libtm/sysdepend/pico_rp2040/usb/usb_tinyusb_glue.c` | Move the necessary `tm_tusb_*` interface declarations out of handwritten `IMPORT` blocks in the owned console source. Match the existing definitions and SDK callback declarations; add no new exported operation. |
| `lib/libtm/sysdepend/pico_rp2040/usb/usb_descriptors.c` | Preserve TinyUSB callback names and signatures already declared by its headers. Pair the module with a guarded header documenting its fixed callback surface; prove that any included SDK declarations are unchanged. |
| `lib/libtm/sysdepend/pico_rp2040/usb/usb_sdk_compat.c` | Resolve each IRQ, assertion and panic declaration to its SDK contract before selecting the paired header's dependencies. Preserve weak/noreturn and calling-convention attributes. |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c` | Inventory each existing SDK/CYW43 callback declaration and weak override. Preserve exact boundary names, attributes and type compatibility; no blanket callback waiver. |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c` | `cyw43_utk_start` is already declared by `cyw43_utk.h`; the source was split to avoid kernel/SDK `size_t` collisions. Add the same-root kernel interface without merging those incompatible include environments. |

The sys_arch resolution probe uses the source's recorded command from Linux
run `44584191817`, artifact `14561042452`, with explicit local path/toolchain
mapping. It substitutes an include-only in-memory probe and does not edit or
compile a target program. Actual direct inclusion is still open.

For any future owned-adapter batch, retain exact before/after declarations and compiler
contexts, run syntax checks across supported profiles, compare preprocessed
declarations, and require the matched target-image gate. Added declarations can
affect optimizer decisions even when their C types are equivalent; the earlier
float-alias investigation shows why source-only reasoning is insufficient.

## Host interfaces and include guards

All 129 wholly team-owned C sources now include their matching header directly
and unconditionally. The final batch paired 26 test fixtures. Fixtures that
supply driver, kernel or SDK callbacks reuse the existing declarations.
Standalone tools expose no additional project API.

For the 23 active fixtures and their affected callers, all 39 before-and-after
compiler comparisons produced identical preprocessed code and object bytes.
The remaining three legacy fixtures already failed to compile at the deployed
baseline. Their new headers compile independently, including when included
twice. The complete fixtures retain the same preprocessed code and compiler
diagnostics as before this batch. Exact paths and results are under
`fixture_interface_followup` in the [grouped evidence](evidence/phase2-mechanical.json).

All 174 headers in scope have distinct enclosing guards. The two observer
fragments still need export and storage review under 4.2.c. Implementation
inclusion by test harnesses remains open under 4.3.f.

## Module names

Five test headers now have unique eight-character prefixes. The motion and
navigation size reports use `wheel_sizes_main.c` and `nav_sizes_main.c` and
keep their original Make targets and executable names. The path map and
compiler comparisons are in the `test_filename_followup` entry of the
[grouped evidence](evidence/phase2-mechanical.json).

Rule 4.1.b remains separate from pairing. Confirmed first-eight-character
collisions include the `test_barcode_*` family and the `control_*` test modules
versus production `control_debug`. Identical basenames such as `host_test`,
`runtime_test`, `controller_test` and fake `tmonitor` require
explicit module identities and build-context mapping; a source/header pair in
different directories is not automatically two modules.

Rename confirmed owned test-module collisions as separate bounded filename
batches, choosing unique prefixes before applying edits. Update paired headers,
include sites, Makefiles, PowerShell scripts and shared historical runners
together. Preserve baseline file lookup per revision, as in the estimator
rename. Record each fixed external include-name constraint individually rather
than exempting all fakes or all tests. Pairing does not grant a naming waiver.

The remaining interface work includes filename collisions, test harnesses
that include implementations, and the export and prototype review for shared
callbacks. The [exception register](BARR_EXCEPTIONS.md) tracks decisions
that need review or approval.

The subsequent host filename review assigned unique prefixes to the barcode
tests and distinct identities to the wheel, terrain, obstacle and Wi-Fi tests.
It also updated the Make, PowerShell, Python and dashboard test source lookups.
Existing Make targets and executables retain their original names. The
current path map and the remaining simulated SDK/kernel include names are in
`phase2_source_review` in the [grouped receipt](evidence/phase2-mechanical.json).
