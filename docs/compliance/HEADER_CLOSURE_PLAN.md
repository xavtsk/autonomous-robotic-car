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
The owned fake-header guards, standalone tests and actual shared test APIs
remain the actionable parts of this plan. In particular, changed
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

## Remaining host modules

Reparse each source using its real host command. For files defining only `main`
and private helpers, apply the no-export pair used in batch 1. For actual shared
test APIs, move only the existing necessary declarations to the paired header
and include it in both defining and calling modules. For example,
`testing/subsystem3/test_navigation.c` defines `test_navigation`, which is
declared and called by `test_core.c`; that existing interface must not be
misrepresented as empty. Fake SDK/kernel callbacks keep their established
external declarations and receive a symbol-by-symbol ABI record.

The current lexical scan at `4dd87b3` reports the 34 same-root source candidates
listed below, after the second standalone-header batch. They still require actual
compiler-context/interface review. Implementation-inclusion harnesses remain
independently open under 4.3.f; adding a header does not close private-access review.

```
testing/change_requirement_1/struct_sizes.c
testing/compliance/barcode_work_test.c
testing/compliance/motion_storage_test.c
testing/compliance/mqtt_receive_test.c
testing/diagnostics/firmware_test.c
testing/diagnostics/motion_smoke_test.c
testing/diagnostics/startup_test.c
testing/integration/builtin_test.c
testing/integration/controller_test.c
testing/integration/course_sim.c
testing/integration/course_test.c
testing/integration/map_freeze_test.c
testing/integration/operator_test.c
testing/integration/remote_bridge_test.c
testing/integration/runtime_test.c
testing/integration/team_adapter_test.c
testing/network_wifi/runtime_test.c
testing/subsystem1/host_test.c
testing/subsystem2/controller_adapter_test.c
testing/subsystem2/host_regression.c
testing/subsystem2/pack_test.c
testing/subsystem3/bench_trace.c
testing/subsystem3/host_trace.c
testing/subsystem3/test_drive.c
testing/subsystem3/test_motion_feedback.c
testing/subsystem4/integration_test.c
testing/subsystem4/transport_test.c
testing/subsystem5/bypass_course_test.c
testing/subsystem5/controller_test.c
testing/subsystem5/course_clear_test.c
testing/subsystem5/hardware_test.c
testing/subsystem5/runtime_test.c
testing/subsystem5/telemetry_test.c
tools/compliance/traces/motion_trace.c
```

## Guards and module names

The eleven fake-header guards were completed in their separately verified
batch. The two remaining observer fragments are listed below. They contain
private definitions and, for MQTT, storage; guards alone do not resolve 4.2.c.ii
or the distinction between an interface header and generated instrumentation.
Review their actual insertion contexts before changing those boundaries.

```
tools/compliance/traces/mqtt_observer.h
tools/compliance/traces/runtime_observer.h
```

Rule 4.1.b remains separate from pairing. Confirmed first-eight-character
collisions include the `test_barcode_*` family and the `control_*` test modules
versus production `control_debug`. Identical basenames such as `host_test`,
`size_report`, `runtime_test`, `controller_test` and fake `tmonitor` require
explicit module identities and build-context mapping; a source/header pair in
different directories is not automatically two modules.

Rename confirmed owned test-module collisions as separate bounded filename
batches, choosing unique prefixes before applying edits. Update paired headers,
include sites, Makefiles, PowerShell scripts and shared historical runners
together. Preserve baseline file lookup per revision, as in the estimator
rename. Record each fixed external include-name constraint individually rather
than exempting all fakes or all tests. Pairing does not grant a naming waiver.

Sequence: finish the isolated estimator target gate; publish batch 1 and refresh
source-bound inventory; execute the Linux host/sanitizer gates; then proceed
with guard, actual shared-test-interface, naming and adapter batches separately.
Each batch keeps a concrete inverse proof and records any unresolved case with
its exact symbol/profile. No deviation is self-approved by this plan.
