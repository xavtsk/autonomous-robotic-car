# Compiler-context repair for function documentation evidence

Run `44604115125`, production artifact `14570590232`, reported 1,708 function definition sites, of which 980 had only recovery observations. The associated documentation artifact `14569583700` correctly remained incomplete. These numbers were not comparable with the checked-in field inventory's three partial files: the function run received only the production Arm command database, while the local field inventory had additional host and targeted compiler observations.

The original function receipt contains 1,021 translation-unit observations, 102 with errors across 74 distinct source/header paths. Of the 980 recovery function sites, 936 are in tests or trace tools that were given an unrelated Arm command. The remaining 44 comprise 13 definitions in `control_debug.c` and 31 in the two owned network adapters. The former expose a parser-header mismatch: GCC's freestanding `stdint-gcc.h` expands integer constant macros through GCC-specific builtins such as `__UINT64_C`, which libclang does not provide. The network adapters lost their public `wifi_secrets.h` input when the target builder removed its temporary configuration before the separate collector ran. These are context/evidence defects, not evidence of new firmware failures.

The previous command cleanup also retained joined `-MF`/`-MT` output operands. All 58 actual application commands differed from the generic command until these operands were removed; afterward their relevant arguments matched. This caused duplicate configured parses. Cleanup now removes joined and separate output/dependency destinations while preserving `-pthread`, whose `_REENTRANT` definition can affect header branches.

## Revised observation contract

`barr_functions.py` now records each original command, mapped command, database hash and entry, target, profile override, diagnostics, owned include inputs, and context origin. The origin distinguishes a recorded source command, an explicitly derived profile, an unrecorded production-source fallback, and a standalone-header fallback. Test or trace files without a relevant recorded command remain source-bound and unobserved. Headers are also observed through their actual consumers.

The separate host GCC and Clang jobs use their actual compiler receipts. Paths from the second checkout are remapped only after verifying the same Git revision, identical owned contents, and unchanged tracked trees. The target collector uses the installed Clang 18.1 resource headers before GCC's private headers while retaining Arm/newlib system headers. The parser package and resource-header provenance are recorded. This changes the observation environment, not the production compiler command or source.

For target observation, the collector derives a copy of `config/wifi_secrets.example.h` outside the source checkout and records both hashes and the added include path. An existing local settings file causes rejection based on presence alone; its contents are not read. Linked inputs/destinations and reused output directories are rejected. The production build's original cleanup remains unchanged.

`function_union.py` requires exactly `target-production`, `host-gcc`, and `host-clang` receipts, from artifact directories matching their profile and revision. It checks the complete current source set, source text/location hashes, included-header bindings, parser provenance, and every function's context references. Missing, stale, contradictory, or duplicate receipts leave a retained `INCOMPLETE` result.

A clean host observation can provide a usable function representation, but it cannot erase a failed target observation. Every recovery, fallback and derived observation remains attached to its function and translation-unit receipt. `diagnostic_translation_unit_count` and `recovery_function_observation_count` retain these outcomes independently of the preferred per-function representation. `coverage_complete` remains false: merging available contexts does not prove every configuration or contract.

## Verification and limits

Seventy-four focused Python tests pass across the context, inventory and documentation tests. They exercise include provenance, retained recovery, missing/stale receipts, unsupported promotion, exact argument cleanup, protected configuration inputs, and source preservation. A libclang test confirms the builtin-header search order using isolated test headers; it does not substitute for the Linux check with installed Clang 18.1 headers. No production C/header file changes and no local compiled C execution belong to this tooling batch.

The corrected Linux host/target collection and union still require their first execution. Previous immutable receipts and their incomplete counts are retained. Genuine missing function contracts, undocumented conditional alternatives, Doxygen warnings, unsupported source contexts, and unobserved functions remain open findings.
