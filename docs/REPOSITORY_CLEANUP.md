# Repository housekeeping checkpoint

This archive records the housekeeping decisions from 20 November 2026.
The cleanup used `codex/compliance-preserve-behaviour` and compared firmware
against `4ad2cc42595722cedb47102e4343298339040d40`.

Phase 2 later closed for submission using the existing cleanup and baseline
comparison. See the [Phase 2 closeout](CURRENT_STATUS.md#phase-2-closeout-for-submission)
for the recorded exceptions. The checkpoints below give the results and review
status at each stage. The [submission summary](CURRENT_STATUS.md) covers the final code.

## Keep, move and delete decisions

| Decision | Material | Reason |
| --- | --- | --- |
| Keep | Interrupted operator contracts and their source receipt | Complete comments-only work; existing token/directive/comment-order verification passes. No executable operation was added. |
| Keep | Tilt rename, debug metadata and a3de92a Linux evidence | These record already-completed runs, including failures and physical-validation limits. They are not disposable build output. |
| Keep | Baseline-fixture portability correction | Accepts GCC's real implicit system-header dependency while retaining source-root and content-hash validation. |
| Move | Root `CLAUDE_HANDOFF_2026-10-25.md` to [documentation history](history/CLAUDE_HANDOFF_2026-10-25.md) | Obsolete operating instructions no longer sit beside the current README. The original body remains unchanged beneath a historical notice. No active source/build reference used the old path. |
| Delete | Untracked `tools/compliance/verify_mqtt_braces.py` and `testing/compliance/test_mqtt_braces.py` | Interrupted, unused scaffold: its required `MQTT_BRACES_BATCH.json` does not exist, and the intended source batch was never applied. No active runner imports it; only the two scaffold files reference one another. Existing checks will be reused for later brace cleanup. |
| Simplify | Duplicate `.gitignore` rules for `.elf`, `.uf2`, `.map`, `.o` and `.DS_Store` | Each rule remains once; ignore behaviour is unchanged. |
| Keep | Six pairs of identical logs dated 14 and 19 November in `testing/change_requirement_1/logs/` | Identical output alone does not make separately dated evidence disposable. Their provenance remains historical, not a new validation claim. |
| Keep | Existing compliance tools, inventories, fixtures and evidence | No other safe deletion was established by this bounded review. Files referenced by CI or evidence are not removed merely because there are many of them. |
| Keep | Ignored build/toolchain/SDK files, dashboard captures, credentials and rollback firmware | These may be needed locally. No bulk cleanup or change to supplied assessment documents was performed. |

## Verification

- Searched source, scripts and documentation for references before removing or
  relocating files. No tracked firmware binaries, object files, ZIP archives or
  obvious backup files were found in the initial extension check.
- Compared hashes of 221 tracked Markdown/text/JSON files under documentation,
  tools and testing. The only exact-duplicate groups found were the six dated
  log pairs above; this is not a claim that every repository file was audited.
- Reused `verify_comment_batch.py` for the two operator files: tokens,
  preprocessing directives and original comment order match the recorded base.
- All ten existing focused baseline-fixture Python tests passed. These invoke
  the C preprocessor, not a generated native C executable.
- All 74 local inline documentation-link destinations in the five changed or
  added Markdown files exist; anchors and remote URLs were not tested.
- Four retained JSON receipts parse, and all six host records preserve the
  original subsystem3 failure. The historical body is unchanged, the effective
  ignore-rule set is identical, and the whitespace check passes.

No firmware build, Linux CI run, flash, motor operation, network interruption,
push or merge was performed for this housekeeping phase. Earlier Linux results
are retained evidence for their named revisions, not results of this phase.

## Next phases

Phase 2 addresses the existing BARR findings and introduces the smaller routine
CI path using existing runners. Phase 3 completes grouped representation work.
Phase 4 consolidates documentation and prepares the verified release candidate.
Those phases had not been executed at the end of the Phase 1 checkpoint.

The account meter displayed 1% weekly usage both before work and at the final
check. Its integer precision does not establish zero consumption. This phase
used one agent and stops here; changes remain local and uncommitted.

## Phase 2: first source batch

The first BARR batch changes 60 owned C/header files, mostly formatting and
comments. Its [source receipt](compliance/evidence/phase2-mechanical.json)
records the changed files, before/after hashes, the full 270-file lexical scope
and the retained verification results. This is progress within Phase 2, not
completion of that phase or a full compliance claim.

- C tokens and preprocessing directives are unchanged in 57 files. The other
  three contain 17 MQTT if-body brace pairs and 13 parentheses around existing
  expressions in remote-control and motion-packing helpers.
- Seven comments were moved out of continued macros without changing macro
  values. Tabs, indented owned preprocessor directives, forbidden comment
  tokens and missing end markers were corrected. Imported surrounding bodies
  remain unchanged.
- Remote-control documentation now describes nine inline functions/prototypes,
  including unsigned sequence wrap, STOP precedence, borrowed buffers, mutex
  failures and disabled-integration behaviour. Motion has a module contract.
- The existing lexical checker reports 307 violations, down from 477, with 79
  review findings remaining. Rules 2.1.b, 3.2.d, 3.3.c and 3.5.a have no findings
  across the owned scope and are recorded as verified against the source hashes.
  Other rule statuses remain open where their full requirements are unreviewed.
- Three paired Arm production-context object comparisons have identical
  allocated sections, relocations and normalized compiler stack rows. Eleven
  existing focused Python tests pass. No native C executable was run locally.
- Routine CI now selects four jobs: baseline/candidate GCC, semantic comparison
  and paired production builds. Manual `suite=full` retains the full matrix,
  sanitizers, audits and evidence collectors. Action pins and failure handling
  are unchanged; the first GitHub run is recorded below.

Remaining Phase 2 work includes identifier/API conventions, header interfaces,
long string/macro lines, complete function contracts, type-rule boundaries and
larger-function decomposition. The source-bound CR1/compiler inventories need
refreshing after line movements; their historical measurements are not current
source validation. BARR-P13 failure-diagnostic source locations remain pending.

After the user's separate push instruction, Phase 1 and this Phase 2 batch were
published as `903057d35ecc1c8551f58c357ae1517c58bd16f5`.
[Linux run 44699456784](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44699456784)
executed the four routine jobs. Both GCC host runs fail only at the original
subsystem3 replay gate; all other recorded host steps pass. All 16 semantic
comparisons and all 1008 replay outputs match the fixed baseline. The replay
audit retains the same two expectation failures, so the workflow remains red.

The paired production build succeeds. Independently hashed downloaded flash
and UF2 bytes match the previous a3de92a cleanup build. Static RAM remains
129496 bytes, with 465788 text bytes and 696 data bytes. This is the public
example network profile, not a newly deployed image. Sanitizers and alternate
target profiles were not repeated by quick mode. The existing source receipt
now includes the run, artifact hashes and exact evidence limits.

No firmware was flashed and no vehicle operation was performed. Phase 2 remains
in progress; the Linux gate for this bounded source batch has been examined.

## Phase 2: MQTT interface follow-up

This local batch starts at `888d744f87b8d6958cf4e0769f94d368c0a7f09d` and edits
four MQTT source/header files. The existing
[grouped receipt](compliance/evidence/phase2-mechanical.json) retains the earlier
results and adds `mqtt_interface_followup`; no separate report framework was
introduced.

- Replaced owned `unsigned short` lengths/casts with `uint16_t`. Compiler-only
  probes confirm identical canonical types, ranges, packet size/member offsets
  and callback compatibility on Arm GCC 14.3.1 and Windows MinGW GCC 15.2.
- Prefixed three private bridge helpers and the affected pointer parameters.
  Public callback names, packet fields, cast positions, queue selection and
  all constants remain unchanged. The header now correctly describes this as
  a team-owned interface, rather than claiming upstream ownership.
- Added contracts to 17 definitions and 11 public prototypes, including
  consumption on rejection, partial output, weak-hook defaults and ownership.
  Corrected an existing void-function `@return` comment. Clang 18.1.1 reports
  zero documentation diagnostics in six checked target contexts.
- Compared both translation units before/after in production, diagnostics and
  motion-disabled contexts. All six pairs have identical allocated section
  contents, relocation meaning and stack usage after normalizing only the
  three private names. Eleven host-test compile contexts also pass syntax
  checking. This did not execute the host tests.
- The existing lexical audit now reports 293 violations and 79 review findings
  across 270 owned files, compared with 307 and 79 previously. Imported-body
  integrity checks pass. Remaining rule statuses and pending exceptions stay
  open; these checks do not establish full compliance.

Changes remain local. The last Linux results still apply to published source
revision `903057d`, not this follow-up. No full firmware link, runtime replay,
sanitizer run, flash or physical validation was performed for this batch.
The field/compiler inventories still need their planned source-binding refresh.

## Phase 2: Wi-Fi owner and retry policy

The next local batch edits `cyw43_utk.c` and `wifi_retry_policy.h`, preserving
the earlier MQTT changes. Its `wifi_owner_followup` entry in the same
[grouped receipt](compliance/evidence/phase2-mechanical.json) records:

- Nine previously unbraced control bodies now have braces. Five private
  helpers, the retained status variable and affected local/pointer/Boolean
  identifiers use the relevant naming conventions.
- Two uppercase private macros preserve the original dual-core kernel calls
  and single-core processor value/memory barrier. Imported public interfaces
  and status storage remain unchanged.
- Four long diagnostic strings are split into adjacent C string literals;
  concatenated contents and numeric tokens match. All eleven definitions in
  these two files have compiler-recognized contracts, including scan-only and
  optional diagnostic paths, unsigned clock wrap and noncoherent status reads.
- Eight paired Arm comparisons cover both core configurations with MQTT,
  lwIP without MQTT, startup diagnostic scanning and scan-only operation.
  Allocated sections, relocations and stack rows match after accounting only
  for renamed private symbols. Three existing host source contexts compile;
  they were not executed. Eight Clang contexts have no documentation warnings
  in the edited files; two warnings in unchanged imported SDK headers remain
  recorded separately.
- The lexical audit reports 287 violations and 79 review findings across 270
  files, down from 293 and 79. Neither edited file has a lexical finding.
  Imported-file integrity and the preceding source-binding chain still pass.

The large owner task remains intact. These checks do not complete its broader
BARR review or prove physical timing/reconnection behavior. Linux execution,
full firmware linking and deployment remain pending. No commit, push, flash
or car operation was performed, and no new tracked verification tool was added.

## Phase 2: ordinary strings and comment line lengths

This batch wraps 85 long string literals and seven operator-contract comment
lines across 48 files. Diagnostic messages, JSON examples, test descriptions
and comment wording are preserved. The changes are limited to adjacent C string
concatenation and whitespace; assertions, expected results and fixtures' data
values are unchanged.

The existing [grouped receipt](compliance/evidence/phase2-mechanical.json) adds
`line_wrap_followup`. Sixty-six host compiler contexts and two Arm production
contexts have matching preprocessed tokens after ordinary string concatenation.
The comparison retains assertion expressions and line numbers: string changes
before later assertions were deliberately left out. Temporary source paths are
mapped to the same repository path on both sides. Both Arm object comparisons
also preserve allocated sections, relocations and compiler stack rows.

All checked host sources pass syntax checking. No C executable, replay test or
sanitizer was run for this formatting batch. Eight files without a direct
compiler context in the retained quick-run database were left untouched.

The lexical audit now reports **195 violations and 79 review findings** across
270 files, down from 287 and 79. Long-line findings fell from 146 to 54. Those
remaining lines, other BARR rules and pending exceptions still need resolution;
this is not a full-compliance claim. The source-binding chain and imported-file
integrity checks pass. Changes remain local, uncommitted and unflashed.

## Phase 2: fixed-width host interface types

Five host-test files now use equivalent fixed-width aliases in nine declarations:
the three fake RTOS headers define `UH`/`H` through `uint16_t`/`int16_t`, and the
fake MQTT keep-alive/port declarations and matching implementation use `u16_t`.
No field range, storage width, precision, name, assertion or source line count
changed. Firmware source files were untouched by this batch.

Eight C99 compiler-only probes verify canonical type/range compatibility on
MinGW GCC and Arm GNU GCC; the MQTT probe also checks structure size, member
offset and callback signature. All 21 selected host configurations compile
before and after. Their primary test files' preprocessed tokens match after
only the intended type substitutions. This is compilation evidence, not test
execution. Early temporary-include setup failures and the final passing logs
are retained in the `host_type_followup` entry of the existing
[grouped receipt](compliance/evidence/phase2-mechanical.json).

The lexical audit reports **186 violations and 79 review findings** across 270
files. Three `long` findings remain in the historical bend-filter comparison
and barcode replay CLI. Their platform-dependent counter/parser/format behavior
needs review before replacement; no deviation has been approved. All source
bindings and imported-file integrity checks pass. Changes remain local; Linux
runtime checks, final firmware linking and physical validation remain pending.

## Phase 2: obstacle acquisition and recovery contracts

Eight source/header files now document the scan state machine, ultrasonic
capture/conversion, servo driver and line recovery. The review covers 22
definition variants (including disabled hardware stubs) and 15 public prototypes.
It records ownership, units, retry/time limits, output changes on failure,
partial hardware setup and the line-confirmation latch. Existing callers and
host/hardware tests were reviewed; no production operations or tests changed.

The `obstacle_contract_followup` entry in the existing
[grouped receipt](compliance/evidence/phase2-mechanical.json) records identical
C tokens/directives and preserved original comments in all eight files.
Six matched Arm object comparisons preserve allocated sections, relocations
and stack rows, including enabled and disabled hardware variants. All six
compiler documentation contexts are clean; nine existing host-test translation
units compile. These checks do not execute C tests or validate physical behavior.

The lexical audit remains at **186 violations and 79 review findings** across
270 owned files. That checker does not count these manually reviewed contracts
as closed whole-project documentation requirements. All source bindings and
imported-file integrity checks pass. Phase 2 remains in progress; storage
compaction and final documentation/release consolidation remain later phases.

## Phase 2 host replay and bend-filter tools

The three remaining rule 5.2.b keyword findings were in two host tools. The
bend-filter comparison now uses signed 32-bit counters for its 20,000 trials.
The replay tool still parses both arguments with native `strtoul` and stores
each result losslessly before applying its 10,000 ms limit. Both tools use
type-matched decimal output. Six padding argument cases in the existing replay
runner cover defaults, partial parses, the upper limit and native overflow.

The lexical audit found **182 violations and 79 review findings** in 270
owned files, with zero `short` or `long` keyword findings. Rule 5.2.b is
verified for the recorded source hashes, including owned regions of mixed
files. Review covered continued lines and the single token-pasting macro.
Both tools linked on
Windows and compiled with strict Arm format checks. The earlier Arm replay
source failed the timestamp format check. Windows Application Control blocked
native execution with error 4551. The runtime comparison is pending. See
`host_cli_type_followup` in the [grouped evidence](compliance/evidence/phase2-mechanical.json).

## Phase 2 test headers and include guards

Eight test sources gained matching headers. Nine Windows host compilations
and one Arm compilation produced byte-identical object files before and after
the change with debug information disabled. The target size probe preserved
all 14 measurement symbols and their sizes. The preprocessed tokens also
matched, including the source locations used in assertions.

Rule 4.2.b is verified for all 148 headers in scope. The checker now requires
the guard to enclose the entire file. The two observer fragments compiled
when inserted twice in the actual generated harnesses. Their five ordinary
compiler configurations also produced matching object files.

The audit now reports **175 violations and 69 review findings** across 278
files. There are 26 remaining source/header pairing candidates. The new
`barcode_work_test.h` shares its source file's existing filename-prefix
collision. Linux execution of the local changes remains pending. The
[header closure plan](compliance/HEADER_CLOSURE_PLAN.md) lists the remaining
sources, and `test_header_followup` in the
[grouped evidence](compliance/evidence/phase2-mechanical.json) records the checks.

## Phase 2 completion of test header pairing

The final 26 fixtures now include their matching header. All 129 wholly owned
C sources pass the direct include check, and all 174 headers in scope have
distinct enclosing guards. For the 23 active fixtures and their affected
callers, 39 compiler comparisons matched in both preprocessed code and
object bytes. All 29 audit and header-pair tests passed.

Three legacy fixtures also failed to compile at the fixed behaviour baseline.
Their new headers compile on their own, and each full fixture still produces
its pre-edit diagnostics. Linux runtime verification of the working tree
remains pending.

The lexical audit records **154 violations and 46 review findings** across
304 files. Eight new header paths share filename issues already present in
their corresponding source names. The [header review](compliance/HEADER_CLOSURE_PLAN.md)
and the `fixture_interface_followup` entry in the
[grouped evidence](compliance/evidence/phase2-mechanical.json) identify
the checked scope and remaining work.

## Phase 2 test filenames

Renaming five test headers and two size-report source/header pairs resolved
nine filename findings. All 20 affected host compiler comparisons matched.
The size reports also produced byte-identical linked executables for both
the fixed baseline and candidate using the same compiler settings and
omitting debug symbols and linker timestamps. Their Make targets and output
names are unchanged. Six Make dry runs and 24 Python tests passed. Windows
Application Control blocked execution of the local size reports (error 4551).

The barcode fixture checker now reads the checksum across a wrapped C comment.
It matched all four pages of the supplied sample PDF and rejected a different
PDF. The lexical audit covers 304 files and reports **149 violations and 42
review findings**, including 54 filename findings. The exact old and new paths
and verification results are in `test_filename_followup` in the
[grouped evidence](compliance/evidence/phase2-mechanical.json).

## Phase 2 source review and Linux verification

The 20 November working tree meets the 80-column limit across all 304
owned files or file regions. All 63 host entry-point modules use `main` in
their filenames, including the course test that inherits its entry point.
The Make targets and executable names stay available under their existing
names. The project records the changed source and header paths under
`phase2_source_review.path_renames` in the [grouped evidence](compliance/evidence/phase2-mechanical.json).

Eleven compile-time checks now use the C99 assertion helper. One hundred
kernel storage-class aliases in application declarations now use their
standard C equivalents. The motion controller has 43 documented definitions;
the navigation drive has 28 with avoidance enabled and 23 with it disabled.
The decoder validation helper preserves the work per update. Three host
tests now link their trace observers as separate translation units.

| Software check | Result |
| --- | --- |
| Fixed baseline versus candidate | Sixteen trace and packet comparisons match across 80,959 rows. The independent replay comparison matches 1,008 cases and retains both failed baseline expectations. |
| Target objects | Eighteen paired Arm compilations match on allocated sections, relocations and compiler stack estimates. |
| Linux host and dashboard | Five subsystem suites, integration, diagnostics and Wi-Fi recovery passed. Motion and barcode sanitizer checks passed. All 27 dashboard wire tests passed. |
| Audit tooling | 103 Python tests passed with pinned libclang 18.1.1. |

The lexical audit reports **43 violations** across test code: 27 direct C
implementation includes and 16 lowercase macro names. It also flags seven
shared fake SDK/kernel filenames and 24 reviewed uses of `continue` under
a preference. The rule ledger records **23 verified, three inapplicable and
141 still awaiting source-wide review**. Remaining work includes the legacy
test formatting, API naming, function contracts, expression and nesting
review, and disposition of specific preservation conflicts. Physical course
behaviour and a new linked release image have separate acceptance gates.

## Phase 2 host fixtures and control statements

The follow-up on 20 November reduced the lexical findings from 43 to **22**.
Eight test programs now link at least one previously included implementation
as a separate C module. These cover application startup, motion smoke,
ultrasonic/servo hardware, the IR bench, navigation tracing, MQTT fragments,
the clear-course simulator and map freezing. The motion-storage fixture also
links the existing feedback suite and uses its fake clock, encoder, queue
and mutex interface. It still includes the motion implementation to seed
private boundary values. The receipt lists the **14 remaining implementation
includes and eight lowercase macro names** individually.

All 300 wholly owned files pass the repository formatter. The four mixed
files retain separate owned-region checks. The 22 brace pairs added to three
historical test modules each enclose one existing statement. Their
preprocessing-token comparisons preserve every other token. The legacy wheel
build produces the same 27 GCC errors before and after. The two archive-only
harnesses have source comparisons only.

The source review covered all 304 files or owned regions. All 31 switches
have a default. All 12 empty loops have braces and a comment explaining their
purpose. The bit-field review found 265 named unsigned or Boolean fields and
one unnamed, zero-width unsigned alignment field in the wheel encoder. The
alignment field belongs in Phase 3's layout evidence.

| Check | Result |
| --- | --- |
| Fixed baseline and final source | All 16 semantic and wire traces match across 80,959 rows. All 1,008 barcode replay cases match, including both original failed expectations. |
| Affected host programs | Standard output matched for 13 before/after runs, including 58,915 navigation trace rows in the Make comparison and 2,093 MQTT fragment cases. |
| Target compilation | All 13 paired Arm builds matched on allocated sections, relocations and compiler stack estimates. These cover the explicit final branches in driving, obstacle handling and decoding, plus the startup, MQTT and host-hook declarations. |
| Sanitizers | GCC AddressSanitizer and UndefinedBehaviorSanitizer passed for drive, IR/barcode, obstacle, terrain transport, MQTT fragments and motion storage. |

The BARR ledger now records **29 verified, three inapplicable and 135
unreviewed** entries. The compiler audit now detects unbraced control bodies,
assignments in conditions and switches missing their own default.
Further work includes ownership of private test interfaces, identifier rules,
expression and control-flow review, and function contracts.
[Source bindings and results](compliance/evidence/phase2-mechanical.json)

## Phase 2 owner review

Split 465 declaration groups, clarified 1,467 compound Boolean operands and
placed constants first in 422 simple equality checks. The operator task and
12 private host-test functions now meet their reviewed naming rules. The
switch review accounted for all 232 labels, including the six groups whose
cases share handling. Five runtime modules have compiler-attached function
contracts in every profile exercised in this batch.

All 61 production translation units compiled. Allocated sections, relocations
and stack estimates matched after accounting for the operator task's private
symbol rename. All nine Linux suites passed, and the fixed-baseline comparison
matched for every exercised trace and barcode case. The overall replay gate
still reports the two existing expectation failures. Rule 8.5.b is pending
review for host task harnesses and simulator exits.
[Rule ledger](compliance/BARR_RULE_MATRIX.md) · [Source and compiler evidence](compliance/evidence/phase2-mechanical.json)

## Phase 2 control flow and headers

Each of the 49 outstanding conditional chains now has an explicit fallback.
For the ultrasonic scan, the forward-only update sits inside the final branch
to retain the same Arm instructions across its three build profiles. Another
98 comma declarations were split. The obstacle-course simulator now links as
a separate translation unit. Ten simulator runs retained their output and
exit status, including the short-turn failure.

Four barcode test arrays now live with the test that consumes them. Independent
rendering of the supplied PDF reproduced all 116 run widths. The MQTT observer
uses the callback slots populated by the host network fake. Both sides of the
fixed-baseline comparison also reproduce the earlier instrumentation's exact
trace digests. Four affected host binaries passed under ASan and UBSan. The
software evidence and outstanding rules are in the
[comparison receipt](compliance/evidence/phase2-mechanical.json) and
[rule ledger](compliance/BARR_RULE_MATRIX.md).

## Phase 2 types, interfaces and test contracts

The next batches clarified callback signatures, public implementation
parameter names and bounded loops. The barcode decoder now names the number
of wide and narrow elements, characters and inter-character gaps it expects.
Its work budget, arithmetic and both tested direction profiles retain the
same target instructions. The two wire codecs share definitions for their
existing binary32 and binary64 views. The IMU slope fixture now links against
its runtime harness separately.

Strict C99 syntax checks passed for 241 Linux variants across 45 application
sources. Of 61 Arm translation units, 59 passed. The other two produced 21
distinct diagnostics in imported headers. One primitive `float` remains in
the terrain monitor because replacing it with the project alias changed the
Arm instructions. Four affected host programs passed with address and
undefined-behaviour sanitizers at the type-change checkpoint.

The interface checkpoint covers all 61 production objects, 14 conditional
compilations, nine Linux suites and all 17 fixed-baseline comparisons.
The ledger then contained 53 verified, five inapplicable, 11 pending and 98
unreviewed rules. The two original barcode expectations still fail on both
revisions. [Source bindings, compiler checks and traces](compliance/evidence/phase2-mechanical.json)

## Phase 2 fixture and interface types

The next checkpoint documented the remaining host fixtures, named 22
anonymous enums and completed the aggregate typedef review. Across 113
compiler configurations, all 255 distinct aggregate definitions had a
corresponding typedef. Binary packet code now uses the existing uint8_t
alias at 59 sites. The 24 retained unsigned-character uses cover character
handling or access to object representations.

The nine Linux suites passed. The 16 semantic and wire traces and all 1,008
barcode cases matched the fixed baseline. All 61 production Arm object
comparisons and 16 alternate compilations matched after accounting for
private symbol names. The ledger at this checkpoint had 56 verified rules,
five inapplicable rules, 12 pending deviations and 94 unreviewed rules.
[Source bindings and comparisons](compliance/evidence/phase2-mechanical.json)

## Phase 2 callback names, declarations and loop controls

The cleanup gave names to 262 host-test loop counters and clarified 49
function-local Boolean names in the firmware. The MQTT state, IMU bus,
motion callbacks and operator callbacks now identify their roles. Both
revisions ran the shared course test with the default and zero axle offset
after the fixture was adapted to the renamed drive callback.

The final six grouped declarations were in the Buddy 1 and Buddy 4 device
tests. Their test-enabled Arm code, relocations and stack estimates matched,
and all 41 affected check IDs retained their source lines. Across the owned
source, 784 `for` headers and 84 `while` or `do…while` conditions passed
the loop-control review. Compiler checks covered 119 configurations.

All nine Linux suites passed, 80,959 semantic and wire rows matched, and all
1,008 barcode outcomes matched, including the two existing failed expectations.
Three affected binaries passed ASan and UBSan on the latest source. The
ledger reached 60 verified, five inapplicable, 14 pending and 88 unreviewed.
[Compiler and regression evidence](compliance/evidence/phase2-mechanical.json)

The 21 November checkpoint added explicit null initializers to 23 pointer
declarations and used `NULL` for an existing zero pointer literal. The S3
integration module now groups the mailbox and retained state before its
private helpers. The stationary bench host fixture passed ASan and UBSan.
Rule 7.2.d remains open for alternate type spellings and aggregate
initialization. [Pointer and module-order evidence](compliance/evidence/phase2-mechanical.json)

## Phase 2 declarations, expression grouping and startup checks

Shared fixture headers now declare the kernel and radio callbacks that their
tests implement. The compiler found a header for all 802 external function
definitions other than the C runtime entry points across 113 configurations.
The written review also checked two legacy fixture headers. The startup error
reporter now uses a private inline function. All six checked startup Arm
configurations retained their code, relocations and stack estimates.

Compound logical operands and variable-to-constant equality tests now
follow the standard at another 1,152 and 325 locations respectively. The
firmware uses `const` for 109 more scalar locals with read-only uses. The
barcode decoder's proposed const changes and the telemetry snapshot helper
extraction changed Arm output, so those proposals were left out.

All nine Linux suites passed. The 80,959 trace rows and 1,008 barcode
outcomes matched the fixed baseline, including its two failed expectations.
The Arm comparisons matched in 61 production and 16 conditional builds.
Normal and sanitized startup tests passed. The diagnostics-disabled object
was checked without sanitizer instrumentation so the test could verify the
absence of emitted symbols. [Source bindings and results](compliance/evidence/phase2-mechanical.json)

## Phase 2 module layout and private state

Private definitions now follow the public bodies in 22 firmware modules.
The checked builds retained their Arm instructions and stack estimates.
Buddy 1's telemetry queue handle is private. The host fixture records the
handle returned when that queue is created. Five relocations now point
directly to the same four-byte storage through a local section symbol.

The fixture pass gave global prefixes to 321 file-scope variables.
Another 113 pointer checks now compare explicitly with the C null pointer
constant. A second statement scan caught 22 lines with multiple statements
inside decoder test macro arguments. The setup statements now use separate
lines. All nine Linux suites passed, and all 17 baseline comparisons
matched. [Compiler and test results](compliance/evidence/phase2-mechanical.json)

## Phase 2 comparisons, handles and source contracts

The source scan now finds 1,753 documented ordinary function definitions
and nine documented weak MQTT hooks. The host fixtures gained descriptive
variable names and explicit operand grouping. The device assertion locations
were preserved in this comment pass.

The next batch added unsigned suffixes to 49 comparisons, named four private
RTOS handles and separated IR invalid-sample handling. The target size
fixture now declares byte arrays explicitly and all 14 measured symbol sizes
are unchanged. The production and conditional Arm comparisons and all nine
Linux suites passed. The two baseline barcode replay failures persist.
[Definition, comparison and size evidence](compliance/evidence/phase2-mechanical.json)

The capture distance, remote request ID and barcode extrema count now
have descriptive member names. The shared fixtures select the fixed
baseline's spellings while retaining the same trace labels and wire bytes.
The compiler sweep found zero short firmware names in 113 configurations.
The terrain sample and hump paths now have shallower conditionals.
[Field names and control-flow evidence](compliance/evidence/phase2-mechanical.json)
