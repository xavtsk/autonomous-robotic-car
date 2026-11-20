# Repository housekeeping checkpoint

Phase 1 of the controlled-usage cleanup, 20 November 2026. Work remains on
`codex/compliance-preserve-behaviour`; the behaviour baseline remains
`4ad2cc42595722cedb47102e4343298339040d40`.

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
