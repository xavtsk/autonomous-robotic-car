# Specific BARR-C preservation conflicts

Every case below is **pending review/approval**. None is an approved deviation.
The source references identify the `4ad2cc4` baseline; check current definitions
after a rename. Ordinary naming, whitespace or documentation work is not grounds
for an exception. Resolve a case with an equivalent implementation where evidence
supports it; otherwise retain the exact behavior and request a concrete approval.

An eventual approved entry must record the rule, exact files/symbols/profile,
why a compliant alternative cannot meet the preserved contract, supporting tests,
owner, approver name and date, limits and revisit trigger. Place a short reference
near the affected source. Do not approve an entire chapter or imported folder to
hide owned modifications.

| ID | Rules | Exact baseline case | Preservation issue and required resolution | Status |
| --- | --- | --- | --- | --- |
| BARR-P01 | 1.1.a | `build_make/pico_rp2040.mk:306`; `app_program/controller/integration_codec.c:3`; other owned `_Static_assert` uses | The shared build selects GNU11 and the owned code uses C11 assertions. Replace owned assertions with C99 compile-time checks without weakening conditions; validate an owned C99 subset without changing imported build semantics. This is a remediation candidate, not an approved language deviation. | pending review |
| BARR-P02 | 1.1.c, 5.5.a-b | `app_program/controller/bc_capture.h:36` (`bcap_sample_t`), `app_program/subsystem3/bc_decoder.h:20,38`, `ir_acquisition.h:35`, `ir_calibration.h:5`, `ir_reflectance.h:26` | Packed objects retain RAM/layout contracts; removing attributes can widen the 4,000-entry capture pool. Evaluate explicit bytes/accessors with size, exact-value and timing evidence. Merely hiding attributes behind a macro does not satisfy the hardware-driver localization rule. | pending review |
| BARR-P03 | 5.4.b.iv | `app_program/subsystem2/speed_pid.c:52` (`speed_pid_update`) | Exact positive/negative zero triggers PID reset. Epsilon tests would change behavior. Prove an exact classification replacement across zero, subnormal, finite, infinity and NaN cases, including relevant exception/timing semantics. | pending review |
| BARR-P04 | 5.4.b.iv-v | `app_program/controller/s3_drive.c:45`, `app_program/subsystem3/line_controller.c:5`, `app_program/subsystem2/motion.c:136`, `motion_pack.c:74,117` | Self-equality and subtraction implement finite/NaN checks. Use equivalent classification only with differential evidence. Do not add new clipping, invalid-input fallbacks or arithmetic regrouping. | pending review |
| BARR-P05 | 1.8.c | `app_program/subsystem4/buddy4.c:32-45` (`g_queue`), `app_program/controller/s3_integration.c:29-37,52-55` | Existing mutex-owned packed objects and a separately volatile hint have distinct access contracts. Record readers/writers, locks/barriers and access width. Adding volatile changes generated accesses and does not replace synchronization; preserve the ownership contract. | pending review |
| BARR-P06 | 5.6.a, 5.5.b | `app_program/controller/s3_integration.c:54`; `app_program/subsystem5/avoid_obstacle.h:75-82` | Unsigned one-bit Boolean members share allocation units with other fields. Changing them to `bool : 1` requires target layout and access checks; do not alter atomicity, read-modify-write ownership or retained values. | pending review |
| BARR-P07 | 2.1.c.ii | `app_program/controller/s3_integration.c` (`APP_IR_DEBUG`, barcode capture); `app_program/subsystem4/buddy4.c:54` (`APP_IMU_DEBUG`); `build_make/pico_rp2040.mk:512` | Runtime diagnostic publications are selected by application profiles; a network build uses NDEBUG. Blind NDEBUG wrapping would remove established diagnostic behavior. Classify operational telemetry versus optional increased debug output, then identify any exact unavoidable exception. | pending review |
| BARR-P08 | 6.1.i, 5.1.a-c, 7.1 | `app_program/subsystem1/mqtt_bridge.c:73,152,164,173,184`: `mqtt_utk_app_take`, `mqtt_utk_app_status`, `mqtt_utk_app_link`, `mqtt_utk_app_connection`, `mqtt_utk_app_receive` | External ABI callback names/types cannot be renamed unilaterally. Keep exact boundary declarations with a symbol-level inventory and internal compliant wrappers where useful; this entry does not waive unrelated owned adapter code. | pending review |
| BARR-P09 | 6.3 | `app_program/controller/control_debug.h:9,61,70-71` (`CONTROL_DEBUG_FIELDS`, `CD_*`) | X-macros generate declarations and constant expressions that an ordinary function cannot produce. Document expansion semantics and evaluation counts; review each macro safeguard instead of suppressing all macro checks. | pending review |
| BARR-P10 | 4.3.f, 4.2.a, 6.1.i | `testing/subsystem3/test_barcode_incremental.c:6`, `test_barcode_fit_error.c:6`; `testing/subsystem5/runtime_test.c:43`; `testing/subsystem4/transport_test.c:85`; `testing/diagnostics/firmware_test.c:82` (`main`) | The named harnesses include production implementation for private access; evaluate normal compilation without exposing production state solely for tests. `main` cannot acquire a module prefix while remaining the C entry point. Every additional affected harness must be individually inventoried; this is not a testing-folder exemption. | pending review |
| BARR-P11 | 6.2.a, 8.2.b | `app_program/subsystem3/bc_decoder.c:1510-1949` (`work_advance`) | Decomposition must preserve per-call work units, fit-tick limits, phase transitions, evaluation order and replay output. Function length is a reasonable-effort requirement, not permission to reset the budget in extracted helpers. | pending review |
| BARR-P12 | 5.4.b.i | `app_program/subsystem4/terr_data.c`, `terrain_monitor_process`, local `value_n` | Arm GNU 14.3.1 emits different terrain instructions when this one local uses the exact `float32_t` alias. Tested alias declaration variants do not restore the original bytes. Retaining primitive `float` here restores every allocated object section and relocation, preserving the original 200-byte compiler stack estimate. This is a narrowly documented preservation choice, not an approved deviation; see [target evidence](evidence/ALIAS_TARGET_INVESTIGATION.md). | pending approval; no approver assigned |

The standard's preferences for a single exit, avoiding continue, local declaration
placement and shortest branch first do not authorize changing evaluation order,
resource cleanup or state transitions. Record reviewed retention under the
specific rule; do not label it a violation of an unconditional requirement.

Precision-sensitive fields are also tracked in the
[Change Requirement 1 inventory](CR1_FIELD_INVENTORY.md). Their widths and
quantization must not be inferred from displayed decimal precision.
