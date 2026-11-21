# Specific BARR-C preservation conflicts

Entries marked pending require review or approval. Resolved entries record a verified source change.
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
| BARR-P01 | 1.1.a | Shared GNU11 target recipe in `build_make/pico_rp2040.mk` and two target units that include headers with dialect dependencies | The owned C11 assertions have been replaced with C99 checks. All 241 inspected host variants and 59 of 61 target units passed the strict C99 syntax trials. The remaining compiler diagnostics come from imported headers. The shared dialect still applies to imported code in the target build. See the [C99 and assertion review](evidence/phase2-mechanical.json#phase2_definition_contracts_and_binding_review). | pending review |
| BARR-P02 | 1.1.c, 5.5.a-b | `app_program/controller/bc_capture.h:36` (`bc_capture_sample_t`), `app_program/subsystem3/bc_decoder.h:20,38`, `ir_acquisition.h:35`, `ir_calibration.h:5`, `ir_reflectance.h:26` | Packed objects retain RAM/layout contracts; removing attributes can widen the 4,000-entry capture pool. Evaluate explicit bytes/accessors with size, exact-value and timing evidence. Merely hiding attributes behind a macro does not satisfy the hardware-driver localization rule. | pending review |
| BARR-P03 | 5.4.b.iv | `app_program/subsystem2/speed_pid.c:52` (`speed_pid_update`) | Exact positive/negative zero triggers PID reset. Epsilon tests would change behavior. Prove an exact classification replacement across zero, subnormal, finite, infinity and NaN cases, including relevant exception/timing semantics. | pending review |
| BARR-P04 | 5.4.b.iv-v | `app_program/controller/s3_drive.c:45`, `app_program/subsystem3/line_controller.c:5`, `app_program/subsystem2/motion.c:136`, `motion_pack.c:74,117` | Self-equality and subtraction implement finite/NaN checks. Use equivalent classification only with differential evidence. Do not add new clipping, invalid-input fallbacks or arithmetic regrouping. | pending review |
| BARR-P05 | 1.8.c | `app_program/subsystem4/buddy4.c:32-45` (`g_queue`), `app_program/controller/s3_integration.c:29-37,52-55` | Existing mutex-owned packed objects and a separately volatile hint have distinct access contracts. Record readers/writers, locks/barriers and access width. Adding volatile changes generated accesses and does not replace synchronization; preserve the ownership contract. | pending review |
| BARR-P06 | 5.6.a, 5.5.b | `app_program/controller/s3_integration.c:54`; `app_program/subsystem5/avoid_obstacle.h:75-82` | Unsigned one-bit Boolean members share allocation units with other fields. Changing them to `bool : 1` requires target layout and access checks; do not alter atomicity, read-modify-write ownership or retained values. | pending review |
| BARR-P07 | 2.1.c.ii | `app_program/controller/s3_integration.c` (`APP_IR_DEBUG`, barcode capture); `app_program/subsystem4/buddy4.c:54` (`APP_IMU_DEBUG`); `build_make/pico_rp2040.mk:512` | Runtime diagnostic publications are selected by application profiles; a network build uses NDEBUG. Blind NDEBUG wrapping would remove established diagnostic behavior. Classify operational telemetry versus optional increased debug output, then identify any exact unavoidable exception. | pending review |
| BARR-P08 | 6.1.i, 5.1.a-c, 7.1 | `app_program/subsystem1/mqtt_bridge.c:73,152,164,173,184`: `mqtt_utk_app_take`, `mqtt_utk_app_status`, `mqtt_utk_app_link`, `mqtt_utk_app_connection`, `mqtt_utk_app_receive` | External ABI callback names/types cannot be renamed unilaterally. Keep exact boundary declarations with a symbol-level inventory and internal compliant wrappers where useful; this entry does not waive unrelated owned adapter code. | pending review |
| BARR-P09 | 6.3 | `app_program/controller/control_debug.h:9,61,70-71` (`CONTROL_DEBUG_FIELDS`, `CD_*`) | X-macros generate declarations and constant expressions that an ordinary function cannot produce. Document expansion semantics and evaluation counts; review each macro safeguard instead of suppressing all macro checks. | pending review |
| BARR-P10 | 4.3.f, 6.1.i | `testing/subsystem3/test_barcode_incremental.c:6`, `test_barcode_fit_error.c:6`; `testing/subsystem5/runtime_test.c:43`; `testing/subsystem4/transport_test.c:85`; `testing/diagnostics/firmware_test.c:82` (`main`) | The named harnesses include production implementation for private access; evaluate normal compilation without exposing production state solely for tests. `main` cannot acquire a module prefix while remaining the C entry point. Every additional affected harness must be individually inventoried; this is not a testing-folder exemption. | pending review |
| BARR-P11 | 6.2.a, 8.2.b | `app_program/subsystem3/bc_decoder.c:1510-1949` (`work_advance`) | Decomposition must preserve per-call work units, fit-tick limits, phase transitions, evaluation order and replay output. Function length is a reasonable-effort requirement, not permission to reset the budget in extracted helpers. | pending review |
| BARR-P12 | 5.4.b.i | `app_program/subsystem4/terr_data.c`, `terr_data_process`, local `value_n` | Arm GNU 14.3.1 emits different terrain instructions when this one local uses the exact `float32_t` alias. Tested alias declaration variants do not restore the original bytes. Retaining primitive `float` here restores every allocated object section and relocation, preserving the original 200-byte compiler stack estimate. This is a narrowly documented preservation choice, not an approved deviation; see [target evidence](evidence/ALIAS_TARGET_INVESTIGATION.md). | pending approval; no approver assigned |
| BARR-P13 | Preservation constraint alongside 1.2.a, 2.2.h, 3.3.a-c | `app_program/subsystem1/buddy1_test.c`, `subsystem2/buddy2_test.c`, `subsystem4/buddy4_test.c`; imported `app_program/test_report.h:39` | Formatting and added documentation move the source line numbers printed by an intentionally failed `TEST_CHECK` under `APP_TEST`. The imported macro still reports the actual current source location. Freezing old locations would make failure diagnostics misleading. [Source-bound evidence](SOURCE_LOCATION_DIAGNOSTICS.md) identifies all 63 check IDs and the USB/UART route; these messages are not MQTT packet fields. This is a pending disposition of failure-console metadata, not permission to change assertions, test results, normal telemetry or car behavior. | pending preservation disposition; no approval granted |
| BARR-P14 | 6.1.g | `app_program/subsystem2/motion.h` and `motion.c` (`moveForward`, `moveBackward`, `turnLeft`, `turnRight`). `testing/subsystem2/wheel_kernel.h` and `host_regression_main.c` (`EnableInt`). | Keep the course brief's Buddy 2 spellings and the simulator's exact stand-in for the kernel symbol. Renaming either boundary would break existing callers. The written definition scan identifies these five mixed-case names. | pending approval |
| BARR-P15 | 7.1.n | `testing/subsystem1/host_kernel.h` (`T_RMBF.wtsk`, `T_RMBF.stsk`) and `include/tk/syscall.h` at lines 341 and 342 | The host shim mirrors the source-level T-Kernel interface for the receive and send wait task IDs. The compiler survey found these two handle names after converting the ordinary application and fixture handles. A divergent shim name would require a compatibility decision. | pending approval |
| BARR-P16 | 5.6.b | `motion_calibration_expand`, `motion_init`, `motion_process`, `motion_get_feedback`, `run_calibration`, `sensor_sample_task`, `bc_decoder_reset`, `bc_decoder_update`, `rec_finish`, `bc_decoder_record_update`, `line_detection_reset`, `line_navigation_process`, `obstacle_task`, `line_recovery_observe` | Thirty direct non-Boolean inputs retain C's implicit conversion to `bool`. Explicit nonzero comparisons changed the generated instructions in every affected production object trial. Review the exact conversions and target cost before accepting a timing-sensitive change. The remaining conditional expressions and fixture conversions have their own review under 5.6.b. | pending review |
| BARR-P17 | 1.8.a, 4.2.c, 6.2.e | `app_program/subsystem1/command.c` and `buddy1.h` (`buddy1_command_kind`). `app_program/controller/op_logic.c` and `op_logic.h` (`op_logic_drive_state_name`, `op_logic_drive_reason_name`). | Repository callers of these three helpers are in their respective implementation files. Making them static changed the generated instructions and stack estimates for command enqueue and the operator step. Keep the existing linkage until the exact call and timing differences can be assessed. | pending review |
| BARR-P18 | 6.3.a | `app_program/subsystem3/bc_decoder.c` (`REC_DARK`, `REC_STEP_US`, used in `rec_extract_runs`) | The inline substitutions changed the generated barcode extraction instructions. Retain the parenthesized accessors and original sample-read order until timing and recorded replay can support an equivalent replacement. | pending review |
| BARR-P19 | 4.3.b | `app_program/subsystem2/motion_pack.c`, private packing and command validation helpers | Placing the private bodies after the public APIs compiled in both production and diagnostics. The Arm allocated-section and relocation comparison differed in both profiles. Stack estimates matched. The definitions retain their previous order while the object-level differences are assessed. | pending review |
| BARR-P20 | 7.1.k | Host fakes in `testing/diagnostics/startup_main.h`, `testing/integration/runtime_main.h`, `testing/subsystem4/transport_main.h`, `testing/subsystem1/host_kernel.h`, `testing/subsystem1/fakes/lwip/apps/mqtt.h`, and the Wi-Fi fixture's `cyw43_poll` declaration and definition | The kernel stand-ins expose `task`, `cychdr` and `exinf` to the real application sources. The lwIP and CYW43 fakes expose `client_id` and `cyw43_poll` to the real network integration. These eight sites retain the field or symbol names used by their platform interfaces. | pending approval |
| BARR-P21 | 7.1.j | `tm_usb_link_state` in `app_program/controller/s3_integration.c`; `cyw43_utk_poll_requested` in `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`; `cyw43_state`, `cyw43_poll` and `cyw43_utk_poll_requested` in the Wi-Fi host fixture and its fake header | The production references use platform-owned USB and radio symbols. The host fixture mirrors the radio names so it can compile the real integration. These seven declaration sites retain their platform spelling. | pending approval |
| BARR-P22 | 5.3.b | `app_program/subsystem4/buddy4.c`, initial IMU debug row flags | Making all four initial conditional masks unsigned changed the Arm executable sections for the preview and preview-bench builds. Stack estimates matched. The three later flag merges passed exact comparison with unsigned operands and were applied. The original initializer remains pending timing and target-output review. | pending review |
| BARR-P23 | 6.2.c, preferred | `imu_sensor_init` in `app_program/subsystem4/imu_sensor.c` and `scan_servo_init` in `app_program/subsystem5/scan_servo.c` | Combining the final failure return with the successful readback or servo-enable result changed the enabled Arm executable sections. Stack estimates matched. Keep the ordered guards while the startup instruction differences are unresolved. | pending review |
| BARR-P24 | 6.1.i | `motion_cmd_cancel` in `app_program/subsystem2/motion_pack.c` and `motion_pack.h`, used by the Buddy 2 device self-test | The prefixed spelling shifts five enabled self-test source locations under the checked formatter, even after a trial consolidated the final status comparisons. A failed device assertion prints its source location. The other thirteen names preserve those locations. Arm sections, relocations and stack estimates match after normalizing the renamed symbols. | pending review |
| BARR-P25 | 5.1.c | `T_MQTT_UTK_APP_MSG` in `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h` | The MQTT integration and its adapters share this source-level interface type. Retaining its spelling preserves the boundary for existing consumers. The two other unmatched names, `float32_t` and `float64_t`, are prescribed by 5.4.b.i. | pending approval |
| BARR-P26 | 6.1.i | `buddy1_decode`, `buddy1_encode`, `buddy1_map_pack`, `enc_l_count`, `enc_r_count`, `robot_distance_m`, `robot_heading_deg` | Renaming these APIs and formatting their calls moved printed failure locations in the Buddy 1 and Buddy 2 device self-tests. The [module-prefix receipt](evidence/phase2-mechanical.json#phase2_public_function_module_prefixes) binds both location lists. | pending review |
| BARR-P27 | 6.1.i | `usermain`, `subsystem1_init` through `subsystem5_init`, the corresponding five `subsystemN_start` functions, the nine `mqtt_utk_*` callbacks listed in the module-prefix receipt, and Buddy 2's `moveForward`, `moveBackward`, `turnLeft`, `turnRight` and `stop` | Keep the RTOS and network entry points, lifecycle names declared by the original course template, and the existing motion interface. Renaming externally referenced symbols requires agreement on caller compatibility. | pending approval |
| BARR-P28 | 1.8.b, preferred | `bias` in `rec_correct_shared_runs`, `app_program/subsystem3/bc_decoder.c` | Adding `const` to this one immutable local changes Arm instructions and raises the compiler stack estimate for `bc_decoder_record_correct_runs` from 248 to 264 bytes. The [barcode const receipt](evidence/phase2-mechanical.json#phase2_fixture_loop_bounds_and_barcode_const) records 29 other qualifiers that passed both paired comparisons. | pending review |
| BARR-P29 | 5.3.b | `control_debug.c` (`read_bits`), `ir_debug.c` (`ir_debug_pack`, `get`), `ctrl_motion.c` (`mv_sign`), `terr_data.c` (`get_bits`), and `s5_codec.c` (`get`) | The source values are unsigned bytes or bit-fields that C promotes before shifting. Widening them explicitly changes allocated Arm sections in the tested profiles. The original operations are retained pending review of the code generation difference. See the [promotion receipt](evidence/phase2-mechanical.json#phase2_small_bitwise_promotions_and_diagrams). | pending review |
| BARR-P30 | 6.2.a, 8.2.b | `bc_capture_binary_next` in `app_program/controller/bc_capture.c` and `vehicle_controller_observe` in `app_program/controller/vehicle_controller.c` | Extracting the sample preflight or obstacle publication changes generated Arm output. Moving the existing end/availability guards also changes the enabled objects. Only the local capture count clamp matched and was applied. See the [comparison receipt](evidence/phase2-mechanical.json#phase2_capture_clamp_and_include_review). | pending review |
| BARR-P31 | 7.1.e | `SYSTIM.hi` and `SYSTIM.lo` in `testing/diagnostics/wheel_smoke_main.h` | The host shim mirrors the RTOS time record in `include/tk/typedef.h`. The real smoke diagnostic reads `time.lo` directly. The [expanded compiler review](evidence/phase2-mechanical.json#phase2_short_names_and_enabled_profiles) found these two remaining short fields after enabling the device self-test, motion-test, IR bench, IMU preview and barcode reference options. | pending approval |
| BARR-P32 | 7.1.m | Former `U(collecting, 1)` in `app_program/controller/control_debug.h` | The earlier direct rename changed the trace key. The C member is now `b_is_collecting`, and the host fixture binds its original `collecting` key to the member offset. All 487 controller-debug trace rows match the fixed baseline, including the packet bytes. See the [debug trace comparison](evidence/phase2-mechanical.json#phase2_debug_predicates_stable_wire_labels). | resolved |
| BARR-P33 | 7.2.b | Local declarations in `bc_capture.c`, `s3_integration.c`, `terr_link.c`, `vehicle_controller.c`, `buddy1.c`, `telemetry_codec.c`, `bc_decoder.c`, and `buddy4.c` | Grouped declaration moves changed Arm output, stack estimates or formatted line widths. The declaration sites, original and trial hashes, and accepted moves are in the [first-use comparison](evidence/phase2-mechanical.json#phase2_locals_near_first_use). The preferred placement remains open for these groups. | pending review |
| BARR-P34 | 8.2.a | `peek_result` in `app_program/subsystem5/avoid_obstacle.c` and `obstacle_task` in `app_program/subsystem5/buddy5.c` | Reversing the two branch pairs put shorter branches first but changed the allocated Arm output in active profiles. The original branches and their proposal ordering are retained. See the [branch comparisons](evidence/phase2-mechanical.json#phase2_short_first_branch). | pending review |
| BARR-P35 | 6.2.a | `op_logic_step` in `app_program/controller/op_logic.c` | Extracting GO inhibition changed allocated Arm output in both tested profiles. A separate trial extracting the first-status and active-run switch branches also changed the output. All three blocks retain their existing location. See the [GO comparison](evidence/phase2-mechanical.json#phase2_line_follow_dispatch_helper) and [status comparison](evidence/phase2-mechanical.json#phase2_operator_status_helper_preservation). | pending review |
| BARR-P36 | 6.2.a | `profile_obstacle_build` in `app_program/subsystem5/profile_obstacle.c` | Moving the accumulation of a validated echo's geometry into an inline helper changed allocated Arm output. The geometry calculations remain in the existing scan loop. See the [geometry extraction comparison](evidence/phase2-mechanical.json#phase2_codec_fixture_and_function_length_review). | pending review |
| BARR-P37 | 6.2.a | `bc_capture_record` in `app_program/controller/bc_capture.c` | Extracting episode creation and the buffered sample copy changed allocated Arm output in production and offline profiles. The original code remains in the capture sampling path. See the [capture extraction comparison](evidence/phase2-mechanical.json#phase2_capture_episode_extraction_preservation). | pending review |
| BARR-P38 | 4.3.b | `debug_metadata_count_t` in `app_program/controller/control_debug.c` | The assertion counts the actual initializers using the inferred extent of `g_debug_fields`. The array declaration must precede the assertion. Declaring a fixed array extent first would make the check accept missing initializers. See the [section review](evidence/phase2-mechanical.json#phase2_mandatory_source_sections_1). | pending review |
| BARR-P39 | 4.3.b | Static reporting and conversion helpers in `app_program/controller/vehicle_controller.c` | Placing private bodies after the public definitions changed allocated Arm output with avoidance enabled. The disabled profile matched. See the [paired comparisons](evidence/phase2-mechanical.json#phase2_mandatory_source_sections_1). | pending review |
| BARR-P40 | 5.3.b | The validity-mask test in `ir_debug_pack` and the final-byte padding test in `buddy1_decode`, in `app_program/controller/ir_debug.c` and `app_program/subsystem1/telemetry_codec.c` | Explicitly widening the unsigned input bytes before inversion or shifting changed allocated Arm output in every tested profile. The stack estimates matched. The original tests still reject the same inputs, and the write-side masks passed paired Arm comparison independently. Review the generated read paths before approving the source deviation. See the [mask comparisons](evidence/phase2-mechanical.json#phase2_unsigned_packed_masks). | pending review |
| BARR-P41 | 8.2.b | `work_primary` in `app_program/subsystem3/bc_decoder.c`; snapshot and pose branches of `buddy1_encode` in `app_program/subsystem1/telemetry_codec.c` | Removing the barcode branch after its terminating return changed allocated Arm output in three profiles. Combining the telemetry flag and write checks into an ordered conditional also changed allocated output. The originals retain the reviewed decoder work and packet paths pending code-generation review. See the [branch trials](evidence/phase2-mechanical.json#phase2_mqtt_branch_depth). | pending review |
| BARR-P42 | 7.1.i, 7.1.m | Former `b_ok` in `run_unit_tests`, `app_program/subsystem2/buddy2_test.c` | The earlier `b_passed` trial moved 14 printed locations. The line-stable `b_met` spelling preserves all 22 printed locations and matches allocated instructions, data and stack estimates in three Arm profiles. See the [device comparison](evidence/phase2-mechanical.json#phase2_device_outcome_name_preservation). | resolved |
| BARR-P43 | 4.3.b | `sampling_storage_byte_t` and `shared_storage_byte_t` in `app_program/controller/s3_integration.c` | Each assertion checks the actual static object. Moving the assertion ahead of the declaration would require substituting a different subject. The object checks stay after their declarations. The independent range assertions moved earlier and matched the Arm profiles. See the [section checks](evidence/phase2-mechanical.json#phase2_types_before_static_storage). | pending review |
| BARR-P44 | 5.3.b | `hump_detection_update_right` in `app_program/subsystem4/hump_detection.c` | Using explicitly unsigned operands for the seven debug flags changed allocated Arm output and the stack estimate in the IMU preview profile. The disabled profile matched. The original flag expression stays in place pending review of the enabled code generation. See the [preview trial](evidence/phase2-mechanical.json#phase2_imu_preview_integer_domains). | pending review |
| BARR-P45 | 6.2.a, 8.2.b | The scan state in `avoid_obstacle_step` and the snapshot field and padding checks in `buddy1_decode` | Extracting the scan action changed allocated Arm output in both avoidance profiles. Extracting the decoder section changed allocated output and the reported stack estimate. The original branches stay in their owning functions pending review of those differences. See the [branch comparisons](evidence/phase2-mechanical.json#phase2_stationary_bench_publication_helper). | pending review |
| BARR-P46 | 8.2.b | Terrain reset in `terrain_task`, capture end pose in `process_barcode`, heading rate in `terr_data_process`, and calibration reply in `cal_collect` | The private-helper trials changed allocated Arm output in all seven comparisons. The heading-rate extraction also changed the production stack estimate from 200 to 192 bytes. Retain the original branches pending review of the instruction and timing differences. See the [helper comparisons](evidence/phase2-mechanical.json#phase2_owner_helper_preservation) and [calibration guard trial](evidence/phase2-mechanical.json#phase2_operator_calibration_ack_guard_trial). | pending review |
| BARR-P47 | 4.3.b | `subsystem2_test` in `app_program/subsystem2/buddy2_test.c` | Placing the public device-test entry before the private bodies changed the source positions of all 22 `TEST_CHECK` calls. A failed check prints that location in the device console. Keep the current order pending a line-stable layout or approval of changes to device diagnostics. See the [location comparison](evidence/phase2-mechanical.json#phase2_device_selftest_source_order). | pending review |
| BARR-P48 | 4.3.b | `poll_driver` and `cyw43_poll` in `testing/network_wifi/wifi_service_main.c` | The fixture initializes the platform callback pointer with the address of the private function. Moving the sole declaration below the initializer leaves the name undeclared. The compatible pointer is ready when the fixture starts. See the [compiler diagnostic](evidence/phase2-mechanical.json#phase2_fixture_private_declaration_sections). | pending review |
| BARR-P49 | 8.4.a | Fourteen loop sites in `testing/subsystem2/host_regression_main.c` | The old IRQ harness is retained as historical evidence. Its optional `legacy-test` target already fails against the current motion API and hardware fakes. The supported pack, feedback and drive tests cover the current implementation. This checkpoint preserves the archived fixture and records the [existing compile diagnostics](evidence/phase2-mechanical.json#phase2_supported_loop_bound_disposition). See the [historical scope](../../testing/subsystem2/EVIDENCE.md). | pending review |
| BARR-P50 | 8.2.b | Remote reset reply in `sensor_sample_task` and junction advance completion in `s3_drive_process` | Flattening the reset reply changed allocated Arm output in three integrated sampling profiles. Extracting the turn request changed output and the stack estimate in both tested drive profiles. Keep the existing paths pending instruction and timing review. See the [paired comparisons](evidence/phase2-mechanical.json#phase2_barcode_start_guard_and_motion_trials). | pending review |
| BARR-P51 | 8.2.b | Paper noise learning in `bc_decoder_record_update`, IR preview channel filtering in `sensor_sample_task`, and scan aiming in `avoid_obstacle_step` | Each ordered guard trial changed allocated Arm output in both tested profiles. Stack estimates matched. The existing branches retain their original evaluation and timing pending instruction review. See the [paired comparisons](evidence/phase2-mechanical.json#phase2_validation_guard_preservation). | pending review |
| BARR-P52 | 1.6.a, 2.2.g | `search_leg_us` in `app_program/controller/s3_drive.c` and the completed-leg lateral distance in `app_program/subsystem5/avoid_obstacle.c` | The option setter accepts any positive finite search speed through 0.30 m/s. A sufficiently small positive speed can put the computed duration outside the integer range. The lateral distance branch checks the lower bound before converting and caps the stored value afterwards. Its conversion relies on the caller supplying a bounded completed-leg distance. Validation or saturation would change the affected input behavior. The current profile uses 0.05 m/s for search. See the [cast review](evidence/phase2-mechanical.json#phase2_floating_conversion_ranges). | pending review |
| BARR-P53 | 6.2.a, 8.2.b | Issued-motion completion and the finish state in `avoid_obstacle_step`, `app_program/subsystem5/avoid_obstacle.c` | The ordered moving-completion guard, ordered finish-release checks and a private finish-state helper each changed allocated Arm output in both tested profiles. The function stack estimate remained 80 bytes. The original motion proposal and tape-recovery paths remain pending instruction and timing review. See the [transition trials](evidence/phase2-mechanical.json#phase2_avoidance_issued_transition_preservation). | pending review |
| BARR-P54 | 6.2.a, 8.2.b | Sensor/servo initialization in `obstacle_task` and signed bias-pair validation in `work_advance` | Moving the initialization guard changed the obstacle task instructions in both tested profiles, including hardware-confirmed avoidance. Extracting the fit-pair check changed recorder code in both tested decoder profiles. Stack estimates matched. The startup and fit-budget paths remain in place pending instruction and timing review. See the [paired trials](evidence/phase2-mechanical.json#phase2_capture_cursor_and_echo_count). | pending review |
| BARR-P55 | 6.2.a, 8.2.b | The held STOP request branch in `op_logic_step` and the odometry travel/speed calculation in `terr_data_process` | An ordered STOP guard changed allocated Arm code in both tested profiles while the 88-byte stack estimate matched. Extracting the odometry calculation changed allocated output while retaining the 200-byte production and 192-byte preview estimates. The original STOP retries, locking and terrain timing remain in place pending instruction and timing review. See the [paired trials](evidence/phase2-mechanical.json#phase2_obstacle_telemetry_ordered_guards). | pending review |
| BARR-P56 | 8.2.b | State dispatch in `avoid_obstacle_step` and sample validation in `bc_capture_binary_next` | Converting the four-state avoidance cascade to a switch changed allocated Arm output in both evaluated profiles. A separate attempt to carry capture validation failure out of the selection block also changed allocated output in both evaluated profiles. Keep the existing state and capture work paths pending timing and instruction review. See the [paired trials](evidence/phase2-mechanical.json#phase2_confirmed_braking_request). | pending review |
| BARR-P57 | 6.2.a, 8.2.b | Completed-run collection and validation in `bc_decoder_update` | Moving the completed-run branch into a private inline helper changed allocated Arm output in the integrated and synchronous-reference profiles. Keep the original run-update and overflow/rejection paths pending work and timing review. See the [streaming run trial](evidence/phase2-mechanical.json#phase2_held_barcode_and_route_publication). | pending review |
| BARR-P58 | 8.2.b | Emergency distance check during an active manoeuvre in `obstacle_control`, `app_program/controller/s3_drive.c` | The ordered-guard trial changed allocated Arm output with avoidance enabled. The disabled profile and both stack estimates matched. Keep the existing emergency halt path pending instruction and timing review. See the [paired trials](evidence/phase2-mechanical.json#phase2_capture_lifecycle_and_obstacle_latch). | pending review |
| BARR-P59 | 8.2.b | Confirmed button edge in `op_logic_button_update` and timer/task startup in `subsystem4_start` | The private-helper trials changed allocated Arm output in both compared profiles for each function. Stack estimates matched. Keep the current press classification and kernel cleanup order pending review of the instruction changes. See the [paired trials](evidence/phase2-mechanical.json#phase2_endpoint_branch_preservation). | pending review |
| BARR-P60 | 8.2.b | Forward echo processing in `obstacle_task`, resumed ink in `bc_decoder_record_update`, and local/remote replies in `sensor_sample_task` | The forward-echo helper, ordered decoder guard and reply helper changed allocated Arm output in seven comparisons. Stack estimates matched. Keep the original paths pending review of observation freshness, decoder work and sampling timing. See the [paired trials](evidence/phase2-mechanical.json#phase2_endpoint_branch_preservation). | pending review |
| BARR-P61 | 8.2.b | Reverse-distance guard in `avoid_obstacle_step` and scan/peek completion in `obstacle_task` | Moving the distance guard and extracting the profile-completion helper changed allocated Arm output in both tested profiles for each trial. A second scan helper taking the peek ID by value also changed both Arm outputs. The motion proposal and scan publication paths remain pending timing review. See the [follow-up](evidence/phase2-mechanical.json#phase2_post_turn_recovery_helper). See the [paired trials](evidence/phase2-mechanical.json#phase2_drive_search_obstacle_votes_and_reference_capacity). | pending review |
| BARR-P62 | 8.2.b | First-edge/run output selection in `rec_extract_runs` | Selecting the output pointer from the existing first-edge condition changed allocated Arm code in the production and synchronous-reference profile comparisons. Preserve the original output path pending review of the reference decoder timing and scratch writes. See the [paired trials](evidence/phase2-mechanical.json#phase2_barcode_run_selection_and_diagonal_clearance). | pending review |
| BARR-P63 | 8.2.b | Tape-triggered axle placement in `avoid_obstacle_step` | Unconditionally storing the shared leg value changed allocated Arm output in both profiles. The original branch makes the store only when changing from the passing leg. Leave the packed write and STOP proposal on their existing paths pending review. See the [paired trials](evidence/phase2-mechanical.json#phase2_obstacle_diagonal_return_and_barcode_retry). | pending review |
| BARR-P64 | 8.2.b | Pending remote RESET staging in `sensor_sample_task` and interpolation step guard in `rec_extract_runs` | The helper around the lock-owned request record changed allocated Arm output in three profiles. The continue guard for the reference interpolation changed allocated output in both compared decoder profiles. Retain the original mutex and edge-accumulation paths pending timing review. See the [paired trials](evidence/phase2-mechanical.json#phase2_unsigned_clock_order). | pending review |
| BARR-P65 | 8.2.b | First-scan aiming and completed passing-leg lateral correction in `avoid_obstacle_step` | The aim classification and lateral correction helper trials each changed allocated Arm output in both compared avoidance profiles. Keep the original action issue sequence and lateral calculation pending timing review. See the [paired trials](evidence/phase2-mechanical.json#phase2_confirmed_alternate_diagonal_guard). | pending review |
| BARR-P66 | 8.2.b | Forward-echo publication in `obstacle_task` and pre-trigger noise learning in `bc_decoder_record_update` | The forward-echo else-if and the independent noise guard each changed allocated output in two Arm profiles, with identical compiler stack estimates. Retain the original measurement and pre-trigger timing paths pending instruction review. See the [paired trials](evidence/phase2-mechanical.json#phase2_forward_echo_and_recorder_noise_guard_trials). | pending review |
| BARR-P67 | 8.2.b | Tape STOP at avoidance handoff in `avoid_obstacle_step` and quiet-break handling in `bc_decoder_record_update` | Moving the STOP guard and splitting the recorder processing guard changed allocated Arm output in both profiles for each site. Retain the current handoff and restart order pending instruction and timing review. See the [paired trials](evidence/phase2-mechanical.json#phase2_finish_tape_and_ink_restart_trials). | pending review |

The [header review](HEADER_CLOSURE_PLAN.md) resolved P10's original pairing
question under 4.2.a. Its implementation includes and entry-point naming
remain pending.

### Disposition of the source review on 20 November

- **P01.** The last eleven C11 assertions in owned test code now use
  `BUILD_ASSERT`. The predicates and eight matched linked images were checked
  at the conversion. The GNU11 target recipe and whole-program C99 review
  remain open. Nine selected production/runtime units subsequently passed
  strict C99 on Arm. The Wi-Fi owner hit 21 diagnostics, all in the pinned
  SDK headers, including statement expressions, layout assertions and
  anonymous aggregates. The existing GNU11 target recipe stays in use.
  [Strict C99 slice](evidence/phase2-mechanical.json#phase2_c99_owned_runtime_slice)
  Expanding the same check to 61 recorded production commands yielded 59
  passes. The additional blocker was a repeated TCB typedef in the
  imported kernel header `kernel/knlinc/smp.h`. All 22 diagnostics in the
  two failed commands point into the imported kernel or SDK.
  [Production C99 results](evidence/phase2-mechanical.json#phase2_c99_recorded_production_profiles)
- **P10.** The current paths and lines for every direct implementation
  include appear under `remaining_findings` in the grouped receipt. They
  include private decoder instrumentation, course simulators and hardware
  fakes. Each needs an individual separate-compilation decision.
- **P11.** Extracting the decoder validation phase preserved Arm instructions,
  relocations and compiler stack estimates in both exercised decoder profiles.
  Extracting the fit-begin, fit-tick or fit-end phases changed the generated
  update code. Extracting either the scanning or finishing arm of
  `avoid_obstacle_step` also changed its generated code. The candidate keeps
  those phases in place. Function length and nesting remain under review.
- **P13.** The team authorized current filenames, line numbers and expression
  spelling in deliberately failed **host** assertions. Preserve the condition,
  result and successful output. The device-side `APP_TEST` USB/UART messages
  in the original entry still await their own disposition.

The compiler comparisons, exact assertion conditions and remaining locations
are in the [source-review receipt](evidence/phase2-mechanical.json).

The P10 checkpoint now lists twelve direct implementation includes. They
provide access to private decoder, motion, diagnostics and integration
state, hardware simulation or the legacy fault adapter. Six lowercase
macros remain. Three mirror `PWM_CHx_DIV` or `XOSC_MHz` and three
interpose IMU or terrain calls in the transport fixture. The exact
directives and source locations remain open for individual review.
[Current lexical findings](evidence/phase2-mechanical.json#phase2_current_lexical_fixtures)

The fit-score and bounded-work harnesses now link the decoder in separate
translation units. The original fit and work probes are available only to
the host fixtures. Ten direct implementation includes remain from the prior
inventory. [Fit-score evidence](evidence/phase2-mechanical.json#phase2_separate_barcode_fit_fixture) ·
[Bounded-work evidence](evidence/phase2-mechanical.json#phase2_separate_barcode_work_fixture)

The Buddy 4 transport fixture now uses uppercase interposition names. The
three remaining lowercase definitions mirror imported register or clock
macros required by the firmware headers. The transport fixture now compiles
Buddy 4 separately, leaving nine direct implementation includes in other
fixtures. [Macro evidence](evidence/phase2-mechanical.json#phase2_terrain_transport_macro_names) ·
[Transport evidence](evidence/phase2-mechanical.json#phase2_separate_terrain_transport_fixture)

The sonar task fixture now links Buddy 5 separately. Eight direct
implementation includes remain in other fixtures.
[Sonar task evidence](evidence/phase2-mechanical.json#phase2_separate_sonar_task_fixture)

The built-in calibration fixture also links the sampling owner separately.
Seven direct implementation includes remain. Its report-task stubs are
fixture boundaries and the fixture exercises the sampling task.
[Built-in calibration evidence](evidence/phase2-mechanical.json#phase2_separate_builtin_calibration_fixture)

The diagnostic publication fixture also compiles the implementation
separately. Six direct implementation includes remain in other fixtures.
[Diagnostic evidence](evidence/phase2-mechanical.json#phase2_separate_diagnostic_publication_fixture)

The map-freeze fixture links the command owner, facade and sampling
runtime separately. Five direct implementation includes remain in other
fixtures. The [post-map audit](evidence/phase2-mechanical.json#phase2_post_map_fixture_review)
records all five directives and the three remaining lowercase macros.
[Map-freeze evidence](evidence/phase2-mechanical.json#phase2_separate_map_freeze_fixture)

The barcode phase-boundary fixture also links the decoder separately. Four
implementation includes remain. The runner extracts each revision’s phase
constants and keeps the original boundary assertions and complete traces.
[Barcode phase-boundary evidence](evidence/phase2-mechanical.json#phase2_separate_decoder_boundary_fixture)

The retained-motion fixture also links its owner separately. Three direct
implementation includes remain in the operator, runtime and superseded
wheel-adapter fixtures. The motion checks retained the exact trace and
each revision’s original storage size output.
[Retained-motion evidence](evidence/phase2-mechanical.json#phase2_separate_retained_motion_fixture)

The GP20/GP21 fixture also links the sampling owner separately. Two direct
implementation includes remain in the sampling runtime and superseded
wheel-adapter fixtures. The GP20/GP21 console and trace pairs matched.
[Operator evidence](evidence/phase2-mechanical.json#phase2_separate_operator_control_fixture)

The integrated sampling and terrain slope fixtures now link the sampling
owner separately. The sole remaining direct implementation include is
in the superseded wheel-adapter fixture, which has no supported build
recipe. Retain it as an open historical source issue.
[Sampling runtime evidence](evidence/phase2-mechanical.json#phase2_separate_sampling_runtime_fixture)
The preserved adapter calls `report_motion_faults`, which is absent from
both the fixed baseline and the current implementation. The three remaining
macro names correspond to `PWM_CHx_DIV` in
`include/sys/sysdepend/cpu/rp2040/sysdef.h` and `XOSC_MHz` in
`include/sys/sysdepend/pico_rp2040/sysdef.h`. The module and shim names
must still agree for the affected host builds.
[Current source locations](evidence/phase2-mechanical.json#phase2_remaining_mandatory_checkpoints)

### Remaining conditional nesting

The matched compiler reviews found 74 sites before this batch and 67
afterward. The telemetry layout switches removed six sites, and the
terrain startup cleanup removed one. The table describes the remaining
sites. [Final location inventory](evidence/phase2-mechanical.json#phase2_end_of_phase_verification_20261122)

The 67 findings remain pending under 8.2.b at the Phase 2 submission
closeout. Further per-site trials and formal approvals are deferred unless
the rubric or lecturer requires them. [Phase 2 closeout](../CURRENT_STATUS.md#phase-2-closeout-for-submission)

Trial references identify alternatives
already attempted in that module. A row can also contain individual
branches that have yet to be compared. The instruction and stack differences
from a trial are reasons to keep the original path until its effects can
be evaluated. They do not establish a functional regression by themselves.

| Source and procedure | Sites | Preserved behaviour and earlier trials |
| --- | ---: | --- |
| `avoid_obstacle.c`, `avoid_obstacle_step` | 31 | Keep scan aim, stop and completion checks ahead of planning. Keep the action acknowledgement before leg updates, the reverse-distance and alternate-route guards, and the fresh-range and blockage checks before straight legs. Retain the conditional axle placement and tape STOP at handoff. See P45, P51, P53, P56, P61, P63, P65 and P67. |
| `bc_decoder.c`, `bc_decoder_update` | 3 | Check capacity before storing a completed run. Validate the last run and candidate before setting the pending result. See P57. |
| `bc_decoder.c`, `bc_decoder_record_update` | 2 | Learn noise only on the existing near-paper path. On resumed ink, preserve the difference between provisional or early work and other processing. See P51, P60, P66 and P67. |
| `bc_decoder.c`, `rec_extract_runs`, `work_primary`, `work_advance` | 4 | Interpolate reference crossings before selecting the first edge or an interior run. Preserve capacity rejection and paired bias checks within the budgeted work steps. See P41, P54, P62 and P64. |
| `buddy5.c`, `obstacle_task` | 9 | Preserve initialization, servo angle and scan ownership. Publish a completed sweep or peek through its existing path. Keep current forward-echo freshness, fault and recovery decisions. See P54, P60, P61 and P66. |
| `s3_integration.c`, `sensor_sample_task` | 6 | Preserve the lock and STOP checks on RESET staging, completion replies to the right origin, and per-channel validity before debug publication. See P50, P51, P60 and P64. |
| `op_logic.c`, `cal_collect`, `op_logic_button_update`, `op_logic_step` | 4 | Keep arming ahead of a press edge, callback acknowledgement ahead of STOP release or warning, and calibration failure reporting on its current completion path. See P46, P55 and P59. |
| `bc_capture.c`, `bc_capture_binary_next` | 2 | Validate summary, header and sample eligibility before writing the packet or advancing the retained cursor. See P30 and P56. |
| `s3_drive.c`, `obstacle_control`, `s3_drive_process` | 2 | Issue a junction turn only after advance completes. Keep emergency stopping during another manoeuvre ahead of ordinary obstacle decisions. See P50 and P58. |
| `buddy4.c`, `terrain_task` | 1 | Apply recalibration only under the shared terrain mutex and as part of the current reset sequence. See P46. |
| `terr_data.c`, `terr_data_process` | 2 | Publish heading rate and integrate travel only after separate continuity and range checks. See P46 and P55. |
| `line_navigation.c`, `process_barcode` | 1 | Report the capture end pose only after checking start validity and the odometry epoch. See P46. |

P41 and P45 cover earlier telemetry alternatives. P59 covers the earlier
terrain startup alternative. Those trials remain historical evidence. The
current switch cases preserve field order and early packet rejection in
source. The startup rewrite preserves the distinction between a failed
cyclic start and failure after the cycle started, and only the latter path
stops the cycle. The baseline semantic and packet checks matched. Both
rewrites changed allocated Arm instructions, and the telemetry encoder's
stack estimate moved from 48 to 40 bytes. Physical timing verification
remains pending. [Final comparison](evidence/phase2-mechanical.json#phase2_end_of_phase_verification_20261122)

**Rule 8.5.b, pending.** Host RTOS fixtures use `setjmp` and `longjmp` to
return control to the test driver after a chosen number of cycles in an
otherwise unbounded task. Host simulator error paths also call `exit`.
Replacing these calls can change which operations the tests observe or the
failure exit seen by the invoking script. The 30 source occurrences and
their snapshot hashes are recorded under `phase2_owner_review` in the
[comparison receipt](evidence/phase2-mechanical.json).

The standard's preferences for a single exit, avoiding continue, local declaration
placement and shortest branch first do not authorize changing evaluation order,
resource cleanup or state transitions. Record reviewed retention under the
specific rule; do not label it a violation of an unconditional requirement.

Precision-sensitive fields are also tracked in the
[Change Requirement 1 inventory](CR1_FIELD_INVENTORY.md). Their widths and
quantization must not be inferred from displayed decimal precision.

**Rule 3.1.b, pending.** Consecutive assignments to members of different
name lengths in `app_start_task` illustrate a conflict between exactly one
space before an assignment and aligning adjacent assignments under rule
3.2.c. The formatter aligns these assignments. Approval of that interpretation
is outstanding. [Task initialization](../../app_program/app_main.c) ·
[Spacing review](evidence/phase2-mechanical.json)

**Rule 6.3.a, reviewed barcode accessors.** Replacing either recorded-sample
accessor with an inline function changed the Arm instructions in
`rec_extract_runs`. Both accessors retain their parenthesized array indices.
The capture-terminal macro did admit a private inline replacement with
matching target output in every compared profile.
[Decoder](../../app_program/subsystem3/bc_decoder.c) · [Compiler comparison](evidence/phase2-mechanical.json)

### Remaining interface and source-layout cases

- **P01, dialect.** All 241 inspected host syntax variants pass strict C99.
  Two of the 61 production Arm translation units encounter diagnostics in
  imported headers. The compiler and exact diagnostics are bound in
  `phase2_types_and_fixtures`. The original assertion conversion is complete.
- **P10, implementation includes.** Separate compilation of the course and
  slope fixtures reduced the current count to 12. Six lowercase macro names
  remain, covering three platform spellings and three terrain test
  substitutions. The counts higher on this page describe earlier snapshots.
- **Rule 1.1.d.** Shared kernel fixtures still define `IMPORT` and `LOCAL`
  as linkage keywords for historical Buddy 1 sources. Removing them from a
  fixture that also compiles the fixed baseline can break that comparison.
- **Rules 3.1.l and 8.4.c.** The 15 prescribed `for (;;)` loops have two
  adjacent semicolons. A separate bounded loop has an empty increment clause.
  The literal semicolon-spacing interpretation needs agreement.
- **Rule 4.1.b.** Five basename groups repeat across isolated include roots.
  They are `mqtt_utk`, `sysdef`, `timeouts`, `tmonitor` and the
  `wifi_sec` prefix. The fixtures stand in for existing include paths.
  Each group needs a compatibility decision before renaming.

The interface receipt lists the individual paths, checked configurations
and layouts. These cases await disposition.
[Source and compiler evidence](evidence/phase2-mechanical.json)

**P08 and rule 5.1.a.** Twenty-one kernel-compatible type names in host
fixtures mirror the RTOS headers. The MQTT integration also exposes
`T_MQTT_UTK_APP_MSG` to its consumers. The fixture/type checkpoint lists
each spelling and its paths for compatibility review.
[Alias inventory](evidence/phase2-mechanical.json)

**Rule 5.3.b.** Widening signed operands in two candidate rewrites changed
the generated Arm code for IR debug packing and terrain debug reporting.
The source retains the earlier expressions at those sites. The same
checkpoint records the accepted unsigned shift edits and the two compiler
comparisons. Disposition of the retained expressions is open.
[Compiler comparisons](evidence/phase2-mechanical.json)

**P11 and rules 6.2.a and 8.2.b.** Smaller telemetry snapshot and capture
helpers were tested after the source-section cleanup. Moving either the
capture episode setup or wire batch validation changed allocated Arm
content with the same stack estimate. The inline telemetry snapshot
extraction also changed allocated content. The tested release source keeps
those operations in their owning functions.
[Trial and preservation results](evidence/phase2-mechanical.json)

**P13 and rule 1.3.b.** The on-device Buddy 1 command-drain loop still
explains its empty body beside the opening brace. Moving the comment to its
own line shifts a following assertion location. An extracted helper that
kept those locations changed the allocated Arm instructions. The candidate
keeps the loop and its explanation together. The later function contracts
preserve all device assertion positions in the checked snapshot.
[Definition and symbol comparison](evidence/phase2-mechanical.json)

**P02, P06 and P08.** The compiler review recorded 299 named bit-fields.
The 88 unsigned one-bit entries include encoded states and Boolean flags,
with ten named fields showing different offsets across profiles or compiler
targets. The six packed IR and capture records need ABI and allocation
checks before changing their declaration form. The nine weak lwIP boundary
functions provide the fallback hook implementations. Replacing those
extensions requires preserving hook selection as well as the tested packets.
[Compiler field and boundary review](evidence/phase2-mechanical.json)

**P09 and the host comparison macros.** The remaining unparenthesized
macro bodies generate declarations, initializer fragments or complete
statements. The test fixtures also use structure selectors, stringified
field labels and braced setup. Those uses require the C syntax listed in
the macro inventory. The current snapshot keeps the ten body exceptions
and their exact parameter uses pending review.
[Macro inventory and compiler evidence](evidence/phase2-mechanical.json)

**P11 and the obstacle branches.** A narrower aiming helper and a hoisted
reverse-clearance guard both changed allocated Arm output. The candidate
keeps those two blocks in the obstacle owner. The accepted terrain and hump
changes reduce nested control flow with matching Arm output and traces.
[Accepted and declined trials](evidence/phase2-mechanical.json)
