# Documentation index

The submission results describe the final software checks. The supporting
records identify the firmware revisions and test conditions for earlier results.

| Read first | Purpose |
| --- | --- |
| [Submission results](CURRENT_STATUS.md) | Demo status, final software results and dated firmware comparisons. |
| [Build and test](BUILD_AND_TEST.md) | Explicit build profiles, host commands and result interpretation. |
| [Team ownership](../TEAM.md) | Pins, peripherals, tasks and module owners. |
| [Testing guide](../testing/README.md) | Subsystem, integration and hardware test records. |
| [Dashboard guide](DASHBOARD.md) | Laptop dashboard and communication behaviour. |

| Compliance and interfaces | Purpose |
| --- | --- |
| [BARR-C matrix](compliance/BARR_RULE_MATRIX.md) | Complete chapter/subclause ledger and exact checker limitations. |
| [BARR-C exceptions](compliance/BARR_EXCEPTIONS.md) | Affected rules, source locations, reasons for retaining code and recorded review status. |
| [CR1 data representation table](compliance/CR1_SUBMISSION_WIDTHS.md) | Current scoped widths, valid ranges and codec totals. |
| [CR1 evidence](CHANGE_REQUIREMENT_1_EVIDENCE.md) | Submission field table, matched RAM and message sizes, software results and timing limits. |
| [MQTT protocol](../app_program/subsystem1/MQTT_PROTOCOL.md) | Topic and byte contracts. |
| [Remote control](REMOTE_CONTROL_PROTOCOL.md) | START/STOP/RESET request, reply and ownership contracts. |
| [Barcode capture protocol](BARCODE_WIRE_PROTOCOL.md) | Trace transfer and replay data. |

## Supporting and historical records

| Record | Purpose |
| --- | --- |
| [Archived hardware comparison procedure](PHYSICAL_VALIDATION.md) | Reference method for comparing course behaviour, task timing and network recovery on two firmware images. |
| [Repository cleanup](REPOSITORY_CLEANUP.md) | Phase 1 housekeeping decisions and earlier Phase 2 verification. |
| [Cleanup checkpoints through Phase 3](history/2026-11-22-compliance-checkpoints.md) | Earlier status reports with their original scope and results. |
| [Broader CR1 declaration inventory](https://github.com/xavtsk/autonomous-robotic-car/blob/a3de92a3465749f962540bcb75c275959e1a9ef0/docs/compliance/CR1_FIELD_INVENTORY.md) | Historical declaration counts with source links pinned to the reviewed revision. |
| [Abbreviations](compliance/ABBREVIATIONS.md) | Shared terminology. |
| [Module templates](compliance/templates/README.md) | Header/source structure and documentation. |
| [Alias/assertion batch](compliance/ALIAS_ASSERT_BATCH.md) | Exact type-alias/assertion transformation and verification limits. |
| [Formatting batch](compliance/FORMAT_BATCH.md) | Token preservation, target-image comparison and executed trace evidence. |
| [Function documentation](compliance/FUNCTION_DOCUMENTATION.md) | Contracts reviewed and scope of the documentation check. |
| [Generated reference](compliance/GENERATED_REFERENCE.md) | Pinned Doxygen generation, retained warnings and compiler-definition comparison. |
| [Operator observations](compliance/OPERATOR_TRACE.md) | Exact button/state/console traces against the fixed baseline. |
| [Estimator module rename](compliance/MOTION_MODULE_RENAME.md) | Filename/API alignment with unchanged implementation and link order. |
| [Template boundary correction](compliance/TEMPLATE_BOUNDARY_CORRECTION.md) | Provenance-based restoration of unchanged imported adapters. |
| [Conditional compilation](compliance/CONDITIONAL_COVERAGE.md) | Build profile records and conditional branches observed across those profiles. |
| [Motion contracts](compliance/MOTION_FUNCTION_CONTRACTS.md) | Exact PID, packed motion-word and failed-output semantics. |
| [Sensor and terrain contracts](compliance/SENSOR_TERRAIN_FUNCTION_CONTRACTS.md) | IMU transaction failures, hump state and classification priority. |

### Subsystem guides

[Buddy 1](../app_program/subsystem1/README.md),
[Buddy 2](../app_program/subsystem2/README.md),
[Buddy 3](../app_program/subsystem3/README.md),
[Buddy 4](../app_program/subsystem4/README.md),
[Buddy 5](../app_program/subsystem5/README.md),
[controller](../app_program/controller/README.md).

The [25 October handover](history/CLAUDE_HANDOFF_2026-10-25.md),
[19 November cleanup record](COMPLIANCE_CLEANUP.md), October integration
notes, dated reviews, logs and upstream documents describe their original
revisions and sessions.
