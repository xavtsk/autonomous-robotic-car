# Change Requirement 1 evidence scripts

> Current guide: [documentation index](../../docs/README.md), [baseline status](../../docs/CURRENT_STATUS.md), and [build/test commands](../../docs/BUILD_AND_TEST.md). Dated results below retain their original evidence limits.

Results and explanation: [`docs/CHANGE_REQUIREMENT_1_EVIDENCE.md`](../../docs/CHANGE_REQUIREMENT_1_EVIDENCE.md).

| File | What it does |
|---|---|
| `run_all.sh` | Runs everything below plus all host test suites; one log per step in `logs/` |
| `check_widths.py` | Checks the selected field ranges, minimum widths, packet totals and bound C declarations listed in the [submission table](../../docs/compliance/CR1_SUBMISSION_WIDTHS.md). |
| `measure.py` | Builds the firmware, linked RAM per image and per buddy, packed vs unpacked storage sizes (Arm compiler) |
| `struct_sizes.c` | The storage types `measure.py` sizes; never linked into the firmware |
| `bend_filter_check_main.c` | Old float vs new integer bend filter: same decisions |
| `timing_check.py` | Screens a recorded diagnostics log for missed cycles. For motion at 10 ms and IR at 5 ms it flags gaps of at least 20 ms and 10 ms respectively. |
