# Source-location diagnostic boundary

The [hash-bound audit](evidence/source-location-diagnostics.json) found no literal
`__FILE__` or `__LINE__` tokens in the 270 owned C/header inputs inspected. There
are 63 indirect uses through `TEST_CHECK`: 38 in `buddy1_test.c`, 22 in
`buddy2_test.c`, and 3 in `buddy4_test.c`. The evidence lists every check ID and
its current and fixed-baseline lexical invocation line. These are source
locations, not measured compiler expansions or captured device output.

The imported, preserved `app_program/test_report.h` adds file/line information
only to failed checks. Its text still matches fixed baseline `4ad2cc4`.
Formatting, inserted documentation, and renamed source paths can therefore
change failed `APP_TEST` console messages. PASS and summary messages do not
include this metadata. The inspected route is `tm_printf` through the T-Monitor
console to USB CDC and/or UART; it does not publish these messages through MQTT.
The production profile sets `TEST=0`, while the separate self-test profile sets
`TEST=1`.

This is an explicit observable-output limitation requiring disposition in the
preservation record. It grants no exception or compliance approval. The audit
does not establish a normal-production MQTT byte change from source metadata,
nor does it prove whole-program output equivalence, every imported macro's
expansion, or physical timing. The JSON binds the inspected working-tree inputs
by hash; later source edits require a fresh audit.
