/* Optional USB observations. These functions never request or reset motion. */
#ifndef ROBOT_DIAGNOSTICS_H
#define ROBOT_DIAGNOSTICS_H

#include "../app.h"
#include <stdint.h>

#ifndef APP_DIAGNOSTICS
#define APP_DIAGNOSTICS 0
#endif

enum { ROBOT_DIAG_MOTION = 0, ROBOT_DIAG_IR, ROBOT_DIAG_SOURCE_COUNT };

#if APP_DIAGNOSTICS
/* Init before starting producers. Each source must have exactly one task
 * writer: the motion task or the IR sampling task. Never call from an ISR.
 * Disabled builds retain no diagnostic objects or code. */
ER robot_diagnostics_init(void);
uint32_t robot_diagnostics_now_us(void);
void robot_diagnostics_record(unsigned source, uint32_t started_us,
                              uint32_t finished_us);
/* Call from the low-priority reporter. A failed polling lock simply defers
 * this report. No console or Buddy 2 call runs with the diagnostic lock held.
 * Encoder/status observations are sequential, not one simultaneous sample. */
void robot_diagnostics_report(void);
#endif

#endif
