/* GP20/GP21 operator buttons for the S3_MOTION profile (see operator_logic.h).
 * GP20 = next calibration step / START (hold 2 s = recalibrate),
 * GP21 = STOP at any time. Software stop only: the power switch remains the
 * independent cutoff. Returns E_NOSPT in every other profile. */
#ifndef OPERATOR_PANEL_H
#define OPERATOR_PANEL_H
#include "../app.h"
#define OP_PRI_PANEL (24) /* below every subsystem band and diagnostics (23) */
ER operator_panel_start(void);
#endif
