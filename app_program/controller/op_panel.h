/** @file op_panel.h
 * @brief GP20/GP21 operator buttons for the S3_MOTION profile (see op_logic.h).
 *
 * GP20 = next calibration step / START (hold 2 s = recalibrate),
 * GP21 = STOP at any time. Software stop only: the power switch remains the
 * independent cutoff. Returns E_NOSPT in every other profile.
 */
#ifndef OPERATOR_PANEL_H
#define OPERATOR_PANEL_H
#include "../app.h"
/* Run below the subsystems and diagnostics. */
#define OP_PRI_PANEL (24)
ER op_panel_start (void);
#endif

/* End of file. */

