/** @file buddy2.h
 * @brief RTOS lifecycle interface.
 *
 * @par Ownership
 * Buddy 2 - Motion control using motors and wheel encoders.
 *
 * Kernel objects (TEAM.md section 6): 1 task, 1 mutex, 1 message buffer
 * (report queue), 1 cyclic handler.
 * Other subsystems use the functions in motion.h; they need no object IDs.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

#define S2_PRI_MAIN (14) /* Use the highest application priority band. */
#define S2_MOTION_CORE (TP_PRC1) /* core 0                                */
#define S2_PERIOD_MS (10U)       /* Set the control period in milliseconds. */

/* Initialization creates objects only; start launches the motion task. */
ER subsystem2_init (void);
ER subsystem2_start (void);

#if APP_TEST
void subsystem2_test (void);
#endif

#endif /* SUBSYSTEM2_H */

/* End of file. */

