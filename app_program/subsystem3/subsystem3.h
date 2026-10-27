/*
 * File: subsystem3.h
 * Owner: Buddy 3 - IR line following and barcode decoding.
 * Purpose: RTOS lifecycle interface for the opt-in IR bench diagnostic.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_MAIN (16)
#define S3_PRI_REPORT (17)
#define S3_SAMPLE_MS (10U)

/* APP_IR_BENCH starts stationary tasks; APP_S3_MOTION starts the disarmed
 * Buddy 2 integration. These profiles are mutually exclusive. */
ER subsystem3_init(void);
ER subsystem3_start(void);

#if APP_TEST
void subsystem3_test(void);
#endif

#endif /* SUBSYSTEM3_H */
