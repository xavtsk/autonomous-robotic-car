/*
 * File: subsystem4.h
 * Owner: Buddy 4 - IMU-based motion and terrain monitoring.
 * Purpose: RTOS lifecycle interface. Application functionality is not implemented.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_MAIN (18)

/* Initialization creates objects only; start may launch tasks once implemented. */
ER subsystem4_init(void);
ER subsystem4_start(void);

#if APP_TEST
void subsystem4_test(void);
#endif

#endif /* SUBSYSTEM4_H */
