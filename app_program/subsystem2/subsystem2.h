/*
 * File: subsystem2.h
 * Owner: Buddy 2 - Motion control using motors and wheel encoders.
 * Purpose: RTOS lifecycle interface. Application functionality is not implemented.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

#define S2_PRI_MAIN (14)

/* Initialization creates objects only; start may launch tasks once implemented. */
ER subsystem2_init(void);
ER subsystem2_start(void);

#if APP_TEST
void subsystem2_test(void);
#endif

#endif /* SUBSYSTEM2_H */
