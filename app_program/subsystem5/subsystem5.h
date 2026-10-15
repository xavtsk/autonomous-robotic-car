/*
 * File: subsystem5.h
 * Owner: Buddy 5 - Ultrasonic scanning and obstacle avoidance.
 * Purpose: RTOS lifecycle interface. Application functionality is not implemented.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_MAIN (20)

/* Initialization creates objects only; start may launch tasks once implemented. */
ER subsystem5_init(void);
ER subsystem5_start(void);

#if APP_TEST
void subsystem5_test(void);
#endif

#endif /* SUBSYSTEM5_H */
