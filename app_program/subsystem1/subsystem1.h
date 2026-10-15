/*
 * File: subsystem1.h
 * Owner: Buddy 1 - WiFi communication, command and telemetry.
 * Purpose: RTOS lifecycle interface. Application functionality is not implemented.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_PRI_MAIN (12)

/* Initialization creates objects only; start may launch tasks once implemented. */
ER subsystem1_init(void);
ER subsystem1_start(void);

#if APP_TEST
void subsystem1_test(void);
#endif

#endif /* SUBSYSTEM1_H */
