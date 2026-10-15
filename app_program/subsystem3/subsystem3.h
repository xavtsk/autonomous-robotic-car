/*
 * File: subsystem3.h
 * Owner: Buddy 3 - IR line following and barcode decoding.
 * Purpose: RTOS lifecycle interface. Application functionality is not implemented.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_MAIN (16)

/* Initialization creates objects only; start may launch tasks once implemented. */
ER subsystem3_init(void);
ER subsystem3_start(void);

#if APP_TEST
void subsystem3_test(void);
#endif

#endif /* SUBSYSTEM3_H */
