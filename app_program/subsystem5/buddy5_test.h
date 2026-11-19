/** @file buddy5_test.h
 * On-device self-check entry point for Buddy 5.
 */
#ifndef BUDDY5_TEST_H
#define BUDDY5_TEST_H

#include "buddy5.h"

#if APP_TEST
/** Run Buddy 5's built-in checks and report their results to the console. */
void subsystem5_test (void);
#endif

#endif /* BUDDY5_TEST_H */
