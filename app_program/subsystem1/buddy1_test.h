/** @file buddy1_test.h
 * On-device self-check entry point for Buddy 1.
 */
#ifndef BUDDY1_TEST_H
#define BUDDY1_TEST_H

#include "buddy1.h"

#if APP_TEST
/** Run Buddy 1's built-in checks and report their results to the console. */
void subsystem1_test (void);
#endif

#endif /* BUDDY1_TEST_H */

/* End of file. */

