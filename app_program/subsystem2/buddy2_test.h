/** @file buddy2_test.h
 * On-device self-check entry point for Buddy 2.
 */
#ifndef BUDDY2_TEST_H
#define BUDDY2_TEST_H

#include "buddy2.h"

#if APP_TEST
/** Run Buddy 2's built-in checks and report their results to the console. */
void subsystem2_test (void);
#endif

#endif /* BUDDY2_TEST_H */
