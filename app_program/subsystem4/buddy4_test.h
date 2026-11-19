/** @file buddy4_test.h
 * On-device self-check entry point for Buddy 4.
 */
#ifndef BUDDY4_TEST_H
#define BUDDY4_TEST_H

#include "buddy4.h"

#if APP_TEST
/** Run Buddy 4's built-in checks and report their results to the console. */
void subsystem4_test (void);
#endif

#endif /* BUDDY4_TEST_H */
