/** @file buddy3_test.h
 * On-device self-check entry point for Buddy 3.
 */
#ifndef BUDDY3_TEST_H
#define BUDDY3_TEST_H

#include "buddy3.h"

#if APP_TEST
/** Run Buddy 3's built-in checks and report their results to the console. */
void subsystem3_test (void);
#endif

#endif /* BUDDY3_TEST_H */

/* End of file. */

