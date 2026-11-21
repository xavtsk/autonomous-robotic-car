/** @file test_core_main.h
 * @brief Core test suite and Code 39 fixture shared with the trace runner.
 */
#ifndef TESTING_SUBSYSTEM3_TEST_CORE_MAIN_H
#define TESTING_SUBSYSTEM3_TEST_CORE_MAIN_H

#include <stdint.h>

int test_core_run (void);
#ifdef TEST_NAV_TRACE
void test_core_make_runs (char letter, uint32_t * runs);
#endif

#endif /* TESTING_SUBSYSTEM3_TEST_CORE_MAIN_H */

/* End of file. */

