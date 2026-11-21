/** @file buddy2_test.h
 * On-device self-check entry point for Buddy 2.
 */
#ifndef BUDDY2_TEST_H
#define BUDDY2_TEST_H

#include "buddy2.h"

#if APP_TEST
/** @brief Run the selected motion self-test or calibration after startup.
 * The S2_TEST_MOVE profile includes physical moves. S2_TEST_CALIBRATE runs
 * the calibration routine in place of the unit tests. Results go to the
 * device console.
 */
void subsystem2_test (void);
#endif

#endif /* BUDDY2_TEST_H */

/* End of file. */

