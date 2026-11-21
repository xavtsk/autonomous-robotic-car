/** @file buddy4_test.h
 * On-device self-check entry point for Buddy 4.
 */
#ifndef BUDDY4_TEST_H
#define BUDDY4_TEST_H

#include "buddy4.h"

#if APP_TEST
/** @brief Check pitch sign, signed pitch packing and invalid peak height
 * using synthetic samples. Report each result on the device console.
 */
void subsystem4_test (void);
#endif

#endif /* BUDDY4_TEST_H */

/* End of file. */

