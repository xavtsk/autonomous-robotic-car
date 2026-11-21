/** @file buddy5_test.h
 * On-device self-check entry point for Buddy 5.
 */
#ifndef BUDDY5_TEST_H
#define BUDDY5_TEST_H

#include "buddy5.h"

#if APP_TEST
/** @brief Check pulse conversion, timeout rejection and an observation
 * packet round trip using synthetic inputs. Print the result on the console.
 */
void subsystem5_test (void);
#endif

#endif /* BUDDY5_TEST_H */

/* End of file. */

