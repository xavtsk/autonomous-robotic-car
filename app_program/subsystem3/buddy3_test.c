/** @file buddy3_test.c
 * @brief Target hook; portable algorithm tests run separately on the host.
 */
#include "buddy3_test.h"

#if APP_TEST
/** @brief Report that this target defers Buddy 3 checks to the host suite. */
void subsystem3_test (void)
{
    tm_printf((UB *)"[TEST] subsystem3 SKIP: stationary host suite: "
                    "tools/test-buddy3.ps1; hardware validation pending.\n");
}
#endif /* APP_TEST */

/* End of file. */

