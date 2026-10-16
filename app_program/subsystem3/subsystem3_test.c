/*
 * File: subsystem3_test.c
 * Purpose: Target hook; portable algorithm tests run separately on the host.
 */

#include "subsystem3.h"

#if APP_TEST
void subsystem3_test(void)
{
    tm_printf((UB *) "[TEST] subsystem3 SKIP: stationary host suite: tools/test-buddy3.ps1; hardware validation pending.\n");
}
#endif /* APP_TEST */
