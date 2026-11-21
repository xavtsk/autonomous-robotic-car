/** @file sysdef.h
 * @brief Provide a controlled timer register for diagnostic host tests.
 */
#ifndef DIAGNOSTICS_TEST_SYSDEF_H
#define DIAGNOSTICS_TEST_SYSDEF_H
#include <stdint.h>
#define TIMER_TIMERAWL UINT32_C(0x40054028)
uint32_t diagnostics_test_in_w (uint32_t address);

/** @brief Read the diagnostic fixture's simulated timer register.
 * @param address RP2040 register address requested by the diagnostic code.
 * @return The 32-bit value supplied by the host fixture.
 */
static inline uint32_t in_w (uint32_t address)
{
    return diagnostics_test_in_w(address);
}
#endif

/* End of file. */

