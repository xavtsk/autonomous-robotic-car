/** @file sysdef.h
 * @brief Provide the fake timer register used by controller runtime tests.
 */
#ifndef INTEGRATION_RUNTIME_SYSDEF_H
#define INTEGRATION_RUNTIME_SYSDEF_H
#include <stdint.h>
#define TIMER_TIMERAWL 0U
uint32_t runtime_read_clock (uint32_t address);

/** @brief Read the integration fixture's timer through the register API.
 * @param address Register address passed to the fixture's clock callback.
 * @return The current 32-bit simulated timer value.
 */
static inline uint32_t in_w (uint32_t address)
{
    return runtime_read_clock(address);
}
#endif

/* End of file. */

