/** @file motion_lease.h
 * @brief Portable lease check shared by the motion owner and host tests.
 */
#ifndef MOTION_LEASE_H
#define MOTION_LEASE_H
#include <stdbool.h>
#include <stdint.h>
#define MOTION_NAVIGATION_LEASE_US (30000U)

/** @brief Return true when an enabled navigation lease is strictly older than
 * 30,000 microseconds, using wrapping unsigned timestamps.
 * @pre Both timestamps come from the same wrapping microsecond counter.
 * The elapsed interval used for a live lease fits one counter revolution.
 */
static inline bool motion_lease_expired (bool b_is_enabled, uint32_t now,
                                         uint32_t refreshed)
{
    return b_is_enabled && ((now - refreshed) > MOTION_NAVIGATION_LEASE_US);
}
#endif

/* End of file. */

