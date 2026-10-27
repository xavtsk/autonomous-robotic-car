/* Portable lease check shared by the motion owner and host tests. */
#ifndef MOTION_LEASE_H
#define MOTION_LEASE_H
#include <stdbool.h>
#include <stdint.h>
#define MOTION_NAVIGATION_LEASE_US (30000U)
static inline bool motion_lease_expired(bool enabled, uint32_t now,
                                        uint32_t refreshed)
{
    return enabled &&
           ((uint32_t)(now - refreshed) > MOTION_NAVIGATION_LEASE_US);
}
#endif
