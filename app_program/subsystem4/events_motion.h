/** @file events_motion.h
 * @brief Classify terrain motion from pitch, speed and longitudinal
 * acceleration.
 */
#ifndef BUDDY4_MOTION_EVENTS_H
#define BUDDY4_MOTION_EVENTS_H
#include "../app_types.h"
#include <stdbool.h>

/* Exactly 3 bits. Flags/measurement validity are separate. */
typedef enum
{
    TERRAIN_UNKNOWN,
    TERRAIN_STATIONARY,
    TERRAIN_MOVING,
    TERRAIN_ACCELERATING,
    TERRAIN_TURNING,
    TERRAIN_CLIMBING,
    TERRAIN_DESCENDING,
    TERRAIN_IMPACT
} terrain_state_t;

terrain_state_t motion_classify (float32_t pitch, float32_t speed,
                                 float32_t longitudinal_g, float32_t turn_dps,
                                 bool turn_valid, bool impact);
#endif

/* End of file. */

