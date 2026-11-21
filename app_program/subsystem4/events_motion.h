/** @file events_motion.h
 * @brief Classify terrain motion from pitch, speed and longitudinal
 * acceleration.
 */
#ifndef BUDDY4_MOTION_EVENTS_H
#define BUDDY4_MOTION_EVENTS_H
#include "../app_types.h"
#include <stdbool.h>

/* The wire state occupies three bits. Separate flags carry validity. */
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
} events_motion_state_t;

/**
 * @brief Select one motion category using the existing priority order.
 * @param pitch Vehicle pitch in degrees, positive nose-up.
 * @param speed Signed forward speed in metres per second.
 * @param longitudinal_g Longitudinal dynamic acceleration in g.
 * @param turn_dps Signed turn rate in degrees per second.
 * @param b_is_turn_valid Whether the turn-rate comparison is enabled.
 * @param b_has_impact Whether impact takes precedence over all other
 * categories.
 * @return Impact, turning, climbing, descending, accelerating, stationary or
 * moving, in that priority order. This function does not return UNKNOWN.
 * @note Inputs are not checked for finiteness or physical range; ordinary
 * floating comparisons determine the outcome. No state or pointers are used.
 */
events_motion_state_t events_motion_classify (float32_t pitch, float32_t speed,
                                              float32_t longitudinal_g,
                                              float32_t turn_dps,
                                              bool      b_is_turn_valid,
                                              bool      b_has_impact);
#endif

/* End of file. */

