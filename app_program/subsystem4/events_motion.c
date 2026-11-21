/** @file events_motion.c
 * @brief Classify terrain motion from pitch, speed and longitudinal
 * acceleration.
 */
#include "events_motion.h"
#include "../app_types.h"
#include <math.h>

/**
 * @brief Evaluate the stateless priority contract in events_motion.h.
 * @param pitch Nose-up pitch in degrees; not range-checked here.
 * @param speed Signed speed in metres per second.
 * @param longitudinal_g Longitudinal dynamic acceleration in g.
 * @param turn_dps Signed turn rate in degrees per second.
 * @param b_is_turn_valid Enables the turn comparison when true.
 * @param b_has_impact Selects impact before all numerical comparisons when
 * true.
 * @return One category; no input or shared state is modified.
 */
events_motion_state_t events_motion_classify (float32_t pitch, float32_t speed,
                                              float32_t longitudinal_g,
                                              float32_t turn_dps,
                                              bool      b_is_turn_valid,
                                              bool      b_has_impact)
{
    if (b_has_impact)
    {
        return TERRAIN_IMPACT;
    }

    if (b_is_turn_valid && (fabsf(turn_dps) > 15))
    {
        return TERRAIN_TURNING;
    }

    if ((fabsf(speed) > 0.01f) && (pitch > 4))
    {
        return TERRAIN_CLIMBING;
    }

    if ((fabsf(speed) > 0.01f) && (pitch < -4))
    {
        return TERRAIN_DESCENDING;
    }

    if (fabsf(longitudinal_g) > 0.12f)
    {
        return TERRAIN_ACCELERATING;
    }

    return fabsf(speed) < 0.01f ? TERRAIN_STATIONARY : TERRAIN_MOVING;
}

/* End of file. */

