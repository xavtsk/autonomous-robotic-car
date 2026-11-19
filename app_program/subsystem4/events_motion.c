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
 * @param turn Signed turn rate in degrees per second.
 * @param turn_valid Enables the turn comparison when true.
 * @param impact Selects impact before all numerical comparisons when true.
 * @return One category; no input or shared state is modified.
 */
terrain_state_t motion_classify (float32_t pitch, float32_t speed,
                                 float32_t longitudinal_g, float32_t turn,
                                 bool turn_valid, bool impact)
{
    if (impact)
    {
        return TERRAIN_IMPACT;
    }
    if (turn_valid && fabsf(turn) > 15)
    {
        return TERRAIN_TURNING;
    }
    if (fabsf(speed) > 0.01f && pitch > 4)
    {
        return TERRAIN_CLIMBING;
    }
    if (fabsf(speed) > 0.01f && pitch < -4)
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

