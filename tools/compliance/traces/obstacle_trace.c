/** @file obstacle_trace.c
 * @brief Existing avoidance scenarios/assertions, with every requested action
 * and state.
 */
#include "obstacle_trace.h"
#include <inttypes.h>
#include <stdio.h>

/** @brief Print the requested action and the state that produced it. */
void obstacle_trace_action (const avoid_obstacle_t *        p_avoidance,
                            const avoid_obstacle_input_t *  p_input,
                            const avoid_obstacle_action_t * p_action)
{
    printf("O %" PRIu32
           " %u %d %u %u %u %u %u %u %u %u %u %u %u %d %d %u %u %u\n",
           p_input->now_us, p_action->kind, p_action->value,
           p_action->speed_mms, p_avoidance->state, p_avoidance->fault,
           p_avoidance->leg, p_avoidance->b_has_issued, p_avoidance->pass,
           p_avoidance->b_has_swapped, p_avoidance->b_has_backed,
           p_avoidance->b_is_backing, p_avoidance->b_has_aimed,
           p_avoidance->b_is_aiming, avoid_obstacle_relative_get(p_avoidance),
           avoid_obstacle_aim_get(p_avoidance), p_avoidance->searched_mm,
           p_avoidance->lat_mm, p_avoidance->peek_lat_mm);
}

/* End of file. */

