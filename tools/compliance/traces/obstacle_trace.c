/** @file obstacle_trace.c
 * @brief Existing avoidance scenarios/assertions, with every requested action
 * and state.
 */
/* Existing avoidance scenarios/assertions, with every requested action and
 * state. */
#include "obstacle_trace.h"
#include "avoid_obstacle.h"
#include <inttypes.h>
#include <stdio.h>
static s5_action_t trace_obstacle (s5_avoidance_t *, const s5_input_t *);
#define s5_avoidance_step trace_obstacle
#include "host_test.c"
#undef s5_avoidance_step

static s5_action_t trace_obstacle (s5_avoidance_t * avoidance,
                                   const s5_input_t * input)
{
    s5_action_t result = s5_avoidance_step(avoidance, input);
    printf("O %" PRIu32
           " %u %d %u %u %u %u %u %u %u %u %u %u %u %d %d %u %u %u\n",
           input->now_us, result.kind, result.value, result.speed_mms,
           avoidance->state, avoidance->fault, avoidance->leg,
           avoidance->issued, avoidance->pass, avoidance->swapped,
           avoidance->backed, avoidance->backing, avoidance->aimed,
           avoidance->aiming, s5_rel_get(avoidance), s5_aim_get(avoidance),
           avoidance->searched_mm, avoidance->lat_mm, avoidance->peek_lat_mm);
    return result;
}

/* End of file. */

