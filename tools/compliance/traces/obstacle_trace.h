/** @file obstacle_trace.h
 * @brief Record the obstacle state and action after one avoidance update.
 */
#ifndef TOOLS_COMPLIANCE_TRACES_OBSTACLE_TRACE_H
#define TOOLS_COMPLIANCE_TRACES_OBSTACLE_TRACE_H

#include "avoid_obstacle.h"

void obstacle_trace_action (const avoid_obstacle_t *        p_avoidance,
                            const avoid_obstacle_input_t *  p_input,
                            const avoid_obstacle_action_t * p_action);

#endif /* TOOLS_COMPLIANCE_TRACES_OBSTACLE_TRACE_H */

/* End of file. */

