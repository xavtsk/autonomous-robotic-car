/** @file avoidance_planner.h
 * @brief Select a bounded bypass plan from the measured obstacle profile.
 */
#ifndef BUDDY5_AVOIDANCE_PLANNER_H
#define BUDDY5_AVOIDANCE_PLANNER_H
#include "profile_obstacle.h"

typedef enum
{
    S5_ROUTE_NONE,
    S5_ROUTE_LEFT,
    S5_ROUTE_RIGHT
} avoidance_planner_route_t;

typedef struct
{
    unsigned route : 2;
    unsigned offset_mm : 12;
    unsigned forward_mm : 12;
    unsigned alt_offset_mm : 12;
    unsigned b_has_alt_route : 1; /* Use this side if the first is blocked. */
    unsigned closest_mm : 12;     /* Measure the face from the sensor. */
} avoidance_planner_plan_t;

/* Route with 45-degree legs (13 Nov 2026; was 90): out diagonally by
 * offset_mm sideways, parallel past the obstacle, back diagonally. forward_mm
 * reaches the visible front face; the hidden rear edge is found by side peeks
 * while passing, searching at most max_depth_mm before giving up (S5_NO_PATH).
 */
bool avoidance_planner_build (const profile_obstacle_t * p_profile,
                              const s5_config_t *        p_cfg,
                              avoidance_planner_plan_t * p_out);
#endif

/* End of file. */

