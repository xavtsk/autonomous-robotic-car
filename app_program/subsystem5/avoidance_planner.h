#ifndef BUDDY5_AVOIDANCE_PLANNER_H
#define BUDDY5_AVOIDANCE_PLANNER_H
#include "obstacle_profile.h"
typedef enum { S5_ROUTE_NONE, S5_ROUTE_LEFT, S5_ROUTE_RIGHT } s5_route_t;
typedef struct {
    unsigned route:2, offset_mm:12, forward_mm:12;
    unsigned alt_offset_mm:12, alt_ok:1; /* the other side, tried if blocked */
} s5_plan_t;
/* Rectangular route. forward_mm reaches the visible front face; the hidden
 * rear edge is found by side peeks while passing, searching at most
 * max_depth_mm before giving up (S5_NO_PATH). */
bool s5_plan(const s5_profile_t *profile,const s5_config_t *cfg,s5_plan_t *out);
#endif
