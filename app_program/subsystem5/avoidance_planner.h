#ifndef BUDDY5_AVOIDANCE_PLANNER_H
#define BUDDY5_AVOIDANCE_PLANNER_H
#include "obstacle_profile.h"
typedef enum { S5_ROUTE_NONE, S5_ROUTE_LEFT, S5_ROUTE_RIGHT } s5_route_t;
typedef struct {
    unsigned route:2, offset_mm:12, forward_mm:12;
} s5_plan_t;
/* Conservative rectangular route. max_depth_mm is a REQUIRED validated
 * operating-envelope bound: a front scan cannot measure a hidden rear edge. */
bool s5_plan(const s5_profile_t *profile,const s5_config_t *cfg,s5_plan_t *out);
#endif
