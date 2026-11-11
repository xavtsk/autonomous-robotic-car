#ifndef BUDDY5_AVOIDANCE_PLANNER_H
#define BUDDY5_AVOIDANCE_PLANNER_H
#include "obstacle_profile.h"
typedef enum { S5_ROUTE_NONE, S5_ROUTE_LEFT, S5_ROUTE_RIGHT } s5_route_t;
typedef struct {
    unsigned route:2, offset_mm:12, forward_mm:12;
    unsigned alt_offset_mm:12, alt_ok:1; /* the other side, tried if blocked */
    unsigned diagonal_mm:12, alt_diagonal_mm:12, alt_forward_mm:12;
    unsigned radius_mm:12, peek_span_mm:12, alt_peek_span_mm:12;
    unsigned outer_mm:12, rear_clear_mm:12;
    unsigned clear_after_mm:12, alt_clear_after_mm:12;
} s5_plan_t;
/* 45-degree out/parallel/45-degree return route. offset_mm is lateral;
 * diagonal_mm is wheel travel. forward_mm is zero: side observations begin
 * immediately after the outbound diagonal. A clear side echo is not evidence
 * of passing the obstacle until actual parallel travel reaches clear_after_mm:
 * the sonar must first reach the observed far-front plane. The alternate
 * route carries its own gate. radius_mm is diagnostic only;
 * outer_mm bounds the lateral footprint during 0..45-degree turns, including
 * margin. rear_clear_mm is the proven clearance behind the hidden rear edge
 * needed by the inward turn, diagonal return and final heading restoration.
 * peek_span_mm reaches the far obstacle edge plus margin. The hidden rear
 * edge still requires side observations, bounded by max_depth_mm. */
bool s5_plan(const s5_profile_t *profile,const s5_config_t *cfg,s5_plan_t *out);
#endif
