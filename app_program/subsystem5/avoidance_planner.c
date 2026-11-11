#include "avoidance_planner.h"
/* Exact integer ceiling avoids rounding a clearance distance down. */
static unsigned ceil_root(uint32_t n)
{
    unsigned lo=0U,hi=8192U;
    while(lo<hi) {
        unsigned mid=lo+(hi-lo)/2U;
        if((uint32_t)mid*mid<n) { lo=mid+1U; } else { hi=mid; }
    }
    return lo;
}
/* ceil(max(a*cos(theta)+b*sin(theta))), 0 <= theta <= 45 degrees.
 * An interior maximum exists at atan(b/a) only when b <= a. Otherwise
 * the maximum is the 45-degree endpoint. No sampled-angle approximation. */
static unsigned sweep_45(unsigned a,unsigned b)
{
    if(b<=a) { return ceil_root(a*a+b*b); }
    unsigned sum=a+b;
    return ceil_root((sum*sum+1U)/2U);
}
typedef struct {
    unsigned front,inner,outer,front_turn,rear_turn,diagonal_width;
} body_t;
typedef struct {
    unsigned offset,diagonal,forward,peek_span,clear_after,reserve;
    bool feasible;
} candidate_t;
static candidate_t candidate(const s5_profile_t *p,const s5_config_t *c,
                             const body_t *body,bool left)
{
    candidate_t v={0};
    int edge=left?p->left_edge_mm:-p->right_edge_mm;
    int opposite=left?-p->right_edge_mm:p->left_edge_mm;
    unsigned space=left?c->left_space_mm:c->right_space_mm;
    unsigned other_space=left?c->right_space_mm:c->left_space_mm;
    unsigned clear=left?p->left_clear_mm:p->right_clear_mm;
    unsigned front=(unsigned)p->front_mm+c->sonar_axle_mm;
    unsigned floor_need,corridor_need,corner_need,turn_need,front_need;
    unsigned front_reserve,floor_reserve,range_reserve;
    /* At the outbound endpoint the rear corner swings toward the obstacle
     * while the heading returns from 45 degrees to parallel. Reserve that
     * exact lateral extent, not the full front/rear bounding circle. */
    v.offset=(edge>0?(unsigned)edge:0U)+body->inner+c->margin_mm;
    if(!space || !other_space || v.offset>c->max_offset_mm || v.offset>4095U) { return v; }
    v.diagonal=ceil_root(2U*v.offset*v.offset);
    /* Begin side observations immediately after turning parallel. Driving
     * blindly to the old far-front estimate can overshoot a short obstacle. */
    v.forward=0U;
    /* An immediate diagonal peek may pass entirely BEFORE a short obstacle,
     * so a far echo cannot yet establish its rear edge. After the outbound
     * diagonal advances the axle by offset, require the sonar to reach the
     * observed far-front plane: offset + parallel + sonar_axle must exceed
     * forward_mm + sonar_axle. The sonar offsets cancel. One millimetre keeps
     * an integer boundary on the passed side; runtime adds any measured
     * outbound shortfall and verifies actual parallel wheel travel. */
    v.clear_after=(unsigned)p->forward_mm+1U>v.offset?
        (unsigned)p->forward_mm+1U-v.offset:0U;
    v.peek_span=v.offset+(opposite>0?(unsigned)opposite:0U)+c->margin_mm;
    if(v.diagonal>4095U || v.forward>4095U || v.peek_span>4095U ||
       v.clear_after>4095U || v.clear_after>c->max_depth_mm) { return v; }
    /* For a left diagonal x=y, the body's transverse separating axis is
     * (x-y)/sqrt(2), independent of progress and front/rear overhang. The
     * obstacle inflated by margin lies wholly below it when
     * front - left_edge >= sqrt(2)*half_width + 2*margin. Mirror for RIGHT.
     * This checks the entire translation, not only its endpoint. */
    corner_need=(edge>0?(unsigned)edge:0U)+body->diagonal_width+2U*c->margin_mm;
    turn_need=body->front_turn+c->margin_mm;
    front_need=corner_need>turn_need?corner_need:turn_need;
    if(front<front_need) { return v; }
    /* Every heading lies in [-45,+45]. The final return points the nose onto
     * the opposite side of the line too, so both floor budgets are needed.
     * Rounding diagonal travel up adds less than 1 mm to either projection. */
    floor_need=v.offset+body->outer+c->margin_mm+1U;
    if(floor_need>space || other_space<body->outer+c->margin_mm+1U) { return v; }
    /* The sonar rotates with the car. A fresh forward check after the turn
     * repeats this preflight along the actual diagonal before driving. */
    corridor_need=v.diagonal+(body->front+c->margin_mm>c->sonar_axle_mm?
        body->front+c->margin_mm-c->sonar_axle_mm:0U);
    if(clear<=corridor_need) { return v; }
    front_reserve=front-front_need;
    floor_reserve=space-floor_need;
    range_reserve=clear-corridor_need;
    v.reserve=front_reserve<floor_reserve?front_reserve:floor_reserve;
    if(range_reserve<v.reserve) { v.reserve=range_reserve; }
    v.feasible=true;
    return v;
}
bool s5_plan(const s5_profile_t *p,const s5_config_t *c,s5_plan_t *out)
{
    unsigned length,radius,rear_clear; candidate_t left,right,chosen,other; bool use_left;
    body_t body={0};
    if(!out) { return false; } *out=(s5_plan_t){0};
    if(!p || !c || !p->valid || !c->half_width_mm || !c->front_axle_mm ||
       !c->rear_axle_mm || !c->margin_mm || !c->max_depth_mm || !p->front_mm ||
       p->front_mm>p->forward_mm || p->forward_mm>S5_RANGE_MAX_MM ||
       p->left_edge_mm<p->right_edge_mm) { return false; }
    length=c->front_axle_mm>c->rear_axle_mm?c->front_axle_mm:c->rear_axle_mm;
    if(c->sensor_axle_mm>length) { length=c->sensor_axle_mm; }
    if(c->sonar_axle_mm>length) { length=c->sonar_axle_mm; }
    radius=ceil_root(length*length+c->half_width_mm*c->half_width_mm)+c->margin_mm;
    if(radius>4095U) { return false; }
    body.front=c->front_axle_mm;
    if(c->sensor_axle_mm>body.front) { body.front=c->sensor_axle_mm; }
    if(c->sonar_axle_mm>body.front) { body.front=c->sonar_axle_mm; }
    body.inner=sweep_45(c->half_width_mm,c->rear_axle_mm);
    body.outer=sweep_45(c->half_width_mm,body.front);
    if(body.inner>body.outer) { body.outer=body.inner; }
    body.front_turn=sweep_45(body.front,c->half_width_mm);
    body.rear_turn=sweep_45(c->rear_axle_mm,c->half_width_mm);
    body.diagonal_width=ceil_root(2U*c->half_width_mm*c->half_width_mm);
    /* The inward turn begins at lateral gap g >= inner >= half_width from
     * the margin-inflated obstacle side. The lowest body point that overlaps
     * that side is (g*cos(theta)-half_width)/sin(theta), decreasing to
     * g-sqrt(2)*half_width at 45 degrees. The return translation has the same
     * separating axis. Thus only this corner clearance is needed past the
     * rear, not the full rear or front radius. The second bound covers the
     * final heading-restoration turn even for unusually long rear overhang. */
    rear_clear=c->margin_mm+(body.diagonal_width>body.inner?
        body.diagonal_width-body.inner:0U);
    if(body.rear_turn>body.inner && body.rear_turn-body.inner>rear_clear) {
        rear_clear=body.rear_turn-body.inner;
    }
    if(body.outer+c->margin_mm>4095U || rear_clear>4095U) { return false; }
    left=candidate(p,c,&body,true); right=candidate(p,c,&body,false);
    if(!left.feasible && !right.feasible) { return false; }
    use_left=left.feasible && (!right.feasible || left.reserve>right.reserve ||
             (left.reserve==right.reserve && left.offset<=right.offset));
    chosen=use_left?left:right; other=use_left?right:left;
    out->route=use_left?S5_ROUTE_LEFT:S5_ROUTE_RIGHT;
    out->offset_mm=chosen.offset; out->diagonal_mm=chosen.diagonal;
    out->forward_mm=chosen.forward; out->peek_span_mm=chosen.peek_span;
    out->clear_after_mm=chosen.clear_after;
    out->radius_mm=radius; out->alt_ok=other.feasible;
    out->outer_mm=body.outer+c->margin_mm;out->rear_clear_mm=rear_clear;
    if(other.feasible) {
        out->alt_offset_mm=other.offset; out->alt_diagonal_mm=other.diagonal;
        out->alt_forward_mm=other.forward; out->alt_peek_span_mm=other.peek_span;
        out->alt_clear_after_mm=other.clear_after;
    }
    return true;
}
