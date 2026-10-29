#include "avoidance_planner.h"
bool s5_plan(const s5_profile_t *p,const s5_config_t *c,s5_plan_t *out)
{
    unsigned left,right,forward,pad; bool l,r;
    if(!out) { return false; } *out=(s5_plan_t){0};
    if(!p || !c || !p->valid || !c->half_width_mm || !c->margin_mm || !c->max_depth_mm) { return false; }
    pad=c->half_width_mm+c->margin_mm;
    left=(p->left_edge_mm>0?(unsigned)p->left_edge_mm:0U)+pad;
    right=(p->right_edge_mm<0?(unsigned)-p->right_edge_mm:0U)+pad;
    /* Approach to the visible front face only; the unknown depth is measured
     * afterwards by side peeks, bounded by max_depth_mm (obstacle_avoidance.c). */
    forward=p->forward_mm;
    /* Diagonal 30/150 deg readings only choose the side (they also catch the
     * car's own wheels / track edges). The real check is leg 1: after turning,
     * the sensor faces the sideways path and must read past offset + margin. */
    l=left<=c->max_offset_mm;
    r=right<=c->max_offset_mm;
    if((!l&&!r) || forward==0U || forward>3000U) { return false; }
    if(S5_PREFER_ROUTE==1U) { out->route=l?S5_ROUTE_LEFT:S5_ROUTE_RIGHT; }
    else if(S5_PREFER_ROUTE==2U) { out->route=r?S5_ROUTE_RIGHT:S5_ROUTE_LEFT; }
    else { out->route=(l&&(!r || p->left_clear_mm>=p->right_clear_mm))?S5_ROUTE_LEFT:S5_ROUTE_RIGHT; }
    out->offset_mm=out->route==S5_ROUTE_LEFT?left:right;
    out->alt_ok=out->route==S5_ROUTE_LEFT?r:l;
    out->alt_offset_mm=out->route==S5_ROUTE_LEFT?right:left;
    out->forward_mm=forward; return true;
}
