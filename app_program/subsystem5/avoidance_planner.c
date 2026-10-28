#include "avoidance_planner.h"
bool s5_plan(const s5_profile_t *p,const s5_config_t *c,s5_plan_t *out)
{
    unsigned left,right,forward,pad; bool l,r;
    if(!out) { return false; } *out=(s5_plan_t){0};
    if(!p || !c || !p->valid || !c->half_width_mm || !c->margin_mm || !c->max_depth_mm) { return false; }
    pad=c->half_width_mm+c->margin_mm;
    left=(p->left_edge_mm>0?(unsigned)p->left_edge_mm:0U)+pad;
    right=(p->right_edge_mm<0?(unsigned)-p->right_edge_mm:0U)+pad;
    forward=p->forward_mm+c->max_depth_mm+pad;
    l=left<=c->max_offset_mm && p->left_clear_mm>left+pad;
    r=right<=c->max_offset_mm && p->right_clear_mm>right+pad;
    if((!l&&!r) || forward>3000U) { return false; }
    out->route=(l&&(!r || p->left_clear_mm>=p->right_clear_mm))?S5_ROUTE_LEFT:S5_ROUTE_RIGHT;
    out->offset_mm=out->route==S5_ROUTE_LEFT?left:right;
    out->forward_mm=forward; return true;
}
