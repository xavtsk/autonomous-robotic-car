/* Pure geometric regression: sweep the actual rectangular body through the
 * planned path, including every turn. This is not a sonar or wheel-slip model. */
#include "avoidance_planner.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

typedef struct { double x,y; } point_t;
typedef struct {
    s5_config_t cfg; s5_plan_t plan;
    double x,y,heading,front,back,left,right;
    unsigned samples;
} world_t;
static double maximum(double a,double b) { return a>b?a:b; }
static void check(world_t *w)
{
    double radians=w->heading*0.0174532925199433;
    double sx=sin(radians),cy=cos(radians);
    double front=maximum(w->cfg.front_axle_mm,maximum(w->cfg.sensor_axle_mm,w->cfg.sonar_axle_mm));
    /* Independently expand the obstacle in WORLD axes. The 1 mm allowance
     * keeps exact margin-boundary contact distinct from a physical collision;
     * it is smaller than the planner's distance-rounding guard. */
    double margin=w->cfg.margin_mm>1U?w->cfg.margin_mm-1.0:0.0;
    point_t body[4],box[4]={{w->left+margin,w->front-margin},
        {w->left+margin,w->back+margin},{w->right-margin,w->back+margin},
        {w->right-margin,w->front-margin}};
    point_t axes[4]={{1,0},{0,1},{sx,cy},{cy,-sx}};
    bool separate=false;
    for(unsigned i=0;i<4;i++) {
        double along=i<2?front:-(double)w->cfg.rear_axle_mm;
        double across=(i==0 || i==3)?w->cfg.half_width_mm:-(double)w->cfg.half_width_mm;
        body[i]=(point_t){w->x+along*sx+across*cy,w->y+along*cy-across*sx};
        assert(body[i].x<=w->cfg.left_space_mm-margin+0.001);
        assert(body[i].x>=-(double)w->cfg.right_space_mm+margin-0.001);
    }
    for(unsigned a=0;a<4;a++) {
        double bmin=1e9,bmax=-1e9,omin=1e9,omax=-1e9;
        for(unsigned i=0;i<4;i++) {
            double b=body[i].x*axes[a].x+body[i].y*axes[a].y;
            double o=box[i].x*axes[a].x+box[i].y*axes[a].y;
            if(b<bmin) { bmin=b; } if(b>bmax) { bmax=b; }
            if(o<omin) { omin=o; } if(o>omax) { omax=o; }
        }
        if(bmax<omin || omax<bmin) { separate=true; }
    }
    assert(separate);w->samples++;
}
static void turn(world_t *w,int degrees)
{
    int sign=degrees>0?1:-1;
    for(int i=0;i<degrees*sign;i++) { w->heading+=sign;check(w); }
}
static void drive(world_t *w,double distance)
{
    unsigned steps=(unsigned)ceil(distance);
    if(!steps) { return; }
    double radians=w->heading*0.0174532925199433;
    for(unsigned i=0;i<steps;i++) {
        w->x+=distance/steps*sin(radians);w->y+=distance/steps*cos(radians);check(w);
    }
}
static s5_profile_t profile(unsigned range,unsigned width,int shift)
{
    return (s5_profile_t){.valid=true,.closest_mm=(uint16_t)range,
        .front_mm=(uint16_t)range,.forward_mm=(uint16_t)range,
        .left_edge_mm=(int16_t)((int)width/2+shift),
        .right_edge_mm=(int16_t)(-(int)width/2+shift),
        .width_mm=(uint16_t)width,.left_clear_mm=2000,.right_clear_mm=2000};
}
static bool sweep(s5_config_t cfg,s5_profile_t p,unsigned depth,unsigned side,unsigned *samples)
{
    world_t w={.cfg=cfg};
    /* Select either side without pretending its opposite floor is absent. */
    if(side) { p.left_clear_mm=30; } else { p.right_clear_mm=30; }
    if(!s5_plan(&p,&w.cfg,&w.plan)) { return false; }
    assert(w.plan.route==(side?S5_ROUTE_RIGHT:S5_ROUTE_LEFT));
    w.left=p.left_edge_mm;w.right=p.right_edge_mm;
    w.front=p.front_mm+w.cfg.sonar_axle_mm;w.back=w.front+depth;
    int sign=side?-1:1;check(&w);
    turn(&w,sign*45);drive(&w,w.plan.diagonal_mm);turn(&w,-sign*45);
    drive(&w,w.plan.forward_mm);
    /* Worst rear edge consistent with the first confirmed clear ray. This
     * deliberately makes the inward turn occur at its minimum planned
     * clearance; a deeper obstacle cannot hide behind an extra-long approach. */
    double first_clear=maximum(w.y+w.plan.clear_after_mm,
        w.back-w.cfg.sonar_axle_mm-w.plan.peek_span_mm+1.0);
    drive(&w,first_clear-w.y);
    drive(&w,w.plan.peek_span_mm+w.cfg.sonar_axle_mm+w.plan.rear_clear_mm+1U);
    turn(&w,-sign*45);drive(&w,w.plan.diagonal_mm);turn(&w,sign*45);
    assert(fabs(w.x)<0.001 && fabs(w.heading)<0.001 && w.y>w.back);
    *samples+=w.samples;
    return true;
}
int main(void)
{
    unsigned cases=0,samples=0,rejected=0,at_distance[3]={0};
    for(unsigned range=200;range<=300;range+=50) {
        for(unsigned width=40;width<=360;width+=40) {
            for(int shift=-40;shift<=40;shift+=40) {
                if(shift>=((int)width/2) || shift<=-((int)width/2)) { continue; }
                for(unsigned depth=50;depth<=450;depth+=100) {
                    for(unsigned side=0;side<2;side++) {
                        if(sweep(s5_defaults(),profile(range,width,shift),depth,side,&samples)) {
                            cases++;at_distance[(range-200)/50]++;
                        } else { rejected++; }
                    }
                }
            }
        }
    }
    assert(cases>=100 && rejected>0 && samples>100000);
    for(unsigned i=0;i<3;i++) { assert(at_distance[i]>0); }

    s5_config_t c=s5_defaults();s5_plan_t plan;
    s5_profile_t p=profile(200,200,0);
    assert(s5_plan(&p,&c,&plan)); /* Compact geometry fits where old circle did not. */
    assert(plan.rear_clear_mm<plan.radius_mm && plan.rear_clear_mm<=40);
    assert(plan.offset_mm+plan.radius_mm>p.front_mm+c.sonar_axle_mm);
    assert(plan.diagonal_mm>plan.offset_mm);
    p=profile(200,320,0);assert(!s5_plan(&p,&c,&plan)); /* Too close to near corner. */
    p=profile(100,40,0);assert(!s5_plan(&p,&c,&plan)); /* Initial turn sweeps forward. */
    p=profile(300,360,0);assert(!s5_plan(&p,&c,&plan)); /* Floor, not just front space. */

    p=profile(300,100,0);c.left_space_mm=c.right_space_mm=0;
    assert(!s5_plan(&p,&c,&plan)); /* Unknown floor must remain unknown. */
    c=s5_defaults();c.left_space_mm=c.right_space_mm=200;
    assert(!s5_plan(&p,&c,&plan));
    c=s5_defaults();p.left_clear_mm=p.right_clear_mm=100;
    assert(!s5_plan(&p,&c,&plan)); /* Both measured diagonal corridors blocked. */

    /* Asymmetric placement must work when each feasible side is selected. */
    c=s5_defaults();assert(sweep(c,profile(250,160,30),450,0,&samples));
    assert(sweep(c,profile(250,160,-30),450,1,&samples));
    /* A long rear needs an additional bound at the final heading-restoring
     * turn, even when the initial inward turn's corner is already clear. */
    c.rear_axle_mm=200;c.half_width_mm=20;c.margin_mm=10;
    c.left_space_mm=c.right_space_mm=900;
    assert(sweep(c,profile(300,80,0),700,0,&samples));
    assert(sweep(c,profile(300,80,0),700,1,&samples));
    printf("PASS: %u compact body-sweep cases (%u/%u/%u at 200/250/300 mm), %u poses, %u rejected geometries\n",
        cases,at_distance[0],at_distance[1],at_distance[2],samples,rejected);
}
