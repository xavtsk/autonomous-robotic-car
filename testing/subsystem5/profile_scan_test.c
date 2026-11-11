#include "obstacle_scan.h"
#include <assert.h>
#include <stdio.h>

static void three(s5_scan_t *s,uint32_t *clock,unsigned a,unsigned b,unsigned c)
{
    unsigned before=s->count,angle=s->angle;
    *clock+=70000U;s5_scan_accept(s,*clock,true,a);
    assert(!s->done && s->count==before && s->angle==angle);
    *clock+=70000U;s5_scan_accept(s,*clock,true,b);
    assert(!s->done && s->count==before && s->angle==angle);
    *clock+=70000U;s5_scan_accept(s,*clock,true,c);
}
static void median_peek(void)
{
    s5_scan_t s;uint32_t clock=0;unsigned angle,mm;bool valid;
    assert(s5_scan_begin_at(&s,clock,30));
    three(&s,&clock,1000,210,200);
    assert(s.done && !s.failed && s.count==1);
    assert(s5_sample_unpack(&s.sample[0],&angle,&mm,&valid) && angle==30 && mm==210 && valid);
    assert(s5_scan_begin_at(&s,clock,150));
    s5_scan_accept(&s,clock+=70000U,true,300);
    s5_scan_accept(&s,clock+=70000U,false,0);
    assert(!s.done && s.reading_count==1);
    s5_scan_accept(&s,clock+=70000U,true,320);
    s5_scan_accept(&s,clock+=70000U,true,310);
    assert(s.done && !s.failed && s.count==1);
    assert(s5_sample_unpack(&s.sample[0],&angle,&mm,&valid) && mm==310);
    assert(s5_scan_begin_at(&s,clock,30));
    s5_scan_accept(&s,clock+=70000U,true,300);
    for(unsigned i=0;i<3;i++) { s5_scan_accept(&s,clock+=70000U,false,0); }
    assert(s.done && s.failed && s.count==0);
    assert(s5_scan_begin_at(&s,clock,30));
    s5_scan_accept(&s,clock+S5_SCAN_TIMEOUT_US,true,200);
    assert(s.done && s.failed && s.count==0);
}
static void edge_sweep(void)
{
    s5_config_t cfg=s5_defaults();s5_scan_t s;uint32_t clock=0,seen=0;
    s5_scan_begin_profile(&s,clock,&cfg);
    while(!s.done) {
        unsigned angle=s.angle;
        assert(!(seen&(1U<<((angle-30U)/5U))));
        seen|=1U<<((angle-30U)/5U);
        /* One outlier per angle must not become the profile's width. */
        unsigned mm=angle>=75U && angle<=105U?300U:2000U;
        three(&s,&clock,mm,mm,mm==300U?900U:250U);
        assert(clock<S5_SCAN_TIMEOUT_US);
    }
    assert(!s.failed && s.count==17 && s.fine && s.fine_count==10);
    /* Both boundary intervals are refined, including points beyond +/-15. */
    assert((seen&(1U<<7)) && (seen&(1U<<17))); /* 65 and 115 degrees */
    for(unsigned i=0;i<s.count;i++) {
        unsigned angle,mm;bool valid;
        assert(s5_sample_unpack(&s.sample[i],&angle,&mm,&valid));
        assert(mm==(angle>=75U && angle<=105U?300U:2000U));
    }
    s5_profile_t p;assert(s5_profile_build(s.sample,s.count,&cfg,&p));
    assert(p.valid && p.width_mm>=300 && p.width_mm<=304);
    /* Unknown/open limits must never be invented as obstacle edges. */
    clock=0;s5_scan_begin_profile(&s,clock,&cfg);
    while(!s.done) { three(&s,&clock,400,400,400); }
    assert(s.failed && s.count==7 && !s.fine);
}
static void coherent_profile(void)
{
    s5_config_t c=s5_defaults();s5_profile_t p;s5_sample_t samples[7];
    unsigned angles[7]={30,45,60,90,120,135,150};
    unsigned ranges[7]={2000,2000,600,300,600,2000,2000};
    for(unsigned i=0;i<7;i++) { assert(s5_sample_pack(angles[i],ranges[i],true,&samples[i])); }
    assert(s5_profile_build(samples,7,&c,&p));
    /* 600 mm surroundings do not inflate the 300 mm target, but remain
     * possible blockers rather than being erased from route clearance. */
    assert(p.width_mm>=156 && p.width_mm<=160 && p.left_clear_mm==600 && p.right_clear_mm==600);
    assert(s5_profile_front_class(60,600,300,&c)==S5_FRONT_FAR);
    assert(s5_profile_front_class(60,100,300,&c)==S5_FRONT_BLOCKED);
    assert(s5_profile_front_class(45,520,300,&c)==S5_FRONT_BLOCKED);
    assert(s5_sample_pack(45,300,true,&samples[1]));
    assert(s5_profile_build(samples,7,&c,&p) && p.width_mm<=160 && p.right_clear_mm==300);
    assert(s5_sample_pack(120,100,true,&samples[4]));
    assert(!s5_profile_build(samples,7,&c,&p)); /* nearer foreign echo is not a clear bracket */
    assert(s5_sample_pack(120,0,false,&samples[4]));
    assert(!s5_profile_build(samples,7,&c,&p));
}
int main(void)
{
    median_peek();edge_sweep();coherent_profile();
    puts("PASS: per-angle medians, two edge refinements, distinct angles, coherent front profile, unknown edges and bounded failures");
}
