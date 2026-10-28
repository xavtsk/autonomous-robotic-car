#include "obstacle_avoidance.h"
#include "obstacle_scan.h"
#include "ultrasonic_sensor.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static s5_profile_t profile(void)
{
    s5_sample_t samples[5]; s5_config_t cfg=s5_defaults(); s5_profile_t p;
    unsigned range[5]={1800,1500,250,1600,1900};
    for(unsigned i=0;i<5;i++) { assert(s5_sample_pack(30U+30U*i,range[i],true,&samples[i])); }
    assert(s5_profile_build(samples,5,&cfg,&p));
    assert(p.valid && p.closest_mm==250 && p.width_mm==538 && p.left_clear_mm==1600 && p.right_clear_mm==1500);
    return p;
}
static void codecs(void)
{
    for(unsigned a=30;a<=150;a+=5) {
        for(unsigned d=30;d<=4000;d++) {
            s5_sample_t s;unsigned angle,mm;bool valid;
            assert(s5_sample_pack(a,d,true,&s));assert(s5_sample_unpack(&s,&angle,&mm,&valid));
            assert(angle==a && mm==d && valid);
        }
    }
    s5_sample_t bad={{0,0,0x80}};unsigned a,d;bool v;assert(!s5_sample_unpack(&bad,&a,&d,&v));
    s5_observation_t in={.range_us=0xfffffff0U,.scan_us=123,.scan_id=65535,.range_mm=4000,.range_valid=true,.scan_done=true,.profile=profile()},out={0};
    uint8_t bytes[S5_PACKET_BYTES],again[S5_PACKET_BYTES];
    assert(s5_observation_pack(&in,bytes));assert(s5_observation_unpack(bytes,&out));
    assert(out.range_us==in.range_us && out.range_mm==4000 && out.scan_id==65535 && out.profile.left_clear_mm==1600);
    assert(s5_observation_pack(&out,again) && !memcmp(bytes,again,sizeof(bytes)));
    bytes[22]|=0x80;assert(!s5_observation_unpack(bytes,&out));
    in.no_echo=true;assert(!s5_observation_pack(&in,bytes));in.no_echo=false;
    in.range_mm=4096;assert(!s5_observation_pack(&in,bytes));
    unsigned seed=42;
    for(unsigned n=0;n<20000;n++) {
        for(unsigned i=0;i<sizeof(bytes);i++) { seed=seed*1664525U+1013904223U;bytes[i]=(uint8_t)(seed>>24); }
        if(s5_observation_unpack(bytes,&out)) { assert(s5_observation_pack(&out,again) && !memcmp(bytes,again,sizeof(bytes))); }
    }
    uint16_t mm;assert(ultrasonic_pulse_mm(2000,&mm) && mm==343);
    assert(!ultrasonic_pulse_mm(0,&mm));assert(!ultrasonic_pulse_mm(25000,&mm));
    in=(s5_observation_t){.range_valid=true,.range_mm=1000,.range_us=0xfffffff0U};
    assert(s5_range_ready(&in,100));assert(!s5_range_ready(&in,300000));
}
static void scans(void)
{
    s5_scan_t s;uint32_t now=0xfffffff0U;s5_scan_begin(&s,now);
    while(!s.done) { now+=300000U;s5_scan_accept(&s,now,true,s.angle==90?250:1800); }
    assert(!s.failed && s.count==12 && s.fine);
    s5_config_t cfg=s5_defaults();s5_profile_t p;assert(s5_profile_build(s.sample,s.count,&cfg,&p));
    s5_plan_t plan;assert(s5_plan(&p,&cfg,&plan) && plan.route==S5_ROUTE_LEFT);
    p.left_clear_mm=10;p.right_clear_mm=10;assert(!s5_plan(&p,&cfg,&plan));
    s5_scan_begin(&s,0);for(unsigned i=1;i<=3;i++) { s5_scan_accept(&s,i*70000,false,0); }
    assert(s.failed && s.done && s.count==0);
    s5_scan_begin(&s,0);s5_scan_accept(&s,S5_SCAN_TIMEOUT_US,true,200);assert(s.failed);
    s5_sample_t invalid[5];for(unsigned i=0;i<5;i++) { assert(s5_sample_pack(30+30*i,0,false,&invalid[i])); }
    assert(!s5_profile_build(invalid,5,&cfg,&p));
}
static void fsm(bool see_line,bool block,bool bad_range)
{
    s5_config_t cfg=s5_defaults();s5_avoidance_t f;s5_avoidance_init(&f,&cfg);
    s5_input_t in={.healthy=true,.stopped=false,.line_ready=true};
    s5_avoidance_begin(&f,0);
    assert(s5_avoidance_step(&f,&in).kind==S5_ACT_NONE && f.state==S5_WAIT_STOP);
    in.now_us=10000;in.stopped=true;assert(s5_avoidance_step(&f,&in).kind==S5_ACT_SCAN);
    in.now_us=20000;in.observation.scan_done=true;in.observation.scan_us=20000;in.observation.profile=profile();
    if(block) { in.observation.profile.left_clear_mm=in.observation.profile.right_clear_mm=10; }
    s5_action_t a=s5_avoidance_step(&f,&in);
    if(block) { assert(a.kind==S5_ACT_STOP && f.fault==S5_NO_PATH);return; }
    for(unsigned tick=0;tick<1000 && f.state!=S5_COMPLETE && f.state!=S5_FAULT;tick++) {
        in.now_us+=10000;in.observation.range_us=in.now_us;in.observation.range_mm=2000;in.observation.range_valid=true;
        in.tape=false;in.motion_done=false;
        if(f.issued) {
            if(bad_range && f.leg==1) { in.observation.range_valid=false; }
            if(f.leg==5) {
                in.centre_m=f.leg_start_m+(double)f.plan.offset_mm/1000.0;
                in.tape=see_line;
                /* Hold real tape for 30 ms before completing return. */
                if(see_line && (!f.recovery.touching || (uint32_t)(in.now_us-f.recovery.since_us)<40000U)) {
                    a=s5_avoidance_step(&f,&in);continue;
                }
            }
            in.motion_done=true;
        }
        a=s5_avoidance_step(&f,&in);
    }
    if(bad_range) { assert(f.state==S5_FAULT && f.fault==S5_BAD_RANGE); }
    else if(see_line) { assert(f.state==S5_COMPLETE && a.kind==S5_ACT_RESUME); }
    else { assert(f.state==S5_FAULT && f.fault==S5_NO_LINE); }
    s5_avoidance_abort(&f);assert(f.state==S5_IDLE);
}
int main(void)
{
    codecs();scans();fsm(true,false,false);fsm(true,true,false);fsm(true,false,true);fsm(false,false,false);
    /* White alone must never confirm the original line, even in corridor. */
    s5_recovery_t rec={0};for(unsigned i=0;i<100;i++) { assert(!s5_recovery_observe(&rec,i*10000,true,false,false,true)); }
    assert(!s5_recovery_observe(&rec,1000000,true,true,false,false));
    s5_config_t cfg=s5_defaults();s5_avoidance_t f;s5_avoidance_init(&f,&cfg);s5_avoidance_begin(&f,0);
    s5_input_t in={.now_us=1000001,.healthy=true};assert(s5_avoidance_step(&f,&in).kind==S5_ACT_STOP && f.fault==S5_DEADLINE);
    puts("PASS: range/codec exhaustive boundaries, hostile packets, coarse/fine scans, invalid echo, route clearance, bounded FSM, faults and line evidence");
}
