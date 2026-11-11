#include "obstacle_avoidance.h"
#include "obstacle_scan.h"
#include "ultrasonic_sensor.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/* Explicit simulation dimensions, not physical car measurements. */
static s5_config_t test_config(void)
{
    s5_config_t c=s5_defaults();
    c.half_width_mm=60; c.front_axle_mm=80; c.rear_axle_mm=120;
    c.sonar_axle_mm=50; c.sensor_axle_mm=0; c.margin_mm=20;
    c.profile_mm=800; /* Deliberately farther synthetic profile than production. */
    c.left_space_mm=c.right_space_mm=1500;
    return c;
}
static s5_profile_t profile(void)
{
    s5_sample_t samples[5]; s5_config_t cfg=test_config(); s5_profile_t p;
    unsigned range[5]={1800,1500,600,1600,1900};
    for(unsigned i=0;i<5;i++) { assert(s5_sample_pack(30U+30U*i,range[i],true,&samples[i])); }
    assert(s5_profile_build(samples,5,&cfg,&p));
    /* A centre return bounds a bearing interval rather than a point. */
    assert(p.valid && p.closest_mm==600 && p.width_mm>=310 && p.front_mm<=580 && p.forward_mm>=600 && p.left_clear_mm==1600 && p.right_clear_mm==1500);
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
    bytes[S5_PACKET_BYTES-1U]|=0x80;assert(!s5_observation_unpack(bytes,&out));
    in.no_echo=true;assert(!s5_observation_pack(&in,bytes));in.no_echo=false;
    in.range_mm=4096;assert(!s5_observation_pack(&in,bytes));
    unsigned seed=42;
    for(unsigned n=0;n<20000;n++) {
        for(unsigned i=0;i<sizeof(bytes);i++) { seed=seed*1664525U+1013904223U;bytes[i]=(uint8_t)(seed>>24); }
        if(s5_observation_unpack(bytes,&out)) { assert(s5_observation_pack(&out,again) && !memcmp(bytes,again,sizeof(bytes))); }
    }
    uint16_t mm;assert(ultrasonic_pulse_mm(2000,&mm) && mm==343);
    assert(!ultrasonic_pulse_mm(0,&mm));assert(!ultrasonic_pulse_mm(25000,&mm));
    /* Actual long echoes are capped; missing or too-short pulses stay invalid. */
    assert(ultrasonic_echo_mm(2000,&mm) && mm==343);
    assert(ultrasonic_echo_mm(25000,&mm) && mm==S5_RANGE_MAX_MM);
    assert(ultrasonic_echo_mm(23900,&mm) && mm==S5_RANGE_MAX_MM);
    assert(!ultrasonic_echo_mm(0,&mm) && mm==0);assert(!ultrasonic_echo_mm(100,&mm));
    in=(s5_observation_t){.range_valid=true,.range_mm=1000,.range_us=0xfffffff0U};
    assert(s5_range_ready(&in,100));assert(!s5_range_ready(&in,300000));
    in.range_us=20000;
    assert(s5_range_ready(&in,10000)); /* newer publication than the ADC frame */
    in.range_us=40001;assert(!s5_range_ready(&in,10000));
}
static void scans(void)
{
    s5_config_t cfg=test_config();
    s5_scan_t s;uint32_t now=0xfffffff0U;s5_scan_begin_profile(&s,now,&cfg);
    while(!s.done) { now+=70000U;s5_scan_accept(&s,now,true,s.angle==90?600:1800); }
    assert(!s.failed && s.count>7U && s.count<=17U && s.fine);
    bool fine_left=false,fine_right=false;
    for(unsigned i=0;i<s.count;i++) {
        unsigned angle,range;bool valid;
        assert(s5_sample_unpack(&s.sample[i],&angle,&range,&valid) && valid);
        fine_left|=angle>90U && angle<120U;
        fine_right|=angle>60U && angle<90U;
    }
    assert(fine_left && fine_right); /* Both front-component boundaries refined. */
    s5_profile_t p;assert(s5_profile_build(s.sample,s.count,&cfg,&p));
    s5_plan_t plan;assert(s5_plan(&p,&cfg,&plan) && plan.route==S5_ROUTE_LEFT);
    assert(plan.diagonal_mm>plan.offset_mm && plan.diagonal_mm*707U/1000U>=plan.offset_mm-1U);
    assert(plan.forward_mm==0U); /* Peek immediately after becoming parallel. */
    /* A protruding sensing head belongs to the swept body envelope too. */
    unsigned base_radius=plan.radius_mm;
    cfg.sensor_axle_mm=180;
    assert(s5_plan(&p,&cfg,&plan) && plan.radius_mm>base_radius);
    cfg=test_config();
    /* Unknown floor limits cannot be inferred from an ultrasonic range. */
    s5_config_t unknown=cfg;unknown.front_axle_mm=0;assert(!s5_plan(&p,&unknown,&plan));
    unknown=cfg;unknown.left_space_mm=unknown.right_space_mm=0;assert(!s5_plan(&p,&unknown,&plan));
    cfg.left_space_mm=0;
    assert(!s5_plan(&p,&cfg,&plan)); /* Nose still sweeps over the opposite side on return. */
    cfg.left_space_mm=200;
    assert(s5_plan(&p,&cfg,&plan) && plan.route==S5_ROUTE_RIGHT && !plan.alt_ok);
    cfg=test_config();p.left_clear_mm=100;
    assert(s5_plan(&p,&cfg,&plan) && plan.route==S5_ROUTE_RIGHT && !plan.alt_ok);
    p.right_clear_mm=100;assert(!s5_plan(&p,&cfg,&plan));
    p=profile();cfg.left_space_mm=500;
    assert(s5_plan(&p,&cfg,&plan) && plan.route==S5_ROUTE_RIGHT);
    cfg=test_config();p.front_mm=100;assert(!s5_plan(&p,&cfg,&plan));
    p=profile();p.left_edge_mm=900;p.right_edge_mm=-900;assert(!s5_plan(&p,&cfg,&plan));
    s5_scan_begin(&s,0);for(unsigned i=1;i<=3;i++) { s5_scan_accept(&s,i*70000,false,0); }
    assert(s.failed && s.done && s.count==0);
    s5_scan_begin(&s,0);s5_scan_accept(&s,S5_SCAN_TIMEOUT_US,true,200);assert(s.failed);
    s5_sample_t invalid[5];for(unsigned i=0;i<5;i++) { assert(s5_sample_pack(30+30*i,0,false,&invalid[i])); }
    assert(!s5_profile_build(invalid,5,&cfg,&p));
}
static unsigned peeks; static bool first_side_only;
static void fsm_depth(bool see_line,bool block,bool bad_range,unsigned deep);
static void fsm(bool see_line,bool block,bool bad_range) { fsm_depth(see_line,block,bad_range,0); }
static void fsm_depth(bool see_line,bool block,bool bad_range,unsigned deep)
{
    s5_config_t cfg=test_config();s5_avoidance_t f;s5_avoidance_init(&f,&cfg);
    s5_input_t in={.healthy=true,.stopped=false,.line_ready=true};
    s5_avoidance_begin(&f,0);
    assert(s5_avoidance_step(&f,&in).kind==S5_ACT_NONE && f.state==S5_WAIT_STOP);
    in.now_us=10000;in.stopped=true;assert(s5_avoidance_step(&f,&in).kind==S5_ACT_SCAN);
    in.now_us=20000;in.observation.scan_done=true;in.observation.scan_us=20000;in.observation.profile=profile();
    s5_action_t a=s5_avoidance_step(&f,&in);
    peeks=0; double drive_goal=0.0; int heading=0;
    unsigned route_turns=0,step_moves=0,early_clears=0;
    for(unsigned tick=0;tick<3000 && f.state!=S5_COMPLETE && f.state!=S5_FAULT;tick++) {
        in.now_us+=10000;in.observation.range_us=in.now_us;in.observation.range_valid=true;
        /* block: after turning, the sideways path reads 150 mm (< offset + margin). */
        in.observation.range_mm=(block && f.leg==1 && (!first_side_only || !f.swapped))?150:2000;
        in.tape=false;in.motion_done=false;
        bool early_clear=false;
        if(a.kind==S5_ACT_DRIVE) {
            assert(a.value>0);
            assert(f.leg!=3U || f.pass!=S5_PASS_APPROACH);
            if(f.leg==3U && f.pass==S5_PASS_STEP) { step_moves++; }
            drive_goal=in.centre_m+(double)a.value/1000.0;
        }
        if(a.kind==S5_ACT_TURN) {
            assert(a.value!=90 && a.value!=-90 && a.value!=180 && a.value!=-180);
            if(f.leg<7U) { assert(a.value==45 || a.value==-45); route_turns++; }
            heading+=a.value;
        }
        if(a.kind==S5_ACT_PEEK) {
            /* LEFT bypass looks right (30 deg); report that side's reading. */
            assert(f.leg==3 && f.pass==S5_PASS_PEEKING && a.value==(f.plan.route==S5_ROUTE_LEFT?30:150));
            in.observation.scan_done=true;in.observation.scan_us=in.now_us;
            unsigned side=peeks++<deep?200U:2000U; /* 200 mm: still beside us */
            early_clear=side==2000U &&
                (in.centre_m-f.pass_start_m)*1000.0<f.plan.clear_after_mm;
            if(early_clear) { early_clears++; }
            if(f.plan.route==S5_ROUTE_LEFT) { in.observation.profile.right_clear_mm=(uint16_t)side; }
            else { in.observation.profile.left_clear_mm=(uint16_t)side; }
        }
        if(f.issued) {
            if(f.leg==1U || f.leg==3U || f.leg==5U) { in.centre_m=drive_goal; }
            if(bad_range && f.leg==1) { in.observation.range_valid=false; }
            if(f.leg==5) {
                in.tape=see_line;
                /* Hold real tape for 30 ms before completing return. */
                if(see_line && (!f.recovery.touching || (uint32_t)(in.now_us-f.recovery.since_us)<40000U)) {
                    a=s5_avoidance_step(&f,&in);continue;
                }
            }
            in.motion_done=true;
        }
        a=s5_avoidance_step(&f,&in);
        if(early_clear) { assert(f.pass==S5_PASS_STEP && f.clear_count==0U); }
    }
    if(block && first_side_only) { assert(f.swapped && f.state==S5_COMPLETE && heading==0);return; }
    if(block) { assert(f.state==S5_FAULT && f.fault==S5_NO_PATH);return; }
    if(deep*S5_PEEK_STEP_MM>cfg.max_depth_mm) { assert(f.state==S5_FAULT && f.fault==S5_NO_PATH);return; }
    if(!bad_range && !block) {
        assert(peeks==step_moves+2U && f.searched_mm==step_moves*S5_PEEK_STEP_MM);
        assert(f.searched_mm>=f.plan.clear_after_mm && step_moves>=deep);
        if(!deep) { assert(early_clears>0); } /* Empty space before the front is not a passed rear. */
    }
    if(bad_range) { assert(f.state==S5_FAULT && f.fault==S5_BAD_RANGE); }
    else if(see_line) { assert(f.state==S5_COMPLETE && a.kind==S5_ACT_RESUME && route_turns==4 && heading==0); }
    else { assert(f.state==S5_FAULT && f.fault==S5_NO_LINE); }
    s5_avoidance_abort(&f);assert(f.state==S5_IDLE);
}
int main(void)
{
    codecs();scans();fsm(true,false,false);fsm(true,true,false);fsm(true,false,true);fsm(false,false,false);
    { /* A close obstacle must not silently become an unsafe diagonal departure. */
        unsigned ang[12]={30,60,90,120,150,75,80,85,90,95,100,105},rr[12]={4000,215,196,215,4000,203,199,197,196,197,199,203};
        s5_sample_t ss[12];s5_config_t c=test_config();s5_profile_t pp;s5_plan_t pl;
        for(unsigned i=0;i<12;i++) { assert(s5_sample_pack(ang[i],rr[i],true,&ss[i])); }
        assert(s5_profile_build(ss,12,&c,&pp));
        assert(!s5_plan(&pp,&c,&pl));
    }
    /* A blocked first heading returns through two 45 degree turns to the alternate side. */
    first_side_only=true;fsm(true,true,false);first_side_only=false;
    /* Occupied or pre-front clear peeks advance in 25 mm steps; only two
     * distinct post-front clear observations can confirm the rear. */
    fsm_depth(true,false,false,4);
    /* Deeper than max_depth_mm: stop with NO_PATH instead of driving on blind. */
    fsm_depth(true,false,false,21);
    { /* Single-angle peek scan and merge. */
        s5_scan_t s;
        assert(!s5_scan_begin_at(&s,0,90) && !s5_scan_begin_at(&s,0,65) && s5_scan_begin_at(&s,0,30));
        s5_scan_accept(&s,300000,true,180);assert(!s.done && s.count==0);
        s5_scan_accept(&s,370000,true,185);assert(!s.done && s.count==0);
        s5_scan_accept(&s,440000,true,175);assert(s.done && !s.failed && s.count==1);
        s5_profile_t q=profile();unsigned width=q.width_mm,front=q.front_mm;
        assert(s5_peek_merge(&q,&s.sample[0]) && q.right_clear_mm==180 && q.width_mm==width && q.front_mm==front);
    }
    /* White alone must never confirm the original line, even in corridor. */
    s5_recovery_t rec={0};for(unsigned i=0;i<100;i++) { assert(!s5_recovery_observe(&rec,i*10000,true,false,false,true)); }
    assert(!s5_recovery_observe(&rec,1000000,true,true,false,false));
    s5_config_t cfg=test_config();s5_avoidance_t f;s5_avoidance_init(&f,&cfg);s5_avoidance_begin(&f,0);
    s5_input_t in={.now_us=1000001,.healthy=true};assert(s5_avoidance_step(&f,&in).kind==S5_ACT_STOP && f.fault==S5_DEADLINE);
    { /* A DONE report without the required rear-clearance wheel travel must
       * not authorize the inward turn. Old scan results cannot confirm twice. */
        s5_avoidance_init(&f,&cfg);s5_avoidance_begin(&f,0);
        f.state=S5_MOVING;f.leg=3;f.pass=S5_PASS_PEEKING;f.phase_us=10000;
        f.clear_count=1;f.plan.route=S5_ROUTE_LEFT;f.plan.peek_span_mm=300;
        in=(s5_input_t){.now_us=20000,.healthy=true,.stopped=true,
            .observation={.scan_done=true,.scan_us=10000,.profile=profile()}};
        assert(s5_avoidance_step(&f,&in).kind==S5_ACT_NONE && f.clear_count==1);
        in.observation.scan_us=20000;
        assert(s5_avoidance_step(&f,&in).kind==S5_ACT_NONE && f.clear_count==2 && f.pass==S5_PASS_EXTRA);
        f.issued=true;f.plan.rear_clear_mm=20;f.clear_at_m=0.0;
        in.now_us=30000;in.motion_done=true;in.centre_m=0.100;
        in.observation.range_valid=true;in.observation.range_mm=2000;in.observation.range_us=30000;
        assert(s5_avoidance_step(&f,&in).kind==S5_ACT_STOP && f.fault==S5_MOTION_ERROR);
    }
    { /* Concrete regression: width40/front300 leaves the sonar's initial side
       * cone before the obstacle. Two clear readings there cannot mean the
       * rear has passed. Start with the measured real-car footprint. */
        s5_config_t real=s5_defaults();s5_plan_t plan;
        s5_profile_t near={.valid=true,.closest_mm=300,.front_mm=300,.forward_mm=300,
            .left_edge_mm=20,.right_edge_mm=-20,.width_mm=40,
            .left_clear_mm=2000,.right_clear_mm=2000};
        assert(s5_plan(&near,&real,&plan) && plan.clear_after_mm>100U);
        s5_avoidance_init(&f,&real);s5_avoidance_begin(&f,0);
        f.state=S5_MOVING;f.leg=3;f.pass=S5_PASS_PEEKING;f.plan=plan;
        f.phase_us=10000;f.pass_start_m=0.0;
        in=(s5_input_t){.now_us=20000,.healthy=true,.stopped=true,
            .observation={.scan_done=true,.scan_us=20000,.profile=near}};
        assert(s5_avoidance_step(&f,&in).kind==S5_ACT_NONE);
        assert(f.pass==S5_PASS_STEP && f.clear_count==0U);
        f.pass=S5_PASS_PEEKING;in.now_us=in.observation.scan_us=30000;
        assert(s5_avoidance_step(&f,&in).kind==S5_ACT_NONE);
        assert(f.pass==S5_PASS_STEP && f.clear_count==0U);
        /* After the measured front-plane gate, require two NEW completions. */
        in.centre_m=(plan.clear_after_mm+1U)/1000.0;
        f.pass=S5_PASS_PEEKING;in.now_us=in.observation.scan_us=40000;
        assert(s5_avoidance_step(&f,&in).kind==S5_ACT_NONE);
        assert(f.pass==S5_PASS_PEEK && f.clear_count==1U);
        f.pass=S5_PASS_PEEKING;in.now_us=in.observation.scan_us=50000;
        assert(s5_avoidance_step(&f,&in).kind==S5_ACT_NONE);
        assert(f.pass==S5_PASS_EXTRA && f.clear_count==2U);
    }
    puts("PASS: range/codec boundaries, hostile packets, both-edge refinement, invalid echo, compact 45 degree FSM with immediate peeks, floor limits, faults and line evidence");
}
