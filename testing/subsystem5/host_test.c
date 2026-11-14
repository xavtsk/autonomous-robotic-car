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
    /* Edge padding scales with the farthest obstacle return: 250 mm * .173 -> 44 each side. */
    assert(S5_EDGE_INFLATE_PERMIL==173U && cfg.detect_mm==250U && S5_LEG_CLEAR_MM==200U);
    assert(p.valid && p.closest_mm==250 && p.width_mm==88 && p.left_clear_mm==1900 && p.right_clear_mm==1800);
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
    assert(out.range_us==in.range_us && out.range_mm==4000 && out.scan_id==65535 && out.profile.left_clear_mm==1900);
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
    /* Open space is clear, not a fault; too-short pulses stay invalid. */
    assert(ultrasonic_echo_mm(2000,&mm) && mm==343);
    assert(ultrasonic_echo_mm(25000,&mm) && mm==S5_RANGE_MAX_MM);
    assert(ultrasonic_echo_mm(23900,&mm) && mm==S5_RANGE_MAX_MM);
    assert(!ultrasonic_echo_mm(0,&mm) && mm==0);assert(!ultrasonic_echo_mm(100,&mm));
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
    /* Diagonal side readings only pick the side; too wide an offset still fails. */
    p.left_clear_mm=10;p.right_clear_mm=20;
    assert(s5_plan(&p,&cfg,&plan) && plan.route==(S5_PREFER_ROUTE==2U?S5_ROUTE_RIGHT:S5_PREFER_ROUTE==1U?S5_ROUTE_LEFT:S5_ROUTE_RIGHT) && plan.alt_ok==(S5_ALLOW_OTHER_SIDE!=0));
    /* Left too wide but right fine: left-only never plans the right side. */
    { s5_profile_t q=p;q.left_edge_mm=900;q.right_edge_mm=-20;
      assert(s5_plan(&q,&cfg,&plan)==(S5_ALLOW_OTHER_SIDE!=0 || S5_PREFER_ROUTE!=1U)); }
    p.left_edge_mm=900;p.right_edge_mm=-900;assert(!s5_plan(&p,&cfg,&plan));
    s5_scan_begin(&s,0);for(unsigned i=1;i<=3;i++) { s5_scan_accept(&s,i*70000,false,0); }
    assert(s.failed && s.done && s.count==0);
    s5_scan_begin(&s,0);s5_scan_accept(&s,S5_SCAN_TIMEOUT_US,true,200);assert(s.failed);
    s5_sample_t invalid[5];for(unsigned i=0;i<5;i++) { assert(s5_sample_pack(30+30*i,0,false,&invalid[i])); }
    assert(!s5_profile_build(invalid,5,&cfg,&p));
}
static unsigned peeks; static bool first_side_only;
static int turns[16]; static unsigned nturns, leg1_drive, leg5_drive;
unsigned pass_extra_test(const s5_avoidance_t *a);
static void fsm_depth(bool see_line,bool block,bool bad_range,unsigned deep);
static void fsm(bool see_line,bool block,bool bad_range) { fsm_depth(see_line,block,bad_range,0); }
static void fsm_depth(bool see_line,bool block,bool bad_range,unsigned deep)
{
    s5_config_t cfg=s5_defaults();s5_avoidance_t f;s5_avoidance_init(&f,&cfg);
    s5_input_t in={.healthy=true,.stopped=false,.line_ready=true};
    s5_avoidance_begin(&f,0);
    assert(s5_avoidance_step(&f,&in).kind==S5_ACT_NONE && f.state==S5_WAIT_STOP);
    in.now_us=10000;in.stopped=true;assert(s5_avoidance_step(&f,&in).kind==S5_ACT_SCAN);
    in.now_us=20000;in.observation.scan_done=true;in.observation.scan_us=20000;in.observation.profile=profile();
    if(block) { in.observation.profile.left_clear_mm=in.observation.profile.right_clear_mm=10; }
    s5_action_t a=s5_avoidance_step(&f,&in);
    peeks=0;nturns=0;leg1_drive=0;leg5_drive=0;
    for(unsigned tick=0;tick<3000 && f.state!=S5_COMPLETE && f.state!=S5_FAULT;tick++) {
        in.now_us+=10000;in.observation.range_us=in.now_us;in.observation.range_valid=true;
        /* block: after turning, the sideways path reads 150 mm (< offset + margin). */
        in.observation.range_mm=(block && f.leg==1 && (!first_side_only || !f.swapped))?150:2000;
        in.tape=false;in.motion_done=false;
        if(a.kind==S5_ACT_TURN && nturns<16U) { turns[nturns++]=a.value; }
        if(a.kind==S5_ACT_DRIVE && f.leg==1) { leg1_drive=(unsigned)a.value; }
        if(a.kind==S5_ACT_DRIVE && f.leg==5) { leg5_drive=(unsigned)a.value; }
        if(a.kind==S5_ACT_PEEK) {
            /* LEFT bypass looks right (30 deg); report that side's reading. */
            assert(f.leg==3 && f.pass==S5_PASS_PEEKING && a.value==(f.plan.route==S5_ROUTE_LEFT?30:150));
            in.observation.scan_done=true;in.observation.scan_us=in.now_us;
            unsigned side=peeks++<deep?120U:2000U; /* 120 mm: still beside us */
            if(f.plan.route==S5_ROUTE_LEFT) { in.observation.profile.right_clear_mm=(uint16_t)side; }
            else { in.observation.profile.left_clear_mm=(uint16_t)side; }
        }
        if(f.issued) {
            if(bad_range && f.leg==1) { in.observation.range_valid=false; }
            if(f.leg==6) {
                /* Back on the original line: tape under the IR while turning
                 * back to the heading. Hold it 40 ms (needs 30 ms). */
                in.tape=see_line;
                if(see_line && (!f.recovery.touching || (uint32_t)(in.now_us-f.recovery.since_us)<40000U)) {
                    a=s5_avoidance_step(&f,&in);continue;
                }
            }
            in.motion_done=true;
        }
        a=s5_avoidance_step(&f,&in);
    }
    if(block && first_side_only) {
        /* Out 45, blocked, 2 x 45 to the other diagonal, then that side. */
        int s=f.plan.route==S5_ROUTE_LEFT?1:-1;
        assert(f.swapped && f.state==S5_COMPLETE && nturns>=5U);
        assert(turns[0]==-s*45 && turns[1]==s*90 && turns[2]==-s*45 && turns[3]==-s*45 && turns[4]==s*45);
        return;
    }
    if(!block && !bad_range && see_line && deep*S5_PEEK_STEP_MM<=cfg.max_depth_mm) {
        /* 45-degree legs: out, parallel, back, realign (then the bounded search). */
        int s=f.plan.route==S5_ROUTE_LEFT?1:-1;
        assert(nturns>=4U && turns[0]==s*45 && turns[1]==-s*45 && turns[2]==-s*45 && turns[3]==s*45);
        assert(leg1_drive==(f.plan.offset_mm*1414U+999U)/1000U && leg5_drive==leg1_drive);
    }
    if(block) { assert(f.state==S5_FAULT && f.fault==S5_NO_PATH && f.leg==1 && !f.issued &&
                       f.swapped==(S5_ALLOW_OTHER_SIDE!=0) && f.plan.route==S5_ROUTE_LEFT+(S5_ALLOW_OTHER_SIDE!=0));return; }
    if(deep*S5_PEEK_STEP_MM>cfg.max_depth_mm) { assert(f.state==S5_FAULT && f.fault==S5_NO_PATH);return; }
    if(!bad_range && !block && peeks!=deep+1U) fprintf(stderr,"deep=%u peeks=%u searched=%u state=%u fault=%u leg=%u pass=%u\n",deep,peeks,f.searched_mm,f.state,f.fault,f.leg,f.pass);
    if(bad_range) { assert(f.state==S5_FAULT && f.fault==S5_BAD_RANGE); }
    else if(see_line) { assert(f.state==S5_COMPLETE && a.kind==S5_ACT_RESUME); }
    else { assert(f.state==S5_FAULT && f.fault==S5_NO_LINE); }
    s5_avoidance_abort(&f);assert(f.state==S5_IDLE);
}
/* Line re-acquisition after the diagonal return: tape appears only while
 * crossing on the return diagonal (found_leg 5, must NOT count), only during
 * the realign turn (6) or one sweep turn (7/8/9); -1 never. */
static void fsm_line(int found_leg,int *sweep,unsigned *back_drive)
{
    s5_config_t cfg=s5_defaults();s5_avoidance_t f;s5_avoidance_init(&f,&cfg);
    s5_input_t in={.healthy=true,.stopped=true,.line_ready=true};
    s5_action_t a;
    unsigned n=0;
    s5_avoidance_begin(&f,0);in.now_us=10000;(void)s5_avoidance_step(&f,&in);
    in.now_us=20000;in.observation.scan_done=true;in.observation.scan_us=20000;in.observation.profile=profile();
    a=s5_avoidance_step(&f,&in);*back_drive=0;
    for(unsigned tick=0;tick<4000 && f.state!=S5_COMPLETE && f.state!=S5_FAULT;tick++) {
        in.now_us+=10000;in.observation.range_us=in.now_us;in.observation.range_valid=true;in.observation.range_mm=2000;
        if(a.kind==S5_ACT_PEEK) {
            in.observation.scan_done=true;in.observation.scan_us=in.now_us;
            in.observation.profile.right_clear_mm=in.observation.profile.left_clear_mm=2000;
        }
        if(a.kind==S5_ACT_DRIVE && f.leg==5) { *back_drive=(unsigned)a.value; }
        if(a.kind==S5_ACT_TURN && f.leg>=7 && n<4U) { sweep[n++]=a.value; }
        assert(!(a.kind==S5_ACT_DRIVE && f.leg>5));   /* no straight creeping off the path */
        in.tape=found_leg>=5 && (int)f.leg==found_leg && f.issued;
        in.motion_done=false;
        if(f.issued) {
            /* Hold the motion open for 50 ms while tape is visible (the
             * crossing on leg 5 is brief: it just passes under the IR). */
            if(in.tape && f.leg>=6 && (!f.recovery.touching || (uint32_t)(in.now_us-f.recovery.since_us)<50000U)) {
                a=s5_avoidance_step(&f,&in);continue;
            }
            in.motion_done=true;
        }
        a=s5_avoidance_step(&f,&in);
    }
    if(found_leg<6) { assert(f.state==S5_FAULT && f.fault==S5_NO_LINE); }
    else { assert(f.state==S5_COMPLETE && a.kind==S5_ACT_RESUME); }
}
/* Obstacle closer than the 45-degree out-leg allows: reverse the shortfall
 * (once), then the normal out turn. Returns the reverse distance. */
static int fsm_backup(unsigned closest,int left_edge)
{
    s5_config_t c=s5_defaults();s5_avoidance_t f;s5_avoidance_init(&f,&c);
    s5_input_t in={.healthy=true,.stopped=true,.line_ready=true};
    s5_action_t a;int back=0;unsigned turn=0;
    s5_avoidance_begin(&f,0);in.now_us=10000;(void)s5_avoidance_step(&f,&in);
    in.now_us=20000;in.observation.scan_done=true;in.observation.scan_us=20000;
    in.observation.profile=profile();in.observation.profile.closest_mm=(uint16_t)closest;
    in.observation.profile.left_edge_mm=(int16_t)left_edge;
    in.observation.profile.right_edge_mm=(int16_t)-left_edge;   /* centred: no aim turn */
    a=s5_avoidance_step(&f,&in);assert(f.state==S5_MOVING);
    for(unsigned t=0;t<20 && f.state==S5_MOVING && !turn;t++) {
        in.now_us+=10000;in.observation.range_us=in.now_us;in.observation.range_valid=true;
        in.observation.range_mm=f.backing?60:2000;   /* close face while reversing is fine */
        in.motion_done=f.issued;a=s5_avoidance_step(&f,&in);
        if(a.kind==S5_ACT_DRIVE) { assert(f.leg==0 && f.backing && !back);back=a.value; }
        if(a.kind==S5_ACT_TURN) { assert(f.leg==0 && a.value==S5_BYPASS_DEG);turn=1; }
    }
    if(f.state==S5_FAULT) { assert(f.fault==S5_NO_PATH && !back);return 1; }
    assert(turn && f.backed && !f.backing);
    return back;
}
/* Stopped crooked on a bend: obstacle centre off the nose. The car turns
 * to face it once, scans again, then plans. Returns the aim turn (0 none). */
static int fsm_aim_turn(int left_edge,int right_edge,unsigned closest,bool second_off,double turn)
{
    s5_config_t c=s5_defaults();s5_avoidance_t f;s5_avoidance_init(&f,&c);
    s5_input_t in={.healthy=true,.stopped=true,.line_ready=true,.turn_ratio=turn};
    s5_action_t a;int turn_deg=0;unsigned scans=1;
    s5_avoidance_begin(&f,0);in.now_us=10000;assert(s5_avoidance_step(&f,&in).kind==S5_ACT_SCAN);
    in.now_us=20000;in.observation.scan_done=true;in.observation.scan_us=20000;
    in.observation.profile=profile();in.observation.profile.closest_mm=(uint16_t)closest;
    in.observation.profile.left_edge_mm=(int16_t)left_edge;in.observation.profile.right_edge_mm=(int16_t)right_edge;
    a=s5_avoidance_step(&f,&in);
    for(unsigned t=0;t<50 && f.state==S5_SCANNING;t++) {
        if(a.kind==S5_ACT_TURN) { assert(!turn_deg && f.aiming && f.state==S5_SCANNING);turn_deg=a.value; }
        if(a.kind==S5_ACT_SCAN) {
            /* Rescan after the aim: centred now, unless the test keeps it off. */
            scans++;in.observation.scan_us=in.now_us+5000;
            if(!second_off) { in.observation.profile=profile(); }
        }
        in.now_us+=10000;in.motion_done=f.issued;
        in.stopped=!f.issued;
        a=s5_avoidance_step(&f,&in);
    }
    assert(f.state==S5_MOVING && !f.issued && !f.aiming && f.aimed);
    assert(scans==(turn_deg?2U:1U) && f.aim_deg==turn_deg && f.bend==(turn>=S5_BEND_RATIO || turn<=-S5_BEND_RATIO));
    return turn_deg;
}
static int fsm_aim(int l,int r,unsigned c,bool off) { return fsm_aim_turn(l,r,c,off,0.0); }
/* Return diagonal. tape_at_mm: leg-5 travel where the IR first sees tape (0 =
 * never on leg 5). Records the leg-5 drives and whether the car stopped early.
 * Tape is also under the IR during the turn back. */
static void fsm_cross(double turn,unsigned tape_at_mm,unsigned *drives,unsigned *n,bool *stopped_early)
{
    s5_config_t c=s5_defaults();s5_avoidance_t f;s5_avoidance_init(&f,&c);
    s5_input_t in={.healthy=true,.stopped=true,.line_ready=true,.turn_ratio=turn};
    s5_action_t a;double pos=0.0;unsigned want=0;
    s5_avoidance_begin(&f,0);in.now_us=10000;(void)s5_avoidance_step(&f,&in);
    in.now_us=20000;in.observation.scan_done=true;in.observation.scan_us=20000;in.observation.profile=profile();
    a=s5_avoidance_step(&f,&in);*n=0;*stopped_early=false;
    for(unsigned tick=0;tick<6000 && f.state!=S5_COMPLETE && f.state!=S5_FAULT;tick++) {
        in.now_us+=10000;in.observation.range_us=in.now_us;in.observation.range_valid=true;in.observation.range_mm=2000;
        if(a.kind==S5_ACT_PEEK) {
            in.observation.scan_done=true;in.observation.scan_us=in.now_us;
            in.observation.profile.right_clear_mm=in.observation.profile.left_clear_mm=2000;
        }
        if(a.kind==S5_ACT_DRIVE && f.leg==5) { assert(*n<4U);drives[(*n)++]=(unsigned)a.value;want=(unsigned)a.value;pos=0.0; }
        if(a.kind==S5_ACT_STOP && f.leg==5) { *stopped_early=true; }
        in.motion_done=false;in.tape=false;
        if(f.issued && f.leg==5) {
            unsigned before=0;for(unsigned i=0;i+1<*n;i++) { before+=drives[i]; }
            pos+=5.0;
            in.tape=tape_at_mm && !f.tail && before+(unsigned)pos>=tape_at_mm && before+(unsigned)pos<tape_at_mm+20U;
            if(pos>=(double)want) { in.motion_done=true; }
        } else if(f.issued) {
            in.tape=f.leg==6;
            if(in.tape && (!f.recovery.touching || (uint32_t)(in.now_us-f.recovery.since_us)<50000U)) {
                a=s5_avoidance_step(&f,&in);continue;
            }
            in.motion_done=true;
        }
        a=s5_avoidance_step(&f,&in);
    }
    assert(f.state==S5_COMPLETE);
}
int main(void)
{
    codecs();scans();fsm(true,false,false);fsm(true,true,false);fsm(true,false,true);fsm(false,false,false);
    { /* 10 cm box at 196 mm: beam smear at 60/120 must not block either side. */
        unsigned ang[12]={30,60,90,120,150,75,80,85,90,95,100,105},rr[12]={4000,215,196,215,4000,203,199,197,196,197,199,203};
        s5_sample_t ss[12];s5_config_t c=s5_defaults();s5_profile_t pp;s5_plan_t pl;
        for(unsigned i=0;i<12;i++) { assert(s5_sample_pack(ang[i],rr[i],true,&ss[i])); }
        assert(s5_profile_build(ss,12,&c,&pp) && pp.width_mm<=230 && pp.left_clear_mm==4000 && pp.right_clear_mm==4000);
        assert(s5_plan(&pp,&c,&pl) && pl.offset_mm<300);
    }
    { /* 45-degree out-leg sweeps forward: a wide obstacle close ahead blocks
       * the diagonal even with a clear sensor path (no hidden corner hit). */
        s5_config_t c=s5_defaults();s5_avoidance_t f;s5_avoidance_init(&f,&c);
        s5_input_t in={.healthy=true,.stopped=true,.line_ready=true};
        s5_avoidance_begin(&f,0);in.now_us=10000;(void)s5_avoidance_step(&f,&in);
        in.now_us=20000;in.observation.scan_done=true;in.observation.scan_us=20000;
        in.observation.profile=profile();
        in.observation.profile.closest_mm=120;in.observation.profile.left_edge_mm=300;
        in.observation.profile.right_edge_mm=-300;in.observation.profile.width_mm=600;
        (void)s5_avoidance_step(&f,&in);assert(f.state==S5_MOVING);
        for(unsigned t=0;t<40 && f.state==S5_MOVING;t++) {
            s5_action_t a;
            in.now_us+=10000;in.observation.range_us=in.now_us;in.observation.range_valid=true;
            in.observation.range_mm=2000;in.motion_done=f.issued;a=s5_avoidance_step(&f,&in);
            assert(a.kind!=S5_ACT_DRIVE);  /* never drives the blocked diagonal */
        }
        assert(f.state==S5_FAULT && f.fault==S5_NO_PATH && f.swapped==(S5_ALLOW_OTHER_SIDE!=0));
    }
#if S5_ALLOW_OTHER_SIDE
    /* Preferred side blocked after the turn: 2 x 45 deg, take the other side. */
    first_side_only=true;fsm(true,true,false);first_side_only=false;
#endif
    { /* Aim angle: centre x over (closest + sensor-to-axle), + = left. */
        s5_profile_t q=profile();
        q.left_edge_mm=44;q.right_edge_mm=-44;assert(s5_aim_deg(&q)==0);        /* centred */
        q.left_edge_mm=60;q.right_edge_mm=-30;assert(s5_aim_deg(&q)==0);        /* 15/325: 2.6 deg, under 5 */
        q.left_edge_mm=150;q.right_edge_mm=-10;assert(s5_aim_deg(&q)==12);      /* atan(70/325) = 12.2 */
        q.left_edge_mm=10;q.right_edge_mm=-150;assert(s5_aim_deg(&q)==-12);     /* mirror: turn right */
        q.left_edge_mm=700;q.right_edge_mm=300;assert(s5_aim_deg(&q)==S5_AIM_MAX_DEG); /* 57 deg: capped */
        q.left_edge_mm=-100;q.right_edge_mm=-800;assert(s5_aim_deg(&q)==-S5_AIM_MAX_DEG);
        q.closest_mm=150;q.left_edge_mm=130;q.right_edge_mm=10;assert(s5_aim_deg(&q)==17); /* closer: atan(70/225) = 17.3 */
    }
    { /* Bend only: bigger aim when the follower was turning toward the obstacle side. */
        s5_profile_t q=profile();bool bend;
        q.left_edge_mm=150;q.right_edge_mm=-10;                                  /* bearing 12 deg left */
        assert(S5_BEND_RATIO>0.19 && S5_BEND_RATIO<0.21);
        assert(s5_aim_deg_bend(&q,0.05,&bend)==12 && !bend);                     /* straight: unchanged */
        assert(s5_aim_deg_bend(&q,0.15,&bend)==12 && !bend);                     /* wobble: still straight */
        assert(s5_aim_deg_bend(&q,0.20,&bend)==25 && bend);                      /* left bend: 2 x 12.3 */
        assert(s5_aim_deg_bend(&q,-0.20,&bend)==12 && bend);                     /* right bend, obstacle left: 1x */
        q.left_edge_mm=10;q.right_edge_mm=-150;assert(s5_aim_deg_bend(&q,-0.20,&bend)==-25);
        q.left_edge_mm=60;q.right_edge_mm=-30;assert(s5_aim_deg_bend(&q,0.30,&bend)==0); /* under 5 deg */
        q.left_edge_mm=700;q.right_edge_mm=300;assert(s5_aim_deg_bend(&q,0.30,&bend)==S5_BEND_AIM_MAX_DEG);
    }
    assert(fsm_aim_turn(150,-10,250,false,0.20)==25);   /* bend: 2x, rescan, plan */
    assert(fsm_aim_turn(150,-10,250,false,0.05)==12);   /* gentle steering = straight */
    { /* Return diagonal: straight keeps the full diagonal (tape crossing ignored);
       * bend stops on tape, then drives IR-to-axle; bend away gets one extra drive. */
        s5_config_t c=s5_defaults();s5_plan_t pl;s5_profile_t pp=profile();assert(s5_plan(&pp,&c,&pl));
        unsigned diag=(pl.offset_mm*1414U+999U)/1000U,d[4],n;bool early;
        /* Straight: tape stops the car too, then only IR-to-axle (never on past the line). */
        fsm_cross(0.0,60,d,&n,&early);   assert(early && n==2 && d[0]==diag && d[1]==S5_IR_AXLE_MM);
        fsm_cross(0.0,diag-S5_IR_AXLE_MM,d,&n,&early); assert(early && n==2 && d[1]==S5_IR_AXLE_MM);
        fsm_cross(0.0,0,d,&n,&early);    assert(!early && n==1 && d[0]==diag);   /* no extra on a straight */
        fsm_cross(0.25,60,d,&n,&early);  assert(early && n==2 && d[0]==diag && d[1]==S5_IR_AXLE_MM);
        fsm_cross(-0.25,diag+50,d,&n,&early); assert(early && n==3 && d[1]==S5_RETURN_EXTRA_MM && d[2]==S5_IR_AXLE_MM);
        fsm_cross(0.25,0,d,&n,&early);   assert(!early && n==2 && d[1]==S5_RETURN_EXTRA_MM);
    }
    { /* 20 cm worst case: the offset never plans for less than a 20 cm wide
       * obstacle about the scanned centre; wider readings still win. */
        s5_config_t c=s5_defaults();s5_plan_t pl;s5_profile_t q=profile();
        unsigned pad=c.half_width_mm+c.margin_mm;
        assert(S5_OBSTACLE_MAX_MM==200);
        q.left_edge_mm=22;q.right_edge_mm=-22;assert(s5_plan(&q,&c,&pl) && pl.offset_mm==100U+pad);
        q.left_edge_mm=150;q.right_edge_mm=-150;assert(s5_plan(&q,&c,&pl) && pl.offset_mm==150U+pad);
        q.left_edge_mm=120;q.right_edge_mm=40;assert(s5_plan(&q,&c,&pl) && pl.offset_mm==180U+pad); /* centre 80 + 100 */
    }
    assert(fsm_aim(44,-44,250,false)==0);      /* straight: no aim, one scan */
    assert(fsm_aim(150,-10,250,false)==12);    /* straight, car crooked: face it, rescan, then plan */
    assert(fsm_aim(150,-10,250,true)==12);     /* still off after the rescan: no second aim */
    /* Too close: 100 mm face, edge 75 (20 cm worst case: 100) -> corner needs 90+100+30=220 vs 175: back 45. */
    assert(fsm_backup(100,75)==-45);
    assert(fsm_backup(250,44)==0);     /* far enough: no reverse */
    assert(fsm_backup(60,200)==1);     /* would need 185 mm > S5_BACKUP_MAX_MM: NO_PATH */
    /* Unknown depth: 4 peeks see the obstacle (4 x 50 mm steps), the 5th is clear. */
    fsm_depth(true,false,false,4);
    /* Deeper than max_depth_mm: stop with NO_PATH instead of driving on blind. */
    fsm_depth(true,false,false,11);
    { /* Finding the line after the return diagonal (same length as the out one). */
        int sweep[4]={0};unsigned back;
        s5_config_t c=s5_defaults();s5_avoidance_t g;s5_avoidance_init(&g,&c);
        s5_plan_t pl;s5_profile_t pp=profile();assert(s5_plan(&pp,&c,&pl));
        fsm_line(-1,sweep,&back);     /* never: sweep +45/-90/+45, NO_LINE */
        assert(back==(pl.offset_mm*1414U+999U)/1000U);
        assert(S5_LINE_SWEEP_DEG==45 && sweep[0]==45 && sweep[1]==-90 && sweep[2]==45);
        sweep[0]=sweep[1]=sweep[2]=0;
        fsm_line(5,sweep,&back);      /* tape only while crossing at 45 deg: ignored, still sweeps */
        assert(sweep[0]==45 && sweep[1]==-90 && sweep[2]==45);
        sweep[0]=sweep[1]=sweep[2]=0;
        fsm_line(6,sweep,&back);      /* on the line after turning back: no sweep */
        assert(sweep[0]==0);
        fsm_line(8,sweep,&back);      /* found mid-sweep: stops there, no third turn */
        assert(sweep[0]==45 && sweep[1]==-90 && sweep[2]==0);
    }
    { /* Pass distance after "clear": peek geometry + sensor-to-rear + margin
       * (was 0.577 x whole offset + sensor-to-axle + half width + margin). */
        s5_config_t c=s5_defaults();s5_avoidance_t f;s5_avoidance_init(&f,&c);
        unsigned pad=c.half_width_mm+c.margin_mm;
        assert(c.half_width_mm==63U && S5_SONAR_AXLE_MM==75U && S5_PEEK_STEP_MM==25U);
        f.plan.offset_mm=250U;
        assert(pass_extra_test(&f)==(pad*577U+999U)/1000U+S5_SONAR_AXLE_MM+S5_REAR_AXLE_MM+c.margin_mm);
    }
    { /* Single-angle peek scan and merge. */
        s5_scan_t s;s5_profile_t m={0};
        assert(!s5_scan_begin_at(&s,0,90) && !s5_scan_begin_at(&s,0,65) && s5_scan_begin_at(&s,0,30));
        s5_scan_accept(&s,300000,true,180);assert(s.done && !s.failed && s.count==1);
        assert(s5_peek_merge(&m,&s.sample[0]) && m.valid && m.right_clear_mm==180 && m.left_clear_mm==S5_RANGE_MAX_MM);
        s5_profile_t q=profile();assert(s5_peek_merge(&q,&s.sample[0]) && q.right_clear_mm==180 && q.width_mm==88);
    }
    /* White alone must never confirm the original line, even in corridor. */
    s5_recovery_t rec={0};for(unsigned i=0;i<100;i++) { assert(!s5_recovery_observe(&rec,i*10000,true,false,false,true)); }
    assert(!s5_recovery_observe(&rec,1000000,true,true,false,false));
    s5_config_t cfg=s5_defaults();s5_avoidance_t f;s5_avoidance_init(&f,&cfg);s5_avoidance_begin(&f,0);
    s5_input_t in={.now_us=1000001,.healthy=true};assert(s5_avoidance_step(&f,&in).kind==S5_ACT_STOP && f.fault==S5_DEADLINE);
    puts("PASS: 45-degree bypass legs (equal out/back diagonals, left-only, reverse-if-too-close, realign+sweep line search); range/codec exhaustive boundaries, hostile packets, coarse/fine scans, invalid echo, route clearance, bounded FSM, faults and line evidence");
}
