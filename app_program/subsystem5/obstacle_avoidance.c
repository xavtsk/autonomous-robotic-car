#include "obstacle_avoidance.h"
static s5_action_t action(s5_action_kind_t kind,int value,unsigned speed)
{ return (s5_action_t){kind,(int16_t)value,(uint16_t)speed}; }
static s5_action_t fail(s5_avoidance_t *a,s5_fault_t reason)
{ a->state=S5_FAULT;a->fault=reason;return action(S5_ACT_STOP,0,0); }
void s5_avoidance_init(s5_avoidance_t *a,const s5_config_t *c)
{ *a=(s5_avoidance_t){0};a->config=*c; }
bool s5_range_ready(const s5_observation_t *o,uint32_t now)
{ return o && o->range_valid && !o->no_echo && o->range_mm>=30U && o->range_mm<=4000U && (uint32_t)(now-o->range_us)<=S5_RANGE_FRESH_US; }
void s5_avoidance_begin(s5_avoidance_t *a,uint32_t now)
{
    a->state=S5_WAIT_STOP;a->fault=S5_OK;a->phase_us=a->started_us=now;
    a->leg=0;a->issued=false;a->plan=(s5_plan_t){0};s5_recovery_reset(&a->recovery);
    a->pass=S5_PASS_APPROACH;a->searched_mm=0;a->swapped=0;a->backed=0;a->backing=0;
    a->aimed=0;a->aiming=0;a->aim_deg=0;a->bend=0;a->tail=0;a->extended=0;a->rel=0;a->side_turn=0;a->side_turns=0;a->centre_dir=0;a->centre_tries=0;a->lat_mm=0;a->peek_lat_mm=0;a->blocked_reads=0;
}
void s5_avoidance_abort(s5_avoidance_t *a)
{ a->state=S5_IDLE;a->issued=false;s5_recovery_reset(&a->recovery); }
/* Distance still needed after the side peek reads clear. On a ray S5_PEEK_DEG
 * from straight ahead, a return at lateral distance d lies d*tan(90-60) =
 * 0.577*d ahead of the sensor, so "clear" means the obstacle's rear edge is at
 * most 0.577*d ahead of the sensor. Drive until the car's BACK (sonar-to-axle
 * + axle-to-rear behind the sensor) is past it by the margin. d is the lateral
 * gap between the sensor line and the obstacle edge, not the whole offset. */
static unsigned pass_extra_mm(const s5_avoidance_t *a)
{
    const unsigned pad=a->config.half_width_mm+a->config.margin_mm;
    const unsigned edge=a->plan.offset_mm>pad?a->plan.offset_mm-pad:0U;
    const unsigned gap=a->plan.offset_mm>edge?a->plan.offset_mm-edge:0U;
#if S5_PASS_EXTRA_V2
    /* 17 Nov 2026. The return is a 45-degree turn on the spot, so the car does
     * not need its whole body past the rear edge first:
     * - "clear" means nothing in the peek cone (60 +/- 15 deg off the heading),
     *   so the side face ends before 0.268 x gap ahead of the sensor
     *   (cot 75 deg = 0.268);
     * - after turning back 45 degrees the car's line-side edge runs along
     *   x + y = -1.414 x half width; the rear corner of the obstacle (at the
     *   pass clearance half width + margin to the side) stays outside it by
     *   the margin once the axle is 0.414 x (half width + margin) past it.
     * Example, half width 63, margin 30: 75 + 25 + 39 = 139 mm (was 214). */
    return S5_SONAR_AXLE_MM+(gap*268U+pad*414U+999U)/1000U;
#else
    return (gap*577U+999U)/1000U+S5_SONAR_AXLE_MM+S5_REAR_AXLE_MM+a->config.margin_mm;
#endif
}
/* 45-degree legs: travel along the diagonal for a sideways offset d. */
static unsigned diagonal_mm(unsigned offset_mm) { return (offset_mm*1414U+999U)/1000U; }
/* Driving out at 45 degrees also moves the car forward, so its inner front
 * corner sweeps toward the obstacle face. With the face D = closest + sonar-
 * to-axle ahead of the axle and the near obstacle edge e = offset - (half
 * width + margin) to the side, that corner reaches the face plane d = D -
 * 1.414*half_width to the side: it must be past e + margin by then. */
static unsigned diagonal_short(const s5_avoidance_t *a,unsigned offset_mm)
{
    const unsigned pad=a->config.half_width_mm+a->config.margin_mm;
    const unsigned edge=offset_mm>pad?offset_mm-pad:0U;
    const unsigned face=a->plan.closest_mm+S5_SONAR_AXLE_MM;
    const unsigned corner=(a->config.half_width_mm*1414U+999U)/1000U;
    const unsigned need=corner+edge+a->config.margin_mm;
    return need>face?need-face:0U;   /* how much nearer the face is than safe */
}
static bool diagonal_clear(const s5_avoidance_t *a,unsigned offset_mm)
{ return diagonal_short(a,offset_mm)==0U; }
/* Bearing of the obstacle centre from the wheel axle, degrees, + = left. */
static double bearing_deg(const s5_profile_t *p)
{
    double x,y,t,t2=0.0;
    x=((double)p->left_edge_mm+(double)p->right_edge_mm)/2.0;   /* + = left */
    y=(double)p->closest_mm+(double)S5_SONAR_AXLE_MM;            /* axle to face */
    if(y<=0.0) { return 0.0; }
    t=x/y;
    if(t>1.0 || t<-1.0) { return t>0.0?90.0:-90.0; }   /* beyond 45: always capped */
    (void)t2;
    /* atan(t) for |t|<=1 without libm: pi/4*t + 0.273*t*(1-|t|), < 0.3 deg error. */
    return (0.7853982*t+0.273*t*(1.0-(t<0.0?-t:t)))*57.29578;
}
static int round_clamp(double deg,int max)
{
    int d=(int)(deg+(deg>=0.0?0.5:-0.5));
    return d>max?max:(d<-max?-max:d);
}
int s5_aim_deg(const s5_profile_t *p)   /* straight: as in the 17:31 version */
{
    int d;
    if(!p || !p->valid) { return 0; }
    /* Aiming assumes one obstacle on the line. A picture wider than 1.5 x the
     * largest obstacle is the obstacle plus something beside the track (a
     * wall): its middle is not the line, so do not turn toward it. */
    if(p->width_mm>(S5_OBSTACLE_MAX_MM*3U)/2U) { return 0; }
    d=round_clamp(bearing_deg(p),1000);
    if(d<S5_AIM_MIN_DEG && d>-S5_AIM_MIN_DEG) { return 0; }
    return d>S5_AIM_MAX_DEG?S5_AIM_MAX_DEG:(d<-S5_AIM_MAX_DEG?-S5_AIM_MAX_DEG:d);
}
int s5_aim_deg_bend(const s5_profile_t *p,int turn,bool *bend)
{
    double deg;
    *bend=turn>=S5_BEND_PCT || turn<=-S5_BEND_PCT;
    if(!*bend) { return s5_aim_deg(p); }           /* straight: unchanged */
    if(!p || !p->valid || p->width_mm>(S5_OBSTACLE_MAX_MM*3U)/2U) { return 0; }
    deg=bearing_deg(p);
    { int d=round_clamp(deg,1000); if(d<S5_AIM_MIN_DEG && d>-S5_AIM_MIN_DEG) { return 0; } }
    /* Obstacle on the side the bend goes: track direction there is ~2x. */
    if((deg>0.0)!=(turn>0)) { return round_clamp(deg,S5_AIM_MAX_DEG); }
    return round_clamp(deg*(double)S5_BEND_AIM_PCT/100.0,S5_BEND_AIM_MAX_DEG);
}
/* Sideways distance to cover on the way back: planned offset plus any turn
 * away while passing, or what the last side peek measured if that is more
 * (drifted away). Too long is safe: the line seen on the way stops the car. */
static unsigned return_lat(const s5_avoidance_t *a)
{
    const unsigned planned=a->plan.offset_mm+a->lat_mm;
    return a->peek_lat_mm>planned?a->peek_lat_mm:planned;
}
/* Swerve heading targets (deg vs the heading at the obstacle, + = bypass
 * side), legs 6, 7, 8. Line already seen (tail driven): look toward the
 * bypass side first, where it continues. Not seen at all: it can only be on
 * the line side still (a bend away), so look there first. */
static int swerve_target(const s5_avoidance_t *a)
{
    const int out=S5_SWERVE_OUT_DEG, in=-S5_SWERVE_IN_DEG;
    if(a->leg==6U) { return a->tail?out:in; }
    if(a->leg==7U) { return a->tail?in:out; }
    return 0;
}
#ifdef S5_HOST_TEST
unsigned pass_extra_test(const s5_avoidance_t *a) { return pass_extra_mm(a); }
#endif
static s5_action_t peek_result(s5_avoidance_t *a,const s5_input_t *in,unsigned elapsed)
{
    const s5_profile_t *p=&in->observation.profile;
    unsigned side,limit;
    if(elapsed>S5_SCAN_TIMEOUT_US) { return fail(a,S5_DEADLINE); }
    if(!in->observation.scan_done || (int32_t)(in->observation.scan_us-a->phase_us)<=0) { return action(S5_ACT_NONE,0,0); }
    if(!p->valid) { return fail(a,S5_SCAN_FAILED); }
    /* Obstacle on the car's right after a LEFT bypass, so look right (30 deg). */
    side=a->plan.route==S5_ROUTE_LEFT?p->right_clear_mm:p->left_clear_mm;
    /* Anything nearer than the original line (+margin) along the 60 deg ray:
     * range = lateral / sin(60 deg), 1/sin(60) = 1.155. */
    limit=((a->plan.offset_mm+a->config.margin_mm)*1155U+999U)/1000U;
    a->phase_us=in->now_us;
    if(side<=limit) {
        /* Still beside it: is the side as far as planned (half width +
         * margin from the sensor line)? If not, turn away, then peek again. */
        const unsigned pad=a->config.half_width_mm+a->config.margin_mm, gap=(side*966U)/1000U;
        const unsigned edge=a->plan.offset_mm>pad?a->plan.offset_mm-pad:0U;
        /* Measured distance from the original line now (obstacle edge + gap):
         * catches a car that drifted away while passing (left a bend crooked). */
        a->peek_lat_mm=edge+gap>4095U?4095U:edge+gap;
        if(gap+S5_SIDE_TOL_MM<pad && a->side_turns<3U) {
            unsigned deg=(pad-gap+1U)/2U;
            a->side_turn=deg>S5_SIDE_TURN_MAX_DEG?S5_SIDE_TURN_MAX_DEG:deg;
            a->side_turns++;
        }
        if(a->searched_mm+S5_PEEK_STEP_MM>a->config.max_depth_mm) { return fail(a,S5_NO_PATH); }
        a->pass=S5_PASS_STEP;
    } else { a->pass=S5_PASS_EXTRA; }
    return action(S5_ACT_NONE,0,0);
}
s5_action_t s5_avoidance_step(s5_avoidance_t *a,const s5_input_t *in)
{
    unsigned elapsed; int sign;
    if(!a || !in) { return action(S5_ACT_STOP,0,0); }
    if(a->state==S5_IDLE || a->state==S5_COMPLETE || a->state==S5_FAULT) { return action(S5_ACT_NONE,0,0); }
    if(!in->healthy) { return fail(a,S5_MOTION_ERROR); }
    if((uint32_t)(in->now_us-a->started_us)>60000000U) { return fail(a,S5_DEADLINE); }
    elapsed=in->now_us-a->phase_us;
    if(a->state==S5_WAIT_STOP) {
        if(elapsed>1000000U) { return fail(a,S5_DEADLINE); }
        if(in->stopped) { a->state=S5_SCANNING;a->phase_us=in->now_us;return action(S5_ACT_SCAN,0,0); }
    } else if(a->state==S5_SCANNING) {
        if(elapsed>S5_SCAN_TIMEOUT_US) { return fail(a,S5_DEADLINE); }
        if(a->aiming) {
            /* Face-the-obstacle turn: wait for it, then scan again square-on. */
            if(a->issued) {
                if(in->motion_done) { a->issued=false;a->phase_us=in->now_us; }
                return action(S5_ACT_NONE,0,0);
            }
            if(!in->stopped) { return action(S5_ACT_NONE,0,0); }
            a->aiming=0;a->phase_us=in->now_us;return action(S5_ACT_SCAN,0,0);
        }
        if(in->observation.scan_done && (int32_t)(in->observation.scan_us-a->phase_us)>0) {
            if(!in->observation.profile.valid) { return fail(a,S5_SCAN_FAILED); }
            if(!a->aimed) {
                bool bend;
                int turn=s5_aim_deg_bend(&in->observation.profile,in->turn_pct,&bend);
                a->aimed=1;a->bend=bend;
                if(turn) {
                    a->aim_deg=turn;a->aiming=1;a->issued=true;a->phase_us=in->now_us;
                    return action(S5_ACT_TURN,turn,a->config.speed_mms);
                }
            }
            if(!s5_plan(&in->observation.profile,&a->config,&a->plan)) { return fail(a,S5_NO_PATH); }
            a->state=S5_MOVING;a->phase_us=in->now_us;
        }
    } else if(a->state==S5_MOVING) {
        bool straight=(a->leg==1U || a->leg==3U || a->leg==5U);
        if(elapsed>20000000U) { return fail(a,S5_DEADLINE); }
        /* Servo faces sideways during a peek: no forward range until it ends. */
        if(a->leg==3U && a->pass==S5_PASS_PEEKING) { return peek_result(a,in,elapsed); }
        /* Each straight leg requires a new forward reading after its turn.
         * During movement, loss of valid ranging or a close return brakes. */
        /* Reversing away from the face (leg 0) is exempt: the range only grows. */
        if(a->issued && !a->backing && (!s5_range_ready(&in->observation,in->now_us) ||
            in->observation.range_mm<=a->config.emergency_mm)) { return fail(a,S5_BAD_RANGE); }
        /* Line seen while passing (leg 3, only after the side reads clear,
         * so never beside the obstacle: a bend toward the bypass side) or on
         * the return diagonal (leg 5): stop, then the
         * IR-to-axle drive puts the axle on the line (as leg 5) before the
         * swerve. That tape is not kept: the swerve must see it again. */
        if(a->issued && !a->tail &&
           (a->leg==5U || (a->leg==3U && a->pass==S5_PASS_EXTRA)) &&
           s5_recovery_observe(&a->recovery,in->now_us,in->healthy,in->tape,in->junction,true)) {
            if(a->leg==3U) { a->leg=5U; }   /* rel: still the passing heading */
            a->issued=false;a->tail=1;a->phase_us=in->now_us;return action(S5_ACT_STOP,0,0);
        }
        if(a->issued && a->leg>=6U &&
           s5_recovery_observe(&a->recovery,in->now_us,in->healthy,in->tape,in->junction,true)) {
            /* Swerve: stop the moment the line is confirmed under the IR. */
            a->centre_dir=swerve_target(a)>a->rel?1U:0U;a->centre_tries=0;
            a->issued=false;a->state=S5_FINISH;a->phase_us=in->now_us;return action(S5_ACT_STOP,0,0);
        }
        if(a->issued) {
            if(in->motion_done) {
                /* Leg-3 drives after a turn away also move the car sideways
                 * (rel degrees off the passing heading): count it, so the
                 * return diagonal still ends on the original line. */
                if(a->leg==3U && a->rel>0 && a->pass!=S5_PASS_APPROACH) {
                    const double mm=(in->centre_m-a->leg_start_m)*1000.0*(double)a->rel*0.017453;
                    if(mm>0.0) {
                        const unsigned sum=a->lat_mm+(unsigned)(mm+0.5);
                        a->lat_mm=sum>511U?511U:sum;
                    }
                }
                a->issued=false;a->phase_us=in->now_us;
                /* Leg 3 ends only after the final clearance move. */
                if(a->leg==3U && a->pass!=S5_PASS_EXTRA) { a->pass=S5_PASS_PEEK;return action(S5_ACT_NONE,0,0); }
                /* Reverse finished: now the leg-0 turn. */
                if(a->leg==0U && a->backing) { a->backing=0;return action(S5_ACT_NONE,0,0); }
                if(a->leg==4U) { a->rel=-S5_BYPASS_DEG; }
                if(a->leg==5U) {
                    /* No line by the end (bend away): once more along the diagonal. */
                    if(S5_RETURN_EXTRA_MM>0U && !a->tail && !a->extended) { a->extended=1;return action(S5_ACT_NONE,0,0); }
                    /* Only tape seen by the swerve from here counts. */
                    s5_recovery_reset(&a->recovery);
                }
                if(a->leg>=6U) { a->rel=swerve_target(a); }
                if(a->leg==8U) { return fail(a,S5_NO_LINE); }   /* swerved both ways, back on heading */
                a->leg++;
            }
            return action(S5_ACT_NONE,0,0);
        }
        if(!in->stopped) { return action(S5_ACT_NONE,0,0); }
        if(!s5_range_ready(&in->observation,in->now_us) ||
           (int32_t)(in->observation.range_us-a->phase_us)<=0) {
            return elapsed>1000000U?fail(a,S5_BAD_RANGE):action(S5_ACT_NONE,0,0);
        }
        if(a->leg==0U && !a->backed) {
            /* Too close for the 45-degree out-leg: reverse the shortfall first. */
            unsigned back=diagonal_short(a,a->plan.offset_mm);
            a->backed=1;
            if(back>S5_BACKUP_MAX_MM && !a->swapped && a->plan.alt_ok &&
               diagonal_short(a,a->plan.alt_offset_mm)<=S5_BACKUP_MAX_MM) {
                /* Preferred side needs more room than reversing gives: still
                 * facing the obstacle, take the other side instead. */
                const unsigned other=a->plan.alt_offset_mm;
                a->swapped=1;a->plan.alt_offset_mm=a->plan.offset_mm;a->plan.offset_mm=other;
                a->plan.route=a->plan.route==S5_ROUTE_LEFT?S5_ROUTE_RIGHT:S5_ROUTE_LEFT;
                back=diagonal_short(a,a->plan.offset_mm);
            }
            if(back) {
                if(back>S5_BACKUP_MAX_MM) { return fail(a,S5_NO_PATH); }
                a->plan.closest_mm+=back;a->plan.forward_mm+=back;
                a->backing=1;a->issued=true;
                return action(S5_ACT_DRIVE,-(int)back,a->config.speed_mms);
            }
        }
        if(in->observation.range_mm<=a->config.emergency_mm) { return fail(a,S5_NO_PATH); }
        sign=a->plan.route==S5_ROUTE_LEFT?1:-1;
        if(a->leg==3U && a->pass==S5_PASS_PEEK) {
            a->pass=S5_PASS_PEEKING;a->phase_us=in->now_us;
            return action(S5_ACT_PEEK,(int)(a->plan.route==S5_ROUTE_LEFT?90U-S5_PEEK_DEG:90U+S5_PEEK_DEG),0);
        }
        if(a->leg==3U && a->pass==S5_PASS_STEP && a->side_turn) {
            /* Turn away from the obstacle side, then peek again. rel keeps the
             * heading vs the one at the obstacle, so the way back still aims
             * at the original line. */
            const int deg=(int)a->side_turn;
            a->side_turn=0;a->issued=true;a->pass=S5_PASS_APPROACH;a->rel+=deg;
            return action(S5_ACT_TURN,sign*deg,a->config.speed_mms);
        }
        if(straight) {
            unsigned mm=diagonal_mm(a->leg==5U?return_lat(a):a->plan.offset_mm);
            if(a->leg==5U && (a->tail || a->extended)) { mm=a->tail?S5_IR_AXLE_MM:S5_RETURN_EXTRA_MM; }
            if(a->leg==3U) {
                /* The diagonal out-leg already moved the car offset forward. */
                unsigned forward=a->plan.forward_mm,offset=a->plan.offset_mm;
                unsigned approach=forward>offset?forward-offset:0U;
                if(a->pass==S5_PASS_APPROACH && approach<S5_PEEK_STEP_MM) {
                    a->pass=S5_PASS_PEEK;return action(S5_ACT_NONE,0,0);   /* already at the face */
                }
                mm=a->pass==S5_PASS_APPROACH?approach:
                   (a->pass==S5_PASS_STEP?S5_PEEK_STEP_MM:pass_extra_mm(a));
            }
            if(!s5_range_ready(&in->observation,in->now_us) ||
               (int32_t)(in->observation.range_us-a->phase_us)<=0) {
                return elapsed>1000000U?fail(a,S5_BAD_RANGE):action(S5_ACT_NONE,0,0);
            }
            {   /* Leg 1 faces the diagonal path itself: it must be clear for
                 * the whole diagonal + margin (measured), and the forward sweep
                 * of the 45-degree leg must not reach the obstacle face. */
                unsigned need=S5_LEG_CLEAR_MM;
                bool blocked;
                if(a->leg==1U && mm+a->config.margin_mm>need) { need=mm+a->config.margin_mm; }
                blocked=in->observation.range_mm<=need || (a->leg==1U && !diagonal_clear(a,a->plan.offset_mm));
                if(blocked && a->leg==3U && a->side_turns<3U) {
                    /* Passing but the obstacle's side shows ahead: the car is
                     * converging on it. Turn away, then peek again. */
                    a->side_turns++;a->issued=true;a->pass=S5_PASS_APPROACH;a->rel+=(int)S5_SIDE_TURN_MAX_DEG;
                    return action(S5_ACT_TURN,sign*(int)S5_SIDE_TURN_MAX_DEG,a->config.speed_mms);
                }
                if(blocked && a->leg==1U && a->blocked_reads+1U<S5_BLOCK_CONFIRM) {
                    /* Wait for another fresh reading before believing it. */
                    a->blocked_reads++;a->phase_us=in->now_us;return action(S5_ACT_NONE,0,0);
                }
                if(!blocked) { a->blocked_reads=0; }
                if(blocked) {
                    a->blocked_reads=0;
                    /* First side blocked: turn 2 x 45 deg to face the other
                     * diagonal and re-check it as leg 1 (once). */
                    if(a->leg==1U && !a->swapped && a->plan.alt_ok) {
                        unsigned other=a->plan.alt_offset_mm;
                        a->swapped=1;a->plan.alt_offset_mm=a->plan.offset_mm;a->plan.offset_mm=other;
                        a->plan.route=a->plan.route==S5_ROUTE_LEFT?S5_ROUTE_RIGHT:S5_ROUTE_LEFT;
                        a->leg=0;a->issued=true;
                        return action(S5_ACT_TURN,a->plan.route==S5_ROUTE_LEFT?2*S5_BYPASS_DEG:-2*S5_BYPASS_DEG,
                                      a->config.speed_mms);
                    }
                    return fail(a,S5_NO_PATH);
                }
            }
            a->issued=true;a->leg_start_m=in->centre_m;
            if(a->leg==3U && a->pass==S5_PASS_STEP) { a->searched_mm=(uint16_t)(a->searched_mm+mm); }
            return action(S5_ACT_DRIVE,(int)mm,a->config.speed_mms);
        }
        a->issued=true;
        if(a->leg==0U) { return action(S5_ACT_TURN,sign*S5_BYPASS_DEG,a->config.speed_mms); }
        if(a->leg==2U) { return action(S5_ACT_TURN,-sign*S5_BYPASS_DEG,a->config.speed_mms); }
        /* Turn in to 45 degrees off the passing heading (less any turn away). */
        if(a->leg==4U) { return action(S5_ACT_TURN,-sign*(S5_BYPASS_DEG+a->rel),a->config.speed_mms); }
        /* Swerve (legs 6..8), each turn stopped as soon as the line is seen:
         * toward the bypass side up to S5_SWERVE_OUT_DEG past the heading at
         * the obstacle, then the other way up to S5_SWERVE_IN_DEG past it,
         * then back to it. Never more than 90 deg either side, so the car
         * cannot pick up the line behind it and follow it backwards. */
        return action(S5_ACT_TURN,sign*(swerve_target(a)-a->rel),a->config.speed_mms);
    } else if(a->state==S5_FINISH) {
        /* Hand back only with the tape under the IR now. Braking can carry
         * the IR past the edge it stopped on: then turn back slowly (the way
         * it came, then the other way), stopping on the tape. */
        const int side=a->plan.route==S5_ROUTE_LEFT?1:-1, dir=a->centre_dir?side:-side;
        if(elapsed>3000000U) { return fail(a,S5_NO_LINE); }
        if(a->issued) {
            if(in->tape && !in->junction) { a->issued=false;a->phase_us=in->now_us;return action(S5_ACT_STOP,0,0); }
            if(in->motion_done) { a->issued=false;a->phase_us=in->now_us; }
            return action(S5_ACT_NONE,0,0);
        }
        if(!in->stopped) { return action(S5_ACT_NONE,0,0); }
        if(in->tape && in->line_ready && !in->junction) {
            a->state=S5_COMPLETE;return action(S5_ACT_RESUME,0,0);
        }
        if(a->centre_tries>=2U) { return fail(a,S5_NO_LINE); }
        a->issued=true;a->phase_us=in->now_us;
        a->centre_tries++;
        return action(S5_ACT_TURN,(a->centre_tries==1U?-dir:dir)*(a->centre_tries==1U?S5_CENTRE_DEG:2*S5_CENTRE_DEG),
                      S5_CENTRE_SPEED_MMS);
    }
    return action(S5_ACT_NONE,0,0);
}
