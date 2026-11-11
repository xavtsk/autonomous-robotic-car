#include "obstacle_avoidance.h"
static s5_action_t action(s5_action_kind_t kind,int value,unsigned speed)
{ return (s5_action_t){kind,(int16_t)value,(uint16_t)speed}; }
static s5_action_t fail(s5_avoidance_t *a,s5_fault_t reason)
{ a->state=S5_FAULT;a->fault=reason;return action(S5_ACT_STOP,0,0); }
void s5_avoidance_init(s5_avoidance_t *a,const s5_config_t *c)
{ *a=(s5_avoidance_t){0};a->config=*c; }
bool s5_range_ready(const s5_observation_t *o,uint32_t now)
{
    /* The controller timestamps its ADC frame before reading the concurrent
     * sonar publication. That publication may be a few milliseconds newer.
     * Permit only one bounded frame of this skew; never refresh an old echo. */
    if(!o || !o->range_valid || o->no_echo || o->range_mm<30U || o->range_mm>4000U) { return false; }
    int32_t age=(int32_t)(now-o->range_us);
    return age>=-30000 && age<=(int32_t)S5_RANGE_FRESH_US;
}
void s5_avoidance_begin(s5_avoidance_t *a,uint32_t now)
{
    a->state=S5_WAIT_STOP;a->fault=S5_OK;a->phase_us=a->started_us=now;
    a->leg=0;a->issued=false;a->plan=(s5_plan_t){0};s5_recovery_reset(&a->recovery);
    a->pass=S5_PASS_APPROACH;a->searched_mm=0;a->swapped=0;
    a->switching=0;a->clear_count=0;
    a->bypass_offset_mm=a->return_mm=a->clear_at_m=a->pass_start_m=0.0;
}
void s5_avoidance_abort(s5_avoidance_t *a)
{ a->state=S5_IDLE;a->issued=false;s5_recovery_reset(&a->recovery); }
/* A 60-degree peek has a configured +/-15-degree bearing envelope. Its most
 * forward ray is 45 degrees, so the full obstacle cross-section can extend
 * peek_span ahead of the sonar when it first reads clear. Put the complete
 * inward-turn corner clearance past that upper bound, using actual travel since
 * that observation. This is a conservative bound, not an exact hidden edge. */
static unsigned pass_extra_mm(const s5_avoidance_t *a,const s5_input_t *in)
{
    double remaining=(double)a->plan.peek_span_mm+a->config.sonar_axle_mm+
        a->plan.rear_clear_mm-(in->centre_m-a->clear_at_m)*1000.0;
    return remaining>0.0?(unsigned)(remaining+0.999):0U;
}
static s5_action_t peek_result(s5_avoidance_t *a,const s5_input_t *in,unsigned elapsed)
{
    const s5_profile_t *p=&in->observation.profile;
    unsigned side,limit;
    if(elapsed>S5_SCAN_TIMEOUT_US) { return fail(a,S5_DEADLINE); }
    if(!in->observation.scan_done || (int32_t)(in->observation.scan_us-a->phase_us)<=0) { return action(S5_ACT_NONE,0,0); }
    if(!p->valid) { return fail(a,S5_SCAN_FAILED); }
    /* Obstacle on the car's right after a LEFT bypass, so look right (30 deg). */
    side=a->plan.route==S5_ROUTE_LEFT?p->right_clear_mm:p->left_clear_mm;
    /* Cover the FAR side of the measured obstacle, including the beam's
     * 45-degree boundary. A centre-line-only test can miss a wide rear face. */
    limit=(a->plan.peek_span_mm*1415U+999U)/1000U;
    if(side<S5_RANGE_MIN_MM || side>S5_RANGE_MAX_MM) { return fail(a,S5_SCAN_FAILED); }
    a->phase_us=in->now_us;
    /* Early side peeks can look through empty floor BEFORE the front face.
     * Only after the sonar has passed the entire observed front bound can
     * a clear ray constrain the hidden rear. Use measured straight travel. */
    bool past_front=(in->centre_m-a->pass_start_m)*1000.0>=a->plan.clear_after_mm;
    if(side<=limit || !past_front) {
        a->clear_count=0;
        if(a->searched_mm+S5_PEEK_STEP_MM>a->config.max_depth_mm) { return fail(a,S5_NO_PATH); }
        a->pass=S5_PASS_STEP;
    } else {
        if(!a->clear_count) { a->clear_at_m=in->centre_m; }
        a->clear_count++;
        /* Separate fresh scan tokens; never reuse a single echo as both
         * confirmations. Recentring and a new forward ping precede each. */
        a->pass=a->clear_count>=2U?S5_PASS_EXTRA:S5_PASS_PEEK;
    }
    return action(S5_ACT_NONE,0,0);
}
s5_action_t s5_avoidance_step(s5_avoidance_t *a,const s5_input_t *in)
{
    unsigned elapsed; int sign;
    if(!a || !in) { return action(S5_ACT_STOP,0,0); }
    if(a->state==S5_IDLE || a->state==S5_COMPLETE || a->state==S5_FAULT) { return action(S5_ACT_NONE,0,0); }
    if(!in->healthy) { return fail(a,S5_MOTION_ERROR); }
    if((uint32_t)(in->now_us-a->started_us)>120000000U) { return fail(a,S5_DEADLINE); }
    elapsed=in->now_us-a->phase_us;
    if(a->state==S5_WAIT_STOP) {
        if(elapsed>1000000U) { return fail(a,S5_DEADLINE); }
        if(in->stopped) { a->state=S5_SCANNING;a->phase_us=in->now_us;return action(S5_ACT_SCAN,0,0); }
    } else if(a->state==S5_SCANNING) {
        if(elapsed>S5_SCAN_TIMEOUT_US) { return fail(a,S5_DEADLINE); }
        if(in->observation.scan_done && (int32_t)(in->observation.scan_us-a->phase_us)>0) {
            if(!in->observation.profile.valid) { return fail(a,S5_SCAN_FAILED); }
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
        if(a->issued && (!s5_range_ready(&in->observation,in->now_us) ||
            in->observation.range_mm<=a->config.emergency_mm)) { return fail(a,S5_BAD_RANGE); }
        if(a->issued && a->leg==5U) {
            double progressed=(in->centre_m-a->leg_start_m)*1000.0;
            double error=(progressed+a->config.sensor_axle_mm)*0.7071067812-a->bypass_offset_mm;
            bool corridor=error>=-(double)a->config.recovery_gate_mm && error<=(double)a->config.recovery_gate_mm;
            (void)s5_recovery_observe(&a->recovery,in->now_us,in->healthy,in->tape,in->junction,corridor);
        } else if(a->issued && a->leg>=7U) {
            (void)s5_recovery_observe(&a->recovery,in->now_us,in->healthy,in->tape,in->junction,true);
        }
        if(a->issued) {
            if(in->motion_done) {
                a->issued=false;a->phase_us=in->now_us;
                /* A fallback uses two separately checked 45-degree turns:
                 * first restore forward, then turn toward the other side. */
                if(a->switching==1U) { a->switching=2U;return action(S5_ACT_NONE,0,0); }
                if(a->switching==2U) { a->switching=0; }
                if(a->leg==1U) {
                    double travelled=(in->centre_m-a->leg_start_m)*1000.0;
                    double tolerance=(double)a->config.margin_mm*0.5;
                    if(!(travelled>0.0 && travelled>=(double)a->plan.diagonal_mm-tolerance &&
                         travelled<=(double)a->plan.diagonal_mm+tolerance)) { return fail(a,S5_MOTION_ERROR); }
                    a->return_mm=travelled;
                    a->bypass_offset_mm=travelled*0.7071067812;
                    if(a->bypass_offset_mm>a->plan.offset_mm) {
                        unsigned span=a->plan.peek_span_mm+(unsigned)(a->bypass_offset_mm-a->plan.offset_mm+0.999);
                        if(span>4095U) { return fail(a,S5_MOTION_ERROR); }
                        a->plan.peek_span_mm=span;
                    } else {
                        /* A shorter lateral offset needs more clearance at
                         * the rear corner before starting the inward turn. */
                        unsigned shortfall=(unsigned)(a->plan.offset_mm-a->bypass_offset_mm+0.999);
                        unsigned rear=a->plan.rear_clear_mm+shortfall;
                        unsigned gate=a->plan.clear_after_mm+shortfall;
                        if(rear>4095U || gate>4095U) { return fail(a,S5_MOTION_ERROR); }
                        a->plan.rear_clear_mm=rear;
                        a->plan.clear_after_mm=gate;
                    }
                }
                if(a->leg==2U) { a->pass_start_m=in->centre_m; }
                if(a->leg==3U && a->pass==S5_PASS_STEP) {
                    double travelled=(in->centre_m-a->leg_start_m)*1000.0;
                    double tolerance=(double)a->config.margin_mm*0.5;
                    if(!(travelled>=S5_PEEK_STEP_MM-tolerance && travelled<=S5_PEEK_STEP_MM+tolerance)) { return fail(a,S5_MOTION_ERROR); }
                    unsigned searched=a->searched_mm+(unsigned)(travelled+0.5);
                    if(searched>a->config.max_depth_mm) { return fail(a,S5_NO_PATH); }
                    a->searched_mm=(uint16_t)searched;
                }
                if(a->leg==3U && a->pass==S5_PASS_EXTRA &&
                   pass_extra_mm(a,in)>a->config.margin_mm/2U) { return fail(a,S5_MOTION_ERROR); }
                /* Leg 3 ends only after the final clearance move. */
                if(a->leg==3U && a->pass!=S5_PASS_EXTRA) { a->pass=S5_PASS_PEEK;return action(S5_ACT_NONE,0,0); }
                a->leg++;
                if(a->leg==7U && a->recovery.confirmed) { a->state=S5_FINISH;return action(S5_ACT_STOP,0,0); }
                if(a->leg==10U) {
                    if(!a->recovery.confirmed) { return fail(a,S5_NO_LINE); }
                    a->state=S5_FINISH;return action(S5_ACT_STOP,0,0);
                }
            }
            return action(S5_ACT_NONE,0,0);
        }
        if(!in->stopped) { return action(S5_ACT_NONE,0,0); }
        if(!s5_range_ready(&in->observation,in->now_us) ||
           (int32_t)(in->observation.range_us-a->phase_us)<=0) {
            return elapsed>1000000U?fail(a,S5_BAD_RANGE):action(S5_ACT_NONE,0,0);
        }
        if(in->observation.range_mm<=a->config.emergency_mm) { return fail(a,S5_NO_PATH); }
        sign=a->plan.route==S5_ROUTE_LEFT?1:-1;
        if(a->leg==3U && a->pass==S5_PASS_PEEK) {
            a->pass=S5_PASS_PEEKING;a->phase_us=in->now_us;
            return action(S5_ACT_PEEK,(int)(a->plan.route==S5_ROUTE_LEFT?90U-S5_PEEK_DEG:90U+S5_PEEK_DEG),0);
        }
        if(straight) {
            unsigned mm=a->leg==5U?(unsigned)(a->return_mm+0.5):a->plan.diagonal_mm;
            if(a->leg==3U) {
                mm=a->pass==S5_PASS_APPROACH?a->plan.forward_mm:
                   (a->pass==S5_PASS_STEP?S5_PEEK_STEP_MM:pass_extra_mm(a,in));
                if(a->pass==S5_PASS_APPROACH && mm) {
                    double corrected=(double)mm+a->plan.offset_mm-a->bypass_offset_mm;
                    mm=corrected>0.0?(unsigned)(corrected+0.999):0U;
                }
                if(!mm) {
                    if(a->pass==S5_PASS_EXTRA) { a->leg++; }
                    else { a->pass=S5_PASS_PEEK; }
                    a->phase_us=in->now_us;return action(S5_ACT_NONE,0,0);
                }
            }
            if(!s5_range_ready(&in->observation,in->now_us) ||
               (int32_t)(in->observation.range_us-a->phase_us)<=0) {
                return elapsed>1000000U?fail(a,S5_BAD_RANGE):action(S5_ACT_NONE,0,0);
            }
            {   /* Check the whole proposed straight move from the sonar face,
                 * including the body ahead of it, after each heading change. */
                unsigned body_front=a->config.front_axle_mm;
                if(a->config.sensor_axle_mm>body_front) { body_front=a->config.sensor_axle_mm; }
                unsigned front=body_front>a->config.sonar_axle_mm?
                    body_front-a->config.sonar_axle_mm:0U;
                unsigned need=mm+front+a->config.margin_mm;
                if(need<a->config.emergency_mm) { need=a->config.emergency_mm; }
                if(in->observation.range_mm<=need) {
                    /* First route blocked before translation: restore the
                     * original heading, then try the prevalidated other side. */
                    if(a->leg==1U && !a->swapped && a->plan.alt_ok) {
                        a->swapped=1;a->plan.offset_mm=a->plan.alt_offset_mm;
                        a->plan.diagonal_mm=a->plan.alt_diagonal_mm;
                        a->plan.forward_mm=a->plan.alt_forward_mm;
                        a->plan.peek_span_mm=a->plan.alt_peek_span_mm;
                        a->plan.clear_after_mm=a->plan.alt_clear_after_mm;
                        a->plan.alt_ok=0;
                        a->plan.route=a->plan.route==S5_ROUTE_LEFT?S5_ROUTE_RIGHT:S5_ROUTE_LEFT;
                        a->leg=0;a->issued=true;a->switching=1;a->phase_us=in->now_us;
                        return action(S5_ACT_TURN,a->plan.route==S5_ROUTE_LEFT?45:-45,a->config.speed_mms);
                    }
                    return fail(a,S5_NO_PATH);
                }
            }
            a->issued=true;a->leg_start_m=in->centre_m;
            return action(S5_ACT_DRIVE,(int)mm,a->config.speed_mms);
        }
        a->issued=true;
        if(a->leg==0U || a->leg==6U) { return action(S5_ACT_TURN,sign*45,a->config.speed_mms); }
        if(a->leg==2U || a->leg==4U) { return action(S5_ACT_TURN,-sign*45,a->config.speed_mms); }
        /* Bounded search at the predicted original corridor; returns to the
         * original heading before handing back. No unbounded blind reversing. */
        return action(S5_ACT_TURN,a->leg==8U?-40:20,a->config.speed_mms);
    } else if(a->state==S5_FINISH) {
        if(elapsed>1000000U) { return fail(a,S5_NO_LINE); }
        if(in->stopped && in->line_ready && !in->junction && a->recovery.confirmed) {
            a->state=S5_COMPLETE;return action(S5_ACT_RESUME,0,0);
        }
    }
    return action(S5_ACT_NONE,0,0);
}
