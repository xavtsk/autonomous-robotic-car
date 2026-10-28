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
}
void s5_avoidance_abort(s5_avoidance_t *a)
{ a->state=S5_IDLE;a->issued=false;s5_recovery_reset(&a->recovery); }
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
        if(in->observation.scan_done && (int32_t)(in->observation.scan_us-a->phase_us)>0) {
            if(!in->observation.profile.valid) { return fail(a,S5_SCAN_FAILED); }
            if(!s5_plan(&in->observation.profile,&a->config,&a->plan)) { return fail(a,S5_NO_PATH); }
            a->state=S5_MOVING;a->phase_us=in->now_us;
        }
    } else if(a->state==S5_MOVING) {
        bool straight=(a->leg==1U || a->leg==3U || a->leg==5U);
        if(elapsed>20000000U) { return fail(a,S5_DEADLINE); }
        /* Each straight leg requires a new forward reading after its turn.
         * During movement, loss of valid ranging or a close return brakes. */
        if(a->issued && (!s5_range_ready(&in->observation,in->now_us) ||
            in->observation.range_mm<=a->config.emergency_mm)) { return fail(a,S5_BAD_RANGE); }
        if(a->issued && a->leg==5U) {
            double progressed=(in->centre_m-a->leg_start_m)*1000.0;
            double error=progressed+a->config.sensor_axle_mm-a->plan.offset_mm;
            bool corridor=error>=-(double)a->config.recovery_gate_mm && error<=(double)a->config.recovery_gate_mm;
            (void)s5_recovery_observe(&a->recovery,in->now_us,in->healthy,in->tape,in->junction,corridor);
        } else if(a->issued && a->leg>=7U) {
            (void)s5_recovery_observe(&a->recovery,in->now_us,in->healthy,in->tape,in->junction,true);
        }
        if(a->issued) {
            if(in->motion_done) {
                a->issued=false;a->leg++;a->phase_us=in->now_us;
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
        if(straight) {
            unsigned mm=a->leg==3U?a->plan.forward_mm:a->plan.offset_mm;
            if(!s5_range_ready(&in->observation,in->now_us) ||
               (int32_t)(in->observation.range_us-a->phase_us)<=0) {
                return elapsed>1000000U?fail(a,S5_BAD_RANGE):action(S5_ACT_NONE,0,0);
            }
            if(in->observation.range_mm<=a->config.detect_mm) { return fail(a,S5_NO_PATH); }
            a->issued=true;a->leg_start_m=in->centre_m;
            return action(S5_ACT_DRIVE,(int)mm,a->config.speed_mms);
        }
        a->issued=true;
        if(a->leg==0U || a->leg==6U) { return action(S5_ACT_TURN,sign*90,a->config.speed_mms); }
        if(a->leg==2U || a->leg==4U) { return action(S5_ACT_TURN,-sign*90,a->config.speed_mms); }
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
