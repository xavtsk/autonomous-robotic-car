#include "terrain_monitor.h"
#include "tilt_estimation.h"
#if APP_IMU_DEBUG
#include <float.h>
#endif
#include <math.h>
#include <string.h>

#define SAFETY_MAX_SAMPLE_GAP_US 50000U
#define SAFETY_LIFT_CONFIRM_US 40000U
#define SAFETY_RECOVERY_US 300000U

static void safety_update(terrain_monitor_t *t,const float a[3],float norm,
                          uint32_t now,bool continuous)
{
    bool near_gravity=norm>=0.8f && norm<=1.2f;
    bool tipped=norm>=0.7f && norm<=1.35f && a[2]<=0.7071068f*norm; /* 45 deg */
    bool recovering=t->safety!=TERRAIN_SAFETY_NONE;
    bool settled=norm>=0.9f && norm<=1.1f && a[2]>=0.9781476f*norm; /* 12 deg */
    /* Use the raw calibrated vector rather than the filtered pitch/roll values.
     * Those values lag and are unavailable when the vehicle is upside down. */
    bool lift_tilt=near_gravity && a[2]<=0.9063078f*norm; /* 25 deg */
    const float horizontal_sq = a[0]*a[0] + a[1]*a[1];
    const float upward_excess = a[2] - 1.0f;
    const bool gentle_pickup = a[2] >= 1.15f && horizontal_sq <= 0.15f*0.15f;
    /* For a firm pickup, upward acceleration above 1 g must be at least
     * twice the horizontal acceleration. Compare squared magnitudes. */
    const bool firm_pickup = a[2] >= 1.30f && upward_excess*upward_excess >= 4.0f*horizontal_sq;
    const bool upward = gentle_pickup || firm_pickup;
#if APP_IMU_DEBUG
    t->safety_upward=upward;
    t->safety_lift_tilt=lift_tilt;
#endif
    if(!continuous) { t->safety_tracking=false; }
    if(tipped || norm<0.3f) {
        if(tipped) { t->safety=TERRAIN_SAFETY_OVERTURNED; }
        else if(!recovering) { t->safety=TERRAIN_SAFETY_LIFTED; }
        t->safety_tracking=false;
    } else if(!(recovering?settled:(lift_tilt || upward))) {
        t->safety_tracking=false;
    } else if(!t->safety_tracking) {
        t->safety_tracking=true; t->safety_since_us=now;
    } else if((uint32_t)(now-t->safety_since_us)>=(recovering?SAFETY_RECOVERY_US:SAFETY_LIFT_CONFIRM_US)) {
        t->safety=recovering?TERRAIN_SAFETY_NONE:TERRAIN_SAFETY_LIFTED;
        t->safety_tracking=false;
    }
}

void terrain_monitor_init(terrain_monitor_t *t)
{
    *t=(terrain_monitor_t){0}; t->mag_scale[0]=t->mag_scale[1]=t->mag_scale[2]=1;
}
void terrain_monitor_reset_run(terrain_monitor_t *t)
{
    hump_reset(&t->hump); t->seen=false; t->odometry_seen=false; t->heading_seen=false;
    t->stable_count=0; t->safety_tracking=false;
    t->result=(terrain_result_t){.safety=t->safety};
}
bool terrain_monitor_set_hump_geometry(terrain_monitor_t *t,
                                      terrain_hump_geometry_t geometry,float track_m)
{
    if(!t || (geometry!=TERRAIN_HUMP_LONGITUDINAL && geometry!=TERRAIN_HUMP_RIGHT_WHEEL) ||
       (geometry==TERRAIN_HUMP_RIGHT_WHEEL && (!isfinite(track_m) || track_m<=0 || track_m>0.5f))) {
        return false;
    }
    t->hump_geometry=geometry; t->wheel_track_m=track_m;
    terrain_monitor_reset_run(t);
    return true;
}
bool terrain_monitor_set_magnetic_calibration(terrain_monitor_t *t,const float offset[3],const float scale[3])
{
    unsigned i;
    if(!t || !offset || !scale) { return false; }
    for(i=0;i<3;++i) {
        if(!isfinite(offset[i]) || !isfinite(scale[i]) || fabsf(offset[i])>5 || scale[i]<0.1f || scale[i]>10) { return false; }
    }
    memcpy(t->mag_offset,offset,sizeof(t->mag_offset)); memcpy(t->mag_scale,scale,sizeof(t->mag_scale));
    t->magnetic_calibrated=true; t->heading_seen=false; return true;
}
void terrain_monitor_process(terrain_monitor_t *t,const imu_sample_t *s,const terrain_odometry_t *odo)
{
    float a[3],m[3],pitch=0,roll=0,heading,ds=0,speed=0,dt=0;
    float n, dynamic_g;
    uint32_t gap=t->seen?s->timestamp_us-t->previous_us:0;
    bool valid=imu_sample_is_valid(s), tilt_ok=false, movement_ok=false;
    /* Measure continuity from sample timestamps. A sensor overrun can still
     * yield a valid current sample. */
    bool continuity=t->seen && gap>0 && gap<=100000U;
    unsigned i;
    terrain_result_t r={.timestamp_us=s->timestamp_us,.safety=t->safety};
#if APP_IMU_DEBUG
    hump_reason_t odometry_reason=HUMP_REASON_ODOMETRY_UNAVAILABLE;
    t->hump.debug=(hump_debug_t){0};
#endif
    if(t->hump.highest_valid) { r.peak_m=t->hump.highest_m; r.flags|=TERRAIN_PEAK_VALID; }
    if(!valid || (t->seen && !continuity)) {
        r.flags|=TERRAIN_FAULT; t->heading_seen=false; t->odometry_seen=false;
        t->safety_tracking=false;
        if(!t->calibration.ready) { imu_calibration_reset(&t->calibration); }
        if(t->hump_geometry==TERRAIN_HUMP_RIGHT_WHEEL) {
            hump_update_right_wheel(&t->hump,0,0,t->wheel_track_m,0,HUMP_SAMPLE_INVALID,s->timestamp_us);
#if APP_IMU_DEBUG
            t->hump.debug.reason=valid ? HUMP_REASON_IMU_GAP : HUMP_REASON_INVALID_IMU;
#endif
        } else { hump_update(&t->hump,0,0,false,s->timestamp_us); }
        t->seen=false; t->result=r; return;
    }
    if(!t->calibration.ready) {
        imu_calibration_feed(&t->calibration,s,odo && odo->stationary);
        t->previous_us=s->timestamp_us; t->seen=!t->calibration.ready;
#if APP_IMU_DEBUG
        t->hump.debug.reason=HUMP_REASON_CALIBRATION_PENDING;
#endif
        t->result=r; return;
    }
    r.flags|=TERRAIN_CALIBRATED;
    for(i=0;i<3;++i) {
        a[i]=s->accel_g[i]-t->calibration.bias_g[i];
        t->filtered[i]=continuity?t->filtered[i]+0.25f*(a[i]-t->filtered[i]):a[i];
        m[i]=(s->mag_gauss[i]-t->mag_offset[i])*t->mag_scale[i];
    }
    n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    safety_update(t,a,n,s->timestamp_us,continuity && gap<=SAFETY_MAX_SAMPLE_GAP_US);
    r.safety=t->safety;
    r.impact=isfinite(n) && (n>2.0f || n<0.3f);
    tilt_ok=n>=0.9f && n<=1.1f && tilt_estimation(t->filtered,&pitch,&roll);
    if(tilt_ok) { r.pitch_deg=pitch; r.roll_deg=roll; r.flags|=TERRAIN_TILT_VALID; }
    if(tilt_ok && s->mag_valid && t->magnetic_calibrated && tilt_heading(t->filtered,m,&heading)) {
        uint32_t elapsed=s->timestamp_us-t->heading_us;
        if(t->heading_seen && elapsed>=10000 && elapsed<=200000) {
            float rate=tilt_angle_delta(heading,t->heading)*1000000.0f/elapsed;
            if(fabsf(rate)<=360) { r.turn_dps=rate; r.flags|=TERRAIN_TURN_VALID; }
        }
        t->heading=heading; t->heading_us=s->timestamp_us; t->heading_seen=true;
    } else if(s->mag_valid || !tilt_ok || (uint32_t)(s->timestamp_us-t->heading_us)>100000) {
        t->heading_seen=false;
    } else if(t->heading_seen && (t->result.flags&TERRAIN_TURN_VALID)) {
        r.turn_dps=t->result.turn_dps; r.flags|=TERRAIN_TURN_VALID;
    }
    if(odo && odo->valid && isfinite(odo->distance_m)) {
#if APP_IMU_DEBUG
        odometry_reason=HUMP_REASON_ODOMETRY_BASELINE;
#endif
        if(t->odometry_seen && continuity && odo->epoch==t->epoch) {
            double delta=odo->distance_m-t->previous_distance;
#if APP_IMU_DEBUG
            odometry_reason=HUMP_REASON_ODOMETRY_STEP_OUT_OF_RANGE;
            if(isfinite(delta) && fabs(delta)<=FLT_MAX) {
                t->hump.debug.wheel_delta_m=(float)delta;
                t->hump.debug.wheel_delta_valid=true;
            }
#endif
            dt=gap/1000000.0f;
            if(fabs(delta)<=0.05) { ds=(float)delta; speed=ds/dt; movement_ok=fabsf(speed)<=1.5f; }
        }
        t->previous_distance=odo->distance_m; t->epoch=odo->epoch; t->odometry_seen=true;
    } else { t->odometry_seen=false; }
    dynamic_g=a[0]-t->filtered[0];
    /* Acceleration alone cannot distinguish slope from sustained acceleration.
     * Reject obvious dynamics/roll/turns; known-height testing remains required. */
    {
        bool hump_valid=tilt_ok && movement_ok && !r.impact && r.safety==TERRAIN_SAFETY_NONE &&
            (!(r.flags&TERRAIN_TURN_VALID) || fabsf(r.turn_dps)<15);
        bool forward_stable=fabsf(dynamic_g)<0.08f;
        if(t->hump_geometry==TERRAIN_HUMP_RIGHT_WHEEL) {
            /* Lateral acceleration can imitate a raised right wheel. */
            hump_sample_quality_t quality=HUMP_SAMPLE_INVALID;
            if(hump_valid) {
                quality=forward_stable && fabsf(a[1]-t->filtered[1])<0.08f ?
                    HUMP_SAMPLE_VALID : HUMP_SAMPLE_ACCELERATION_SPIKE;
            }
            hump_update_right_wheel(&t->hump,pitch,roll,t->wheel_track_m,ds,quality,s->timestamp_us);
#if APP_IMU_DEBUG
            if(quality==HUMP_SAMPLE_INVALID) {
                if(r.impact || r.safety!=TERRAIN_SAFETY_NONE) {
                    t->hump.debug.reason=HUMP_REASON_SAFETY_OR_IMPACT;
                } else if(!tilt_ok) {
                    t->hump.debug.reason=HUMP_REASON_INVALID_TILT;
                } else if(!movement_ok) {
                    t->hump.debug.reason=(uint8_t)odometry_reason;
                } else {
                    t->hump.debug.reason=HUMP_REASON_TURN_RATE;
                }
            }
#endif
        } else {
            hump_update(&t->hump,pitch,ds,hump_valid && forward_stable && fabsf(roll)<10,s->timestamp_us);
        }
    }
    if(t->hump.highest_valid) { r.peak_m=t->hump.highest_m; r.flags|=TERRAIN_PEAK_VALID; }
    r.hump_completed=t->hump.completed;
    /* Live climb: height so far on the hump being crossed (not yet a peak). */
    r.hump_active=t->hump.active;
    if(r.hump_active) { r.hump_height_m=t->hump.height_m<0?0.0f:(t->hump.height_m>0.5f?0.5f:t->hump.height_m); }
    if(r.impact) { r.state=TERRAIN_IMPACT; t->stable_count=0; }
    else if(tilt_ok && movement_ok) {
        terrain_state_t candidate=motion_classify(pitch,speed,dynamic_g,r.turn_dps,(r.flags&TERRAIN_TURN_VALID)!=0,false);
        if(candidate!=t->candidate) { t->candidate=candidate; t->stable_count=1; }
        else if(t->stable_count<3) { ++t->stable_count; }
        r.state=t->stable_count>=3?candidate:t->result.state;
        if(r.state==TERRAIN_IMPACT) { r.state=TERRAIN_UNKNOWN; }
    } else { t->stable_count=0; r.state=TERRAIN_UNKNOWN; }
    t->previous_us=s->timestamp_us; t->seen=true; t->result=r;
}
static void put_bits(uint8_t *out,unsigned *bit,uint32_t value,unsigned width)
{
    unsigned i;
    for(i=0;i<width;++i,++*bit) { if(value&(1UL<<i)) { out[*bit/8]|=(uint8_t)(1U<<(*bit%8)); } }
}
void terrain_pack(const terrain_result_t *r,uint8_t out[TERRAIN_PACKET_SIZE])
{
    unsigned bit=0;
    uint8_t flags=r->flags;
    int32_t pitch=0,roll=0,rate=0; uint32_t height=0,climb=0;
    bool active=r->hump_active && isfinite(r->hump_height_m) && r->hump_height_m>=0 && r->hump_height_m<=0.5f;
    memset(out,0,TERRAIN_PACKET_SIZE);
    if(!isfinite(r->pitch_deg)||!isfinite(r->roll_deg)||fabsf(r->pitch_deg)>90||fabsf(r->roll_deg)>90) { flags&=(uint8_t)~TERRAIN_TILT_VALID; }
    if(!isfinite(r->turn_dps)||fabsf(r->turn_dps)>360) { flags&=(uint8_t)~TERRAIN_TURN_VALID; }
    if(!isfinite(r->peak_m)||r->peak_m<0||r->peak_m>0.5f) { flags&=(uint8_t)~TERRAIN_PEAK_VALID; }
    if(flags&TERRAIN_TILT_VALID) { pitch=(int32_t)lroundf(r->pitch_deg*10); roll=(int32_t)lroundf(r->roll_deg*10); }
    if(flags&TERRAIN_TURN_VALID) { rate=(int32_t)lroundf(r->turn_dps); }
    if(flags&TERRAIN_PEAK_VALID) { height=(uint32_t)lroundf(r->peak_m*1000); }
    if(active) { climb=(uint32_t)lroundf(r->hump_height_m*1000); }
    put_bits(out,&bit,r->timestamp_us,32); put_bits(out,&bit,(uint32_t)r->state,3); put_bits(out,&bit,flags,5);
    put_bits(out,&bit,(uint32_t)pitch,11); put_bits(out,&bit,(uint32_t)roll,11);
    put_bits(out,&bit,(uint32_t)rate,10); put_bits(out,&bit,height,9);
    put_bits(out,&bit,r->hump_completed,1); put_bits(out,&bit,r->impact,1);
    put_bits(out,&bit,active,1); put_bits(out,&bit,climb,9);
    put_bits(out,&bit,(uint32_t)r->safety,2);
}

static uint32_t get_bits(const uint8_t *p,unsigned *bit,unsigned width)
{
    uint32_t v=0; unsigned i;
    for(i=0;i<width;++i,++*bit) { v|=(uint32_t)((p[*bit/8]>>(*bit%8))&1U)<<i; }
    return v;
}
static int32_t signed_value(uint32_t v,unsigned width)
{ return (v&(1U<<(width-1U)))?(int32_t)v-(int32_t)(1U<<width):(int32_t)v; }
void terrain_unpack(const uint8_t in[TERRAIN_PACKET_SIZE],terrain_result_t *r)
{
    unsigned bit=0; *r=(terrain_result_t){0};
    r->timestamp_us=get_bits(in,&bit,32); r->state=(terrain_state_t)get_bits(in,&bit,3);
    r->flags=(uint8_t)get_bits(in,&bit,5);
    r->pitch_deg=signed_value(get_bits(in,&bit,11),11)/10.0f;
    r->roll_deg=signed_value(get_bits(in,&bit,11),11)/10.0f;
    r->turn_dps=(float)signed_value(get_bits(in,&bit,10),10); r->peak_m=get_bits(in,&bit,9)/1000.0f;
    r->hump_completed=get_bits(in,&bit,1)!=0; r->impact=get_bits(in,&bit,1)!=0;
    r->hump_active=get_bits(in,&bit,1)!=0; r->hump_height_m=get_bits(in,&bit,9)/1000.0f;
    r->safety=(terrain_safety_t)get_bits(in,&bit,2);
}
