#include "terrain_monitor.h"
#include "tilt_estimation.h"
#include <math.h>
#include <string.h>
void terrain_monitor_init(terrain_monitor_t *t)
{
    *t=(terrain_monitor_t){0}; t->mag_scale[0]=t->mag_scale[1]=t->mag_scale[2]=1;
}
void terrain_monitor_reset_run(terrain_monitor_t *t)
{
    hump_reset(&t->hump); t->seen=false; t->odometry_seen=false; t->heading_seen=false;
    t->stable_count=0; t->result=(terrain_result_t){0};
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
    /* The real gap check covers missed samples. The overrun bit fires whenever the
     * ~22 ms loop (10 ms kernel tick) skips one 20 ms sample; it is not a fault. */
    bool continuity=t->seen && gap>0 && gap<=100000U;
    unsigned i;
    terrain_result_t r={0}; r.timestamp_us=s->timestamp_us;
    if(t->hump.highest_valid) { r.peak_m=t->hump.highest_m; r.flags|=TERRAIN_PEAK_VALID; }
    if(!valid || (t->seen && !continuity)) {
        r.flags|=TERRAIN_FAULT; t->heading_seen=false; t->odometry_seen=false;
        if(!t->calibration.ready) { imu_calibration_reset(&t->calibration); }
        hump_update(&t->hump,0,0,false,s->timestamp_us);
        t->seen=false; t->result=r; return;
    }
    if(!t->calibration.ready) {
        imu_calibration_feed(&t->calibration,s,odo && odo->stationary);
        t->previous_us=s->timestamp_us; t->seen=!t->calibration.ready;
        t->result=r; return;
    }
    r.flags|=TERRAIN_CALIBRATED;
    for(i=0;i<3;++i) {
        a[i]=s->accel_g[i]-t->calibration.bias_g[i];
        t->filtered[i]=continuity?t->filtered[i]+0.25f*(a[i]-t->filtered[i]):a[i];
        m[i]=(s->mag_gauss[i]-t->mag_offset[i])*t->mag_scale[i];
    }
    n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
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
        if(t->odometry_seen && continuity && odo->epoch==t->epoch) {
            double delta=odo->distance_m-t->previous_distance;
            dt=gap/1000000.0f;
            if(fabs(delta)<=0.05) { ds=(float)delta; speed=ds/dt; movement_ok=fabsf(speed)<=1.5f; }
        }
        t->previous_distance=odo->distance_m; t->epoch=odo->epoch; t->odometry_seen=true;
    } else { t->odometry_seen=false; }
    dynamic_g=a[0]-t->filtered[0];
    /* Acceleration alone cannot distinguish slope from sustained acceleration.
     * Reject obvious dynamics/roll/turns; known-height testing remains required. */
    hump_update(&t->hump,pitch,ds,tilt_ok && movement_ok && !r.impact &&
                fabsf(dynamic_g)<0.08f && fabsf(roll)<10 &&
                (!(r.flags&TERRAIN_TURN_VALID) || fabsf(r.turn_dps)<15),s->timestamp_us);
    if(t->hump.highest_valid) { r.peak_m=t->hump.highest_m; r.flags|=TERRAIN_PEAK_VALID; }
    r.hump_completed=t->hump.completed;
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
    int32_t pitch=0,roll=0,rate=0; uint32_t height=0;
    memset(out,0,TERRAIN_PACKET_SIZE);
    if(!isfinite(r->pitch_deg)||!isfinite(r->roll_deg)||fabsf(r->pitch_deg)>90||fabsf(r->roll_deg)>90) { flags&=(uint8_t)~TERRAIN_TILT_VALID; }
    if(!isfinite(r->turn_dps)||fabsf(r->turn_dps)>360) { flags&=(uint8_t)~TERRAIN_TURN_VALID; }
    if(!isfinite(r->peak_m)||r->peak_m<0||r->peak_m>0.5f) { flags&=(uint8_t)~TERRAIN_PEAK_VALID; }
    if(flags&TERRAIN_TILT_VALID) { pitch=(int32_t)lroundf(r->pitch_deg*10); roll=(int32_t)lroundf(r->roll_deg*10); }
    if(flags&TERRAIN_TURN_VALID) { rate=(int32_t)lroundf(r->turn_dps); }
    if(flags&TERRAIN_PEAK_VALID) { height=(uint32_t)lroundf(r->peak_m*1000); }
    put_bits(out,&bit,r->timestamp_us,32); put_bits(out,&bit,(uint32_t)r->state,3); put_bits(out,&bit,flags,5);
    put_bits(out,&bit,(uint32_t)pitch,11); put_bits(out,&bit,(uint32_t)roll,11);
    put_bits(out,&bit,(uint32_t)rate,10); put_bits(out,&bit,height,9);
    put_bits(out,&bit,r->hump_completed,1); put_bits(out,&bit,r->impact,1);
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
}
