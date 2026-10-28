#include "integration_codec.h"
#include <float.h>
_Static_assert(sizeof(double)==8 && DBL_MANT_DIG==53 && sizeof(float)==4,
               "exact odometry and calibration representations");
_Static_assert(sizeof(s3_status_packet_t)==146 && sizeof(s3_control_packet_t)==11,
               "copied runtime storage");
static void put(uint8_t *p,unsigned *bit,uint32_t value,unsigned width)
{ for(unsigned i=0;i<width;i++,(*bit)++) if(value&((uint32_t)1U<<i)) p[*bit/8]|=(uint8_t)(1U<<(*bit%8)); }
static uint32_t get(const uint8_t *p,unsigned *bit,unsigned width)
{ uint32_t v=0;for(unsigned i=0;i<width;i++,(*bit)++) if(p[*bit/8]&(1U<<(*bit%8))) v|=(uint32_t)1U<<i;return v; }
static void put_double(uint8_t *p,unsigned *bit,double value)
{ union{double f;uint64_t u;}v;v.f=value;put(p,bit,(uint32_t)v.u,32);put(p,bit,(uint32_t)(v.u>>32),32); }
static double get_double(const uint8_t *p,unsigned *bit)
{ union{double f;uint64_t u;}v;uint32_t lo=get(p,bit,32),hi=get(p,bit,32);v.u=(uint64_t)lo|((uint64_t)hi<<32);return v.f; }
static void put_float(uint8_t *p,unsigned *bit,float value)
{ union{float f;uint32_t u;}v;v.f=value;put(p,bit,v.u,32); }
static float get_float(const uint8_t *p,unsigned *bit)
{ union{float f;uint32_t u;}v;v.u=get(p,bit,32);return v.f; }
bool s3_control_pack(const s3_control_request_t *v,s3_control_packet_t *out)
{
    s3_control_packet_t p={{0}};unsigned bit=0;
    bool sensor;
    if(!v||!out||(unsigned)v->kind>S3_CONTROL_RESET)return false;
    sensor=v->kind==S3_CONTROL_WHITE||v->kind==S3_CONTROL_BLACK||v->kind==S3_CONTROL_APPLY_CALIBRATION;
    if(sensor&&(unsigned)v->sensor>=S3_SENSOR_COUNT)return false;
    put(p.bytes,&bit,v->kind,3);put(p.bytes,&bit,sensor?v->sensor:0U,2);
    for(unsigned i=0;i<26;i++){
        unsigned action=v->kind==S3_CONTROL_MAPPING?(unsigned)v->map[i]:0U;
        if(action>=S3_NAV_ACTION_COUNT)return false;
        put(p.bytes,&bit,action,3);
    }
    *out=p;return true;
}
bool s3_control_unpack(const s3_control_packet_t *p,s3_control_request_t *out)
{
    s3_control_request_t v={0};s3_control_packet_t canonical;unsigned bit=0;
    if(!p||!out||(p->bytes[10]&0xf8U))return false;
    v.kind=(s3_control_kind_t)get(p->bytes,&bit,3);
    v.sensor=(s3_sensor_id_t)get(p->bytes,&bit,2);
    for(unsigned i=0;i<26;i++)v.map[i]=(s3_nav_action_t)get(p->bytes,&bit,3);
    if(!s3_control_pack(&v,&canonical))return false;
    for(unsigned i=0;i<sizeof(*p);i++)if(p->bytes[i]!=canonical.bytes[i])return false;
    *out=v;return true;
}
static bool summary_valid(const s3_calibration_summary_t *v)
{return v->count<=65535U&&v->minimum<=4095U&&v->maximum<=4095U;}
static void put_summary(uint8_t *p,unsigned *bit,const s3_calibration_summary_t *v)
{put(p,bit,v->count,16);put(p,bit,v->minimum,12);put(p,bit,v->maximum,12);put_float(p,bit,v->average);put(p,bit,v->available,1);}
static void get_summary(const uint8_t *p,unsigned *bit,s3_calibration_summary_t *v)
{v->count=get(p,bit,16);v->minimum=(uint16_t)get(p,bit,12);v->maximum=(uint16_t)get(p,bit,12);v->average=get_float(p,bit);v->available=get(p,bit,1)!=0;}
bool s3_status_pack(const s3_integration_status_t *v,s3_status_packet_t *out)
{
    s3_status_packet_t p={0};unsigned bit=0;
    if(!v||!out||(unsigned)v->state>S3_DRIVE_FAULT||(unsigned)v->reason>S3_DRIVE_TERRAIN_FAULT||
       (unsigned)v->motion.state>MOTION_FAULT||(unsigned)v->motion.fault>MOTION_FAULT_TIMEOUT||
       (unsigned)v->calibration_sensor>=S3_SENSOR_COUNT||!summary_valid(&v->white)||!summary_valid(&v->black)||
       !s3_navigation_pack(&v->navigation,&p.navigation))return false;
    put_double(p.feedback,&bit,v->motion.centre_m);
    put(p.feedback,&bit,v->motion.timestamp_us,32);put(p.feedback,&bit,v->motion.epoch,32);
    put(p.feedback,&bit,v->motion.state,3);put(p.feedback,&bit,v->motion.fault,3);
    put(p.feedback,&bit,v->motion.valid,1);put(p.feedback,&bit,v->motion.braked,1);
    put(p.feedback,&bit,v->motion.command_id,8);put(p.feedback,&bit,v->motion.report_loss_count,32);
    bit=0;put(p.metadata,&bit,v->state,3);put(p.metadata,&bit,v->reason,4);
    put(p.metadata,&bit,v->samples,32);put(p.metadata,&bit,v->max_gap_us,32);
    put(p.metadata,&bit,v->above_target,32);put(p.metadata,&bit,v->command_sequence,32);
    put(p.metadata,&bit,v->command_ok,1);put(p.metadata,&bit,v->collecting,1);put(p.metadata,&bit,v->calibration_sensor,2);
    put_summary(p.metadata,&bit,&v->white);put_summary(p.metadata,&bit,&v->black);
    put(p.metadata,&bit,(uint32_t)v->adc_status,32);
    put(p.metadata,&bit,v->run_timing.started_us,32);put(p.metadata,&bit,v->run_timing.samples,32);
    put(p.metadata,&bit,v->run_timing.max_gap_us,32);put(p.metadata,&bit,v->run_timing.above_target,32);
    put(p.metadata,&bit,v->run_timing.above_safety,32);put(p.metadata,&bit,v->run_timing.last_safety_us,32);
    *out=p;return true;
}
bool s3_status_unpack(const s3_status_packet_t *p,s3_integration_status_t *out)
{
    s3_integration_status_t v={0};unsigned bit=0;uint32_t error;
    if(!p||!out||(p->metadata[63]&0xe0U)||!s3_navigation_unpack(&p->navigation,&v.navigation))return false;
    v.motion.centre_m=get_double(p->feedback,&bit);
    v.motion.timestamp_us=get(p->feedback,&bit,32);v.motion.epoch=get(p->feedback,&bit,32);
    v.motion.state=(motion_state_t)get(p->feedback,&bit,3);v.motion.fault=(motion_fault_t)get(p->feedback,&bit,3);
    v.motion.valid=get(p->feedback,&bit,1)!=0;v.motion.braked=get(p->feedback,&bit,1)!=0;
    v.motion.command_id=(uint8_t)get(p->feedback,&bit,8);v.motion.report_loss_count=get(p->feedback,&bit,32);
    if(v.motion.state>MOTION_FAULT||v.motion.fault>MOTION_FAULT_TIMEOUT)return false;
    bit=0;v.state=(s3_drive_state_t)get(p->metadata,&bit,3);v.reason=(s3_drive_reason_t)get(p->metadata,&bit,4);
    if(v.state>S3_DRIVE_FAULT||v.reason>S3_DRIVE_TERRAIN_FAULT)return false;
    v.samples=get(p->metadata,&bit,32);v.max_gap_us=get(p->metadata,&bit,32);
    v.above_target=get(p->metadata,&bit,32);v.command_sequence=get(p->metadata,&bit,32);
    v.command_ok=get(p->metadata,&bit,1)!=0;v.collecting=get(p->metadata,&bit,1)!=0;
    v.calibration_sensor=(s3_sensor_id_t)get(p->metadata,&bit,2);
    if(v.calibration_sensor>=S3_SENSOR_COUNT)return false;
    get_summary(p->metadata,&bit,&v.white);get_summary(p->metadata,&bit,&v.black);
    error=get(p->metadata,&bit,32);v.adc_status=error<=INT32_MAX?(int32_t)error:-1-(int32_t)(UINT32_MAX-error);
    v.run_timing.started_us=get(p->metadata,&bit,32);v.run_timing.samples=get(p->metadata,&bit,32);
    v.run_timing.max_gap_us=get(p->metadata,&bit,32);v.run_timing.above_target=get(p->metadata,&bit,32);
    v.run_timing.above_safety=get(p->metadata,&bit,32);v.run_timing.last_safety_us=get(p->metadata,&bit,32);
    *out=v;return true;
}
