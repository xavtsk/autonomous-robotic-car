#include "../../app_program/controller/integration_codec.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void roundtrip(const s3_integration_status_t *in)
{
    s3_status_packet_t packet,again;
    s3_integration_status_t out;
    assert(s3_status_pack(in,&packet));assert(s3_status_unpack(&packet,&out));
    assert(s3_status_pack(&out,&again)&&memcmp(&packet,&again,sizeof(packet))==0);
    assert(memcmp(&in->motion.centre_m,&out.motion.centre_m,sizeof(double))==0);
    assert(out.motion.command_id==in->motion.command_id&&out.motion.report_loss_count==in->motion.report_loss_count);
    assert(out.adc_status==in->adc_status&&out.navigation.latched_faults==in->navigation.latched_faults);
    assert(memcmp(&in->run_timing,&out.run_timing,sizeof(in->run_timing))==0);
    assert(memcmp(&in->navigation.steering_preview.left_mps,&out.navigation.steering_preview.left_mps,sizeof(float))==0);
}
int main(void)
{
    s3_integration_status_t v={0};
    v.navigation.lifecycle=S3_RUN_FAULT_LATCHED;v.navigation.readiness_blockers=31;
    v.navigation.latched_faults=15;v.navigation.accepted_calibration=7;
    v.navigation.stop_requested=v.navigation.odometry_available=v.navigation.rearming=v.navigation.diagnostic=true;
    v.navigation.steering_preview=(s3_steering_request_t){true,-0.0f,0.1234567f};
    v.navigation.capture_distance=(s3_capture_distance_t){-1234.56789,9876.54321,UINT32_MAX,true,true};
    v.motion=(motion_feedback_t){-0.0,UINT32_MAX,UINT32_MAX,MOTION_FAULT,MOTION_FAULT_TIMEOUT,true,true,255,UINT32_MAX};
    v.white=(s3_calibration_summary_t){65535,4095,4095,4095.0f,true};v.black=v.white;
    v.samples=v.max_gap_us=v.above_target=v.command_sequence=UINT32_MAX;
    v.run_timing=(s3_run_timing_t){UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX};
    v.adc_status=INT32_MIN;v.calibration_sensor=S3_SENSOR_RIGHT;
    for(unsigned state=0;state<=S3_DRIVE_FAULT;state++)for(unsigned reason=0;reason<=S3_DRIVE_BAD_INPUT;reason++) {
        v.state=(s3_drive_state_t)state;v.reason=(s3_drive_reason_t)reason;roundtrip(&v);
    }
    s3_status_packet_t packet,copy;assert(s3_status_pack(&v,&packet));copy=packet;
    v.white.count=65536;assert(!s3_status_pack(&v,&packet)&&memcmp(&packet,&copy,sizeof(packet))==0);v.white.count=65535;
    packet.metadata[63]|=0x80;assert(!s3_status_unpack(&packet,&v));
    s3_control_request_t c={0},decoded;s3_control_packet_t p,q;
    for(unsigned kind=0;kind<=S3_CONTROL_RESET;kind++)for(unsigned sensor=0;sensor<S3_SENSOR_COUNT;sensor++) {
        c.kind=(s3_control_kind_t)kind;c.sensor=(s3_sensor_id_t)sensor;
        for(unsigned i=0;i<26;i++)c.map[i]=(s3_nav_action_t)(i%S3_NAV_ACTION_COUNT);
        assert(s3_control_pack(&c,&p)&&s3_control_unpack(&p,&decoded)&&s3_control_pack(&decoded,&q));
        assert(memcmp(&p,&q,sizeof(p))==0);
    }
    uint32_t rng=0x7238a1U;unsigned accepted=0;
    for(unsigned i=0;i<20000;i++) {
        uint8_t *bytes=(uint8_t *)&packet;
        for(unsigned j=0;j<sizeof(packet);j++){rng=rng*1664525U+1013904223U;bytes[j]=(uint8_t)(rng>>24);}
        if(s3_status_unpack(&packet,&v)) {assert(s3_status_pack(&v,&copy)&&memcmp(&packet,&copy,sizeof(packet))==0);accepted++;}
    }
    printf("PASS: full 60-byte nav/146-byte shared status/11-byte control, exact doubles/floats and run timing, boundaries/padding; hostile status packets accepted=%u\n",accepted);
}
