/* Execute the actual sampling/controller runtime with mocked IMU transport.
 * Existing runtime harness provides ADC, RTOS and motion boundaries. */
#undef APP_H
#define main baseline_runtime_main
#include "../integration/runtime_test.c"
#undef main
#include "../../app_program/subsystem1/subsystem1.h"

static terrain_result_t imu;
static bool readable=true, reset_pending, reject_begin, event_pending, event_lost;
static uint16_t run_id, count, sent_count, sent_height;
static unsigned begins, reports, invalidations;
static UB faults=BUDDY1_FAULT_WIFI|BUDDY1_FAULT_STALL|BUDDY1_FAULT_NO_ECHO;
static terrain_result_t queued;
bool subsystem4_read(terrain_result_t *out,uint16_t *run,uint16_t *humps)
{
    assert(!locked);
    if(!readable || reset_pending) { return false; }
    *out=imu; *run=run_id; *humps=count; return true;
}
bool subsystem4_begin_run(uint16_t *token)
{
    assert(!locked); if(reject_begin) { return false; }
    begins++; *token=++run_id; reset_pending=true; return true;
}
bool subsystem4_event(terrain_result_t *out,bool *lost)
{
    assert(!locked); if(!event_pending) { return false; }
    *out=queued; *lost=event_lost; event_pending=false; return true;
}
ER buddy1_update_faults(UB changed,UB active)
{ assert(changed==BUDDY1_FAULT_IMU && !(active&~changed)); faults=(faults&~changed)|active; return E_OK; }
ER buddy1_report_hump(UH humps,UH mm)
{ reports++; sent_count=humps; sent_height=mm; return E_OK; }
ER buddy1_invalidate(UB topic)
{ assert(topic==BUDDY1_TOPIC_HUMP); invalidations++; return E_OK; }
static void fresh(void)
{
    imu=(terrain_result_t){.timestamp_us=clock_value+10000U,
        .flags=TERRAIN_CALIBRATED|TERRAIN_TILT_VALID,.state=TERRAIN_STATIONARY};
}
static void step(void) { fresh(); cycle(); terrain_control_report(); }
static void command(s3_control_kind_t kind)
{
    s3_control_request_t c={.kind=kind};
    assert(s3_integration_submit(&c)); step();
}
static void reset_fault(void)
{
    command(S3_CONTROL_RESET); step();
    assert(g_drive.state==S3_DRIVE_OFF);
}
static void begin(void)
{
    command(S3_CONTROL_START);
    assert(g_drive.state==S3_DRIVE_FOLLOW && reset_pending);
    assert(fb_command_id==0); /* START has not requested motor velocity. */
}
static void ack(void)
{
    reset_pending=false; count=0; step();
    for(unsigned i=0;i<5 && fb_command_id==0;i++) { step(); }
    assert(g_drive.state==S3_DRIVE_FOLLOW && !g_drive.external_hold && fb_command_id==1);
}
int main(void)
{
    s3_calibration_collector_t cal;
    s3_integration_status_t status;
    terrain_control_init(); assert(s3_integration_init()==E_OK);
    s3_calibration_begin(&cal);
    for(unsigned i=0;i<12;i++) {
        assert(s3_calibration_add(&cal,IR_SURFACE_WHITE,300));
        assert(s3_calibration_add(&cal,IR_SURFACE_BLACK,2500));
    }
    for(unsigned i=0;i<3;i++) { assert(s3_navigation_apply_calibration(&g_drive.navigation,(s3_sensor_id_t)i,&cal)); }
    raw=1400; for(unsigned i=0;i<12;i++) { step(); }
    imu.flags=0; imu.timestamp_us=clock_value+10000; cycle();
    assert(!execute(&(s3_control_request_t){.kind=S3_CONTROL_START},clock_value));
    assert(begins==0 && freeze_calls==0);
    step(); reject_begin=true; command(S3_CONTROL_START);
    assert(g_drive.state==S3_DRIVE_OFF && cancel_calls==1 && begins==0);
    reject_begin=false; begin();
    for(unsigned i=0;i<5;i++) { step(); assert(g_drive.external_hold && fb_command_id==0); }
    ack();
    /* Cumulative snapshot includes missed delivery of earlier hump events. */
    fresh(); imu.flags|=TERRAIN_PEAK_VALID; imu.peak_m=0.123f; count=3;
    cycle(); terrain_control_report();
    assert(reports==1 && sent_count==3 && sent_height==123);
    terrain_control_report(); assert(reports==1);
    /* An impact event survives even when the latest sample is normal. */
    queued=(terrain_result_t){.impact=true}; event_pending=true; step();
    assert(g_drive.state==S3_DRIVE_FAULT && g_drive.reason==S3_DRIVE_TERRAIN_FAULT);
    assert(s3_integration_status(&status) && status.reason==S3_DRIVE_TERRAIN_FAULT);
    assert((faults&(BUDDY1_FAULT_WIFI|BUDDY1_FAULT_STALL|BUDDY1_FAULT_NO_ECHO|BUDDY1_FAULT_IMU))==faults);
    assert(faults&BUDDY1_FAULT_IMU);
    command(S3_CONTROL_START); assert(g_drive.state==S3_DRIVE_FAULT);
    reset_fault(); begin(); ack();
    assert(invalidations>0 && !(faults&BUDDY1_FAULT_IMU));
    { unsigned before=invalidations;
      for(unsigned i=0;i<100;++i) terrain_control_report();
      assert(invalidations==before); /* already unavailable: no repeated lock/write */ }
    assert(faults==(BUDDY1_FAULT_WIFI|BUDDY1_FAULT_STALL|BUDDY1_FAULT_NO_ECHO));
    /* One missed nonblocking read uses a still-fresh cached sample. */
    readable=false; cycle(); assert(g_drive.state==S3_DRIVE_FOLLOW);
    for(unsigned i=0;i<16;i++) { cycle(); }
    assert(g_drive.state==S3_DRIVE_FAULT);
    readable=true; reset_fault(); begin(); ack();
    queued=(terrain_result_t){0}; event_pending=event_lost=true; step();
    assert(g_drive.state==S3_DRIVE_FAULT); event_lost=false;
    reset_fault(); begin();
    for(unsigned i=0;i<51;i++) { step(); }
    assert(g_drive.state==S3_DRIVE_FAULT); reset_pending=false;
    reset_fault(); begin(); command(S3_CONTROL_STOP);
    assert(g_drive.state==S3_DRIVE_OFF && !g_drive.external_hold && fb_command_id==0);
    reset_pending=false; step(); begin(); ack();
    run_id++; step(); assert(g_drive.state==S3_DRIVE_FAULT);
    puts("PASS: IMU START/reset acknowledgement and timeout, STOP priority, stale data, lost/impact events, latched reset, cumulative humps and owned fault bit");
    return 0;
}
