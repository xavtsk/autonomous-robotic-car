/* Execute the production runtime with fake task/ADC boundaries. This proves
 * queue/snapshot ownership, not scheduling deadlines or physical calibration. */
#define APP_H
#define APP_S3_MOTION 1
#define APP_DIAGNOSTICS 0
#define APP_TEST 0
#define S3_BUILTIN_CALIBRATION 0 /* these tests start uncalibrated */
/* Original Buddy 3 line/junction behaviour; the course profile (tape between
 * the sensors, straight across unannounced crossings) is tested in course_test.c */
#define S3_LINE_BETWEEN 0
#define S3_CROSS_STRAIGHT 0
#define S3_COURSE_PENDING_WAIT_US (5000000U)
#include "controller_test_kernel.h"
#include "../../app_program/controller/s3_integration.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#define TA_HLNG 2U
#define TSK_SELF 0
typedef struct { unsigned cycatr; FP cychdr; unsigned cyctim,cycphs; } T_CCYC;
static jmp_buf boundary;
static unsigned sleeps,locked,observed,freeze_calls,stops;
static uint32_t clock_value;
static int32_t adc_error;
static uint16_t raw=150;
static bool fb_braked=true;
static bool fb_fault, reset_clears_fault=true;
static uint8_t fb_command_id;
static unsigned reset_calls;
static unsigned cancel_calls;
static unsigned velocity_requests;
static void (*during_start)(void);
static bool test_mutex_busy;
static uint32_t debug_motion_advance, debug_motion_age;
static bool debug_motion_out_of_range;
ID tk_cre_mtx(const T_CMTX *m) { assert(m->mtxatr==TA_INHERIT); return 1; }
ER tk_loc_mtx(ID id,TMO t) { if(test_mutex_busy){assert(t==TMO_POL);return E_TMOUT;}assert(id==1&&!locked);locked=1;return E_OK; }
ER tk_unl_mtx(ID id) { assert(id==1&&locked);locked=0;return E_OK; }
ID tk_cre_cyc(const T_CCYC *c)
{ assert(c->cyctim==S3_SAMPLE_PERIOD_MS&&c->cycphs==S3_SAMPLE_PERIOD_MS&&c->cychdr); return 2; }
ER tk_stp_cyc(ID id) { (void)id;return E_OK; }
ER tk_del_cyc(ID id) { (void)id;return E_OK; }
ER tk_del_mtx(ID id) { (void)id;return E_OK; }
ER tk_del_tsk(ID id) { (void)id;return E_OK; }
ER tk_ter_tsk(ID id) { (void)id;return E_OK; }
ER tk_sta_cyc(ID id) { (void)id;return E_OK; }
ER tk_wup_tsk(ID id) { (void)id;return E_OK; }
ER tk_can_wup(ID id) { (void)id;return E_OK; }
ER tk_slp_tsk(TMO t) { assert(t==TMO_FEVR&&!locked);if(++sleeps==2)longjmp(boundary,1);return E_OK; }
ER tk_dly_tsk(unsigned ms) { (void)ms;return E_OK; }
ID app_start_task(FP f,PRI p,UW core,const char *name)
{ (void)f;(void)p;(void)core;(void)name;return 3; }
int tm_printf(const UB *format,...) { (void)format;assert(!locked);return 0; }
uint32_t runtime_read_clock(uint32_t address) { (void)address;return clock_value; }
int32_t ir_sensor_init(const s3_acquisition_config_t *cfg)
{ assert(!locked&&cfg->enabled_mask==7);return E_OK; }
int32_t ir_sensor_read_frame(s3_sensor_frame_t *f)
{ assert(!locked);*f=(s3_sensor_frame_t){.timestamp_us=clock_value};
  for(unsigned i=0;i<3;i++)f->sensor[i]=(s3_raw_sample_t){clock_value,raw,0,true,true};
  return adc_error; }
bool motion_get_feedback(motion_feedback_t *f)
{ assert(!locked);*f=(motion_feedback_t){.timestamp_us=clock_value,.epoch=1,.valid=true,.braked=fb_braked,.command_id=fb_command_id,
    .state=fb_fault?MOTION_FAULT:MOTION_IDLE,.fault=fb_fault?MOTION_FAULT_STALL:MOTION_FAULT_NONE};return true; }
bool motion_try_debug_snapshot(motion_debug_snapshot_t *m)
{ assert(!locked);clock_value+=debug_motion_advance;
  *m=(motion_debug_snapshot_t){.timestamp_us=clock_value-debug_motion_age,.valid=true,.braked=fb_braked,
    .setpoint_mps={0.04f,-0.05f},.measured_mps={0.035f,-0.042f},.encoder={-123,456},.duty={0.2f,-0.3f}};
  if(debug_motion_out_of_range){m->measured_mps[0]=10.0f;}
  return true; }
motion_result_t motion_request_velocity(float l,float r)
{ (void)l;(void)r;assert(!locked);velocity_requests++;fb_command_id=1;return MOTION_OK; }
motion_result_t motion_request_drive(float m,float v) { (void)m;(void)v;return MOTION_OK; }
motion_result_t motion_request_manoeuvre(manoeuvre_t m)
{ (void)m;assert(!locked);return MOTION_OK; }
void stop(void) { assert(!locked);stops++;fb_command_id=0; }
bool motion_get_report(motion_report_t *r) { (void)r;assert(!locked);return false; }
uint8_t motion_command_id(void) { return 1; }
bool motion_reset(void) { assert(!locked);reset_calls++;if(reset_clears_fault)fb_fault=false;return true; }
void motion_navigation_heartbeat(bool enabled) { (void)enabled;assert(!locked); }
bool vehicle_controller_prepare_start(s3_navigation_t *n)
{ (void)n;assert(!locked);freeze_calls++;if(during_start)during_start();return true; }
bool vehicle_controller_apply_mapping(const s3_nav_action_t m[26])
{ (void)m;assert(!locked);return true; }
void vehicle_controller_cancel_start(void)
{ assert(!locked);cancel_calls++; }
void vehicle_controller_commit_start(void) {}
void vehicle_controller_observe(const s3_integration_status_t *s)
{ assert(!locked&&s);observed++; }
#include "../../app_program/controller/s3_integration.c"
static void cycle(void)
{ sleeps=0;clock_value+=S3_SAMPLE_PERIOD_US;if(setjmp(boundary)==0)sample(0,NULL);assert(!locked); }
static void remote_send(uint8_t action,uint32_t id,uint32_t epoch,uint32_t timestamp)
{
    uint8_t p[REMOTE_COMMAND_SIZE]={REMOTE_VERSION,action};
    remote_put32(p+2,id);remote_put32(p+6,epoch);remote_put32(p+10,timestamp);
    s3_remote_receive(p,sizeof p);
}
static void remote_expect(uint32_t id,uint8_t result,uint8_t reason)
{
    uint8_t p[REMOTE_REPLY_SIZE];
    assert(s3_remote_reply_take(p,sizeof p)==sizeof p);
    assert(p[0]==REMOTE_VERSION&&remote_get32(p+2)==id&&p[6]==result&&p[7]==reason);
}
static void stop_during_start(void)
{ during_start=0;remote_send(REMOTE_STOP,5,0x87654321U,clock_value); }
static void reconnect_during_start(void)
{ during_start=0;s3_remote_link(0,false);s3_remote_link(0xabcddcbaU,true); }
int main(void)
{
    s3_integration_status_t status;
    s3_control_request_t cmd={.kind=S3_CONTROL_WHITE,.sensor=S3_SENSOR_BARCODE};
    assert(!s3_integration_submit(&cmd));
    assert(s3_integration_init()==E_OK);
    assert(!s3_integration_status(&status));
    assert(s3_integration_submit(&cmd));
    cmd.kind=S3_CONTROL_BLACK;assert(!s3_integration_submit(&cmd));
    cmd.kind=S3_CONTROL_STOP;assert(s3_integration_submit(&cmd));
    cmd.kind=S3_CONTROL_START;assert(!s3_integration_submit(&cmd));
    cycle();assert(s3_integration_status(&status));
    assert(status.command_sequence==1&&status.command_ok&&!status.collecting&&stops>0);
    assert(s3_integration_submit(&cmd));cycle();
    assert(s3_integration_status(&status)&&!status.command_ok&&freeze_calls==0);
    cmd.kind=S3_CONTROL_WHITE;assert(s3_integration_submit(&cmd));cycle();cycle();
    assert(s3_integration_status(&status)&&status.collecting&&status.white.count==1);
    assert(status.navigation.sensor[0].valid&&status.motion.epoch==1);
    cmd.kind=S3_CONTROL_STOP;assert(s3_integration_submit(&cmd));cycle();
    assert(s3_integration_status(&status)&&!status.collecting);
    adc_error=-1234567;cycle();
    assert(s3_integration_status(&status)&&status.adc_status==-1234567);
    assert(observed==6&&freeze_calls==0);
    /* Complete production calibration flow, no shortcut: WHITE then BLACK
     * collected for 1 s each, then APPLY, for every sensor. The earlier reset
     * bug kept both minima at 0, so this valid pair was always rejected. */
    adc_error=0;cycle();
    assert(g_drive.navigation.accepted_calibration==0U);
    for(unsigned s=0;s<S3_SENSOR_COUNT;s++){
        for(unsigned surface=0;surface<2;surface++){
            uint16_t base=surface?2500:300;
            cmd=(s3_control_request_t){.kind=surface?S3_CONTROL_BLACK:S3_CONTROL_WHITE,.sensor=(s3_sensor_id_t)s};
            assert(s3_integration_submit(&cmd));
            for(unsigned k=0;k<1100000U/S3_SAMPLE_PERIOD_US;k++){raw=(uint16_t)(base+k%7);cycle();}
            assert(s3_integration_status(&status)&&status.command_ok&&!status.collecting);
            assert(status.calibration_sensor==s);
        }
        assert(status.white.count>=10&&status.white.minimum==300&&status.white.maximum==306);
        assert(status.black.count>=10&&status.black.minimum==2500&&status.black.maximum==2506);
        cmd=(s3_control_request_t){.kind=S3_CONTROL_APPLY_CALIBRATION,.sensor=(s3_sensor_id_t)s};
        assert(s3_integration_submit(&cmd));cycle();
        assert(s3_integration_status(&status)&&status.command_ok);
        assert(status.navigation.accepted_calibration==((1U<<(s+1))-1U));
    }
    /* Overlapping surfaces must still be rejected and leave the sensor's
     * accepted calibration untouched. */
    cmd=(s3_control_request_t){.kind=S3_CONTROL_WHITE,.sensor=S3_SENSOR_LEFT};
    assert(s3_integration_submit(&cmd));for(unsigned k=0;k<1100000U/S3_SAMPLE_PERIOD_US;k++){raw=(uint16_t)(1000+k%7);cycle();}
    cmd.kind=S3_CONTROL_BLACK;
    assert(s3_integration_submit(&cmd));for(unsigned k=0;k<1100000U/S3_SAMPLE_PERIOD_US;k++){raw=(uint16_t)(1003+k%7);cycle();}
    cmd.kind=S3_CONTROL_APPLY_CALIBRATION;assert(s3_integration_submit(&cmd));cycle();
    assert(s3_integration_status(&status)&&!status.command_ok);
    assert(g_drive.navigation.accepted_calibration==S3_ALL_SENSORS);
    raw=150;cycle();
    /* A START refused by Buddy 2's stillness check must not leave the
     * pre-run map frozen: the freeze is undone exactly once. */
    assert(s3_navigation_readiness(&g_drive.navigation,clock_value)==0U);
    fb_braked=false;cmd.kind=S3_CONTROL_START;assert(s3_integration_submit(&cmd));cycle();
    assert(s3_integration_status(&status)&&!status.command_ok&&status.state==S3_DRIVE_OFF);
    assert(freeze_calls==1&&cancel_calls==1);
    fb_braked=true;assert(s3_integration_submit(&cmd));cycle();
    assert(s3_integration_status(&status)&&status.command_ok&&status.state==S3_DRIVE_FOLLOW);
    assert(freeze_calls==2&&cancel_calls==1);
    /* Idle/boot maxima cannot be mistaken for the current run's statistics.
     * START resets the run counters; include the interval causing a fault. */
    assert(status.run_timing.samples==0&&status.run_timing.max_gap_us==0);
    raw=900;cycle();
    assert(s3_integration_status(&status)&&status.state==S3_DRIVE_FOLLOW);
    assert(status.run_timing.samples==1&&status.run_timing.max_gap_us==S3_SAMPLE_PERIOD_US);
    assert(status.run_timing.above_target==0&&status.run_timing.above_safety==0);
    clock_value+=120000;cycle();
    assert(s3_integration_status(&status)&&status.state==S3_DRIVE_FAULT);
    assert(status.run_timing.max_gap_us==120000U+S3_SAMPLE_PERIOD_US&&status.run_timing.above_safety==1);
    assert(status.run_timing.last_safety_us==clock_value);
    s3_run_timing_t frozen=status.run_timing;
    cycle();cycle();assert(s3_integration_status(&status));
    assert(status.run_timing.samples==frozen.samples&&status.run_timing.last_safety_us==frozen.last_safety_us);

    /* One queued RESET waits for fresh feedback/health, then acknowledges once.
     * Ordinary commands are blocked; accepting the request is not success. */
    uint32_t sequence=status.command_sequence;
    cmd.kind=S3_CONTROL_RESET;assert(s3_integration_submit(&cmd));cycle();
    assert(s3_integration_status(&status)&&status.command_sequence==sequence&&reset_calls==1);
    cmd.kind=S3_CONTROL_START;assert(!s3_integration_submit(&cmd));
    for(unsigned k=0;k<200000U/S3_SAMPLE_PERIOD_US;k++)cycle();
    assert(s3_integration_status(&status)&&status.command_sequence==sequence+1&&status.command_ok&&status.state==S3_DRIVE_OFF);
    assert(!g_shared.reset_pending&&!g_reset_waiting);

    /* A genuine motion fault is cleared by the ONE accepted request, but
     * completion still waits until the next feedback publication. */
    cmd.kind=S3_CONTROL_START;assert(s3_integration_submit(&cmd));cycle();
    assert(s3_integration_status(&status)&&status.state==S3_DRIVE_FOLLOW&&status.run_timing.samples==0);
    fb_fault=true;cycle();assert(s3_integration_status(&status)&&status.state==S3_DRIVE_FAULT);
    sequence=status.command_sequence;cmd.kind=S3_CONTROL_RESET;
    assert(s3_integration_submit(&cmd));cycle();
    assert(s3_integration_status(&status)&&status.command_sequence==sequence&&!fb_fault);
    cycle();assert(s3_integration_status(&status)&&status.command_sequence==sequence+1&&status.command_ok&&status.state==S3_DRIVE_OFF);

    /* STOP cancels an in-flight reset; the old completion must not produce a
     * later success or a second sequence increment after cancellation. */
    cmd.kind=S3_CONTROL_START;assert(s3_integration_submit(&cmd));cycle();
    fb_fault=true;cycle();reset_clears_fault=false;
    cmd.kind=S3_CONTROL_RESET;assert(s3_integration_submit(&cmd));cycle();
    assert(g_reset_waiting&&s3_integration_status(&status));sequence=status.command_sequence;
    cmd.kind=S3_CONTROL_STOP;assert(s3_integration_submit(&cmd));cycle();
    assert(s3_integration_status(&status)&&status.state==S3_DRIVE_FAULT&&status.command_sequence==sequence+1);
    sequence=status.command_sequence;for(unsigned k=0;k<600000U/S3_SAMPLE_PERIOD_US;k++)cycle();
    assert(s3_integration_status(&status)&&status.command_sequence==sequence&&!g_reset_waiting&&!g_shared.reset_pending);

    /* Persistent fault: bounded refusal with no automatic start. */
    cmd.kind=S3_CONTROL_RESET;assert(s3_integration_submit(&cmd));cycle();
    for(unsigned k=0;k<510000U/S3_SAMPLE_PERIOD_US;k++)cycle();
    assert(s3_integration_status(&status)&&!status.command_ok&&status.state==S3_DRIVE_FAULT);
    assert(status.command_sequence==sequence+1&&!g_reset_waiting&&!g_shared.reset_pending);
#if defined(TM_MQTT) && TM_MQTT
    {   /* After the run ended, the barcode trace goes out over Wi-Fi once,
         * binary summary first and binary end last; then nothing more. */
        unsigned char line[MQTT_UTK_DEBUG_PAYLOAD_MAX];
        unsigned n, lines = 0U;
        unsigned first_kind=99,last_kind=99;
        /* A busy capture lock must skip, not wait or abandon the capture. */
        test_mutex_busy=true;assert(mqtt_utk_debug_take(line,sizeof line)==0U);
        test_mutex_busy=false;assert(g_bcap_net_pending);
        while ((n = mqtt_utk_debug_take(line, sizeof line)) != 0U)
        {
            assert(n <= BCAP_WIRE_MAX && n>=6U && line[0]==BCAP_WIRE_VERSION);
            if (lines++ == 0U) first_kind=line[5]&3U;
            last_kind=line[5]&3U;
        }
        assert(lines >= 2U && first_kind==0U && last_kind==3U);
        assert(mqtt_utk_debug_take(line, sizeof line) == 0U);
        puts("PASS: barcode trace published once over the Wi-Fi debug hook after the run");
    }
#endif
#if APP_IR_DEBUG && defined(TM_MQTT) && TM_MQTT
    {
        unsigned char bytes[IR_DEBUG_PACKET_SIZE];
        ir_debug_packet_t preview;
        ir_debug_sample_t decoded;
        for(unsigned k=0;k<50000U/S3_SAMPLE_PERIOD_US;k++)cycle();
        assert(mqtt_utk_ir_debug_take(bytes,sizeof bytes-1U)==0U);
        assert(mqtt_utk_ir_debug_take(bytes,sizeof bytes)==16U);
        memcpy(preview.bytes,bytes,sizeof bytes);
        assert(ir_debug_unpack(&preview,&decoded)&&decoded.raw_valid==7U);
        assert(decoded.raw[0]==raw&&decoded.raw[1]==raw&&decoded.raw[2]==raw);
        assert(mqtt_utk_ir_debug_take(bytes,sizeof bytes)==0U);
        for(unsigned k=0;k<50000U/S3_SAMPLE_PERIOD_US;k++)cycle();
        test_mutex_busy=true;
        assert(mqtt_utk_ir_debug_take(bytes,sizeof bytes)==0U);
        test_mutex_busy=false;
        assert(mqtt_utk_ir_debug_take(bytes,sizeof bytes)==16U);
        for(unsigned k=0;k<50000U/S3_SAMPLE_PERIOD_US;k++)cycle();
        clock_value+=100001U;
        assert(mqtt_utk_ir_debug_take(bytes,sizeof bytes)==0U&&!g_ir_ready);
        puts("PASS: production IR preview latest-only mailbox, physical channel mapping, bounded age and nonblocking network access");
        {
            unsigned char control[CONTROL_DEBUG_PACKET_SIZE];
            control_debug_packet_t cp;
            control_debug_sample_t cd;
            for(unsigned k=0;k<200000U/S3_SAMPLE_PERIOD_US;k++)cycle();
            assert(mqtt_utk_control_debug_take(control,sizeof control-1)==0);
            assert(mqtt_utk_control_debug_take(control,sizeof control)==70);
            memcpy(cp.bytes,control,sizeof control);
            assert(control_debug_unpack(&cp,&cd));
            assert(cd.motion_valid && cd.setpoint_right==-50 && cd.encoder_left==-123 && cd.measured_right==-42);
            assert(cd.accepted_calibration==7 && cd.left_white_limit<=1000);
            assert(mqtt_utk_control_debug_take(control,sizeof control)==0);
            for(unsigned k=0;k<200000U/S3_SAMPLE_PERIOD_US;k++)cycle();
            test_mutex_busy=true;assert(mqtt_utk_control_debug_take(control,sizeof control)==0);test_mutex_busy=false;
            assert(mqtt_utk_control_debug_take(control,sizeof control)==70);
            for(unsigned k=0;k<200000U/S3_SAMPLE_PERIOD_US;k++){cycle();}
            clock_value+=400001;
            assert(mqtt_utk_control_debug_take(control,sizeof control)==0&&!g_control_ready);
            /* A newer motion tick after the ADC stamp is fresh, not wrapped
             * negative age. Old or out-of-range feedback remains invalid. */
            debug_motion_advance=100;
            for(unsigned k=0;k<200000U/S3_SAMPLE_PERIOD_US;k++)cycle();
            assert(mqtt_utk_control_debug_take(control,sizeof control)==70);
            memcpy(cp.bytes,control,sizeof control);assert(control_debug_unpack(&cp,&cd));
            assert(cd.motion_valid && (int32_t)(cd.motion_timestamp_us-cd.timestamp_us)>0);
            debug_motion_advance=0;debug_motion_age=60000;
            for(unsigned k=0;k<200000U/S3_SAMPLE_PERIOD_US;k++)cycle();
            assert(mqtt_utk_control_debug_take(control,sizeof control)==70);
            memcpy(cp.bytes,control,sizeof control);assert(control_debug_unpack(&cp,&cd));
            assert(cd.motion_available&&!cd.motion_valid&&cd.measured_left==0&&cd.encoder_left==0);
            debug_motion_age=0;debug_motion_out_of_range=true;
            for(unsigned k=0;k<200000U/S3_SAMPLE_PERIOD_US;k++)cycle();
            assert(mqtt_utk_control_debug_take(control,sizeof control)==70);
            memcpy(cp.bytes,control,sizeof control);assert(control_debug_unpack(&cp,&cd));
            assert(!cd.motion_valid&&cd.setpoint_left==0&&cd.duty_left==0);
            debug_motion_out_of_range=false;
            puts("PASS: controller debug real runtime, signed motion/calibration fields, latest-only nonblocking mailbox and stale drop");
        }
    }
#endif
    /* Remote commands share the physical owner/gates, but never acknowledge
     * somebody else's GP command_sequence. No network callback moves motors. */
    s3_remote_link(0x12345678U,true);
    sequence=status.command_sequence;
    remote_send(REMOTE_START,1,0x12345678U,clock_value);
    remote_expect(1,REMOTE_QUEUED,REMOTE_NONE);cycle();
    remote_expect(1,REMOTE_REJECTED,REMOTE_NOT_READY); /* FAULT cannot START */
    reset_clears_fault=true;
    remote_send(REMOTE_RESET,2,0x12345678U,clock_value);
    remote_expect(2,REMOTE_QUEUED,REMOTE_NONE);cycle();
    assert(g_reset_waiting);cycle();remote_expect(2,REMOTE_EXECUTED,REMOTE_NONE);
    assert(g_drive.state==S3_DRIVE_OFF&&s3_integration_status(&status));
    sequence=status.command_sequence;
    remote_send(REMOTE_START,3,0x12345678U,clock_value);
    remote_expect(3,REMOTE_QUEUED,REMOTE_NONE);
    assert(g_drive.state==S3_DRIVE_OFF);cycle();remote_expect(3,REMOTE_EXECUTED,REMOTE_NONE);
    assert(g_drive.state==S3_DRIVE_FOLLOW&&s3_integration_status(&status));
    assert(status.command_sequence==sequence);
    remote_send(REMOTE_START,3,0x12345678U,clock_value);
    remote_expect(3,REMOTE_REJECTED,REMOTE_REPLAY);
    remote_send(REMOTE_STOP,4,0x12345678U,0); /* STOP ignores stale time */
    remote_expect(4,REMOTE_QUEUED,REMOTE_NONE);cycle();remote_expect(4,REMOTE_EXECUTED,REMOTE_NONE);
    remote_send(REMOTE_START,5,0x12345678U,clock_value);
    remote_expect(5,REMOTE_QUEUED,REMOTE_NONE);
    remote_send(REMOTE_STOP,6,0x12345678U,clock_value);
    remote_expect(5,REMOTE_CANCELLED,REMOTE_SUPERSEDED);remote_expect(6,REMOTE_QUEUED,REMOTE_NONE);
    cycle();remote_expect(6,REMOTE_EXECUTED,REMOTE_NONE);assert(g_drive.state==S3_DRIVE_OFF);
    remote_send(REMOTE_START,5,0x12345678U,clock_value);
    remote_expect(5,REMOTE_REJECTED,REMOTE_REPLAY); /* late START after STOP */
    remote_send(REMOTE_START,7,0x12345678U,clock_value);
    remote_expect(7,REMOTE_QUEUED,REMOTE_NONE);
    cmd.kind=S3_CONTROL_STOP;assert(s3_integration_submit(&cmd));
    remote_expect(7,REMOTE_CANCELLED,REMOTE_SUPERSEDED);cycle();assert(g_drive.state==S3_DRIVE_OFF);
    remote_send(REMOTE_START,8,0x12345678U,clock_value);
    remote_expect(8,REMOTE_QUEUED,REMOTE_NONE);
    clock_value+=REMOTE_MAX_AGE_US;cycle();remote_expect(8,REMOTE_REJECTED,REMOTE_STALE);
    assert(g_drive.state==S3_DRIVE_OFF);
    remote_send(REMOTE_START,9,0x12345678U,clock_value-REMOTE_MAX_AGE_US-1);
    remote_expect(9,REMOTE_REJECTED,REMOTE_STALE);
    remote_send(REMOTE_START,10,0x12345678U,clock_value);
    remote_expect(10,REMOTE_QUEUED,REMOTE_NONE);
    s3_remote_link(0,false);remote_expect(10,REMOTE_CANCELLED,REMOTE_SUPERSEDED);
    s3_remote_link(0x87654321U,true);cycle();assert(g_drive.state==S3_DRIVE_OFF);
    remote_send(REMOTE_START,11,0x12345678U,clock_value);
    remote_expect(11,REMOTE_REJECTED,REMOTE_STALE);
    {
        uint8_t p[REMOTE_STATUS_SIZE];
        assert(s3_remote_status_take(p,sizeof p)==sizeof p&&remote_get32(p+1)==0x87654321U);
        assert((p[13]&REMOTE_AVAILABLE)!=0U);
        assert(s3_remote_status_take(p,sizeof p)==0U);
        clock_value+=500000U;
        assert(s3_remote_status_take(p,sizeof p)==sizeof p&&p[13]==0); /* stalled owner unavailable */
    }
    s3_remote_physical_stop(true);
    remote_send(REMOTE_START,1,0x87654321U,clock_value);
    remote_expect(1,REMOTE_REJECTED,REMOTE_NOT_READY);
    remote_send(REMOTE_RESET,2,0x87654321U,clock_value);
    remote_expect(2,REMOTE_REJECTED,REMOTE_NOT_READY);
    s3_remote_physical_stop(false);
    remote_send(REMOTE_START,3,0x87654321U,clock_value);
    remote_expect(3,REMOTE_QUEUED,REMOTE_NONE);
    s3_remote_physical_stop(true);cycle();remote_expect(3,REMOTE_REJECTED,REMOTE_STALE);
    assert(g_drive.state==S3_DRIVE_OFF);s3_remote_physical_stop(false);
    /* STOP between precheck and execute's state change: no success reply or
     * wheel request is allowed to escape before the next STOP sampling tick. */
    const unsigned before_velocity=velocity_requests;
    during_start=stop_during_start;
    remote_send(REMOTE_START,4,0x87654321U,clock_value);
    remote_expect(4,REMOTE_QUEUED,REMOTE_NONE);cycle();
    remote_expect(5,REMOTE_QUEUED,REMOTE_NONE);remote_expect(4,REMOTE_REJECTED,REMOTE_STALE);
    assert(g_drive.state==S3_DRIVE_OFF&&velocity_requests==before_velocity);
    cycle();remote_expect(5,REMOTE_EXECUTED,REMOTE_NONE);
    during_start=reconnect_during_start;
    remote_send(REMOTE_START,6,0x87654321U,clock_value);
    remote_expect(6,REMOTE_QUEUED,REMOTE_NONE);cycle();
    {
        uint8_t p[REMOTE_REPLY_SIZE];
        assert(s3_remote_reply_take(p,sizeof p)==sizeof p&&remote_get32(p+2)==6);
        assert(p[6]==REMOTE_REJECTED&&p[7]==REMOTE_STALE&&remote_get32(p+8)==0x87654321U);
    }
    assert(g_drive.state==S3_DRIVE_OFF&&velocity_requests==before_velocity);
    /* An asynchronous RESET cancelled on reconnect stays a remote operation
     * during safe owner housekeeping, never a spurious GP acknowledgement. */
    remote_send(REMOTE_START,1,0xabcddcbaU,clock_value);
    remote_expect(1,REMOTE_QUEUED,REMOTE_NONE);cycle();remote_expect(1,REMOTE_EXECUTED,REMOTE_NONE);
    fb_fault=true;cycle();assert(g_drive.state==S3_DRIVE_FAULT);
    remote_send(REMOTE_RESET,2,0xabcddcbaU,clock_value);
    remote_expect(2,REMOTE_QUEUED,REMOTE_NONE);cycle();assert(g_reset_waiting&&g_reset_remote);
    assert(s3_integration_status(&status));sequence=status.command_sequence;
    s3_remote_link(0,false);
    {
        uint8_t p[REMOTE_REPLY_SIZE];
        assert(s3_remote_reply_take(p,sizeof p)==sizeof p&&remote_get32(p+2)==2);
        assert(p[6]==REMOTE_CANCELLED&&remote_get32(p+8)==0xabcddcbaU);
    }
    s3_remote_link(0x43211234U,true);cycle();
    assert(s3_integration_status(&status)&&status.command_sequence==sequence);
    assert(g_drive.state==S3_DRIVE_OFF&&!g_reset_waiting&&!g_reset_remote);
    {
        uint8_t p[REMOTE_REPLY_SIZE];assert(s3_remote_reply_take(p,sizeof p)==0);
        uint8_t vector[REMOTE_COMMAND_SIZE]={3,REMOTE_START};
        const uint8_t expected[]={0x03,0x01,0x78,0x56,0x34,0x12,0x44,0x33,0x22,0x11,0xdd,0xcc,0xbb,0xaa};
        remote_put32(vector+2,0x12345678U);remote_put32(vector+6,0x11223344U);remote_put32(vector+10,0xaabbccddU);
        assert(!memcmp(vector,expected,sizeof vector));
        const uint8_t reply_vector[]={0x03,0x01,0x78,0x56,0x34,0x12,0x01,0x00,0x44,0x33,0x22,0x11};
        const remote_request_t r={.action=REMOTE_START,.id=0x12345678U,.epoch=0x11223344U,.valid=true};
        remote_reply(&r,REMOTE_EXECUTED,REMOTE_NONE);
        assert(s3_remote_reply_take(p,sizeof p)==sizeof p&&!memcmp(p,reply_vector,sizeof p));
        assert(remote_newer(1U,0xffffffffU)&&!remote_newer(0xfffffffeU,1U));
        uint8_t status_vector[REMOTE_STATUS_SIZE];
        const uint8_t status_expected[]={0x03,0x44,0x33,0x22,0x11,0xdd,0xcc,0xbb,0xaa,0x78,0x56,0x34,0x12,0x03};
        clock_value=0xaabbccddU;g_remote.epoch=0x11223344U;g_remote.last_id=0x12345678U;
        g_remote.flags=3;g_remote.sampled_us=clock_value;g_remote.published=false;
        assert(s3_remote_status_take(status_vector,sizeof status_vector)==sizeof status_vector);
        assert(!memcmp(status_vector,status_expected,sizeof status_vector));
    }
    puts("PASS: remote queued/executed/fault gates, RESET acknowledgement, duplicate/late/expired START, remote/GP STOP priority and reconnect cancellation");
    puts("PASS: production runtime STOP precedence, packed status, calibration, signed errors, map freeze, per-run timing/fault interval, one-press asynchronous RESET, cancellation and timeout");
    return 0;
}
