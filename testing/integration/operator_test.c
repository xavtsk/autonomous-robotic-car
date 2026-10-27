/* End-to-end host test of the operator controls: simulated GP20/GP21 presses
 * -> operator_logic -> the PRODUCTION s3_integration runtime (fake ADC/motion
 * boundaries, as in runtime_test.c) -> published status -> operator_logic.
 * Proves the button walkthrough really calibrates, starts and stops through
 * the single controller owner. Not a timing or hardware test. */
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
#include "../../app_program/controller/operator_logic.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define TA_HLNG 2U
#define TSK_SELF 0
typedef struct { unsigned cycatr; FP cychdr; unsigned cyctim,cycphs; } T_CCYC;
static jmp_buf boundary;
static unsigned sleeps,locked,starts_requested,stops;
static uint32_t clock_value;
static uint16_t raw[3]={150,150,150};
static bool fb_braked=true;
static uint8_t fb_command_id; /* last accepted velocity command, echoed back like Buddy 2 */
static bool fb_fault;
ID tk_cre_mtx(const T_CMTX *m) { (void)m; return 1; }
ER tk_loc_mtx(ID id,TMO t) { (void)t;assert(id==1&&!locked);locked=1;return E_OK; }
ER tk_unl_mtx(ID id) { assert(id==1&&locked);locked=0;return E_OK; }
ID tk_cre_cyc(const T_CCYC *c) { (void)c; return 2; }
ER tk_stp_cyc(ID id) { (void)id;return E_OK; }
ER tk_del_cyc(ID id) { (void)id;return E_OK; }
ER tk_del_mtx(ID id) { (void)id;return E_OK; }
ER tk_del_tsk(ID id) { (void)id;return E_OK; }
ER tk_ter_tsk(ID id) { (void)id;return E_OK; }
ER tk_sta_cyc(ID id) { (void)id;return E_OK; }
ER tk_wup_tsk(ID id) { (void)id;return E_OK; }
ER tk_can_wup(ID id) { (void)id;return E_OK; }
ER tk_slp_tsk(TMO t) { (void)t;assert(!locked);if(++sleeps==2)longjmp(boundary,1);return E_OK; }
ER tk_dly_tsk(unsigned ms) { (void)ms;return E_OK; }
ID app_start_task(FP f,PRI p,UW core,const char *name)
{ (void)f;(void)p;(void)core;(void)name;return 3; }
uint32_t runtime_read_clock(uint32_t address) { (void)address;return clock_value; }
int32_t ir_sensor_init(const s3_acquisition_config_t *cfg) { (void)cfg;return E_OK; }
int32_t ir_sensor_read_frame(s3_sensor_frame_t *f)
{ *f=(s3_sensor_frame_t){.timestamp_us=clock_value};
  for(unsigned i=0;i<3;i++)f->sensor[i]=(s3_raw_sample_t){clock_value,(uint16_t)(raw[i]+clock_value/10000U%5U),0,true,true};
  return E_OK; }
bool motion_get_feedback(motion_feedback_t *f)
{ *f=(motion_feedback_t){.timestamp_us=clock_value,.epoch=1,.valid=true,.braked=fb_braked,.command_id=fb_command_id,.fault=fb_fault?MOTION_FAULT_STALL:MOTION_FAULT_NONE};return true; }
motion_result_t motion_request_velocity(float l,float r) { (void)l;(void)r;fb_command_id=1;return MOTION_OK; }
motion_result_t motion_request_drive(float m,float v) { (void)m;(void)v;return MOTION_OK; }
motion_result_t motion_request_manoeuvre(manoeuvre_t m) { (void)m;return MOTION_OK; }
void stop(void) { stops++;fb_command_id=0; }
bool motion_get_report(motion_report_t *r) { (void)r;return false; }
uint8_t motion_command_id(void) { return 1; }
bool motion_reset(void) { return true; }
void motion_navigation_heartbeat(bool enabled) { (void)enabled; }
bool vehicle_controller_prepare_start(s3_navigation_t *n) { (void)n;starts_requested++;return true; }
bool vehicle_controller_apply_mapping(const s3_nav_action_t m[26]) { (void)m;return true; }
void vehicle_controller_cancel_start(void) {}
void vehicle_controller_commit_start(void) {}
void vehicle_controller_observe(const s3_integration_status_t *s) { (void)s; }

/* Console capture: everything the operator would read on USB. */
static char console[1<<16];
static size_t console_len;
int tm_printf(const UB *format,...)
{
    va_list ap; int n;
    assert(!locked);
    va_start(ap,format);
    n=vsnprintf(console+console_len,sizeof(console)-console_len,(const char*)format,ap);
    va_end(ap);
    if(n>0&&console_len+(size_t)n<sizeof(console)) console_len+=(size_t)n;
    return n;
}
#include "../../app_program/controller/s3_integration.c"

static operator_t op;
static op_button_t go_b,stop_b;
static bool go_down,stop_down;
static s3_integration_status_t st;
static uint32_t now_ms(void) { return clock_value/1000U; }
/* One 10 ms period: controller cycle, then the operator poll. */
static void period(void)
{
    bool stop_edge=false,go_edge=false; op_press_t go;
    sleeps=0;clock_value+=10000;
    if(setjmp(boundary)==0)sample(0,NULL);
    assert(!locked);
    (void)op_button_update(&stop_b,stop_down,now_ms(),&stop_edge);
    go=op_button_update(&go_b,go_down,now_ms(),&go_edge);
    operator_step(&op,now_ms(),go,op_button_held(&stop_b),op_button_held(&go_b),
                  s3_integration_status(&st)?&st:0);
}
static void run_ms(unsigned ms) { for(unsigned t=0;t<ms;t+=10)period(); }
static void press_go(unsigned held_ms) { go_down=true;run_ms(held_ms);go_down=false;run_ms(60); }
static void press_stop(void) { stop_down=true;run_ms(60);stop_down=false;run_ms(60); }
static size_t mark(void) { return console_len; }
static bool said_since(size_t from,const char *text) { console[console_len]=0;return strstr(console+from,text)!=NULL; }
static void set_all(uint16_t v) { raw[0]=raw[1]=raw[2]=v; }
static void calibrate_all(void)
{
    /* order: LEFT, RIGHT, BARCODE; each WHITE then BLACK */
    static const s3_sensor_id_t order[3]={S3_SENSOR_LEFT,S3_SENSOR_RIGHT,S3_SENSOR_BARCODE};
    for(unsigned i=0;i<3;i++){
        set_all(300);press_go(80);run_ms(1200);
        set_all(300);raw[order[i]]=2500;press_go(80);run_ms(1300);
    }
}

int main(void)
{
    const op_port_t port={s3_integration_submit,tm_printf};
    size_t m;
    assert(s3_integration_init()==E_OK);
    assert(operator_init(&op,&port));
    op_button_init(&go_b);op_button_init(&stop_b);

    /* 0. GP21 held from power-on: STOP is sent once, and no GP20 press is
     *    accepted while it is held (GP21 never "arms", but holding counts). */
    stop_down=true;run_ms(300);
    for(int k=0;k<3;k++)press_go(80);
    run_ms(500);
    assert(said_since(0,"STOP sent."));
    assert(strstr(strstr(console,"STOP sent.")+1,"STOP sent.")==NULL); /* once, not every poll */
    assert(said_since(0,"Ignored GP20: release GP21 first."));
    assert(!said_since(0,"Sampling for 1 s"));
    stop_down=false;run_ms(300);

    /* 1. GP20 held at power-up must not count as a press. */
    op_button_init(&go_b);m=mark();
    go_down=true;run_ms(500);go_down=false;run_ms(200);
    assert(said_since(0,"Calibration 1/6: put the front-LEFT line sensor (GP27) over plain WHITE"));
    assert(!said_since(m,"Sampling for 1 s"));

    /* 2. Full walkthrough calibrates all three sensors for real. */
    m=mark();calibrate_all();
    assert(said_since(m,"Calibration 2/6: put the front-LEFT line sensor (GP27) over the BLACK line"));
    assert(said_since(m,"front-RIGHT line sensor (GP26) calibrated."));
    assert(said_since(m,"BARCODE sensor (GP28) calibrated."));
    assert(said_since(m,"All 3 sensors calibrated."));
    assert(said_since(m,"READY: put the car on the line"));
    assert(g_drive.navigation.accepted_calibration==S3_ALL_SENSORS);

    /* P1a. GP21 held, GP20 pressed and released: nothing starts. */
    set_all(300);m=mark();
    stop_down=true;run_ms(100);press_go(80);run_ms(4000);
    assert(said_since(m,"Ignored GP20: release GP21 first.")&&!said_since(m,"Starting in"));
    assert(starts_requested==0&&g_drive.state==S3_DRIVE_OFF);
    /* P1b. GP21 held through a whole countdown period: START never sent. */
    stop_down=false;run_ms(100);m=mark();press_go(80);run_ms(500);
    assert(said_since(m,"Starting in 3..."));
    stop_down=true;run_ms(4000);
    assert(said_since(m,"Start cancelled.")&&!said_since(m,"RUNNING"));
    assert(starts_requested==0&&g_drive.state==S3_DRIVE_OFF);
    stop_down=false;run_ms(100);
    /* P1c. Both pressed together, either release order: GP20 is ignored. */
    m=mark();go_down=true;stop_down=true;run_ms(200);stop_down=false;run_ms(100);go_down=false;run_ms(4000);
    go_down=true;stop_down=true;run_ms(200);go_down=false;run_ms(100);stop_down=false;run_ms(4000);
    assert(!said_since(m,"Starting in")&&starts_requested==0);

    /* 3. STOP during the countdown cancels; START is never requested. */
    set_all(300);m=mark();press_go(80);run_ms(1000);
    assert(said_since(m,"Starting in 3..."));
    press_stop();
    assert(said_since(m,"STOP sent.")&&said_since(m,"Start cancelled."));
    run_ms(3000);assert(starts_requested==0&&g_drive.state==S3_DRIVE_OFF);

    /* 4. Buddy 2 not braked: START refused with a readable reason. */
    fb_braked=false;m=mark();press_go(80);run_ms(3300);
    assert(said_since(m,"Starting in 1...")&&said_since(m,"START refused: state=OFF"));
    assert(g_drive.state==S3_DRIVE_OFF);fb_braked=true;

    /* 5. Real START; sensors all on white -> controller halts on line loss,
     *    and the operator reports why. */
    m=mark();press_go(80);run_ms(3300);
    assert(said_since(m,"RUNNING. GP21 = STOP."));
    run_ms(2000);
    assert(g_drive.state==S3_DRIVE_HALTED);
    assert(said_since(m,"Run ended: HALTED - line lost."));

    /* 6. START again from HALTED, then GP21 stops the run. */
    /* Both line sensors partly over the tape: line present, no junction. */
    raw[S3_SENSOR_LEFT]=900;raw[S3_SENSOR_RIGHT]=900;
    m=mark();press_go(80);run_ms(3300);
    assert(said_since(m,"RUNNING."));
    run_ms(1000);
    if(g_drive.state!=S3_DRIVE_FOLLOW)printf("state=%d reason=%d\n",g_drive.state,g_drive.reason);
    assert(g_drive.state==S3_DRIVE_FOLLOW&&!said_since(m,"Run ended:"));
    { unsigned before=stops;press_stop();assert(stops>before); }
    assert(g_drive.state==S3_DRIVE_OFF);
    assert(said_since(m,"Run ended: OFF - ok/stopped by operator."));

    /* P3a. Motor fault while driving -> FAULT. Holding GP20 must NOT enter a
     *      calibration the controller would refuse; it asks for RESET. */
    m=mark();press_go(80);run_ms(3300);assert(said_since(m,"RUNNING."));
    fb_fault=true;run_ms(100);
    assert(g_drive.state==S3_DRIVE_FAULT&&said_since(m,"Run ended: FAULT - motor/encoder fault."));
    assert(said_since(m,"FAULT: motor/encoder fault. Fix the cause, then press GP20 to reset."));
    m=mark();press_go(2100);run_ms(600);       /* long press, fault persists through reset timeout */
    assert(said_since(m,"Reset refused")&&!said_since(m,"Recalibrating")&&!said_since(m,"Sampling refused"));
    assert(g_drive.state==S3_DRIVE_FAULT);
    fb_fault=false;m=mark();press_go(80);run_ms(200);
    assert(said_since(m,"Fault reset.")&&g_drive.state==S3_DRIVE_OFF);
    /* P3b. FAULT while at a calibration prompt: GP20 resets, then calibration
     *      continues (white-box fault injection; the reset path is real). */
    m=mark();press_go(2100);assert(said_since(m,"Calibration 1/6"));
    g_drive.state=S3_DRIVE_FAULT;g_drive.reason=S3_DRIVE_SENSOR_FAULT;run_ms(100);
    m=mark();run_ms(5100);
    assert(said_since(m,"FAULT: IR sensor fault. Calibration is blocked. Press GP20 to reset"));
    press_go(80);run_ms(200);
    assert(said_since(m,"Fault reset.")&&said_since(m,"Calibration 1/6")&&!said_since(m,"Sampling refused"));
    assert(g_drive.state==S3_DRIVE_OFF);
    /* finish this calibration pass so READY is reached again */
    calibrate_all();assert(said_since(m,"All 3 sensors calibrated."));

    /* 7. Hold GP20 2 s in READY -> recalibration starts at step 1. */
    set_all(300);m=mark();press_go(2100);
    assert(said_since(m,"Recalibrating all sensors.")&&said_since(m,"Calibration 1/6"));

    /* 8. Overlapping white/black is rejected and the same sensor is redone. */
    m=mark();set_all(1000);press_go(80);run_ms(1200);press_go(80);run_ms(1300);
    assert(said_since(m,"front-LEFT line sensor (GP27) REJECTED"));
    assert(said_since(m,"Calibration 1/6"));

    /* 9. STOP while sampling returns to the same step. */
    m=mark();press_go(80);run_ms(200);press_stop();run_ms(1500);
    assert(said_since(m,"STOP sent.")&&said_since(m,"Calibration 1/6"));
    assert(!said_since(m,"Calibration 2/6"));

    if(getenv("OP_TEST_VERBOSE")){console[console_len]=0;fputs(console,stdout);}
    puts("PASS: operator buttons -> production runtime: GP21 held at power-on/before/during countdown and both-button release orders block START, power-up GP20 press ignored, full 3-sensor calibration, countdown + STOP cancel, refused START reason, START/line-lost report, restart + STOP, FAULT: long press asks for RESET (no calibration trap) and reset from a calibration prompt, long-press recalibration, overlap rejection, STOP during sampling");
    return 0;
}
