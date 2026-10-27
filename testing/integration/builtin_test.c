/* Built-in calibration at boot (S3_BUILTIN_CALIBRATION=1), through the
 * production runtime and operator logic. Derived from operator_test.c.
 * Original header: End-to-end host test of the operator controls: simulated GP20/GP21 presses
 * -> operator_logic -> the PRODUCTION s3_integration runtime (fake ADC/motion
 * boundaries, as in runtime_test.c) -> published status -> operator_logic.
 * Proves the button walkthrough really calibrates, starts and stops through
 * the single controller owner. Not a timing or hardware test. */
#define APP_H
#define APP_S3_MOTION 1
#define APP_DIAGNOSTICS 0
#define APP_TEST 0
#define S3_BUILTIN_CALIBRATION 1
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
static float last_l,last_r;
motion_result_t motion_request_velocity(float l,float r) { last_l=l;last_r=r;fb_command_id=1;return MOTION_OK; }
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
int main(void)
{
    const op_port_t port={s3_integration_submit,tm_printf};
    size_t m;
    assert(s3_integration_init()==E_OK);
    /* All three accepted through the normal validation path at init. */
    assert(g_drive.navigation.accepted_calibration==S3_ALL_SENSORS);
    {
        const ir_calibration_t *l=&g_drive.navigation.config.calibration[S3_SENSOR_LEFT];
        const ir_calibration_t *r=&g_drive.navigation.config.calibration[S3_SENSOR_RIGHT];
        const ir_calibration_t *b=&g_drive.navigation.config.calibration[S3_SENSOR_BARCODE];
        assert(l->white_raw>=130&&l->white_raw<=180&&l->black_raw>=1560&&l->black_raw<=1640);
        assert(r->white_raw>=155&&r->white_raw<=228&&r->black_raw>=3130&&r->black_raw<=3250);
        assert(b->white_raw>=130&&b->white_raw<=186&&b->black_raw>=1880&&b->black_raw<=1930);
    }
    assert(operator_init(&op,&port));
    op_button_init(&go_b);op_button_init(&stop_b);
    run_ms(300);
    /* Straight to READY: no 6-step walkthrough. */
    assert(said_since(0,"Using built-in calibration."));
    assert(said_since(0,"READY: put the car on the line"));
    assert(!said_since(0,"Calibration 1/6"));
    /* Readings with the built-in values: each line sensor half on the tape
     * (left raw ~880 of 150..1600, right ~1700 of 190..3190) -> line present,
     * no junction, so a START drives. */
    raw[S3_SENSOR_LEFT]=880;raw[S3_SENSOR_RIGHT]=1700;raw[S3_SENSOR_BARCODE]=150;
    run_ms(200);
    assert(g_drive.output.line.present&&!g_drive.output.line.junction_candidate);
    m=mark();press_go(80);run_ms(3300);
    assert(said_since(m,"RUNNING."));
    run_ms(500);
    assert(g_drive.state==S3_DRIVE_FOLLOW);
    press_stop();
    assert(g_drive.state==S3_DRIVE_OFF&&said_since(m,"Run ended: OFF"));
    /* Minimum wheel speed: a slow positive request is raised, others pass. */
    assert(velocity_with_floor(0.159f,0.041f)==MOTION_OK&&last_l==0.159f&&last_r==S3_MIN_WHEEL_MPS);
    assert(velocity_with_floor(0.10f,0.10f)==MOTION_OK&&last_l==0.10f&&last_r==0.10f);
    assert(velocity_with_floor(0.0f,-0.05f)==MOTION_OK&&last_l==0.0f&&last_r==-0.05f);
    /* Hand recalibration still available and replaces the built-in values. */
    m=mark();press_go(2100);
    assert(said_since(m,"Recalibrating all sensors.")&&said_since(m,"Calibration 1/6"));
    if(getenv("OP_TEST_VERBOSE")){console[console_len]=0;fputs(console,stdout);}
    puts("PASS: built-in calibration accepted at boot via normal validation, operator skips walkthrough, START/STOP work, minimum wheel speed floor, long press still recalibrates");
    return 0;
}
