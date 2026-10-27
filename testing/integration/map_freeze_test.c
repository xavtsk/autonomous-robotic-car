/* Map-freeze ownership with the REAL pieces wired together: Buddy 1 map
 * storage (command.c + telemetry_codec.c), the facade (vehicle_controller.c),
 * the production runtime (s3_integration.c) and s3_drive/navigation. Only the
 * RTOS, ADC, motor and telemetry-publish boundaries are fake.
 * Contract: the map freezes at the first accepted START and stays frozen until
 * reboot; a rejected START may only undo a freeze that THIS attempt took. */
#define APP_H
#define APP_S3_MOTION 1
#define APP_DIAGNOSTICS 0
#define APP_TEST 0
#define S3_BUILTIN_CALIBRATION 0 /* these tests start uncalibrated */
#include "controller_test_kernel.h"
#define LOCAL static
#define EXPORT
#define TMO_POL 0
#define E_SYS (-5)
ER tk_snd_mbf(ID id, const void *msg, INT size, TMO timeout);
INT tk_rcv_mbf(ID id, void *msg, TMO timeout);
#include "../../app_program/controller/s3_integration.h"
#include "../../app_program/controller/vehicle_controller.h"
#include "../../app_program/subsystem1/subsystem1.h"
#include "../../app_program/subsystem2/motion_pack.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#define TA_HLNG 2U
#define TSK_SELF 0
typedef struct { unsigned cycatr; FP cychdr; unsigned cyctim,cycphs; } T_CCYC;
static jmp_buf boundary;
static unsigned sleeps,locked[8];
static uint32_t clock_value;
static bool fb_braked=true;
static uint8_t fb_command_id;
ID mtx_buddy1_state=5, mbf_buddy1_commands=6, mbf_buddy1_outbox=7;
ID tk_cre_mtx(const T_CMTX *m) { (void)m;return 1; }
ER tk_loc_mtx(ID id,TMO t) { (void)t;assert(id>0&&id<8&&!locked[id]);locked[id]=1;return E_OK; }
ER tk_unl_mtx(ID id) { assert(id>0&&id<8&&locked[id]);locked[id]=0;return E_OK; }
ID tk_cre_cyc(const T_CCYC *c) { (void)c;return 2; }
ER tk_stp_cyc(ID id) { (void)id;return E_OK; }
ER tk_del_cyc(ID id) { (void)id;return E_OK; }
ER tk_del_mtx(ID id) { (void)id;return E_OK; }
ER tk_del_tsk(ID id) { (void)id;return E_OK; }
ER tk_ter_tsk(ID id) { (void)id;return E_OK; }
ER tk_sta_cyc(ID id) { (void)id;return E_OK; }
ER tk_wup_tsk(ID id) { (void)id;return E_OK; }
ER tk_can_wup(ID id) { (void)id;return E_OK; }
ER tk_slp_tsk(TMO t) { (void)t;if(++sleeps==2)longjmp(boundary,1);return E_OK; }
ER tk_dly_tsk(unsigned ms) { (void)ms;return E_OK; }
ER tk_snd_mbf(ID id,const void *msg,INT size,TMO t) { (void)id;(void)msg;(void)size;(void)t;return E_OK; }
INT tk_rcv_mbf(ID id,void *msg,TMO t) { (void)id;(void)msg;(void)t;return 0; }
ID app_start_task(FP f,PRI p,UW core,const char *name) { (void)f;(void)p;(void)core;(void)name;return 3; }
int tm_printf(const UB *format,...) { (void)format;return 0; }
uint32_t runtime_read_clock(uint32_t address) { (void)address;return clock_value; }
int32_t ir_sensor_init(const s3_acquisition_config_t *cfg) { (void)cfg;return E_OK; }
int32_t ir_sensor_read_frame(s3_sensor_frame_t *f)
{ *f=(s3_sensor_frame_t){.timestamp_us=clock_value};
  /* line sensors partly dark: line present, no junction, no barcode */
  f->sensor[0]=(s3_raw_sample_t){clock_value,150,0,true,true};
  f->sensor[1]=(s3_raw_sample_t){clock_value,(uint16_t)(400+clock_value/10000U%3U),0,true,true};
  f->sensor[2]=(s3_raw_sample_t){clock_value,(uint16_t)(400+clock_value/10000U%3U),0,true,true};
  return E_OK; }
bool motion_get_feedback(motion_feedback_t *f)
{ *f=(motion_feedback_t){.timestamp_us=clock_value,.epoch=1,.valid=true,.braked=fb_braked,.command_id=fb_command_id};return true; }
motion_result_t motion_request_velocity(float l,float r) { (void)l;(void)r;fb_command_id=1;return MOTION_OK; }
motion_result_t motion_request_manoeuvre(manoeuvre_t m) { (void)m;return MOTION_OK; }
void stop(void) { fb_command_id=0; }
bool motion_get_report(motion_report_t *r) { (void)r;return false; }
uint8_t motion_command_id(void) { return 1; }
bool motion_reset(void) { return true; }
void motion_navigation_heartbeat(bool enabled) { (void)enabled; }
bool motion_get_status(motion_status_t *out) { *out=(motion_status_t){0};return false; }
/* Telemetry publish boundary (subsystem1.c) */
ER buddy1_report_barcode(UB l) { (void)l;return E_OK; }
ER buddy1_update_faults(UB c,UB a) { (void)c;(void)a;return E_OK; }
ER buddy1_report_robot_state(BUDDY1_ROBOT_STATE s) { (void)s;return E_OK; }
ER buddy1_invalidate(UB t) { (void)t;return E_OK; }
ER buddy1_report_motion(H l,H r) { (void)l;(void)r;return E_OK; }
ER buddy1_queue_publish(UB t,const UB *p,UH n) { (void)t;(void)p;(void)n;return E_OK; }
BOOL buddy1_snapshot_packet(UB p[BUDDY1_PAYLOAD_MAX]) { (void)p;return TRUE; }

#include "../../app_program/subsystem1/command.c"
#include "../../app_program/controller/vehicle_controller.c"
#include "../../app_program/controller/s3_integration.c"

static void cycle(void)
{ sleeps=0;clock_value+=10000;if(setjmp(boundary)==0)sample(0,NULL);
  for(unsigned i=0;i<8;i++)assert(!locked[i]); }
static bool submit(s3_control_kind_t kind)
{ s3_integration_status_t s; uint32_t seq;
  assert(s3_integration_status(&s));seq=s.command_sequence;
  const s3_control_request_t r={.kind=kind};
  assert(s3_integration_submit(&r));cycle();
  assert(s3_integration_status(&s)&&s.command_sequence==seq+1U);
  return s.command_ok; }
static ER try_map(char right)
{ const BUDDY1_BARCODE_MAP m={'A','B',(uint8_t)right,'D'};
  return buddy1_apply_barcode_map(&m); }

int main(void)
{
    assert(vehicle_controller_init()==E_OK);
    assert(s3_integration_init()==E_OK);
    cycle();cycle();
    g_drive.navigation.accepted_calibration=S3_ALL_SENSORS; /* calibration is tested elsewhere */
    cycle();
    assert(try_map('Y')==E_OK);                       /* pre-run: Y = RIGHT */

    /* 1. First START refused by Buddy 2 (not braked): no run used the map,
     *    so the freeze this attempt took is undone. */
    fb_braked=false;assert(!submit(S3_CONTROL_START));
    assert(g_drive.state==S3_DRIVE_OFF);
    assert(try_map('Z')==E_OK&&try_map('Y')==E_OK);   /* still configurable */

    /* 2. Accepted START freezes the map and installs Y = RIGHT. */
    fb_braked=true;assert(submit(S3_CONTROL_START));
    assert(g_drive.state==S3_DRIVE_FOLLOW);
    assert(g_drive.navigation.config.map['Y'-'A']==S3_NAV_RIGHT);
    assert(try_map('Z')==E_OBJ);

    /* 3. Duplicate START while driving: refused, map stays frozen, the run
     *    keeps its installed map. */
    assert(!submit(S3_CONTROL_START));
    assert(g_drive.state==S3_DRIVE_FOLLOW);
    assert(try_map('Z')==E_OBJ);
    assert(g_drive.navigation.config.map['Y'-'A']==S3_NAV_RIGHT);

    /* 4. STOP: frozen until reboot. */
    assert(submit(S3_CONTROL_STOP));
    assert(g_drive.state==S3_DRIVE_OFF&&try_map('Z')==E_OBJ);

    /* 5. Later restart refused by Buddy 2: must NOT undo the earlier freeze. */
    fb_braked=false;assert(!submit(S3_CONTROL_START));
    assert(try_map('Z')==E_OBJ);

    /* 6. Restart accepted with the same frozen map. */
    fb_braked=true;assert(submit(S3_CONTROL_START));
    assert(g_drive.state==S3_DRIVE_FOLLOW&&g_drive.navigation.config.map['Y'-'A']==S3_NAV_RIGHT);
    assert(try_map('Z')==E_OBJ);

    puts("PASS: real map storage + facade + runtime: refused first START reopens, accepted START freezes, duplicate START while driving and refused restart keep the freeze, restart keeps the installed map");
    return 0;
}
