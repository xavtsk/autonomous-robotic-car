/* Production S4 task/transport with fake RTOS and sensor boundaries. */
#include <assert.h>
#include <stdbool.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#define TMO_POL 0
#define TA_HLNG 2
#define TA_RNG3 0
#define EXPORT
typedef struct { FP task; unsigned itskpri,stksz,tskatr; } T_CTSK;
static uint32_t clock_us=1000;
static uint32_t feedback_read_delay_us,feedback_age_us,feedback_future_us;
static unsigned owner_cycles_left;
static bool locked,fail_lock;
static jmp_buf finish;
static void (*entry)(INT,void *);
ER tk_dly_tsk(unsigned ms);
void tk_ext_tsk(void) { assert(!"Unexpected task exit"); }
ID tk_cre_tsk(const T_CTSK *c)
{ assert(c->itskpri==18 && c->stksz==4096);entry=(void (*)(INT,void *))c->task;return 2; }
ER tk_sta_tsk(ID id,INT value) { (void)id;(void)value;return E_OK; }
ER tk_del_tsk(ID id) { (void)id;return E_OK; }
ID tk_cre_mtx(const T_CMTX *m) { assert(m->mtxatr==TA_INHERIT);return 1; }
ER tk_loc_mtx(ID id,TMO timeout)
{ (void)timeout;assert(id==1&&!locked);if(fail_lock)return E_TMOUT;locked=true;return E_OK; }
ER tk_unl_mtx(ID id) { assert(id==1&&locked);locked=false;return E_OK; }
int tm_printf(const UB *format,...) { (void)format;return 0; }
uint32_t runtime_read_clock(uint32_t address) { (void)address;return clock_us; }
#include "../../app_program/subsystem4/subsystem4.h"
#include "../../app_program/subsystem2/motion.h"
bool motion_get_feedback(motion_feedback_t *out)
{
    /* Model a motion update while S4 waits for the feedback mutex. */
    clock_us+=feedback_read_delay_us;
    *out=(motion_feedback_t){.valid=true,.braked=true,
        .timestamp_us=clock_us-feedback_age_us+feedback_future_us};
    return true;
}
bool motion_is_calibrated(void) { return true; }
static terrain_odometry_t observed_odometry;
static terrain_hump_geometry_t observed_geometry;
static float observed_track_m;
static void observe_monitor_process(terrain_monitor_t *monitor,const imu_sample_t *sample,
                                    const terrain_odometry_t *odometry)
{
    observed_odometry=*odometry;
    observed_geometry=monitor->hump_geometry;observed_track_m=monitor->wheel_track_m;
    terrain_monitor_process(monitor,sample,odometry);
}
static bool fake_sensor_init(const imu_bus_t *bus) { (void)bus;return true; }
static bool fake_sensor_read(const imu_bus_t *bus,uint32_t now,imu_sample_t *s)
{ (void)bus;*s=(imu_sample_t){.timestamp_us=now,.valid=true,.accel_g={0,0,1}};return true; }
#define imu_sensor_init fake_sensor_init
#define imu_sensor_read fake_sensor_read
#define terrain_monitor_process observe_monitor_process
/* The unchanged BSP dev_i2c.h lacks its final newline. */
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wnewline-eof"
#endif
#include "../../app_program/subsystem4/subsystem4.c"
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#undef imu_sensor_init
#undef imu_sensor_read
#undef terrain_monitor_process
ER tk_swri_dev(ID id,int start,const void *buffer,SZ length,SZ *actual)
{ (void)id;(void)start;(void)buffer;*actual=length;return E_OK; }
ID tk_opn_dev(UB *name,unsigned mode) { assert(!strcmp((char *)name,"iicb"));(void)mode;return 3; }
ER tk_dly_tsk(unsigned ms)
{ assert(!locked);clock_us+=ms*1000;if(ms==20 && --owner_cycles_left==0)longjmp(finish,1);return E_OK; }
static void owner_cycles(unsigned count)
{ assert(count>0);owner_cycles_left=count;if(!setjmp(finish))entry(0,NULL); }
static void owner_cycle(void) { owner_cycles(1); }
int main(void)
{
    terrain_result_t r={.timestamp_us=clock_us,.flags=TERRAIN_CALIBRATED|TERRAIN_PEAK_VALID,
        .peak_m=0.05f,.state=TERRAIN_MOVING},out;
    uint16_t token,humps,requested; bool lost;
    assert(!subsystem4_read(&out,&token,&humps));
    assert(subsystem4_init()==E_OK && subsystem4_start()==E_OK);
    assert(!subsystem4_read(&out,&token,&humps));
    publish(&r); assert(subsystem4_read(&out,&token,&humps) && token==0 && humps==0);
    r.hump_completed=true;
    for(unsigned i=0;i<10;i++) { publish(&r); }
    assert(subsystem4_read(&out,&token,&humps) && humps==10);
    assert(subsystem4_event(&out,&lost) && lost);
    assert(subsystem4_begin_run(&requested) && requested==1);
    assert(!subsystem4_begin_run(&token));
    assert(!subsystem4_read(&out,&token,&humps) && !subsystem4_event(&out,&lost));
    publish(&r); assert(!subsystem4_read(&out,&token,&humps)); /* old sample suppressed */
    owner_cycle();
    assert(observed_geometry==S4_HUMP_GEOMETRY && observed_track_m==S4_WHEEL_TRACK_M);
    assert(subsystem4_read(&out,&token,&humps) && token==requested && humps==0);
    assert(!(out.flags&TERRAIN_PEAK_VALID));
    while(subsystem4_event(&out,&lost)) {}
    r.timestamp_us=clock_us; r.hump_completed=false; r.impact=false; publish(&r);
    while(subsystem4_event(&out,&lost)) {}
    r.impact=true; publish(&r);
    assert(subsystem4_event(&out,&lost) && out.impact);
    g_humps=UINT16_MAX; r.hump_completed=true; publish(&r);
    assert(subsystem4_read(&out,&token,&humps) && humps==UINT16_MAX);
    clock_us+=160000; assert(subsystem4_read(&out,&token,&humps) && (out.flags&TERRAIN_FAULT));
    assert((out.flags&TERRAIN_PEAK_VALID) && !out.impact && !out.hump_completed);
    fail_lock=true; assert(!subsystem4_read(&out,&token,&humps)); assert(!subsystem4_begin_run(&token)); fail_lock=false;
    g_run=UINT16_MAX; assert(subsystem4_begin_run(&requested) && requested==0);
    owner_cycle(); assert(subsystem4_read(&out,&token,&humps) && token==0 && humps==0);
    /* Observe the second loop so sensor startup delay is not the source of
     * the race. Fresh feedback may be published after S4's loop-start clock. */
    owner_cycles(2); assert(observed_odometry.valid && observed_odometry.stationary);
    feedback_read_delay_us=1000;
    owner_cycles(2);
    assert(observed_odometry.valid && "Fresh feedback after a mutex wait must remain valid");
    feedback_age_us=50000;
    owner_cycles(2); assert(observed_odometry.valid);
    feedback_age_us=50001;
    owner_cycles(2); assert(!observed_odometry.valid && !observed_odometry.stationary);
    feedback_age_us=0; feedback_future_us=1;
    owner_cycles(2); assert(!observed_odometry.valid);
    feedback_future_us=0;
    /* The first loop consumes 160 ms startup, 1 ms feedback and 20 ms sleep;
     * the second feedback read crosses the uint32_t hardware-clock wrap. */
    clock_us=UINT32_MAX-181500U;
    owner_cycles(2); assert(observed_odometry.valid);
    puts("PASS: real S4 transport/task reset acknowledgement, cumulative count/saturation, event overflow, impact delivery, stale validity, token wrap and feedback freshness across mutex waits/clock wrap");
    return 0;
}
