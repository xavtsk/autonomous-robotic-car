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
static bool locked[3],fail_lock,fail_debug_lock;
static ID next_mutex;
static jmp_buf finish;
static void (*entry)(INT,void *);
ER tk_dly_tsk(unsigned ms);
void tk_ext_tsk(void) { assert(!"Unexpected task exit"); }
ID tk_cre_tsk(const T_CTSK *c)
{ assert(c->itskpri==18 && c->stksz==4096);entry=(void (*)(INT,void *))c->task;return 2; }
ER tk_sta_tsk(ID id,INT value) { (void)id;(void)value;return E_OK; }
ER tk_del_tsk(ID id) { (void)id;return E_OK; }
ID tk_cre_mtx(const T_CMTX *m) { assert(m->mtxatr==TA_INHERIT && next_mutex<2);return ++next_mutex; }
ER tk_loc_mtx(ID id,TMO timeout)
{
    assert(id>0 && id<=2 && !locked[id]);
    if (id==2) { assert(timeout==TMO_POL); }
    if ((id==1 && fail_lock) || (id==2 && fail_debug_lock)) { return E_TMOUT; }
    locked[id]=true;return E_OK;
}
ER tk_unl_mtx(ID id) { assert(id>0 && id<=2 && locked[id]);locked[id]=false;return E_OK; }
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
{
    assert(!locked[1]&&!locked[2]);
    clock_us+=ms*1000;
    if(ms==S4_IMU_POLL_MS && --owner_cycles_left==0) { longjmp(finish,1); }
    return E_OK;
}
static void owner_cycles(unsigned count)
{ assert(count>0);owner_cycles_left=count;if(!setjmp(finish))entry(0,NULL); }
static void owner_cycle(void) { owner_cycles(1); }
#if APP_IMU_DEBUG
static uint16_t wire16(const uint8_t *bytes)
{
    return (uint16_t)(bytes[0] | (uint16_t)bytes[1] << 8);
}
static uint32_t wire32(const uint8_t *bytes)
{
    return (uint32_t)wire16(bytes) | (uint32_t)wire16(bytes + 2) << 16;
}
static void test_imu_debug(void)
{
    terrain_monitor_t monitor;
    terrain_monitor_init(&monitor);
    monitor.calibration.ready=true;
    monitor.calibration.bias_g[0]=0.01f;
    monitor.calibration.bias_g[2]=0.02f;
    imu_sample_t sample={.valid=true,.overrun=true,.accel_g={-0.02f,0.03f,1.19f}};
    const terrain_odometry_t odometry={0};
    uint8_t packet[MQTT_UTK_IMU_DEBUG_MAX];
    uint16_t next_run;
    g_imu_debug_head=0;
    g_imu_debug_count=0;
    const uint32_t first_sequence=g_imu_debug_sequence;
    const uint16_t first_run=g_run;
    const uint32_t first_timestamp=clock_us=2000000U;
    for (unsigned i=0; i<5; ++i) {
        sample.timestamp_us=clock_us;
        process_sample(&monitor,&sample,&odometry);
        clock_us+=20000U;
        if (i<4) { assert(mqtt_utk_imu_debug_take(packet,sizeof packet)==0); }
    }
    assert(mqtt_utk_imu_debug_take(packet,151)==0);
    assert(mqtt_utk_imu_debug_take(packet,sizeof packet)==152);
    assert(packet[0]==2 && packet[1]==5);
    assert(wire16(packet + 2)==first_run && wire32(packet + 4)==first_sequence);
    const uint32_t boot_tag=wire32(packet + 8);
    assert(boot_tag!=0);
    const uint8_t *row=packet + 12;
    assert(wire32(row)==first_timestamp);
    assert(wire16(row + 4)==(uint16_t)-20 && wire16(row + 6)==30 && wire16(row + 8)==1190);
    assert(wire16(row + 10)==(uint16_t)-30 && wire16(row + 12)==30 && wire16(row + 14)==1170);
    assert(row[16]==(1U|2U|4U|16U|32U) && row[17]==TERRAIN_SAFETY_NONE);
    assert(wire32(row + 18)==(uint32_t)INT32_MIN && wire32(row + 22)==(uint32_t)INT32_MIN);
    assert(row[26]==0 && row[27]==0);
    row+=2 * 28;
    assert(row[16]==(1U|2U|4U|32U) && row[17]==TERRAIN_SAFETY_LIFTED);

    /* A fault row followed by a new run flushes as its own packet. */
    sample=(imu_sample_t){.timestamp_us=clock_us};
    process_sample(&monitor,&sample,&odometry);
    assert(subsystem4_begin_run(&next_run));
    process_sample(&monitor,&sample,&odometry); /* blocked by pending reset */
    owner_cycle();
    assert(mqtt_utk_imu_debug_take(packet,sizeof packet)==40);
    assert(packet[1]==1 && wire16(packet + 2)==first_run && wire32(packet + 4)==first_sequence+5U);
    assert(packet[28]==0 && packet[29]==TERRAIN_SAFETY_LIFTED);
    clock_us+=250000U;
    assert(mqtt_utk_imu_debug_take(packet,sizeof packet)==40);
    assert(wire16(packet + 2)==next_run && wire32(packet + 4)==first_sequence+6U);
    assert(wire32(packet + 8)==boot_tag && packet[28]==1U);

    /* Lock contention is visible as a gap, even within the same run. */
    owner_cycle();
    const uint32_t before_gap=g_imu_debug_sequence-1U;
    fail_debug_lock=true;
    owner_cycle();
    terrain_result_t latest;
    uint16_t token,humps;
    assert(subsystem4_read(&latest,&token,&humps) && token==next_run);
    fail_debug_lock=false;
    owner_cycle();
    assert(mqtt_utk_imu_debug_take(packet,sizeof packet)==40);
    assert(wire32(packet + 4)==before_gap);
    clock_us+=250000U;
    assert(mqtt_utk_imu_debug_take(packet,sizeof packet)==40);
    assert(wire32(packet + 4)==before_gap+2U);

    const uint32_t before_overflow=g_imu_debug_sequence;
    owner_cycles(IMU_DEBUG_QUEUE_ROWS+2U);
    assert(mqtt_utk_imu_debug_take(packet,sizeof packet)==152);
    assert(wire32(packet + 4)==before_overflow+2U);
}

static void test_hump_imu_debug(void)
{
    terrain_monitor_t monitor;
    terrain_monitor_init(&monitor);
    monitor.hump_geometry=TERRAIN_HUMP_RIGHT_WHEEL;
    monitor.hump.debug=(hump_debug_t){.wheel_delta_m=-0.000275f,.travel_m=0.011325f,
        .wheel_delta_valid=true,.flags=HUMP_DEBUG_DISCARDED,.reason=HUMP_REASON_REVERSE_TRAVEL};
    imu_sample_t sample={.timestamp_us=clock_us,.valid=true,.accel_g={0,0,1}};
    uint8_t packet[MQTT_UTK_IMU_DEBUG_MAX];
    g_imu_debug_head=0;
    g_imu_debug_count=0;
    imu_debug_record(&sample,&monitor,g_run);

    monitor.hump.debug=(hump_debug_t){.wheel_delta_valid=false,.reason=HUMP_REASON_ODOMETRY_UNAVAILABLE};
    sample.timestamp_us=clock_us+=40000U;
    imu_debug_record(&sample,&monitor,g_run);
    clock_us+=250000U;
    assert(mqtt_utk_imu_debug_take(packet,sizeof packet)==68);
    assert(packet[0]==2 && packet[1]==2);
    const uint8_t *row=packet+12;
    assert(wire32(row+18)==(uint32_t)-275 && wire32(row+22)==11325U);
    assert(row[26]==4 && row[27]==10);
    row+=28;
    assert(wire32(row+18)==(uint32_t)INT32_MIN && wire32(row+22)==0);
    assert(row[26]==0 && row[27]==4);
}
#endif
int main(void)
{
    terrain_result_t r={.timestamp_us=clock_us,.flags=TERRAIN_CALIBRATED|TERRAIN_PEAK_VALID,
        .peak_m=0.05f,.state=TERRAIN_MOVING},out;
    uint16_t token,humps,requested; bool lost;
    assert(!subsystem4_read(&out,&token,&humps));
    assert(subsystem4_init()==E_OK && subsystem4_start()==E_OK);
    assert(!subsystem4_read(&out,&token,&humps));
    publish(&r,NULL); assert(subsystem4_read(&out,&token,&humps) && token==0 && humps==0);
    r.hump_completed=true;
    for(unsigned i=0;i<10;i++) { publish(&r,NULL); }
    assert(subsystem4_read(&out,&token,&humps) && humps==10);
    assert(subsystem4_event(&out,&lost) && lost);
    assert(subsystem4_begin_run(&requested) && requested==1);
    assert(!subsystem4_begin_run(&token));
    assert(!subsystem4_read(&out,&token,&humps) && !subsystem4_event(&out,&lost));
    publish(&r,NULL); assert(!subsystem4_read(&out,&token,&humps)); /* old sample suppressed */
    owner_cycle();
    assert(observed_geometry==S4_HUMP_GEOMETRY && observed_track_m==S4_WHEEL_TRACK_M);
    assert(subsystem4_read(&out,&token,&humps) && token==requested && humps==0);
    assert(!(out.flags&TERRAIN_PEAK_VALID));
    while(subsystem4_event(&out,&lost)) {}
    r.timestamp_us=clock_us; r.hump_completed=false; r.impact=false; publish(&r,NULL);
    while(subsystem4_event(&out,&lost)) {}
    r.safety=TERRAIN_SAFETY_LIFTED;
    for(unsigned i=0;i<12;i++) { publish(&r,NULL); }
    assert(subsystem4_read(&out,&token,&humps) && out.safety==TERRAIN_SAFETY_LIFTED);
    assert(subsystem4_event(&out,&lost) && !lost && out.safety==TERRAIN_SAFETY_LIFTED);
    assert(!subsystem4_event(&out,&lost));
    r.safety=TERRAIN_SAFETY_OVERTURNED; publish(&r,NULL);
    r.safety=TERRAIN_SAFETY_NONE; publish(&r,NULL);
    assert(subsystem4_read(&out,&token,&humps) && out.safety==TERRAIN_SAFETY_NONE);
    assert(subsystem4_event(&out,&lost) && !lost && out.safety==TERRAIN_SAFETY_OVERTURNED);
    while(subsystem4_event(&out,&lost)) {}
    r.impact=true; publish(&r,NULL);
    assert(subsystem4_event(&out,&lost) && out.impact);
    g_humps=UINT16_MAX; r.hump_completed=true; publish(&r,NULL);
    assert(subsystem4_read(&out,&token,&humps) && humps==UINT16_MAX);
    clock_us+=160000; assert(subsystem4_read(&out,&token,&humps) && (out.flags&TERRAIN_FAULT));
    assert((out.flags&TERRAIN_PEAK_VALID) && !out.impact && !out.hump_completed);
    fail_lock=true; assert(!subsystem4_read(&out,&token,&humps)); assert(!subsystem4_begin_run(&token)); fail_lock=false;
    g_run=UINT16_MAX; assert(subsystem4_begin_run(&requested) && requested==0);
    owner_cycle(); assert(subsystem4_read(&out,&token,&humps) && token==0 && humps==0);
    /* Observe the second loop so sensor startup delay is not the source of
     * the race. Fresh feedback may be published after S4's loop-start clock. */
    const uint32_t owner_start_us=clock_us;
    owner_cycles(2); assert(observed_odometry.valid && observed_odometry.stationary);
    assert(clock_us-owner_start_us==180000U); /* 160 ms startup and two 10 ms polls. */
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
    /* Leave 500 us after the first loop so the second feedback read crosses
     * the uint32_t hardware-clock wrap during its 1 ms mutex wait. */
    clock_us=UINT32_MAX-(161500U + S4_IMU_POLL_MS * 1000U);
    owner_cycles(2); assert(observed_odometry.valid);
#if APP_IMU_DEBUG
    test_imu_debug();
    test_hump_imu_debug();
#endif
    puts("PASS: real S4 transport/task reset acknowledgement, count/saturation, event overflow, safety/impact delivery, stale validity, token wrap and feedback freshness");
    return 0;
}
