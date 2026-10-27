#include "subsystem4.h"
#ifndef APP_S4
#define APP_S4 0
#endif
#ifndef APP_S4_BENCH
#define APP_S4_BENCH 0
#endif
#if APP_S4
#include <sys/sysdef.h>
#include <tk/device.h>
#include "../../device/include/dev_i2c.h"
#include "../subsystem2/motion.h"
#define S4_QUEUE_LEN 8U
static ID g_mutex, g_task, g_device;
static uint8_t g_snapshot[TERRAIN_PACKET_SIZE], g_events[S4_QUEUE_LEN][TERRAIN_PACKET_SIZE];
static unsigned g_head,g_count;
static bool g_lost,g_reset,g_recalibrate,g_mag_pending;
static float g_offset[3],g_scale[3];
static uint32_t now_us(void) { return in_w(TIMER_TIMERAWL); }
static bool bus_read(void *ctx,uint8_t addr,uint8_t reg,uint8_t *out,unsigned n)
{
    SZ actual=0; ER er; T_I2C_EXEC request={0}; (void)ctx;
    request.sadr=addr; request.snd_size=1; request.snd_data=&reg;
    request.rcv_size=(SZ)n; request.rcv_data=out;
    er=tk_swri_dev(g_device,TDN_I2C_EXEC,&request,sizeof(request),&actual);
    return er==E_OK && actual==(SZ)sizeof(request);
}
static bool bus_write(void *ctx,uint8_t addr,uint8_t reg,uint8_t value)
{
    uint8_t data[2]={reg,value}; SZ actual=0; ER er; (void)ctx;
    er=tk_swri_dev(g_device,addr,data,2,&actual);
    return er==E_OK && actual==2;
}
static void publish(const terrain_result_t *r)
{
    if(tk_loc_mtx(g_mutex,TMO_FEVR)!=E_OK) { return; }
    terrain_result_t previous;
    if(g_reset) { tk_unl_mtx(g_mutex); return; }
    terrain_unpack(g_snapshot,&previous);
    if(r->state!=previous.state || r->flags!=previous.flags || r->hump_completed) {
        if(g_count==S4_QUEUE_LEN) { g_head=(g_head+1)%S4_QUEUE_LEN; --g_count; g_lost=true; }
        terrain_pack(r,g_events[(g_head+g_count)%S4_QUEUE_LEN]); ++g_count;
    }
    terrain_pack(r,g_snapshot);
    tk_unl_mtx(g_mutex);
}
static void task(INT stacd,void *exinf)
{
    terrain_monitor_t monitor; imu_bus_t bus={NULL,bus_read,bus_write};
    uint32_t last_good=0,report_at=0,retry_at=0; bool connected=false;
    (void)stacd; (void)exinf;
    terrain_monitor_init(&monitor);
    g_device=tk_opn_dev((UB *)"iicb",TD_UPDATE);
    if(g_device<0) { tm_printf((UB *)"[S4] iicb open failed: %d\n",g_device); tk_ext_tsk(); }
    for(;;) {
        uint32_t now=now_us(); imu_sample_t sample={0}; terrain_odometry_t odo={0};
        if(tk_loc_mtx(g_mutex,TMO_FEVR)==E_OK) {
            if(g_reset) {
                terrain_monitor_reset_run(&monitor);
                if(g_recalibrate) { imu_calibration_reset(&monitor.calibration); }
                g_reset=false; g_recalibrate=false; g_count=0; g_head=0; g_lost=false;
            }
            if(g_mag_pending) { terrain_monitor_set_magnetic_calibration(&monitor,g_offset,g_scale); g_mag_pending=false; }
            tk_unl_mtx(g_mutex);
        }
        if(!connected && (retry_at==0 || (uint32_t)(now-retry_at)>=1000000U)) {
            retry_at=now; connected=imu_sensor_init(&bus);
            if(connected) { tk_dly_tsk(160); last_good=now_us(); }
            else { sample.timestamp_us=now; terrain_monitor_process(&monitor,&sample,&odo); publish(&monitor.result); }
        }
        if(connected) {
#if APP_S4_BENCH
            odo.stationary=true; /* Bench: user keeps car level/still for startup. */
#else
            motion_feedback_t feedback={0};
            if(motion_get_feedback(&feedback) && feedback.valid && motion_is_calibrated() &&
                feedback.fault==MOTION_FAULT_NONE && (uint32_t)(now-feedback.timestamp_us)<=50000U) {
                odo.valid=true; odo.stationary=feedback.braked;
                odo.distance_m=feedback.centre_m; odo.epoch=feedback.epoch;
            }
#endif
            if(imu_sensor_read(&bus,now_us(),&sample)) {
                last_good=sample.timestamp_us;
                terrain_monitor_process(&monitor,&sample,&odo); publish(&monitor.result);
            } else if((uint32_t)(now_us()-last_good)>100000U) {
                sample.valid=false; sample.timestamp_us=now_us();
                terrain_monitor_process(&monitor,&sample,&odo); publish(&monitor.result); connected=false;
            }
        }
#if APP_S4_BENCH
        if((uint32_t)(now_us()-report_at)>=1000000U) {
            const terrain_result_t *r=&monitor.result; report_at=now_us();
            tm_printf((UB *)"[S4] connected=%d cal=%d flags=%u pitch_x10=%d roll_x10=%d state=%u peak_mm=%d\n",
                connected,monitor.calibration.ready,r->flags,(int)(r->pitch_deg*10),(int)(r->roll_deg*10),
                (unsigned)r->state,(int)(r->peak_m*1000));
        }
#else
        (void)report_at;
#endif
        tk_dly_tsk(20);
    }
}
ER subsystem4_init(void)
{
    T_CMTX m={0}; if(g_mutex>0) { return E_OBJ; }
    m.mtxatr=TA_INHERIT; g_mutex=tk_cre_mtx(&m);
    return g_mutex<0?(ER)g_mutex:E_OK;
}
ER subsystem4_start(void)
{
    T_CTSK c={0}; ER er;
    if(g_mutex<=0 || g_task>0) { return E_OBJ; }
    c.task=(FP)task; c.itskpri=S4_PRI_MAIN; c.stksz=4096; c.tskatr=TA_HLNG|TA_RNG3;
#if TK_SUPPORT_SMP
    c.tskatr|=TA_ASSPRC; c.assprc=TP_PRC1;
#endif
    g_task=tk_cre_tsk(&c); if(g_task<0) { return (ER)g_task; }
    er=tk_sta_tsk(g_task,0);
    if(er<0) { tk_del_tsk(g_task); g_task=0; }
    return er;
}
bool subsystem4_snapshot(terrain_result_t *out)
{
    if(!out || g_mutex<=0 || tk_loc_mtx(g_mutex,TMO_POL)!=E_OK) { return false; }
    terrain_unpack(g_snapshot,out); tk_unl_mtx(g_mutex);
    if((uint32_t)(now_us()-out->timestamp_us)>150000U) {
        out->flags=(out->flags & (TERRAIN_PEAK_VALID|TERRAIN_CALIBRATED))|TERRAIN_FAULT;
        out->state=TERRAIN_UNKNOWN; out->impact=false;
    }
    return true;
}
bool subsystem4_event(terrain_result_t *out,bool *lost)
{
    bool ok=false;
    if(!out || !lost || g_mutex<=0 || tk_loc_mtx(g_mutex,TMO_POL)!=E_OK) { return false; }
    if(g_count) { terrain_unpack(g_events[g_head],out); g_head=(g_head+1)%S4_QUEUE_LEN; --g_count; *lost=g_lost; g_lost=false; ok=true; }
    tk_unl_mtx(g_mutex); return ok;
}
bool subsystem4_reset_run(bool recalibrate)
{
    if(g_mutex<=0 || tk_loc_mtx(g_mutex,TMO_POL)!=E_OK) { return false; }
    g_reset=true; g_recalibrate|=recalibrate;
    { unsigned i; for(i=0;i<TERRAIN_PACKET_SIZE;++i) { g_snapshot[i]=0; } } g_head=0; g_count=0;
    tk_unl_mtx(g_mutex); return true;
}
bool subsystem4_set_magnetic_calibration(const float offset[3],const float scale[3])
{
    terrain_monitor_t check;
    terrain_monitor_init(&check);
    if(!terrain_monitor_set_magnetic_calibration(&check,offset,scale) || g_mutex<=0 || tk_loc_mtx(g_mutex,TMO_POL)!=E_OK) { return false; }
    { unsigned i; for(i=0;i<3;++i) { g_offset[i]=offset[i]; g_scale[i]=scale[i]; } } g_mag_pending=true;
    tk_unl_mtx(g_mutex); return true;
}
#else
ER subsystem4_init(void) { return E_OK; }
ER subsystem4_start(void) { return E_OK; }
bool subsystem4_snapshot(terrain_result_t *out) { (void)out; return false; }
bool subsystem4_event(terrain_result_t *out,bool *lost) { (void)out; (void)lost; return false; }
bool subsystem4_reset_run(bool recalibrate) { (void)recalibrate; return false; }
bool subsystem4_set_magnetic_calibration(const float o[3],const float s[3]) { (void)o; (void)s; return false; }
#endif
