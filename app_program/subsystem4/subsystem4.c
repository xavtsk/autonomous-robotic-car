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
#if APP_IMU_DEBUG
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"
#endif
/* This car's test obstacle raises the right wheel only. Keep the portable
 * monitor's longitudinal default available through an explicit build choice. */
#ifndef S4_HUMP_GEOMETRY
#define S4_HUMP_GEOMETRY TERRAIN_HUMP_RIGHT_WHEEL
#endif
#ifndef S4_WHEEL_TRACK_M
/* Wheel-centre spacing; matches Buddy 2 TRACK_WIDTH_M (116 mm). */
#define S4_WHEEL_TRACK_M (0.116f)
#endif
#define S4_QUEUE_LEN 8U
/* Wake every kernel tick to check for the accelerometer's 50 Hz output. */
#define S4_IMU_POLL_MS 10U
static ID g_mutex, g_task, g_cycle, g_device;
static uint8_t g_snapshot[TERRAIN_PACKET_SIZE], g_events[S4_QUEUE_LEN][TERRAIN_PACKET_SIZE];
static unsigned g_head,g_count;
static uint16_t g_run,g_published_run,g_humps;
static bool g_published;
static bool g_lost,g_reset,g_recalibrate,g_mag_pending;
static float g_offset[3],g_scale[3];
static uint32_t now_us(void) { return in_w(TIMER_TIMERAWL); }

#if APP_IMU_DEBUG
#define IMU_DEBUG_VERSION 2U
#define IMU_DEBUG_HEADER_BYTES 12U
#define IMU_DEBUG_ROW_BYTES 28U
#define IMU_DEBUG_BATCH_ROWS 5U
#define IMU_DEBUG_PACKET_BYTES (IMU_DEBUG_HEADER_BYTES + IMU_DEBUG_BATCH_ROWS * IMU_DEBUG_ROW_BYTES)
#define IMU_DEBUG_QUEUE_ROWS 128U
#define IMU_DEBUG_FLUSH_US 250000U

enum {
    IMU_DEBUG_VALID = 1U << 0,
    IMU_DEBUG_EVALUATED = 1U << 1,
    IMU_DEBUG_OVERRUN = 1U << 2,
    IMU_DEBUG_IMPACT = 1U << 3,
    IMU_DEBUG_PENDING = 1U << 4,
    IMU_DEBUG_UPWARD = 1U << 5,
    IMU_DEBUG_TILTED = 1U << 6
};

typedef struct {
    uint32_t timestamp_us, sequence;
    int32_t wheel_delta_um, hump_travel_um;
    int16_t raw_mg[3], calibrated_mg[3];
    uint16_t run;
    uint8_t flags, safety;
    uint8_t hump_flags, hump_reason;
} imu_debug_row_t;

_Static_assert(IMU_DEBUG_PACKET_BYTES <= MQTT_UTK_IMU_DEBUG_MAX,
               "IMU debug packet exceeds MQTT staging capacity");
static ID g_imu_debug_mutex;
static imu_debug_row_t g_imu_debug_rows[IMU_DEBUG_QUEUE_ROWS];
static unsigned g_imu_debug_head, g_imu_debug_count;
static uint32_t g_imu_debug_sequence, g_imu_debug_boot_tag;

static int16_t imu_debug_mg(float g)
{
    /* The sensor range is ±4 g, and level calibration adds at most 0.15 g. */
    float mg = g * 1000.0f;
    return (int16_t)(mg + (mg < 0.0f ? -0.5f : 0.5f));
}

static int32_t imu_debug_um(float metres)
{
    float um = metres * 1000000.0f;
    /* Leave INT32_MIN available as a sentinel for an unavailable distance. */
    if (!(um > -2147483648.0f && um < 2147483648.0f)) { return INT32_MIN; }
    return (int32_t)(um + (um < 0.0f ? -0.5f : 0.5f));
}

static void imu_debug_record(const imu_sample_t *sample, const terrain_monitor_t *monitor,
                             uint16_t run)
{
    if (g_imu_debug_mutex <= 0) { return; }

    /* The calibrated flag is set only when this sample reached the detector. */
    const bool evaluated = (monitor->result.flags & TERRAIN_CALIBRATED) != 0;
    const bool valid = sample->valid;
    imu_debug_row_t row = {
        .timestamp_us = sample->timestamp_us,
        .sequence = g_imu_debug_sequence++,
        .wheel_delta_um = INT32_MIN,
        .hump_travel_um = INT32_MIN,
        .run = run,
        .flags = (uint8_t)((valid ? IMU_DEBUG_VALID : 0) |
                           (evaluated ? IMU_DEBUG_EVALUATED : 0) |
                           (sample->overrun ? IMU_DEBUG_OVERRUN : 0) |
                           (monitor->result.impact ? IMU_DEBUG_IMPACT : 0)),
        .safety = (uint8_t)monitor->result.safety
    };
    if (monitor->hump_geometry == TERRAIN_HUMP_RIGHT_WHEEL) {
        if (monitor->hump.debug.wheel_delta_valid) {
            row.wheel_delta_um = imu_debug_um(monitor->hump.debug.wheel_delta_m);
        }
        row.hump_travel_um = imu_debug_um(monitor->hump.debug.travel_m);
        row.hump_flags = monitor->hump.debug.flags;
        row.hump_reason = monitor->hump.debug.reason;
    }
    if (evaluated) {
        if (monitor->safety_tracking && monitor->result.safety == TERRAIN_SAFETY_NONE) {
            row.flags |= IMU_DEBUG_PENDING;
        }
        if (monitor->safety_upward) { row.flags |= IMU_DEBUG_UPWARD; }
        if (monitor->safety_lift_tilt) { row.flags |= IMU_DEBUG_TILTED; }
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (valid) { row.raw_mg[axis] = imu_debug_mg(sample->accel_g[axis]); }
        if (evaluated) {
            /* Match the unfiltered vector used in safety_update(). */
            row.calibrated_mg[axis] = imu_debug_mg(sample->accel_g[axis] -
                                                  monitor->calibration.bias_g[axis]);
        }
    }

    /* Debug contention may cost a row, detectable by its sequence gap. */
    if (tk_loc_mtx(g_imu_debug_mutex, TMO_POL) != E_OK) { return; }
    if (g_imu_debug_boot_tag == 0U) {
        /* A best-effort session tag from startup timing and the first reading. */
        uint32_t tag = row.timestamp_us;
        for (unsigned axis = 0; axis < 3; ++axis) {
            tag = (tag ^ (uint16_t)row.raw_mg[axis]) * 16777619U;
        }
        g_imu_debug_boot_tag = tag | 1U;
    }
    if (g_imu_debug_count == IMU_DEBUG_QUEUE_ROWS) {
        g_imu_debug_head = (g_imu_debug_head + 1U) % IMU_DEBUG_QUEUE_ROWS;
        --g_imu_debug_count;
    }
    unsigned tail = (g_imu_debug_head + g_imu_debug_count) % IMU_DEBUG_QUEUE_ROWS;
    g_imu_debug_rows[tail] = row;
    ++g_imu_debug_count;
    tk_unl_mtx(g_imu_debug_mutex);
}

static void imu_debug_put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static void imu_debug_put32(uint8_t *out, uint32_t value)
{
    imu_debug_put16(out, (uint16_t)value);
    imu_debug_put16(out + 2, (uint16_t)(value >> 16));
}

unsigned int mqtt_utk_imu_debug_take(unsigned char *bytes, unsigned int size)
{
    if (!bytes || size < IMU_DEBUG_PACKET_BYTES || g_imu_debug_mutex <= 0 ||
        tk_loc_mtx(g_imu_debug_mutex, TMO_POL) != E_OK) { return 0; }
    if (g_imu_debug_count == 0U) {
        tk_unl_mtx(g_imu_debug_mutex);
        return 0;
    }

    const imu_debug_row_t *first = &g_imu_debug_rows[g_imu_debug_head];
    unsigned count = 1;
    while (count < IMU_DEBUG_BATCH_ROWS && count < g_imu_debug_count) {
        const imu_debug_row_t *next = &g_imu_debug_rows[(g_imu_debug_head + count) % IMU_DEBUG_QUEUE_ROWS];
        if (next->run != first->run || next->sequence != first->sequence + count) { break; }
        ++count;
    }
    if (count < IMU_DEBUG_BATCH_ROWS && count == g_imu_debug_count &&
        now_us() - first->timestamp_us < IMU_DEBUG_FLUSH_US) {
        tk_unl_mtx(g_imu_debug_mutex);
        return 0;
    }

    bytes[0] = IMU_DEBUG_VERSION;
    bytes[1] = (uint8_t)count;
    imu_debug_put16(bytes + 2, first->run);
    imu_debug_put32(bytes + 4, first->sequence);
    imu_debug_put32(bytes + 8, g_imu_debug_boot_tag);
    uint8_t *out = bytes + IMU_DEBUG_HEADER_BYTES;
    for (unsigned i = 0; i < count; ++i, out += IMU_DEBUG_ROW_BYTES) {
        const imu_debug_row_t *row = &g_imu_debug_rows[(g_imu_debug_head + i) % IMU_DEBUG_QUEUE_ROWS];
        imu_debug_put32(out, row->timestamp_us);
        for (unsigned axis = 0; axis < 3; ++axis) {
            imu_debug_put16(out + 4 + 2 * axis, (uint16_t)row->raw_mg[axis]);
            imu_debug_put16(out + 10 + 2 * axis, (uint16_t)row->calibrated_mg[axis]);
        }
        out[16] = row->flags;
        out[17] = row->safety;
        imu_debug_put32(out + 18, (uint32_t)row->wheel_delta_um);
        imu_debug_put32(out + 22, (uint32_t)row->hump_travel_um);
        out[26] = row->hump_flags;
        out[27] = row->hump_reason;
    }
    g_imu_debug_head = (g_imu_debug_head + count) % IMU_DEBUG_QUEUE_ROWS;
    g_imu_debug_count -= count;
    tk_unl_mtx(g_imu_debug_mutex);
    return IMU_DEBUG_HEADER_BYTES + count * IMU_DEBUG_ROW_BYTES;
}
#endif

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
static bool publish(const terrain_result_t *r, uint16_t *run)
{
    if(tk_loc_mtx(g_mutex,TMO_FEVR)!=E_OK) { return false; }
    terrain_result_t previous;
    if(g_reset) { tk_unl_mtx(g_mutex); return false; }
    terrain_unpack(g_snapshot,&previous);
    if(r->state!=previous.state || r->flags!=previous.flags || r->safety!=previous.safety ||
       r->hump_completed || r->impact) {
        if(g_count==S4_QUEUE_LEN) { g_head=(g_head+1)%S4_QUEUE_LEN; --g_count; g_lost=true; }
        terrain_pack(r,g_events[(g_head+g_count)%S4_QUEUE_LEN]); ++g_count;
    }
    if(r->hump_completed && (r->flags&TERRAIN_PEAK_VALID) && g_humps<UINT16_MAX) { ++g_humps; }
    terrain_pack(r,g_snapshot); g_published_run=g_run; g_published=true;
    if (run) { *run = g_published_run; }
    tk_unl_mtx(g_mutex);
    return true;
}

static void process_sample(terrain_monitor_t *monitor, const imu_sample_t *sample,
                           const terrain_odometry_t *odometry)
{
    terrain_monitor_process(monitor, sample, odometry);
#if APP_IMU_DEBUG
    uint16_t run;
    if (publish(&monitor->result, &run)) { imu_debug_record(sample, monitor, run); }
#else
    (void)publish(&monitor->result, NULL);
#endif
}
static void imu_poll_tick(void *unused)
{
    (void)unused;
    (void)tk_wup_tsk(g_task);
}

static void task(INT stacd,void *exinf)
{
    terrain_monitor_t monitor; imu_bus_t bus={NULL,bus_read,bus_write};
    uint32_t last_good=0,report_at=0,retry_at=0; bool connected=false;
    (void)stacd; (void)exinf;
    terrain_monitor_init(&monitor);
    if(!terrain_monitor_set_hump_geometry(&monitor,S4_HUMP_GEOMETRY,S4_WHEEL_TRACK_M)) {
        tm_printf((UB *)"[S4] invalid hump geometry configuration\n");
        (void)tk_stp_cyc(g_cycle);
        tk_ext_tsk();
        return;
    }
    g_device=tk_opn_dev((UB *)"iicb",TD_UPDATE);
    if(g_device<0) {
        tm_printf((UB *)"[S4] iicb open failed: %d\n",g_device);
        (void)tk_stp_cyc(g_cycle);
        tk_ext_tsk();
        return;
    }
    for(;;) {
        uint32_t now=now_us(); imu_sample_t sample={0}; terrain_odometry_t odo={0};
        if(tk_loc_mtx(g_mutex,TMO_FEVR)==E_OK) {
            if(g_reset) {
                terrain_monitor_reset_run(&monitor);
                if(g_recalibrate) { imu_calibration_reset(&monitor.calibration); }
                g_reset=false; g_recalibrate=false; g_count=0; g_head=0; g_lost=false; g_humps=0;
            }
            if(g_mag_pending) { terrain_monitor_set_magnetic_calibration(&monitor,g_offset,g_scale); g_mag_pending=false; }
            tk_unl_mtx(g_mutex);
        }
        if(!connected && (retry_at==0 || (uint32_t)(now-retry_at)>=1000000U)) {
            retry_at=now; connected=imu_sensor_init(&bus);
            if(connected) { tk_dly_tsk(160); last_good=now_us(); }
            else { sample.timestamp_us=now; process_sample(&monitor,&sample,&odo); }
        }
        if(connected) {
#if APP_S4_BENCH
            odo.stationary=true; /* Bench: user keeps car level/still for startup. */
#else
            motion_feedback_t feedback={0};
            /* Waiting for the motion mutex can produce feedback newer than
             * this loop's start. Check its age against a clock read after the
             * copy, so fresh odometry is not rejected as an unsigned wrap. */
            if(motion_get_feedback(&feedback) && feedback.valid && motion_is_calibrated() &&
                feedback.fault==MOTION_FAULT_NONE && (uint32_t)(now_us()-feedback.timestamp_us)<=50000U) {
                odo.valid=true; odo.stationary=feedback.braked;
                odo.distance_m=feedback.centre_m; odo.epoch=feedback.epoch;
            }
#endif
            if(imu_sensor_read(&bus,now_us(),&sample)) {
                last_good=sample.timestamp_us;
                process_sample(&monitor,&sample,&odo);
            } else if((uint32_t)(now_us()-last_good)>100000U) {
                sample.valid=false; sample.timestamp_us=now_us();
                process_sample(&monitor,&sample,&odo); connected=false;
            }
        }
#if APP_S4_BENCH
        if((uint32_t)(now_us()-report_at)>=1000000U) {
            const terrain_result_t *r=&monitor.result; report_at=now_us();
            tm_printf((UB *)"[S4] connected=%d cal=%d flags=%u pitch_x10=%d roll_x10=%d state=%u peak_mm=%d safety=%u\n",
                connected,monitor.calibration.ready,r->flags,(int)(r->pitch_deg*10),(int)(r->roll_deg*10),
                (unsigned)r->state,(int)(r->peak_m*1000),(unsigned)r->safety);
        }
#else
        (void)report_at;
#endif
        /* Skip ticks accumulated while busy, including during sensor startup. */
        (void)tk_can_wup(TSK_SELF);
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
ER subsystem4_init(void)
{
    T_CMTX m={0}; if(g_mutex>0) { return E_OBJ; }
    m.mtxatr=TA_INHERIT; g_mutex=tk_cre_mtx(&m);
#if APP_IMU_DEBUG
    /* Optional logging must never prevent the safety task from starting. */
    if (g_mutex > 0) {
        g_imu_debug_mutex = tk_cre_mtx(&m);
        if (g_imu_debug_mutex < 0) { tm_printf((UB *)"[S4] IMU debug queue unavailable\n"); }
    }
#endif
    return g_mutex<0?(ER)g_mutex:E_OK;
}
ER subsystem4_start(void)
{
    T_CTSK c={0};
    T_CCYC cycle={0};
    ER er;
    if(g_mutex<=0 || g_task>0) { return E_OBJ; }
    c.task=(FP)task; c.itskpri=S4_PRI_MAIN; c.stksz=4096; c.tskatr=TA_HLNG|TA_RNG3;
#if TK_SUPPORT_SMP
    c.tskatr|=TA_ASSPRC; c.assprc=TP_PRC1;
#endif
    g_task=tk_cre_tsk(&c); if(g_task<0) { return (ER)g_task; }
    cycle.cycatr=TA_HLNG;
    cycle.cychdr=(FP)imu_poll_tick;
    cycle.cyctim=S4_IMU_POLL_MS;
    cycle.cycphs=S4_IMU_POLL_MS;
    g_cycle=tk_cre_cyc(&cycle);
    if (g_cycle>0) {
        er=tk_sta_cyc(g_cycle);
        if (er==E_OK) {
            er=tk_sta_tsk(g_task,0);
            if (er==E_OK) { return E_OK; }
            (void)tk_stp_cyc(g_cycle);
        }
        (void)tk_del_cyc(g_cycle);
    } else {
        er=(ER)g_cycle;
    }
    (void)tk_del_tsk(g_task);
    g_task=0;
    g_cycle=0;
    return er;
}
bool subsystem4_read(terrain_result_t *out,uint16_t *token,uint16_t *humps)
{
    if(!out || !token || !humps || g_mutex<=0 || tk_loc_mtx(g_mutex,TMO_POL)!=E_OK) { return false; }
    if(!g_published || g_reset || g_published_run!=g_run) { tk_unl_mtx(g_mutex); return false; }
    terrain_unpack(g_snapshot,out); *token=g_published_run; *humps=g_humps;
    tk_unl_mtx(g_mutex);
    if((uint32_t)(now_us()-out->timestamp_us)>150000U) {
        out->flags=(out->flags & (TERRAIN_PEAK_VALID|TERRAIN_CALIBRATED))|TERRAIN_FAULT;
        out->state=TERRAIN_UNKNOWN; out->impact=false; out->hump_completed=false;
    }
    return true;
}
bool subsystem4_snapshot(terrain_result_t *out)
{
    uint16_t token,humps;
    return subsystem4_read(out,&token,&humps);
}
bool subsystem4_event(terrain_result_t *out,bool *lost)
{
    bool ok=false;
    if(!out || !lost || g_mutex<=0 || tk_loc_mtx(g_mutex,TMO_POL)!=E_OK) { return false; }
    if(g_count) { terrain_unpack(g_events[g_head],out); g_head=(g_head+1)%S4_QUEUE_LEN; --g_count; *lost=g_lost; g_lost=false; ok=true; }
    tk_unl_mtx(g_mutex); return ok;
}
static bool request_reset(bool recalibrate,uint16_t *token)
{
    if(g_mutex<=0 || tk_loc_mtx(g_mutex,TMO_POL)!=E_OK) { return false; }
    if(g_reset) { tk_unl_mtx(g_mutex); return false; }
    ++g_run; if(token) { *token=g_run; }
    g_reset=true; g_recalibrate=recalibrate; g_published=false;
    g_head=0; g_count=0; g_lost=false;
    tk_unl_mtx(g_mutex); return true;
}
bool subsystem4_reset_run(bool recalibrate) { return request_reset(recalibrate,NULL); }
bool subsystem4_begin_run(uint16_t *token) { return token && request_reset(false,token); }
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
bool subsystem4_begin_run(uint16_t *token) { (void)token; return false; }
bool subsystem4_read(terrain_result_t *out,uint16_t *token,uint16_t *humps)
{ (void)out; (void)token; (void)humps; return false; }
bool subsystem4_set_magnetic_calibration(const float o[3],const float s[3]) { (void)o; (void)s; return false; }
#endif
