/* Opt-in runtime. One sampling owner serialises all navigation/control calls.
 */
#include "s3_integration.h"
#include "subsystem3.h"
#include "ir_sensor.h"
#include <sys/sysdef.h>
#if APP_S3_MOTION
#define S3_COLLECTION_US (1000000U)
static ID g_task, g_report, g_cycle, g_mutex;
static s3_drive_t g_drive;
static s3_integration_status_t g_status;
static s3_control_request_t g_command;
static bool g_pending, g_stop;
static s3_calibration_collector_t g_collector[S3_SENSOR_COUNT];
static bool g_collecting;
static s3_sensor_id_t g_sensor;
static ir_surface_t g_label;
static uint32_t g_collect_since;
static uint32_t now_us(void)
{
    return in_w(TIMER_TIMERAWL);
}
bool s3_integration_submit(const s3_control_request_t *request)
{
    bool accepted = false;
    if ((request == 0) || (g_mutex <= 0) ||
        ((unsigned int)request->kind > S3_CONTROL_RESET))
    {
        return false;
    }
    if (tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }
    if (request->kind == S3_CONTROL_STOP)
    {
        g_stop = true;
        g_pending = false;
        accepted = true;
    }
    else if (!g_pending && !g_stop)
    {
        g_command = *request;
        g_pending = true;
        accepted = true;
    }
    (void)tk_unl_mtx(g_mutex);
    return accepted;
}
bool s3_integration_status(s3_integration_status_t *status)
{
    if ((status == 0) || (g_mutex <= 0))
    {
        return false;
    }
    if (tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }
    *status = g_status;
    (void)tk_unl_mtx(g_mutex);
    return true;
}
static bool execute(const s3_control_request_t *c, uint32_t now)
{
    if (c->kind == S3_CONTROL_STOP)
    {
        s3_drive_stop(&g_drive);
        g_collecting = false;
        return true;
    }
    if (c->kind == S3_CONTROL_RESET)
    {
        return s3_drive_reset(&g_drive, now);
    }
    if (c->kind == S3_CONTROL_START)
    {
        return !g_collecting && s3_drive_start(&g_drive, now);
    }
    if ((g_drive.navigation.lifecycle != S3_RUN_INACTIVE) || g_collecting ||
        (g_drive.state == S3_DRIVE_FAULT))
    {
        return false;
    }
    if (c->kind == S3_CONTROL_MAPPING)
    {
        return s3_navigation_set_mapping(&g_drive.navigation, c->map);
    }
    if ((unsigned int)c->sensor >= S3_SENSOR_COUNT)
    {
        return false;
    }
    if (c->kind == S3_CONTROL_APPLY_CALIBRATION)
    {
        return s3_navigation_apply_calibration(&g_drive.navigation, c->sensor,
                                               &g_collector[c->sensor]);
    }
    g_sensor = c->sensor;
    g_collecting = true;
    g_collect_since = now;
    g_label = c->kind == S3_CONTROL_WHITE ? IR_SURFACE_WHITE : IR_SURFACE_BLACK;
    if (g_label == IR_SURFACE_WHITE)
    {
        g_collector[g_sensor].white = (s3_calibration_range_t){0};
    }
    else
    {
        g_collector[g_sensor].black = (s3_calibration_range_t){0};
    }
    return true;
}
static void tick(void *unused)
{
    (void)unused;
    if (g_task > 0)
    {
        (void)tk_wup_tsk(g_task);
    }
}
static void sample(INT code, void *unused)
{
    static s3_sensor_frame_t frame;
    static motion_feedback_t feedback;
    static s3_control_request_t command;
    static s3_integration_status_t working;
    int32_t adc;
    (void)code;
    (void)unused;
    (void)tk_slp_tsk(TMO_FEVR);
    adc = ir_sensor_init(&g_drive.navigation.config.acquisition);
    for (;;)
    {
        bool pending = false;
        /* STOP is handled before sampling, even if an earlier command queued.
         */
        if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
        {
            if (g_stop)
            {
                command = (s3_control_request_t){.kind = S3_CONTROL_STOP};
                pending = true;
            }
            else if (g_pending)
            {
                command = g_command;
                pending = true;
            }
            g_stop = false;
            g_pending = false;
            (void)tk_unl_mtx(g_mutex);
        }
        if (pending && (command.kind == S3_CONTROL_STOP))
        {
            working.command_ok = execute(&command, now_us());
            working.command_sequence++;
            pending = false;
        }
        feedback = (motion_feedback_t){0};
        (void)motion_get_feedback(&feedback);
        working.adc_status = adc;
        if (adc == E_OK)
        {
            working.adc_status = ir_sensor_read_frame(&frame);
        }
        if (working.adc_status != E_OK)
        {
            frame = (s3_sensor_frame_t){0};
            frame.timestamp_us = now_us();
        }
        (void)s3_drive_process(&g_drive, &frame, &feedback);
        if (g_collecting)
        {
            const s3_raw_sample_t *raw = &frame.sensor[g_sensor];
            if (raw->valid && (raw->error == 0))
            {
                (void)s3_calibration_add(&g_collector[g_sensor], g_label,
                                         raw->raw);
            }
            if ((uint32_t)(frame.timestamp_us - g_collect_since) >=
                S3_COLLECTION_US)
            {
                g_collecting = false;
            }
        }
        if (pending)
        {
            working.command_ok = execute(&command, now_us());
            working.command_sequence++;
        }
        working.navigation = g_drive.output;
        /* Controls can change readiness after this cycle's observation. */
        working.navigation.lifecycle = g_drive.navigation.lifecycle;
        working.navigation.accepted_calibration =
            g_drive.navigation.accepted_calibration;
        working.navigation.readiness_blockers =
            s3_navigation_readiness(&g_drive.navigation, now_us());
        working.motion = feedback;
        working.state = g_drive.state;
        working.reason = g_drive.reason;
        working.samples++;
        if (g_drive.output.sample_gap_us > working.max_gap_us)
        {
            working.max_gap_us = g_drive.output.sample_gap_us;
        }
        if (g_drive.output.sample_gap_us > 5000U)
        {
            working.over_5ms++;
        }
        working.collecting = g_collecting;
        working.calibration_sensor = g_sensor;
        working.white = s3_calibration_summary(&g_collector[g_sensor].white);
        working.black = s3_calibration_summary(&g_collector[g_sensor].black);
        if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
        {
            g_status = working;
            (void)tk_unl_mtx(g_mutex);
        }
        (void)tk_can_wup(TSK_SELF);
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
static void report(INT code, void *unused)
{
    static s3_integration_status_t s;
    (void)code;
    (void)unused;
    for (;;)
    {
        (void)tk_dly_tsk(200U);
        if (!s3_integration_status(&s))
        {
            continue;
        }
        tm_printf(
            (UB *)"[S3+S2] state=%u reason=%u ready=0x%x cal=0x%x motion=%s "
                  "odom_valid=%u adc=%d gap=%u max=%u over5ms=%u\n",
            (unsigned)s.state, (unsigned)s.reason,
            (unsigned)s.navigation.readiness_blockers,
            (unsigned)s.navigation.accepted_calibration,
            motion_state_name(s.motion.state), (unsigned)s.motion.valid,
            (int)s.adc_status, (unsigned)s.navigation.sample_gap_us,
            (unsigned)s.max_gap_us, (unsigned)s.over_5ms);
        tm_printf((UB *)"  command=%u ok=%u collecting=%u sensor=%u white n=%u "
                        "[%u,%u] black n=%u [%u,%u]\n",
                  (unsigned)s.command_sequence, (unsigned)s.command_ok,
                  (unsigned)s.collecting, (unsigned)s.calibration_sensor,
                  (unsigned)s.white.count, (unsigned)s.white.minimum,
                  (unsigned)s.white.maximum, (unsigned)s.black.count,
                  (unsigned)s.black.minimum, (unsigned)s.black.maximum);
    }
}
static void cleanup(void)
{
    if (g_cycle > 0)
    {
        (void)tk_stp_cyc(g_cycle);
    }
    if (g_task > 0)
    {
        (void)tk_ter_tsk(g_task);
        (void)tk_del_tsk(g_task);
    }
    if (g_report > 0)
    {
        (void)tk_ter_tsk(g_report);
        (void)tk_del_tsk(g_report);
    }
    if (g_cycle > 0)
    {
        (void)tk_del_cyc(g_cycle);
    }
    if (g_mutex > 0)
    {
        (void)tk_del_mtx(g_mutex);
    }
    g_task = g_report = g_cycle = g_mutex = 0;
}
int32_t s3_integration_init(void)
{
    T_CMTX mutex = {0};
    T_CCYC cycle = {0};
    s3_navigation_config_t cfg = s3_navigation_defaults();
    const s3_drive_port_t port = {motion_request_velocity,
                                  motion_request_manoeuvre,
                                  stop,
                                  motion_poll,
                                  motion_reset,
                                  motion_navigation_heartbeat};
    if (g_mutex > 0)
    {
        return E_OBJ;
    }
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    if (!s3_drive_init(&g_drive, &cfg, &port))
    {
        return E_PAR;
    }
    mutex.mtxatr = TA_INHERIT;
    g_mutex = tk_cre_mtx(&mutex);
    if (g_mutex < E_OK)
    {
        return g_mutex;
    }
    cycle.cycatr = TA_HLNG;
    cycle.cychdr = (FP)tick;
    cycle.cyctim = 10U;
    cycle.cycphs = 10U;
    g_cycle = tk_cre_cyc(&cycle);
    if (g_cycle < E_OK)
    {
        int32_t er = g_cycle;
        cleanup();
        return er;
    }
    return E_OK;
}
int32_t s3_integration_start(void)
{
    ER er;
    if ((g_mutex <= 0) || (g_cycle <= 0) || (g_task > 0))
    {
        return E_OBJ;
    }
    g_task = app_start_task((FP)sample, S3_PRI_MAIN, TP_PRC1, "s3-motion");
    if (g_task < E_OK)
    {
        er = g_task;
        cleanup();
        return er;
    }
    g_report = app_start_task((FP)report, S3_PRI_REPORT, TP_PRC1, "s3-status");
    if (g_report < E_OK)
    {
        er = g_report;
        cleanup();
        return er;
    }
    er = tk_sta_cyc(g_cycle);
    if (er < E_OK)
    {
        cleanup();
    }
    return er;
}
#else
bool s3_integration_submit(const s3_control_request_t *request)
{
    (void)request;
    return false;
}
bool s3_integration_status(s3_integration_status_t *status)
{
    (void)status;
    return false;
}
int32_t s3_integration_init(void)
{
    return E_NOSPT;
}
int32_t s3_integration_start(void)
{
    return E_NOSPT;
}
#endif
