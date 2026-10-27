/* Opt-in runtime. One sampling owner serialises all navigation/control calls.
 */
#include "s3_integration.h"
#include "vehicle_controller.h"
#include "integration_codec.h"
#include "../diagnostics/robot_diagnostics.h"
#include "../subsystem3/ir_sensor.h"
#include "../subsystem3/subsystem3.h"
#include "builtin_calibration.h"
#include "course_profile.h"
#include <sys/sysdef.h>
#if APP_S3_MOTION
#define S3_COLLECTION_US (1000000U)
#define S3_EMPTY_RANGE_MINIMUM (4095U) /* 12-bit ADC maximum */
static ID g_task, g_report, g_cycle, g_mutex;
static s3_drive_t g_drive;
static s3_status_packet_t g_status;
static s3_control_packet_t g_command;
static struct { unsigned pending:1, stop:1, status_ready:1, reset_pending:1; } g_shared;
static bool g_reset_waiting; /* sampling-owner-only completion state */
static s3_calibration_collector_t g_collector[S3_SENSOR_COUNT];
static bool g_collecting;
static s3_sensor_id_t g_sensor;
static ir_surface_t g_label;
static uint32_t g_collect_since;
static uint32_t now_us(void) { return in_w(TIMER_TIMERAWL); }
bool s3_integration_submit(const s3_control_request_t *request)
{
    bool accepted = false;
    s3_control_packet_t packet;
    if ((request == 0) || (g_mutex <= 0) ||
        ((unsigned int)request->kind > S3_CONTROL_RESET) ||
        !s3_control_pack(request, &packet))
    {
        return false;
    }
    if (tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }
    if (request->kind == S3_CONTROL_STOP)
    {
        g_shared.stop = true;
        g_shared.pending = false;
        g_shared.reset_pending = false;
        accepted = true;
    }
    else if (!g_shared.pending && !g_shared.stop && !g_shared.reset_pending)
    {
        g_command = packet;
        g_shared.pending = true;
        g_shared.reset_pending = request->kind == S3_CONTROL_RESET;
        accepted = true;
    }
    (void)tk_unl_mtx(g_mutex);
    return accepted;
}
bool s3_integration_status(s3_integration_status_t *status)
{
    s3_status_packet_t packet;
    bool ready;
    if ((status == 0) || (g_mutex <= 0))
    {
        return false;
    }
    if (tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }
    packet = g_status;
    ready = g_shared.status_ready;
    (void)tk_unl_mtx(g_mutex);
    return ready && s3_status_unpack(&packet, status);
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
        /* Ineligible STARTs (already driving, FAULT, active lifecycle,
         * collecting, not ready) are refused BEFORE touching the map. */
        if (g_collecting ||
            ((g_drive.state != S3_DRIVE_OFF) &&
             (g_drive.state != S3_DRIVE_HALTED)) ||
            (g_drive.navigation.lifecycle != S3_RUN_INACTIVE) ||
            (s3_navigation_readiness(&g_drive.navigation, now) != 0U) ||
            !vehicle_controller_prepare_start(&g_drive.navigation))
        {
            return false;
        }
        /* s3_drive_start() also checks Buddy 2 (fresh, IDLE, braked, no fault).
         * If it refuses, undo only a map freeze this attempt newly took. */
        if (!s3_drive_start(&g_drive, now))
        {
            vehicle_controller_cancel_start();
            return false;
        }
        vehicle_controller_commit_start();
        return true;
    }
    if ((g_drive.navigation.lifecycle != S3_RUN_INACTIVE) || g_collecting ||
        (g_drive.state == S3_DRIVE_FAULT))
    {
        return false;
    }
    if (c->kind == S3_CONTROL_MAPPING)
    {
        return vehicle_controller_apply_mapping(c->map) &&
               s3_navigation_set_mapping(&g_drive.navigation, c->map);
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
    /* Same empty-range convention as s3_calibration_begin(): the minimum
     * starts at the 12-bit maximum. Starting it at 0 kept both minima at 0,
     * so every white/black pair looked overlapping and was rejected. */
    if (g_label == IR_SURFACE_WHITE)
    {
        g_collector[g_sensor].white =
            (s3_calibration_range_t){.minimum = S3_EMPTY_RANGE_MINIMUM};
    }
    else
    {
        g_collector[g_sensor].black =
            (s3_calibration_range_t){.minimum = S3_EMPTY_RANGE_MINIMUM};
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
static bool drive_running(s3_drive_state_t state)
{
    return (state >= S3_DRIVE_FOLLOW) && (state <= S3_DRIVE_REACQUIRE);
}
static void reset_completed(void)
{
    if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
    {
        g_shared.reset_pending = false;
        (void)tk_unl_mtx(g_mutex);
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
#if APP_DIAGNOSTICS
        uint32_t started_us = robot_diagnostics_now_us();
#endif
        bool pending = false;
        const bool was_running = drive_running(g_drive.state);
        /* STOP is handled before sampling, even if an earlier command queued.
         */
        if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
        {
            if (g_shared.stop)
            {
                command = (s3_control_request_t){.kind = S3_CONTROL_STOP};
                pending = true;
            }
            else if (g_shared.pending)
            {
                pending = s3_control_unpack(&g_command, &command);
            }
            g_shared.stop = false;
            g_shared.pending = false;
            (void)tk_unl_mtx(g_mutex);
        }
        if (pending && (command.kind == S3_CONTROL_STOP))
        {
            g_reset_waiting = false;
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
        /* A reset request is not a reset acknowledgement. The drive owner
         * waits for a newer safe feedback publication, or its 500 ms timeout.
         * Publish exactly one command result after that check, not the old
         * pre-reset FAULT snapshot. STOP above can always cancel the wait. */
        if (g_reset_waiting &&
            (s3_drive_reset_result(&g_drive) != S3_RESET_PENDING))
        {
            working.command_ok = s3_drive_reset_result(&g_drive) == S3_RESET_OK;
            working.command_sequence++;
            g_reset_waiting = false;
            reset_completed();
        }
        if (was_running)
        {
            /* Include the interval which caused a halt/fault; do not allow
             * later idle samples to erase it or add startup noise to a run. */
            s3_run_timing_record(&working.run_timing,
                                 g_drive.output.sample_gap_us,
                                 frame.timestamp_us,
                                 g_drive.navigation.config.max_sample_age_us);
        }
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
            const bool ok = execute(&command, now_us());
            if ((command.kind == S3_CONTROL_RESET) && ok &&
                (s3_drive_reset_result(&g_drive) == S3_RESET_PENDING))
            {
                g_reset_waiting = true;
            }
            else
            {
                working.command_ok = ok;
                working.command_sequence++;
                if (command.kind == S3_CONTROL_RESET) { reset_completed(); }
            }
            if ((command.kind == S3_CONTROL_START) && ok)
            {
                s3_run_timing_begin(&working.run_timing, now_us());
            }
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
        s3_timing_increment(&working.samples);
        if (g_drive.output.sample_gap_us > working.max_gap_us)
        {
            working.max_gap_us = g_drive.output.sample_gap_us;
        }
        if (g_drive.output.sample_gap_us > S3_SAMPLE_PERIOD_US)
        {
            s3_timing_increment(&working.above_target);
        }
        working.collecting = g_collecting;
        working.calibration_sensor = g_sensor;
        working.white = s3_calibration_summary(&g_collector[g_sensor].white);
        working.black = s3_calibration_summary(&g_collector[g_sensor].black);
        s3_status_packet_t packet;
        bool packed = s3_status_pack(&working, &packet);
        if (!packed) { s3_drive_stop(&g_drive); }
        if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
        {
            if (packed) { g_status = packet; }
            g_shared.status_ready = packed;
            (void)tk_unl_mtx(g_mutex);
        }
        if (packed) { vehicle_controller_observe(&working); }
#if APP_DIAGNOSTICS
        robot_diagnostics_record(ROBOT_DIAG_IR, started_us,
                                 robot_diagnostics_now_us());
#endif
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
        /* 1 s, not 200 ms: keeps operator prompts readable on USB. */
        (void)tk_dly_tsk(1000U);
        if (!s3_integration_status(&s))
        {
            continue;
        }
        tm_printf(
            (UB *)"[S3+S2] state=%u reason=%u ready=0x%x cal=0x%x motion=%s "
                  "odom_valid=%u adc=%d gap=%u boot_max=%u above10ms=%u "
                  "target_ms=%u\n",
            (unsigned)s.state, (unsigned)s.reason,
            (unsigned)s.navigation.readiness_blockers,
            (unsigned)s.navigation.accepted_calibration,
            motion_state_name(s.motion.state), (unsigned)s.motion.valid,
            (int)s.adc_status, (unsigned)s.navigation.sample_gap_us,
            (unsigned)s.max_gap_us, (unsigned)s.above_target,
            (unsigned)S3_SAMPLE_PERIOD_MS);
        tm_printf((UB *)"  RUN timing: start_us=%u active=%u samples=%u "
                        "max_gap_us=%u above10ms=%u over30ms=%u "
                        "last_over30_us=%u\n",
                  (unsigned)s.run_timing.started_us,
                  (unsigned)drive_running(s.state),
                  (unsigned)s.run_timing.samples,
                  (unsigned)s.run_timing.max_gap_us,
                  (unsigned)s.run_timing.above_target,
                  (unsigned)s.run_timing.above_safety,
                  (unsigned)s.run_timing.last_safety_us);
        tm_printf(
            (UB *)"  motion_id=%u stopped=%u odom_epoch=%u report_losses=%u "
                  "motion_fault=%s\n",
            (unsigned)s.motion.command_id, (unsigned)s.motion.braked,
            (unsigned)s.motion.epoch, (unsigned)s.motion.report_loss_count,
            motion_fault_name(s.motion.fault));
        tm_printf((UB *)"  command=%u ok=%u collecting=%u sensor=%u white n=%u "
                        "[%u,%u] black n=%u [%u,%u]\n",
                  (unsigned)s.command_sequence, (unsigned)s.command_ok,
                  (unsigned)s.collecting, (unsigned)s.calibration_sensor,
                  (unsigned)s.white.count, (unsigned)s.white.minimum,
                  (unsigned)s.white.maximum, (unsigned)s.black.count,
                  (unsigned)s.black.minimum, (unsigned)s.black.maximum);
        /* Live calibrated readings: 0 = white, 1000 = black. Meaningful
         * only after that sensor's calibration has been accepted. */
        tm_printf((UB *)"  IR dark: L(GP%u)=%u R(GP%u)=%u BAR(GP%u)=%u | "
                        "line present=%u error=%d junction=%u\n",
                  S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_LEFT),
                  (unsigned)s.navigation.sensor[S3_SENSOR_LEFT].darkness,
                  S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_RIGHT),
                  (unsigned)s.navigation.sensor[S3_SENSOR_RIGHT].darkness,
                  S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_BARCODE),
                  (unsigned)s.navigation.sensor[S3_SENSOR_BARCODE].darkness,
                  (unsigned)s.navigation.line.present,
                  (int)s.navigation.line.error,
                  (unsigned)s.navigation.line.junction_candidate);
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
/* Minimum forward wheel speed while line following. On the car (2026-10-26)
 * a strongly-off-centre start made Buddy 3's steering ask the inner wheel for
 * ~0.04 m/s; that wheel never started turning and Buddy 2's stall check
 * (0.6 s without encoder progress) faulted the run. The motor smoke test
 * moved reliably at 0.05 m/s. Raise only small positive requests; zero and
 * reverse requests pass through unchanged, and Buddy 3's steering algorithm
 * itself is untouched. Tune on the car; set to 0 to disable. */
#define S3_MIN_WHEEL_MPS (0.06f)
static float floor_wheel(float mps)
{
    return ((mps > 0.0f) && (mps < S3_MIN_WHEEL_MPS)) ? S3_MIN_WHEEL_MPS
                                                      : mps;
}
static motion_result_t velocity_with_floor(float left_mps, float right_mps)
{
    return motion_request_velocity(floor_wheel(left_mps),
                                   floor_wheel(right_mps));
}
#if S3_BUILTIN_CALIBRATION
/* Feed the measured ranges through the SAME validation as a button
 * calibration (sample count, non-overlap, reflectance init). Returns the
 * mask of sensors accepted; any sensor that fails still needs the buttons. */
static uint32_t apply_builtin_calibration(void)
{
    static const uint16_t table[S3_SENSOR_COUNT][4] = {
        S3_BUILTIN_CAL_BARCODE, S3_BUILTIN_CAL_LEFT, S3_BUILTIN_CAL_RIGHT};
    unsigned int i;
    for (i = 0U; i < S3_SENSOR_COUNT; i++)
    {
        s3_calibration_collector_t c;
        s3_calibration_begin(&c);
        (void)s3_calibration_add(&c, IR_SURFACE_WHITE, table[i][0]);
        (void)s3_calibration_add(&c, IR_SURFACE_BLACK, table[i][2]);
        for (unsigned int k = 0U; k < 10U; k++)
        {
            (void)s3_calibration_add(&c, IR_SURFACE_WHITE,
                                     (uint16_t)((table[i][0] + table[i][1]) / 2U));
            (void)s3_calibration_add(&c, IR_SURFACE_BLACK,
                                     (uint16_t)((table[i][2] + table[i][3]) / 2U));
        }
        (void)s3_calibration_add(&c, IR_SURFACE_WHITE, table[i][1]);
        (void)s3_calibration_add(&c, IR_SURFACE_BLACK, table[i][3]);
        (void)s3_navigation_apply_calibration(&g_drive.navigation,
                                              (s3_sensor_id_t)i, &c);
    }
    return g_drive.navigation.accepted_calibration;
}
#endif
int32_t s3_integration_init(void)
{
    T_CMTX mutex = {0};
    T_CCYC cycle = {0};
    s3_navigation_config_t cfg = s3_navigation_defaults();
    const s3_drive_port_t port = {
        velocity_with_floor,        motion_request_manoeuvre, stop,
        motion_get_report,          motion_command_id,        motion_reset,
        motion_navigation_heartbeat};
    const s3_drive_options_t options = s3_course_options();
    if (g_mutex > 0)
    {
        return E_OBJ;
    }
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    s3_course_navigation(&cfg);
    if (!s3_drive_init(&g_drive, &cfg, &port) ||
        !s3_drive_set_options(&g_drive, &options))
    {
        return E_PAR;
    }
#if S3_BUILTIN_CALIBRATION
    (void)apply_builtin_calibration();
#endif
    mutex.mtxatr = TA_INHERIT;
    g_mutex = tk_cre_mtx(&mutex);
    if (g_mutex < E_OK)
    {
        return g_mutex;
    }
    cycle.cycatr = TA_HLNG;
    cycle.cychdr = (FP)tick;
    cycle.cyctim = S3_SAMPLE_PERIOD_MS;
    cycle.cycphs = S3_SAMPLE_PERIOD_MS;
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
int32_t s3_integration_init(void) { return E_NOSPT; }
int32_t s3_integration_start(void) { return E_NOSPT; }
#endif
