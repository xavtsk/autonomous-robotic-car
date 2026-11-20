/** @file buddy5.c
 * @brief Own ultrasonic and servo scanning and publish protected obstacle
 * snapshots.
 */
#include "buddy5.h"
#include "scan_obstacle.h"
#include "scan_servo.h"
#include "ultrasonic_sensor.h"
#if APP_S5
#include <sys/sysdef.h>
#include "../subsystem2/motion.h"
static ID g_mutex_id, g_task_id;
static uint8_t g_published[S5_PACKET_BYTES];
static uint16_t g_controller_status;
/* Single aligned byte: written only by the S5 task, read whole by telemetry. */
static volatile uint8_t g_servo_deg;
static uint16_t g_requested;
static uint8_t g_requested_angle; /* 0 = full scan, else side peek angle */

static struct
{
    unsigned cancel : 1, busy : 1;
} g_shared;

/* Raw readings of the last finished scan or side peek, for telemetry
 * (car/telemetry/scan_sample). Written by the S5 task, read by the
 * controller's report code; both under mutex_id. */
static struct
{
    s5_sample_t sample[S5_SAMPLES_MAX];
    uint16_t id;
    unsigned count : 5, peek : 1;
} g_last_scan;

static uint32_t now_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

static void servo_to (unsigned angle)
{
    if (scan_servo_position(angle) && angle >= 1U && angle <= 180U)
    {
        g_servo_deg = (uint8_t)angle;
    }
}
#ifndef S5_SCAN_LOG
/* TEMP (28 Oct): print every full-scan reading on USB */
#define S5_SCAN_LOG 1
#endif
static void scan_log (const s5_scan_t * p_s, const s5_profile_t * p_p)
{
#if S5_SCAN_LOG
    tm_printf((UB *)"[S5SCAN] n=%u", (unsigned)p_s->count);
    for (unsigned value_i = 0; value_i < p_s->count; value_i++)
    {
        unsigned value_a, value_mm;
        bool value_v;
        if (s5_sample_unpack(&p_s->sample[value_i], &value_a, &value_mm,
                             &value_v))
        {
            tm_printf((UB *)" %u:%u", value_a, value_mm);
        }
    }
    tm_printf(
        (UB *)" | valid=%u width=%u L=%d R=%d fwd=%u lclear=%u rclear=%u\n",
        (unsigned)p_p->valid, (unsigned)p_p->width_mm, (int)p_p->left_edge_mm,
        (int)p_p->right_edge_mm, (unsigned)p_p->forward_mm,
        (unsigned)p_p->left_clear_mm, (unsigned)p_p->right_clear_mm);
#else
    (void)p_s;
    (void)p_p;
#endif
}

static void keep_samples (const s5_scan_t * p_s, uint16_t value_id, bool peek)
{
    unsigned value_n = p_s->count;
    if (tk_loc_mtx(g_mutex_id, TMO_FEVR) != E_OK)
    {
        return;
    }
    for (unsigned value_i = 0; value_i < value_n; value_i++)
    {
        g_last_scan.sample[value_i] = p_s->sample[value_i];
    }
    /* A failed scan also shows where it stopped: one "no reading" entry. */
    if (p_s->failed && value_n < S5_SAMPLES_MAX &&
        s5_sample_pack(p_s->angle, 0U, false, &g_last_scan.sample[value_n]))
    {
        value_n++;
    }
    g_last_scan.count = value_n;
    g_last_scan.id = value_id;
    g_last_scan.peek = peek ? 1U : 0U;
    tk_unl_mtx(g_mutex_id);
}

static void publish (const s5_observation_t * p_o)
{
    uint8_t packet[S5_PACKET_BYTES];
    if (s5_observation_pack(p_o, packet) &&
        tk_loc_mtx(g_mutex_id, TMO_FEVR) == E_OK)
    {
        for (unsigned value_i = 0; value_i < S5_PACKET_BYTES; value_i++)
        {
            g_published[value_i] = packet[value_i];
        }
        tk_unl_mtx(g_mutex_id);
    }
}

static bool stationary (uint32_t now)
{
    motion_feedback_t value_m = {0};
    return motion_get_feedback(&value_m) && value_m.valid && value_m.braked &&
           value_m.fault == MOTION_FAULT_NONE &&
           (uint32_t)(now - value_m.timestamp_us) <= 30000U;
}

static void obstacle_task (INT code, void * p_unused)
{
    s5_observation_t obs = {0};
    s5_scan_t scan = {0};
    const s5_config_t config = s5_defaults();
    uint32_t positioned = 0, last_ping = 0, ping_at = 0;
    unsigned angle = 90, failures = 0;
    bool initialized = false, scanning = false, pinging = false;
    uint16_t peek_id = 0; /* nonzero while a side peek is running */
    (void)code;
    (void)p_unused;
    for (;;)
    {
        uint32_t now = now_us();
        uint16_t req = 0, value_mm = 0;
        unsigned req_angle = 0;
        bool cancel = false, valid = false, result = false;
        if (tk_loc_mtx(g_mutex_id, TMO_FEVR) == E_OK)
        {
            cancel = g_shared.cancel;
            g_shared.cancel = false;
            req = g_requested;
            g_requested = 0;
            req_angle = g_requested_angle;
            g_requested_angle = 0;
            tk_unl_mtx(g_mutex_id);
        }
        /* Buddy 2 starts first and owns PIO0 instructions 0..23. Wait for its
         * actual publication, not task creation, before enabling SM2. */
        if (!initialized)
        {
            motion_feedback_t value_m = {0};
            if (motion_get_feedback(&value_m) && value_m.valid)
            {
                initialized = ultrasonic_sensor_init() && scan_servo_init();
                if (initialized)
                {
                    positioned = now;
                    last_ping = now;
                }
            }
            publish(&obs);
            tk_dly_tsk(10);
            continue;
        }
        if (cancel || (scanning && !stationary(now)))
        {
            ultrasonic_sensor_cancel();
            pinging = false;
            scanning = false;
            servo_to(90);
            angle = 90;
            positioned = now;
            obs.scan_done = false;
            obs.profile = (s5_profile_t){0};
            peek_id = 0;
            if (tk_loc_mtx(g_mutex_id, TMO_FEVR) == E_OK)
            {
                g_shared.busy = false;
                tk_unl_mtx(g_mutex_id);
            }
        }
        if (req && req_angle && !cancel)
        {
            /* Side peek: the last profile stays published (telemetry) until the
             * reading completes under the new id; moving = failed peek. */
            if (!stationary(now) || !s5_scan_begin_at(&scan, now, req_angle))
            {
                obs.scan_id = req;
                obs.scan_done = true;
                obs.scan_us = now;
                obs.profile = (s5_profile_t){0};
            }
            else
            {
                ultrasonic_sensor_cancel();
                pinging = false;
                scanning = true;
                peek_id = req;
                angle = scan.angle;
                servo_to(angle);
                positioned = now;
            }
        }
        else if (req && !cancel)
        {
            obs.scan_id = req;
            obs.scan_done = false;
            obs.profile = (s5_profile_t){0};
            if (!stationary(now))
            {
                obs.scan_done = true;
                obs.scan_us = now;
            }
            else
            {
                ultrasonic_sensor_cancel();
                pinging = false;
                s5_scan_begin(&scan, now);
                scanning = true;
                angle = scan.angle;
                servo_to(angle);
                positioned = now;
            }
        }
        if (pinging)
        {
            result = ultrasonic_sensor_poll(now, &value_mm, &valid);
            if (result)
            {
                pinging = false;
            }
        }
        else if ((uint32_t)(now - positioned) >= 250000U &&
                 (uint32_t)(now - last_ping) >= S5_PING_PERIOD_US)
        {
            last_ping = ping_at = now;
            pinging = ultrasonic_sensor_trigger(now);
            if (!pinging)
            {
                result = true;
                valid = false;
            }
        }
        if (result)
        {
            if (valid)
            {
                failures = 0;
                obs.no_echo = false;
            }
            else
            {
                if (failures < 3U)
                {
                    failures++;
                }
                obs.no_echo = failures >= 3U;
            }
            if (scanning)
            {
                unsigned old = scan.angle;
                s5_scan_accept(&scan, now, valid, value_mm);
                if (scan.done)
                {
                    obs.scan_done = true;
                    obs.scan_us = now;
                    keep_samples(&scan, peek_id ? peek_id : obs.scan_id,
                                 peek_id != 0U);
                    if (peek_id)
                    {
                        obs.scan_id = peek_id;
                        peek_id = 0;
                        if (scan.failed ||
                            !s5_peek_merge(&obs.profile, &scan.sample[0]))
                        {
                            obs.profile = (s5_profile_t){0};
                        }
                    }
                    else
                    {
                        if (!scan.failed)
                        {
                            (void)s5_profile_build(scan.sample, scan.count,
                                                   &config, &obs.profile);
                        }
                        scan_log(&scan, &obs.profile);
                    }
                    scanning = false;
                    angle = 90;
                    servo_to(angle);
                    positioned = now;
                }
                else if (scan.angle != old)
                {
                    angle = scan.angle;
                    servo_to(angle);
                    positioned = now;
                }
            }
            else if (angle == 90U)
            {
                /* A single bad ping keeps the last good reading (still
                 * timestamped at its own ping, so it ages out after
                 * S5_RANGE_FRESH_US). Three in a row latch no_echo and clear
                 * it: then the controller faults. */
                if (valid)
                {
                    obs.range_us = ping_at;
                    obs.range_mm = value_mm;
                    obs.range_valid = true;
                }
                else if (obs.no_echo)
                {
                    obs.range_mm = 0U;
                    obs.range_valid = false;
                }
            }
        }
        if (scanning && (uint32_t)(now - scan.started_us) >= S5_SCAN_TIMEOUT_US)
        {
            ultrasonic_sensor_cancel();
            pinging = false;
            scanning = false;
            scan.failed = 1U;
            keep_samples(&scan, peek_id ? peek_id : obs.scan_id, peek_id != 0U);
            if (peek_id)
            {
                obs.scan_id = peek_id;
                peek_id = 0;
            }
            obs.scan_done = true;
            obs.scan_us = now;
            obs.profile = (s5_profile_t){0};
            angle = 90;
            servo_to(angle);
            positioned = now;
        }
        if (!scanning && tk_loc_mtx(g_mutex_id, TMO_FEVR) == E_OK)
        {
            g_shared.busy = false;
            tk_unl_mtx(g_mutex_id);
        }
        /* Front ranging is unavailable while the servo faces elsewhere. */
        if (scanning || (uint32_t)(now - positioned) < 250000U)
        {
            obs.range_valid = false;
            obs.range_mm = 0;
        }
        publish(&obs);
        tk_dly_tsk(10);
    }
}

ER subsystem5_init (void)
{
    T_CMTX value_m = {0};
    s5_observation_t empty = {0};
    if (g_mutex_id > 0)
    {
        return E_OBJ;
    }
    if (!S5_HARDWARE_CONFIRMED)
    {
        return E_PAR;
    }
    value_m.mtxatr = TA_INHERIT;
    g_mutex_id = tk_cre_mtx(&value_m);
    if (g_mutex_id <= 0)
    {
        return (ER)g_mutex_id;
    }
    (void)s5_observation_pack(&empty, g_published);
    return E_OK;
}

ER subsystem5_start (void)
{
    if (g_mutex_id <= 0 || g_task_id > 0)
    {
        return E_OBJ;
    }
    g_task_id = app_start_task((FP)obstacle_task, S5_PRI_MAIN, TP_PRC1,
                               "obstacle-scan");
    return g_task_id < 0 ? (ER)g_task_id : E_OK;
}

s5_snapshot_result_t subsystem5_read_snapshot (s5_observation_t * p_out)
{
    uint8_t value_p[S5_PACKET_BYTES];
    ER value_er;
    if (!p_out || g_mutex_id <= 0)
    {
        return S5_SNAPSHOT_UNAVAILABLE;
    }
    value_er = tk_loc_mtx(g_mutex_id, TMO_POL);
    if (value_er != E_OK)
    {
        return value_er == E_TMOUT ? S5_SNAPSHOT_BUSY : S5_SNAPSHOT_UNAVAILABLE;
    }
    for (unsigned value_i = 0; value_i < S5_PACKET_BYTES; value_i++)
    {
        value_p[value_i] = g_published[value_i];
    }
    tk_unl_mtx(g_mutex_id);
    return s5_observation_unpack(value_p, p_out) ? S5_SNAPSHOT_OK
                                                 : S5_SNAPSHOT_INVALID;
}

bool subsystem5_snapshot (s5_observation_t * p_out)
{
    return subsystem5_read_snapshot(p_out) == S5_SNAPSHOT_OK;
}

bool subsystem5_request_scan (uint16_t value_id)
{
    bool value_ok = false;
    if (!value_id || g_mutex_id <= 0 || tk_loc_mtx(g_mutex_id, TMO_POL) != E_OK)
    {
        return false;
    }
    if (!g_requested && !g_shared.busy && !g_shared.cancel)
    {
        g_requested = value_id;
        g_shared.busy = true;
        value_ok = true;
    }
    tk_unl_mtx(g_mutex_id);
    return value_ok;
}

bool subsystem5_request_peek (uint16_t value_id, unsigned angle)
{
    bool value_ok = false;
    if (!value_id || !s5_peek_angle_valid(angle) || g_mutex_id <= 0 ||
        tk_loc_mtx(g_mutex_id, TMO_POL) != E_OK)
    {
        return false;
    }
    if (!g_requested && !g_shared.busy && !g_shared.cancel)
    {
        g_requested = value_id;
        g_requested_angle = (uint8_t)angle;
        g_shared.busy = true;
        value_ok = true;
    }
    tk_unl_mtx(g_mutex_id);
    return value_ok;
}

void subsystem5_cancel_scan (void)
{
    if (g_mutex_id > 0 && tk_loc_mtx(g_mutex_id, TMO_FEVR) == E_OK)
    {
        g_shared.cancel = true;
        g_requested = 0;
        g_requested_angle = 0;
        tk_unl_mtx(g_mutex_id);
    }
}

void subsystem5_set_control_status (unsigned state, unsigned fault,
                                    unsigned route, unsigned leg, unsigned pass,
                                    unsigned swapped)
{
    if (g_mutex_id > 0 && tk_loc_mtx(g_mutex_id, TMO_POL) == E_OK)
    {
        g_controller_status =
            (uint16_t)((state & 7U) | ((fault & 7U) << 3) |
                       ((route & 3U) << 6) | ((leg & 15U) << 8) |
                       ((pass & 7U) << 12) | ((swapped & 1U) << 15));
        tk_unl_mtx(g_mutex_id);
    }
}

uint8_t subsystem5_servo_angle (void)
{
    return g_servo_deg;
}

bool subsystem5_scan_samples (uint16_t * p_id, bool * p_peek,
                              s5_sample_t p_out[S5_SAMPLES_MAX],
                              unsigned * p_count)
{
    if (!p_id || !p_peek || !p_out || !p_count || g_mutex_id <= 0 ||
        tk_loc_mtx(g_mutex_id, TMO_POL) != E_OK)
    {
        return false;
    }
    *p_count = g_last_scan.count;
    *p_id = g_last_scan.id;
    *p_peek = g_last_scan.peek != 0U;
    for (unsigned value_i = 0; value_i < *p_count; value_i++)
    {
        p_out[value_i] = g_last_scan.sample[value_i];
    }
    tk_unl_mtx(g_mutex_id);
    return true;
}

uint16_t subsystem5_controller_status (void)
{
    uint16_t value = 0;
    if (g_mutex_id > 0 && tk_loc_mtx(g_mutex_id, TMO_POL) == E_OK)
    {
        value = g_controller_status;
        tk_unl_mtx(g_mutex_id);
    }
    return value;
}
#else
ER subsystem5_init (void)
{
    return E_OK;
}

ER subsystem5_start (void)
{
    return E_OK;
}

bool subsystem5_snapshot (s5_observation_t * out)
{
    (void)out;
    return false;
}

s5_snapshot_result_t subsystem5_read_snapshot (s5_observation_t * out)
{
    (void)out;
    return S5_SNAPSHOT_UNAVAILABLE;
}

bool subsystem5_request_scan (uint16_t id)
{
    (void)id;
    return false;
}

bool subsystem5_request_peek (uint16_t id, unsigned angle)
{
    (void)id;
    (void)angle;
    return false;
}

void subsystem5_cancel_scan (void)
{
}

void subsystem5_set_control_status (unsigned state, unsigned fault,
                                    unsigned route, unsigned leg, unsigned pass,
                                    unsigned swapped)
{
    (void)state;
    (void)fault;
    (void)route;
    (void)leg;
    (void)pass;
    (void)swapped;
}

uint16_t subsystem5_controller_status (void)
{
    return 0;
}

uint8_t subsystem5_servo_angle (void)
{
    return 0;
}

bool subsystem5_scan_samples (uint16_t * id, bool * peek,
                              s5_sample_t out[S5_SAMPLES_MAX], unsigned * count)
{
    (void)id;
    (void)peek;
    (void)out;
    (void)count;
    return false;
}
#endif

/* End of file. */

