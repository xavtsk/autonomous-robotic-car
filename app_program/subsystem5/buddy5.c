/** @file buddy5.c
 * @brief Own ultrasonic and servo scanning and publish protected obstacle
 * snapshots.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 */
#include "buddy5.h"
#include "scan_obstacle.h"
#include "scan_servo.h"
#include "ultrasonic_sensor.h"
#if APP_S5
#include <sys/sysdef.h>
#include "../subsystem2/motion.h"
#ifdef S5_TASK_TEST
#include "s5_task_main.h"
#endif

typedef struct
{
    unsigned b_is_cancel_pending : 1;
    unsigned b_is_busy : 1;
} scan_flags_t;

/* Raw readings of the last finished scan or side peek, for telemetry
 * (car/telemetry/scan_sample). Written by the S5 task, read by the
 * controller's report code under gh_obstacle_mutex. */
typedef struct
{
    profile_obstacle_sample_t sample[S5_SAMPLES_MAX];
    uint16_t                  scan_id;
    unsigned                  count : 5;
    unsigned                  b_is_peek : 1;
} scan_snapshot_t;

#ifndef S5_SCAN_LOG
/* Enable the USB report after each completed full scan. */
#define S5_SCAN_LOG 1
#endif

static ID       gh_obstacle_mutex;
static ID       gh_obstacle_task;
static uint8_t  g_published[S5_PACKET_BYTES];
static uint16_t g_controller_status;
/* Single aligned byte: written only by the S5 task, read whole by telemetry. */
static volatile uint8_t g_servo_deg;
static uint16_t         g_requested;

static uint8_t g_requested_angle; /* Zero requests a sweep. Otherwise set the
                                     peek angle. */

static scan_flags_t    g_shared;
static scan_snapshot_t g_last_scan;

static uint32_t now_us (void);
static void     servo_to (unsigned angle);
static void     scan_log (const scan_obstacle_t *    p_scan,
                          const profile_obstacle_t * p_profile);
static void     finish_sweep (const scan_obstacle_t * p_scan,
                              const s5_config_t *     p_cfg,
                              profile_obstacle_t *    p_profile);
static void     keep_samples (const scan_obstacle_t * p_scan, uint16_t value_id,
                              bool b_is_peek);
static void     publish (const s5_codec_observation_t * p_observation);
static bool     has_stationary_feedback (uint32_t now);
static void     obstacle_task (INT code, void * p_unused);

/** @brief Create the observation mutex and install an empty snapshot. Return
 * E_OBJ if already initialized, E_PAR for unconfirmed hardware, or the
 * kernel's mutex error.
 */
ER subsystem5_init (void)
{
    T_CMTX                 mutex_spec = {0};
    s5_codec_observation_t empty      = {0};

    if (gh_obstacle_mutex > 0)
    {
        return E_OBJ;
    }

    if (!S5_HARDWARE_CONFIRMED)
    {
        return E_PAR;
    }

    mutex_spec.mtxatr = TA_INHERIT;
    gh_obstacle_mutex = tk_cre_mtx(&mutex_spec);

    if (gh_obstacle_mutex <= 0)
    {
        return gh_obstacle_mutex;
    }

    (void)s5_codec_observation_pack(&empty, g_published);
    return E_OK;
}

/** @brief Start the obstacle owner on processor one. Return E_OBJ if the mutex
 * is unavailable or the task already exists, otherwise report the task
 * creation result.
 */
ER subsystem5_start (void)
{
    if ((gh_obstacle_mutex <= 0) || (gh_obstacle_task > 0))
    {
        return E_OBJ;
    }

    /* The scan task takes the kernel INT start code and void * context.
     * app_start_task stores the entry address in the kernel's FP field. */
    gh_obstacle_task = app_start_task((FP)obstacle_task, S5_PRI_MAIN, TP_PRC1,
                                      "obstacle-scan");
    return gh_obstacle_task < 0 ? gh_obstacle_task : E_OK;
}

/** @brief Poll the mutex, copy the packed observation, then decode outside the
 * lock. Distinguish lock contention, unavailable service and an
 * invalid packet. A successful read can still report no echo.
 */
s5_api_snapshot_result_t s5_api_read_snapshot (s5_codec_observation_t * p_out)
{
    uint8_t packet[S5_PACKET_BYTES];

    if ((0 == p_out) || (gh_obstacle_mutex <= 0))
    {
        return S5_SNAPSHOT_UNAVAILABLE;
    }

    ER value_er = tk_loc_mtx(gh_obstacle_mutex, TMO_POL);

    if (value_er != E_OK)
    {
        return E_TMOUT == value_er ? S5_SNAPSHOT_BUSY : S5_SNAPSHOT_UNAVAILABLE;
    }

    for (unsigned byte_index = 0; byte_index < S5_PACKET_BYTES; byte_index++)
    {
        packet[byte_index] = g_published[byte_index];
    }

    tk_unl_mtx(gh_obstacle_mutex);
    return s5_codec_observation_unpack(packet, p_out) ? S5_SNAPSHOT_OK
                                                      : S5_SNAPSHOT_INVALID;
}

/** @brief Return true when the caller receives a decoded observation. Range
 * validity is a separate field in that observation.
 */
bool s5_api_snapshot (s5_codec_observation_t * p_out)
{
    return s5_api_read_snapshot(p_out) == (unsigned int)S5_SNAPSHOT_OK;
}

/** @brief Queue a full sweep with a nonzero request ID. Poll the mutex and
 * return false while busy, cancelling, or already holding a pending
 * request.
 */
bool s5_api_request_scan (uint16_t value_id)
{
    bool b_did_succeed = false;

    if (((!value_id) || (gh_obstacle_mutex <= 0)) ||
        (tk_loc_mtx(gh_obstacle_mutex, TMO_POL) != E_OK))
    {
        return false;
    }

    if (((!g_requested) && (!g_shared.b_is_busy)) &&
        (!g_shared.b_is_cancel_pending))
    {
        g_requested        = value_id;
        g_shared.b_is_busy = true;
        b_did_succeed      = true;
    }

    tk_unl_mtx(gh_obstacle_mutex);
    return b_did_succeed;
}

/** @brief Queue one side reading for a nonzero ID at a validated angle in
 * degrees. The same mutex and busy checks apply as for a full sweep.
 */
bool s5_api_request_peek (uint16_t value_id, unsigned angle)
{
    bool b_did_succeed = false;

    if ((((!value_id) || (!scan_obstacle_peek_angle_valid(angle))) ||
         (gh_obstacle_mutex <= 0)) ||
        (tk_loc_mtx(gh_obstacle_mutex, TMO_POL) != E_OK))
    {
        return false;
    }

    if (((!g_requested) && (!g_shared.b_is_busy)) &&
        (!g_shared.b_is_cancel_pending))
    {
        g_requested        = value_id;
        g_requested_angle  = (uint8_t)angle;
        g_shared.b_is_busy = true;
        b_did_succeed      = true;
    }

    tk_unl_mtx(gh_obstacle_mutex);
    return b_did_succeed;
}

/** @brief With the mutex held, discard any queued request and ask the owner to
 * cancel its active scan. The owner handles sensor cancellation on its
 * next pass.
 */
void s5_api_cancel_scan (void)
{
    if ((gh_obstacle_mutex > 0) &&
        (tk_loc_mtx(gh_obstacle_mutex, TMO_FEVR) == E_OK))
    {
        g_shared.b_is_cancel_pending = true;
        g_requested                  = 0;
        g_requested_angle            = 0;
        tk_unl_mtx(gh_obstacle_mutex);
    }
}

/** @brief Mask and pack the controller's state, fault, route, leg, pass and
 * side-swap fields into one telemetry word. A failed mutex poll
 * retains the previous word.
 */
void s5_api_set_control_status (unsigned state, unsigned fault, unsigned route,
                                unsigned leg, unsigned pass, unsigned swapped)
{
    if ((gh_obstacle_mutex > 0) &&
        (tk_loc_mtx(gh_obstacle_mutex, TMO_POL) == E_OK))
    {
        /* The masks keep every input in its allotted 16-bit word field. */
        g_controller_status =
            (uint16_t)((state & 7U) | ((fault & 7U) << 3) |
                       ((route & 3U) << 6) | ((leg & 15U) << 8) |
                       ((pass & 7U) << 12) | ((swapped & 1U) << 15));
        tk_unl_mtx(gh_obstacle_mutex);
    }
}

/** @brief Read the last published servo command in degrees. Zero means no
 * successful position has been published yet.
 */
uint8_t s5_api_servo_angle (void)
{
    return g_servo_deg;
}

/** @brief Poll the mutex and copy the last scan ID, peek flag, count and that
 * many packed samples. The caller supplies S5_SAMPLES_MAX sample
 * slots. Invalid pointers or lock failure leave all outputs untouched.
 */
bool s5_api_scan_samples (uint16_t * p_scan_id, bool * pb_is_peek,
                          profile_obstacle_sample_t p_out[S5_SAMPLES_MAX],
                          unsigned *                p_count)
{
    if ((((((0 == p_scan_id) || (0 == pb_is_peek)) || (!p_out)) ||
          (0 == p_count)) ||
         (gh_obstacle_mutex <= 0)) ||
        (tk_loc_mtx(gh_obstacle_mutex, TMO_POL) != E_OK))
    {
        return false;
    }

    *p_count    = g_last_scan.count;
    *p_scan_id  = g_last_scan.scan_id;
    *pb_is_peek = g_last_scan.b_is_peek != 0U;

    for (unsigned sample_index = 0; sample_index < *p_count; sample_index++)
    {
        p_out[sample_index] = g_last_scan.sample[sample_index];
    }

    tk_unl_mtx(gh_obstacle_mutex);
    return true;
}

/** @brief Poll the mutex for the current packed bypass status. Return zero
 * when the owner or mutex is unavailable.
 */
uint16_t s5_api_controller_status (void)
{
    uint16_t value = 0;

    if ((gh_obstacle_mutex > 0) &&
        (tk_loc_mtx(gh_obstacle_mutex, TMO_POL) == E_OK))
    {
        value = g_controller_status;
        tk_unl_mtx(gh_obstacle_mutex);
    }

    return value;
}

#ifdef S5_TASK_TEST
/** @brief Reset owner registration and mailbox state between fixture runs. */
void s5_task_test_reset_mailbox (void)
{
    gh_obstacle_task             = 0;
    gh_obstacle_mutex            = 0;
    g_requested                  = 0;
    g_requested_angle            = 0;
    g_shared.b_is_busy           = false;
    g_shared.b_is_cancel_pending = false;
}

/** @brief Inject or clear the reserved high bit in the fixture snapshot. */
void s5_task_test_set_reserved_bit (bool b_should_set)
{
    if (b_should_set)
    {
        g_published[S5_PACKET_BYTES - 1U] |= 0x80U;
    }
    else
    {
        g_published[S5_PACKET_BYTES - 1U] &= 0x7fU;
    }
}
#endif

/** @brief Read the wrapping 32-bit microsecond clock used to time servo
 * settling, echoes and scans.
 */
static uint32_t now_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

/** @brief Command a servo angle in degrees. Publish it for telemetry only when
 * positioning succeeds and the angle is between 1 and 180 inclusive.
 */
static void servo_to (unsigned angle)
{
    if (((scan_servo_position(angle)) && (angle >= 1U)) && (angle <= 180U))
    {
        g_servo_deg = (uint8_t)angle;
    }
}

/** @brief Print the completed scan angles, millimetre readings and fitted
 * clearances when USB scan logging is enabled. Both inputs belong to
 * the obstacle owner.
 */
static void scan_log (const scan_obstacle_t *    p_scan,
                      const profile_obstacle_t * p_profile)
{
#if S5_SCAN_LOG
    tm_printf((UB *)"[S5SCAN] n=%u", (unsigned)p_scan->count);

    for (unsigned sample_index = 0; sample_index < p_scan->count;
         sample_index++)
    {
        bool b_is_range_valid;

        unsigned angle_deg;
        unsigned value_mm;
        if (profile_obstacle_sample_unpack(&p_scan->sample[sample_index],
                                           &angle_deg, &value_mm,
                                           &b_is_range_valid))
        {
            tm_printf((UB *)" %u:%u", angle_deg, value_mm);
        }
    }

    tm_printf((UB *)" | valid=%u width=%u L=%d R=%d fwd=%u lclear=%u rclear=%u "
                    "face_min=%u face_max=%u\n",
              (unsigned)p_profile->b_is_valid, (unsigned)p_profile->width_mm,
              (int)p_profile->left_edge_mm, (int)p_profile->right_edge_mm,
              (unsigned)p_profile->forward_mm,
              (unsigned)p_profile->left_clear_mm,
              (unsigned)p_profile->right_clear_mm,
              (unsigned)p_profile->face_width_min_mm,
              (unsigned)p_profile->face_width_max_mm);
#else
    (void)p_scan;
    (void)p_profile;
#endif
}

/** @brief Build and log the routing profile and optional face width bounds.
 * @param[in] p_scan Completed sweep owned by the obstacle task.
 * @param[in] p_cfg Stable obstacle configuration.
 * @param[out] p_profile Writable cleared profile, unchanged on scan failure.
 */
static void finish_sweep (const scan_obstacle_t * p_scan,
                          const s5_config_t *     p_cfg,
                          profile_obstacle_t *    p_profile)
{
    if ((!p_scan->b_has_failed) &&
        (profile_obstacle_build(p_scan->sample,
                                scan_obstacle_profile_count(p_scan), p_cfg,
                                p_profile)))
    {
        profile_obstacle_face_width(p_scan->sample, p_scan->count, p_cfg,
                                    p_profile);
    }
    scan_log(p_scan, p_profile);
}

/** @brief Copy the owner's finished scan into the telemetry snapshot under the
 * mutex. Where space permits, append a missing reading at the failed
 * angle. Preserve the scan ID and whether it was a side peek.
 */
static void keep_samples (const scan_obstacle_t * p_scan, uint16_t value_id,
                          bool b_is_peek)
{
    unsigned sample_count = p_scan->count;

    if (tk_loc_mtx(gh_obstacle_mutex, TMO_FEVR) != E_OK)
    {
        return;
    }

    for (unsigned sample_index = 0; sample_index < sample_count; sample_index++)
    {
        g_last_scan.sample[sample_index] = p_scan->sample[sample_index];
    }
    /* A failed scan also shows where it stopped: one "no reading" entry. */
    if (((p_scan->b_has_failed) && (sample_count < S5_SAMPLES_MAX)) &&
        (profile_obstacle_sample_pack(p_scan->angle, 0U, false,
                                      &g_last_scan.sample[sample_count])))
    {
        sample_count++;
    }

    g_last_scan.count     = sample_count;
    g_last_scan.scan_id   = value_id;
    g_last_scan.b_is_peek = b_is_peek ? 1U : 0U;
    tk_unl_mtx(gh_obstacle_mutex);
}

/** @brief Encode one observation before locking. Replace the shared packet
 * only after successful encoding and mutex acquisition.
 */
static void publish (const s5_codec_observation_t * p_observation)
{
    uint8_t packet[S5_PACKET_BYTES];

    if ((s5_codec_observation_pack(p_observation, packet)) &&
        (tk_loc_mtx(gh_obstacle_mutex, TMO_FEVR) == E_OK))
    {
        for (unsigned byte_index = 0; byte_index < S5_PACKET_BYTES;
             byte_index++)
        {
            g_published[byte_index] = packet[byte_index];
        }

        tk_unl_mtx(gh_obstacle_mutex);
    }
}

/** @brief Require valid, braked, fault-free motion feedback no older than 30
 * ms at the supplied wrapping microsecond timestamp.
 */
static bool has_stationary_feedback (uint32_t now)
{
    motion_feedback_t feedback = {0};
    return ((((motion_get_feedback(&feedback)) &&
              (feedback.b_is_feedback_valid)) &&
             (feedback.b_is_braked)) &&
            ((unsigned int)MOTION_FAULT_NONE == feedback.fault)) &&
           ((now - feedback.timestamp_us) <= 30000U);
}

/** @brief Own the ultrasonic sensor and scanning servo. Wait for motor
 * feedback before enabling the hardware. Service queued sweeps and
 * peeks while stationary, recenter after cancellation or timeout, and
 * publish front range only after the servo settles.
 * @details Consume cancellation and scan requests from the obstacle mutex.
 * Loss of stationary feedback aborts a scan. Range validity belongs to the
 * last forward ping and ages independently of the scan profile. Three
 * consecutive misses latch no echo. A completed side peek can update an
 * existing profile. A failed peek clears it.
 */
static void obstacle_task (INT code, void * p_unused)
{
    s5_codec_observation_t obs              = {0};
    scan_obstacle_t        scan             = {0};
    const s5_config_t      config           = s5_config_defaults();
    uint32_t               positioned       = 0;
    uint32_t               last_ping        = 0;
    uint32_t               ping_at          = 0;
    unsigned               angle            = 90;
    unsigned               failures         = 0;
    bool                   b_is_initialized = false;
    bool                   b_is_scanning    = false;
    bool                   b_is_pinging     = false;
    uint16_t peek_id = 0; /* Hold the request ID during a side peek. */
    (void)code;
    (void)p_unused;

    for (;;)
    {
        const uint32_t now                  = now_us();
        uint16_t       req                  = 0;
        uint16_t       value_mm             = 0;
        unsigned       req_angle            = 0;
        bool           b_has_cancel_request = false;
        bool           b_is_range_valid     = false;
        bool           b_has_result         = false;

        if (tk_loc_mtx(gh_obstacle_mutex, TMO_FEVR) == E_OK)
        {
            b_has_cancel_request         = g_shared.b_is_cancel_pending;
            g_shared.b_is_cancel_pending = false;
            req                          = g_requested;
            g_requested                  = 0;
            req_angle                    = g_requested_angle;
            g_requested_angle            = 0;
            tk_unl_mtx(gh_obstacle_mutex);
        }
        /* Buddy 2 starts first and owns PIO0 instructions 0..23. Wait for its
         * actual publication, not task creation, before enabling SM2. */
        if (!b_is_initialized)
        {
            motion_feedback_t feedback = {0};

            if ((motion_get_feedback(&feedback)) &&
                (feedback.b_is_feedback_valid))
            {
                b_is_initialized =
                    (ultrasonic_sensor_init()) && (scan_servo_init());

                if (b_is_initialized)
                {
                    positioned = now;
                    last_ping  = now;
                }
            }

            publish(&obs);
            tk_dly_tsk(10);
            continue;
        }

        if (b_has_cancel_request ||
            (b_is_scanning && (!has_stationary_feedback(now))))
        {
            ultrasonic_sensor_cancel();
            b_is_pinging  = false;
            b_is_scanning = false;
            servo_to(90);
            angle              = 90;
            positioned         = now;
            obs.b_is_scan_done = false;
            obs.profile        = (profile_obstacle_t){0};
            peek_id            = 0;

            if (tk_loc_mtx(gh_obstacle_mutex, TMO_FEVR) == E_OK)
            {
                g_shared.b_is_busy = false;
                tk_unl_mtx(gh_obstacle_mutex);
            }
        }

        if ((req && req_angle) && (!b_has_cancel_request))
        {
            /* Side peek: the last profile stays published (telemetry) until the
             * reading completes under the new id; moving = failed peek. */
            if ((!has_stationary_feedback(now)) ||
                (!scan_obstacle_begin_at(&scan, now, req_angle)))
            {
                obs.scan_id        = req;
                obs.b_is_scan_done = true;
                obs.scan_us        = now;
                obs.profile        = (profile_obstacle_t){0};
            }
            else
            {
                ultrasonic_sensor_cancel();
                b_is_pinging  = false;
                b_is_scanning = true;
                peek_id       = req;
                angle         = scan.angle;
                servo_to(angle);
                positioned = now;
            }
        }
        else if (req && (!b_has_cancel_request))
        {
            obs.scan_id        = req;
            obs.b_is_scan_done = false;
            obs.profile        = (profile_obstacle_t){0};

            if (!has_stationary_feedback(now))
            {
                obs.b_is_scan_done = true;
                obs.scan_us        = now;
            }
            else
            {
                ultrasonic_sensor_cancel();
                b_is_pinging = false;
                scan_obstacle_begin(&scan, now);
                b_is_scanning = true;
                angle         = scan.angle;
                servo_to(angle);
                positioned = now;
            }
        }
        else
        {
            /* Keep the current scan request and servo position. */
        }

        if (b_is_pinging)
        {
            b_has_result =
                ultrasonic_sensor_poll(now, &value_mm, &b_is_range_valid);

            if (b_has_result)
            {
                b_is_pinging = false;
            }
        }
        else if (((now - positioned) >= 250000U) &&
                 ((now - last_ping) >= S5_PING_PERIOD_US))
        {
            ping_at   = now;
            last_ping = now;

            b_is_pinging = ultrasonic_sensor_trigger(now);

            if (!b_is_pinging)
            {
                b_has_result     = true;
                b_is_range_valid = false;
            }
        }
        else
        {
            /* Wait for the current echo or the next permitted ping. */
        }

        if (b_has_result)
        {
            if (b_is_range_valid)
            {
                failures          = 0;
                obs.b_has_no_echo = false;
            }
            else
            {
                failures          = failures < 3U ? failures + 1U : failures;
                obs.b_has_no_echo = failures >= 3U;
            }

            if (b_is_scanning)
            {
                const unsigned old = scan.angle;
                scan_obstacle_accept(&scan, now, b_is_range_valid, value_mm);

                if (scan.b_is_done)
                {
                    obs.b_is_scan_done = true;
                    obs.scan_us        = now;
                    keep_samples(&scan, peek_id ? peek_id : obs.scan_id,
                                 peek_id != 0U);

                    if (peek_id)
                    {
                        obs.scan_id = peek_id;
                        peek_id     = 0;

                        if ((scan.b_has_failed) ||
                            (!profile_obstacle_merge_peek(&obs.profile,
                                                          &scan.sample[0])))
                        {
                            obs.profile = (profile_obstacle_t){0};
                        }
                    }
                    else
                    {
                        finish_sweep(&scan, &config, &obs.profile);
                    }

                    b_is_scanning = false;
                    angle         = 90;
                    servo_to(angle);
                    positioned = now;
                }
                else if (scan.angle != old)
                {
                    angle = scan.angle;
                    servo_to(angle);
                    positioned = now;
                }
                else
                {
                    /* The servo is already at the requested scan angle. */
                }
            }
            else
            {
                /* Between scans only a forward ping can update the range. */
                if (90U == angle)
                {
                    /* A single bad ping keeps the last good reading (still
                     * timestamped at its own ping, so it ages out after
                     * S5_RANGE_FRESH_US). Three in a row latch no_echo and
                     * clear it: then the controller faults. */
                    if (b_is_range_valid)
                    {
                        obs.range_us         = ping_at;
                        obs.range_mm         = value_mm;
                        obs.b_is_range_valid = true;
                    }
                    else if (obs.b_has_no_echo)
                    {
                        obs.range_mm         = 0U;
                        obs.b_is_range_valid = false;
                    }
                    else
                    {
                        /* Keep the last good range until the no-echo limit is
                         * reached. */
                    }
                }
            }
        }

        if (b_is_scanning && ((now - scan.started_us) >= S5_SCAN_TIMEOUT_US))
        {
            ultrasonic_sensor_cancel();
            b_is_pinging      = false;
            b_is_scanning     = false;
            scan.b_has_failed = !scan.b_has_profile;
            keep_samples(&scan, peek_id ? peek_id : obs.scan_id, peek_id != 0U);

            if (peek_id)
            {
                obs.scan_id = peek_id;
                peek_id     = 0;
            }

            obs.b_is_scan_done = true;
            obs.scan_us        = now;
            obs.profile        = (profile_obstacle_t){0};
            if (scan.b_has_profile)
            {
                finish_sweep(&scan, &config, &obs.profile);
            }
            angle = 90;
            servo_to(angle);
            positioned = now;
        }

        if ((!b_is_scanning) &&
            (tk_loc_mtx(gh_obstacle_mutex, TMO_FEVR) == E_OK))
        {
            g_shared.b_is_busy = false;
            tk_unl_mtx(gh_obstacle_mutex);
        }
        /* Front ranging is unavailable while the servo faces elsewhere. */
        if (b_is_scanning || ((now - positioned) < 250000U))
        {
            obs.b_is_range_valid = false;
            obs.range_mm         = 0;
        }

        publish(&obs);
        tk_dly_tsk(10);
    }
}
#else
/** @brief Return E_OK so startup can proceed with obstacle handling disabled.
 */
ER subsystem5_init (void)
{
    return E_OK;
}

/** @brief Return E_OK without creating an obstacle task.
 */
ER subsystem5_start (void)
{
    return E_OK;
}

/** @brief Return false and leave the destination untouched.
 */
bool s5_api_snapshot (s5_codec_observation_t * p_out)
{
    (void)p_out;
    return false;
}

/** @brief Return S5_SNAPSHOT_UNAVAILABLE without writing the caller's
 * observation.
 */
s5_api_snapshot_result_t s5_api_read_snapshot (s5_codec_observation_t * p_out)
{
    (void)p_out;
    return S5_SNAPSHOT_UNAVAILABLE;
}

/** @brief Return false because the disabled owner cannot accept a sweep.
 */
bool s5_api_request_scan (uint16_t scan_id)
{
    (void)scan_id;
    return false;
}

/** @brief Return false without retaining the request ID or side angle.
 */
bool s5_api_request_peek (uint16_t scan_id, unsigned angle)
{
    (void)scan_id;
    (void)angle;
    return false;
}

/** @brief Ignore the cancellation request in the disabled profile.
 */
void s5_api_cancel_scan (void)
{
}

/** @brief Ignore the supplied status fields in the disabled profile.
 */
void s5_api_set_control_status (unsigned state, unsigned fault, unsigned route,
                                unsigned leg, unsigned pass, unsigned swapped)
{
    (void)state;
    (void)fault;
    (void)route;
    (void)leg;
    (void)pass;
    (void)swapped;
}

/** @brief Return zero for the unavailable controller status.
 */
uint16_t s5_api_controller_status (void)
{
    return 0;
}

/** @brief Return zero for the unavailable servo angle.
 */
uint8_t s5_api_servo_angle (void)
{
    return 0;
}

/** @brief Return false without modifying any scan metadata or sample storage.
 */
bool s5_api_scan_samples (uint16_t * p_scan_id, bool * pb_is_peek,
                          profile_obstacle_sample_t p_out[S5_SAMPLES_MAX],
                          unsigned *                p_count)
{
    (void)p_scan_id;
    (void)pb_is_peek;
    (void)p_out;
    (void)p_count;
    return false;
}
#endif

/* End of file. */

