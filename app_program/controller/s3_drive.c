/** @file s3_drive.c
 * @brief Own line following, manoeuvres, fault handling and optional avoidance
 * arbitration. The integration task calls this module with samples and motion
 * callbacks. The drive state machine has no GPIO or RTOS dependencies.
 * @see app_program/controller/README.md for the motion and report paths.
 * Mask constants here are nonnegative. Converting to unsigned preserves
 * each mask while making bitwise operations unsigned.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 * The selected unsigned bit fields fit in int before promotion.
 * Unsigned casts spell out the original modulo conversion.
 */
#include "s3_drive.h"
#include "../app_types.h"
#if APP_S5
#include "../subsystem5/s5_api.h"
/* Publish the bypass status (state, fault, route, leg, leg-3 step, swap) for
 * telemetry; the controller task is the only writer. */
/* Telemetry-only leg numbers (the bypass itself still uses legs 0..8):
 * 9 = reversing before the 45-degree out-turn (too close to turn),
 * 10 = turning to face the obstacle before the second scan. */
#define S5_TELEMETRY_LEG_REVERSE 9U
#define S5_TELEMETRY_LEG_AIM 10U

#endif
#define S3_BRAKE_WAIT_US (500000U)
#define S3_MANOEUVRE_WAIT_US (10000000U)
#define S3_REACQUIRE_WAIT_US (1000000U)
#define S3_PENDING_WAIT_US (5000000U)
#define S3_MIN_ADVANCE_M                                                       \
    (0.003f) /* Skip advances shorter than this distance. */

static inline void finish_braking_request (s3_drive_t * p_drive, uint32_t now);
static bool        finite_f (float32_t operand);
static bool        feedback_fresh (const s3_drive_t * p_drive, uint32_t now);
static void        halt_drive (s3_drive_t * p_drive, s3_drive_state_t state,
                               s3_drive_reason_t reason);
static void        finish_reset (s3_drive_t * p_drive, uint32_t now);
static bool        request_turn (s3_drive_t * p_drive);
static bool        start_manoeuvre (s3_drive_t * p_drive);
static bool        straight_across (const s3_drive_t * p_drive);
static void        request_stop (s3_drive_t * p_drive, uint32_t now);
static inline bool feedback_is_later (uint32_t received, uint32_t reference);

static inline void resume_after_manoeuvre (s3_drive_t * p_drive, uint32_t now,
                                           bool b_was_turn);
static bool        stopped_after_request (const s3_drive_t * p_drive);
static bool        receive_reports (s3_drive_t * p_drive, bool b_is_running,
                                    bool * pb_is_complete);
static uint32_t    search_leg_us (const s3_drive_t * p_drive);
static void        start_search (s3_drive_t * p_drive, uint32_t now);
static void        search_step (s3_drive_t * p_drive, uint32_t now);
static void        follow_line (s3_drive_t * p_drive, uint32_t now);
static inline void submit_follow_speed (s3_drive_t * p_drive, float32_t left,
                                        float32_t right, uint32_t now);

#if APP_S5
static void   report_obstacle_status (const s3_drive_t * p_drive);
static void   turn_track (s3_drive_t * p_drive, float32_t left, float32_t right,
                          uint32_t now);
static int8_t turn_pct (int16_t turn_q);
static bool   obstacle_snapshot (s3_drive_t * p_drive, uint32_t now,
                                 s5_codec_observation_t * p_out);
static bool   obstacle_control (s3_drive_t * p_drive, uint32_t now,
                                bool b_is_done);

#endif

/** @brief Return drive defaults by value, with search and pre-turn axle
 * advance disabled.
 */
s3_drive_options_t s3_drive_default_options (void)
{
    return (s3_drive_options_t){false, 0.0f, 0.10f, 0,     S3_PENDING_WAIT_US,
                                0U,    0.0f, 0.05f, 0.116f};
}

/** @brief Halt a configured drive on a terrain fault. A null or unconfigured
 * drive has no effect.
 */
void s3_drive_terrain_fault (s3_drive_t * p_drive)
{
    if (p_drive && (p_drive->b_is_configured))
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_TERRAIN_FAULT);
    }
}

/** @brief Initialize caller-owned drive storage, copy the motor callbacks and
 * initialize navigation. A non-null destination is cleared before
 * validating the mandatory callbacks. Return true on successful setup.
 */
bool s3_drive_init (s3_drive_t *                     p_drive,
                    const line_navigation_config_t * p_cfg,
                    const s3_drive_port_t *          p_port)
{
    if (0 == p_drive)
    {
        return false;
    }

    *p_drive = (s3_drive_t){false};

    if (((((((((0 == p_port) || (0 == p_port->p_velocity)) ||
              (0 == p_port->p_manoeuvre)) ||
             (0 == p_port->p_stop)) ||
            (0 == p_port->p_report)) ||
           (0 == p_port->p_command_id)) ||
          (0 == p_port->p_reset)) ||
         (0 == p_port->p_heartbeat)) ||
        (!line_navigation_init(&p_drive->navigation, p_cfg)))
    {
        return false;
    }

    p_drive->port            = *p_port;
    p_drive->options         = s3_drive_default_options();
    p_drive->b_is_configured = true;
#if APP_S5
    const s5_config_t obstacle_config = s5_config_defaults();
    avoid_obstacle_init(&p_drive->obstacle, &obstacle_config);
#endif
    return true;
}

/** @brief Validate and copy the turn advance, search and timeout settings.
 * Distances are metres, speeds are m/s and timeouts are microseconds.
 * A rejected request leaves the installed options intact.
 */
bool s3_drive_set_options (s3_drive_t *               p_drive,
                           const s3_drive_options_t * p_options)
{
    if ((((((((((((((((0 == p_drive) || (!p_drive->b_is_configured)) ||
                     (0 == p_options)) ||
                    (((p_drive->state != (unsigned int)S3_DRIVE_OFF) &&
                      (p_drive->state != (unsigned int)S3_DRIVE_HALTED)) &&
                     (p_drive->state != (unsigned int)S3_DRIVE_FAULT))) ||
                   (!finite_f(p_options->turn_advance_m))) ||
                  (!finite_f(p_options->advance_mps))) ||
                 (p_options->turn_advance_m < 0.0f)) ||
                (p_options->turn_advance_m > 0.30f)) ||
               ((p_options->turn_advance_m > 0.0f) &&
                ((0 == p_options->p_drive) ||
                 (p_options->advance_mps <= 0.0f)))) ||
              (0U == p_options->pending_wait_us)) ||
             (p_options->pending_wait_us >= 0x80000000U)) ||
            (p_options->turn_line_window_us >= 0x80000000U)) ||
           (!finite_f(p_options->search_deg))) ||
          (p_options->search_deg < 0.0f)) ||
         (p_options->search_deg > 60.0f)) ||
        ((p_options->search_deg > 0.0f) &&
         (((((((0U == p_options->turn_line_window_us) ||
               (!finite_f(p_options->search_mps))) ||
              (p_options->search_mps <= 0.0f)) ||
             (p_options->search_mps > 0.30f)) ||
            (!finite_f(p_options->track_m))) ||
           (p_options->track_m < 0.05f)) ||
          (p_options->track_m > 0.50f))))
    {
        return false;
    }

    p_drive->options = *p_options;
    return true;
}

/** @brief Start following only from OFF or HALTED with ready navigation and
 * fresh, idle, braked, fault-free motor feedback. With avoidance
 * enabled, require a ready range snapshot too. Success enables the
 * heartbeat.
 */
bool s3_drive_start (s3_drive_t * p_drive, uint32_t now)
{
#if APP_S5
    s5_codec_observation_t obstacle;

    if ((!s5_api_snapshot(&obstacle)) ||
        (!avoid_obstacle_range_ready(&obstacle, now)))
    {
        return false;
    }
#endif
    if ((((((((0 == p_drive) || (!p_drive->b_is_configured)) ||
             ((p_drive->state != (unsigned int)S3_DRIVE_OFF) &&
              (p_drive->state != (unsigned int)S3_DRIVE_HALTED))) ||
            (!feedback_fresh(p_drive, now))) ||
           (p_drive->feedback.state != (unsigned int)MOTION_IDLE)) ||
          (p_drive->feedback.fault != (unsigned int)MOTION_FAULT_NONE)) ||
         (!p_drive->feedback.b_is_braked)) ||
        (!line_navigation_start(&p_drive->navigation, now)))
    {
        return false;
    }

    (void)line_navigation_set_mode(&p_drive->navigation, S3_MODE_TRACK);
#if APP_S5
    /* clang-format off */
    p_drive->b_has_obstacle_snapshot =
        s5_codec_observation_pack(&obstacle, p_drive->obstacle_snapshot);
    p_drive->obstacle_snapshot_us    = now;
    /* clang-format on */
    avoid_obstacle_abort(&p_drive->obstacle);
    p_drive->obstacle.fault = S5_OK;
    s5_api_set_control_status(S5_IDLE, S5_OK, S5_ROUTE_NONE, 0U, 0U, 0U);
#endif
    p_drive->state   = S3_DRIVE_FOLLOW;
    p_drive->reason  = S3_DRIVE_OK;
    p_drive->pending = S3_NAV_NONE;
    p_drive->port.p_heartbeat(true);
    return true;
}

/** @brief Stop a configured drive immediately. Preserve a latched sensor or
 * drive fault; otherwise transition to OFF.
 */
void s3_drive_stop (s3_drive_t * p_drive)
{
    if ((0 == p_drive) || (!p_drive->b_is_configured))
    {
        return;
    }

    if (((unsigned int)S3_DRIVE_FAULT == p_drive->state) ||
        (S3_RUN_FAULT_LATCHED == (int)p_drive->navigation.lifecycle))
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, p_drive->reason);
    }
    else
    {
        halt_drive(p_drive, S3_DRIVE_OFF, S3_DRIVE_OK);
    }
}

/** @brief Request reset from FAULT. Stop and disable the heartbeat before
 * asking the motor owner to reset. A true result means the request is
 * pending; the next feedback sample determines completion.
 */
bool s3_drive_reset (s3_drive_t * p_drive, uint32_t now)
{
    if ((((0 == p_drive) || (!p_drive->b_is_configured)) ||
         (p_drive->state != (unsigned int)S3_DRIVE_FAULT)) ||
        (S3_RESET_PENDING == (int)p_drive->reset_result))
    {
        return false;
    }

    p_drive->port.p_stop();
    p_drive->port.p_heartbeat(false);
    p_drive->reset_result = S3_RESET_FAILED;

    if (!p_drive->port.p_reset())
    {
        return false;
    }
    /* Buddy 2 resets its controller synchronously but publishes feedback on
     * its next control tick. The cached snapshot is therefore pre-reset;
     * checking it here incorrectly required a second button press. */
    p_drive->reset_us     = now;
    p_drive->reset_result = S3_RESET_PENDING;
    return true;
}

/** @brief Read the last reset outcome without changing it. A null drive
 * returns FAILED.
 */
s3_drive_reset_result_t s3_drive_reset_result (const s3_drive_t * p_drive)
{
    return p_drive ? (s3_drive_reset_result_t)p_drive->reset_result
                   : (unsigned)S3_RESET_FAILED;
}

/** @brief Process one timestamped IR frame and optional motion feedback. Drain
 * reports once before navigation and arbitration. Return false for an
 * unconfigured drive, a missing frame or failed navigation processing.
 * A true result can include a recorded stop or fault.
 * @details Refresh odometry before navigation. An external hold permits
 * sampling but prevents driving. Avoidance may consume the cycle before
 * normal following or manoeuvre handling. Motor command completion clears
 * the command ID before requesting the next leg. Tape observations during a
 * turn drive the post-turn reacquisition check.
 */
bool s3_drive_process (s3_drive_t *                   p_drive,
                       const ir_acquisition_frame_t * p_frame,
                       const motion_feedback_t *      p_feedback)
{
    line_navigation_odometry_t odometry = {0};

    if ((0 == p_drive) || (!p_drive->b_is_configured))
    {
        return false;
    }

    if (0 == p_frame)
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
        return false;
    }

    const uint32_t now = p_frame->timestamp_us;
    p_drive->feedback  = p_feedback ? *p_feedback : (motion_feedback_t){0};
    bool b_is_running  = (p_drive->state >= (unsigned int)S3_DRIVE_FOLLOW) &&
                        (p_drive->state <= (unsigned int)S3_DRIVE_REACQUIRE);
    bool b_is_motion_done;
    bool b_are_reports_valid =
        receive_reports(p_drive, b_is_running, &b_is_motion_done);

    if (b_is_running &&
        (p_drive->feedback.report_loss_count != p_drive->report_loss_count))
    {
        b_are_reports_valid = false;
    }

    p_drive->report_loss_count = p_drive->feedback.report_loss_count;

    odometry = (line_navigation_odometry_t){
        p_drive->feedback.centre_m, p_drive->feedback.timestamp_us,
        p_drive->feedback.epoch, p_drive->feedback.b_is_feedback_valid};

    if (!line_navigation_process(&p_drive->navigation, p_frame, &odometry,
                                 &p_drive->output))
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
        return false;
    }

    if (S3_RESET_PENDING == (int)p_drive->reset_result)
    {
        finish_reset(p_drive, now);
        /* Still disarmed whether the reset completed, is waiting, or timed
         * out. Never let a fault reset fall through to a driving action. */
        return true;
    }

    b_is_running = (p_drive->state >= (unsigned int)S3_DRIVE_FOLLOW) &&
                   (p_drive->state <= (unsigned int)S3_DRIVE_REACQUIRE);

    if (p_drive->output.b_is_stop_requested)
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_SENSOR_FAULT);
        return true;
    }

    if (b_is_running &&
        (((!feedback_fresh(p_drive, now)) ||
          ((unsigned int)MOTION_FAULT == p_drive->feedback.state)) ||
         (p_drive->feedback.fault != (unsigned int)MOTION_FAULT_NONE)))
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_MOTION_FAULT);
        return true;
    }

    if (!b_is_running)
    {
        return true;
    }

    if ((!b_are_reports_valid) ||
        ((p_drive->command_id != 0U) &&
         (p_drive->feedback.command_id != p_drive->command_id)))
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_REPORT_FAULT);
        return true;
    }

    if (p_drive->b_has_external_hold)
    {
        request_stop(p_drive, now);
        p_drive->port.p_heartbeat(false);
        return true;
    }

    p_drive->port.p_heartbeat(true);
#if APP_S5
    if (obstacle_control(p_drive, now, b_is_motion_done))
    {
        return true;
    }
#endif
    if ((unsigned int)S3_DRIVE_FOLLOW == p_drive->state)
    {
        follow_line(p_drive, now);
    }
    else if ((unsigned int)S3_DRIVE_BRAKING == p_drive->state)
    {
        if ((now - p_drive->phase_us) > S3_BRAKE_WAIT_US)
        {
            halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
        }
        else if (stopped_after_request(p_drive))
        {
            finish_braking_request(p_drive, now);
        }
        else
        {
            /* Keep braking until feedback confirms the requested stop. */
        }
    }
    else if (((unsigned int)S3_DRIVE_MANOEUVRE == p_drive->state) &&
             (p_drive->b_is_searching))
    {
        if ((now - p_drive->phase_us) > S3_MANOEUVRE_WAIT_US)
        {
            halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
        }
        else
        {
            search_step(p_drive, now);
        }
    }
    else if ((unsigned int)S3_DRIVE_MANOEUVRE == p_drive->state)
    {
        if (((!p_drive->b_is_advancing) && (p_drive->output.line.b_is_valid)) &&
            (p_drive->output.line.b_is_touching_tape))
        {
            p_drive->touch_us           = now;
            p_drive->b_has_touched_line = true;
        }

        if ((now - p_drive->phase_us) > S3_MANOEUVRE_WAIT_US)
        {
            halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
        }
        else if (b_is_motion_done && (p_drive->b_is_advancing))
        {
            /* Axle now over the crossing: turn on the spot from here. */
            p_drive->command_id     = 0U;
            p_drive->b_is_advancing = false;

            if (!request_turn(p_drive))
            {
                halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
            }
            else
            {
                p_drive->phase_us = now;
            }
        }
        else if (b_is_motion_done)
        {
            const bool b_was_turn =
                (((unsigned int)S3_NAV_LEFT == p_drive->turn) ||
                 ((unsigned int)S3_NAV_RIGHT == p_drive->turn)) ||
                ((unsigned int)S3_NAV_UTURN == p_drive->turn);
            p_drive->command_id = 0U;
            p_drive->port.p_stop();

            resume_after_manoeuvre(p_drive, now, b_was_turn);
        }
        else if (((p_drive->feedback.state != (unsigned int)MOTION_TURN) &&
                  (p_drive->feedback.state != (unsigned int)MOTION_DRIVE)) &&
                 (p_drive->feedback.state != (unsigned int)MOTION_COMPLETE))
        {
            halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
        }
        else
        {
            /* Wait for the completion report for the active manoeuvre. */
        }
    }
    else
    {
        p_drive->port.p_stop();

        if ((now - p_drive->phase_us) > S3_REACQUIRE_WAIT_US)
        {
            halt_drive(p_drive, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
        }
        else if ((p_drive->output.steering.b_is_valid) &&
                 (stopped_after_request(p_drive)))
        {
            p_drive->state = S3_DRIVE_FOLLOW;
        }
        else
        {
            /* Keep reacquiring until both steering and stopped feedback are
             * ready. */
        }
    }

    return true;
}

#if APP_S5
/** @brief Publish one avoidance status snapshot. Only the controller task
 * writes the telemetry leg, route and fault fields.
 */
static void report_obstacle_status (const s3_drive_t * p_drive)
{
    unsigned leg = p_drive->obstacle.leg;

    if (((S5_MOVING == (int)p_drive->obstacle.state) &&
         (0U == p_drive->obstacle.leg)) &&
        (p_drive->obstacle.b_is_backing))
    {
        leg = S5_TELEMETRY_LEG_REVERSE;
    }

    if ((S5_SCANNING == (int)p_drive->obstacle.state) &&
        (p_drive->obstacle.b_is_aiming))
    {
        leg = S5_TELEMETRY_LEG_AIM;
    }

    s5_api_set_control_status(p_drive->obstacle.state, p_drive->obstacle.fault,
                              p_drive->obstacle.plan.route, leg,
                              p_drive->obstacle.pass,
                              p_drive->obstacle.b_has_swapped);
}
#endif

/** @brief Test the input with the original equality and subtraction checks;
 * NaN and either infinity fail.
 */
static bool finite_f (float32_t operand)
{
    return (operand == operand) && ((operand - operand) == 0.0f);
}

/** @brief Require valid cached feedback no older than the navigation
 * sample-age limit. Both timestamps are wrapping microseconds.
 */
static bool feedback_fresh (const s3_drive_t * p_drive, uint32_t now)
{
    return (p_drive->feedback.b_is_feedback_valid) &&
           ((now - p_drive->feedback.timestamp_us) <=
            p_drive->navigation.config.max_sample_age_us);
}

/** @brief Issue the motor stop before scanner cleanup, disarm the heartbeat
 * and store the terminal state and reason. Clear pending actions while
 * preserving the current barcode and sensor-fault event bits.
 */
static void halt_drive (s3_drive_t * p_drive, s3_drive_state_t state,
                        s3_drive_reason_t reason)
{
    if (S3_RESET_PENDING == (int)p_drive->reset_result)
    {
        p_drive->reset_result = S3_RESET_FAILED;
    }
    /* Brake before obstacle cleanup, which can wait for the scanner mutex. */
    p_drive->b_has_external_hold = false;
    p_drive->port.p_stop();
    p_drive->port.p_heartbeat(false);
#if APP_S5
    s5_api_cancel_scan();
    p_drive->b_has_obstacle_snapshot = false;

    if ((((unsigned int)S3_DRIVE_FAULT == state) &&
         (p_drive->b_is_bypassing)) &&
        ((int)p_drive->obstacle.state != S5_FAULT))
    {
        p_drive->obstacle.state = S5_FAULT;
        p_drive->obstacle.fault = S5_INTERRUPTED;
    }

    p_drive->b_is_bypassing = false;

    if (state != (unsigned int)S3_DRIVE_FAULT)
    {
        avoid_obstacle_abort(&p_drive->obstacle);
    }

    report_obstacle_status(p_drive);
#endif
    (void)line_navigation_end_run(
        &p_drive->navigation); /* A sensor latch is retained. */
    p_drive->pending                    = S3_NAV_NONE;
    p_drive->turn                       = S3_NAV_NONE;
    p_drive->b_is_advancing             = false;
    p_drive->b_has_touched_line         = false;
    p_drive->b_is_searching             = false;
    p_drive->cancel_id                  = 0U;
    p_drive->command_id                 = 0U;
    p_drive->b_waits_for_stop           = false;
    p_drive->output.steering.b_is_valid = false;
    p_drive->output.action              = S3_NAV_NONE;
    /* Keep this cycle's barcode event too: a barcode that causes the halt
     * (e.g. an unmapped letter) must still reach telemetry. */
    /* clang-format off */
    p_drive->output.events         &=
        ((unsigned)S3_EVENT_SENSOR_FAULT | (unsigned)S3_EVENT_BARCODE);
    p_drive->output.lifecycle       = p_drive->navigation.lifecycle;
    p_drive->output.b_is_diagnostic = true;
    p_drive->state                  = state;
    p_drive->reason                 = reason;
    /* clang-format on */
}

/** @brief Check fresh feedback strictly newer than the reset request. Return
 * to OFF only after braked, idle, fault-free feedback and ready
 * navigation. Mark the pending request failed at its deadline.
 */
static void finish_reset (s3_drive_t * p_drive, uint32_t now)
{
    if ((now - p_drive->reset_us) >= S3_RESET_WAIT_US)
    {
        p_drive->reset_result = S3_RESET_FAILED;
        return;
    }

    if (((((!feedback_fresh(p_drive, now)) ||
           (!feedback_is_later(p_drive->feedback.timestamp_us,
                               p_drive->reset_us))) ||
          (p_drive->feedback.state != (unsigned int)MOTION_IDLE)) ||
         (p_drive->feedback.fault != (unsigned int)MOTION_FAULT_NONE)) ||
        (!p_drive->feedback.b_is_braked))
    {
        return;
    }

    if ((S3_RUN_FAULT_LATCHED == (int)p_drive->navigation.lifecycle) &&
        (!line_navigation_reset_fault(&p_drive->navigation, now)))
    {
        return;
    }

    if (line_navigation_readiness(&p_drive->navigation, now) != 0U)
    {
        return;
    }

    p_drive->state                      = S3_DRIVE_OFF;
    p_drive->reason                     = S3_DRIVE_OK;
    p_drive->output.lifecycle           = p_drive->navigation.lifecycle;
    p_drive->output.b_is_stop_requested = false;
    p_drive->reset_result               = S3_RESET_OK;
}

/** @brief Submit the stored navigation action as a motor manoeuvre. Return
 * true only for an accepted request with a nonzero command ID.
 */
static bool request_turn (s3_drive_t * p_drive)
{
    motion_manoeuvre_t manoeuvre;

    switch (p_drive->turn)
    {
    case S3_NAV_LEFT:
        manoeuvre = MANOEUVRE_LEFT;
    break;
    case S3_NAV_RIGHT:
        manoeuvre = MANOEUVRE_RIGHT;
    break;
    case S3_NAV_STRAIGHT:
        manoeuvre = MANOEUVRE_STRAIGHT;
    break;
    case S3_NAV_UTURN:
        manoeuvre = MANOEUVRE_U_TURN;
    break;
    default:
        return false;
    }

    if (p_drive->port.p_manoeuvre(manoeuvre) != (unsigned int)MOTION_OK)
    {
        return false;
    }

    p_drive->command_id = p_drive->port.p_command_id();
    return p_drive->command_id != 0U;
}

/** @brief Submit the pending motion after the stop is confirmed.
 * @param p_drive State held by the sampling owner after confirmed braking.
 * @param now Current microsecond time on the modulo-2^32 clock.
 * @post Enter the manoeuvre only on acceptance; otherwise halt with rejection.
 */
static inline void finish_braking_request (s3_drive_t * p_drive, uint32_t now)
{
    p_drive->b_waits_for_stop = false;
    (void)line_navigation_set_mode(&p_drive->navigation, S3_MODE_SUPPRESSED);
    p_drive->turn = p_drive->pending;

    if (!start_manoeuvre(p_drive))
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
    }
    else
    {
        p_drive->pending  = S3_NAV_NONE;
        p_drive->state    = S3_DRIVE_MANOEUVRE;
        p_drive->phase_us = now;
    }
}

/** @brief Request the configured pre-turn advance or issue the turn
 * immediately. Return false if the motion request fails or has no
 * usable command ID.
 *
 * LEFT/RIGHT pivot on the axle, which is behind the sensors. Optionally drive
 * on first so the axle reaches the crossing; the braking distance already
 * travelled since detection is subtracted (same odometry epoch only).
 */
static bool start_manoeuvre (s3_drive_t * p_drive)
{
    const float64_t travelled =
        p_drive->feedback.centre_m - p_drive->junction_m;
    const float32_t advance =
        p_drive->options.turn_advance_m - (float32_t)travelled;
    p_drive->b_has_touched_line = false;

    if (((((unsigned int)S3_NAV_LEFT == p_drive->turn) ||
          ((unsigned int)S3_NAV_RIGHT == p_drive->turn)) &&
         (p_drive->options.turn_advance_m > 0.0f)) &&
        (advance > S3_MIN_ADVANCE_M))
    {
        if (p_drive->options.p_drive(advance, p_drive->options.advance_mps) !=
            (unsigned int)MOTION_OK)
        {
            return false;
        }

        p_drive->b_is_advancing = true;
        p_drive->command_id     = p_drive->port.p_command_id();
        return p_drive->command_id != 0U;
    }

    p_drive->b_is_advancing = false;
    return request_turn(p_drive);
}

/** @brief Return whether the current junction candidate may be crossed at base
 * speed under the selected route policy.
 *
 * Unannounced crossing (or a STRAIGHT action) in cross_straight mode: steering
 * is undefined while both sensors are dark, so hold both wheels at base speed
 * until the crossing has passed under the sensors.
 */
static bool straight_across (const s3_drive_t * p_drive)
{
    return (((p_drive->options.b_allows_cross_straight) &&
             (p_drive->output.line.b_is_valid)) &&
            (p_drive->output.line.b_has_junction_candidate)) &&
           (((unsigned int)S3_NAV_NONE == p_drive->pending) ||
            ((unsigned int)S3_NAV_STRAIGHT == p_drive->pending));
}

/** @brief Record the microsecond stop time and outstanding cancellation ID
 * once. Repeated calls leave the original request intact.
 *
 * A stop request is not a stopped acknowledgement. Wait for the motion
 * owner to apply its brake and observe stationary wheels before resuming.
 */
static void request_stop (s3_drive_t * p_drive, uint32_t now)
{
    if (!p_drive->b_waits_for_stop)
    {
        p_drive->cancel_id        = p_drive->command_id;
        p_drive->command_id       = 0U;
        p_drive->b_waits_for_stop = true;
        p_drive->phase_us         = now;
        p_drive->port.p_stop();
    }
}

/** @brief Compare timestamps strictly within the forward half range.
 * Both arguments are microseconds from the same wrapping 32-bit clock.
 * The exactly half-range case, like equality, is not a fresh sample.
 */
static inline bool feedback_is_later (uint32_t received, uint32_t reference)
{
    const uint32_t elapsed = received - reference;
    return (elapsed != 0U) && (elapsed <= (UINT32_MAX / 2U));
}

/** @brief Reacquire the line after a stopped manoeuvre or halt.
 * Continue in TRACK when tape was seen recently or the check is disabled.
 * Otherwise search within the configured bounds or report a lost line.
 */
static inline void resume_after_manoeuvre (s3_drive_t * p_drive, uint32_t now,
                                           bool b_was_turn)
{
    const bool b_needs_tape_search =
        (b_was_turn && (p_drive->options.turn_line_window_us != 0U)) &&
        (!((p_drive->b_has_touched_line) &&
           ((now - p_drive->touch_us) <=
            p_drive->options.turn_line_window_us)));

    if (!b_needs_tape_search)
    {
        (void)line_navigation_set_mode(&p_drive->navigation, S3_MODE_TRACK);
        p_drive->state    = S3_DRIVE_REACQUIRE;
        p_drive->phase_us = now;
    }
    else if (p_drive->options.search_deg > 0.0f)
    {
        /* Sweep where the turn should have intersected the tape. */
        start_search(p_drive, now);
    }
    else
    {
        halt_drive(p_drive, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
    }
}

/** @brief Require fresh post-request idle and braked feedback plus a cleared
 * cancellation ID before authorizing further motion.
 */
static bool stopped_after_request (const s3_drive_t * p_drive)
{
    return (((p_drive->feedback.b_is_braked) &&
             ((unsigned int)MOTION_IDLE == p_drive->feedback.state)) &&
            (0U == p_drive->cancel_id)) &&
           feedback_is_later(p_drive->feedback.timestamp_us, p_drive->phase_us);
}

/** @brief Drain at most one queue capacity and write the matching completion
 * flag to pb_is_complete. Return false for report loss, zero IDs or unexpected
 * outcomes for the command being tracked.
 *
 * Bounded drain, also while disarmed. Never use motion_poll(): it does not
 * consume reports. Only a matching DONE may finish our current manoeuvre.
 */
static bool receive_reports (s3_drive_t * p_drive, bool b_is_running,
                             bool * pb_is_complete)
{
    motion_report_t report;
    bool            b_is_healthy = true;
    *pb_is_complete              = false;

    for (unsigned int report_index = 0U; report_index < MOTION_REPORT_QUEUE_LEN;
         report_index++)
    {
        if (!p_drive->port.p_report(&report))
        {
            break;
        }

        if (!b_is_running)
        {
            continue;
        }

        if (report.b_was_lost)
        {
            b_is_healthy = false;
        }

        if (0U == report.command_id)
        {
            b_is_healthy = false;
            continue;
        }

        if (report.command_id == p_drive->cancel_id)
        {
            if (report.outcome != (unsigned int)MOTION_OUTCOME_CANCELLED)
            {
                b_is_healthy = false;
            }

            p_drive->cancel_id = 0U;
        }
        else if (report.command_id == p_drive->command_id)
        {
            if (((((unsigned int)S3_DRIVE_MANOEUVRE == p_drive->state) &&
                  ((unsigned int)MOTION_OUTCOME_DONE == report.outcome)) &&
                 (!*pb_is_complete)) &&
                ((unsigned int)MOTION_FAULT_NONE == report.fault))
            {
                *pb_is_complete = true;
            }
            else
            {
                b_is_healthy = false;
            }
        }
        else
        {
            /* Old reports cannot complete a newer command. Draining before
             * issuing a command prevents aliasing when IDs wrap. */
        }
    }

    return b_is_healthy;
}

/** @brief Calculate a search leg in microseconds from the configured angle,
 * track width and wheel speed. The validated search configuration has
 * nonzero speed.
 *
 * @pre The configured leg duration fits uint32_t microseconds after
 * truncation and before adding one. See BARR-P52 for the setter range.
 */
static uint32_t search_leg_us (const s3_drive_t * p_drive)
{
    const float32_t rad = p_drive->options.search_deg * 0.0174533f;
    const float32_t seconds =
        rad * 0.5f * p_drive->options.track_m / p_drive->options.search_mps;
    return (uint32_t)(seconds * 1000000.0f) + 1U;
}

/** @brief Initialize the two-leg line search using the turn direction and the
 * time of the most recent line contact.
 *
 * The turn ended with no front sensor on the new line. If the last touch was
 * in the second half of the spin, the line was passed (overshoot): sweep back
 * first; otherwise (undershoot, or only the old line touched early) keep
 * turning the same way first. Both sides are swept either way.
 */
static void start_search (s3_drive_t * p_drive, uint32_t now)
{
    const int8_t turn_sign =
        ((unsigned int)S3_NAV_RIGHT == p_drive->turn) ? -1 : 1;
    const uint32_t spin_us = now - p_drive->phase_us;
    const bool     b_has_overshot =
        ((p_drive->b_has_touched_line) && (spin_us > 0U)) &&
        ((now - p_drive->touch_us) < (spin_us / 2U));
    p_drive->search_sign    = b_has_overshot ? (int8_t)-turn_sign : turn_sign;
    p_drive->search_leg     = 0U;
    p_drive->search_us      = now;
    p_drive->phase_us       = now;
    p_drive->b_is_searching = true;
}

/** @brief Stop searching on line contact or after the second sweep. While
 * searching, request opposite wheel velocities and fault on rejection.
 */
static void search_step (s3_drive_t * p_drive, uint32_t now)
{

    if ((p_drive->output.line.b_is_valid) &&
        (p_drive->output.line.b_is_touching_tape))
    {
        p_drive->b_is_searching = false;
        p_drive->port.p_stop();
        (void)line_navigation_set_mode(&p_drive->navigation, S3_MODE_TRACK);
        p_drive->state    = S3_DRIVE_REACQUIRE;
        p_drive->phase_us = now;
        return;
    }

    if ((now - p_drive->search_us) >=
        (search_leg_us(p_drive) * (0U == p_drive->search_leg ? 1U : 2U)))
    {
        if (p_drive->search_leg != 0U)
        {
            halt_drive(p_drive, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
            return;
        }

        p_drive->search_leg  = 1U;
        p_drive->search_sign = (int8_t)-p_drive->search_sign;
        p_drive->search_us   = now;
    }

    const float32_t wheel_mps =
        p_drive->options.search_mps *
        (float32_t)p_drive->search_sign; /* Positive means anticlockwise. */
    if (p_drive->port.p_velocity(-wheel_mps, wheel_mps) !=
        (unsigned int)MOTION_OK)
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
    }
}
#if APP_S5
/** @brief Update filtered turn intensity from requested wheel speeds in m/s
 * and the wrapping microsecond timestamp. Refresh the timestamp even
 * when speed or elapsed time makes this sample unusable.
 *
 * How hard the line follower is turning (+ = left), time-averaged, tau 0.5 s.
 */
static void turn_track (s3_drive_t * p_drive, float32_t left, float32_t right,
                        uint32_t now)
{
    const float32_t sum = left + right;
    const uint32_t  elapsed_us =
        p_drive->turn_us ? (now - p_drive->turn_us) : 0U;
    p_drive->turn_us = now;

    if (((sum < 0.01f) || (0U == elapsed_us)) || (elapsed_us > 200000U))
    {
        return;
    }

    {
        /* Fixed point, 1/10000: s += (x - s) * dt / (tau + dt), tau 0.5 s. */
        float32_t turn_ratio = (right - left) / sum;
        int32_t   smoothed   = p_drive->turn_q;
        turn_ratio           = turn_ratio > 1.0f
                                   ? 1.0f
                                   : (turn_ratio < -1.0f ? -1.0f : turn_ratio);
        /* Clamping the finite steering ratio bounds the conversion to
         * +/-10000 in the fixed-point scale. */
        const int32_t input_q = (int32_t)(turn_ratio * 10000.0f);
        /* clang-format off */
        smoothed       +=
            (int32_t)(((int64_t)(input_q - smoothed) * (int64_t)elapsed_us) /
                      (int64_t)(500000U + elapsed_us));
        /* Clamp to the retained signed 1/10000 scale before narrowing. */
        p_drive->turn_q =
            (int16_t)(smoothed > 10000 ? 10000
                                : (smoothed < -10000 ? -10000 : smoothed));
        /* clang-format on */
    }
}

/** @brief Convert the retained turn estimate from hundredths of a percent to a
 * whole signed percent by truncating toward zero.
 *
 * Latched shared value: whole percent toward zero, -100..100 (so 19.99 % is
 * still below the 20 % bend threshold, as with the old float compare).
 */
static int8_t turn_pct (int16_t turn_q)
{
    return (int8_t)(turn_q / 100);
}
#endif

/** @brief Apply the current navigation output. Latch a mapped barcode until
 * its junction, brake for manoeuvres, and stop for an unmapped code or
 * expired action. Store the ID of each accepted velocity command.
 * @details Line loss has priority. A second pending barcode is a direction
 * fault. A configured straight crossing can consume a straight action or
 * cross without one. Other manoeuvres brake and record their junction pose.
 * A pending stop prevents new speeds until fresh feedback acknowledges it
 * and either valid steering or the straight-crossing hold can supply speeds.
 */
static void follow_line (s3_drive_t * p_drive, uint32_t now)
{
    const line_navigation_output_t * p_out = &p_drive->output;

    if (p_out->events & (unsigned)S3_EVENT_LINE_LOST)
    {
        halt_drive(p_drive, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
        return;
    }

    if (p_out->events & (unsigned)S3_EVENT_BARCODE)
    {
        if ((unsigned int)S3_NAV_NONE == p_out->action)
        {
            halt_drive(p_drive, S3_DRIVE_HALTED, S3_DRIVE_UNMAPPED);
            return;
        }

        if (p_drive->pending != (unsigned int)S3_NAV_NONE)
        {
            halt_drive(p_drive, S3_DRIVE_HALTED, S3_DRIVE_NO_DIRECTION);
            return;
        }

        p_drive->pending    = p_out->action;
        p_drive->pending_us = now;
    }

    if ((p_drive->pending != (unsigned int)S3_NAV_NONE) &&
        ((now - p_drive->pending_us) > p_drive->options.pending_wait_us))
    {
        halt_drive(p_drive, S3_DRIVE_HALTED, S3_DRIVE_TIMEOUT);
        return;
    }

    if (p_out->events & (unsigned)S3_EVENT_JUNCTION)
    {
        if ((p_drive->options.b_allows_cross_straight) &&
            (((unsigned int)S3_NAV_NONE == p_drive->pending) ||
             ((unsigned int)S3_NAV_STRAIGHT == p_drive->pending)))
        {
            p_drive->pending =
                S3_NAV_NONE; /* Continue following straight across. */
        }
        else if ((unsigned int)S3_NAV_NONE == p_drive->pending)
        {
            halt_drive(p_drive, S3_DRIVE_HALTED, S3_DRIVE_NO_DIRECTION);
            return;
        }
        else
        {
            p_drive->junction_m = p_drive->feedback.centre_m;
            request_stop(p_drive, now);
            p_drive->state = S3_DRIVE_BRAKING;
            return;
        }
    }

    if (p_drive->b_waits_for_stop)
    {
        if ((now - p_drive->phase_us) > S3_BRAKE_WAIT_US)
        {
            halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
            return;
        }

        if (!stopped_after_request(p_drive))
        {
            return;
        }

        if ((!p_out->steering.b_is_valid) && (!straight_across(p_drive)))
        {
            return;
        }

        p_drive->b_waits_for_stop = false;
    }

    float32_t left;
    float32_t right;
    if (p_out->steering.b_is_valid)
    {
        left  = p_out->steering.left_mps;
        right = p_out->steering.right_mps;
    }
    else if (straight_across(p_drive))
    {
        left = right = p_drive->navigation.config.steering.base_mps;
    }
    else
    {
        request_stop(p_drive, now);
        return;
    }
    submit_follow_speed(p_drive, left, right, now);
}

/** @brief Submit the selected wheel speeds and record the command ID.
 * Track the bend before sending the command when avoidance is enabled.
 */
static inline void submit_follow_speed (s3_drive_t * p_drive, float32_t left,
                                        float32_t right, uint32_t now)
{
#if !APP_S5
    (void)now;
#endif
#if APP_S5
    turn_track(p_drive, left, right, now);
#endif
    if (p_drive->port.p_velocity(left, right) != (unsigned int)MOTION_OK)
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
    }
    else
    {
        p_drive->command_id = p_drive->port.p_command_id();

        if (0U == p_drive->command_id)
        {
            halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_REPORT_FAULT);
        }
    }
}

#if APP_S5
/** @brief Read a current obstacle observation. If the scanner is busy, use the
 * last packed observation only within its freshness interval. Clear
 * the cache-valid flag on an unusable read.
 *
 * Called only by the existing sample/controller task after its one report
 * drain. S5's pure FSM proposes actions; this owner alone authorises Buddy 2
 * requests.
 */
static bool obstacle_snapshot (s3_drive_t * p_drive, uint32_t now,
                               s5_codec_observation_t * p_out)
{
    const s5_api_snapshot_result_t result = s5_api_read_snapshot(p_out);

    if ((unsigned int)S5_SNAPSHOT_OK == result)
    {
        /* clang-format off */
        p_drive->b_has_obstacle_snapshot =
            s5_codec_observation_pack(p_out, p_drive->obstacle_snapshot);
        p_drive->obstacle_snapshot_us    = now;
        /* clang-format on */
        return p_drive->b_has_obstacle_snapshot;
    }

    if ((((unsigned int)S5_SNAPSHOT_BUSY == result) &&
         (p_drive->b_has_obstacle_snapshot)) &&
        ((now - p_drive->obstacle_snapshot_us) <= S5_RANGE_FRESH_US))
    {
        return s5_codec_observation_unpack(p_drive->obstacle_snapshot, p_out);
    }

    p_drive->b_has_obstacle_snapshot = false;
    return false;
}

/** @brief Arbitrate avoidance for one controller update. Return true when this
 * path handled the cycle, including faults. On resume, credit detour
 * time to a pending barcode action.
 * @details A fresh forward sonar reading can trigger a detour in FOLLOW.
 * An emergency during another manoeuvre halts the drive. Preserve any
 * eligible barcode decoded on the trigger cycle. During an active detour,
 * feed scan and motion confirmations to the avoidance state machine before
 * issuing the next action.
 */
static bool obstacle_control (s3_drive_t * p_drive, uint32_t now,
                              bool b_is_done)
{
    s5_codec_observation_t observation = {0};

    const bool b_has_observation =
        obstacle_snapshot(p_drive, now, &observation);

    if (!p_drive->b_is_bypassing)
    {
        if ((!b_has_observation) ||
            (!avoid_obstacle_range_ready(&observation, now)))
        {
            p_drive->obstacle.fault = S5_BAD_RANGE;
            p_drive->obstacle.state = S5_FAULT;
            halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
            return true;
        }

        if (p_drive->state != (unsigned int)S3_DRIVE_FOLLOW)
        {
            if (observation.range_mm <= p_drive->obstacle.config.emergency_mm)
            {
                p_drive->obstacle.fault = S5_NO_PATH;
                p_drive->obstacle.state = S5_FAULT;
                halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
                return true;
            }

            return false;
        }

        if (observation.range_mm > p_drive->obstacle.config.detect_mm)
        {
            return false;
        }

        if (p_drive->output.events & (unsigned)S3_EVENT_LINE_LOST)
        {
            halt_drive(p_drive, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
            return true;
        }
        /* Do not lose a barcode decoded on the same cycle as obstacle
         * detection. Its pending lifetime is paused for the detour, never
         * executed off-line. */
        const bool b_has_barcode =
            (p_drive->output.events & (unsigned)S3_EVENT_BARCODE) != 0U;

        if (b_has_barcode &&
            ((p_drive->pending != (unsigned int)S3_NAV_NONE) ||
             ((unsigned int)S3_NAV_NONE == p_drive->output.action)))
        {
            halt_drive(p_drive, S3_DRIVE_HALTED, S3_DRIVE_UNMAPPED);
            return true;
        }

        if (b_has_barcode)
        {
            p_drive->pending    = p_drive->output.action;
            p_drive->pending_us = now;
        }

        avoid_obstacle_begin(&p_drive->obstacle, now);
        /* clang-format off */
        p_drive->b_is_bypassing    = true;
        p_drive->obstacle_turn_pct =
            turn_pct(p_drive->turn_q); /* Use steering at detection. */
        /* clang-format on */
        request_stop(p_drive, now);
        p_drive->state = S3_DRIVE_MANOEUVRE;
        (void)line_navigation_set_mode(&p_drive->navigation,
                                       S3_MODE_SUPPRESSED);
    }

    if (b_is_done)
    {
        p_drive->command_id = 0U;
    }

    if (observation.scan_id != p_drive->obstacle_scan_id)
    {
        observation.b_is_scan_done = false;
    }

    const avoid_obstacle_input_t input = {
        /* Snapshot the current drive feedback and line state for this step. */
        .now_us           = now,
        .centre_m         = p_drive->feedback.centre_m,
        .b_is_stopped     = stopped_after_request(p_drive),
        .b_is_motion_done = b_is_done,
        .b_is_healthy = b_has_observation && (p_drive->output.line.b_is_valid),
        .b_has_tape   = p_drive->navigation.config.line.b_is_tape_between
                            ? p_drive->output.line.b_is_touching_tape
                            : p_drive->output.line.b_is_line_present,
        .b_has_junction  = p_drive->output.line.b_has_junction_candidate,
        .b_is_line_ready = p_drive->output.steering_preview.b_is_valid,
        .turn_pct        = p_drive->obstacle_turn_pct,
        .observation     = observation};
    avoid_obstacle_action_t request =
        avoid_obstacle_step(&p_drive->obstacle, &input);
    motion_result_t result = MOTION_OK;

    if (S5_FAULT == (int)p_drive->obstacle.state)
    {
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
        return true;
    }

    switch (request.kind)
    {
    case S5_ACT_STOP:
        p_drive->port.p_stop();
        p_drive->phase_us   = now;
        p_drive->command_id = 0U;
    break;
    case S5_ACT_SCAN:
        p_drive->b_waits_for_stop = false;

        if (0U == ++p_drive->obstacle_scan_id)
        {
            p_drive->obstacle_scan_id = 1U;
        }

        if (!s5_api_request_scan(p_drive->obstacle_scan_id))
        {
            result = MOTION_ERR_BUSY;
        }

    break;
    case S5_ACT_PEEK: /* Take one side reading while stopped on leg 3. */
        p_drive->b_waits_for_stop = false;

        if (0U == ++p_drive->obstacle_scan_id)
        {
            p_drive->obstacle_scan_id = 1U;
        }

        if (!s5_api_request_peek(p_drive->obstacle_scan_id,
                                 (unsigned)request.value))
        {
            result = MOTION_ERR_BUSY;
        }

    break;
    case S5_ACT_DRIVE:
        result = motion_request_drive((float32_t)request.value / 1000.0f,
                                      (float32_t)request.speed_mms / 1000.0f);
    break;
    case S5_ACT_TURN:
        result = motion_request_turn((float32_t)request.value,
                                     (float32_t)request.speed_mms / 1000.0f);
    break;
    case S5_ACT_RESUME:
        p_drive->b_is_bypassing   = false;
        p_drive->b_waits_for_stop = false;
        p_drive->cancel_id        = 0U;
        p_drive->command_id       = 0U;

        if (p_drive->pending != (unsigned int)S3_NAV_NONE)
        {
            p_drive->pending_us += now - p_drive->obstacle.started_us;
        }

        (void)line_navigation_set_mode(&p_drive->navigation, S3_MODE_TRACK);
        p_drive->state = S3_DRIVE_FOLLOW;
    break;
    default:
    break;
    }

    if (((unsigned int)MOTION_OK == result) &&
        (((unsigned int)S5_ACT_DRIVE == request.kind) ||
         ((unsigned int)S5_ACT_TURN == request.kind)))
    {
        p_drive->command_id = p_drive->port.p_command_id();
        p_drive->phase_us   = now;

        if (!p_drive->command_id)
        {
            result = MOTION_ERR_BUSY;
        }
    }

    if (result != (unsigned int)MOTION_OK)
    {
        p_drive->obstacle.state = S5_FAULT;
        p_drive->obstacle.fault = S5_MOTION_ERROR;
        halt_drive(p_drive, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
    }

    report_obstacle_status(p_drive);
    return true;
}
#endif

/* End of file. */

