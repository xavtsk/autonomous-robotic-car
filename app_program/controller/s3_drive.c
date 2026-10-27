/* Temporary controller for Buddy 2 + Buddy 3; replace with the team controller.
 * This is orchestration, not Buddy 3 sensing. No GPIO/RTOS in this file. */
#include "s3_drive.h"
#define S3_BRAKE_WAIT_US (500000U)
#define S3_MANOEUVRE_WAIT_US (10000000U)
#define S3_REACQUIRE_WAIT_US (1000000U)
#define S3_PENDING_WAIT_US (5000000U)
#define S3_MIN_ADVANCE_M (0.003f) /* shorter advances are skipped */
s3_drive_options_t s3_drive_default_options(void)
{
    return (s3_drive_options_t){false, 0.0f, 0.10f, 0, S3_PENDING_WAIT_US, 0U,
                                0.0f, 0.05f, 0.116f};
}
static bool finite_f(float x) { return (x == x) && ((x - x) == 0.0f); }
static bool feedback_fresh(const s3_drive_t *d, uint32_t now)
{
    return d->feedback.valid && ((uint32_t)(now - d->feedback.timestamp_us) <=
                                 d->navigation.config.max_sample_age_us);
}
static void halt(s3_drive_t *d, s3_drive_state_t state,
                 s3_drive_reason_t reason)
{
    if (d->reset_result == S3_RESET_PENDING)
    {
        d->reset_result = S3_RESET_FAILED;
    }
    d->port.stop();
    d->port.heartbeat(false);
    (void)s3_navigation_end_run(
        &d->navigation); /* A sensor latch is retained. */
    d->pending = S3_NAV_NONE;
    d->turn = S3_NAV_NONE;
    d->advancing = false;
    d->touched = false;
    d->searching = false;
    d->command_id = d->cancel_id = 0U;
    d->waiting_stop = false;
    d->output.steering.valid = false;
    d->output.action = S3_NAV_NONE;
    /* Keep this cycle's barcode event too: a barcode that causes the halt
     * (e.g. an unmapped letter) must still reach telemetry. */
    d->output.events &= (S3_EVENT_SENSOR_FAULT | S3_EVENT_BARCODE);
    d->output.lifecycle = d->navigation.lifecycle;
    d->output.diagnostic = true;
    d->state = state;
    d->reason = reason;
}
bool s3_drive_init(s3_drive_t *d, const s3_navigation_config_t *cfg,
                   const s3_drive_port_t *port)
{
    if (d == 0)
    {
        return false;
    }
    *d = (s3_drive_t){0};
    if ((port == 0) || !port->velocity || !port->manoeuvre || !port->stop ||
        !port->report || !port->command_id || !port->reset ||
        !port->heartbeat || !s3_navigation_init(&d->navigation, cfg))
    {
        return false;
    }
    d->port = *port;
    d->options = s3_drive_default_options();
    d->configured = true;
    return true;
}
bool s3_drive_set_options(s3_drive_t *d, const s3_drive_options_t *o)
{
    if ((d == 0) || !d->configured || (o == 0) ||
        ((d->state != S3_DRIVE_OFF) && (d->state != S3_DRIVE_HALTED) &&
         (d->state != S3_DRIVE_FAULT)) ||
        !finite_f(o->turn_advance_m) || !finite_f(o->advance_mps) ||
        (o->turn_advance_m < 0.0f) || (o->turn_advance_m > 0.30f) ||
        ((o->turn_advance_m > 0.0f) &&
         ((o->drive == 0) || (o->advance_mps <= 0.0f))) ||
        (o->pending_wait_us == 0U) || (o->pending_wait_us >= 0x80000000U) ||
        (o->turn_line_window_us >= 0x80000000U) ||
        !finite_f(o->search_deg) || (o->search_deg < 0.0f) ||
        (o->search_deg > 60.0f) ||
        ((o->search_deg > 0.0f) &&
         ((o->turn_line_window_us == 0U) || !finite_f(o->search_mps) ||
          (o->search_mps <= 0.0f) || (o->search_mps > 0.30f) ||
          !finite_f(o->track_m) || (o->track_m < 0.05f) ||
          (o->track_m > 0.50f))))
    {
        return false;
    }
    d->options = *o;
    return true;
}
bool s3_drive_start(s3_drive_t *d, uint32_t now)
{
    if ((d == 0) || !d->configured ||
        ((d->state != S3_DRIVE_OFF) && (d->state != S3_DRIVE_HALTED)) ||
        !feedback_fresh(d, now) || (d->feedback.state != MOTION_IDLE) ||
        (d->feedback.fault != MOTION_FAULT_NONE) || !d->feedback.braked ||
        !s3_navigation_start(&d->navigation, now))
    {
        return false;
    }
    (void)s3_navigation_set_mode(&d->navigation, S3_MODE_TRACK);
    d->state = S3_DRIVE_FOLLOW;
    d->reason = S3_DRIVE_OK;
    d->pending = S3_NAV_NONE;
    d->port.heartbeat(true);
    return true;
}
void s3_drive_stop(s3_drive_t *d)
{
    if ((d == 0) || !d->configured)
    {
        return;
    }
    if ((d->state == S3_DRIVE_FAULT) ||
        (d->navigation.lifecycle == S3_RUN_FAULT_LATCHED))
    {
        halt(d, S3_DRIVE_FAULT, d->reason);
    }
    else
    {
        halt(d, S3_DRIVE_OFF, S3_DRIVE_OK);
    }
}
bool s3_drive_reset(s3_drive_t *d, uint32_t now)
{
    if ((d == 0) || !d->configured || (d->state != S3_DRIVE_FAULT) ||
        (d->reset_result == S3_RESET_PENDING))
    {
        return false;
    }
    d->port.stop();
    d->port.heartbeat(false);
    d->reset_result = S3_RESET_FAILED;
    if (!d->port.reset())
    {
        return false;
    }
    /* Buddy 2 resets its controller synchronously but publishes feedback on
     * its next control tick. The cached snapshot is therefore pre-reset;
     * checking it here incorrectly required a second button press. */
    d->reset_us = now;
    d->reset_result = S3_RESET_PENDING;
    return true;
}
s3_reset_result_t s3_drive_reset_result(const s3_drive_t *d)
{
    return d ? (s3_reset_result_t)d->reset_result : S3_RESET_FAILED;
}
static void finish_reset(s3_drive_t *d, uint32_t now)
{
    if ((uint32_t)(now - d->reset_us) >= S3_RESET_WAIT_US)
    {
        d->reset_result = S3_RESET_FAILED;
        return;
    }
    if (!feedback_fresh(d, now) ||
        ((int32_t)(d->feedback.timestamp_us - d->reset_us) <= 0) ||
        (d->feedback.state != MOTION_IDLE) ||
        (d->feedback.fault != MOTION_FAULT_NONE) || !d->feedback.braked)
    {
        return;
    }
    if ((d->navigation.lifecycle == S3_RUN_FAULT_LATCHED) &&
        !s3_navigation_reset_fault(&d->navigation, now))
    {
        return;
    }
    if (s3_navigation_readiness(&d->navigation, now) != 0U)
    {
        return;
    }
    d->state = S3_DRIVE_OFF;
    d->reason = S3_DRIVE_OK;
    d->output.lifecycle = d->navigation.lifecycle;
    d->output.stop_requested = false;
    d->reset_result = S3_RESET_OK;
}
static bool request_turn(s3_drive_t *d)
{
    manoeuvre_t m;
    switch (d->turn)
    {
    case S3_NAV_LEFT:
        m = MANOEUVRE_LEFT;
        break;
    case S3_NAV_RIGHT:
        m = MANOEUVRE_RIGHT;
        break;
    case S3_NAV_STRAIGHT:
        m = MANOEUVRE_STRAIGHT;
        break;
    case S3_NAV_UTURN:
        m = MANOEUVRE_U_TURN;
        break;
    default:
        return false;
    }
    if (d->port.manoeuvre(m) != MOTION_OK)
    {
        return false;
    }
    d->command_id = d->port.command_id();
    return d->command_id != 0U;
}
/* LEFT/RIGHT pivot on the axle, which is behind the sensors. Optionally drive
 * on first so the axle reaches the crossing; the braking distance already
 * travelled since detection is subtracted (same odometry epoch only). */
static bool start_manoeuvre(s3_drive_t *d)
{
    const double travelled = d->feedback.centre_m - d->junction_m;
    const float advance = d->options.turn_advance_m - (float)travelled;
    d->touched = false;
    if (((d->turn == S3_NAV_LEFT) || (d->turn == S3_NAV_RIGHT)) &&
        (d->options.turn_advance_m > 0.0f) && (advance > S3_MIN_ADVANCE_M))
    {
        if (d->options.drive(advance, d->options.advance_mps) != MOTION_OK)
        {
            return false;
        }
        d->advancing = true;
        d->command_id = d->port.command_id();
        return d->command_id != 0U;
    }
    d->advancing = false;
    return request_turn(d);
}
/* Unannounced crossing (or a STRAIGHT action) in cross_straight mode: steering
 * is undefined while both sensors are dark, so hold both wheels at base speed
 * until the crossing has passed under the sensors. */
static bool straight_across(const s3_drive_t *d)
{
    return d->options.cross_straight && d->output.line.valid &&
           d->output.line.junction_candidate &&
           ((d->pending == S3_NAV_NONE) || (d->pending == S3_NAV_STRAIGHT));
}
/* A stop request is not a stopped acknowledgement. Wait for the motion
 * owner to apply its brake and observe stationary wheels before resuming. */
static void request_stop(s3_drive_t *d, uint32_t now)
{
    if (!d->waiting_stop)
    {
        d->cancel_id = d->command_id;
        d->command_id = 0U;
        d->waiting_stop = true;
        d->phase_us = now;
        d->port.stop();
    }
}
static bool stopped_after_request(const s3_drive_t *d)
{
    return d->feedback.braked && d->feedback.state == MOTION_IDLE &&
           d->cancel_id == 0U &&
           (int32_t)(d->feedback.timestamp_us - d->phase_us) > 0;
}
/* Bounded drain, also while disarmed. Never use motion_poll(): it does not
 * consume reports. Only a matching DONE may finish our current manoeuvre. */
static bool receive_reports(s3_drive_t *d, bool running, bool *done)
{
    motion_report_t report;
    bool healthy = true;
    *done = false;
    for (unsigned int i = 0U; i < MOTION_REPORT_QUEUE_LEN; i++)
    {
        if (!d->port.report(&report))
        {
            break;
        }
        if (!running)
        {
            continue;
        }
        if (report.b_lost)
        {
            healthy = false;
        }
        if (report.command_id == 0U)
        {
            healthy = false;
            continue;
        }
        if (report.command_id == d->cancel_id)
        {
            if (report.outcome != MOTION_OUTCOME_CANCELLED)
            {
                healthy = false;
            }
            d->cancel_id = 0U;
        }
        else if (report.command_id == d->command_id)
        {
            if (d->state == S3_DRIVE_MANOEUVRE &&
                report.outcome == MOTION_OUTCOME_DONE && !*done &&
                report.fault == MOTION_FAULT_NONE)
            {
                *done = true;
            }
            else
            {
                healthy = false;
            }
        }
        /* Old reports cannot complete a newer command. The single-owner
         * contract and draining before new commands prevent ID-wrap aliasing.
         */
    }
    return healthy;
}
/* Duration of one search_deg of spin on the spot at search_mps. */
static uint32_t search_leg_us(const s3_drive_t *d)
{
    const float rad = d->options.search_deg * 0.0174533f;
    const float s = rad * 0.5f * d->options.track_m / d->options.search_mps;
    return (uint32_t)(s * 1000000.0f) + 1U;
}
/* The turn ended with no front sensor on the new line. If the last touch was
 * in the second half of the spin, the line was passed (overshoot): sweep back
 * first; otherwise (undershoot, or only the old line touched early) keep
 * turning the same way first. Both sides are swept either way. */
static void start_search(s3_drive_t *d, uint32_t now)
{
    const int8_t turn_sign = (d->turn == S3_NAV_RIGHT) ? -1 : 1;
    const uint32_t spin_us = now - d->phase_us;
    const bool overshoot = d->touched && (spin_us > 0U) &&
                           ((uint32_t)(now - d->touch_us) < spin_us / 2U);
    d->search_sign = overshoot ? (int8_t)-turn_sign : turn_sign;
    d->search_leg = 0U;
    d->search_us = now;
    d->phase_us = now;
    d->searching = true;
}
static void search_step(s3_drive_t *d, uint32_t now)
{
    float v;
    if (d->output.line.valid && d->output.line.touching)
    {
        d->searching = false;
        d->port.stop();
        (void)s3_navigation_set_mode(&d->navigation, S3_MODE_TRACK);
        d->state = S3_DRIVE_REACQUIRE;
        d->phase_us = now;
        return;
    }
    if ((uint32_t)(now - d->search_us) >=
        search_leg_us(d) * (d->search_leg == 0U ? 1U : 2U))
    {
        if (d->search_leg != 0U)
        {
            halt(d, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
            return;
        }
        d->search_leg = 1U;
        d->search_sign = (int8_t)-d->search_sign;
        d->search_us = now;
    }
    v = d->options.search_mps * (float)d->search_sign; /* + = anticlockwise */
    if (d->port.velocity(-v, v) != MOTION_OK)
    {
        halt(d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
    }
}
static void follow(s3_drive_t *d, uint32_t now)
{
    const s3_navigation_output_t *out = &d->output;
    float left, right;
    if (out->events & S3_EVENT_LINE_LOST)
    {
        halt(d, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
        return;
    }
    if (out->events & S3_EVENT_BARCODE)
    {
        if (out->action == S3_NAV_NONE)
        {
            halt(d, S3_DRIVE_HALTED, S3_DRIVE_UNMAPPED);
            return;
        }
        if (d->pending != S3_NAV_NONE)
        {
            halt(d, S3_DRIVE_HALTED, S3_DRIVE_NO_DIRECTION);
            return;
        }
        d->pending = out->action;
        d->pending_us = now;
    }
    if ((d->pending != S3_NAV_NONE) &&
        ((uint32_t)(now - d->pending_us) > d->options.pending_wait_us))
    {
        halt(d, S3_DRIVE_HALTED, S3_DRIVE_TIMEOUT);
        return;
    }
    if (out->events & S3_EVENT_JUNCTION)
    {
        if (d->options.cross_straight &&
            ((d->pending == S3_NAV_NONE) || (d->pending == S3_NAV_STRAIGHT)))
        {
            d->pending = S3_NAV_NONE; /* straight across, keep following */
        }
        else if (d->pending == S3_NAV_NONE)
        {
            halt(d, S3_DRIVE_HALTED, S3_DRIVE_NO_DIRECTION);
            return;
        }
        else
        {
            d->junction_m = d->feedback.centre_m;
            request_stop(d, now);
            d->state = S3_DRIVE_BRAKING;
            return;
        }
    }
    if (d->waiting_stop)
    {
        if ((uint32_t)(now - d->phase_us) > S3_BRAKE_WAIT_US)
        {
            halt(d, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
            return;
        }
        if (!stopped_after_request(d))
        {
            return;
        }
        if (!out->steering.valid && !straight_across(d))
        {
            return;
        }
        d->waiting_stop = false;
    }
    if (out->steering.valid)
    {
        left = out->steering.left_mps;
        right = out->steering.right_mps;
    }
    else if (straight_across(d))
    {
        left = right = d->navigation.config.steering.base_mps;
    }
    else
    {
        request_stop(d, now);
        return;
    }
    if (d->port.velocity(left, right) != MOTION_OK)
    {
        halt(d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
    }
    else
    {
        d->command_id = d->port.command_id();
        if (d->command_id == 0U)
        {
            halt(d, S3_DRIVE_FAULT, S3_DRIVE_REPORT_FAULT);
        }
    }
}
bool s3_drive_process(s3_drive_t *d, const s3_sensor_frame_t *frame,
                      const motion_feedback_t *feedback)
{
    s3_odometry_sample_t odometry = {0};
    uint32_t now;
    bool running, done, reports_healthy;
    if ((d == 0) || !d->configured)
    {
        return false;
    }
    if (frame == 0)
    {
        halt(d, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
        return false;
    }
    now = frame->timestamp_us;
    d->feedback = feedback ? *feedback : (motion_feedback_t){0};
    running = (d->state >= S3_DRIVE_FOLLOW) && (d->state <= S3_DRIVE_REACQUIRE);
    reports_healthy = receive_reports(d, running, &done);
    if (running && d->feedback.report_loss_count != d->report_loss_count)
    {
        reports_healthy = false;
    }
    d->report_loss_count = d->feedback.report_loss_count;
    odometry =
        (s3_odometry_sample_t){d->feedback.centre_m, d->feedback.timestamp_us,
                               d->feedback.epoch, d->feedback.valid};
    if (!s3_navigation_process(&d->navigation, frame, &odometry, &d->output))
    {
        halt(d, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
        return false;
    }
    if (d->reset_result == S3_RESET_PENDING)
    {
        finish_reset(d, now);
        /* Still disarmed whether the reset completed, is waiting, or timed
         * out. Never let a fault reset fall through to a driving action. */
        return true;
    }
    running = (d->state >= S3_DRIVE_FOLLOW) && (d->state <= S3_DRIVE_REACQUIRE);
    if (d->output.stop_requested)
    {
        halt(d, S3_DRIVE_FAULT, S3_DRIVE_SENSOR_FAULT);
        return true;
    }
    if (running &&
        (!feedback_fresh(d, now) || (d->feedback.state == MOTION_FAULT) ||
         (d->feedback.fault != MOTION_FAULT_NONE)))
    {
        halt(d, S3_DRIVE_FAULT, S3_DRIVE_MOTION_FAULT);
        return true;
    }
    if (!running)
    {
        return true;
    }
    if (!reports_healthy ||
        (d->command_id != 0U && d->feedback.command_id != d->command_id))
    {
        halt(d, S3_DRIVE_FAULT, S3_DRIVE_REPORT_FAULT);
        return true;
    }
    d->port.heartbeat(true);
    if (d->state == S3_DRIVE_FOLLOW)
    {
        follow(d, now);
    }
    else if (d->state == S3_DRIVE_BRAKING)
    {
        if ((uint32_t)(now - d->phase_us) > S3_BRAKE_WAIT_US)
        {
            halt(d, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
        }
        else if (stopped_after_request(d))
        {
            d->waiting_stop = false;
            (void)s3_navigation_set_mode(&d->navigation, S3_MODE_SUPPRESSED);
            d->turn = d->pending;
            if (!start_manoeuvre(d))
            {
                halt(d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
            }
            else
            {
                d->pending = S3_NAV_NONE;
                d->state = S3_DRIVE_MANOEUVRE;
                d->phase_us = now;
            }
        }
    }
    else if ((d->state == S3_DRIVE_MANOEUVRE) && d->searching)
    {
        if ((uint32_t)(now - d->phase_us) > S3_MANOEUVRE_WAIT_US)
        {
            halt(d, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
        }
        else
        {
            search_step(d, now);
        }
    }
    else if (d->state == S3_DRIVE_MANOEUVRE)
    {
        if (!d->advancing && d->output.line.valid && d->output.line.touching)
        {
            d->touch_us = now;
            d->touched = true;
        }
        if ((uint32_t)(now - d->phase_us) > S3_MANOEUVRE_WAIT_US)
        {
            halt(d, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
        }
        else if (done && d->advancing)
        {
            /* Axle now over the crossing: turn on the spot from here. */
            d->command_id = 0U;
            d->advancing = false;
            if (!request_turn(d))
            {
                halt(d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
            }
            else
            {
                d->phase_us = now;
            }
        }
        else if (done)
        {
            const bool turned = (d->turn == S3_NAV_LEFT) ||
                                (d->turn == S3_NAV_RIGHT) ||
                                (d->turn == S3_NAV_UTURN);
            d->command_id = 0U;
            d->port.stop();
            if (turned && (d->options.turn_line_window_us != 0U) &&
                !(d->touched && ((uint32_t)(now - d->touch_us) <=
                                 d->options.turn_line_window_us)))
            {
                /* The sensors never crossed the new line near the end of the
                 * turn: they are beside it, not over it. Do not drive off:
                 * sweep for it if enabled, else stop. */
                if (d->options.search_deg > 0.0f)
                {
                    start_search(d, now); /* stays in MANOEUVRE */
                }
                else
                {
                    halt(d, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
                }
            }
            else
            {
                (void)s3_navigation_set_mode(&d->navigation, S3_MODE_TRACK);
                d->state = S3_DRIVE_REACQUIRE;
                d->phase_us = now;
            }
        }
        else if ((d->feedback.state != MOTION_TURN) &&
                 (d->feedback.state != MOTION_DRIVE) &&
                 (d->feedback.state != MOTION_COMPLETE))
        {
            halt(d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
        }
    }
    else
    {
        d->port.stop();
        if ((uint32_t)(now - d->phase_us) > S3_REACQUIRE_WAIT_US)
        {
            halt(d, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
        }
        else if (d->output.steering.valid && stopped_after_request(d))
        {
            d->state = S3_DRIVE_FOLLOW;
        }
    }
    return true;
}
