/* Controller slice for Buddy 2 + Buddy 3 only. No GPIO/RTOS in this file. */
#include "s3_drive.h"
#define S3_BRAKE_WAIT_US (500000U)
#define S3_MANOEUVRE_WAIT_US (10000000U)
#define S3_REACQUIRE_WAIT_US (1000000U)
#define S3_PENDING_WAIT_US (5000000U)
static bool feedback_fresh(const s3_drive_t *d, uint32_t now)
{
    return d->feedback.valid && ((uint32_t)(now - d->feedback.timestamp_us) <=
                                 d->navigation.config.max_sample_age_us);
}
static void halt(s3_drive_t *d, s3_drive_state_t state,
                 s3_drive_reason_t reason)
{
    d->port.stop();
    d->port.heartbeat(false);
    (void)s3_navigation_end_run(
        &d->navigation); /* A sensor latch is retained. */
    d->pending = S3_NAV_NONE;
    d->output.steering.valid = false;
    d->output.action = S3_NAV_NONE;
    d->output.events &= S3_EVENT_SENSOR_FAULT;
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
        !port->poll || !port->reset || !port->heartbeat ||
        !s3_navigation_init(&d->navigation, cfg))
    {
        return false;
    }
    d->port = *port;
    d->configured = true;
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
    if ((d == 0) || !d->configured || (d->state != S3_DRIVE_FAULT))
    {
        return false;
    }
    d->port.stop();
    d->port.heartbeat(false);
    if (!d->port.reset() || !feedback_fresh(d, now) ||
        (d->feedback.state != MOTION_IDLE) ||
        (d->feedback.fault != MOTION_FAULT_NONE) || !d->feedback.braked)
    {
        return false;
    }
    if (d->navigation.lifecycle == S3_RUN_FAULT_LATCHED)
    {
        if (!s3_navigation_reset_fault(&d->navigation, now))
        {
            return false;
        }
    }
    if (s3_navigation_readiness(&d->navigation, now) != 0U)
    {
        return false;
    }
    d->state = S3_DRIVE_OFF;
    d->reason = S3_DRIVE_OK;
    return true;
}
static bool request_turn(s3_drive_t *d)
{
    manoeuvre_t m;
    switch (d->pending)
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
    return d->port.manoeuvre(m) == MOTION_OK;
}
static void follow(s3_drive_t *d, uint32_t now)
{
    const s3_navigation_output_t *out = &d->output;
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
        ((uint32_t)(now - d->pending_us) > S3_PENDING_WAIT_US))
    {
        halt(d, S3_DRIVE_HALTED, S3_DRIVE_TIMEOUT);
        return;
    }
    if (out->events & S3_EVENT_JUNCTION)
    {
        if (d->pending == S3_NAV_NONE)
        {
            halt(d, S3_DRIVE_HALTED, S3_DRIVE_NO_DIRECTION);
            return;
        }
        d->port.stop();
        d->state = S3_DRIVE_BRAKING;
        d->phase_us = now;
        return;
    }
    if (!out->steering.valid)
    {
        d->port.stop();
        return;
    }
    if (d->port.velocity(out->steering.left_mps, out->steering.right_mps) !=
        MOTION_OK)
    {
        halt(d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
    }
}
bool s3_drive_process(s3_drive_t *d, const s3_sensor_frame_t *frame,
                      const motion_feedback_t *feedback)
{
    s3_odometry_sample_t odometry = {0};
    uint32_t now;
    bool running;
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
    odometry =
        (s3_odometry_sample_t){d->feedback.centre_m, d->feedback.timestamp_us,
                               d->feedback.epoch, d->feedback.valid};
    if (!s3_navigation_process(&d->navigation, frame, &odometry, &d->output))
    {
        halt(d, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
        return false;
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
        else if (d->feedback.braked && (d->feedback.state == MOTION_IDLE) &&
                 ((int32_t)(d->feedback.timestamp_us - d->phase_us) > 0))
        {
            (void)s3_navigation_set_mode(&d->navigation, S3_MODE_SUPPRESSED);
            if (!request_turn(d))
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
    else if (d->state == S3_DRIVE_MANOEUVRE)
    {
        if ((uint32_t)(now - d->phase_us) > S3_MANOEUVRE_WAIT_US)
        {
            halt(d, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
        }
        else if (d->feedback.state == MOTION_COMPLETE)
        {
            if (d->port.poll() != MOTION_COMPLETE)
            {
                halt(d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
            }
            else
            {
                d->port.stop();
                (void)s3_navigation_set_mode(&d->navigation, S3_MODE_TRACK);
                d->state = S3_DRIVE_REACQUIRE;
                d->phase_us = now;
            }
        }
        else if ((d->feedback.state != MOTION_TURN) &&
                 (d->feedback.state != MOTION_DRIVE))
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
        else if (d->output.steering.valid)
        {
            d->state = S3_DRIVE_FOLLOW;
        }
    }
    return true;
}
