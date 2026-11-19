/** @file s3_drive.c
 * @brief Own line following, manoeuvres, fault handling and optional avoidance
 * arbitration.
 */
/* Temporary controller for Buddy 2 + Buddy 3; replace with the team controller.
 * This is orchestration, not Buddy 3 sensing. No GPIO/RTOS in this file. */
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

static void report_obstacle_status (const s3_drive_t * p_d)
{
    unsigned leg = p_d->obstacle.leg;
    if (p_d->obstacle.state == S5_MOVING && p_d->obstacle.leg == 0U &&
        p_d->obstacle.backing)
    {
        leg = S5_TELEMETRY_LEG_REVERSE;
    }
    if (p_d->obstacle.state == S5_SCANNING && p_d->obstacle.aiming)
    {
        leg = S5_TELEMETRY_LEG_AIM;
    }
    subsystem5_set_control_status(p_d->obstacle.state, p_d->obstacle.fault,
                                  p_d->obstacle.plan.route, leg,
                                  p_d->obstacle.pass, p_d->obstacle.swapped);
}
#endif
#define S3_BRAKE_WAIT_US (500000U)
#define S3_MANOEUVRE_WAIT_US (10000000U)
#define S3_REACQUIRE_WAIT_US (1000000U)
#define S3_PENDING_WAIT_US (5000000U)
#define S3_MIN_ADVANCE_M (0.003f) /* shorter advances are skipped */

s3_drive_options_t s3_drive_default_options (void)
{
    return (s3_drive_options_t){false, 0.0f, 0.10f, 0,     S3_PENDING_WAIT_US,
                                0U,    0.0f, 0.05f, 0.116f};
}

static bool finite_f (float32_t value_x)
{
    return (value_x == value_x) && ((value_x - value_x) == 0.0f);
}

static bool feedback_fresh (const s3_drive_t * p_d, uint32_t now)
{
    return p_d->feedback.valid &&
           ((uint32_t)(now - p_d->feedback.timestamp_us) <=
            p_d->navigation.config.max_sample_age_us);
}

static void halt (s3_drive_t * p_d, s3_drive_state_t state,
                  s3_drive_reason_t reason)
{
    if (p_d->reset_result == S3_RESET_PENDING)
    {
        p_d->reset_result = S3_RESET_FAILED;
    }
    /* Brake before obstacle cleanup, which can wait for the scanner mutex. */
    p_d->external_hold = false;
    p_d->port.stop();
    p_d->port.heartbeat(false);
#if APP_S5
    subsystem5_cancel_scan();
    p_d->obstacle_snapshot_valid = false;
    if (state == S3_DRIVE_FAULT && p_d->obstacle_active &&
        p_d->obstacle.state != S5_FAULT)
    {
        p_d->obstacle.state = S5_FAULT;
        p_d->obstacle.fault = S5_INTERRUPTED;
    }
    p_d->obstacle_active = false;
    if (state != S3_DRIVE_FAULT)
    {
        s5_avoidance_abort(&p_d->obstacle);
    }
    report_obstacle_status(p_d);
#endif
    (void)s3_navigation_end_run(
        &p_d->navigation); /* A sensor latch is retained. */
    p_d->pending = S3_NAV_NONE;
    p_d->turn = S3_NAV_NONE;
    p_d->advancing = false;
    p_d->touched = false;
    p_d->searching = false;
    p_d->command_id = p_d->cancel_id = 0U;
    p_d->waiting_stop = false;
    p_d->output.steering.valid = false;
    p_d->output.action = S3_NAV_NONE;
    /* Keep this cycle's barcode event too: a barcode that causes the halt
     * (e.g. an unmapped letter) must still reach telemetry. */
    p_d->output.events &= (S3_EVENT_SENSOR_FAULT | S3_EVENT_BARCODE);
    p_d->output.lifecycle = p_d->navigation.lifecycle;
    p_d->output.diagnostic = true;
    p_d->state = state;
    p_d->reason = reason;
}

void s3_drive_terrain_fault (s3_drive_t * p_d)
{
    if (p_d && p_d->configured)
    {
        halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_TERRAIN_FAULT);
    }
}

bool s3_drive_init (s3_drive_t * p_d, const s3_navigation_config_t * p_cfg,
                    const s3_drive_port_t * p_port)
{
    if (p_d == 0)
    {
        return false;
    }
    *p_d = (s3_drive_t){0};
    if ((p_port == 0) || !p_port->velocity || !p_port->manoeuvre ||
        !p_port->stop || !p_port->report || !p_port->command_id ||
        !p_port->reset || !p_port->heartbeat ||
        !s3_navigation_init(&p_d->navigation, p_cfg))
    {
        return false;
    }
    p_d->port = *p_port;
    p_d->options = s3_drive_default_options();
    p_d->configured = true;
#if APP_S5
    const s5_config_t obstacle_config = s5_defaults();
    s5_avoidance_init(&p_d->obstacle, &obstacle_config);
#endif
    return true;
}

bool s3_drive_set_options (s3_drive_t * p_d, const s3_drive_options_t * p_o)
{
    if ((p_d == 0) || !p_d->configured || (p_o == 0) ||
        ((p_d->state != S3_DRIVE_OFF) && (p_d->state != S3_DRIVE_HALTED) &&
         (p_d->state != S3_DRIVE_FAULT)) ||
        !finite_f(p_o->turn_advance_m) || !finite_f(p_o->advance_mps) ||
        (p_o->turn_advance_m < 0.0f) || (p_o->turn_advance_m > 0.30f) ||
        ((p_o->turn_advance_m > 0.0f) &&
         ((p_o->drive == 0) || (p_o->advance_mps <= 0.0f))) ||
        (p_o->pending_wait_us == 0U) || (p_o->pending_wait_us >= 0x80000000U) ||
        (p_o->turn_line_window_us >= 0x80000000U) ||
        !finite_f(p_o->search_deg) || (p_o->search_deg < 0.0f) ||
        (p_o->search_deg > 60.0f) ||
        ((p_o->search_deg > 0.0f) &&
         ((p_o->turn_line_window_us == 0U) || !finite_f(p_o->search_mps) ||
          (p_o->search_mps <= 0.0f) || (p_o->search_mps > 0.30f) ||
          !finite_f(p_o->track_m) || (p_o->track_m < 0.05f) ||
          (p_o->track_m > 0.50f))))
    {
        return false;
    }
    p_d->options = *p_o;
    return true;
}

bool s3_drive_start (s3_drive_t * p_d, uint32_t now)
{
#if APP_S5
    s5_observation_t obstacle;
    if (!subsystem5_snapshot(&obstacle) || !s5_range_ready(&obstacle, now))
    {
        return false;
    }
#endif
    if ((p_d == 0) || !p_d->configured ||
        ((p_d->state != S3_DRIVE_OFF) && (p_d->state != S3_DRIVE_HALTED)) ||
        !feedback_fresh(p_d, now) || (p_d->feedback.state != MOTION_IDLE) ||
        (p_d->feedback.fault != MOTION_FAULT_NONE) || !p_d->feedback.braked ||
        !s3_navigation_start(&p_d->navigation, now))
    {
        return false;
    }
    (void)s3_navigation_set_mode(&p_d->navigation, S3_MODE_TRACK);
#if APP_S5
    p_d->obstacle_snapshot_valid =
        s5_observation_pack(&obstacle, p_d->obstacle_snapshot);
    p_d->obstacle_snapshot_us = now;
    s5_avoidance_abort(&p_d->obstacle);
    p_d->obstacle.fault = S5_OK;
    subsystem5_set_control_status(S5_IDLE, S5_OK, S5_ROUTE_NONE, 0U, 0U, 0U);
#endif
    p_d->state = S3_DRIVE_FOLLOW;
    p_d->reason = S3_DRIVE_OK;
    p_d->pending = S3_NAV_NONE;
    p_d->port.heartbeat(true);
    return true;
}

void s3_drive_stop (s3_drive_t * p_d)
{
    if ((p_d == 0) || !p_d->configured)
    {
        return;
    }
    if ((p_d->state == S3_DRIVE_FAULT) ||
        (p_d->navigation.lifecycle == S3_RUN_FAULT_LATCHED))
    {
        halt(p_d, S3_DRIVE_FAULT, p_d->reason);
    }
    else
    {
        halt(p_d, S3_DRIVE_OFF, S3_DRIVE_OK);
    }
}

bool s3_drive_reset (s3_drive_t * p_d, uint32_t now)
{
    if ((p_d == 0) || !p_d->configured || (p_d->state != S3_DRIVE_FAULT) ||
        (p_d->reset_result == S3_RESET_PENDING))
    {
        return false;
    }
    p_d->port.stop();
    p_d->port.heartbeat(false);
    p_d->reset_result = S3_RESET_FAILED;
    if (!p_d->port.reset())
    {
        return false;
    }
    /* Buddy 2 resets its controller synchronously but publishes feedback on
     * its next control tick. The cached snapshot is therefore pre-reset;
     * checking it here incorrectly required a second button press. */
    p_d->reset_us = now;
    p_d->reset_result = S3_RESET_PENDING;
    return true;
}

s3_reset_result_t s3_drive_reset_result (const s3_drive_t * p_d)
{
    return p_d ? (s3_reset_result_t)p_d->reset_result : S3_RESET_FAILED;
}

static void finish_reset (s3_drive_t * p_d, uint32_t now)
{
    if ((uint32_t)(now - p_d->reset_us) >= S3_RESET_WAIT_US)
    {
        p_d->reset_result = S3_RESET_FAILED;
        return;
    }
    if (!feedback_fresh(p_d, now) ||
        ((int32_t)(p_d->feedback.timestamp_us - p_d->reset_us) <= 0) ||
        (p_d->feedback.state != MOTION_IDLE) ||
        (p_d->feedback.fault != MOTION_FAULT_NONE) || !p_d->feedback.braked)
    {
        return;
    }
    if ((p_d->navigation.lifecycle == S3_RUN_FAULT_LATCHED) &&
        !s3_navigation_reset_fault(&p_d->navigation, now))
    {
        return;
    }
    if (s3_navigation_readiness(&p_d->navigation, now) != 0U)
    {
        return;
    }
    p_d->state = S3_DRIVE_OFF;
    p_d->reason = S3_DRIVE_OK;
    p_d->output.lifecycle = p_d->navigation.lifecycle;
    p_d->output.stop_requested = false;
    p_d->reset_result = S3_RESET_OK;
}

static bool request_turn (s3_drive_t * p_d)
{
    manoeuvre_t value_m;
    switch (p_d->turn)
    {
    case S3_NAV_LEFT:
        value_m = MANOEUVRE_LEFT;
        break;
    case S3_NAV_RIGHT:
        value_m = MANOEUVRE_RIGHT;
        break;
    case S3_NAV_STRAIGHT:
        value_m = MANOEUVRE_STRAIGHT;
        break;
    case S3_NAV_UTURN:
        value_m = MANOEUVRE_U_TURN;
        break;
    default:
        return false;
    }
    if (p_d->port.manoeuvre(value_m) != MOTION_OK)
    {
        return false;
    }
    p_d->command_id = p_d->port.command_id();
    return p_d->command_id != 0U;
}

/* LEFT/RIGHT pivot on the axle, which is behind the sensors. Optionally drive
 * on first so the axle reaches the crossing; the braking distance already
 * travelled since detection is subtracted (same odometry epoch only). */
static bool start_manoeuvre (s3_drive_t * p_d)
{
    const float64_t travelled = p_d->feedback.centre_m - p_d->junction_m;
    const float32_t advance =
        p_d->options.turn_advance_m - (float32_t)travelled;
    p_d->touched = false;
    if (((p_d->turn == S3_NAV_LEFT) || (p_d->turn == S3_NAV_RIGHT)) &&
        (p_d->options.turn_advance_m > 0.0f) && (advance > S3_MIN_ADVANCE_M))
    {
        if (p_d->options.drive(advance, p_d->options.advance_mps) != MOTION_OK)
        {
            return false;
        }
        p_d->advancing = true;
        p_d->command_id = p_d->port.command_id();
        return p_d->command_id != 0U;
    }
    p_d->advancing = false;
    return request_turn(p_d);
}

/* Unannounced crossing (or a STRAIGHT action) in cross_straight mode: steering
 * is undefined while both sensors are dark, so hold both wheels at base speed
 * until the crossing has passed under the sensors. */
static bool straight_across (const s3_drive_t * p_d)
{
    return p_d->options.cross_straight && p_d->output.line.valid &&
           p_d->output.line.junction_candidate &&
           ((p_d->pending == S3_NAV_NONE) || (p_d->pending == S3_NAV_STRAIGHT));
}

/* A stop request is not a stopped acknowledgement. Wait for the motion
 * owner to apply its brake and observe stationary wheels before resuming. */
static void request_stop (s3_drive_t * p_d, uint32_t now)
{
    if (!p_d->waiting_stop)
    {
        p_d->cancel_id = p_d->command_id;
        p_d->command_id = 0U;
        p_d->waiting_stop = true;
        p_d->phase_us = now;
        p_d->port.stop();
    }
}

static bool stopped_after_request (const s3_drive_t * p_d)
{
    return p_d->feedback.braked && p_d->feedback.state == MOTION_IDLE &&
           p_d->cancel_id == 0U &&
           (int32_t)(p_d->feedback.timestamp_us - p_d->phase_us) > 0;
}

/* Bounded drain, also while disarmed. Never use motion_poll(): it does not
 * consume reports. Only a matching DONE may finish our current manoeuvre. */
static bool receive_reports (s3_drive_t * p_d, bool running, bool * p_done)
{
    motion_report_t report;
    bool healthy = true;
    *p_done = false;
    for (unsigned int value_i = 0U; value_i < MOTION_REPORT_QUEUE_LEN;
         value_i++)
    {
        if (!p_d->port.report(&report))
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
        if (report.command_id == p_d->cancel_id)
        {
            if (report.outcome != MOTION_OUTCOME_CANCELLED)
            {
                healthy = false;
            }
            p_d->cancel_id = 0U;
        }
        else if (report.command_id == p_d->command_id)
        {
            if (p_d->state == S3_DRIVE_MANOEUVRE &&
                report.outcome == MOTION_OUTCOME_DONE && !*p_done &&
                report.fault == MOTION_FAULT_NONE)
            {
                *p_done = true;
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
static uint32_t search_leg_us (const s3_drive_t * p_d)
{
    const float32_t rad = p_d->options.search_deg * 0.0174533f;
    const float32_t value_s =
        rad * 0.5f * p_d->options.track_m / p_d->options.search_mps;
    return (uint32_t)(value_s * 1000000.0f) + 1U;
}

/* The turn ended with no front sensor on the new line. If the last touch was
 * in the second half of the spin, the line was passed (overshoot): sweep back
 * first; otherwise (undershoot, or only the old line touched early) keep
 * turning the same way first. Both sides are swept either way. */
static void start_search (s3_drive_t * p_d, uint32_t now)
{
    const int8_t turn_sign = (p_d->turn == S3_NAV_RIGHT) ? -1 : 1;
    const uint32_t spin_us = now - p_d->phase_us;
    const bool overshoot = p_d->touched && (spin_us > 0U) &&
                           ((uint32_t)(now - p_d->touch_us) < spin_us / 2U);
    p_d->search_sign = overshoot ? (int8_t)-turn_sign : turn_sign;
    p_d->search_leg = 0U;
    p_d->search_us = now;
    p_d->phase_us = now;
    p_d->searching = true;
}

static void search_step (s3_drive_t * p_d, uint32_t now)
{
    float32_t value_v;
    if (p_d->output.line.valid && p_d->output.line.touching)
    {
        p_d->searching = false;
        p_d->port.stop();
        (void)s3_navigation_set_mode(&p_d->navigation, S3_MODE_TRACK);
        p_d->state = S3_DRIVE_REACQUIRE;
        p_d->phase_us = now;
        return;
    }
    if ((uint32_t)(now - p_d->search_us) >=
        search_leg_us(p_d) * (p_d->search_leg == 0U ? 1U : 2U))
    {
        if (p_d->search_leg != 0U)
        {
            halt(p_d, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
            return;
        }
        p_d->search_leg = 1U;
        p_d->search_sign = (int8_t)-p_d->search_sign;
        p_d->search_us = now;
    }
    value_v = p_d->options.search_mps *
              (float32_t)p_d->search_sign; /* + = anticlockwise */
    if (p_d->port.velocity(-value_v, value_v) != MOTION_OK)
    {
        halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
    }
}
#if APP_S5
/* How hard the line follower is turning (+ = left), time-averaged, tau 0.5 s.
 */
static void turn_track (s3_drive_t * p_d, float32_t left, float32_t right,
                        uint32_t now)
{
    const float32_t sum = left + right;
    const uint32_t value_dt =
        p_d->turn_us ? (uint32_t)(now - p_d->turn_us) : 0U;
    p_d->turn_us = now;
    if (sum < 0.01f || value_dt == 0U || value_dt > 200000U)
    {
        return;
    }
    {
        /* Fixed point, 1/10000: s += (x - s) * dt / (tau + dt), tau 0.5 s. */
        float32_t value_r = (right - left) / sum;
        int32_t value_x, value_s = p_d->turn_q;
        value_r = value_r > 1.0f ? 1.0f : (value_r < -1.0f ? -1.0f : value_r);
        value_x = (int32_t)(value_r * 10000.0f);
        value_s +=
            (int32_t)(((int64_t)(value_x - value_s) * (int64_t)value_dt) /
                      (int64_t)(500000U + value_dt));
        p_d->turn_q =
            (int16_t)(value_s > 10000 ? 10000
                                      : (value_s < -10000 ? -10000 : value_s));
    }
}

/* Latched shared value: whole percent toward zero, -100..100 (so 19.99 % is
 * still below the 20 % bend threshold, as with the old float compare). */
static int8_t turn_pct (int16_t value_q)
{
    return (int8_t)(value_q / 100);
}
#endif

static void follow (s3_drive_t * p_d, uint32_t now)
{
    const s3_navigation_output_t * p_out = &p_d->output;
    float32_t left, right;
    if (p_out->events & S3_EVENT_LINE_LOST)
    {
        halt(p_d, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
        return;
    }
    if (p_out->events & S3_EVENT_BARCODE)
    {
        if (p_out->action == S3_NAV_NONE)
        {
            halt(p_d, S3_DRIVE_HALTED, S3_DRIVE_UNMAPPED);
            return;
        }
        if (p_d->pending != S3_NAV_NONE)
        {
            halt(p_d, S3_DRIVE_HALTED, S3_DRIVE_NO_DIRECTION);
            return;
        }
        p_d->pending = p_out->action;
        p_d->pending_us = now;
    }
    if ((p_d->pending != S3_NAV_NONE) &&
        ((uint32_t)(now - p_d->pending_us) > p_d->options.pending_wait_us))
    {
        halt(p_d, S3_DRIVE_HALTED, S3_DRIVE_TIMEOUT);
        return;
    }
    if (p_out->events & S3_EVENT_JUNCTION)
    {
        if (p_d->options.cross_straight && ((p_d->pending == S3_NAV_NONE) ||
                                            (p_d->pending == S3_NAV_STRAIGHT)))
        {
            p_d->pending = S3_NAV_NONE; /* straight across, keep following */
        }
        else if (p_d->pending == S3_NAV_NONE)
        {
            halt(p_d, S3_DRIVE_HALTED, S3_DRIVE_NO_DIRECTION);
            return;
        }
        else
        {
            p_d->junction_m = p_d->feedback.centre_m;
            request_stop(p_d, now);
            p_d->state = S3_DRIVE_BRAKING;
            return;
        }
    }
    if (p_d->waiting_stop)
    {
        if ((uint32_t)(now - p_d->phase_us) > S3_BRAKE_WAIT_US)
        {
            halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
            return;
        }
        if (!stopped_after_request(p_d))
        {
            return;
        }
        if (!p_out->steering.valid && !straight_across(p_d))
        {
            return;
        }
        p_d->waiting_stop = false;
    }
    if (p_out->steering.valid)
    {
        left = p_out->steering.left_mps;
        right = p_out->steering.right_mps;
    }
    else if (straight_across(p_d))
    {
        left = right = p_d->navigation.config.steering.base_mps;
    }
    else
    {
        request_stop(p_d, now);
        return;
    }
#if APP_S5
    turn_track(p_d, left, right, now);
#endif
    if (p_d->port.velocity(left, right) != MOTION_OK)
    {
        halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
    }
    else
    {
        p_d->command_id = p_d->port.command_id();
        if (p_d->command_id == 0U)
        {
            halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_REPORT_FAULT);
        }
    }
}

#if APP_S5
/* Called only by the existing sample/controller task after its one report
 * drain. S5's pure FSM proposes actions; this owner alone authorises Buddy 2
 * requests. */
static bool obstacle_snapshot (s3_drive_t * p_d, uint32_t now,
                               s5_observation_t * p_out)
{
    s5_snapshot_result_t result = subsystem5_read_snapshot(p_out);
    if (result == S5_SNAPSHOT_OK)
    {
        p_d->obstacle_snapshot_valid =
            s5_observation_pack(p_out, p_d->obstacle_snapshot);
        p_d->obstacle_snapshot_us = now;
        return p_d->obstacle_snapshot_valid;
    }
    if (result == S5_SNAPSHOT_BUSY && p_d->obstacle_snapshot_valid &&
        (uint32_t)(now - p_d->obstacle_snapshot_us) <= S5_RANGE_FRESH_US)
    {
        return s5_observation_unpack(p_d->obstacle_snapshot, p_out);
    }
    p_d->obstacle_snapshot_valid = false;
    return false;
}

static bool obstacle_control (s3_drive_t * p_d, uint32_t now, bool done)
{
    s5_observation_t observation = {0};
    bool have = obstacle_snapshot(p_d, now, &observation);
    if (!p_d->obstacle_active)
    {
        if (!have || !s5_range_ready(&observation, now))
        {
            p_d->obstacle.fault = S5_BAD_RANGE;
            p_d->obstacle.state = S5_FAULT;
            halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
            return true;
        }
        if (p_d->state != S3_DRIVE_FOLLOW)
        {
            if (observation.range_mm <= p_d->obstacle.config.emergency_mm)
            {
                p_d->obstacle.fault = S5_NO_PATH;
                p_d->obstacle.state = S5_FAULT;
                halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
                return true;
            }
            return false;
        }
        if (observation.range_mm > p_d->obstacle.config.detect_mm)
        {
            return false;
        }
        if (p_d->output.events & S3_EVENT_LINE_LOST)
        {
            halt(p_d, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
            return true;
        }
        /* Do not lose a barcode decoded on the same cycle as obstacle
         * detection. Its pending lifetime is paused for the detour, never
         * executed off-line. */
        if (p_d->output.events & S3_EVENT_BARCODE)
        {
            if (p_d->pending != S3_NAV_NONE ||
                p_d->output.action == S3_NAV_NONE)
            {
                halt(p_d, S3_DRIVE_HALTED, S3_DRIVE_UNMAPPED);
                return true;
            }
            p_d->pending = p_d->output.action;
            p_d->pending_us = now;
        }
        s5_avoidance_begin(&p_d->obstacle, now);
        p_d->obstacle_active = true;
        p_d->obstacle_turn_pct =
            turn_pct(p_d->turn_q); /* bend or straight, at detection */
        request_stop(p_d, now);
        p_d->state = S3_DRIVE_MANOEUVRE;
        (void)s3_navigation_set_mode(&p_d->navigation, S3_MODE_SUPPRESSED);
    }
    if (done)
    {
        p_d->command_id = 0U;
    }
    if (observation.scan_id != p_d->obstacle_scan_id)
    {
        observation.scan_done = false;
    }
    const s5_input_t input = {.now_us = now,
                              .centre_m = p_d->feedback.centre_m,
                              .stopped = stopped_after_request(p_d),
                              .motion_done = done,
                              .healthy = have && p_d->output.line.valid,
                              .tape = p_d->navigation.config.line.between
                                          ? p_d->output.line.touching
                                          : p_d->output.line.present,
                              .junction = p_d->output.line.junction_candidate,
                              .line_ready = p_d->output.steering_preview.valid,
                              .turn_pct = p_d->obstacle_turn_pct,
                              .observation = observation};
    s5_action_t request = s5_avoidance_step(&p_d->obstacle, &input);
    motion_result_t result = MOTION_OK;
    if (p_d->obstacle.state == S5_FAULT)
    {
        halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
        return true;
    }
    switch (request.kind)
    {
    case S5_ACT_STOP:
        p_d->port.stop();
        p_d->phase_us = now;
        p_d->command_id = 0U;
        break;
    case S5_ACT_SCAN:
        p_d->waiting_stop = false;
        if (++p_d->obstacle_scan_id == 0U)
        {
            p_d->obstacle_scan_id = 1U;
        }
        if (!subsystem5_request_scan(p_d->obstacle_scan_id))
        {
            result = MOTION_ERR_BUSY;
        }
        break;
    case S5_ACT_PEEK: /* one side reading while stopped (bypass leg 3) */
        p_d->waiting_stop = false;
        if (++p_d->obstacle_scan_id == 0U)
        {
            p_d->obstacle_scan_id = 1U;
        }
        if (!subsystem5_request_peek(p_d->obstacle_scan_id,
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
        p_d->obstacle_active = false;
        p_d->waiting_stop = false;
        p_d->command_id = p_d->cancel_id = 0U;
        if (p_d->pending != S3_NAV_NONE)
        {
            p_d->pending_us += now - p_d->obstacle.started_us;
        }
        (void)s3_navigation_set_mode(&p_d->navigation, S3_MODE_TRACK);
        p_d->state = S3_DRIVE_FOLLOW;
        break;
    default:
        break;
    }
    if (result == MOTION_OK &&
        (request.kind == S5_ACT_DRIVE || request.kind == S5_ACT_TURN))
    {
        p_d->command_id = p_d->port.command_id();
        p_d->phase_us = now;
        if (!p_d->command_id)
        {
            result = MOTION_ERR_BUSY;
        }
    }
    if (result != MOTION_OK)
    {
        p_d->obstacle.state = S5_FAULT;
        p_d->obstacle.fault = S5_MOTION_ERROR;
        halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
    }
    report_obstacle_status(p_d);
    return true;
}
#endif

bool s3_drive_process (s3_drive_t * p_d, const s3_sensor_frame_t * p_frame,
                       const motion_feedback_t * p_feedback)
{
    s3_odometry_sample_t odometry = {0};
    uint32_t now;
    bool running, done, reports_healthy;
    if ((p_d == 0) || !p_d->configured)
    {
        return false;
    }
    if (p_frame == 0)
    {
        halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
        return false;
    }
    now = p_frame->timestamp_us;
    p_d->feedback = p_feedback ? *p_feedback : (motion_feedback_t){0};
    running =
        (p_d->state >= S3_DRIVE_FOLLOW) && (p_d->state <= S3_DRIVE_REACQUIRE);
    reports_healthy = receive_reports(p_d, running, &done);
    if (running && p_d->feedback.report_loss_count != p_d->report_loss_count)
    {
        reports_healthy = false;
    }
    p_d->report_loss_count = p_d->feedback.report_loss_count;
    odometry = (s3_odometry_sample_t){p_d->feedback.centre_m,
                                      p_d->feedback.timestamp_us,
                                      p_d->feedback.epoch, p_d->feedback.valid};
    if (!s3_navigation_process(&p_d->navigation, p_frame, &odometry,
                               &p_d->output))
    {
        halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_BAD_INPUT);
        return false;
    }
    if (p_d->reset_result == S3_RESET_PENDING)
    {
        finish_reset(p_d, now);
        /* Still disarmed whether the reset completed, is waiting, or timed
         * out. Never let a fault reset fall through to a driving action. */
        return true;
    }
    running =
        (p_d->state >= S3_DRIVE_FOLLOW) && (p_d->state <= S3_DRIVE_REACQUIRE);
    if (p_d->output.stop_requested)
    {
        halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_SENSOR_FAULT);
        return true;
    }
    if (running &&
        (!feedback_fresh(p_d, now) || (p_d->feedback.state == MOTION_FAULT) ||
         (p_d->feedback.fault != MOTION_FAULT_NONE)))
    {
        halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_MOTION_FAULT);
        return true;
    }
    if (!running)
    {
        return true;
    }
    if (!reports_healthy ||
        (p_d->command_id != 0U && p_d->feedback.command_id != p_d->command_id))
    {
        halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_REPORT_FAULT);
        return true;
    }
    if (p_d->external_hold)
    {
        request_stop(p_d, now);
        p_d->port.heartbeat(false);
        return true;
    }
    p_d->port.heartbeat(true);
#if APP_S5
    if (obstacle_control(p_d, now, done))
    {
        return true;
    }
#endif
    if (p_d->state == S3_DRIVE_FOLLOW)
    {
        follow(p_d, now);
    }
    else if (p_d->state == S3_DRIVE_BRAKING)
    {
        if ((uint32_t)(now - p_d->phase_us) > S3_BRAKE_WAIT_US)
        {
            halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
        }
        else if (stopped_after_request(p_d))
        {
            p_d->waiting_stop = false;
            (void)s3_navigation_set_mode(&p_d->navigation, S3_MODE_SUPPRESSED);
            p_d->turn = p_d->pending;
            if (!start_manoeuvre(p_d))
            {
                halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
            }
            else
            {
                p_d->pending = S3_NAV_NONE;
                p_d->state = S3_DRIVE_MANOEUVRE;
                p_d->phase_us = now;
            }
        }
    }
    else if ((p_d->state == S3_DRIVE_MANOEUVRE) && p_d->searching)
    {
        if ((uint32_t)(now - p_d->phase_us) > S3_MANOEUVRE_WAIT_US)
        {
            halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
        }
        else
        {
            search_step(p_d, now);
        }
    }
    else if (p_d->state == S3_DRIVE_MANOEUVRE)
    {
        if (!p_d->advancing && p_d->output.line.valid &&
            p_d->output.line.touching)
        {
            p_d->touch_us = now;
            p_d->touched = true;
        }
        if ((uint32_t)(now - p_d->phase_us) > S3_MANOEUVRE_WAIT_US)
        {
            halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_TIMEOUT);
        }
        else if (done && p_d->advancing)
        {
            /* Axle now over the crossing: turn on the spot from here. */
            p_d->command_id = 0U;
            p_d->advancing = false;
            if (!request_turn(p_d))
            {
                halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
            }
            else
            {
                p_d->phase_us = now;
            }
        }
        else if (done)
        {
            const bool turned = (p_d->turn == S3_NAV_LEFT) ||
                                (p_d->turn == S3_NAV_RIGHT) ||
                                (p_d->turn == S3_NAV_UTURN);
            p_d->command_id = 0U;
            p_d->port.stop();
            if (turned && (p_d->options.turn_line_window_us != 0U) &&
                !(p_d->touched && ((uint32_t)(now - p_d->touch_us) <=
                                   p_d->options.turn_line_window_us)))
            {
                /* The sensors never crossed the new line near the end of the
                 * turn: they are beside it, not over it. Do not drive off:
                 * sweep for it if enabled, else stop. */
                if (p_d->options.search_deg > 0.0f)
                {
                    start_search(p_d, now); /* stays in MANOEUVRE */
                }
                else
                {
                    halt(p_d, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
                }
            }
            else
            {
                (void)s3_navigation_set_mode(&p_d->navigation, S3_MODE_TRACK);
                p_d->state = S3_DRIVE_REACQUIRE;
                p_d->phase_us = now;
            }
        }
        else if ((p_d->feedback.state != MOTION_TURN) &&
                 (p_d->feedback.state != MOTION_DRIVE) &&
                 (p_d->feedback.state != MOTION_COMPLETE))
        {
            halt(p_d, S3_DRIVE_FAULT, S3_DRIVE_REJECTED);
        }
    }
    else
    {
        p_d->port.stop();
        if ((uint32_t)(now - p_d->phase_us) > S3_REACQUIRE_WAIT_US)
        {
            halt(p_d, S3_DRIVE_HALTED, S3_DRIVE_LINE_LOST);
        }
        else if (p_d->output.steering.valid && stopped_after_request(p_d))
        {
            p_d->state = S3_DRIVE_FOLLOW;
        }
    }
    return true;
}

/* End of file. */

