/** @file avoid_obstacle.c
 * @brief Advance obstacle avoidance and line hand-back through motion
 * proposals.
 * @see app_program/subsystem5/README.md for the task and proposal path.
 * The selected unsigned bit fields fit in int before promotion.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 */
#include "avoid_obstacle.h"
#include "../app_types.h"

static avoid_obstacle_action_t
make_detour_action (avoid_obstacle_action_kind_t kind, int value,
                    unsigned speed);
static avoid_obstacle_action_t fail_detour (avoid_obstacle_t *     p_detour,
                                            avoid_obstacle_fault_t reason);
static unsigned pass_extra_mm (const avoid_obstacle_t * p_detour);
static unsigned diagonal_mm (unsigned offset_mm);

static unsigned diagonal_short (const avoid_obstacle_t * p_detour,
                                unsigned                 offset_mm);

static bool diagonal_clear (const avoid_obstacle_t * p_detour,
                            unsigned                 offset_mm);

static float64_t   bearing_deg (const profile_obstacle_t * p_scan_profile);
static inline bool observation_is_later (uint32_t received, uint32_t reference);

static int      round_clamp (float64_t deg, int max);
static unsigned return_lat (const avoid_obstacle_t * p_detour);
static int      swerve_target (const avoid_obstacle_t * p_detour);
static avoid_obstacle_action_t peek_result (avoid_obstacle_t * p_detour,
                                            const avoid_obstacle_input_t * p_in,
                                            unsigned elapsed);

/** @brief Initialize idle state from the supplied configuration. */
void avoid_obstacle_init (avoid_obstacle_t *  p_detour,
                          const s5_config_t * p_config)
{
    *p_detour        = (avoid_obstacle_t){0};
    p_detour->config = *p_config;
}

/** @brief Check the range, validity and wrap-safe age of one echo. */
bool avoid_obstacle_range_ready (
    const s5_codec_observation_t * p_range_observation, uint32_t now)
{
    return ((((p_range_observation &&
               (p_range_observation->b_is_range_valid)) &&
              (!p_range_observation->b_has_no_echo)) &&
             (p_range_observation->range_mm >= 30U)) &&
            (p_range_observation->range_mm <= 4000U)) &&
           ((now - p_range_observation->range_us) <= S5_RANGE_FRESH_US);
}

/** @brief Reset the active plan and enter the wait-for-stop phase. */
void avoid_obstacle_begin (avoid_obstacle_t * p_detour, uint32_t now)
{
    p_detour->state        = S5_WAIT_STOP;
    p_detour->fault        = S5_OK;
    p_detour->started_us   = now;
    p_detour->phase_us     = now;
    p_detour->leg          = 0;
    p_detour->b_has_issued = false;
    p_detour->plan         = (avoidance_planner_plan_t){0};
    line_recovery_reset(&p_detour->recovery);
    p_detour->pass          = S5_PASS_APPROACH;
    p_detour->searched_mm   = 0;
    p_detour->b_has_swapped = 0;
    p_detour->b_has_backed  = 0;
    p_detour->b_is_backing  = 0;
    p_detour->b_has_aimed   = 0;
    p_detour->b_is_aiming   = 0;
    avoid_obstacle_aim_set(p_detour, 0);
    p_detour->b_is_on_bend    = 0;
    p_detour->b_is_after_tape = 0;
    p_detour->b_has_extended  = 0;
    avoid_obstacle_relative_set(p_detour, 0);
    p_detour->side_turn           = 0;
    p_detour->side_turns          = 0;
    p_detour->b_was_toward_bypass = 0;
    p_detour->centre_tries        = 0;
    p_detour->lat_mm              = 0;
    p_detour->peek_lat_mm         = 0;
    p_detour->blocked_reads       = 0;
}

/** @brief Clear the pending issue flag and leave avoidance idle. */
void avoid_obstacle_abort (avoid_obstacle_t * p_detour)
{
    p_detour->state        = S5_IDLE;
    p_detour->b_has_issued = false;
    line_recovery_reset(&p_detour->recovery);
}

/** @brief Return the bounded straight-track aiming turn in degrees. */
int avoid_obstacle_aim_deg (
    const profile_obstacle_t * p_scan_profile) /* Use the straight-track aim. */
{

    if ((0 == p_scan_profile) || (!p_scan_profile->b_is_valid))
    {
        return 0;
    }
    /* Aiming assumes one obstacle on the line. A picture wider than 1.5 x the
     * largest obstacle is the obstacle plus something beside the track (a
     * wall): its middle is not the line, so do not turn toward it. */
    if (p_scan_profile->width_mm >
        (((unsigned int)S5_OBSTACLE_MAX_MM * 3U) / 2U))
    {
        return 0;
    }

    const int angle_deg = round_clamp(bearing_deg(p_scan_profile), 1000);

    if ((angle_deg < S5_AIM_MIN_DEG) && (angle_deg > -S5_AIM_MIN_DEG))
    {
        return 0;
    }

    return angle_deg > S5_AIM_MAX_DEG
               ? S5_AIM_MAX_DEG
               : (angle_deg < -S5_AIM_MAX_DEG ? -S5_AIM_MAX_DEG : angle_deg);
}

/** @brief Set the bend flag and apply the appropriate signed aiming limit. */
int avoid_obstacle_aim_deg_bend (const profile_obstacle_t * p_scan_profile,
                                 int turn_pct, bool * pb_is_bend)
{
    *pb_is_bend = (turn_pct >= S5_BEND_PCT) || (turn_pct <= -S5_BEND_PCT);

    if (!*pb_is_bend)
    {
        /* Use the unadjusted bearing on a straight. */
        return avoid_obstacle_aim_deg(p_scan_profile);
    }

    if (((0 == p_scan_profile) || (!p_scan_profile->b_is_valid)) ||
        (p_scan_profile->width_mm >
         (((unsigned int)S5_OBSTACLE_MAX_MM * 3U) / 2U)))
    {
        return 0;
    }

    const float64_t deg = bearing_deg(p_scan_profile);
    {
        const int angle_deg = round_clamp(deg, 1000);

        if ((angle_deg < S5_AIM_MIN_DEG) && (angle_deg > -S5_AIM_MIN_DEG))
        {
            return 0;
        }
    }
    /* Obstacle on the side the bend goes: track direction there is ~2x. */
    if ((deg > 0.0) != (turn_pct > 0))
    {
        return round_clamp(deg, S5_AIM_MAX_DEG);
    }

    return round_clamp(deg * (float64_t)S5_BEND_AIM_PCT / 100.0,
                       S5_BEND_AIM_MAX_DEG);
}

/** @brief Select a single action from the motion, echo and line snapshot.
 * @details The current detour state consumes completed motion and fresh
 * scan/line evidence in order. An unhealthy input or expired overall
 * deadline enters the existing fault path before dispatch. The returned action
 * is a proposal for the caller to submit; this function owns no motor I/O.
 * @see avoid_obstacle.h for ownership, units and fault handling.
 * @see app_program/subsystem5/README.md for task hand-off and scan geometry.
 */
avoid_obstacle_action_t
avoid_obstacle_step (avoid_obstacle_t *             p_detour,
                     const avoid_obstacle_input_t * p_in)
{

    if ((0 == p_detour) || (0 == p_in))
    {
        return make_detour_action(S5_ACT_STOP, 0, 0);
    }

    if (((S5_IDLE == (int)p_detour->state) ||
         (S5_COMPLETE == (int)p_detour->state)) ||
        (S5_FAULT == (int)p_detour->state))
    {
        return make_detour_action(S5_ACT_NONE, 0, 0);
    }

    if (!p_in->b_is_healthy)
    {
        return fail_detour(p_detour, S5_MOTION_ERROR);
    }

    if ((p_in->now_us - p_detour->started_us) > 60000000U)
    {
        return fail_detour(p_detour, S5_DEADLINE);
    }

    const unsigned elapsed = p_in->now_us - p_detour->phase_us;

    int sign;
    if (S5_WAIT_STOP == (int)p_detour->state)
    {
        if (elapsed > 1000000U)
        {
            return fail_detour(p_detour, S5_DEADLINE);
        }

        if (p_in->b_is_stopped)
        {
            p_detour->state    = S5_SCANNING;
            p_detour->phase_us = p_in->now_us;
            return make_detour_action(S5_ACT_SCAN, 0, 0);
        }
    }
    else if (S5_SCANNING == (int)p_detour->state)
    {
        if (elapsed > S5_SCAN_TIMEOUT_US)
        {
            return fail_detour(p_detour, S5_DEADLINE);
        }

        if (p_detour->b_is_aiming)
        {
            /* Face-the-obstacle turn: wait for it, then scan again square-on.
             */
            if (p_detour->b_has_issued)
            {
                if (p_in->b_is_motion_done)
                {
                    p_detour->b_has_issued = false;
                    p_detour->phase_us     = p_in->now_us;
                }

                return make_detour_action(S5_ACT_NONE, 0, 0);
            }

            if (!p_in->b_is_stopped)
            {
                return make_detour_action(S5_ACT_NONE, 0, 0);
            }

            p_detour->b_is_aiming = 0;
            p_detour->phase_us    = p_in->now_us;
            return make_detour_action(S5_ACT_SCAN, 0, 0);
        }

        if ((p_in->observation.b_is_scan_done) &&
            observation_is_later(p_in->observation.scan_us, p_detour->phase_us))
        {
            if (!p_in->observation.profile.b_is_valid)
            {
                return fail_detour(p_detour, S5_SCAN_FAILED);
            }

            if (!p_detour->b_has_aimed)
            {
                bool      b_is_bend;
                const int turn = avoid_obstacle_aim_deg_bend(
                    &p_in->observation.profile, p_in->turn_pct, &b_is_bend);
                p_detour->b_has_aimed  = 1;
                p_detour->b_is_on_bend = b_is_bend;

                if (turn)
                {
                    avoid_obstacle_aim_set(p_detour, turn);
                    p_detour->b_is_aiming  = 1;
                    p_detour->b_has_issued = true;
                    p_detour->phase_us     = p_in->now_us;
                    return make_detour_action(S5_ACT_TURN, turn,
                                              p_detour->config.speed_mms);
                }
            }

            if (!avoidance_planner_build(&p_in->observation.profile,
                                         &p_detour->config, &p_detour->plan))
            {
                return fail_detour(p_detour, S5_NO_PATH);
            }

            p_detour->state    = S5_MOVING;
            p_detour->phase_us = p_in->now_us;
        }
    }
    else if (S5_MOVING == (int)p_detour->state)
    {
        const bool b_is_straight =
            (((1U == p_detour->leg) || (3U == p_detour->leg)) ||
             (5U == p_detour->leg));

        if (elapsed > 20000000U)
        {
            return fail_detour(p_detour, S5_DEADLINE);
        }
        /* Servo faces sideways during a peek: no forward range until it ends.
         */
        if ((3U == p_detour->leg) && (S5_PASS_PEEKING == (int)p_detour->pass))
        {
            return peek_result(p_detour, p_in, elapsed);
        }
        /* Each straight leg requires a new forward reading after its turn.
         * During movement, loss of valid ranging or a close return brakes. */
        /* Reversing away from the face (leg 0) is exempt: the range only grows.
         */
        if (((p_detour->b_has_issued) && (!p_detour->b_is_backing)) &&
            ((!avoid_obstacle_range_ready(&p_in->observation, p_in->now_us)) ||
             (p_in->observation.range_mm <= p_detour->config.emergency_mm)))
        {
            return fail_detour(p_detour, S5_BAD_RANGE);
        }
        /* Line seen while passing (leg 3, only after the side reads clear,
         * so never beside the obstacle: a bend toward the bypass side) or on
         * the return diagonal (leg 5): stop, then the
         * IR-to-axle drive puts the axle on the line (as leg 5) before the
         * swerve. That tape is not kept: the swerve must see it again. */
        if ((((p_detour->b_has_issued) && (!p_detour->b_is_after_tape)) &&
             ((5U == p_detour->leg) ||
              ((3U == p_detour->leg) &&
               (S5_PASS_EXTRA == (int)p_detour->pass)))) &&
            (line_recovery_observe(&p_detour->recovery, p_in->now_us,
                                   p_in->b_is_healthy, p_in->b_has_tape,
                                   p_in->b_has_junction, true)))
        {
            if (3U == p_detour->leg)
            {
                /* Keep the heading from the passing leg. */
                p_detour->leg = 5U;
            }

            p_detour->b_has_issued    = false;
            p_detour->b_is_after_tape = 1;
            p_detour->phase_us        = p_in->now_us;
            return make_detour_action(S5_ACT_STOP, 0, 0);
        }

        if (((p_detour->b_has_issued) && (p_detour->leg >= 6U)) &&
            (line_recovery_observe(&p_detour->recovery, p_in->now_us,
                                   p_in->b_is_healthy, p_in->b_has_tape,
                                   p_in->b_has_junction, true)))
        {
            /* Swerve: stop the moment the line is confirmed under the IR. */
            /* clang-format off */
            p_detour->b_was_toward_bypass =
                swerve_target(p_detour) > avoid_obstacle_relative_get(p_detour)
                    ? 1U
                    : 0U;
            p_detour->centre_tries        = 0;
            p_detour->b_has_issued        = false;
            p_detour->state               = S5_FINISH;
            p_detour->phase_us            = p_in->now_us;
            /* clang-format on */
            return make_detour_action(S5_ACT_STOP, 0, 0);
        }

        if (p_detour->b_has_issued)
        {
            if (p_in->b_is_motion_done)
            {
                /* Leg-3 drives after a turn away also move the car sideways
                 * (rel degrees off the passing heading): count it, so the
                 * return diagonal still ends on the original line. */
                if (((3U == p_detour->leg) &&
                     (avoid_obstacle_relative_get(p_detour) > 0)) &&
                    ((int)p_detour->pass != S5_PASS_APPROACH))
                {
                    const float64_t value_mm =
                        (p_in->centre_m - p_detour->leg_start_m) * 1000.0 *
                        (float64_t)avoid_obstacle_relative_get(p_detour) *
                        0.017453;

                    if (value_mm > 0.0)
                    {
                        /* This conversion assumes bounded completed-leg
                         * odometry. Check the range in BARR-P52. */
                        const unsigned sum =
                            p_detour->lat_mm + (unsigned)(value_mm + 0.5);
                        p_detour->lat_mm = sum > 511U ? 511U : sum;
                    }
                }

                p_detour->b_has_issued = false;
                p_detour->phase_us     = p_in->now_us;
                /* Leg 3 ends only after the final clearance move. */
                if ((3U == p_detour->leg) &&
                    ((int)p_detour->pass != S5_PASS_EXTRA))
                {
                    p_detour->pass = S5_PASS_PEEK;
                    return make_detour_action(S5_ACT_NONE, 0, 0);
                }
                /* Reverse finished: now the leg-0 turn. */
                if ((0U == p_detour->leg) && (p_detour->b_is_backing))
                {
                    p_detour->b_is_backing = 0;
                    return make_detour_action(S5_ACT_NONE, 0, 0);
                }

                if (4U == p_detour->leg)
                {
                    avoid_obstacle_relative_set(p_detour, -S5_BYPASS_DEG);
                }

                if ((((5U == p_detour->leg) && (S5_RETURN_EXTRA_MM > 0U)) &&
                     (!p_detour->b_is_after_tape)) &&
                    (!p_detour->b_has_extended))
                {
                    /* After missing the tape, allow one extra diagonal. */
                    p_detour->b_has_extended = 1;
                    return make_detour_action(S5_ACT_NONE, 0, 0);
                }

                if (5U == p_detour->leg)
                {
                    /* Count tape observations only from the next sweep. */
                    line_recovery_reset(&p_detour->recovery);
                }

                if (p_detour->leg >= 6U)
                {
                    avoid_obstacle_relative_set(p_detour,
                                                swerve_target(p_detour));
                }

                if (8U == p_detour->leg)
                {
                    /* Both recovery sweeps failed and the car returned to its
                     * heading. */
                    return fail_detour(p_detour, S5_NO_LINE);
                }

                p_detour->leg++;
            }

            return make_detour_action(S5_ACT_NONE, 0, 0);
        }

        if (!p_in->b_is_stopped)
        {
            return make_detour_action(S5_ACT_NONE, 0, 0);
        }

        if ((!avoid_obstacle_range_ready(&p_in->observation, p_in->now_us)) ||
            (!observation_is_later(p_in->observation.range_us,
                                   p_detour->phase_us)))
        {
            return elapsed > 1000000U ? fail_detour(p_detour, S5_BAD_RANGE)
                                      : make_detour_action(S5_ACT_NONE, 0, 0);
        }

        if ((0U == p_detour->leg) && (!p_detour->b_has_backed))
        {
            /* Too close for the 45-degree out-leg: reverse the shortfall first.
             */
            unsigned back = diagonal_short(p_detour, p_detour->plan.offset_mm);
            p_detour->b_has_backed = 1;

            if ((((back > S5_BACKUP_MAX_MM) && (!p_detour->b_has_swapped)) &&
                 (p_detour->plan.b_has_alt_route)) &&
                (diagonal_short(p_detour, p_detour->plan.alt_offset_mm) <=
                 S5_BACKUP_MAX_MM))
            {
                /* Preferred side needs more room than reversing gives: still
                 * facing the obstacle, take the other side instead. */
                const unsigned other = p_detour->plan.alt_offset_mm;
                /* clang-format off */
                p_detour->b_has_swapped      = 1;
                p_detour->plan.alt_offset_mm = p_detour->plan.offset_mm;
                p_detour->plan.offset_mm     = other;
                p_detour->plan.route         =
                    S5_ROUTE_LEFT == (int)p_detour->plan.route ? S5_ROUTE_RIGHT
                                                               : S5_ROUTE_LEFT;
                back                         =
                    diagonal_short(p_detour, p_detour->plan.offset_mm);
                /* clang-format on */
            }

            if (back)
            {
                if (back > S5_BACKUP_MAX_MM)
                {
                    return fail_detour(p_detour, S5_NO_PATH);
                }

                p_detour->plan.closest_mm += back;
                p_detour->plan.forward_mm += back;
                p_detour->b_is_backing     = 1;
                p_detour->b_has_issued     = true;
                return make_detour_action(S5_ACT_DRIVE, -(int)back,
                                          p_detour->config.speed_mms);
            }
        }

        if (p_in->observation.range_mm <= p_detour->config.emergency_mm)
        {
            return fail_detour(p_detour, S5_NO_PATH);
        }

        sign = S5_ROUTE_LEFT == (int)p_detour->plan.route ? 1 : -1;

        if ((3U == p_detour->leg) && (S5_PASS_PEEK == (int)p_detour->pass))
        {
            p_detour->pass     = S5_PASS_PEEKING;
            p_detour->phase_us = p_in->now_us;
            return make_detour_action(
                S5_ACT_PEEK,
                (int)(S5_ROUTE_LEFT == (int)p_detour->plan.route
                          ? 90U - S5_PEEK_DEG
                          : 90U + S5_PEEK_DEG),
                0);
        }

        if (((3U == p_detour->leg) && (S5_PASS_STEP == (int)p_detour->pass)) &&
            (p_detour->side_turn))
        {
            /* Turn away from the obstacle side, then peek again. rel keeps the
             * heading vs the one at the obstacle, so the way back still aims
             * at the original line. */
            const int deg          = (int)p_detour->side_turn;
            p_detour->side_turn    = 0;
            p_detour->b_has_issued = true;
            p_detour->pass         = S5_PASS_APPROACH;
            avoid_obstacle_relative_set(
                p_detour, avoid_obstacle_relative_get(p_detour) + deg);
            return make_detour_action(S5_ACT_TURN, sign * deg,
                                      p_detour->config.speed_mms);
        }

        if (b_is_straight)
        {
            unsigned value_mm =
                diagonal_mm(5U == p_detour->leg ? return_lat(p_detour)
                                                : p_detour->plan.offset_mm);

            const bool b_uses_extended_return =
                (5U == p_detour->leg) &&
                ((p_detour->b_is_after_tape) || (p_detour->b_has_extended));
            value_mm = b_uses_extended_return
                           ? (p_detour->b_is_after_tape ? S5_IR_AXLE_MM
                                                        : S5_RETURN_EXTRA_MM)
                           : value_mm;

            if (3U == p_detour->leg)
            {
                /* The diagonal out-leg already moved the car offset forward. */
                const unsigned forward = p_detour->plan.forward_mm;
                const unsigned offset  = p_detour->plan.offset_mm;
                const unsigned approach =
                    forward > offset ? forward - offset : 0U;

                if ((S5_PASS_APPROACH == (int)p_detour->pass) &&
                    (approach < S5_PEEK_STEP_MM))
                {
                    p_detour->pass = S5_PASS_PEEK;
                    return make_detour_action(S5_ACT_NONE, 0,
                                              0); /* The car is at the face. */
                }

                value_mm = S5_PASS_APPROACH == (int)p_detour->pass
                               ? approach
                               : (S5_PASS_STEP == (int)p_detour->pass
                                      ? S5_PEEK_STEP_MM
                                      : pass_extra_mm(p_detour));
            }

            if ((!avoid_obstacle_range_ready(&p_in->observation,
                                             p_in->now_us)) ||
                (!observation_is_later(p_in->observation.range_us,
                                       p_detour->phase_us)))
            {
                return elapsed > 1000000U
                           ? fail_detour(p_detour, S5_BAD_RANGE)
                           : make_detour_action(S5_ACT_NONE, 0, 0);
            }

            {
                /* Leg 1 faces the diagonal path itself: it must be clear for
                 * the whole diagonal + margin (measured), and the forward sweep
                 * of the 45-degree leg must not reach the obstacle face. */
                const unsigned need =
                    ((1U == p_detour->leg) &&
                     ((value_mm + p_detour->config.margin_mm) >
                      S5_LEG_CLEAR_MM))
                        ? (value_mm + p_detour->config.margin_mm)
                        : S5_LEG_CLEAR_MM;

                const bool b_is_blocked =
                    (p_in->observation.range_mm <= need) ||
                    ((1U == p_detour->leg) &&
                     (!diagonal_clear(p_detour, p_detour->plan.offset_mm)));

                if ((b_is_blocked && (3U == p_detour->leg)) &&
                    (p_detour->side_turns < 3U))
                {
                    /* Passing but the obstacle's side shows ahead: the car is
                     * converging on it. Turn away, then peek again. */
                    p_detour->side_turns++;
                    p_detour->b_has_issued = true;
                    p_detour->pass         = S5_PASS_APPROACH;
                    avoid_obstacle_relative_set(
                        p_detour, avoid_obstacle_relative_get(p_detour) +
                                      (int)S5_SIDE_TURN_MAX_DEG);
                    return make_detour_action(S5_ACT_TURN,
                                              sign * (int)S5_SIDE_TURN_MAX_DEG,
                                              p_detour->config.speed_mms);
                }

                if ((b_is_blocked && (1U == p_detour->leg)) &&
                    ((p_detour->blocked_reads + 1U) < S5_BLOCK_CONFIRM))
                {
                    /* Wait for another fresh reading before believing it. */
                    p_detour->blocked_reads++;
                    p_detour->phase_us = p_in->now_us;
                    return make_detour_action(S5_ACT_NONE, 0, 0);
                }

                /* A confirmed blockage or clear range ends the count. */
                p_detour->blocked_reads = 0;

                /* Try the other diagonal once after confirming blockage. */
                if ((b_is_blocked &&
                     ((1U == p_detour->leg) && (!p_detour->b_has_swapped))) &&
                    (p_detour->plan.b_has_alt_route))
                {
                    unsigned other = p_detour->plan.alt_offset_mm;
                    /* clang-format off */
                    p_detour->b_has_swapped      = 1;
                    p_detour->plan.alt_offset_mm = p_detour->plan.offset_mm;
                    p_detour->plan.offset_mm     = other;
                    p_detour->plan.route         =
                        S5_ROUTE_LEFT == (int)p_detour->plan.route
                            ? S5_ROUTE_RIGHT
                            : S5_ROUTE_LEFT;
                    p_detour->leg                = 0;
                    p_detour->b_has_issued       = true;
                    /* clang-format on */
                    return make_detour_action(S5_ACT_TURN,
                                              S5_ROUTE_LEFT ==
                                                      (int)p_detour->plan.route
                                                  ? 2 * S5_BYPASS_DEG
                                                  : -2 * S5_BYPASS_DEG,
                                              p_detour->config.speed_mms);
                }

                if (b_is_blocked)
                {
                    return fail_detour(p_detour, S5_NO_PATH);
                }
            }
            p_detour->b_has_issued = true;
            p_detour->leg_start_m  = p_in->centre_m;

            if ((3U == p_detour->leg) && (S5_PASS_STEP == (int)p_detour->pass))
            {
                /* A side peek bounds a normal step by the 12-bit depth
                 * limit. The stored counter retains the low 16 sum bits. */
                p_detour->searched_mm =
                    (uint16_t)(p_detour->searched_mm + value_mm);
            }

            return make_detour_action(S5_ACT_DRIVE, (int)value_mm,
                                      p_detour->config.speed_mms);
        }

        p_detour->b_has_issued = true;

        if (0U == p_detour->leg)
        {
            return make_detour_action(S5_ACT_TURN, sign * S5_BYPASS_DEG,
                                      p_detour->config.speed_mms);
        }

        if (2U == p_detour->leg)
        {
            return make_detour_action(S5_ACT_TURN, -sign * S5_BYPASS_DEG,
                                      p_detour->config.speed_mms);
        }
        /* Turn in to 45 degrees off the passing heading (less any turn away).
         */
        if (4U == p_detour->leg)
        {
            return make_detour_action(
                S5_ACT_TURN,
                -sign * (S5_BYPASS_DEG + avoid_obstacle_relative_get(p_detour)),
                p_detour->config.speed_mms);
        }
        /* Swerve (legs 6..8), each turn stopped as soon as the line is seen:
         * toward the bypass side up to S5_SWERVE_OUT_DEG past the heading at
         * the obstacle, then the other way up to S5_SWERVE_IN_DEG past it,
         * then back to it. Never more than 90 deg either side, so the car
         * cannot pick up the line behind it and follow it backwards. */
        return make_detour_action(S5_ACT_TURN,
                                  sign *
                                      (swerve_target(p_detour) -
                                       avoid_obstacle_relative_get(p_detour)),
                                  p_detour->config.speed_mms);
    }
    else if (S5_FINISH == (int)p_detour->state)
    {
        /* Hand back only with the tape under the IR now. Braking can carry
         * the IR past the edge it stopped on: then turn back slowly (the way
         * it came, then the other way), stopping on the tape. */
        const int side = S5_ROUTE_LEFT == (int)p_detour->plan.route ? 1 : -1;
        const int dir  = p_detour->b_was_toward_bypass ? side : -side;

        if (elapsed > 3000000U)
        {
            return fail_detour(p_detour, S5_NO_LINE);
        }

        if (p_detour->b_has_issued)
        {
            if ((p_in->b_has_tape) && (!p_in->b_has_junction))
            {
                p_detour->b_has_issued = false;
                p_detour->phase_us     = p_in->now_us;
                return make_detour_action(S5_ACT_STOP, 0, 0);
            }

            if (p_in->b_is_motion_done)
            {
                p_detour->b_has_issued = false;
                p_detour->phase_us     = p_in->now_us;
            }

            return make_detour_action(S5_ACT_NONE, 0, 0);
        }

        if (!p_in->b_is_stopped)
        {
            return make_detour_action(S5_ACT_NONE, 0, 0);
        }

        if (((p_in->b_has_tape) && (p_in->b_is_line_ready)) &&
            (!p_in->b_has_junction))
        {
            p_detour->state = S5_COMPLETE;
            return make_detour_action(S5_ACT_RESUME, 0, 0);
        }

        if (p_detour->centre_tries >= 2U)
        {
            return fail_detour(p_detour, S5_NO_LINE);
        }

        p_detour->b_has_issued = true;
        p_detour->phase_us     = p_in->now_us;
        p_detour->centre_tries++;
        return make_detour_action(S5_ACT_TURN,
                                  (1U == p_detour->centre_tries ? -dir : dir) *
                                      (1U == p_detour->centre_tries
                                           ? S5_CENTRE_DEG
                                           : 2 * S5_CENTRE_DEG),
                                  S5_CENTRE_SPEED_MMS);
    }
    else
    {
        /* No new command is due for this state. */
    }

    return make_detour_action(S5_ACT_NONE, 0, 0);
}

#ifdef S5_HOST_TEST
/** @brief Expose the production clearance calculation to the host fixture.
 * @param p_detour Nonnull test state with a valid plan and configuration.
 * @return Additional forward travel in millimetres.
 */
unsigned pass_extra_test (const avoid_obstacle_t * p_detour)
{
    return pass_extra_mm(p_detour);
}
#endif

/** @brief Build a proposal for the caller to dispatch.
 * @param kind Requested operation.
 * @param value Signed command value in mm or degrees, according to kind.
 * @param speed Speed in mm/s. Callers supply values that fit the action fields.
 * @return The command with a 16-bit value and unsigned 16-bit speed.
 */
static avoid_obstacle_action_t
make_detour_action (avoid_obstacle_action_kind_t kind, int value,
                    unsigned speed)
{
    return (avoid_obstacle_action_t){kind, (int16_t)value, (uint16_t)speed};
}

/** @brief Latch an avoidance fault and request a stop.
 * @param p_detour Nonnull controller state owned by the caller.
 * @param reason Fault to retain until the next bypass begins.
 * @return A stop proposal with zero value and speed.
 */
static avoid_obstacle_action_t fail_detour (avoid_obstacle_t *     p_detour,
                                            avoid_obstacle_fault_t reason)
{
    p_detour->state = S5_FAULT;
    p_detour->fault = reason;
    return make_detour_action(S5_ACT_STOP, 0, 0);
}

/** @brief Calculate forward travel after a clear side scan.
 * @param p_detour Nonnull controller with the measured offset and configured
 * margin.
 * @return Additional clearance in millimetres for the selected geometry.
 * @details On a ray S5_PEEK_DEG
 * from straight ahead, a return at lateral distance d lies d*tan(90-60) =
 * 0.577*d ahead of the sensor, so "clear" means the obstacle's rear edge is at
 * most 0.577*d ahead of the sensor. Drive until the car's BACK (sonar-to-axle
 * + axle-to-rear behind the sensor) is past it by the margin. d is the lateral
 * gap between the sensor line and the obstacle edge, not the whole offset. */
static unsigned pass_extra_mm (const avoid_obstacle_t * p_detour)
{
    const unsigned pad = s5_outward_gap_mm(&p_detour->config);
    const unsigned edge =
        p_detour->plan.offset_mm > pad ? p_detour->plan.offset_mm - pad : 0U;
    const unsigned gap =
        p_detour->plan.offset_mm > edge ? p_detour->plan.offset_mm - edge : 0U;
#if S5_PASS_EXTRA_V2
    /* 17 Nov 2026. The return is a 45-degree turn on the spot, so the car does
     * not need its whole body past the rear edge first:
     * - "clear" means nothing in the peek cone (60 +/- 15 deg off the heading),
     *   so the side face ends before 0.268 x gap ahead of the sensor
     *   (cot 75 deg = 0.268);
     * - after turning back 45 degrees the car's line-side edge runs along
     *   x + y = -1.414 x half width; the rear corner of the obstacle (at the
     *   pass clearance half width + margin to the side) stays outside it by
     *   the margin once the axle is 0.414 x (half width + margin) past it.
     * Example, half width 63, margin 30: 75 + 25 + 39 = 139 mm (was 214). */
    return S5_SONAR_AXLE_MM + (gap * 268U + pad * 414U + 999U) / 1000U;
#else
    return (((gap * 577U) + 999U) / 1000U) + S5_SONAR_AXLE_MM +
           S5_REAR_AXLE_MM + p_detour->config.margin_mm;
#endif
}

/** @brief Convert sideways displacement into 45-degree diagonal travel.
 * @param offset_mm Planned sideways displacement in millimetres.
 * @return Diagonal distance in mm using the planner's rounded-up scale factor.
 */
static unsigned diagonal_mm (unsigned offset_mm)
{
    return ((offset_mm * 1414U) + 999U) / 1000U;
}

/** @brief Calculate missing front clearance before an outbound diagonal.
 * @param p_detour Nonnull state containing the chassis dimensions and front
 * range.
 * @param offset_mm Planned lateral offset in mm.
 * @return Additional clearance required in mm, or zero when sufficient.
 * @details Driving out at 45 degrees moves the car forward, so its inner front
 * corner sweeps toward the obstacle face. With the face D = closest + sonar-
 * to-axle ahead of the axle and the near obstacle edge e = offset - (half
 * width + margin) to the side, that corner reaches the face plane d = D -
 * 1.414*half_width to the side: it must be past e + margin by then. */
static unsigned diagonal_short (const avoid_obstacle_t * p_detour,
                                unsigned                 offset_mm)
{
    const unsigned pad  = s5_outward_gap_mm(&p_detour->config);
    const unsigned edge = offset_mm > pad ? offset_mm - pad : 0U;
    const unsigned face = p_detour->plan.closest_mm + S5_SONAR_AXLE_MM;
    const unsigned corner =
        ((p_detour->config.half_width_mm * 1414U) + 999U) / 1000U;
    const unsigned need = corner + edge + p_detour->config.margin_mm;
    return need > face ? need - face : 0U; /* Return the distance shortfall. */
}

/** @brief Test whether the front corner clears the measured obstacle face.
 * @param p_detour Nonnull state with the obstacle profile and chassis
 * dimensions.
 * @param offset_mm Proposed lateral offset in millimetres.
 * @return True when no additional front clearance is required.
 */
static bool diagonal_clear (const avoid_obstacle_t * p_detour,
                            unsigned                 offset_mm)
{
    return diagonal_short(p_detour, offset_mm) == 0U;
}

/** @brief Estimate the bearing from the axle to the obstacle centre.
 * @param p_scan_profile Nonnull profile with signed lateral edges and front
 * range in mm.
 * @return Degrees, positive left. Beyond the approximation interval return
 * signed 90 degrees for downstream clamping. Nonpositive depth returns zero.
 */
static float64_t bearing_deg (const profile_obstacle_t * p_scan_profile)
{
    const float64_t lateral_mm = ((float64_t)p_scan_profile->left_edge_mm +
                                  (float64_t)p_scan_profile->right_edge_mm) /
                                 2.0; /* Positive means left. */
    const float64_t forward_mm =
        (float64_t)p_scan_profile->closest_mm +
        (float64_t)S5_SONAR_AXLE_MM; /* Measure from axle to face. */
    if (forward_mm <= 0.0)
    {
        return 0.0;
    }

    const float64_t slope = lateral_mm / forward_mm;

    if ((slope > 1.0) || (slope < -1.0))
    {
        /* The downstream limit clamps bearings beyond 45 degrees. */
        return slope > 0.0 ? 90.0 : -90.0;
    }
    /* This polynomial matches equation (7) of Rajan et al., "Efficient
     * approximations for the arctangent function" (2006), DOI
     * 10.1109/MSP.2006.1628884. The range check above limits |slope| to 1. */
    return ((0.7853982 * slope) +
            (0.273 * slope * (1.0 - (slope < 0.0 ? -slope : slope)))) *
           57.29578;
}

/** @brief Compare timestamps strictly within the forward half range.
 * Both arguments are microseconds from the same wrapping 32-bit clock.
 * The exactly half-range case, like equality, is not a fresh sample.
 */
static inline bool observation_is_later (uint32_t received, uint32_t reference)
{
    const uint32_t elapsed = received - reference;
    return (elapsed != 0U) && (elapsed <= (UINT32_MAX / 2U));
}

/** @brief Round a finite angle to the nearest degree and clamp it.
 * @param deg Angle in degrees from the bounded bearing calculation.
 * @param max Nonnegative maximum absolute angle.
 * @pre The angle fits int after adding or subtracting half a degree.
 * The bearing is within 90 degrees before optional scaling. The default
 * bend multiplier is one. An override must preserve the cast range.
 * @return The integer angle in [-max, max], rounding half degrees away from 0.
 */
static int round_clamp (float64_t deg, int max)
{
    const int angle_deg = (int)(deg + (deg >= 0.0 ? 0.5 : -0.5));
    return angle_deg > max ? max : (angle_deg < -max ? -max : angle_deg);
}

/** @brief Select the lateral distance used to return to the line.
 * @param p_detour Nonnull controller state with return plan and latest side
 * peek.
 * @return The greater of the measured lateral displacement and the sum of
 * planned offset and accumulated outward travel, in millimetres. If the
 * return leg runs long, line detection can still stop the vehicle on the tape.
 */
static unsigned return_lat (const avoid_obstacle_t * p_detour)
{
    const unsigned planned = p_detour->plan.offset_mm + p_detour->lat_mm;
    return p_detour->peek_lat_mm > planned ? p_detour->peek_lat_mm : planned;
}

/** @brief Select the target heading for the current return swerve.
 * @param p_detour Nonnull controller state including leg and tail flags.
 * @return Signed relative heading in degrees for legs 6 and 7, otherwise zero.
 * @note Positive angles point toward the bypass side. Once the tape has
 * been seen, search that side first. Otherwise search toward the line first.
 */
static int swerve_target (const avoid_obstacle_t * p_detour)
{
    const int out      = S5_SWERVE_OUT_DEG;
    const int value_in = -S5_SWERVE_IN_DEG;

    if (6U == p_detour->leg)
    {
        return p_detour->b_is_after_tape ? out : value_in;
    }

    if (7U == p_detour->leg)
    {
        return p_detour->b_is_after_tape ? value_in : out;
    }

    return 0;
}

/** @brief Process a side scan while the stopped vehicle passes the obstacle.
 * @param p_detour Nonnull controller state to update with clearance and search
 * data.
 * @param p_in Current observations and microsecond clock.
 * @param elapsed Unsigned microseconds since the scan phase began.
 * @return No action while waiting or after selecting the next step. Timeout,
 * invalid profile or exhausted search distance latches a fault and stops.
 */
static avoid_obstacle_action_t peek_result (avoid_obstacle_t * p_detour,
                                            const avoid_obstacle_input_t * p_in,
                                            unsigned elapsed)
{
    const profile_obstacle_t * p_profile = &p_in->observation.profile;

    if (elapsed > S5_SCAN_TIMEOUT_US)
    {
        return fail_detour(p_detour, S5_DEADLINE);
    }

    if ((!p_in->observation.b_is_scan_done) ||
        (!observation_is_later(p_in->observation.scan_us, p_detour->phase_us)))
    {
        return make_detour_action(S5_ACT_NONE, 0, 0);
    }

    if (!p_profile->b_is_valid)
    {
        return fail_detour(p_detour, S5_SCAN_FAILED);
    }
    /* Obstacle on the car's right after a LEFT bypass, so look right (30 deg).
     */
    const unsigned side = S5_ROUTE_LEFT == (int)p_detour->plan.route
                              ? p_profile->right_clear_mm
                              : p_profile->left_clear_mm;
    /* Anything nearer than the original line (+margin) along the 60 deg ray:
     * range = lateral / sin(60 deg), 1/sin(60) = 1.155. The two
     * nonnegative 12-bit distances sum exactly before scaling. */
    const unsigned limit =
        (((unsigned)(p_detour->plan.offset_mm + p_detour->config.margin_mm) *
          1155U) +
         999U) /
        1000U;
    p_detour->phase_us = p_in->now_us;

    if (side <= limit)
    {
        /* Still beside it: is the side as far as planned (half width +
         * margin from the sensor line)? If not, turn away, then peek again. */
        const unsigned pad =
            p_detour->config.half_width_mm + p_detour->config.margin_mm;
        const unsigned gap         = (side * 966U) / 1000U;
        const unsigned planned_gap = s5_outward_gap_mm(&p_detour->config);
        const unsigned offset_mm   = p_detour->plan.offset_mm;
        const unsigned edge =
            (offset_mm > planned_gap) ? (offset_mm - planned_gap) : 0U;
        /* Measured distance from the original line now (obstacle edge + gap):
         * catches a car that drifted away while passing (left a bend crooked).
         */
        p_detour->peek_lat_mm = (edge + gap) > 4095U ? 4095U : edge + gap;

        if (((gap + S5_SIDE_TOL_MM) < pad) && (p_detour->side_turns < 3U))
        {
            const unsigned deg = (pad - gap + 1U) / 2U;
            p_detour->side_turn =
                deg > S5_SIDE_TURN_MAX_DEG ? S5_SIDE_TURN_MAX_DEG : deg;
            p_detour->side_turns++;
        }

        if ((p_detour->searched_mm + S5_PEEK_STEP_MM) >
            p_detour->config.max_depth_mm)
        {
            return fail_detour(p_detour, S5_NO_PATH);
        }

        p_detour->pass = S5_PASS_STEP;
    }
    else
    {
        p_detour->pass = S5_PASS_EXTRA;
    }

    return make_detour_action(S5_ACT_NONE, 0, 0);
}

/* End of file. */

