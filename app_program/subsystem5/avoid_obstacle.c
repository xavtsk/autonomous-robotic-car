#include "avoid_obstacle.h"
#include "../app_types.h"

static s5_action_t action (s5_action_kind_t kind, int value, unsigned speed)
{
    return (s5_action_t){kind, (int16_t)value, (uint16_t)speed};
}

static s5_action_t fail (s5_avoidance_t * p_a, s5_fault_t reason)
{
    p_a->state = S5_FAULT;
    p_a->fault = reason;
    return action(S5_ACT_STOP, 0, 0);
}

void s5_avoidance_init (s5_avoidance_t * p_a, const s5_config_t * p_c)
{
    *p_a = (s5_avoidance_t){0};
    p_a->config = *p_c;
}

bool s5_range_ready (const s5_observation_t * p_o, uint32_t now)
{
    return p_o && p_o->range_valid && !p_o->no_echo && p_o->range_mm >= 30U &&
           p_o->range_mm <= 4000U &&
           (uint32_t)(now - p_o->range_us) <= S5_RANGE_FRESH_US;
}

void s5_avoidance_begin (s5_avoidance_t * p_a, uint32_t now)
{
    p_a->state = S5_WAIT_STOP;
    p_a->fault = S5_OK;
    p_a->phase_us = p_a->started_us = now;
    p_a->leg = 0;
    p_a->issued = false;
    p_a->plan = (s5_plan_t){0};
    s5_recovery_reset(&p_a->recovery);
    p_a->pass = S5_PASS_APPROACH;
    p_a->searched_mm = 0;
    p_a->swapped = 0;
    p_a->backed = 0;
    p_a->backing = 0;
    p_a->aimed = 0;
    p_a->aiming = 0;
    s5_aim_set(p_a, 0);
    p_a->bend = 0;
    p_a->tail = 0;
    p_a->extended = 0;
    s5_rel_set(p_a, 0);
    p_a->side_turn = 0;
    p_a->side_turns = 0;
    p_a->centre_dir = 0;
    p_a->centre_tries = 0;
    p_a->lat_mm = 0;
    p_a->peek_lat_mm = 0;
    p_a->blocked_reads = 0;
}

void s5_avoidance_abort (s5_avoidance_t * p_a)
{
    p_a->state = S5_IDLE;
    p_a->issued = false;
    s5_recovery_reset(&p_a->recovery);
}

/* Distance still needed after the side peek reads clear. On a ray S5_PEEK_DEG
 * from straight ahead, a return at lateral distance d lies d*tan(90-60) =
 * 0.577*d ahead of the sensor, so "clear" means the obstacle's rear edge is at
 * most 0.577*d ahead of the sensor. Drive until the car's BACK (sonar-to-axle
 * + axle-to-rear behind the sensor) is past it by the margin. d is the lateral
 * gap between the sensor line and the obstacle edge, not the whole offset. */
static unsigned pass_extra_mm (const s5_avoidance_t * p_a)
{
    const unsigned pad = p_a->config.half_width_mm + p_a->config.margin_mm;
    const unsigned edge =
        p_a->plan.offset_mm > pad ? p_a->plan.offset_mm - pad : 0U;
    const unsigned gap =
        p_a->plan.offset_mm > edge ? p_a->plan.offset_mm - edge : 0U;
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
    return (gap * 577U + 999U) / 1000U + S5_SONAR_AXLE_MM + S5_REAR_AXLE_MM +
           p_a->config.margin_mm;
#endif
}

/* 45-degree legs: travel along the diagonal for a sideways offset d. */
static unsigned diagonal_mm (unsigned offset_mm)
{
    return (offset_mm * 1414U + 999U) / 1000U;
}

/* Driving out at 45 degrees also moves the car forward, so its inner front
 * corner sweeps toward the obstacle face. With the face D = closest + sonar-
 * to-axle ahead of the axle and the near obstacle edge e = offset - (half
 * width + margin) to the side, that corner reaches the face plane d = D -
 * 1.414*half_width to the side: it must be past e + margin by then. */
static unsigned diagonal_short (const s5_avoidance_t * p_a, unsigned offset_mm)
{
    const unsigned pad = p_a->config.half_width_mm + p_a->config.margin_mm;
    const unsigned edge = offset_mm > pad ? offset_mm - pad : 0U;
    const unsigned face = p_a->plan.closest_mm + S5_SONAR_AXLE_MM;
    const unsigned corner = (p_a->config.half_width_mm * 1414U + 999U) / 1000U;
    const unsigned need = corner + edge + p_a->config.margin_mm;
    return need > face ? need - face
                       : 0U; /* how much nearer the face is than safe */
}

static bool diagonal_clear (const s5_avoidance_t * p_a, unsigned offset_mm)
{
    return diagonal_short(p_a, offset_mm) == 0U;
}

/* Bearing of the obstacle centre from the wheel axle, degrees, + = left. */
static float64_t bearing_deg (const s5_profile_t * p_p)
{
    float64_t value_x, value_y, value_t, value_t2 = 0.0;
    value_x = ((float64_t)p_p->left_edge_mm + (float64_t)p_p->right_edge_mm) /
              2.0; /* + = left */
    value_y =
        (float64_t)p_p->closest_mm + (float64_t)S5_SONAR_AXLE_MM; /* axle to face */
    if (value_y <= 0.0)
    {
        return 0.0;
    }
    value_t = value_x / value_y;
    if (value_t > 1.0 || value_t < -1.0)
    {
        return value_t > 0.0 ? 90.0 : -90.0;
    } /* beyond 45: always capped */
    (void)value_t2;
    /* atan(t) for |t|<=1 without libm: pi/4*t + 0.273*t*(1-|t|), < 0.3 deg
     * error. */
    return (0.7853982 * value_t +
            0.273 * value_t * (1.0 - (value_t < 0.0 ? -value_t : value_t))) *
           57.29578;
}

static int round_clamp (float64_t deg, int max)
{
    int value_d = (int)(deg + (deg >= 0.0 ? 0.5 : -0.5));
    return value_d > max ? max : (value_d < -max ? -max : value_d);
}

int s5_aim_deg (
    const s5_profile_t * p_p) /* straight: as in the 17:31 version */
{
    int value_d;
    if (!p_p || !p_p->valid)
    {
        return 0;
    }
    /* Aiming assumes one obstacle on the line. A picture wider than 1.5 x the
     * largest obstacle is the obstacle plus something beside the track (a
     * wall): its middle is not the line, so do not turn toward it. */
    if (p_p->width_mm > (S5_OBSTACLE_MAX_MM * 3U) / 2U)
    {
        return 0;
    }
    value_d = round_clamp(bearing_deg(p_p), 1000);
    if (value_d < S5_AIM_MIN_DEG && value_d > -S5_AIM_MIN_DEG)
    {
        return 0;
    }
    return value_d > S5_AIM_MAX_DEG
               ? S5_AIM_MAX_DEG
               : (value_d < -S5_AIM_MAX_DEG ? -S5_AIM_MAX_DEG : value_d);
}

int s5_aim_deg_bend (const s5_profile_t * p_p, int turn, bool * p_bend)
{
    float64_t deg;
    *p_bend = turn >= S5_BEND_PCT || turn <= -S5_BEND_PCT;
    if (!*p_bend)
    {
        return s5_aim_deg(p_p);
    } /* straight: unchanged */
    if (!p_p || !p_p->valid || p_p->width_mm > (S5_OBSTACLE_MAX_MM * 3U) / 2U)
    {
        return 0;
    }
    deg = bearing_deg(p_p);
    {
        int value_d = round_clamp(deg, 1000);
        if (value_d < S5_AIM_MIN_DEG && value_d > -S5_AIM_MIN_DEG)
        {
            return 0;
        }
    }
    /* Obstacle on the side the bend goes: track direction there is ~2x. */
    if ((deg > 0.0) != (turn > 0))
    {
        return round_clamp(deg, S5_AIM_MAX_DEG);
    }
    return round_clamp(deg * (float64_t)S5_BEND_AIM_PCT / 100.0,
                       S5_BEND_AIM_MAX_DEG);
}

/* Sideways distance to cover on the way back: planned offset plus any turn
 * away while passing, or what the last side peek measured if that is more
 * (drifted away). Too long is safe: the line seen on the way stops the car. */
static unsigned return_lat (const s5_avoidance_t * p_a)
{
    const unsigned planned = p_a->plan.offset_mm + p_a->lat_mm;
    return p_a->peek_lat_mm > planned ? p_a->peek_lat_mm : planned;
}

/* Swerve heading targets (deg vs the heading at the obstacle, + = bypass
 * side), legs 6, 7, 8. Line already seen (tail driven): look toward the
 * bypass side first, where it continues. Not seen at all: it can only be on
 * the line side still (a bend away), so look there first. */
static int swerve_target (const s5_avoidance_t * p_a)
{
    const int out = S5_SWERVE_OUT_DEG, value_in = -S5_SWERVE_IN_DEG;
    if (p_a->leg == 6U)
    {
        return p_a->tail ? out : value_in;
    }
    if (p_a->leg == 7U)
    {
        return p_a->tail ? value_in : out;
    }
    return 0;
}
#ifdef S5_HOST_TEST
unsigned pass_extra_test (const s5_avoidance_t * a)
{
    return pass_extra_mm(a);
}
#endif
static s5_action_t peek_result (s5_avoidance_t * p_a, const s5_input_t * p_in,
                                unsigned elapsed)
{
    const s5_profile_t * p_p = &p_in->observation.profile;
    unsigned side, limit;
    if (elapsed > S5_SCAN_TIMEOUT_US)
    {
        return fail(p_a, S5_DEADLINE);
    }
    if (!p_in->observation.scan_done ||
        (int32_t)(p_in->observation.scan_us - p_a->phase_us) <= 0)
    {
        return action(S5_ACT_NONE, 0, 0);
    }
    if (!p_p->valid)
    {
        return fail(p_a, S5_SCAN_FAILED);
    }
    /* Obstacle on the car's right after a LEFT bypass, so look right (30 deg).
     */
    side = p_a->plan.route == S5_ROUTE_LEFT ? p_p->right_clear_mm
                                            : p_p->left_clear_mm;
    /* Anything nearer than the original line (+margin) along the 60 deg ray:
     * range = lateral / sin(60 deg), 1/sin(60) = 1.155. */
    limit =
        ((p_a->plan.offset_mm + p_a->config.margin_mm) * 1155U + 999U) / 1000U;
    p_a->phase_us = p_in->now_us;
    if (side <= limit)
    {
        /* Still beside it: is the side as far as planned (half width +
         * margin from the sensor line)? If not, turn away, then peek again. */
        const unsigned pad = p_a->config.half_width_mm + p_a->config.margin_mm,
                       gap = (side * 966U) / 1000U;
        const unsigned edge =
            p_a->plan.offset_mm > pad ? p_a->plan.offset_mm - pad : 0U;
        /* Measured distance from the original line now (obstacle edge + gap):
         * catches a car that drifted away while passing (left a bend crooked).
         */
        p_a->peek_lat_mm = edge + gap > 4095U ? 4095U : edge + gap;
        if (gap + S5_SIDE_TOL_MM < pad && p_a->side_turns < 3U)
        {
            unsigned deg = (pad - gap + 1U) / 2U;
            p_a->side_turn =
                deg > S5_SIDE_TURN_MAX_DEG ? S5_SIDE_TURN_MAX_DEG : deg;
            p_a->side_turns++;
        }
        if (p_a->searched_mm + S5_PEEK_STEP_MM > p_a->config.max_depth_mm)
        {
            return fail(p_a, S5_NO_PATH);
        }
        p_a->pass = S5_PASS_STEP;
    }
    else
    {
        p_a->pass = S5_PASS_EXTRA;
    }
    return action(S5_ACT_NONE, 0, 0);
}

s5_action_t s5_avoidance_step (s5_avoidance_t * p_a, const s5_input_t * p_in)
{
    unsigned elapsed;
    int sign;
    if (!p_a || !p_in)
    {
        return action(S5_ACT_STOP, 0, 0);
    }
    if (p_a->state == S5_IDLE || p_a->state == S5_COMPLETE ||
        p_a->state == S5_FAULT)
    {
        return action(S5_ACT_NONE, 0, 0);
    }
    if (!p_in->healthy)
    {
        return fail(p_a, S5_MOTION_ERROR);
    }
    if ((uint32_t)(p_in->now_us - p_a->started_us) > 60000000U)
    {
        return fail(p_a, S5_DEADLINE);
    }
    elapsed = p_in->now_us - p_a->phase_us;
    if (p_a->state == S5_WAIT_STOP)
    {
        if (elapsed > 1000000U)
        {
            return fail(p_a, S5_DEADLINE);
        }
        if (p_in->stopped)
        {
            p_a->state = S5_SCANNING;
            p_a->phase_us = p_in->now_us;
            return action(S5_ACT_SCAN, 0, 0);
        }
    }
    else if (p_a->state == S5_SCANNING)
    {
        if (elapsed > S5_SCAN_TIMEOUT_US)
        {
            return fail(p_a, S5_DEADLINE);
        }
        if (p_a->aiming)
        {
            /* Face-the-obstacle turn: wait for it, then scan again square-on.
             */
            if (p_a->issued)
            {
                if (p_in->motion_done)
                {
                    p_a->issued = false;
                    p_a->phase_us = p_in->now_us;
                }
                return action(S5_ACT_NONE, 0, 0);
            }
            if (!p_in->stopped)
            {
                return action(S5_ACT_NONE, 0, 0);
            }
            p_a->aiming = 0;
            p_a->phase_us = p_in->now_us;
            return action(S5_ACT_SCAN, 0, 0);
        }
        if (p_in->observation.scan_done &&
            (int32_t)(p_in->observation.scan_us - p_a->phase_us) > 0)
        {
            if (!p_in->observation.profile.valid)
            {
                return fail(p_a, S5_SCAN_FAILED);
            }
            if (!p_a->aimed)
            {
                bool bend;
                int turn = s5_aim_deg_bend(&p_in->observation.profile,
                                           p_in->turn_pct, &bend);
                p_a->aimed = 1;
                p_a->bend = bend;
                if (turn)
                {
                    s5_aim_set(p_a, turn);
                    p_a->aiming = 1;
                    p_a->issued = true;
                    p_a->phase_us = p_in->now_us;
                    return action(S5_ACT_TURN, turn, p_a->config.speed_mms);
                }
            }
            if (!s5_plan(&p_in->observation.profile, &p_a->config, &p_a->plan))
            {
                return fail(p_a, S5_NO_PATH);
            }
            p_a->state = S5_MOVING;
            p_a->phase_us = p_in->now_us;
        }
    }
    else if (p_a->state == S5_MOVING)
    {
        bool straight = (p_a->leg == 1U || p_a->leg == 3U || p_a->leg == 5U);
        if (elapsed > 20000000U)
        {
            return fail(p_a, S5_DEADLINE);
        }
        /* Servo faces sideways during a peek: no forward range until it ends.
         */
        if (p_a->leg == 3U && p_a->pass == S5_PASS_PEEKING)
        {
            return peek_result(p_a, p_in, elapsed);
        }
        /* Each straight leg requires a new forward reading after its turn.
         * During movement, loss of valid ranging or a close return brakes. */
        /* Reversing away from the face (leg 0) is exempt: the range only grows.
         */
        if (p_a->issued && !p_a->backing &&
            (!s5_range_ready(&p_in->observation, p_in->now_us) ||
             p_in->observation.range_mm <= p_a->config.emergency_mm))
        {
            return fail(p_a, S5_BAD_RANGE);
        }
        /* Line seen while passing (leg 3, only after the side reads clear,
         * so never beside the obstacle: a bend toward the bypass side) or on
         * the return diagonal (leg 5): stop, then the
         * IR-to-axle drive puts the axle on the line (as leg 5) before the
         * swerve. That tape is not kept: the swerve must see it again. */
        if (p_a->issued && !p_a->tail &&
            (p_a->leg == 5U ||
             (p_a->leg == 3U && p_a->pass == S5_PASS_EXTRA)) &&
            s5_recovery_observe(&p_a->recovery, p_in->now_us, p_in->healthy,
                                p_in->tape, p_in->junction, true))
        {
            if (p_a->leg == 3U)
            {
                p_a->leg = 5U;
            } /* rel: still the passing heading */
            p_a->issued = false;
            p_a->tail = 1;
            p_a->phase_us = p_in->now_us;
            return action(S5_ACT_STOP, 0, 0);
        }
        if (p_a->issued && p_a->leg >= 6U &&
            s5_recovery_observe(&p_a->recovery, p_in->now_us, p_in->healthy,
                                p_in->tape, p_in->junction, true))
        {
            /* Swerve: stop the moment the line is confirmed under the IR. */
            p_a->centre_dir = swerve_target(p_a) > s5_rel_get(p_a) ? 1U : 0U;
            p_a->centre_tries = 0;
            p_a->issued = false;
            p_a->state = S5_FINISH;
            p_a->phase_us = p_in->now_us;
            return action(S5_ACT_STOP, 0, 0);
        }
        if (p_a->issued)
        {
            if (p_in->motion_done)
            {
                /* Leg-3 drives after a turn away also move the car sideways
                 * (rel degrees off the passing heading): count it, so the
                 * return diagonal still ends on the original line. */
                if (p_a->leg == 3U && s5_rel_get(p_a) > 0 &&
                    p_a->pass != S5_PASS_APPROACH)
                {
                    const float64_t value_mm =
                        (p_in->centre_m - p_a->leg_start_m) * 1000.0 *
                        (float64_t)s5_rel_get(p_a) * 0.017453;
                    if (value_mm > 0.0)
                    {
                        const unsigned sum =
                            p_a->lat_mm + (unsigned)(value_mm + 0.5);
                        p_a->lat_mm = sum > 511U ? 511U : sum;
                    }
                }
                p_a->issued = false;
                p_a->phase_us = p_in->now_us;
                /* Leg 3 ends only after the final clearance move. */
                if (p_a->leg == 3U && p_a->pass != S5_PASS_EXTRA)
                {
                    p_a->pass = S5_PASS_PEEK;
                    return action(S5_ACT_NONE, 0, 0);
                }
                /* Reverse finished: now the leg-0 turn. */
                if (p_a->leg == 0U && p_a->backing)
                {
                    p_a->backing = 0;
                    return action(S5_ACT_NONE, 0, 0);
                }
                if (p_a->leg == 4U)
                {
                    s5_rel_set(p_a, -S5_BYPASS_DEG);
                }
                if (p_a->leg == 5U)
                {
                    /* No line by the end (bend away): once more along the
                     * diagonal. */
                    if (S5_RETURN_EXTRA_MM > 0U && !p_a->tail && !p_a->extended)
                    {
                        p_a->extended = 1;
                        return action(S5_ACT_NONE, 0, 0);
                    }
                    /* Only tape seen by the swerve from here counts. */
                    s5_recovery_reset(&p_a->recovery);
                }
                if (p_a->leg >= 6U)
                {
                    s5_rel_set(p_a, swerve_target(p_a));
                }
                if (p_a->leg == 8U)
                {
                    return fail(p_a, S5_NO_LINE);
                } /* swerved both ways, back on heading */
                p_a->leg++;
            }
            return action(S5_ACT_NONE, 0, 0);
        }
        if (!p_in->stopped)
        {
            return action(S5_ACT_NONE, 0, 0);
        }
        if (!s5_range_ready(&p_in->observation, p_in->now_us) ||
            (int32_t)(p_in->observation.range_us - p_a->phase_us) <= 0)
        {
            return elapsed > 1000000U ? fail(p_a, S5_BAD_RANGE)
                                      : action(S5_ACT_NONE, 0, 0);
        }
        if (p_a->leg == 0U && !p_a->backed)
        {
            /* Too close for the 45-degree out-leg: reverse the shortfall first.
             */
            unsigned back = diagonal_short(p_a, p_a->plan.offset_mm);
            p_a->backed = 1;
            if (back > S5_BACKUP_MAX_MM && !p_a->swapped && p_a->plan.alt_ok &&
                diagonal_short(p_a, p_a->plan.alt_offset_mm) <=
                    S5_BACKUP_MAX_MM)
            {
                /* Preferred side needs more room than reversing gives: still
                 * facing the obstacle, take the other side instead. */
                const unsigned other = p_a->plan.alt_offset_mm;
                p_a->swapped = 1;
                p_a->plan.alt_offset_mm = p_a->plan.offset_mm;
                p_a->plan.offset_mm = other;
                p_a->plan.route = p_a->plan.route == S5_ROUTE_LEFT
                                      ? S5_ROUTE_RIGHT
                                      : S5_ROUTE_LEFT;
                back = diagonal_short(p_a, p_a->plan.offset_mm);
            }
            if (back)
            {
                if (back > S5_BACKUP_MAX_MM)
                {
                    return fail(p_a, S5_NO_PATH);
                }
                p_a->plan.closest_mm += back;
                p_a->plan.forward_mm += back;
                p_a->backing = 1;
                p_a->issued = true;
                return action(S5_ACT_DRIVE, -(int)back, p_a->config.speed_mms);
            }
        }
        if (p_in->observation.range_mm <= p_a->config.emergency_mm)
        {
            return fail(p_a, S5_NO_PATH);
        }
        sign = p_a->plan.route == S5_ROUTE_LEFT ? 1 : -1;
        if (p_a->leg == 3U && p_a->pass == S5_PASS_PEEK)
        {
            p_a->pass = S5_PASS_PEEKING;
            p_a->phase_us = p_in->now_us;
            return action(S5_ACT_PEEK,
                          (int)(p_a->plan.route == S5_ROUTE_LEFT
                                    ? 90U - S5_PEEK_DEG
                                    : 90U + S5_PEEK_DEG),
                          0);
        }
        if (p_a->leg == 3U && p_a->pass == S5_PASS_STEP && p_a->side_turn)
        {
            /* Turn away from the obstacle side, then peek again. rel keeps the
             * heading vs the one at the obstacle, so the way back still aims
             * at the original line. */
            const int deg = (int)p_a->side_turn;
            p_a->side_turn = 0;
            p_a->issued = true;
            p_a->pass = S5_PASS_APPROACH;
            s5_rel_set(p_a, s5_rel_get(p_a) + deg);
            return action(S5_ACT_TURN, sign * deg, p_a->config.speed_mms);
        }
        if (straight)
        {
            unsigned value_mm = diagonal_mm(
                p_a->leg == 5U ? return_lat(p_a) : p_a->plan.offset_mm);
            if (p_a->leg == 5U && (p_a->tail || p_a->extended))
            {
                value_mm = p_a->tail ? S5_IR_AXLE_MM : S5_RETURN_EXTRA_MM;
            }
            if (p_a->leg == 3U)
            {
                /* The diagonal out-leg already moved the car offset forward. */
                unsigned forward = p_a->plan.forward_mm,
                         offset = p_a->plan.offset_mm;
                unsigned approach = forward > offset ? forward - offset : 0U;
                if (p_a->pass == S5_PASS_APPROACH && approach < S5_PEEK_STEP_MM)
                {
                    p_a->pass = S5_PASS_PEEK;
                    return action(S5_ACT_NONE, 0, 0); /* already at the face */
                }
                value_mm =
                    p_a->pass == S5_PASS_APPROACH
                        ? approach
                        : (p_a->pass == S5_PASS_STEP ? S5_PEEK_STEP_MM
                                                     : pass_extra_mm(p_a));
            }
            if (!s5_range_ready(&p_in->observation, p_in->now_us) ||
                (int32_t)(p_in->observation.range_us - p_a->phase_us) <= 0)
            {
                return elapsed > 1000000U ? fail(p_a, S5_BAD_RANGE)
                                          : action(S5_ACT_NONE, 0, 0);
            }
            { /* Leg 1 faces the diagonal path itself: it must be clear for
               * the whole diagonal + margin (measured), and the forward sweep
               * of the 45-degree leg must not reach the obstacle face. */
                unsigned need = S5_LEG_CLEAR_MM;
                bool blocked;
                if (p_a->leg == 1U && value_mm + p_a->config.margin_mm > need)
                {
                    need = value_mm + p_a->config.margin_mm;
                }
                blocked = p_in->observation.range_mm <= need ||
                          (p_a->leg == 1U &&
                           !diagonal_clear(p_a, p_a->plan.offset_mm));
                if (blocked && p_a->leg == 3U && p_a->side_turns < 3U)
                {
                    /* Passing but the obstacle's side shows ahead: the car is
                     * converging on it. Turn away, then peek again. */
                    p_a->side_turns++;
                    p_a->issued = true;
                    p_a->pass = S5_PASS_APPROACH;
                    s5_rel_set(p_a,
                               s5_rel_get(p_a) + (int)S5_SIDE_TURN_MAX_DEG);
                    return action(S5_ACT_TURN, sign * (int)S5_SIDE_TURN_MAX_DEG,
                                  p_a->config.speed_mms);
                }
                if (blocked && p_a->leg == 1U &&
                    p_a->blocked_reads + 1U < S5_BLOCK_CONFIRM)
                {
                    /* Wait for another fresh reading before believing it. */
                    p_a->blocked_reads++;
                    p_a->phase_us = p_in->now_us;
                    return action(S5_ACT_NONE, 0, 0);
                }
                if (!blocked)
                {
                    p_a->blocked_reads = 0;
                }
                if (blocked)
                {
                    p_a->blocked_reads = 0;
                    /* First side blocked: turn 2 x 45 deg to face the other
                     * diagonal and re-check it as leg 1 (once). */
                    if (p_a->leg == 1U && !p_a->swapped && p_a->plan.alt_ok)
                    {
                        unsigned other = p_a->plan.alt_offset_mm;
                        p_a->swapped = 1;
                        p_a->plan.alt_offset_mm = p_a->plan.offset_mm;
                        p_a->plan.offset_mm = other;
                        p_a->plan.route = p_a->plan.route == S5_ROUTE_LEFT
                                              ? S5_ROUTE_RIGHT
                                              : S5_ROUTE_LEFT;
                        p_a->leg = 0;
                        p_a->issued = true;
                        return action(S5_ACT_TURN,
                                      p_a->plan.route == S5_ROUTE_LEFT
                                          ? 2 * S5_BYPASS_DEG
                                          : -2 * S5_BYPASS_DEG,
                                      p_a->config.speed_mms);
                    }
                    return fail(p_a, S5_NO_PATH);
                }
            }
            p_a->issued = true;
            p_a->leg_start_m = p_in->centre_m;
            if (p_a->leg == 3U && p_a->pass == S5_PASS_STEP)
            {
                p_a->searched_mm = (uint16_t)(p_a->searched_mm + value_mm);
            }
            return action(S5_ACT_DRIVE, (int)value_mm, p_a->config.speed_mms);
        }
        p_a->issued = true;
        if (p_a->leg == 0U)
        {
            return action(S5_ACT_TURN, sign * S5_BYPASS_DEG,
                          p_a->config.speed_mms);
        }
        if (p_a->leg == 2U)
        {
            return action(S5_ACT_TURN, -sign * S5_BYPASS_DEG,
                          p_a->config.speed_mms);
        }
        /* Turn in to 45 degrees off the passing heading (less any turn away).
         */
        if (p_a->leg == 4U)
        {
            return action(S5_ACT_TURN,
                          -sign * (S5_BYPASS_DEG + s5_rel_get(p_a)),
                          p_a->config.speed_mms);
        }
        /* Swerve (legs 6..8), each turn stopped as soon as the line is seen:
         * toward the bypass side up to S5_SWERVE_OUT_DEG past the heading at
         * the obstacle, then the other way up to S5_SWERVE_IN_DEG past it,
         * then back to it. Never more than 90 deg either side, so the car
         * cannot pick up the line behind it and follow it backwards. */
        return action(S5_ACT_TURN,
                      sign * (swerve_target(p_a) - s5_rel_get(p_a)),
                      p_a->config.speed_mms);
    }
    else if (p_a->state == S5_FINISH)
    {
        /* Hand back only with the tape under the IR now. Braking can carry
         * the IR past the edge it stopped on: then turn back slowly (the way
         * it came, then the other way), stopping on the tape. */
        const int side = p_a->plan.route == S5_ROUTE_LEFT ? 1 : -1,
                  dir = p_a->centre_dir ? side : -side;
        if (elapsed > 3000000U)
        {
            return fail(p_a, S5_NO_LINE);
        }
        if (p_a->issued)
        {
            if (p_in->tape && !p_in->junction)
            {
                p_a->issued = false;
                p_a->phase_us = p_in->now_us;
                return action(S5_ACT_STOP, 0, 0);
            }
            if (p_in->motion_done)
            {
                p_a->issued = false;
                p_a->phase_us = p_in->now_us;
            }
            return action(S5_ACT_NONE, 0, 0);
        }
        if (!p_in->stopped)
        {
            return action(S5_ACT_NONE, 0, 0);
        }
        if (p_in->tape && p_in->line_ready && !p_in->junction)
        {
            p_a->state = S5_COMPLETE;
            return action(S5_ACT_RESUME, 0, 0);
        }
        if (p_a->centre_tries >= 2U)
        {
            return fail(p_a, S5_NO_LINE);
        }
        p_a->issued = true;
        p_a->phase_us = p_in->now_us;
        p_a->centre_tries++;
        return action(
            S5_ACT_TURN,
            (p_a->centre_tries == 1U ? -dir : dir) *
                (p_a->centre_tries == 1U ? S5_CENTRE_DEG : 2 * S5_CENTRE_DEG),
            S5_CENTRE_SPEED_MMS);
    }
    return action(S5_ACT_NONE, 0, 0);
}
