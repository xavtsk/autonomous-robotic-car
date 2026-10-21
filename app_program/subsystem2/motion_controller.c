/*
 * File: motion_controller.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Motion Control statechart, straight-line correction and
 *          encoder-based turning.
 *
 * Minimum-bit representation (DATA_FORMAT.md, internal fields): the state,
 * latched fault bits, command ID and flags share one word of bit-fields, and
 * the timers, counters and stored encoder counts use only the bits their
 * range needs. Every function runs with the Buddy 2 mutex held
 * (motion_controller.h), so no read-modify-write of a packed word can race.
 * Speeds, distances and PID terms stay float (control precision).
 */

#include <stddef.h>

#include "motion_controller.h"
#include "motion_estimation.h"
#include "motion_pack.h"
#include "motor_control.h"
#include "speed_pid.h"
#include "wheel_encoder.h"

/* ---- Tuning (the defaults should work; change only if needed) ----------- */
#define SPEED_KP_N              (3.2f)   /* P gain, normalised to top speed  */
#define SPEED_KI_N              (5.0f)   /* I gain, per second (same units)  */
#define SPEED_KD_N              (0.0f)   /* D gain, seconds; 0 = PI only     */
#define SYNC_GAIN               (4.0f)   /* m/s per m of left/right mismatch */
#define ACCEL_MPS2              (1.5f)   /* speed ramp; stops wheel slip     */
#define DECEL_MPS2              (0.6f)   /* slow-down before a target        */
#define MIN_SPEED_MPS           (0.04f)  /* crawl speed before a target      */
#define CRAWL_DIST_M            (0.008f) /* crawl this far before braking    */
#define SPEED_LIMIT_FRACTION    (0.9f)   /* requests capped at 90 % of top   */

/* ---- Fault detection --------------------------------------------------- */
#define VELOCITY_TIMEOUT_US     (250000U) /* stale command                   */
#define STALL_TIMEOUT_MS        (600U)    /* driven, no encoder progress     */
#define STALL_MIN_PROGRESS      (2)       /* counts needed to show progress  */
#define DIRECTION_FAULT_MS      (300U)    /* turning against the command     */
#define ENCODER_GLITCH_LIMIT    (25)      /* invalid-signal budget           */
#define RESET_STILL_MS          (200U)    /* fault cleared = wheels still    */
#define STOPPED_STILL_MS        (100U)    /* stopped acknowledgement         */
#define CMD_ACTIVE_MPS          (0.02f)   /* below this a wheel is not driven */
#define DIRECTION_MIN_MPS       (0.05f)
#define STILL_SPEED_MPS         (0.01f)
#define STILL_MS_MAX            (60000U)
#define TIMEOUT_FACTOR          (3.0f)    /* allowed time = 3 x expected     */
#define TIMEOUT_MARGIN_S        (2.0f)    /*                 + 2 s           */
#define MOVE_TIME_MAX_S         (600.0f)  /* longer allowed time: rejected;  */
                                          /* keeps the us deadline far below */
                                          /* the 2^31 us wrap-safe limit     */

/* ---- Field widths (each covers its range; DATA_FORMAT.md) -------------- */
#define STATE_BITS              (3U)      /* 6 states                        */
#define CMD_ID_BITS             (8U)      /* 1 ... 255, 0 = none             */
#define STALL_MS_BITS           (10U)     /* 0 ... 1023 ms, held at the max  */
#define WRONG_MS_BITS           (9U)      /* 0 ... 511 ms, held at the max   */
#define GLITCH_SEEN_BITS        (8U)      /* low bits of the glitch counter  */
#define GLITCH_BUCKET_BITS      (6U)      /* 0 ... 2 x ENCODER_GLITCH_LIMIT  */
#define STALL_MS_MAX            ((1U << STALL_MS_BITS) - 1U)
#define WRONG_MS_MAX            ((1U << WRONG_MS_BITS) - 1U)
#define GLITCH_SEEN_MASK        ((1U << GLITCH_SEEN_BITS) - 1U)

typedef struct
{
    speed_pid_t pid;
    uint32_t    stall_ref     : WHEEL_COUNT_BITS; /* count at stall restart */
    uint32_t    stall_ms      : STALL_MS_BITS;    /* driven, no progress    */
    uint32_t    wrong_ms      : WRONG_MS_BITS;    /* turning against command */
    uint32_t    glitch_seen   : GLITCH_SEEN_BITS; /* glitches counted       */
    uint32_t    glitch_bucket : GLITCH_BUCKET_BITS; /* leaky bucket        */
} wheel_ctrl_t;

/* Statechart state, latched fault bits, command ID and flags: 21 bits */
typedef struct
{
    uint32_t state      : STATE_BITS;               /* motion_state_t      */
    uint32_t fault_bits : MOTION_FAULT_BITS_WIDTH;  /* MOTION_FAULT_BIT()  */
    uint32_t cmd_id     : CMD_ID_BITS;  /* running / last accepted         */
    uint32_t b_busy     : 1;            /* calibration running             */
    uint32_t b_brake_pending : 1;       /* stopped, brake not applied yet  */
    uint32_t b_mv_turn  : 1;            /* move kind: 1 = TURN, 0 = DRIVE  */
    uint32_t mv_back    : WHEEL_COUNT;  /* move direction, bit per wheel:  */
                                        /* 1 = backward                    */
} ctrl_t;

static wheel_ctrl_t   g_wheel[WHEEL_COUNT];
static ctrl_t         g_ctl = { (uint32_t)MOTION_IDLE, 0U, 0U, 0U, 0U, 0U, 0U };
static uint16_t       g_still_ms = 0U;  /* 0 ... STILL_MS_MAX              */
static float          g_max_speed_mps = 1.0f;
static float          g_lead_drive_m = 0.0f;
static float          g_lead_turn_m = 0.0f;

/* Reports go to the sink (the queue in motion.c) */
static motion_report_sink_t g_p_sink = NULL;

/* VELOCITY CONTROL */
static float          g_vel_target[WHEEL_COUNT];
static uint32_t       g_vel_stamp_us = 0U;

/* DRIVE / TURN */
static wheel_count_t  g_mv_start[WHEEL_COUNT];
static float          g_mv_dist_m = 0.0f;
static float          g_mv_speed_mps = 0.0f;
static uint32_t       g_mv_deadline_us = 0U;

/* ---- Helpers ------------------------------------------------------------ */

static float abs_f(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float min_f(float a, float b)
{
    return (a < b) ? a : b;
}

static float max_f(float a, float b)
{
    return (a > b) ? a : b;
}

/* Square root by Newton's method (the build does not link libm) */
static float sqrt_f(float x)
{
    float    guess;
    uint32_t i;

    if (x <= 0.0f)
    {
        return 0.0f;
    }
    guess = (x > 1.0f) ? x : 1.0f;
    for (i = 0U; i < 12U; i++)
    {
        guess = 0.5f * (guess + (x / guess));
    }

    return guess;
}

static int32_t abs_i(int32_t x)
{
    return (x < 0) ? -x : x;
}

/* Add to a timer field, held at its largest value instead of wrapping */
static uint32_t add_ms(uint32_t ms, uint32_t dt_ms, uint32_t max_ms)
{
    return ((max_ms - ms) > dt_ms) ? (ms + dt_ms) : max_ms;
}

/* true once 'now' is past 'deadline' (wrap-safe) */
static bool time_after(uint32_t now_us, uint32_t deadline_us)
{
    return ((int32_t)(now_us - deadline_us) > 0);
}

static motion_state_t state(void)
{
    return (motion_state_t)g_ctl.state;
}

static void set_state(motion_state_t s)
{
    g_ctl.state = (uint32_t)s;
}

static bool is_active(motion_state_t s)
{
    return (s == MOTION_VELOCITY) || (s == MOTION_DRIVE) || (s == MOTION_TURN);
}

/* +1 forward, -1 backward for the running DRIVE / TURN */
static float mv_sign(wheel_id_t wheel)
{
    return (((g_ctl.mv_back >> (uint32_t)wheel) & 1U) != 0U) ? -1.0f : 1.0f;
}

/* Motion task only: brake both motors. Clears a pending stop, which lets
 * requests in again (stop priority). */
static void outputs_safe(void)
{
    uint32_t i;

    motor_control_brake_all();
    g_ctl.b_brake_pending = 0U;
    for (i = 0U; i < (uint32_t)WHEEL_COUNT; i++)
    {
        speed_pid_reset(&g_wheel[i].pid);
    }
}

static void new_command(void)
{
    g_ctl.cmd_id++;                     /* 8-bit field: 255 wraps to 0 */
    if (g_ctl.cmd_id == 0U)
    {
        g_ctl.cmd_id = 1U;              /* 0 means "no command" */
    }
}

static void report(motion_outcome_t outcome, motion_fault_t fault)
{
    motion_report_t r;

    r.command_id = (uint8_t)g_ctl.cmd_id;
    r.outcome = outcome;
    r.fault = fault;
    r.b_lost = false;                   /* set by the queue if needed */
    if (g_p_sink != NULL)
    {
        g_p_sink(&r);
    }
}

/* Latch every new fault bit; the report carries the lowest one */
static void raise_faults(uint32_t fault_bits)
{
    outputs_safe();
    g_ctl.fault_bits |= fault_bits;
    set_state(MOTION_FAULT);
    report(MOTION_OUTCOME_FAULT, motion_fault_first((uint8_t)fault_bits));
}

static void raise_fault(motion_fault_t fault)
{
    raise_faults(MOTION_FAULT_BIT(fault));
}

/* Does not touch the motors: requests call it outside the motion task */
static void complete(void)
{
    set_state(MOTION_COMPLETE);
    report(MOTION_OUTCOME_DONE, MOTION_FAULT_NONE);
}

static void clear_checks(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)WHEEL_COUNT; i++)
    {
        g_wheel[i].stall_ref = wheel_encoder_bits((wheel_id_t)i);
        g_wheel[i].stall_ms = 0U;
        g_wheel[i].wrong_ms = 0U;
    }
}

static void wheel_drive(wheel_id_t wheel, float target_mps, float dt_s)
{
    float duty = speed_pid_update(&g_wheel[wheel].pid, target_mps,
                                  motion_estimation_wheel_speed(wheel),
                                  motion_estimation_wheel_accel(wheel), dt_s);

    motor_control_set(wheel, duty);
}

/* Feedback checks while a wheel is being driven */
static motion_fault_t wheel_check(wheel_id_t wheel, uint32_t dt_ms)
{
    wheel_ctrl_t *p_w = &g_wheel[wheel];
    uint32_t count = wheel_encoder_bits(wheel);
    float    cmd = speed_pid_setpoint(&p_w->pid);
    float    speed = motion_estimation_wheel_speed(wheel);

    /* Stall / no progress: driven, but the encoder has not moved on by at
     * least STALL_MIN_PROGRESS counts (wheel jammed, encoder unplugged, or
     * one encoder wire broken: a single channel only flickers +1 / -1) */
    if ((abs_f(cmd) < CMD_ACTIVE_MPS)
        || (abs_i(wheel_encoder_delta(count, p_w->stall_ref))
            >= STALL_MIN_PROGRESS))
    {
        p_w->stall_ref = count;
        p_w->stall_ms = 0U;
    }
    else
    {
        p_w->stall_ms = add_ms(p_w->stall_ms, dt_ms, STALL_MS_MAX);
    }

    /* Direction: clearly turning against the command (motor or encoder
     * reversed: the controller would otherwise run away) */
    if ((abs_f(cmd) > DIRECTION_MIN_MPS) && (abs_f(speed) > DIRECTION_MIN_MPS)
        && ((cmd > 0.0f) != (speed > 0.0f)))
    {
        p_w->wrong_ms = add_ms(p_w->wrong_ms, dt_ms, WRONG_MS_MAX);
    }
    else
    {
        p_w->wrong_ms = 0U;
    }

    if (p_w->stall_ms >= STALL_TIMEOUT_MS)
    {
        return MOTION_FAULT_STALL;
    }
    if (p_w->wrong_ms >= DIRECTION_FAULT_MS)
    {
        return MOTION_FAULT_DIRECTION;
    }
    if (p_w->glitch_bucket > (uint32_t)ENCODER_GLITCH_LIMIT)
    {
        return MOTION_FAULT_ENCODER;
    }

    return MOTION_FAULT_NONE;
}

static void track_glitches(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)WHEEL_COUNT; i++)
    {
        wheel_ctrl_t *p_w = &g_wheel[i];
        uint32_t seen = wheel_encoder_glitches((wheel_id_t)i);
        uint32_t bucket = p_w->glitch_bucket
                          + ((seen - p_w->glitch_seen) & GLITCH_SEEN_MASK);

        p_w->glitch_seen = seen & GLITCH_SEEN_MASK;
        if (bucket > (2U * (uint32_t)ENCODER_GLITCH_LIMIT))
        {
            bucket = 2U * (uint32_t)ENCODER_GLITCH_LIMIT;
        }
        if (bucket > 0U)
        {
            bucket--;
        }
        p_w->glitch_bucket = bucket;
    }
}

/* Distance moved along each wheel's commanded direction */
static float move_wheel_progress(wheel_id_t wheel)
{
    return mv_sign(wheel)
           * (float)wheel_encoder_delta(wheel_encoder_bits(wheel),
                                        g_mv_start[wheel].bits)
           * motion_estimation_scale();
}

/* DRIVE / TURN step. Returns true when the brake point is reached. */
static bool move_step(float dt_s)
{
    float lead = (g_ctl.b_mv_turn != 0U) ? g_lead_turn_m : g_lead_drive_m;
    float dl = move_wheel_progress(WHEEL_LEFT);
    float dr = move_wheel_progress(WHEEL_RIGHT);
    float to_brake = g_mv_dist_m - lead - (0.5f * (dl + dr));
    float crawl = min_f(g_mv_speed_mps, MIN_SPEED_MPS);
    float v;
    float corr;

    if (to_brake <= 0.0f)
    {
        return true;
    }

    /* Slow to crawl speed CRAWL_DIST_M before the brake point, so every move
     * brakes from the same low speed (repeatable coast) */
    v = sqrt_f(2.0f * DECEL_MPS2 * max_f(to_brake - CRAWL_DIST_M, 0.0f));
    v = max_f(min_f(v, g_mv_speed_mps), crawl);

    /* Straight-line correction: the wheel that is ahead slows (down to a
     * stop if needed), the other speeds up */
    corr = SYNC_GAIN * (dl - dr);
    corr = max_f(min_f(corr, v), -v);

    wheel_drive(WHEEL_LEFT, mv_sign(WHEEL_LEFT) * max_f(v - corr, 0.0f), dt_s);
    wheel_drive(WHEEL_RIGHT, mv_sign(WHEEL_RIGHT) * max_f(v + corr, 0.0f),
                dt_s);

    return false;
}

/* ---- Interface ----------------------------------------------------------- */

void motion_controller_init(float max_speed_mps, float stop_lead_drive_m,
                            float stop_lead_turn_m,
                            motion_report_sink_t p_sink)
{
    float    kff;
    uint32_t i;

    g_max_speed_mps = (max_speed_mps > 0.0f) ? max_speed_mps : 1.0f;
    g_lead_drive_m = stop_lead_drive_m;
    g_lead_turn_m = stop_lead_turn_m;
    g_p_sink = p_sink;
    kff = 1.0f / g_max_speed_mps;

    for (i = 0U; i < (uint32_t)WHEEL_COUNT; i++)
    {
        speed_pid_init(&g_wheel[i].pid, SPEED_KP_N * kff, SPEED_KI_N * kff,
                       SPEED_KD_N * kff, kff, ACCEL_MPS2);
        g_wheel[i].glitch_seen = wheel_encoder_glitches((wheel_id_t)i)
                                 & GLITCH_SEEN_MASK;
        g_wheel[i].glitch_bucket = 0U;
        g_vel_target[i] = 0.0f;
    }
    clear_checks();
    set_state(MOTION_IDLE);
    g_ctl.fault_bits = 0U;
    outputs_safe();
}

void motion_controller_step(float dt_s, uint32_t now_us)
{
    uint32_t       dt_ms = (uint32_t)((dt_s * 1000.0f) + 0.5f);
    bool           b_done = false;
    motion_fault_t fault_l;
    motion_fault_t fault_r;

    track_glitches();
    if ((abs_f(motion_estimation_wheel_speed(WHEEL_LEFT)) < STILL_SPEED_MPS)
        && (abs_f(motion_estimation_wheel_speed(WHEEL_RIGHT))
            < STILL_SPEED_MPS))
    {
        if (g_still_ms < STILL_MS_MAX)
        {
            g_still_ms = (uint16_t)(g_still_ms + dt_ms);
        }
    }
    else
    {
        g_still_ms = 0U;
    }

    if (g_ctl.b_busy != 0U)
    {
        return;                         /* calibration drives the motors */
    }

    switch (state())
    {
    case MOTION_VELOCITY:
        if (time_after(now_us, g_vel_stamp_us + VELOCITY_TIMEOUT_US))
        {
            raise_fault(MOTION_FAULT_STALE_COMMAND);

            return;
        }
        wheel_drive(WHEEL_LEFT,  g_vel_target[WHEEL_LEFT],  dt_s);
        wheel_drive(WHEEL_RIGHT, g_vel_target[WHEEL_RIGHT], dt_s);
        break;

    case MOTION_DRIVE:
    case MOTION_TURN:
        if (time_after(now_us, g_mv_deadline_us))
        {
            raise_fault(MOTION_FAULT_TIMEOUT);

            return;
        }
        b_done = move_step(dt_s);
        break;

    default:                            /* IDLE, COMPLETE, FAULT */
        outputs_safe();

        return;
    }

    /* Active-state feedback checks (both wheels, to keep the timers right).
     * They run before completion, so a fault wins over completion. A fault
     * on each wheel latches both bits. */
    fault_l = wheel_check(WHEEL_LEFT, dt_ms);
    fault_r = wheel_check(WHEEL_RIGHT, dt_ms);
    if ((fault_l != MOTION_FAULT_NONE) || (fault_r != MOTION_FAULT_NONE))
    {
        raise_faults((uint32_t)MOTION_FAULT_BIT(fault_l)
                     | (uint32_t)MOTION_FAULT_BIT(fault_r));
    }
    else if (b_done)
    {
        outputs_safe();
        complete();
    }
    else
    {
        /* still running */
    }
}

void motion_controller_make_safe(void)
{
    if (!is_active(state()) && (g_ctl.b_busy == 0U))
    {
        outputs_safe();
    }
}

motion_result_t motion_controller_request_velocity(float left_mps,
                                                   float right_mps,
                                                   uint32_t now_us)
{
    float cap = motion_controller_speed_cap();

    if (state() == MOTION_FAULT)
    {
        return MOTION_ERR_FAULT;
    }
    /* Stop priority: nothing new until the motion task has braked */
    if ((g_ctl.b_busy != 0U) || (g_ctl.b_brake_pending != 0U)
        || ((state() != MOTION_IDLE) && (state() != MOTION_VELOCITY)))
    {
        return MOTION_ERR_BUSY;
    }
    g_vel_target[WHEEL_LEFT]  = max_f(min_f(left_mps, cap), -cap);
    g_vel_target[WHEEL_RIGHT] = max_f(min_f(right_mps, cap), -cap);
    g_vel_stamp_us = now_us;
    if (state() == MOTION_IDLE)
    {
        new_command();                  /* updates keep this ID */
        clear_checks();
        set_state(MOTION_VELOCITY);
    }

    return MOTION_OK;
}

motion_result_t motion_controller_request_move(motion_state_t kind,
                                               int32_t sign_left,
                                               int32_t sign_right,
                                               float dist_m, float speed_mps,
                                               uint32_t now_us)
{
    float speed = min_f(speed_mps, motion_controller_speed_cap());
    float allowed_s = ((dist_m / speed) * TIMEOUT_FACTOR) + TIMEOUT_MARGIN_S;

    if (state() == MOTION_FAULT)
    {
        return MOTION_ERR_FAULT;
    }
    /* Stop priority: nothing new until the motion task has braked */
    if ((g_ctl.b_busy != 0U) || (g_ctl.b_brake_pending != 0U)
        || (state() != MOTION_IDLE))
    {
        return MOTION_ERR_BUSY;
    }
    /* Reject what the deadline cannot represent (also NaN / infinity) */
    if (!(dist_m >= 0.0f) || !(speed > 0.0f)
        || !(allowed_s <= MOVE_TIME_MAX_S))
    {
        return MOTION_ERR_BAD_ARG;
    }

    g_ctl.b_mv_turn = (kind == MOTION_TURN) ? 1U : 0U;
    g_ctl.mv_back = ((sign_left < 0) ? (1U << (uint32_t)WHEEL_LEFT) : 0U)
                    | ((sign_right < 0) ? (1U << (uint32_t)WHEEL_RIGHT) : 0U);
    g_mv_start[WHEEL_LEFT].bits = wheel_encoder_bits(WHEEL_LEFT);
    g_mv_start[WHEEL_RIGHT].bits = wheel_encoder_bits(WHEEL_RIGHT);
    g_mv_dist_m = dist_m;
    g_mv_speed_mps = speed;
    g_mv_deadline_us = now_us + (uint32_t)(allowed_s * 1000000.0f);
    clear_checks();
    new_command();
    set_state(kind);
    if (dist_m <= 0.0f)
    {
        complete();                     /* zero length: done at once */
    }

    return MOTION_OK;
}

/* One result per command: DONE is final once it is reported, so a stop
 * after completion only acknowledges it (no CANCELLED for the same ID). */
void motion_controller_stop(void)
{
    if (is_active(state()))
    {
        report(MOTION_OUTCOME_CANCELLED, MOTION_FAULT_NONE);
        g_ctl.b_brake_pending = 1U;     /* motors may still be driven */
    }
    if (state() != MOTION_FAULT)
    {
        set_state(MOTION_IDLE);         /* the motion task brakes at once */
    }
}

motion_result_t motion_controller_cancel(uint8_t command_id)
{
    if (state() == MOTION_FAULT)
    {
        return MOTION_ERR_FAULT;        /* a cancel never clears a fault */
    }
    /* Only a running command can be cancelled; a completed one is final */
    if ((command_id == 0U) || (command_id != g_ctl.cmd_id)
        || !is_active(state()))
    {
        return MOTION_ERR_OBSOLETE;
    }
    motion_controller_stop();

    return MOTION_OK;
}

bool motion_controller_reset(void)
{
    if (state() != MOTION_FAULT)
    {
        return true;
    }
    if ((g_still_ms >= RESET_STILL_MS)
        && (g_wheel[WHEEL_LEFT].glitch_bucket
            <= ((uint32_t)ENCODER_GLITCH_LIMIT / 2U))
        && (g_wheel[WHEEL_RIGHT].glitch_bucket
            <= ((uint32_t)ENCODER_GLITCH_LIMIT / 2U)))
    {
        g_ctl.fault_bits = 0U;
        set_state(MOTION_IDLE);

        return true;
    }

    return false;
}

motion_state_t motion_controller_poll(void)
{
    motion_state_t s = state();

    if (s == MOTION_COMPLETE)
    {
        set_state(MOTION_IDLE);         /* completion reported */
    }

    return s;
}

motion_state_t motion_controller_state(void)
{
    return state();
}

motion_fault_t motion_controller_fault(void)
{
    return motion_fault_first((uint8_t)g_ctl.fault_bits);
}

uint8_t motion_controller_fault_bits(void)
{
    return (uint8_t)g_ctl.fault_bits;
}

uint8_t motion_controller_command_id(void)
{
    return (uint8_t)g_ctl.cmd_id;
}

void motion_controller_ack_complete(uint8_t command_id)
{
    if ((state() == MOTION_COMPLETE) && (command_id == g_ctl.cmd_id))
    {
        set_state(MOTION_IDLE);         /* completion reported */
    }
}

/* Never "stopped" while a stop is still being braked or calibration (which
 * drives the motors itself) is running */
bool motion_controller_is_stopped(void)
{
    return !is_active(state()) && (g_ctl.b_busy == 0U)
           && (g_ctl.b_brake_pending == 0U)
           && (g_still_ms >= STOPPED_STILL_MS);
}

float motion_controller_speed_cap(void)
{
    return SPEED_LIMIT_FRACTION * g_max_speed_mps;
}

void motion_controller_set_busy(bool b_busy)
{
    g_ctl.b_busy = b_busy ? 1U : 0U;
}

bool motion_controller_is_busy(void)
{
    return (g_ctl.b_busy != 0U);
}
