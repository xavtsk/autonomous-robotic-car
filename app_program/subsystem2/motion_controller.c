/*
 * File: motion_controller.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Motion Control statechart, straight-line correction and
 *          encoder-based turning.
 */

#include "motion_controller.h"
#include "motion_estimation.h"
#include "motor_control.h"
#include "speed_pid.h"
#include "wheel_encoder.h"

/* ---- Tuning (the defaults should work; change only if needed) ----------- */
#define SPEED_KP_N              (1.6f)   /* P gain, normalised to top speed  */
#define SPEED_KI_N              (10.0f)  /* I gain, per second (same units)  */
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
#define CMD_ACTIVE_MPS          (0.02f)   /* below this a wheel is not driven */
#define DIRECTION_MIN_MPS       (0.05f)
#define STILL_SPEED_MPS         (0.01f)
#define STILL_MS_MAX            (60000U)
#define TIMEOUT_FACTOR          (3.0f)    /* allowed time = 3 x expected     */
#define TIMEOUT_MARGIN_S        (2.0f)    /*                 + 2 s           */

typedef struct
{
    speed_pid_t pid;
    int32_t     stall_ref;      /* count when the stall timer restarted  */
    uint32_t    stall_ms;       /* driven without progress               */
    uint32_t    wrong_ms;       /* turning against the command           */
    uint32_t    glitch_seen;    /* glitches already counted              */
    int32_t     glitch_bucket;  /* leaky bucket of recent glitches       */
} wheel_ctrl_t;

static wheel_ctrl_t   g_wheel[WHEEL_COUNT];
static motion_state_t g_state = MOTION_IDLE;
static motion_fault_t g_fault = MOTION_FAULT_NONE;
static bool           g_b_busy = false;
static float          g_max_speed_mps = 1.0f;
static float          g_lead_drive_m = 0.0f;
static float          g_lead_turn_m = 0.0f;
static uint32_t       g_still_ms = 0U;

/* VELOCITY CONTROL */
static float          g_vel_target[WHEEL_COUNT];
static uint32_t       g_vel_stamp_us = 0U;

/* DRIVE / TURN */
static motion_state_t g_mv_kind = MOTION_DRIVE;
static int32_t        g_mv_sign[WHEEL_COUNT];
static int32_t        g_mv_start[WHEEL_COUNT];
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

/* true once 'now' is past 'deadline' (wrap-safe) */
static bool time_after(uint32_t now_us, uint32_t deadline_us)
{
    return ((int32_t)(now_us - deadline_us) > 0);
}

static bool is_active(motion_state_t s)
{
    return (s == MOTION_VELOCITY) || (s == MOTION_DRIVE) || (s == MOTION_TURN);
}

static void outputs_safe(void)
{
    uint32_t i;

    motor_control_brake_all();
    for (i = 0U; i < (uint32_t)WHEEL_COUNT; i++)
    {
        speed_pid_reset(&g_wheel[i].pid);
    }
}

static void raise_fault(motion_fault_t fault)
{
    outputs_safe();
    g_fault = fault;
    g_state = MOTION_FAULT;
}

static void clear_checks(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)WHEEL_COUNT; i++)
    {
        g_wheel[i].stall_ref = wheel_encoder_count((wheel_id_t)i);
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
    int32_t count = wheel_encoder_count(wheel);
    float   cmd = speed_pid_setpoint(&p_w->pid);
    float   speed = motion_estimation_wheel_speed(wheel);

    /* Stall / no progress: driven, but the encoder has not moved on by at
     * least STALL_MIN_PROGRESS counts (wheel jammed, encoder unplugged, or
     * one encoder wire broken: a single channel only flickers +1 / -1) */
    if ((abs_f(cmd) < CMD_ACTIVE_MPS)
        || (abs_i(count - p_w->stall_ref) >= STALL_MIN_PROGRESS))
    {
        p_w->stall_ref = count;
        p_w->stall_ms = 0U;
    }
    else
    {
        p_w->stall_ms += dt_ms;
    }

    /* Direction: clearly turning against the command (motor or encoder
     * reversed: the controller would otherwise run away) */
    if ((abs_f(cmd) > DIRECTION_MIN_MPS) && (abs_f(speed) > DIRECTION_MIN_MPS)
        && ((cmd > 0.0f) != (speed > 0.0f)))
    {
        p_w->wrong_ms += dt_ms;
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
    if (p_w->glitch_bucket > ENCODER_GLITCH_LIMIT)
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

        p_w->glitch_bucket += (int32_t)(seen - p_w->glitch_seen);
        p_w->glitch_seen = seen;
        if (p_w->glitch_bucket > (2 * ENCODER_GLITCH_LIMIT))
        {
            p_w->glitch_bucket = 2 * ENCODER_GLITCH_LIMIT;
        }
        if (p_w->glitch_bucket > 0)
        {
            p_w->glitch_bucket--;
        }
    }
}

/* Distance moved along each wheel's commanded direction */
static float move_wheel_progress(wheel_id_t wheel)
{
    return (float)(g_mv_sign[wheel]
                   * (wheel_encoder_count(wheel) - g_mv_start[wheel]))
           * motion_estimation_scale();
}

/* DRIVE / TURN step. Returns true when the brake point is reached. */
static bool move_step(float dt_s)
{
    float lead = (g_mv_kind == MOTION_TURN) ? g_lead_turn_m : g_lead_drive_m;
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

    wheel_drive(WHEEL_LEFT,
                (float)g_mv_sign[WHEEL_LEFT] * max_f(v - corr, 0.0f), dt_s);
    wheel_drive(WHEEL_RIGHT,
                (float)g_mv_sign[WHEEL_RIGHT] * max_f(v + corr, 0.0f), dt_s);

    return false;
}

/* ---- Interface ----------------------------------------------------------- */

void motion_controller_init(float max_speed_mps, float stop_lead_drive_m,
                            float stop_lead_turn_m)
{
    float    kff;
    uint32_t i;

    g_max_speed_mps = (max_speed_mps > 0.0f) ? max_speed_mps : 1.0f;
    g_lead_drive_m = stop_lead_drive_m;
    g_lead_turn_m = stop_lead_turn_m;
    kff = 1.0f / g_max_speed_mps;

    for (i = 0U; i < (uint32_t)WHEEL_COUNT; i++)
    {
        speed_pid_init(&g_wheel[i].pid, SPEED_KP_N * kff, SPEED_KI_N * kff,
                       SPEED_KD_N * kff, kff, ACCEL_MPS2);
        g_wheel[i].glitch_seen = wheel_encoder_glitches((wheel_id_t)i);
        g_wheel[i].glitch_bucket = 0;
        g_vel_target[i] = 0.0f;
    }
    clear_checks();
    g_state = MOTION_IDLE;
    g_fault = MOTION_FAULT_NONE;
    outputs_safe();
}

void motion_controller_step(float dt_s, uint32_t now_us)
{
    uint32_t       dt_ms = (uint32_t)((dt_s * 1000.0f) + 0.5f);
    motion_fault_t fault_l;
    motion_fault_t fault_r;

    track_glitches();
    if ((abs_f(motion_estimation_wheel_speed(WHEEL_LEFT)) < STILL_SPEED_MPS)
        && (abs_f(motion_estimation_wheel_speed(WHEEL_RIGHT))
            < STILL_SPEED_MPS))
    {
        if (g_still_ms < STILL_MS_MAX)
        {
            g_still_ms += dt_ms;
        }
    }
    else
    {
        g_still_ms = 0U;
    }

    if (g_b_busy)
    {
        return;                         /* calibration drives the motors */
    }

    switch (g_state)
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
        if (move_step(dt_s))
        {
            outputs_safe();
            g_state = MOTION_COMPLETE;

            return;
        }
        if (time_after(now_us, g_mv_deadline_us))
        {
            raise_fault(MOTION_FAULT_TIMEOUT);

            return;
        }
        break;

    default:                            /* IDLE, COMPLETE, FAULT */
        outputs_safe();

        return;
    }

    /* Active-state feedback checks (both wheels, to keep the timers right) */
    fault_l = wheel_check(WHEEL_LEFT, dt_ms);
    fault_r = wheel_check(WHEEL_RIGHT, dt_ms);
    if (fault_l != MOTION_FAULT_NONE)
    {
        raise_fault(fault_l);
    }
    else if (fault_r != MOTION_FAULT_NONE)
    {
        raise_fault(fault_r);
    }
    else
    {
        /* no fault */
    }
}

void motion_controller_make_safe(void)
{
    if (!is_active(g_state) && !g_b_busy)
    {
        outputs_safe();
    }
}

motion_result_t motion_controller_request_velocity(float left_mps,
                                                   float right_mps,
                                                   uint32_t now_us)
{
    float cap = motion_controller_speed_cap();

    if (g_state == MOTION_FAULT)
    {
        return MOTION_ERR_FAULT;
    }
    if (g_b_busy || ((g_state != MOTION_IDLE) && (g_state != MOTION_VELOCITY)))
    {
        return MOTION_ERR_BUSY;
    }
    g_vel_target[WHEEL_LEFT]  = max_f(min_f(left_mps, cap), -cap);
    g_vel_target[WHEEL_RIGHT] = max_f(min_f(right_mps, cap), -cap);
    g_vel_stamp_us = now_us;
    if (g_state == MOTION_IDLE)
    {
        clear_checks();
        g_state = MOTION_VELOCITY;
    }

    return MOTION_OK;
}

motion_result_t motion_controller_request_move(motion_state_t kind,
                                               int32_t sign_left,
                                               int32_t sign_right,
                                               float dist_m, float speed_mps,
                                               uint32_t now_us)
{
    float allowed_s;

    if (g_state == MOTION_FAULT)
    {
        return MOTION_ERR_FAULT;
    }
    if (g_b_busy || (g_state != MOTION_IDLE))
    {
        return MOTION_ERR_BUSY;
    }

    g_mv_kind = kind;
    g_mv_sign[WHEEL_LEFT] = sign_left;
    g_mv_sign[WHEEL_RIGHT] = sign_right;
    g_mv_start[WHEEL_LEFT] = wheel_encoder_count(WHEEL_LEFT);
    g_mv_start[WHEEL_RIGHT] = wheel_encoder_count(WHEEL_RIGHT);
    g_mv_dist_m = dist_m;
    g_mv_speed_mps = min_f(speed_mps, motion_controller_speed_cap());
    allowed_s = ((dist_m / g_mv_speed_mps) * TIMEOUT_FACTOR) + TIMEOUT_MARGIN_S;
    g_mv_deadline_us = now_us + (uint32_t)(allowed_s * 1000000.0f);
    clear_checks();
    g_state = (dist_m > 0.0f) ? kind : MOTION_COMPLETE;  /* zero length: done */

    return MOTION_OK;
}

void motion_controller_navigation_timeout(void)
{
    raise_fault(MOTION_FAULT_STALE_COMMAND);
}

void motion_controller_stop(void)
{
    if (g_state != MOTION_FAULT)
    {
        g_state = MOTION_IDLE;          /* the motion task brakes at once */
    }
}

bool motion_controller_reset(void)
{
    if (g_state != MOTION_FAULT)
    {
        return true;
    }
    if ((g_still_ms >= RESET_STILL_MS)
        && (g_wheel[WHEEL_LEFT].glitch_bucket <= (ENCODER_GLITCH_LIMIT / 2))
        && (g_wheel[WHEEL_RIGHT].glitch_bucket <= (ENCODER_GLITCH_LIMIT / 2)))
    {
        g_fault = MOTION_FAULT_NONE;
        g_state = MOTION_IDLE;

        return true;
    }

    return false;
}

motion_state_t motion_controller_poll(void)
{
    motion_state_t s = g_state;

    if (s == MOTION_COMPLETE)
    {
        g_state = MOTION_IDLE;          /* completion reported */
    }

    return s;
}

motion_state_t motion_controller_state(void)
{
    return g_state;
}

motion_fault_t motion_controller_fault(void)
{
    return g_fault;
}

float motion_controller_speed_cap(void)
{
    return SPEED_LIMIT_FRACTION * g_max_speed_mps;
}

void motion_controller_set_busy(bool b_busy)
{
    g_b_busy = b_busy;
}

bool motion_controller_is_busy(void)
{
    return g_b_busy;
}
