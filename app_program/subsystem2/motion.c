/*
 * File: motion.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Public motion subsystem interface (thread-safe front end) and the
 *          robot setup values.
 *
 * Requests from other tasks only change the statechart under the Buddy 2
 * mutex; the motors are driven by the motion task alone (single PWM owner).
 */

#include "motion.h"
#include "motion_lease.h"
#include "motion_controller.h"
#include "motion_estimation.h"
#include "motor_control.h"
#include "wheel_encoder.h"
#include "subsystem2.h"

/* ========================================================================== */
/* Robot setup                                                                */
/* ========================================================================== */

#define WHEEL_DIAMETER_M        (0.061f)

/* MEASURE: centre of the left tyre to centre of the right tyre. If turns come
 * up short, increase this slightly; if they overshoot, decrease it. */
#define TRACK_WIDTH_M           (0.130f)

/* CALIBRATE: values from calibration (motion_get_calibration()), or from the
 * motor datasheet: counts per wheel turn = encoder pulses per motor turn x 4
 * x gear ratio. Moves are rejected while ENCODER_COUNTS_PER_REV is 0. */
#define ENCODER_COUNTS_PER_REV  (2000.0f)
#define MOTOR_MAX_SPEED_MPS     (0.30f)     /* wheel speed at full power */
#define M1_INVERTED             (false)     /* left DC motor             */
#define M2_INVERTED             (false)     /* right DC motor            */
#define ENC_L_INVERTED          (false)     /* left wheel encoder        */
#define ENC_R_INVERTED          (false)     /* right wheel encoder       */

/* Stop this far (per wheel) before the target, to allow for the robot
 * coasting after the brakes go on: the average overshoot measured on the
 * robot (distance once stopped minus target). 0 = no correction. */
#define STOP_LEAD_DRIVE_M       (0.000f)
#define STOP_LEAD_TURN_M        (0.000f)

/* Default speeds (brief API and manoeuvres) */
#define DEFAULT_DRIVE_SPEED_MPS (0.20f)     /* straight moves            */
#define DEFAULT_TURN_SPEED_MPS  (0.15f)     /* wheel speed during turns  */
#define MANOEUVRE_STRAIGHT_M    (0.05f)     /* how far Straight drives   */

/* ========================================================================== */

#define PI_F                    (3.14159265f)
#define DEG_PER_TURN            (360.0f)
#define US_PER_S                (1000000.0f)
#define PERIOD_US               (S2_PERIOD_MS * 1000U)
#define DT_MIN_S                (0.002f)
#define DT_MAX_S                (0.050f)

/* Calibration */
#define CAL_TURNS               (10)        /* hand turns per wheel           */
#define CAL_MIN_COUNTS          (40)        /* min counts per encoder         */
#define CAL_MOVE_COUNTS         (2)         /* larger change = wheel moving   */
#define CAL_STILL_US            (3000000U)  /* wheels still: step 1 done      */
#define CAL_STEP1_TIMEOUT_US    (120000000U)
#define CAL_POLL_MS             (10)
#define CAL_LIFT_MS             (5000)      /* time to lift the wheels        */
#define CAL_SPINUP_MS           (600)
#define CAL_MEASURE_MS          (1000)
#define CAL_SETTLE_MS           (1000)
#define CAL_CPR_TOLERANCE       (0.05f)     /* wheels agree within 5 %        */
#define CAL_MIN_SPEED_MPS       (0.02f)

static ID                   g_mutex_id = 0;
static ID                   g_task_id = 0;
static uint32_t             g_last_step_us = 0U;
static volatile bool        g_b_cal_request = false;
static volatile bool        g_b_cal_running = false;
static volatile bool        g_b_cal_done = false;
static motion_calibration_t g_cal_result;

static motion_feedback_t g_feedback;
static bool g_feedback_ready;
static bool g_navigation_lease;
static uint32_t g_navigation_refresh_us;

/* ---- Helpers ------------------------------------------------------------ */

static uint32_t time_now_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

static void motion_lock(void)
{
    (void)tk_loc_mtx(g_mutex_id, TMO_FEVR);
}

static void motion_unlock(void)
{
    (void)tk_unl_mtx(g_mutex_id);
}

static float abs_f(float x)
{
    return (x < 0.0f) ? -x : x;
}

static bool is_number(float x)
{
    return (x == x) && ((x - x) == 0.0f);   /* rejects NaN and infinity */
}

static float m_per_count(float counts_per_rev)
{
    return (counts_per_rev > 0.0f)
           ? (PI_F * WHEEL_DIAMETER_M / counts_per_rev)
           : 0.0f;
}

/* ---- Lifecycle ----------------------------------------------------------- */

int32_t motion_create(void)
{
    T_CMTX cmtx = { 0 };

    cmtx.mtxatr = TA_INHERIT;
    g_mutex_id = tk_cre_mtx(&cmtx);

    return (g_mutex_id < E_OK) ? (int32_t)g_mutex_id : (int32_t)E_OK;
}

void motion_attach_task(int32_t task_id)
{
    g_task_id = (ID)task_id;
}

void motion_init(void)
{
    int32_t er;

    motor_control_init(M1_INVERTED, M2_INVERTED);
    er = wheel_encoder_init(ENC_L_INVERTED, ENC_R_INVERTED);
    motion_estimation_init(m_per_count(ENCODER_COUNTS_PER_REV), TRACK_WIDTH_M);

    motion_lock();
    g_feedback.epoch++;
    g_feedback.valid = false;
    g_feedback_ready = er >= (int32_t)E_OK;
    g_navigation_lease = false;
    motion_controller_init(MOTOR_MAX_SPEED_MPS, STOP_LEAD_DRIVE_M,
                           STOP_LEAD_TURN_M);
    if (er < (int32_t)E_OK)
    {
        /* Encoder interrupt not available: without feedback the robot must
         * not move, so every request is rejected (MOTION_ERR_BUSY) */
        motion_controller_set_busy(true);
    }
    motion_unlock();
    g_last_step_us = time_now_us();
}

static void run_calibration(motion_calibration_t *p_out);

void motion_process(void)
{
    uint32_t now = time_now_us();
    uint32_t elapsed = now - g_last_step_us;
    bool     b_cal;
    float    dt_s;

    motion_lock();
    b_cal = g_b_cal_request;
    if (b_cal)
    {
        g_b_cal_request = false;
        g_b_cal_running = true;
        g_feedback.valid = false;
        motion_controller_set_busy(true);
    }
    motion_unlock();

    if (b_cal)
    {
        motion_calibration_t result;

        run_calibration(&result);
        motion_estimation_init(m_per_count(ENCODER_COUNTS_PER_REV),
                               TRACK_WIDTH_M);
        motion_lock();
        g_feedback.epoch++;
        g_feedback.valid = false;
        g_cal_result = result;
        g_b_cal_done = true;
        g_b_cal_running = false;
        motion_controller_set_busy(false);
        motion_unlock();
        (void)tk_can_wup(TSK_SELF);     /* drop ticks queued meanwhile */
        g_last_step_us = time_now_us();

        return;
    }

    if (elapsed < (PERIOD_US / 2U))
    {
        /* Early wake-up from stop(): brake now, control on the next tick */
        motion_lock();
        motion_controller_make_safe();
        motion_unlock();

        return;
    }

    g_last_step_us = now;
    dt_s = (float)elapsed / US_PER_S;
    if (dt_s < DT_MIN_S)
    {
        dt_s = DT_MIN_S;
    }
    if (dt_s > DT_MAX_S)
    {
        dt_s = DT_MAX_S;
    }

    motion_lock();
    motion_estimation_update(dt_s);
    if (motion_lease_expired(g_navigation_lease, now, g_navigation_refresh_us))
    {
        motion_controller_navigation_timeout();
        g_navigation_lease = false;
    }
    motion_controller_step(dt_s, now);
    g_feedback.centre_m = motion_estimation_robot_distance();
    g_feedback.timestamp_us = now;
    g_feedback.valid = g_feedback_ready && motion_is_calibrated()
                       && !g_b_cal_running && !g_b_cal_request;
    g_feedback.braked = (motor_control_duty(WHEEL_LEFT) == 0.0f)
                       && (motor_control_duty(WHEEL_RIGHT) == 0.0f);
    motion_unlock();
}

/* ---- Requests ------------------------------------------------------------ */

motion_result_t motion_request_velocity(float left_mps, float right_mps)
{
    motion_result_t res;

    if (!motion_is_calibrated())
    {
        return MOTION_ERR_NOT_CALIBRATED;
    }
    if (!is_number(left_mps) || !is_number(right_mps))
    {
        return MOTION_ERR_BAD_ARG;
    }
    motion_lock();
    res = motion_controller_request_velocity(left_mps, right_mps,
                                             time_now_us());
    motion_unlock();

    return res;
}

static motion_result_t request_move(motion_state_t kind, int32_t sign_left,
                                    int32_t sign_right, float dist_m,
                                    float speed_mps)
{
    motion_result_t res;

    motion_lock();
    res = motion_controller_request_move(kind, sign_left, sign_right, dist_m,
                                         speed_mps, time_now_us());
    motion_unlock();

    return res;
}

motion_result_t motion_request_drive(float meters, float speed_mps)
{
    int32_t sign;

    if (!motion_is_calibrated())
    {
        return MOTION_ERR_NOT_CALIBRATED;
    }
    if (!is_number(meters) || !is_number(speed_mps) || (speed_mps <= 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }
    sign = (meters >= 0.0f) ? 1 : -1;

    return request_move(MOTION_DRIVE, sign, sign, abs_f(meters), speed_mps);
}

motion_result_t motion_request_turn(float degrees, float speed_mps)
{
    float   arc_m;
    int32_t sign_left;

    if (!motion_is_calibrated())
    {
        return MOTION_ERR_NOT_CALIBRATED;
    }
    if (!is_number(degrees) || !is_number(speed_mps) || (speed_mps <= 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }
    /* On the spot each wheel travels along a circle the size of the track */
    arc_m = abs_f(degrees) / DEG_PER_TURN * PI_F * TRACK_WIDTH_M;
    sign_left = (degrees > 0.0f) ? -1 : 1;          /* left turn: left back */

    return request_move(MOTION_TURN, sign_left, -sign_left, arc_m, speed_mps);
}

motion_result_t motion_request_u_turn(float speed_mps)
{
    return motion_request_turn(180.0f, speed_mps);
}

motion_result_t motion_request_manoeuvre(manoeuvre_t m)
{
    motion_result_t res;

    switch (m)
    {
    case MANOEUVRE_LEFT:
        res = motion_request_turn(90.0f, DEFAULT_TURN_SPEED_MPS);
        break;
    case MANOEUVRE_RIGHT:
        res = motion_request_turn(-90.0f, DEFAULT_TURN_SPEED_MPS);
        break;
    case MANOEUVRE_STRAIGHT:
        res = motion_request_drive(MANOEUVRE_STRAIGHT_M,
                                   DEFAULT_DRIVE_SPEED_MPS);
        break;
    case MANOEUVRE_U_TURN:
        res = motion_request_turn(180.0f, DEFAULT_TURN_SPEED_MPS);
        break;
    default:
        res = MOTION_ERR_BAD_ARG;
        break;
    }

    return res;
}

/* ---- Brief API ----------------------------------------------------------- */

motion_result_t moveForward(float distance)
{
    if (!is_number(distance) || (distance < 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }

    return motion_request_drive(distance, DEFAULT_DRIVE_SPEED_MPS);
}

motion_result_t moveBackward(float distance)
{
    if (!is_number(distance) || (distance < 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }

    return motion_request_drive(-distance, DEFAULT_DRIVE_SPEED_MPS);
}

motion_result_t turnLeft(float angle)
{
    if (!is_number(angle) || (angle < 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }

    return motion_request_turn(angle, DEFAULT_TURN_SPEED_MPS);
}

motion_result_t turnRight(float angle)
{
    if (!is_number(angle) || (angle < 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }

    return motion_request_turn(-angle, DEFAULT_TURN_SPEED_MPS);
}

void stop(void)
{
    motion_lock();
    motion_controller_stop();
    motion_unlock();
    if (g_task_id > 0)
    {
        (void)tk_wup_tsk(g_task_id);    /* motion task brakes immediately */
    }
}

bool motion_reset(void)
{
    bool b_idle;

    motion_lock();
    b_idle = motion_controller_reset();
    motion_unlock();

    return b_idle;
}

/* ---- Status -------------------------------------------------------------- */

motion_state_t motion_poll(void)
{
    motion_state_t s;

    motion_lock();
    s = motion_controller_poll();
    motion_unlock();

    return s;
}

motion_fault_t motion_get_fault(void)
{
    motion_fault_t f;

    motion_lock();
    f = motion_controller_fault();
    motion_unlock();

    return f;
}

const char *motion_state_name(motion_state_t s)
{
    const char *p_name;

    switch (s)
    {
    case MOTION_IDLE:     p_name = "IDLE";     break;
    case MOTION_VELOCITY: p_name = "VELOCITY"; break;
    case MOTION_DRIVE:    p_name = "DRIVE";    break;
    case MOTION_TURN:     p_name = "TURN";     break;
    case MOTION_COMPLETE: p_name = "COMPLETE"; break;
    case MOTION_FAULT:    p_name = "FAULT";    break;
    default:              p_name = "?";        break;
    }

    return p_name;
}

const char *motion_fault_name(motion_fault_t f)
{
    const char *p_name;

    switch (f)
    {
    case MOTION_FAULT_NONE:
        p_name = "none";
        break;
    case MOTION_FAULT_STALL:
        p_name = "stall / no encoder progress "
                 "(wheel jammed, encoder unplugged or wire broken)";
        break;
    case MOTION_FAULT_STALE_COMMAND:
        p_name = "stale command (no velocity request in time)";
        break;
    case MOTION_FAULT_ENCODER:
        p_name = "encoder signal invalid / noisy";
        break;
    case MOTION_FAULT_DIRECTION:
        p_name = "wheel turning against command (motor or encoder reversed?)";
        break;
    case MOTION_FAULT_TIMEOUT:
        p_name = "drive/turn timed out";
        break;
    default:
        p_name = "?";
        break;
    }

    return p_name;
}

const char *manoeuvre_name(manoeuvre_t m)
{
    const char *p_name;

    switch (m)
    {
    case MANOEUVRE_LEFT:     p_name = "Left";     break;
    case MANOEUVRE_RIGHT:    p_name = "Right";    break;
    case MANOEUVRE_STRAIGHT: p_name = "Straight"; break;
    case MANOEUVRE_U_TURN:   p_name = "U-turn";   break;
    default:                 p_name = "?";        break;
    }

    return p_name;
}

/* ---- Estimates -------------------------------------------------------- */

int32_t enc_l_count(void)
{
    return wheel_encoder_count(WHEEL_LEFT);
}

int32_t enc_r_count(void)
{
    return wheel_encoder_count(WHEEL_RIGHT);
}

float wheel_distance_left_m(void)
{
    return motion_estimation_wheel_distance(WHEEL_LEFT);
}

float wheel_distance_right_m(void)
{
    return motion_estimation_wheel_distance(WHEEL_RIGHT);
}

float wheel_speed_left_mps(void)
{
    return motion_estimation_wheel_speed(WHEEL_LEFT);
}

float wheel_speed_right_mps(void)
{
    return motion_estimation_wheel_speed(WHEEL_RIGHT);
}

float robot_distance_m(void)
{
    return motion_estimation_robot_distance();
}

float robot_speed_mps(void)
{
    return motion_estimation_robot_speed();
}

float robot_heading_deg(void)
{
    return motion_estimation_robot_heading_deg();
}

/* ========================================================================== */
/* Calibration (one-off setup, outside the statechart)                        */
/* ========================================================================== */

bool motion_get_feedback(motion_feedback_t *feedback)
{
    if ((feedback == 0) || (g_mutex_id <= 0))
    {
        return false;
    }
    if (tk_loc_mtx(g_mutex_id, TMO_FEVR) != E_OK)
    {
        return false;
    }
    *feedback = g_feedback;
    feedback->state = motion_controller_state();
    feedback->fault = motion_controller_fault();
    feedback->valid = feedback->valid && !g_b_cal_request && !g_b_cal_running
                      && !motion_controller_is_busy();
    motion_unlock();
    return true;
}
void motion_navigation_heartbeat(bool enabled)
{
    motion_lock();
    g_navigation_lease = enabled;
    if (enabled)
    {
        g_navigation_refresh_us = time_now_us();
    }
    motion_unlock();
}

bool motion_is_calibrated(void)
{
    return (ENCODER_COUNTS_PER_REV > 0.0f);
}

motion_result_t motion_request_calibration(void)
{
    motion_result_t res = MOTION_OK;

    motion_lock();
    if (g_navigation_lease)
    {
        res = MOTION_ERR_BUSY;
    }
    else if (motion_controller_state() == MOTION_FAULT)
    {
        res = MOTION_ERR_FAULT;
    }
    else if ((motion_controller_state() != MOTION_IDLE) || g_b_cal_request
             || motion_controller_is_busy())
    {
        res = MOTION_ERR_BUSY;
    }
    else
    {
        g_b_cal_done = false;
        g_b_cal_request = true;
    }
    motion_unlock();

    return res;
}

bool motion_is_calibrating(void)
{
    bool b_running;

    motion_lock();
    b_running = g_b_cal_request || g_b_cal_running;
    motion_unlock();

    return b_running;
}

bool motion_get_calibration(motion_calibration_t *p_out)
{
    bool b_done;

    motion_lock();
    b_done = g_b_cal_done;
    if (b_done)
    {
        *p_out = g_cal_result;
    }
    motion_unlock();

    return b_done;
}

/* Calibration step 2: run one motor alone at full power (wheels off the
 * ground) and measure how far each encoder moved */
static void cal_spin(wheel_id_t motor, int32_t *p_d_l, int32_t *p_d_r,
                     float *p_secs)
{
    int32_t  l0;
    int32_t  r0;
    uint32_t t0;

    motor_control_set(motor, 1.0f);
    (void)tk_dly_tsk(CAL_SPINUP_MS);
    l0 = wheel_encoder_count(WHEEL_LEFT);
    r0 = wheel_encoder_count(WHEEL_RIGHT);
    t0 = time_now_us();
    (void)tk_dly_tsk(CAL_MEASURE_MS);
    *p_d_l = wheel_encoder_count(WHEEL_LEFT) - l0;
    *p_d_r = wheel_encoder_count(WHEEL_RIGHT) - r0;
    *p_secs = (float)(time_now_us() - t0) / US_PER_S;
    motor_control_brake_all();
    (void)tk_dly_tsk(CAL_SETTLE_MS);
}

static int32_t abs_i(int32_t x)
{
    return (x < 0) ? -x : x;
}

static void run_calibration(motion_calibration_t *p_out)
{
    const int32_t l0 = wheel_encoder_count(WHEEL_LEFT);
    const int32_t r0 = wheel_encoder_count(WHEEL_RIGHT);
    const uint32_t start = time_now_us();
    uint32_t last_move = start;
    int32_t  ref_l = l0;
    int32_t  ref_r = r0;
    int32_t  tl;
    int32_t  tr;
    float    cpr_l;
    float    cpr_r;
    float    cpr;
    float    scale;
    int32_t  m1_l;
    int32_t  m1_r;
    int32_t  m2_l;
    int32_t  m2_r;
    float    s1;
    float    s2;
    float    vl;
    float    vr;
    bool     b_lenc_flip;
    bool     b_renc_flip;

    *p_out = (motion_calibration_t){ 0 };
    motor_control_brake_all();

    /* Step 1: counts per wheel turn and encoder direction (hand turning).
     * Gives up if an encoder is not counting (unplugged, or a broken wire:
     * a single working channel only flickers +1 / -1). */
    for (;;)
    {
        int32_t  l = wheel_encoder_count(WHEEL_LEFT);
        int32_t  r = wheel_encoder_count(WHEEL_RIGHT);
        uint32_t now = time_now_us();
        bool     b_l_ok = (abs_i(l - l0) > CAL_MIN_COUNTS);
        bool     b_r_ok = (abs_i(r - r0) > CAL_MIN_COUNTS);

        if ((abs_i(l - ref_l) > CAL_MOVE_COUNTS)
            || (abs_i(r - ref_r) > CAL_MOVE_COUNTS))
        {
            ref_l = l;
            ref_r = r;
            last_move = now;
        }
        if (b_l_ok && b_r_ok && ((now - last_move) > CAL_STILL_US))
        {
            break;
        }
        if ((now - start) > CAL_STEP1_TIMEOUT_US)
        {
            p_out->b_enc_l_ok = b_l_ok;
            p_out->b_enc_r_ok = b_r_ok;

            return;
        }
        (void)tk_dly_tsk(CAL_POLL_MS);
    }
    p_out->b_enc_l_ok = true;
    p_out->b_enc_r_ok = true;

    tl = wheel_encoder_count(WHEEL_LEFT) - l0;
    tr = wheel_encoder_count(WHEEL_RIGHT) - r0;
    b_lenc_flip = (tl < 0);
    b_renc_flip = (tr < 0);
    cpr_l = (float)abs_i(tl) / (float)CAL_TURNS;
    cpr_r = (float)abs_i(tr) / (float)CAL_TURNS;
    cpr = 0.5f * (cpr_l + cpr_r);
    scale = m_per_count(cpr);

    /* Use the step 1 results for step 2 */
    wheel_encoder_set_inverted(WHEEL_LEFT,  ENC_L_INVERTED != b_lenc_flip);
    wheel_encoder_set_inverted(WHEEL_RIGHT, ENC_R_INVERTED != b_renc_flip);

    /* Step 2: top speed, motor direction and encoder ports, one motor at a
     * time, wheels off the ground */
    (void)tk_dly_tsk(CAL_LIFT_MS);
    cal_spin(WHEEL_LEFT,  &m1_l, &m1_r, &s1);
    cal_spin(WHEEL_RIGHT, &m2_l, &m2_r, &s2);

    /* Each motor should turn its own encoder; if the left motor moves ENC_R
     * and the right motor moves ENC_L, the encoder cables are swapped */
    p_out->b_encoders_swapped = (abs_i(m1_r) > abs_i(m1_l))
                                && (abs_i(m2_l) > abs_i(m2_r));

    vl = (float)m1_l * scale / s1;
    vr = (float)m2_r * scale / s2;

    p_out->counts_per_rev_l = cpr_l;
    p_out->counts_per_rev_r = cpr_r;
    p_out->counts_per_rev   = cpr;
    p_out->top_speed_l_mps  = abs_f(vl);
    p_out->top_speed_r_mps  = abs_f(vr);
    p_out->max_speed_mps    = (abs_f(vl) < abs_f(vr)) ? abs_f(vl) : abs_f(vr);
    p_out->b_m1_inverted    = (M1_INVERTED != (vl < 0.0f));
    p_out->b_m2_inverted    = (M2_INVERTED != (vr < 0.0f));
    p_out->b_enc_l_inverted = (ENC_L_INVERTED != b_lenc_flip);
    p_out->b_enc_r_inverted = (ENC_R_INVERTED != b_renc_flip);

    /* Undo the temporary change: still uncalibrated until values are set */
    wheel_encoder_set_inverted(WHEEL_LEFT,  ENC_L_INVERTED);
    wheel_encoder_set_inverted(WHEEL_RIGHT, ENC_R_INVERTED);
    motor_control_brake_all();

    p_out->b_valid = !p_out->b_encoders_swapped
                     && (abs_f(cpr_l - cpr_r) <= (CAL_CPR_TOLERANCE * cpr))
                     && (p_out->top_speed_l_mps >= CAL_MIN_SPEED_MPS)
                     && (p_out->top_speed_r_mps >= CAL_MIN_SPEED_MPS);
}
