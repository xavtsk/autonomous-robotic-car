/** @file motion.c
 * @brief Provide synchronized motion requests and calibrated robot settings.
 * @note Requests update the statechart under the Buddy 2 mutex. Only the
 * motion task writes motor outputs.
 * @see app_program/subsystem2/README.md for the request and feedback path.
 *
 * @par Ownership
 * Buddy 2 - Motion Control
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 */

#include "motion.h"
#include "../app_types.h"
#include "motion_lease.h"
#include "ctrl_motion.h"
#include "motion_pack.h"
#include "motion_estimation.h"
#include "motor_control.h"
#include "wheel_encoder.h"
#include "buddy2.h"
#include "../monotonic_clock.h"

#if defined(MOTION_STORAGE_PROBE)
#include "motion_storage_main.h"
#endif

/* ========================================================================== */
/* These are the robot's measured settings. */
/* ========================================================================== */

#define WHEEL_DIAMETER_M (0.0654f)

/* Track width measures from one tyre centre to the other. This is the
 * baseline value documented in DATA_FORMAT.md. */
#define TRACK_WIDTH_M (0.116f)

/* Use the measured count per wheel revolution in DATA_FORMAT.md. The
 * theoretical count is encoder pulses per motor turn times four for
 * quadrature edges, times the gearbox ratio. Zero disables move requests. */
#define ENCODER_COUNTS_PER_REV (2534.0f)
#define MOTOR_MAX_SPEED_MPS (0.884f) /* This is the full-power speed. */
#define M1_INVERTED (false)          /* Use normal left-motor polarity. */
#define M2_INVERTED (false)          /* Use normal right-motor polarity. */
#define ENC_L_INVERTED (false)       /* Use normal left-encoder polarity. */
#define ENC_R_INVERTED (false)       /* Use normal right-encoder polarity. */

/* Stop this far (per wheel) before the target, to allow for the robot
 * coasting after the brakes go on: the average overshoot measured on the
 * robot (distance once stopped minus target). 0 = no correction. */
#define STOP_LEAD_DRIVE_M (0.000f)
#define STOP_LEAD_TURN_M (0.000f)

/* Use these defaults for simple manoeuvres. */
#define DEFAULT_DRIVE_SPEED_MPS (0.20f) /* Set straight speed in m/s. */
#define DEFAULT_TURN_SPEED_MPS (0.15f)  /* Set the turn wheel speed. */
#define MANOEUVRE_STRAIGHT_M (0.05f)    /* Set straight travel in metres. */

/* ========================================================================== */

#define PI_F (3.14159265f)
#define DEG_PER_TURN (360.0f)
#define US_PER_S (1000000.0f)
#define PERIOD_US (S2_PERIOD_MS * 1000U)
#define DT_MIN_S (0.002f)
#define DT_MAX_S (0.050f)

/* Calibration */
#define CAL_TURNS (10)          /* Turn each wheel by hand. */
#define CAL_MIN_COUNTS (40)     /* Require 40 counts per encoder. */
#define CAL_MOVE_COUNTS (2)     /* Larger changes indicate motion. */
#define CAL_STILL_US (3000000U) /* Step 1 ends when wheels are still. */
#define CAL_STEP1_TIMEOUT_US (120000000U)
#define CAL_POLL_MS (10)
#define CAL_LIFT_MS (5000) /* Allow time to lift both wheels. */
#define CAL_SPINUP_MS (600)
#define CAL_MEASURE_MS (1000)
#define CAL_SETTLE_MS (1000)
#define CAL_CPR_TOLERANCE (0.05f) /* Require agreement within 5%. */
#define CAL_MIN_SPEED_MPS (0.02f)

/* Report queue: message buffer of MOTION_REPORT_QUEUE_LEN packed 16-bit
 * reports. The kernel stores each as a 4-byte header plus the message
 * rounded up to 4 bytes: 8 bytes a report. */
#define REPORT_MSG_SZ ((INT)sizeof(uint16_t))
#define REPORT_SLOT_SZ (sizeof(INT) + 4U)
#define REPORT_BUF_SZ ((SZ)(MOTION_REPORT_QUEUE_LEN * REPORT_SLOT_SZ))
#define REPORT_COUNT_BITS (4U) /* 0 ... MOTION_REPORT_QUEUE_LEN (8) */

/* Calibration handshake and report-queue counters: minimum-bit fields in
 * one word, shared by the motion task and the requesting tasks. Changed
 * and read only with the Buddy 2 mutex held, so no read-modify-write of
 * the word can race. */
typedef struct
{
    uint32_t b_is_cal_requested : 1; /* The calibration request is pending. */
    uint32_t b_is_cal_running : 1;
    uint32_t b_is_cal_done : 1;      /* The result is ready to read. */
    uint32_t b_should_abort_cal : 1; /* STOP requested a calibration abort. */
    uint32_t b_is_report_lost : 1;   /* A report was dropped. */
    uint32_t report_queued : REPORT_COUNT_BITS;
    uint32_t report_peak : REPORT_COUNT_BITS;
    uint32_t b_is_feedback_valid : 1;
    uint32_t b_is_feedback_ready : 1;
    uint32_t b_has_navigation_lease : 1;
} motion_flags_t;

/* Retain the six unchanged floating-point values and nine Boolean results.
 * The public calibration view stays addressable and keeps its existing ABI.
 * Natural float alignment is retained; no unaligned float accesses are used. */
typedef struct
{
    float32_t counts_per_rev;
    float32_t max_speed_mps;
    float32_t counts_per_rev_l;
    float32_t counts_per_rev_r;
    float32_t top_speed_l_mps;
    float32_t top_speed_r_mps;
    uint16_t  b_is_valid : 1;
    uint16_t  b_is_left_motor_inverted : 1;
    uint16_t  b_is_right_motor_inverted : 1;
    uint16_t  b_is_left_enc_inverted : 1;
    uint16_t  b_is_right_enc_inverted : 1;
    uint16_t  b_is_left_enc_ok : 1;
    uint16_t  b_is_right_enc_ok : 1;
    uint16_t  b_are_encoders_swapped : 1;
    uint16_t  b_was_aborted : 1;
} motion_stored_calibration_t;

/* Other public feedback fields are derived under the mutex at read time. */
typedef struct
{
    float64_t centre_m;
    uint32_t  timestamp_us;
    uint32_t  epoch;
} motion_stored_feedback_t;

BUILD_ASSERT(sizeof(motion_stored_calibration_t) == 28U,
             motion_store_cal_size_t);
BUILD_ASSERT(sizeof(motion_stored_feedback_t) == 16U,
             motion_store_feedback_size_t);
BUILD_ASSERT(sizeof(motion_flags_t) == 4U, motion_store_flags_size_t);

static ID                          gh_motion_mutex = 0;
static ID                          gh_motion_task  = 0;
static uint32_t                    g_last_step_us  = 0U;
static motion_flags_t              g_flags         = {0};
static motion_stored_calibration_t g_cal_result;
static ID                          gh_motion_reports = 0;

static motion_stored_feedback_t g_feedback;
static uint32_t                 g_feedback_count[WHEEL_COUNT];
static uint32_t                 g_report_loss_count;
static uint32_t                 g_navigation_refresh_us;

static void motion_calibration_store (motion_stored_calibration_t * p_stored,
                                      const motion_calibration_t *  p_value);
static void
motion_calibration_expand (const motion_stored_calibration_t * p_stored,
                           motion_calibration_t *              p_value);
static uint32_t        time_now_us (void);
static void            motion_lock (void);
static void            motion_unlock (void);
static float32_t       abs_f (float32_t operand);
static bool            is_number (float32_t operand);
static float32_t       m_per_count (float32_t counts_per_rev);
static void            report_post (const motion_report_t * p_report);
static bool            report_receive (motion_report_t * p_out, TMO tmout);
static motion_result_t request_move (motion_state_t kind, int32_t sign_left,
                                     int32_t sign_right, float32_t dist_m,
                                     float32_t speed_mps);
static bool            cal_wait (int32_t value_ms);
static bool    cal_spin (wheel_encoder_id_t motor, int32_t * p_left_delta,
                         int32_t * p_right_delta, float32_t * p_secs);
static int32_t abs_i (int32_t operand);
static void    run_calibration (motion_calibration_t * p_out);

/** @brief Create the motion mutex followed by the FIFO report buffer. Return
 * the first kernel creation error or E_OK. A buffer creation failure
 * leaves the mutex allocated.
 */
int32_t motion_create (void)
{
    T_CMTX cmtx = {0};
    T_CMBF cmbf = {0};

    cmtx.mtxatr     = TA_INHERIT;
    gh_motion_mutex = tk_cre_mtx(&cmtx);

    if (gh_motion_mutex < E_OK)
    {
        return (int32_t)gh_motion_mutex;
    }

    cmbf.mbfatr       = TA_TFIFO;
    cmbf.bufsz        = REPORT_BUF_SZ;
    cmbf.maxmsz       = REPORT_MSG_SZ;
    gh_motion_reports = tk_cre_mbf(&cmbf);

    return (gh_motion_reports < E_OK) ? (int32_t)gh_motion_reports : E_OK;
}

/** @brief Record the kernel task ID to wake when a stop or cancellation needs
 * service.
 */
void motion_attach_task (int32_t task_id)
{
    gh_motion_task = (ID)task_id;
}

/** @brief Initialize motors, encoders, estimation and controller state from
 * the motion task. Advance the feedback epoch and reject motion
 * commands if encoder initialization fails.
 */
void motion_init (void)
{

    monotonic_clock_start();
    motor_control_init(M1_INVERTED, M2_INVERTED);
    const int32_t value_er = wheel_encoder_init(ENC_L_INVERTED, ENC_R_INVERTED);
    wheel_encoder_sample(); /* This task alone reads the encoder PIO. */
    motion_estimation_init(m_per_count(ENCODER_COUNTS_PER_REV), TRACK_WIDTH_M);

    motion_lock();
    g_feedback.epoch++;
    g_feedback.centre_m            = 0.0;
    g_feedback_count[0]            = wheel_encoder_bits(WHEEL_LEFT);
    g_feedback_count[1]            = wheel_encoder_bits(WHEEL_RIGHT);
    g_flags.b_is_feedback_valid    = false;
    g_flags.b_is_feedback_ready    = value_er >= E_OK;
    g_flags.b_has_navigation_lease = false;
    ctrl_motion_init(MOTOR_MAX_SPEED_MPS, STOP_LEAD_DRIVE_M, STOP_LEAD_TURN_M,
                     report_post);

    if (value_er < E_OK)
    {
        /* Reject motion requests with MOTION_ERR_BUSY when encoder PIO is
         * unavailable and the controller cannot obtain feedback.
         */
        ctrl_motion_set_busy(true);
    }

    motion_unlock();
    g_last_step_us = time_now_us();
}

/** @brief Service one owner update. A pending calibration takes precedence and
 * resets the feedback epoch when finished. On a normal update, sample
 * the encoders, enforce the navigation lease, step the controller and
 * publish timestamped feedback under the mutex.
 */
void motion_process (void)
{
    const uint32_t now     = time_now_us();
    const uint32_t elapsed = now - g_last_step_us;

    motion_lock();
    const bool b_has_calibration_request = (g_flags.b_is_cal_requested != 0U);

    if (b_has_calibration_request)
    {
        g_flags.b_is_cal_requested  = 0U;
        g_flags.b_is_cal_running    = 1U;
        g_flags.b_is_feedback_valid = false;
        ctrl_motion_set_busy(true);
    }

    motion_unlock();

    if (b_has_calibration_request)
    {
        motion_calibration_t result;

        run_calibration(&result);
        motor_control_brake_all(); /* Brake even after an abort. */
        wheel_encoder_sample();
        motion_estimation_init(m_per_count(ENCODER_COUNTS_PER_REV),
                               TRACK_WIDTH_M);
        motion_lock();
        g_feedback.epoch++;
        g_feedback.centre_m         = 0.0;
        g_feedback_count[0]         = wheel_encoder_bits(WHEEL_LEFT);
        g_feedback_count[1]         = wheel_encoder_bits(WHEEL_RIGHT);
        g_flags.b_is_feedback_valid = false;
        motion_calibration_store(&g_cal_result, &result);
        g_flags.b_is_cal_done      = 1U;
        g_flags.b_is_cal_running   = 0U;
        g_flags.b_should_abort_cal = 0U;
        ctrl_motion_set_busy(false);
        motion_unlock();
        (void)tk_can_wup(TSK_SELF); /* Drop unused ticks from the queue. */
        g_last_step_us = time_now_us();

        return;
    }

    if (elapsed < (PERIOD_US / 2U))
    {
        /* An early stop wake-up applies braking before the next control tick.
         */
        motion_lock();
        ctrl_motion_make_safe();
        motion_unlock();

        return;
    }

    g_last_step_us = now;
    /* The unsigned elapsed microseconds may round to float32_t before
     * scaling to seconds. The existing clamps bound the controller step. */
    float32_t dt_s = (float32_t)elapsed / US_PER_S;

    if (dt_s < DT_MIN_S)
    {
        dt_s = DT_MIN_S;
    }

    if (dt_s > DT_MAX_S)
    {
        dt_s = DT_MAX_S;
    }

    motion_lock();
    wheel_encoder_sample(); /* Read the PIO once per period. */
    motion_estimation_update(dt_s);

    if (motion_lease_expired(g_flags.b_has_navigation_lease, now,
                             g_navigation_refresh_us))
    {
        ctrl_motion_nav_timeout();
        g_flags.b_has_navigation_lease = false;
    }

    ctrl_motion_step(dt_s, now);
    /* Accumulate signed deltas from wrapping 20-bit counters. float64_t
     * preserves both the delta and the binary32 scale before accumulation.
     * This distance belongs to the current feedback epoch. */
    for (uint32_t wheel = 0U; wheel < (unsigned int)WHEEL_COUNT; wheel++)
    {
        const uint32_t bits = wheel_encoder_bits((wheel_encoder_id_t)wheel);
        /* clang-format off */
        g_feedback.centre_m    +=
            0.5 *
            (float64_t)wheel_encoder_delta(bits, g_feedback_count[wheel]) *
            (float64_t)motion_estimation_scale();
        g_feedback_count[wheel] = bits;
        /* clang-format on */
    }

    /* clang-format off */
    g_feedback.timestamp_us     = now;
    g_flags.b_is_feedback_valid =
        ((((g_flags.b_is_feedback_ready) && (wheel_encoder_is_running())) &&
          (motion_is_calibrated())) &&
         (!g_flags.b_is_cal_running)) &&
        (!g_flags.b_is_cal_requested);
    /* clang-format on */
    motion_unlock();
}

/* ---- Requests ------------------------------------------------------------ */

/** @brief Submit finite left and right wheel speeds in m/s under the mutex.
 * Calibration is checked before argument validity. Return the
 * controller's request result.
 */
motion_result_t motion_request_velocity (float32_t left_mps,
                                         float32_t right_mps)
{

    if (!motion_is_calibrated())
    {
        return MOTION_ERR_NOT_CALIBRATED;
    }

    if ((!is_number(left_mps)) || (!is_number(right_mps)))
    {
        return MOTION_ERR_BAD_ARG;
    }

    motion_lock();
    const motion_result_t res =
        ctrl_motion_set_velocity(left_mps, right_mps, time_now_us());
    motion_unlock();

    return res;
}

/** @brief Request signed travel in metres at a finite positive speed in m/s.
 * Negative distance drives backward. Return a calibration, argument or
 * controller result.
 */
motion_result_t motion_request_drive (float32_t meters, float32_t speed_mps)
{

    if (!motion_is_calibrated())
    {
        return MOTION_ERR_NOT_CALIBRATED;
    }

    if (((!is_number(meters)) || (!is_number(speed_mps))) ||
        (speed_mps <= 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }

    const int32_t sign = (meters >= 0.0f) ? 1 : -1;

    return request_move(MOTION_DRIVE, sign, sign, abs_f(meters), speed_mps);
}

/** @brief Translate signed degrees into equal and opposite wheel arcs.
 * Positive angles turn left. Require calibration, a finite angle and a
 * finite positive wheel speed in m/s.
 */
motion_result_t motion_request_turn (float32_t degrees, float32_t speed_mps)
{

    if (!motion_is_calibrated())
    {
        return MOTION_ERR_NOT_CALIBRATED;
    }

    if (((!is_number(degrees)) || (!is_number(speed_mps))) ||
        (speed_mps <= 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }
    /* During an in-place turn, each wheel traces a circle of track diameter. */
    const float32_t arc_m =
        abs_f(degrees) / DEG_PER_TURN * PI_F * TRACK_WIDTH_M;
    const int32_t sign_left =
        (degrees > 0.0f) ? -1 : 1; /* Reverse the left wheel for a left turn. */

    return request_move(MOTION_TURN, sign_left, -sign_left, arc_m, speed_mps);
}

/** @brief Request a positive 180-degree turn at the given wheel speed in m/s.
 * Return the turn request result.
 */
motion_result_t motion_request_u_turn (float32_t speed_mps)
{
    return motion_request_turn(180.0f, speed_mps);
}

/** @brief Submit a left turn, right turn, straight advance or U-turn using
 * configured distances and speeds. Return MOTION_ERR_BAD_ARG for an
 * unknown manoeuvre.
 */
motion_result_t motion_request_manoeuvre (motion_manoeuvre_t manoeuvre)
{
    motion_result_t res;

    switch (manoeuvre)
    {
    case MANOEUVRE_LEFT:
        res = motion_request_turn(90.0f, DEFAULT_TURN_SPEED_MPS);
    break;
    case MANOEUVRE_RIGHT:
        res = motion_request_turn(-90.0f, DEFAULT_TURN_SPEED_MPS);
    break;
    case MANOEUVRE_STRAIGHT:
        res =
            motion_request_drive(MANOEUVRE_STRAIGHT_M, DEFAULT_DRIVE_SPEED_MPS);
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

/** @brief Drive forward by a finite, nonnegative distance in metres at the
 * default speed. Reject invalid distance before attempting the
 * request.
 */
motion_result_t moveForward (float32_t distance)
{
    if ((!is_number(distance)) || (distance < 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }

    return motion_request_drive(distance, DEFAULT_DRIVE_SPEED_MPS);
}

/** @brief Drive backward by a finite, nonnegative distance in metres at the
 * default speed. Return the signed drive request's result.
 */
motion_result_t moveBackward (float32_t distance)
{
    if ((!is_number(distance)) || (distance < 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }

    return motion_request_drive(-distance, DEFAULT_DRIVE_SPEED_MPS);
}

/** @brief Request a left turn by a finite, nonnegative angle in degrees at the
 * default wheel speed.
 */
motion_result_t turnLeft (float32_t angle)
{
    if ((!is_number(angle)) || (angle < 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }

    return motion_request_turn(angle, DEFAULT_TURN_SPEED_MPS);
}

/** @brief Request a right turn by a finite, nonnegative angle in degrees at
 * the default wheel speed.
 */
motion_result_t turnRight (float32_t angle)
{
    if ((!is_number(angle)) || (angle < 0.0f))
    {
        return MOTION_ERR_BAD_ARG;
    }

    return motion_request_turn(-angle, DEFAULT_TURN_SPEED_MPS);
}

/** @brief Request a controller stop and cancel pending calibration, or flag an
 * active calibration for abort. Wake the attached motion task after
 * releasing the mutex.
 */
void stop (void)
{
    motion_lock();
    ctrl_motion_stop();

    if ((g_flags.b_is_cal_requested != 0U) || (g_flags.b_is_cal_running != 0U))
    {
        g_flags.b_is_cal_requested =
            0U; /* Cancel the request before it starts. */
        g_flags.b_should_abort_cal = (g_flags.b_is_cal_running != 0U) ? 1U : 0U;
    }

    motion_unlock();

    if (gh_motion_task > 0)
    {
        (void)tk_wup_tsk(
            gh_motion_task); /* The motion task brakes immediately. */
    }
}

/** @brief Ask the controller to cancel the specified command ID. Wake the
 * motion task only if cancellation succeeds and a task is attached.
 */
motion_result_t motion_cancel (uint8_t command_id)
{

    motion_lock();
    const motion_result_t res = ctrl_motion_cancel(command_id);
    motion_unlock();

    if (((unsigned int)MOTION_OK == res) && (gh_motion_task > 0))
    {
        (void)tk_wup_tsk(
            gh_motion_task); /* The motion task brakes immediately. */
    }

    return res;
}

/** @brief Ask the controller to reset under the motion mutex. Return its
 * success flag.
 */
bool motion_reset (void)
{

    motion_lock();
    const bool b_is_idle = ctrl_motion_reset();
    motion_unlock();

    return b_is_idle;
}

/** @brief Decode a 32-bit command and dispatch to its motion operation.
 * Malformed words return MOTION_ERR_BAD_ARG. A rejected reset returns
 * MOTION_ERR_FAULT.
 */
motion_result_t motion_request_packed (uint32_t command)
{
    motion_result_t res;

    motion_pack_cmd_t cmd;
    if (!motion_pack_decode(command, &cmd))
    {
        return MOTION_ERR_BAD_ARG;
    }

    switch (cmd.type)
    {
    case MOTION_CMD_VELOCITY:
        res = motion_request_velocity(cmd.left_mps, cmd.right_mps);
    break;
    case MOTION_CMD_DRIVE:
        res = motion_request_drive(cmd.meters, cmd.speed_mps);
    break;
    case MOTION_CMD_TURN:
        res = motion_request_turn(cmd.degrees, cmd.speed_mps);
    break;
    case MOTION_CMD_MANOEUVRE:
        res = motion_request_manoeuvre(cmd.manoeuvre);
    break;
    case MOTION_CMD_STOP:
        stop();
        res = MOTION_OK;
    break;
    case MOTION_CMD_CANCEL:
        res = motion_cancel(cmd.command_id);
    break;
    default: /* MOTION_CMD_RESET */
        res = motion_reset() ? MOTION_OK : MOTION_ERR_FAULT;
    break;
    }

    return res;
}

/* ---- Status -------------------------------------------------------------- */

/** @brief Poll the controller under the mutex and return its reported motion
 * state.
 */
motion_state_t motion_poll (void)
{

    motion_lock();
    const motion_state_t state = ctrl_motion_poll();
    motion_unlock();

    return state;
}

/** @brief Read the current controller command ID under the motion mutex.
 */
uint8_t motion_command_id (void)
{

    motion_lock();
    const uint8_t value_id = ctrl_motion_command_id();
    motion_unlock();

    return value_id;
}

/** @brief Poll the report queue. Write one decoded report on success and
 * acknowledge completion through the normal receive path.
 */
bool motion_get_report (motion_report_t * p_out)
{
    return report_receive(p_out, TMO_POL);
}

/** @brief Wait up to timeout_ms for a report. A negative timeout waits
 * indefinitely. On success write the report and acknowledge completion
 * where applicable.
 */
bool motion_wait_report (motion_report_t * p_out, int32_t timeout_ms)
{
    return report_receive(p_out, (timeout_ms < 0) ? TMO_FEVR : (TMO)timeout_ms);
}

/** @brief Read the highest recorded report queue occupancy under the motion
 * mutex, in messages.
 */
uint32_t motion_report_peak (void)
{

    motion_lock();
    const uint32_t peak = g_flags.report_peak;
    motion_unlock();

    return peak;
}

/** @brief Read the controller's stopped predicate under the mutex. It accounts
 * for pending braking, maintenance and the stillness interval.
 */
bool motion_is_stopped (void)
{

    motion_lock();
    const bool b_is_stopped = ctrl_motion_is_stopped();
    motion_unlock();

    return b_is_stopped;
}

/** @brief Copy controller state, fault, wheel speeds in m/s, distance in
 * metres and heading in degrees into the caller's status under one
 * lock. A null output returns false.
 */
bool motion_get_status (motion_status_t * p_out)
{
    if (NULL == p_out)
    {
        return false;
    }

    motion_lock();
    p_out->state             = ctrl_motion_state();
    p_out->fault             = ctrl_motion_fault();
    p_out->command_id        = ctrl_motion_command_id();
    p_out->speed_left_mps    = motion_estimation_wheel_speed(WHEEL_LEFT);
    p_out->speed_right_mps   = motion_estimation_wheel_speed(WHEEL_RIGHT);
    p_out->distance_m        = motion_estimation_robot_dist();
    p_out->heading_deg       = motion_estimation_heading_deg();
    p_out->b_is_stopped      = ctrl_motion_is_stopped();
    p_out->b_is_busy         = ctrl_motion_is_busy();
    p_out->b_is_status_valid = wheel_encoder_is_running();
    p_out->fault_bits        = ctrl_motion_fault_bits();
    motion_unlock();

    return true;
}

/** @brief Take a coherent motion status snapshot and return its 64-bit
 * protocol encoding.
 */
uint64_t motion_get_status_packed (void)
{
    motion_status_t status;

    (void)motion_get_status(&status);

    return motion_pack_status(&status);
}

/** @brief Poll the motion mutex for wheel setpoints, speeds, duties, counts
 * and controller status. Return false without writing the destination
 * if the pointer is null or the mutex is unavailable or busy.
 */
bool motion_try_debug_snapshot (motion_debug_snapshot_t * p_snapshot)
{
    if (((NULL == p_snapshot) || (gh_motion_mutex <= 0)) ||
        (tk_loc_mtx(gh_motion_mutex, TMO_POL) != E_OK))
    {
        return false;
    }

    /* clang-format off */
    p_snapshot->timestamp_us     = g_feedback.timestamp_us;
    p_snapshot->state            = ctrl_motion_state();
    p_snapshot->fault            = ctrl_motion_fault();
    p_snapshot->b_is_debug_busy  = ctrl_motion_is_busy();
    p_snapshot->b_is_debug_valid =
        ((g_flags.b_is_feedback_ready) && (g_flags.b_is_feedback_valid)) &&
        (!p_snapshot->b_is_debug_busy);
    p_snapshot->b_is_braked      = ctrl_motion_is_stopped();
    /* clang-format on */

    for (unsigned int wheel_index = 0U; wheel_index < (unsigned int)WHEEL_COUNT;
         ++wheel_index)
    {
        /* clang-format off */
        p_snapshot->setpoint_mps[wheel_index] =
            ctrl_motion_setpoint(wheel_index);
        p_snapshot->measured_mps[wheel_index] =
            motion_estimation_wheel_speed((wheel_encoder_id_t)wheel_index);
        p_snapshot->duty[wheel_index]         =
            motor_control_duty((wheel_encoder_id_t)wheel_index);
        p_snapshot->encoder[wheel_index]      =
            wheel_encoder_count((wheel_encoder_id_t)wheel_index);
        /* clang-format on */
    }

    (void)tk_unl_mtx(gh_motion_mutex);
    return true;
}

/** @brief Read the current primary controller fault under the motion mutex.
 */
motion_fault_t motion_get_fault (void)
{

    motion_lock();
    const motion_fault_t fault = ctrl_motion_fault();
    motion_unlock();

    return fault;
}

/** @brief Read the controller's fault bit mask under the motion mutex.
 */
uint8_t motion_get_fault_bits (void)
{

    motion_lock();
    const uint8_t bits = ctrl_motion_fault_bits();
    motion_unlock();

    return bits;
}

/** @brief Return the static display string for the supplied motion state, or a
 * question mark for an unknown value.
 */
const char * motion_state_name (motion_state_t state)
{
    const char * p_name = NULL;

    switch (state)
    {
    case MOTION_IDLE:
        p_name = "IDLE";
    break;
    case MOTION_VELOCITY:
        p_name = "VELOCITY";
    break;
    case MOTION_DRIVE:
        p_name = "DRIVE";
    break;
    case MOTION_TURN:
        p_name = "TURN";
    break;
    case MOTION_COMPLETE:
        p_name = "COMPLETE";
    break;
    case MOTION_FAULT:
        p_name = "FAULT";
    break;
    default:
        p_name = "?";
    break;
    }

    return p_name;
}

/** @brief Return the static diagnostic text for the primary motion fault, or a
 * question mark for an unknown value.
 */
const char * motion_fault_name (motion_fault_t fault)
{
    const char * p_name = NULL;

    switch (fault)
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

/** @brief Return the static display name for a manoeuvre, or a question mark
 * if the value has no mapping.
 */
const char * motion_manoeuvre_name (motion_manoeuvre_t manoeuvre)
{
    const char * p_name = NULL;

    switch (manoeuvre)
    {
    case MANOEUVRE_LEFT:
        p_name = "Left";
    break;
    case MANOEUVRE_RIGHT:
        p_name = "Right";
    break;
    case MANOEUVRE_STRAIGHT:
        p_name = "Straight";
    break;
    case MANOEUVRE_U_TURN:
        p_name = "U-turn";
    break;
    default:
        p_name = "?";
    break;
    }

    return p_name;
}

/* ---- Estimates -------------------------------------------------------- */

/** @brief Return the left wheel encoder's cached signed count.
 */
int32_t enc_l_count (void)
{
    return wheel_encoder_count(WHEEL_LEFT);
}

/** @brief Return the right wheel encoder's cached signed count.
 */
int32_t enc_r_count (void)
{
    return wheel_encoder_count(WHEEL_RIGHT);
}

/** @brief Read the estimator's left wheel distance in metres.
 */
float32_t motion_wheel_left_distance_m (void)
{
    return motion_estimation_wheel_dist(WHEEL_LEFT);
}

/** @brief Read the estimator's right wheel distance in metres.
 */
float32_t motion_wheel_right_distance_m (void)
{
    return motion_estimation_wheel_dist(WHEEL_RIGHT);
}

/** @brief Read the estimator's left wheel speed in metres per second.
 */
float32_t motion_wheel_left_speed_mps (void)
{
    return motion_estimation_wheel_speed(WHEEL_LEFT);
}

/** @brief Read the estimator's right wheel speed in metres per second.
 */
float32_t motion_wheel_right_speed_mps (void)
{
    return motion_estimation_wheel_speed(WHEEL_RIGHT);
}

/** @brief Return the estimator's centre distance in metres.
 */
float32_t robot_distance_m (void)
{
    return motion_estimation_robot_dist();
}

/** @brief Return the estimator's centre speed in metres per second.
 */
float32_t motion_robot_speed_mps (void)
{
    return motion_estimation_robot_speed();
}

/** @brief Return the estimator's heading in degrees.
 */
float32_t robot_heading_deg (void)
{
    return motion_estimation_heading_deg();
}

/* ========================================================================== */
/* Run calibration outside the motion statechart. */
/* ========================================================================== */

/** @brief Copy epoch-relative centre distance, timestamp, command and brake
 * status under one lock. A failed lock or unavailable destination
 * leaves the output untouched. True means a snapshot was copied. Its
 * valid field reports whether navigation can use it.
 */
bool motion_get_feedback (motion_feedback_t * p_feedback)
{
    if ((0 == p_feedback) || (gh_motion_mutex <= 0))
    {
        return false;
    }

    if (tk_loc_mtx(gh_motion_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }

    p_feedback->centre_m            = g_feedback.centre_m;
    p_feedback->timestamp_us        = g_feedback.timestamp_us;
    p_feedback->epoch               = g_feedback.epoch;
    p_feedback->b_is_feedback_valid = g_flags.b_is_feedback_valid;
    p_feedback->state               = ctrl_motion_state();
    p_feedback->fault               = ctrl_motion_fault();
    p_feedback->command_id          = ctrl_motion_command_id();
    p_feedback->report_loss_count   = g_report_loss_count;
    p_feedback->b_is_braked         = ctrl_motion_is_stopped();
    p_feedback->b_is_feedback_valid =
        ((((p_feedback->b_is_feedback_valid) && (wheel_encoder_is_running())) &&
          (!g_flags.b_is_cal_requested)) &&
         (!g_flags.b_is_cal_running)) &&
        (!ctrl_motion_is_busy());
    motion_unlock();
    return true;
}

/** @brief Arm or disarm the navigation lease under the mutex. Enabling the
 * lease also records the current microsecond time as its latest
 * refresh.
 */
void motion_navigation_heartbeat (bool b_is_enabled)
{
    motion_lock();
    g_flags.b_has_navigation_lease = b_is_enabled;

    if (b_is_enabled)
    {
        g_navigation_refresh_us = time_now_us();
    }

    motion_unlock();
}

/** @brief Return whether the configured encoder counts per revolution is
 * positive. This predicate reads the build calibration setting.
 */
bool motion_is_calibrated (void)
{
    return (ENCODER_COUNTS_PER_REV > 0.0f);
}

/** @brief Queue maintenance calibration when the controller is idle and
 * available. An active navigation lease takes rejection priority over
 * the controller fault check. An accepted request clears the previous
 * completion flag.
 */
motion_result_t motion_request_calibration (void)
{
    motion_result_t res = MOTION_OK;

    motion_lock();

    if (g_flags.b_has_navigation_lease)
    {
        res = MOTION_ERR_BUSY;
    }
    else if (ctrl_motion_state() == (unsigned int)MOTION_FAULT)
    {
        res = MOTION_ERR_FAULT;
    }
    else if (((ctrl_motion_state() != (unsigned int)MOTION_IDLE) ||
              (g_flags.b_is_cal_requested != 0U)) ||
             (ctrl_motion_is_busy()))
    {
        res = MOTION_ERR_BUSY;
    }
    else
    {
        g_flags.b_is_cal_done      = 0U;
        g_flags.b_is_cal_requested = 1U;
    }

    motion_unlock();

    return res;
}

/** @brief Read the pending and running calibration flags together. Return true
 * when either is set.
 */
bool motion_is_calibrating (void)
{

    motion_lock();
    const bool b_is_calibrating =
        (g_flags.b_is_cal_requested != 0U) || (g_flags.b_is_cal_running != 0U);
    motion_unlock();

    return b_is_calibrating;
}

/** @brief Copy the completed calibration under the mutex and return true, even
 * when the recorded calibration failed. Supply writable output for a
 * completed result. Before completion the destination is untouched.
 */
bool motion_get_calibration (motion_calibration_t * p_out)
{

    motion_lock();
    const bool b_is_calibration_done = (g_flags.b_is_cal_done != 0U);

    if (b_is_calibration_done)
    {
        motion_calibration_expand(&g_cal_result, p_out);
    }

    motion_unlock();

    return b_is_calibration_done;
}

#ifdef MOTION_STORAGE_PROBE
/** @brief Replace the mutex ID on the single-threaded fixture and return it.
 */
ID motion_storage_test_exchange_mutex (ID h_mutex)
{
#ifdef MOTION_LEGACY_MUTEX_NAME
    ID * const p_mutex_id = &g_mutex_id;
#else
    ID * const p_mutex_id = &gh_motion_mutex;
#endif
    const ID previous = *p_mutex_id;
    *p_mutex_id       = h_mutex;
    return previous;
}

/** @brief Seed the calibration and completion flag under the motion lock.
 * @param[in] p_value Stable fixture sample with exact floating-point bits.
 * @pre The fake mutex can be acquired. The sample is copied before return.
 */
void motion_storage_test_seed_calibration (const motion_calibration_t * p_value)
{
    motion_lock();
#ifdef MOTION_COMPACT_RETAINED
    motion_calibration_store(&g_cal_result, p_value);
#else
    g_cal_result = *p_value;
#endif
    g_flags.b_is_cal_done = 1U;
    motion_unlock();
}

/** @brief Make calibration unavailable under the motion lock. */
void motion_storage_test_clear_calibration (void)
{
    motion_lock();
    g_flags.b_is_cal_done = 0U;
    motion_unlock();
}

/** @brief Seed the selected feedback layout under the motion lock.
 * @param stamp Wrapping simulated microseconds.
 * @param epoch Full 32-bit feedback epoch, including its terminal value.
 * @param centre Exact binary64 centre distance bits, including NaN payloads.
 * @param losses Full 32-bit count of report losses.
 * @param flags Low three bits select feedback validity, availability and lease.
 */
void motion_storage_test_seed_feedback (uint32_t stamp, uint32_t epoch,
                                        uint64_t centre, uint32_t losses,
                                        unsigned flags)
{
    motion_lock();
    g_flags = (motion_flags_t){0};
    memcpy(&g_feedback.centre_m, &centre, sizeof(centre));
    g_feedback.timestamp_us = stamp;
    g_feedback.epoch        = epoch;
    g_report_loss_count     = losses;
#ifdef MOTION_COMPACT_RETAINED
    g_flags.b_is_feedback_valid    = (flags & 1U) != 0U;
    g_flags.b_is_feedback_ready    = (flags & 2U) != 0U;
    g_flags.b_has_navigation_lease = (flags & 4U) != 0U;
#else
    g_feedback.valid   = (flags & 1U) != 0U;
    g_feedback_ready   = (flags & 2U) != 0U;
    g_navigation_lease = (flags & 4U) != 0U;
#endif
    motion_unlock();
}

/** @brief Fill the fake report mailbox and post the supplied report while
 * holding the motion lock. The callback does not acquire the mutex.
 */
void motion_storage_test_post_to_full (void ( * p_fill)(void),
                                       const motion_report_t * p_report)
{
    motion_lock();
    p_fill();
    report_post(p_report);
    motion_unlock();
}

/** @brief Report the three private allocations in bytes. */
void motion_storage_test_sizes (size_t sizes[3])
{
    sizes[0] = sizeof(g_cal_result);
    sizes[1] = sizeof(g_feedback);
    sizes[2] = sizeof(g_flags);
}
#endif /* MOTION_STORAGE_PROBE */

/* ---- Helpers ------------------------------------------------------------ */

/* Both converters run with the motion mutex held. They only copy values;
 * floating-point precision and the wide caller-visible fields are unchanged. */
/** @brief Copy the public calibration into private storage. Keep all six
 * floating-point values and encode the nine result flags under the
 * motion mutex.
 */
static void motion_calibration_store (motion_stored_calibration_t * p_stored,
                                      const motion_calibration_t *  p_value)
{
    p_stored->counts_per_rev            = p_value->counts_per_rev;
    p_stored->max_speed_mps             = p_value->max_speed_mps;
    p_stored->counts_per_rev_l          = p_value->counts_per_rev_l;
    p_stored->counts_per_rev_r          = p_value->counts_per_rev_r;
    p_stored->top_speed_l_mps           = p_value->top_speed_l_mps;
    p_stored->top_speed_r_mps           = p_value->top_speed_r_mps;
    p_stored->b_is_valid                = p_value->b_is_calibration_valid;
    p_stored->b_is_left_motor_inverted  = p_value->b_is_left_motor_inverted;
    p_stored->b_is_right_motor_inverted = p_value->b_is_right_motor_inverted;
    p_stored->b_is_left_enc_inverted    = p_value->b_is_left_enc_inverted;
    p_stored->b_is_right_enc_inverted   = p_value->b_is_right_enc_inverted;
    p_stored->b_is_left_enc_ok          = p_value->b_is_left_enc_ok;
    p_stored->b_is_right_enc_ok         = p_value->b_is_right_enc_ok;
    p_stored->b_are_encoders_swapped    = p_value->b_are_encoders_swapped;
    p_stored->b_was_aborted             = p_value->b_was_aborted;
}

/** @brief Expand the retained calibration into the caller's writable public
 * result while holding the motion mutex.
 */
static void
motion_calibration_expand (const motion_stored_calibration_t * p_stored,
                           motion_calibration_t *              p_value)
{
    p_value->b_is_calibration_valid    = p_stored->b_is_valid;
    p_value->counts_per_rev            = p_stored->counts_per_rev;
    p_value->max_speed_mps             = p_stored->max_speed_mps;
    p_value->b_is_left_motor_inverted  = p_stored->b_is_left_motor_inverted;
    p_value->b_is_right_motor_inverted = p_stored->b_is_right_motor_inverted;
    p_value->b_is_left_enc_inverted    = p_stored->b_is_left_enc_inverted;
    p_value->b_is_right_enc_inverted   = p_stored->b_is_right_enc_inverted;
    p_value->counts_per_rev_l          = p_stored->counts_per_rev_l;
    p_value->counts_per_rev_r          = p_stored->counts_per_rev_r;
    p_value->top_speed_l_mps           = p_stored->top_speed_l_mps;
    p_value->top_speed_r_mps           = p_stored->top_speed_r_mps;
    p_value->b_is_left_enc_ok          = p_stored->b_is_left_enc_ok;
    p_value->b_is_right_enc_ok         = p_stored->b_is_right_enc_ok;
    p_value->b_are_encoders_swapped    = p_stored->b_are_encoders_swapped;
    p_value->b_was_aborted             = p_stored->b_was_aborted;
}

/** @brief Read the low 32 bits of the RP2040 timer in microseconds.
 */
static uint32_t time_now_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

/** @brief Request the motion mutex with an unlimited wait. Callers rely on
 * successful motion initialization and task context.
 */
static void motion_lock (void)
{
    (void)tk_loc_mtx(gh_motion_mutex, TMO_FEVR);
}

/** @brief Release the motion mutex after a task has finished accessing shared
 * controller or estimator state.
 */
static void motion_unlock (void)
{
    (void)tk_unl_mtx(gh_motion_mutex);
}

/** @brief Return the magnitude of a finite argument. Zero follows the
 * nonnegative branch and retains its floating representation.
 */
static float32_t abs_f (float32_t operand)
{
    return (operand < 0.0f) ? -operand : operand;
}

/** @brief Accept finite inputs using the controller's existing equality and
 * subtraction checks. Unordered input and infinity fail.
 */
static bool is_number (float32_t operand)
{
    return (operand == operand) &&
           ((operand - operand) == 0.0f); /* Reject NaN and infinity. */
}

/** @brief Convert positive counts per wheel revolution to metres per encoder
 * count using the configured wheel diameter. Return zero for a
 * nonpositive or unordered input.
 */
static float32_t m_per_count (float32_t counts_per_rev)
{
    return (counts_per_rev > 0.0f) ? (PI_F * WHEEL_DIAMETER_M / counts_per_rev)
                                   : 0.0f;
}

/* ---- Lifecycle ----------------------------------------------------------- */

/* Report sink for the controller (mutex held). Never waits: if the queue is
 * full the report is dropped and the next one is marked b_lost. */
/** @brief With the motion mutex held, try to enqueue one packed report. Record
 * a failed send so the next successful report can signal loss.
 */
static void report_post (const motion_report_t * p_report)
{
    motion_report_t report = *p_report;

    report.b_was_lost = (g_flags.b_is_report_lost != 0U);
    uint16_t word     = motion_pack_report(&report);

    ER value_er = tk_snd_mbf(gh_motion_reports, &word, REPORT_MSG_SZ, TMO_POL);

    if (E_OK == value_er)
    {
        g_flags.b_is_report_lost = 0U;

        if (g_flags.report_queued < MOTION_REPORT_QUEUE_LEN)
        {
            g_flags.report_queued++;
        }

        if (g_flags.report_queued > g_flags.report_peak)
        {
            g_flags.report_peak = g_flags.report_queued;
        }
    }
    else
    {
        g_flags.b_is_report_lost = 1U;
        g_report_loss_count++;
    }
}

/** @brief Receive one complete report using the supplied RTOS timeout. A
 * missing destination, unavailable buffer or incorrect byte count
 * returns false. Successful delivery updates queue accounting and
 * acknowledges a completed command.
 */
static bool report_receive (motion_report_t * p_out, TMO tmout)
{

    if ((NULL == p_out) || (gh_motion_reports <= 0))
    {
        return false;
    }

    uint16_t word;
    INT      size = tk_rcv_mbf(gh_motion_reports, &word, tmout);

    if (size != REPORT_MSG_SZ)
    {
        return false; /* No complete report was received. */
    }

    motion_pack_read_report(word, p_out);

    motion_lock();

    if (g_flags.report_queued > 0U)
    {
        g_flags.report_queued--;
    }

    if ((unsigned int)MOTION_OUTCOME_DONE == p_out->outcome)
    {
        ctrl_motion_ack_complete(p_out->command_id);
    }

    motion_unlock();

    return true;
}

/** @brief Submit a timed drive or turn under the mutex with the supplied wheel
 * signs, travel in metres and speed in m/s. Return the controller's
 * acceptance or rejection.
 */
static motion_result_t request_move (motion_state_t kind, int32_t sign_left,
                                     int32_t sign_right, float32_t dist_m,
                                     float32_t speed_mps)
{

    motion_lock();
    const motion_result_t res = ctrl_motion_request_move(
        kind, sign_left, sign_right, dist_m, speed_mps, time_now_us());
    motion_unlock();

    return res;
}

/* Calibration wait: CAL_POLL_MS at a time, sampling the encoders. Returns
 * false at once if stop() aborted calibration; the caller then brakes. */
/** @brief Delay in calibration poll intervals, sampling encoders and checking
 * for an abort after each delay. Return false when STOP has requested
 * an abort.
 */
static bool cal_wait (int32_t value_ms)
{
    bool b_should_abort;

    do
    {
        (void)tk_dly_tsk(CAL_POLL_MS);
        value_ms -= CAL_POLL_MS;
        wheel_encoder_sample();
        motion_lock();
        b_should_abort = (g_flags.b_should_abort_cal != 0U);
        motion_unlock();
    } while ((!b_should_abort) && (value_ms > 0));

    return !b_should_abort;
}

/* Calibration step 2: run one motor alone at full power (wheels off the
 * ground) and measure how far each encoder moved. False if aborted. */
/** @brief Run the selected wheel for a calibration measurement and report both
 * signed encoder deltas and elapsed seconds. Brake both motors before
 * returning. An abort can still produce partial measurement outputs.
 */
static bool cal_spin (wheel_encoder_id_t motor, int32_t * p_left_delta,
                      int32_t * p_right_delta, float32_t * p_secs)
{

    motor_control_set(motor, 1.0f);
    bool           b_did_spin           = cal_wait(CAL_SPINUP_MS);
    const uint32_t left_start_count     = wheel_encoder_bits(WHEEL_LEFT);
    const uint32_t right_start_count    = wheel_encoder_bits(WHEEL_RIGHT);
    const uint32_t measurement_start_us = time_now_us();
    /* Measure through rollover in unsigned microseconds, then accept
     * float32_t rounding when converting the interval to seconds. */
    /* clang-format off */
    b_did_spin     = b_did_spin && (cal_wait(CAL_MEASURE_MS));
    *p_left_delta  =
        wheel_encoder_delta(wheel_encoder_bits(WHEEL_LEFT), left_start_count);
    *p_right_delta =
        wheel_encoder_delta(wheel_encoder_bits(WHEEL_RIGHT), right_start_count);
    *p_secs        =
        (float32_t)(time_now_us() - measurement_start_us) / US_PER_S;
    /* clang-format on */
    motor_control_brake_all();

    return b_did_spin && (cal_wait(CAL_SETTLE_MS));
}

/** @brief Return the magnitude of an encoder delta. The caller supplies a
 * value whose magnitude fits in int32_t.
 */
static int32_t abs_i (int32_t operand)
{
    return (operand < 0) ? -operand : operand;
}

/* Runs in the motion task (maintenance mode: requests rejected, never
 * "stopped"). stop() aborts it within CAL_POLL_MS with the motors braked. */
/** @brief Measure counts per wheel revolution from hand turns, then test each
 * lifted wheel separately for polarity, mapping and speed. Fill the
 * caller's result, including partial diagnostics on early exit. The
 * motion task owns the hardware throughout.
 */
static void run_calibration (motion_calibration_t * p_out)
{
    const uint32_t start     = time_now_us();
    uint32_t       last_move = start;
    int32_t        left_motor_left_delta;
    int32_t        left_motor_right_delta;
    int32_t        right_motor_left_delta;
    int32_t        right_motor_right_delta;
    float32_t      left_spin_seconds;
    float32_t      right_spin_seconds;

    *p_out = (motion_calibration_t){false};
    motor_control_brake_all();
    wheel_encoder_sample();
    const uint32_t left_start_count  = wheel_encoder_bits(WHEEL_LEFT);
    const uint32_t right_start_count = wheel_encoder_bits(WHEEL_RIGHT);
    uint32_t       ref_l             = left_start_count;
    uint32_t       ref_r             = right_start_count;

    /* Measure counts per turn and direction while the user turns the wheels.
     * Wait for each encoder to move beyond the minimum count. One broken
     * quadrature channel can make the reading flicker by just one count. */
    for (;;)
    {
        const uint32_t left_count  = wheel_encoder_bits(WHEEL_LEFT);
        const uint32_t right_count = wheel_encoder_bits(WHEEL_RIGHT);
        const uint32_t now         = time_now_us();
        const bool     b_is_left_encoder_valid =
            (abs_i(wheel_encoder_delta(left_count, left_start_count)) >
             CAL_MIN_COUNTS);
        const bool b_is_right_encoder_valid =
            (abs_i(wheel_encoder_delta(right_count, right_start_count)) >
             CAL_MIN_COUNTS);

        if ((abs_i(wheel_encoder_delta(left_count, ref_l)) > CAL_MOVE_COUNTS) ||
            (abs_i(wheel_encoder_delta(right_count, ref_r)) > CAL_MOVE_COUNTS))
        {
            ref_l     = left_count;
            ref_r     = right_count;
            last_move = now;
        }

        if ((b_is_left_encoder_valid && b_is_right_encoder_valid) &&
            ((now - last_move) > CAL_STILL_US))
        {
            break;
        }

        if ((now - start) > CAL_STEP1_TIMEOUT_US)
        {
            p_out->b_is_left_enc_ok  = b_is_left_encoder_valid;
            p_out->b_is_right_enc_ok = b_is_right_encoder_valid;

            return;
        }

        if (!cal_wait(CAL_POLL_MS))
        {
            p_out->b_was_aborted = true;

            return;
        }
    }

    p_out->b_is_left_enc_ok  = true;
    p_out->b_is_right_enc_ok = true;

    const int32_t value_tl =
        wheel_encoder_delta(wheel_encoder_bits(WHEEL_LEFT), left_start_count);
    const int32_t value_tr =
        wheel_encoder_delta(wheel_encoder_bits(WHEEL_RIGHT), right_start_count);

    const bool b_should_flip_left_encoder  = (value_tl < 0);
    const bool b_should_flip_right_encoder = (value_tr < 0);
    /* Each modular 20-bit delta has magnitude at most 524288. That
     * magnitude and the fixed turn count convert exactly before division. */
    const float32_t cpr_l = (float32_t)abs_i(value_tl) / (float32_t)CAL_TURNS;
    const float32_t cpr_r = (float32_t)abs_i(value_tr) / (float32_t)CAL_TURNS;
    const float32_t cpr   = 0.5f * (cpr_l + cpr_r);
    const float32_t scale = m_per_count(cpr);

    /* Use the measured directions and count scale for the second step. */
    wheel_encoder_set_inverted(WHEEL_LEFT,
                               ENC_L_INVERTED != b_should_flip_left_encoder);
    wheel_encoder_set_inverted(WHEEL_RIGHT,
                               ENC_R_INVERTED != b_should_flip_right_encoder);

    /* In step 2, measure top speed, motor direction and encoder ports one
     * motor at a time with the wheels off the ground.
     */
    if (((!cal_wait(CAL_LIFT_MS)) ||
         (!cal_spin(WHEEL_LEFT, &left_motor_left_delta, &left_motor_right_delta,
                    &left_spin_seconds))) ||
        (!cal_spin(WHEEL_RIGHT, &right_motor_left_delta,
                   &right_motor_right_delta, &right_spin_seconds)))
    {
        wheel_encoder_set_inverted(WHEEL_LEFT, ENC_L_INVERTED);
        wheel_encoder_set_inverted(WHEEL_RIGHT, ENC_R_INVERTED);
        motor_control_brake_all();
        p_out->b_was_aborted = true;

        return;
    }

    /* Each motor should move its own encoder. The encoder cables are swapped
     * if the left motor moves ENC_R and the right motor moves ENC_L.
     */
    p_out->b_are_encoders_swapped =
        (abs_i(left_motor_right_delta) > abs_i(left_motor_left_delta)) &&
        (abs_i(right_motor_left_delta) > abs_i(right_motor_right_delta));

    /* Signed 20-bit deltas convert exactly before scaling to m/s. */
    const float32_t value_vl =
        (float32_t)left_motor_left_delta * scale / left_spin_seconds;
    const float32_t value_vr =
        (float32_t)right_motor_right_delta * scale / right_spin_seconds;

    /* clang-format off */
    p_out->counts_per_rev_l          = cpr_l;
    p_out->counts_per_rev_r          = cpr_r;
    p_out->counts_per_rev            = cpr;
    p_out->top_speed_l_mps           = abs_f(value_vl);
    p_out->top_speed_r_mps           = abs_f(value_vr);
    p_out->max_speed_mps             =
        (abs_f(value_vl) < abs_f(value_vr)) ? abs_f(value_vl) : abs_f(value_vr);
    p_out->b_is_left_motor_inverted  = (M1_INVERTED != (value_vl < 0.0f));
    p_out->b_is_right_motor_inverted = (M2_INVERTED != (value_vr < 0.0f));
    p_out->b_is_left_enc_inverted    =
        (ENC_L_INVERTED != b_should_flip_left_encoder);
    p_out->b_is_right_enc_inverted   =
        (ENC_R_INVERTED != b_should_flip_right_encoder);
    /* clang-format on */

    /* Restore the built-in encoder directions after the measurement. */
    wheel_encoder_set_inverted(WHEEL_LEFT, ENC_L_INVERTED);
    wheel_encoder_set_inverted(WHEEL_RIGHT, ENC_R_INVERTED);
    motor_control_brake_all();

    p_out->b_is_calibration_valid =
        (((!p_out->b_are_encoders_swapped) &&
          (abs_f(cpr_l - cpr_r) <= (CAL_CPR_TOLERANCE * cpr))) &&
         (p_out->top_speed_l_mps >= CAL_MIN_SPEED_MPS)) &&
        (p_out->top_speed_r_mps >= CAL_MIN_SPEED_MPS);
}

/* End of file. */

