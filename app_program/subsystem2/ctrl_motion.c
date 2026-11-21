/** @file ctrl_motion.c
 * @brief Motion Control statechart, straight-line correction and encoder-based
 * turning.
 *
 * @par Ownership
 * Buddy 2 - Motion Control
 *
 * Minimum-bit representation (DATA_FORMAT.md, internal fields): the state,
 * latched fault bits, command ID and flags share one word of bit-fields, and
 * the timers, counters and stored encoder counts use only the bits their
 * range needs. Every function runs with the Buddy 2 mutex held
 * (ctrl_motion.h), so no read-modify-write of a packed word can race.
 * Speeds, distances and PID terms stay float (control precision).
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 */

#include <stddef.h>

#include "ctrl_motion.h"
#include "../app_types.h"
#include "motion_estimation.h"
#include "motion_pack.h"
#include "motor_control.h"
#include "speed_pid.h"
#include "wheel_encoder.h"

/* ---- Tuning (the defaults should work; change only if needed) ----------- */
#define SQRT_NEWTON_ITERATIONS (12U)
#define SPEED_KP_N (3.2f)           /* Normalize P to top speed. */
#define SPEED_KI_N (5.0f)           /* Apply normalized I per second. */
#define SPEED_KD_N (0.0f)           /* Zero D selects PI control. */
#define SYNC_GAIN (4.0f)            /* Correct speed per metre of error. */
#define ACCEL_MPS2 (1.5f)           /* Limit acceleration to reduce slip. */
#define DECEL_MPS2 (0.6f)           /* Decelerate before a target. */
#define MIN_SPEED_MPS (0.04f)       /* Crawl at the approach speed. */
#define CRAWL_DIST_M (0.008f)       /* Crawl before applying the brake. */
#define SPEED_LIMIT_FRACTION (0.9f) /* Use at most 90% of top speed. */

/* ---- Fault detection --------------------------------------------------- */
#define VELOCITY_TIMEOUT_US (250000U) /* Expire stale speed commands. */
#define STALL_TIMEOUT_MS (600U)       /* Fault after no encoder progress. */
#define STALL_MIN_PROGRESS (2)        /* Require two counts for progress. */
#define DIRECTION_FAULT_MS (300U)     /* Fault on sustained wrong direction. */
#define ENCODER_GLITCH_LIMIT (25)     /* Limit accumulated invalid edges. */
#define RESET_STILL_MS (200U)         /* Require still wheels to reset. */
#define STOPPED_STILL_MS (100U)       /* Wait for stillness to acknowledge. */
#define CMD_ACTIVE_MPS (0.02f)        /* Ignore stall below this speed. */
#define DIRECTION_MIN_MPS (0.05f)
#define STILL_SPEED_MPS (0.01f)
#define STILL_MS_MAX (60000U)
#define TIMEOUT_FACTOR (3.0f)   /* Triple the expected duration. */
#define TIMEOUT_MARGIN_S (2.0f) /* Add two seconds of margin. */

/* Reject allowances longer than 600 s to stay within half of the
 * 32-bit microsecond counter's range. */
#define MOVE_TIME_MAX_S (600.0f)

/* ---- Field widths (each covers its range; DATA_FORMAT.md) -------------- */
#define STATE_BITS (3U)         /* Cover all six states. */
#define CMD_ID_BITS (8U)        /* Use zero for no command. */
#define STALL_MS_BITS (10U)     /* Saturate at 1023 ms. */
#define WRONG_MS_BITS (9U)      /* Saturate at 511 ms. */
#define GLITCH_SEEN_BITS (8U)   /* Keep the low glitch-count byte. */
#define GLITCH_BUCKET_BITS (6U) /* Cap at twice the glitch limit. */
#define STALL_MS_MAX ((1U << STALL_MS_BITS) - 1U)
#define WRONG_MS_MAX ((1U << WRONG_MS_BITS) - 1U)
#define GLITCH_SEEN_MASK ((1U << GLITCH_SEEN_BITS) - 1U)

typedef struct
{
    speed_pid_t pid;
    uint32_t    stall_ref
        : WHEEL_COUNT_BITS;            /* Store the stall reference count. */
    uint32_t stall_ms : STALL_MS_BITS; /* Time driven without progress. */
    uint32_t wrong_ms : WRONG_MS_BITS; /* Time rotation against the command. */
    uint32_t glitch_seen
        : GLITCH_SEEN_BITS; /* Store the previous counter byte. */
    uint32_t glitch_bucket
        : GLITCH_BUCKET_BITS; /* Accumulate excess glitches. */
} wheel_ctrl_t;

/* Use 21 bits for the state, fault mask, command ID and flags. */
typedef struct
{
    uint32_t state : STATE_BITS;                   /* motion_state_t      */
    uint32_t fault_bits : MOTION_FAULT_BITS_WIDTH; /* motion_pack_fault_bit() */
    uint32_t cmd_id : CMD_ID_BITS;   /* Store the last accepted ID. */
    uint32_t b_is_busy : 1;          /* Calibration is running. */
    uint32_t b_is_brake_pending : 1; /* The brake still needs applying. */
    uint32_t b_is_move_turn : 1;     /* Set for TURN and clear for DRIVE. */
    uint32_t mv_back : WHEEL_COUNT;  /* Store one direction bit per wheel. */
                                     /* Set a bit for reverse. */
} ctrl_t;

static wheel_ctrl_t g_wheel[WHEEL_COUNT];
static ctrl_t       g_ctl = {(uint32_t)MOTION_IDLE, 0U, 0U, 0U, 0U, 0U, 0U};
/* The last bounded step can cross the stillness accumulation limit. */
static uint16_t  g_still_ms      = 0U;
static float32_t g_max_speed_mps = 1.0f;
static float32_t g_lead_drive_m  = 0.0f;
static float32_t g_lead_turn_m   = 0.0f;

/* Send reports to the queue sink provided by motion.c. */
static ctrl_motion_report_sink_t gp_sink = NULL;

/* VELOCITY CONTROL */
static float32_t g_vel_target[WHEEL_COUNT];
static uint32_t  g_vel_stamp_us = 0U;

/* DRIVE / TURN */
static wheel_encoder_count_t g_mv_start[WHEEL_COUNT];
static float32_t             g_mv_dist_m      = 0.0f;
static float32_t             g_mv_speed_mps   = 0.0f;
static uint32_t              g_mv_deadline_us = 0U;

static float32_t abs_f (float32_t operand);
static float32_t min_f (float32_t left_operand, float32_t right_operand);
static float32_t max_f (float32_t left_operand, float32_t right_operand);
static float32_t sqrt_f (float32_t operand);
static int32_t   abs_i (int32_t operand);
static uint32_t  add_ms (uint32_t value_ms, uint32_t dt_ms, uint32_t max_ms);
static bool      time_after (uint32_t now_us, uint32_t deadline_us);
static motion_state_t read_motion_state (void);
static void           set_state (motion_state_t state);
static bool           is_active (motion_state_t state);
static float32_t      mv_sign (wheel_encoder_id_t wheel);
static void           outputs_safe (void);
static void           new_command (void);
static void           post_motion_outcome (motion_outcome_t outcome,
                                           motion_fault_t   fault);
static void           raise_faults (uint32_t fault_bits);
static void           raise_fault (motion_fault_t fault);
static void           complete (void);
static void           clear_checks (void);
static void wheel_drive (wheel_encoder_id_t wheel, float32_t target_mps,
                         float32_t dt_s);
static motion_fault_t wheel_check (wheel_encoder_id_t wheel, uint32_t dt_ms);
static void           track_glitches (void);
static float32_t      move_wheel_progress (wheel_encoder_id_t wheel);
static bool           move_step (float32_t dt_s);

/* ---- Interface ----------------------------------------------------------- */

/** @brief Initialize wheel gains and brake both outputs with the motion mutex
 * held. Positive max_speed_mps sets full-scale speed; otherwise use 1
 * m/s. The drive and turn leads are metres. The optional report sink
 * must consume or copy its argument before returning.
 */
void ctrl_motion_init (float32_t max_speed_mps, float32_t stop_lead_drive_m,
                       float32_t                 stop_lead_turn_m,
                       ctrl_motion_report_sink_t p_sink)
{

    g_max_speed_mps     = (max_speed_mps > 0.0f) ? max_speed_mps : 1.0f;
    g_lead_drive_m      = stop_lead_drive_m;
    g_lead_turn_m       = stop_lead_turn_m;
    gp_sink             = p_sink;
    const float32_t kff = 1.0f / g_max_speed_mps;

    uint32_t wheel_index;
    for (wheel_index = 0U; wheel_index < (uint32_t)WHEEL_COUNT; wheel_index++)
    {
        speed_pid_init(&g_wheel[wheel_index].pid, SPEED_KP_N * kff,
                       SPEED_KI_N * kff, SPEED_KD_N * kff, kff, ACCEL_MPS2);
        /* clang-format off */
        g_wheel[wheel_index].glitch_seen   =
            wheel_encoder_glitches((wheel_encoder_id_t)wheel_index) &
            GLITCH_SEEN_MASK;
        g_wheel[wheel_index].glitch_bucket = 0U;
        g_vel_target[wheel_index]          = 0.0f;
        /* clang-format on */
    }

    clear_checks();
    set_state(MOTION_IDLE);
    g_ctl.fault_bits = 0U;
    outputs_safe();
}

/** @brief Run one motion period with elapsed seconds and a wrapping microsecond
 * clock. Update glitch and stillness tracking even during calibration. For
 * an active command, evaluate both wheels before reporting completion so
 * faults take precedence.
 */
void ctrl_motion_step (float32_t dt_s, uint32_t now_us)
{
    /* The motion owner clamps finite elapsed time to 2..50 ms before
     * calling this function. The conversion rounds to whole ms. */
    const uint32_t dt_ms          = (uint32_t)((dt_s * 1000.0f) + 0.5f);
    bool           b_is_move_done = false;

    track_glitches();

    if ((abs_f(motion_estimation_wheel_speed(WHEEL_LEFT)) < STILL_SPEED_MPS) &&
        (abs_f(motion_estimation_wheel_speed(WHEEL_RIGHT)) < STILL_SPEED_MPS))
    {
        if (g_still_ms < STILL_MS_MAX)
        {
            /* The counter was below 60000 and this step adds at most 50
             * ms, so the possibly overshooting sum still fits uint16_t. */
            g_still_ms = (uint16_t)(g_still_ms + dt_ms);
        }
    }
    else
    {
        g_still_ms = 0U;
    }

    if (g_ctl.b_is_busy != 0U)
    {
        return; /* Calibration owns the motors. */
    }

    switch (read_motion_state())
    {
    case MOTION_VELOCITY:
        if (time_after(now_us, g_vel_stamp_us + VELOCITY_TIMEOUT_US))
        {
            raise_fault(MOTION_FAULT_STALE_COMMAND);

            return;
        }
        wheel_drive(WHEEL_LEFT, g_vel_target[WHEEL_LEFT], dt_s);
        wheel_drive(WHEEL_RIGHT, g_vel_target[WHEEL_RIGHT], dt_s);
    break;

    case MOTION_DRIVE: /* Drives and turns share the deadline and move step. */
    case MOTION_TURN:
        if (time_after(now_us, g_mv_deadline_us))
        {
            raise_fault(MOTION_FAULT_TIMEOUT);

            return;
        }
        b_is_move_done = move_step(dt_s);
    break;

    default: /* IDLE, COMPLETE, FAULT */
        outputs_safe();

        return;
    }

    /* Active-state feedback checks (both wheels, to keep the timers right).
     * They run before completion, so a fault wins over completion. A fault
     * on each wheel latches both bits. */
    const motion_fault_t fault_l = wheel_check(WHEEL_LEFT, dt_ms);
    const motion_fault_t fault_r = wheel_check(WHEEL_RIGHT, dt_ms);

    if ((fault_l != (unsigned int)MOTION_FAULT_NONE) ||
        (fault_r != (unsigned int)MOTION_FAULT_NONE))
    {
        raise_faults((uint32_t)motion_pack_fault_bit(fault_l) |
                     (uint32_t)motion_pack_fault_bit(fault_r));
    }
    else if (b_is_move_done)
    {
        outputs_safe();
        complete();
    }
    else
    {
        /* Keep the command active. */
    }
}

/** @brief Brake and reset both PIDs if neither an active command nor
 * calibration owns the motors. The motion task calls this under its
 * mutex.
 */
void ctrl_motion_make_safe (void)
{
    if ((!is_active(read_motion_state())) && (0U == g_ctl.b_is_busy))
    {
        outputs_safe();
    }
}

/** @brief Accept signed wheel speeds in m/s and refresh their microsecond
 * timestamp. Clamp both targets to the speed cap. The first accepted
 * request gets an ID; refreshes keep it. Return FAULT or BUSY on rejection.
 */
motion_result_t ctrl_motion_set_velocity (float32_t left_mps,
                                          float32_t right_mps, uint32_t now_us)
{
    const float32_t cap = ctrl_motion_speed_cap();

    if (read_motion_state() == (unsigned int)MOTION_FAULT)
    {
        return MOTION_ERR_FAULT;
    }
    /* Reject new motion until requested braking has completed. */
    if (((g_ctl.b_is_busy != 0U) || (g_ctl.b_is_brake_pending != 0U)) ||
        ((read_motion_state() != (unsigned int)MOTION_IDLE) &&
         (read_motion_state() != (unsigned int)MOTION_VELOCITY)))
    {
        return MOTION_ERR_BUSY;
    }

    g_vel_target[WHEEL_LEFT]  = max_f(min_f(left_mps, cap), -cap);
    g_vel_target[WHEEL_RIGHT] = max_f(min_f(right_mps, cap), -cap);
    g_vel_stamp_us            = now_us;

    if (read_motion_state() == (unsigned int)MOTION_IDLE)
    {
        new_command(); /* Later updates keep this ID. */
        clear_checks();
        set_state(MOTION_VELOCITY);
    }

    return MOTION_OK;
}

/** @brief Request DRIVE or TURN with a distance in metres and speed in m/s. A
 * negative wheel sign selects reverse. Only an idle controller with no
 * pending brake or calibration accepts a move. Return a fault, busy or
 * invalid-argument result on rejection. An accepted zero distance reports
 * DONE immediately.
 */
motion_result_t ctrl_motion_request_move (motion_state_t kind,
                                          int32_t sign_left, int32_t sign_right,
                                          float32_t dist_m, float32_t speed_mps,
                                          uint32_t now_us)
{
    const float32_t speed = min_f(speed_mps, ctrl_motion_speed_cap());
    const float32_t allowed_s =
        ((dist_m / speed) * TIMEOUT_FACTOR) + TIMEOUT_MARGIN_S;

    if (read_motion_state() == (unsigned int)MOTION_FAULT)
    {
        return MOTION_ERR_FAULT;
    }
    /* Reject new motion until requested braking has completed. */
    if (((g_ctl.b_is_busy != 0U) || (g_ctl.b_is_brake_pending != 0U)) ||
        (read_motion_state() != (unsigned int)MOTION_IDLE))
    {
        return MOTION_ERR_BUSY;
    }
    /* Reject a duration that cannot be represented by the wrapping deadline. */
    if (((!(dist_m >= 0.0f)) || (!(speed > 0.0f))) ||
        (!(allowed_s <= MOVE_TIME_MAX_S)))
    {
        return MOTION_ERR_BAD_ARG;
    }

    /* clang-format off */
    g_ctl.b_is_move_turn = ((unsigned int)MOTION_TURN == kind) ? 1U : 0U;
    g_ctl.mv_back        =
        ((sign_left < 0) ? (1U << (uint32_t)WHEEL_LEFT) : 0U) |
                    ((sign_right < 0) ? (1U << (uint32_t)WHEEL_RIGHT) : 0U);
    /* clang-format on */

    g_mv_start[WHEEL_LEFT].bits  = wheel_encoder_bits(WHEEL_LEFT);
    g_mv_start[WHEEL_RIGHT].bits = wheel_encoder_bits(WHEEL_RIGHT);
    g_mv_dist_m                  = dist_m;
    g_mv_speed_mps               = speed;
    /* The duration limit keeps the offset at or below 600 million us.
     * Unsigned addition preserves the wrapping deadline clock. */
    g_mv_deadline_us = now_us + (uint32_t)(allowed_s * 1000000.0f);
    clear_checks();
    new_command();
    set_state(kind);

    if (dist_m <= 0.0f)
    {
        complete(); /* Complete a zero-length move immediately. */
    }

    return MOTION_OK;
}

/** @brief Brake both wheels and latch the stale-command fault when the motion
 * task observes that the navigation lease has expired.
 */
void ctrl_motion_nav_timeout (void)
{
    raise_fault(MOTION_FAULT_STALE_COMMAND);
}

/** @brief Cancel an active command once and mark braking pending. A completed
 * command can be acknowledged without a second outcome. Preserve an
 * existing fault; motor braking is serviced by the owner.
 */
void ctrl_motion_stop (void)
{
    if (is_active(read_motion_state()))
    {
        post_motion_outcome(MOTION_OUTCOME_CANCELLED, MOTION_FAULT_NONE);
        g_ctl.b_is_brake_pending = 1U; /* The motors may still be driven. */
    }

    if (read_motion_state() != (unsigned int)MOTION_FAULT)
    {
        set_state(MOTION_IDLE); /* The motion task brakes at once. */
    }
}

/** @brief Cancel only the active command matching the supplied nonzero ID.
 * Return OBSOLETE for a stale or inactive ID, or FAULT while a fault
 * is latched.
 */
motion_result_t ctrl_motion_cancel (uint8_t command_id)
{
    if (read_motion_state() == (unsigned int)MOTION_FAULT)
    {
        return MOTION_ERR_FAULT; /* A cancel leaves any fault latched. */
    }
    /* A cancellation may affect only a running command. */
    if (((0U == command_id) || (command_id != g_ctl.cmd_id)) ||
        (!is_active(read_motion_state())))
    {
        return MOTION_ERR_OBSOLETE;
    }

    ctrl_motion_stop();

    return MOTION_OK;
}

/** @brief Return true outside FAULT. In FAULT, clear the latch and return to
 * IDLE only after the stillness and both glitch-bucket checks pass.
 */
bool ctrl_motion_reset (void)
{
    if (read_motion_state() != (unsigned int)MOTION_FAULT)
    {
        return true;
    }

    if (((g_still_ms >= RESET_STILL_MS) &&
         (g_wheel[WHEEL_LEFT].glitch_bucket <=
          ((uint32_t)ENCODER_GLITCH_LIMIT / 2U))) &&
        (g_wheel[WHEEL_RIGHT].glitch_bucket <=
         ((uint32_t)ENCODER_GLITCH_LIMIT / 2U)))
    {
        g_ctl.fault_bits = 0U;
        set_state(MOTION_IDLE);

        return true;
    }

    return false;
}

/** @brief Return the state observed on entry. Reading COMPLETE through this
 * interface acknowledges it and leaves the controller IDLE.
 */
motion_state_t ctrl_motion_poll (void)
{
    const motion_state_t state = read_motion_state();

    if ((unsigned int)MOTION_COMPLETE == state)
    {
        set_state(MOTION_IDLE); /* Report completion. */
    }

    return state;
}

/** @brief Return the current motion state without acknowledging completion.
 */
motion_state_t ctrl_motion_state (void)
{
    return read_motion_state();
}

/** @brief Return the first fault selected from the retained fault mask.
 */
motion_fault_t ctrl_motion_fault (void)
{
    return motion_pack_fault_first((uint8_t)g_ctl.fault_bits);
}

/** @brief Return all currently latched motion fault bits under the caller-held
 * mutex.
 */
uint8_t ctrl_motion_fault_bits (void)
{
    return (uint8_t)g_ctl.fault_bits;
}

/** @brief Read the current eight-bit command ID. Issued IDs occupy 1 through
 * 255.
 */
uint8_t ctrl_motion_command_id (void)
{
    return (uint8_t)g_ctl.cmd_id;
}

/** @brief Acknowledge COMPLETE only when command_id matches the current
 * command; an obsolete ID has no effect.
 */
void ctrl_motion_ack_complete (uint8_t command_id)
{
    if ((read_motion_state() == (unsigned int)MOTION_COMPLETE) &&
        (command_id == g_ctl.cmd_id))
    {
        set_state(MOTION_IDLE); /* Report completion. */
    }
}

/** @brief Return true when commands, calibration and pending braking are
 * inactive and both wheels have met the stillness interval.
 */
bool ctrl_motion_is_stopped (void)
{
    return (((!is_active(read_motion_state())) && (0U == g_ctl.b_is_busy)) &&
            (0U == g_ctl.b_is_brake_pending)) &&
           (g_still_ms >= STOPPED_STILL_MS);
}

/** @brief Return the allowed magnitude of a velocity request in metres per
 * second, derived from the configured full-scale wheel speed.
 */
float32_t ctrl_motion_speed_cap (void)
{
    return SPEED_LIMIT_FRACTION * g_max_speed_mps;
}

/** @brief Read a wheel PID setpoint in m/s. Left and right are valid indexes;
 * an index outside that range returns zero.
 */
float32_t ctrl_motion_setpoint (unsigned int wheel)
{
    return wheel < (unsigned int)WHEEL_COUNT
               ? speed_pid_setpoint(&g_wheel[wheel].pid)
               : 0.0f;
}

/** @brief Set or clear the exclusive controller reservation used during
 * calibration or when feedback is unavailable.
 */
void ctrl_motion_set_busy (bool b_is_busy)
{
    g_ctl.b_is_busy = b_is_busy ? 1U : 0U;
}

/** @brief Return whether the controller reservation currently prevents new
 * requests.
 */
bool ctrl_motion_is_busy (void)
{
    return (g_ctl.b_is_busy != 0U);
}

/* ---- Helpers ------------------------------------------------------------ */

/** @brief Return the negation of a negative input; otherwise return the
 * original value, including signed zero.
 */
static float32_t abs_f (float32_t operand)
{
    return (operand < 0.0f) ? -operand : operand;
}

/** @brief Select value_a if it is strictly smaller; otherwise return value_b,
 * including on an unordered comparison.
 */
static float32_t min_f (float32_t left_operand, float32_t right_operand)
{
    return (left_operand < right_operand) ? left_operand : right_operand;
}

/** @brief Select value_a if it is strictly larger; otherwise return value_b,
 * including on an unordered comparison.
 */
static float32_t max_f (float32_t left_operand, float32_t right_operand)
{
    return (left_operand > right_operand) ? left_operand : right_operand;
}

/** @brief Approximate a positive square root with twelve Newton iterations.
 * Return zero for nonpositive inputs.
 */
static float32_t sqrt_f (float32_t operand)
{

    if (operand <= 0.0f)
    {
        return 0.0f;
    }

    float32_t guess = (operand > 1.0f) ? operand : 1.0f;

    uint32_t iteration;
    for (iteration = 0U; iteration < SQRT_NEWTON_ITERATIONS; iteration++)
    {
        guess = 0.5f * (guess + (operand / guess));
    }

    return guess;
}

/** @brief Return the magnitude of a signed wheel-count delta. Callers supply a
 * bounded encoder delta greater than INT32_MIN.
 */
static int32_t abs_i (int32_t operand)
{
    return (operand < 0) ? -operand : operand;
}

/** @brief Advance a millisecond timer by dt_ms, saturating at max_ms. The
 * current value_ms must already be within that limit.
 */
static uint32_t add_ms (uint32_t value_ms, uint32_t dt_ms, uint32_t max_ms)
{
    return ((max_ms - value_ms) > dt_ms) ? (value_ms + dt_ms) : max_ms;
}

/** @brief Return true strictly after the microsecond deadline.
 * Compare times less than half a counter turn apart. Zero and the exactly
 * half-range interval both fail the check.
 */
static bool time_after (uint32_t now_us, uint32_t deadline_us)
{
    const uint32_t elapsed = now_us - deadline_us;
    return (elapsed != 0U) && (elapsed <= (UINT32_MAX / 2U));
}

/** @brief Read the state stored in the mutex-protected controller word.
 */
static motion_state_t read_motion_state (void)
{
    return (motion_state_t)g_ctl.state;
}

/** @brief Store the supplied motion state in the controller word under the
 * caller-held motion mutex.
 */
static void set_state (motion_state_t state)
{
    g_ctl.state = (uint32_t)state;
}

/** @brief Return whether the state drives velocity, distance or turn control.
 */
static bool is_active (motion_state_t state)
{
    return (((unsigned int)MOTION_VELOCITY == state) ||
            ((unsigned int)MOTION_DRIVE == state)) ||
           ((unsigned int)MOTION_TURN == state);
}

/** @brief Return the direction multiplier for a valid wheel in the active
 * drive or turn, using its retained reverse flag.
 */
static float32_t mv_sign (wheel_encoder_id_t wheel)
{
    return (((g_ctl.mv_back >> (uint32_t)wheel) & 1U) != 0U) ? -1.0f : 1.0f;
}

/** @brief Brake both wheels immediately, acknowledge a pending brake and reset
 * both wheel PID controllers. Called by the motor owner.
 */
static void outputs_safe (void)
{

    motor_control_brake_all();
    g_ctl.b_is_brake_pending = 0U;

    uint32_t wheel_index;
    for (wheel_index = 0U; wheel_index < (uint32_t)WHEEL_COUNT; wheel_index++)
    {
        speed_pid_reset(&g_wheel[wheel_index].pid);
    }
}

/** @brief Advance the eight-bit command ID. Wrap from 255 to 1 so zero
 * continues to denote no command.
 */
static void new_command (void)
{
    g_ctl.cmd_id++; /* The eight-bit field wraps from 255 to zero. */
    if (0U == g_ctl.cmd_id)
    {
        g_ctl.cmd_id = 1U; /* Zero means no command. */
    }
}

/** @brief Deliver the current ID, outcome and fault synchronously to the
 * optional sink. The record lives only for the callback; the sink runs
 * with the motion mutex held.
 */
static void post_motion_outcome (motion_outcome_t outcome, motion_fault_t fault)
{
    motion_report_t report_entry;

    report_entry.command_id = (uint8_t)g_ctl.cmd_id;
    report_entry.outcome    = outcome;
    report_entry.fault      = fault;
    report_entry.b_was_lost = false; /* The queue sets this when needed. */
    if (gp_sink != NULL)
    {
        gp_sink(&report_entry);
    }
}

/** @brief Brake, latch all supplied fault bits, enter FAULT and report the
 * highest-priority fault in this input mask.
 * @pre Callers combine masks from motion_pack_fault_bit, which sets only
 * the five defined motion fault bits. The narrowing preserves every bit.
 */
static void raise_faults (uint32_t fault_bits)
{
    outputs_safe();
    g_ctl.fault_bits |= fault_bits;
    set_state(MOTION_FAULT);
    post_motion_outcome(MOTION_OUTCOME_FAULT,
                        motion_pack_fault_first((uint8_t)fault_bits));
}

/** @brief Convert one motion fault to its bit mask and run the same brake,
 * latch and notification sequence.
 */
static void raise_fault (motion_fault_t fault)
{
    raise_faults(motion_pack_fault_bit(fault));
}

/** @brief Enter COMPLETE and report DONE for the current command. Callers own
 * the decision to brake; a zero-length request can complete here.
 */
static void complete (void)
{
    set_state(MOTION_COMPLETE);
    post_motion_outcome(MOTION_OUTCOME_DONE, MOTION_FAULT_NONE);
}

/** @brief Capture each wheel count as its new stall reference and clear the
 * stall and wrong-direction timers.
 */
static void clear_checks (void)
{
    uint32_t wheel_index;

    for (wheel_index = 0U; wheel_index < (uint32_t)WHEEL_COUNT; wheel_index++)
    {
        /* clang-format off */
        g_wheel[wheel_index].stall_ref =
            wheel_encoder_bits((wheel_encoder_id_t)wheel_index);
        g_wheel[wheel_index].stall_ms  = 0U;
        g_wheel[wheel_index].wrong_ms  = 0U;
        /* clang-format on */
    }
}

/** @brief Update one valid wheel PID from target speed in m/s, measured speed,
 * acceleration and elapsed seconds. Apply its duty to the motor.
 */
static void wheel_drive (wheel_encoder_id_t wheel, float32_t target_mps,
                         float32_t dt_s)
{
    const float32_t duty = speed_pid_update(
        &g_wheel[wheel].pid, target_mps, motion_estimation_wheel_speed(wheel),
        motion_estimation_wheel_accel(wheel), dt_s);

    motor_control_set(wheel, duty);
}

/** @brief Update one driven wheel's millisecond timers. Return the first
 * applicable fault in stall, direction, encoder order, or NONE.
 */
static motion_fault_t wheel_check (wheel_encoder_id_t wheel, uint32_t dt_ms)
{
    wheel_ctrl_t *  p_wheel = &g_wheel[wheel];
    const uint32_t  count   = wheel_encoder_bits(wheel);
    const float32_t cmd     = speed_pid_setpoint(&p_wheel->pid);
    const float32_t speed   = motion_estimation_wheel_speed(wheel);

    /* Declare a stall when the driven encoder advances fewer than
     * STALL_MIN_PROGRESS counts. A jam, a disconnected encoder or a failed
     * channel can produce this result. A single working channel may
     * alternate between +1 and -1.
     */
    if ((abs_f(cmd) < CMD_ACTIVE_MPS) ||
        (abs_i(wheel_encoder_delta(count, p_wheel->stall_ref)) >=
         STALL_MIN_PROGRESS))
    {
        p_wheel->stall_ref = count;
        p_wheel->stall_ms  = 0U;
    }
    else
    {
        p_wheel->stall_ms = add_ms(p_wheel->stall_ms, dt_ms, STALL_MS_MAX);
    }

    /* Flag motion clearly opposite to the command. Reversed motor or encoder
     * wiring can otherwise drive the controller farther from the target.
     */
    if (((abs_f(cmd) > DIRECTION_MIN_MPS) &&
         (abs_f(speed) > DIRECTION_MIN_MPS)) &&
        ((cmd > 0.0f) != (speed > 0.0f)))
    {
        p_wheel->wrong_ms = add_ms(p_wheel->wrong_ms, dt_ms, WRONG_MS_MAX);
    }
    else
    {
        p_wheel->wrong_ms = 0U;
    }

    if (p_wheel->stall_ms >= STALL_TIMEOUT_MS)
    {
        return MOTION_FAULT_STALL;
    }

    if (p_wheel->wrong_ms >= DIRECTION_FAULT_MS)
    {
        return MOTION_FAULT_DIRECTION;
    }

    if (p_wheel->glitch_bucket > (uint32_t)ENCODER_GLITCH_LIMIT)
    {
        return MOTION_FAULT_ENCODER;
    }

    return MOTION_FAULT_NONE;
}

/** @brief Accumulate each wheel's modular glitch-count delta, cap the bucket,
 * then decay it by one if nonzero.
 */
static void track_glitches (void)
{
    uint32_t wheel_index;

    for (wheel_index = 0U; wheel_index < (uint32_t)WHEEL_COUNT; wheel_index++)
    {
        wheel_ctrl_t * p_wheel = &g_wheel[wheel_index];
        const uint32_t seen =
            wheel_encoder_glitches((wheel_encoder_id_t)wheel_index);
        uint32_t bucket = p_wheel->glitch_bucket +
                          ((seen - p_wheel->glitch_seen) & GLITCH_SEEN_MASK);

        p_wheel->glitch_seen = seen & GLITCH_SEEN_MASK;

        if (bucket > (2U * (uint32_t)ENCODER_GLITCH_LIMIT))
        {
            bucket = 2U * (uint32_t)ENCODER_GLITCH_LIMIT;
        }

        if (bucket > 0U)
        {
            bucket--;
        }

        p_wheel->glitch_bucket = bucket;
    }
}

/** @brief Return signed metres travelled from the saved start count along the
 * valid wheel's commanded drive or turn direction.
 */
static float32_t move_wheel_progress (wheel_encoder_id_t wheel)
{
    return mv_sign(wheel) *
           (float32_t)wheel_encoder_delta(wheel_encoder_bits(wheel),
                                          g_mv_start[wheel].bits) *
           motion_estimation_scale();
}

/** @brief Return true at the brake point. Until then, synchronize wheel
 * progress and apply speed commands using the elapsed dt_s seconds.
 */
static bool move_step (float32_t dt_s)
{
    const float32_t lead =
        (g_ctl.b_is_move_turn != 0U) ? g_lead_turn_m : g_lead_drive_m;
    const float32_t value_dl = move_wheel_progress(WHEEL_LEFT);
    const float32_t value_dr = move_wheel_progress(WHEEL_RIGHT);
    const float32_t to_brake =
        g_mv_dist_m - lead - (0.5f * (value_dl + value_dr));
    const float32_t crawl = min_f(g_mv_speed_mps, MIN_SPEED_MPS);

    if (to_brake <= 0.0f)
    {
        return true;
    }

    /* Reduce speed to a crawl within CRAWL_DIST_M of the brake point to make
     * coasting after the brake more repeatable.
     */
    float32_t speed_mps =
        sqrt_f(2.0f * DECEL_MPS2 * max_f(to_brake - CRAWL_DIST_M, 0.0f));
    speed_mps = max_f(min_f(speed_mps, g_mv_speed_mps), crawl);

    /* Slow the wheel that leads, down to a stop if needed, and speed up the
     * other wheel to hold a straight line.
     */
    float32_t corr = SYNC_GAIN * (value_dl - value_dr);
    corr           = max_f(min_f(corr, speed_mps), -speed_mps);

    wheel_drive(WHEEL_LEFT, mv_sign(WHEEL_LEFT) * max_f(speed_mps - corr, 0.0f),
                dt_s);
    wheel_drive(WHEEL_RIGHT,
                mv_sign(WHEEL_RIGHT) * max_f(speed_mps + corr, 0.0f), dt_s);

    return false;
}

/* End of file. */

