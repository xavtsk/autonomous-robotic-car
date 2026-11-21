/** @file test_motion_feedback_main.c
 * @brief Exercise motion feedback, report handling and counter wrap with fakes.
 * Unsigned casts spell out the original modulo conversion.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "motion.h"
#include "../../app_program/app_types.h"
#ifdef S2_LEGACY_MODULE_NAMES
#include "motion_controller.h"
#else
#include "ctrl_motion.h"
#endif
#include "test_motion_feedback_main.h"
#ifdef TEST_MOTION_STORAGE
#include "../compliance/motion_storage_main.h"
#endif
#include "motor_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef TEST_SEMANTIC_TRACE
#include "motion_trace.h"
#endif

#define STARTUP_SETTLING_TICKS 12U
#define DEBUG_OBSERVATION_COUNT 100U
#define FAULT_RECOVERY_TICKS 30U
#define WRAP_EXERCISE_COMMAND_COUNT 260U

static uint32_t     g_clock_us;
static int32_t      g_count[2];
static float32_t    g_duty[2];
static bool         gb_is_mutex_locked;
static bool         gb_is_debug_lock_busy;
static unsigned int g_wakes;
static bool         gb_should_abort_on_delay;

static uint32_t     g_watchdog_tick;
static uint32_t     g_reset_bits = RESETS_RESET_TIMER;
static unsigned int g_clock_starts;

static uint16_t     g_queue[MOTION_REPORT_QUEUE_LEN];
static unsigned int g_queued;

static bool gb_is_encoder_running = true;

static void step_and_observe (void);
static bool read_and_observe (motion_report_t * p_report);
static void step (void);
static void test_reports_and_wrap (void);

#ifdef TEST_MOTION_STORAGE
/** @brief Report whether the fake RTOS mutex is currently held. */
bool motion_fixture_locked (void)
{
    return gb_is_mutex_locked;
}

/** @brief Set the fixture clock in microseconds, including wraparound. */
void motion_fixture_set_time (uint32_t timestamp_us)
{
    g_clock_us = timestamp_us;
}
#endif

/** @brief Return the fixture mutex ID without allocating a kernel object.
 */
ID tk_cre_mtx (const T_CMTX * p_config)
{
    (void)p_config;
    return 1;
}

/** @brief Acquire the simulated motion lock unless the configured contention
 * or storage test rejects the attempt.
 */
ER tk_loc_mtx (ID h_mutex, int timeout)
{
    (void)h_mutex;
#ifdef TEST_MOTION_STORAGE
    if (!motion_storage_allow_lock(gb_is_mutex_locked))
    {
        return -1;
    }
#endif
    if (gb_is_debug_lock_busy)
    {
        assert(TMO_POL == timeout);
        return -1;
    }

    assert(!gb_is_mutex_locked);
    gb_is_mutex_locked = true;
    return E_OK;
}

/** @brief Mark the simulated mutex free after checking ownership. The
 * optional storage fixture observes the unlock.
 */
ER tk_unl_mtx (ID h_mutex)
{
#ifdef TEST_MOTION_STORAGE
    motion_storage_observe_unlock();
#endif
    (void)h_mutex;
    assert(gb_is_mutex_locked);
    gb_is_mutex_locked = false;
    return E_OK;
}

/** @brief Count the wake request and return success without scheduling
 * another thread.
 */
ER tk_wup_tsk (ID h_task)
{
    (void)h_task;
    g_wakes++;
    return E_OK;
}

/** @brief Advance the clock by the requested milliseconds and inject a stop
 * if armed.
 */
ER tk_dly_tsk (uint32_t milliseconds)
{
    g_clock_us += milliseconds * 1000U;

    if (gb_should_abort_on_delay)
    {
        gb_should_abort_on_delay = false;
        stop();
    }

    return E_OK;
}

/** @brief Report an empty wake queue without blocking the calling task.
 */
ER tk_can_wup (ID h_task)
{
    (void)h_task;
    return E_OK;
}

/** @brief Read the simulated microsecond clock, reset state, reset-done bits
 * or watchdog tick.
 */
uint32_t in_w (uint32_t address)
{
    if (TIMER_TIMERAWL == address)
    {
        return g_clock_us;
    }

    if (RESETS_RESET == address)
    {
        return g_reset_bits;
    }

    if (RESETS_RESET_DONE == address)
    {
        return RESETS_RESET_TIMER;
    }

    return g_watchdog_tick;
}

/** @brief Capture a simulated watchdog write, set its running bit and count
 * a clock start.
 */
void out_w (uint32_t address, uint32_t value)
{
    (void)address;
    g_watchdog_tick = value | (1U << 10);
    g_clock_starts++;
}

/** @brief Clear the selected bits of the simulated reset register.
 */
void clr_w (uint32_t address, uint32_t value)
{
    assert(RESETS_RESET == address);
    g_reset_bits &= ~value;
}

/** @brief Require two-byte messages, empty the fake queue and return its ID.
 */
ID tk_cre_mbf (const T_CMBF * p_config)
{
    assert(2 == p_config->maxmsz);
    g_queued = 0U;
    return 2;
}

/** @brief Copy one two-byte motion report while the lock is held. Return an
 * error if the queue is full.
 */
ER tk_snd_mbf (ID h_message_buffer, const void * p_data, INT size, TMO timeout)
{
    (void)h_message_buffer;
    (void)timeout;
    assert(gb_is_mutex_locked && 2 == size);

    if (MOTION_REPORT_QUEUE_LEN == g_queued)
    {
        return -1;
    }

    memcpy(&g_queue[g_queued++], p_data, 2);
    return E_OK;
}

/** @brief Pop the oldest two-byte report while unlocked. An empty queue
 * returns an error before writing the output.
 */
INT tk_rcv_mbf (ID h_message_buffer, void * p_data, TMO timeout)
{
    (void)h_message_buffer;
    (void)timeout;
    assert(!gb_is_mutex_locked);

    if (0U == g_queued)
    {
        return -1;
    }

    memcpy(p_data, &g_queue[0], 2);

    for (unsigned int report_index = 1U; report_index < g_queued;
         report_index++)
    {
        g_queue[report_index - 1U] = g_queue[report_index];
    }

    g_queued--;
    return 2;
}

#ifdef TEST_MOTION_STORAGE
/** @brief Enable the fake encoder before seeding feedback boundary values. */
void motion_fixture_enable_encoder (void)
{
    gb_is_encoder_running = true;
}

/** @brief Make the next report send observe a full fake message buffer. */
void motion_fixture_fill_reports (void)
{
    g_queued = MOTION_REPORT_QUEUE_LEN;
}
#endif

/** @brief Accept an encoder sampling tick without changing the simulated
 * counts.
 */
void wheel_encoder_sample (void)
{
}

/** @brief Return the fixture flag indicating whether encoder sampling is
 * active.
 */
bool wheel_encoder_is_running (void)
{
    return gb_is_encoder_running;
}

/** @brief Clear both simulated encoder counts and report success.
 */
int32_t wheel_encoder_init (bool b_is_left, bool b_is_right)
{
    (void)b_is_left;
    (void)b_is_right;
    g_count[0] = g_count[1] = 0;
    return 0;
}

/** @brief Return the selected test wheel's signed encoder count.
 */
int32_t wheel_encoder_count (wheel_encoder_id_t wheel)
{
    return g_count[wheel];
}

/** @brief Return the selected encoder count masked to the hardware counter
 * width.
 */
uint32_t wheel_encoder_bits (wheel_encoder_id_t wheel)
{
    return (uint32_t)g_count[wheel] & WHEEL_COUNT_MASK;
}

/** @brief Return signed displacement in counts using the encoder's wrapping
 * counter width.
 */
int32_t wheel_encoder_delta (uint32_t now_bits, uint32_t then_bits)
{
    uint32_t delta = (now_bits - then_bits) & WHEEL_COUNT_MASK;
    return (delta & (1UL << (WHEEL_COUNT_BITS - 1U)))
               ? (int32_t)delta - (int32_t)(WHEEL_COUNT_MASK + 1U)
               : (int32_t)delta;
}

/** @brief Report zero encoder glitches for either simulated wheel.
 */
uint32_t wheel_encoder_glitches (wheel_encoder_id_t wheel)
{
    (void)wheel;
    return 0U;
}

/** @brief Ignore the polarity request and retain the preconfigured encoder
 * direction.
 */
void wheel_encoder_set_inverted (wheel_encoder_id_t wheel, bool b_is_inverted)
{
    (void)wheel;
    (void)b_is_inverted;
}

/** @brief Clear both simulated wheel duties.
 */
void motor_control_init (bool b_is_left, bool b_is_right)
{
    (void)b_is_left;
    (void)b_is_right;
    g_duty[0] = g_duty[1] = 0.0f;
}

/** @brief Save a duty command for the selected test wheel.
 */
void motor_control_set (wheel_encoder_id_t wheel, float32_t value)
{
    g_duty[wheel] = value;
}

/** @brief Return the duty last written to the selected test wheel.
 */
float32_t motor_control_duty (wheel_encoder_id_t wheel)
{
    return g_duty[wheel];
}

/** @brief Set both recorded wheel duties to zero.
 */
void motor_control_brake_all (void)
{
    g_duty[0] = g_duty[1] = 0.0f;
}

/** @brief Run the feedback and fault assertions before storage stress cases.
 * @details Step the production motion owner with a simulated clock and
 * encoder counters. Reading feedback or debug snapshots must preserve
 * motor duty and the published timestamp until the next control step.
 * Exercise a busy debug lock, the stale-command watchdog, reset, a new
 * feedback epoch and calibration admission.
 * @return Zero after all assertions and report boundary cases pass.
 */
int motion_fixture_run (void)
{
    motion_debug_snapshot_t debug;
    assert(!motion_try_debug_snapshot(NULL));
    motion_feedback_t first;
    motion_feedback_t second;
    unsigned int      index;
    assert(!motion_get_feedback(&first));
    assert(motion_create() == E_OK);
    motion_attach_task(1);
    motion_init();
    assert(motion_get_feedback(&first) && !first.b_is_feedback_valid);

    for (index = 0U; index < STARTUP_SETTLING_TICKS; index++)
    {
        g_clock_us += 10000U;
        step_and_observe();
    }

    assert(1U == g_clock_starts && 0U == g_reset_bits);
    assert(motion_get_feedback(&first) && first.b_is_feedback_valid &&
           first.b_is_braked && 1U == first.epoch);
    g_count[0]  = 100;
    g_count[1]  = 100;
    g_clock_us += 5000U;
    assert(motion_get_feedback(&second));
    {
        const float32_t before_l = g_duty[0];
        const float32_t before_r = g_duty[1];

        for (unsigned observation_index = 0;
             observation_index < DEBUG_OBSERVATION_COUNT; ++observation_index)
        {
            assert(motion_try_debug_snapshot(&debug));
        }

        assert(debug.timestamp_us == second.timestamp_us &&
               debug.b_is_debug_valid == second.b_is_feedback_valid);
        assert(g_duty[0] == before_l && g_duty[1] == before_r &&
               !gb_is_mutex_locked);
        gb_is_debug_lock_busy = true;
        assert(!motion_try_debug_snapshot(&debug));
        gb_is_debug_lock_busy = false;
    }
    assert(second.timestamp_us == first.timestamp_us &&
           second.centre_m == first.centre_m);
    g_clock_us += 5000U;
    step_and_observe();
    assert(motion_get_feedback(&second));
    assert(second.timestamp_us == g_clock_us &&
           second.centre_m > first.centre_m);
    motion_navigation_heartbeat(true);
    assert(motion_request_calibration() == (unsigned)MOTION_ERR_BUSY);
    assert(motion_request_manoeuvre(MANOEUVRE_LEFT) == (unsigned)MOTION_OK);
    g_clock_us += 10000U;
    step_and_observe();
    assert(motion_get_feedback(&first) && (unsigned)MOTION_TURN == first.state);
    {
        const float32_t before_l = g_duty[0];
        const float32_t before_r = g_duty[1];

        for (unsigned observation_index = 0;
             observation_index < DEBUG_OBSERVATION_COUNT; ++observation_index)
        {
            assert(motion_try_debug_snapshot(&debug));
        }

        assert((unsigned)MOTION_TURN == debug.state && debug.b_is_debug_valid &&
               debug.timestamp_us == first.timestamp_us);
        assert(debug.setpoint_mps[0] <= 0.0f && debug.setpoint_mps[1] >= 0.0f);
        assert(g_duty[0] == before_l && g_duty[1] == before_r &&
               !gb_is_mutex_locked);
        assert(motion_get_feedback(&second) &&
               second.timestamp_us == first.timestamp_us &&
               second.state == first.state);
    }
    g_clock_us += 40000U;
    step_and_observe();
    assert(motion_get_feedback(&first));
    assert((unsigned)MOTION_FAULT == first.state &&
           (unsigned)MOTION_FAULT_STALE_COMMAND == first.fault);
    assert(0.0f == g_duty[0] && 0.0f == g_duty[1]);
    motion_navigation_heartbeat(false);

    for (index = 0; index < FAULT_RECOVERY_TICKS; index++)
    {
        g_clock_us += 10000U;
        step_and_observe();
    }

    assert(motion_reset());
    stop();
    assert(g_wakes > 0U);
    motion_init();
    assert(motion_get_feedback(&second) && !second.b_is_feedback_valid &&
           2U == second.epoch);
    g_clock_us += 10000U;
    step_and_observe();
    assert(motion_get_feedback(&second) && second.b_is_feedback_valid);
    assert(motion_request_calibration() == (unsigned)MOTION_OK);
    assert(motion_get_feedback(&second) && !second.b_is_feedback_valid);
    assert(!gb_is_mutex_locked);
    test_reports_and_wrap();
    puts("PASS: production motion feedback timestamps, epochs, calibration "
         "exclusion and manoeuvre watchdog");
    return 0;
}

#ifndef TEST_MOTION_STORAGE
/** @brief Return the shared motion-feedback fixture result to the invoking
 * runner.
 */
int main (void)
{
    return motion_fixture_run();
}
#endif

/** @brief Step the real controller once, then sample it for a trace build. */
static void step_and_observe (void)
{
    motion_process();
#ifdef TEST_SEMANTIC_TRACE
    motion_trace_feedback();
#endif
}

/** @brief Consume one report and record the same result in a trace build. */
static bool read_and_observe (motion_report_t * p_report)
{
    const bool b_did_receive_report = motion_get_report(p_report);
#ifdef TEST_SEMANTIC_TRACE
    motion_trace_report(b_did_receive_report, p_report);
#endif
    return b_did_receive_report;
}

/** @brief Advance the simulated clock by 10 ms and process one observed
 * motion update.
 */
static void step (void)
{
    g_clock_us += 10000U;
    step_and_observe();
}

/** @brief Exercise report delivery and timestamp wrap, including reset of
 * the motion feedback distance.
 */
static void test_reports_and_wrap (void)
{
    motion_feedback_t before;
    motion_feedback_t after;
    motion_report_t   report;
    uint8_t           latest;
    unsigned int      original_clock_starts = g_clock_starts;
    assert(motion_create() == E_OK);
    motion_init();
    assert(g_clock_starts ==
           original_clock_starts); /* Leave the running clock intact. */
    for (unsigned int tick_index = 0U; tick_index < STARTUP_SETTLING_TICKS;
         tick_index++)
    {
        step();
    }
    /* Command IDs wrap 255 -> 1; every result is drained before ID reuse. */
    uint8_t expected_id = motion_command_id();

    for (unsigned int command_index = 1U;
         command_index <= WRAP_EXERCISE_COMMAND_COUNT; command_index++)
    {
        assert(motion_request_drive(0.0f, 0.1f) == (unsigned)MOTION_OK);
        expected_id = 255U == expected_id ? 1U : (uint8_t)(expected_id + 1U);
        assert(motion_command_id() == expected_id);
        assert(read_and_observe(&report) &&
               report.command_id == motion_command_id());
        assert((unsigned)MOTION_OUTCOME_DONE == report.outcome);
        assert(!read_and_observe(&report));
    }
    /* Fill queue with zero-distance commands (no motor motion), then overflow.
     */
    for (unsigned int command_index = 0U;
         command_index <= MOTION_REPORT_QUEUE_LEN; command_index++)
    {
        assert(motion_request_drive(0.0f, 0.1f) == (unsigned)MOTION_OK);
        stop();
        step();
    }

    assert(motion_get_feedback(&after) && 1U == after.report_loss_count);
    assert(motion_report_peak() == MOTION_REPORT_QUEUE_LEN);
    /* A newer command must survive consumption of older DONE reports. */
    assert(motion_request_drive(1.0f, 0.1f) == (unsigned)MOTION_OK);
    latest = motion_command_id();

    for (unsigned int report_index = 0U; report_index < MOTION_REPORT_QUEUE_LEN;
         report_index++)
    {
        assert(read_and_observe(&report) && report.command_id != latest);
        assert(ctrl_motion_state() == (unsigned)MOTION_DRIVE);
    }

    assert(!read_and_observe(&report));
    stop();
    assert(read_and_observe(&report) && report.b_was_lost &&
           report.command_id == latest &&
           (unsigned)MOTION_OUTCOME_CANCELLED == report.outcome);
    assert(motion_get_feedback(&after) && 1U == after.report_loss_count);
    step();

    g_count[0] = g_count[1] = (1U << 19U) - 2U;
    step();
    assert(motion_get_feedback(&before));
    g_count[0] += 4;
    g_count[1] += 4;
    step();
    assert(motion_get_feedback(&after));
    assert(after.centre_m > before.centre_m &&
           after.centre_m - before.centre_m < 0.001);
    g_count[0] -= 4;
    g_count[1] -= 4;
    step();
    assert(motion_get_feedback(&after));
    assert(after.centre_m ==
           before.centre_m); /* Wrap the counter in reverse. */
    gb_is_encoder_running = false;
    assert(motion_get_feedback(&after) && !after.b_is_feedback_valid);
    gb_is_encoder_running = true;
    /* Calibration completion (including abort) invalidates old epoch. */
    motion_init();
    step();
    assert(motion_get_feedback(&before));
    assert(motion_request_calibration() == (unsigned)MOTION_OK);
    gb_should_abort_on_delay = true;
    step_and_observe();
    assert(motion_get_feedback(&after) && !after.b_is_feedback_valid &&
           after.epoch == before.epoch + 1U);
    step();
    assert(motion_get_feedback(&after) && after.b_is_feedback_valid &&
           0.0 == after.centre_m);
}

/* End of file. */

