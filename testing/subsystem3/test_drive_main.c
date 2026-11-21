/** @file test_drive_main.c
 * @brief Real Buddy 2 controller/PID/estimator + real Buddy 3 core/bridge.
 *
 * Hardware edges and PWM are mocked; this cannot prove physical behaviour.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 */
#include "ctrl_motion.h"
#include "../../app_program/app_types.h"
#include "motion_estimation.h"
#include "motion_lease.h"
#include "test_drive_main.h"
#include "s3_drive.h"
#include <assert.h>
#include <stdio.h>

#define FIXTURE_IR_CHANNEL_COUNT 3U
#define CALIBRATION_SAMPLE_COUNT 10U
#define SCAN_LEADING_PAPER_TICKS 40U
#define SCAN_SYMBOL_ELEMENT_COUNT 9U
#define SCAN_SYMBOL_GAP_TICKS 4U
#define SCAN_TRAILING_PAPER_TICKS 45U
#define JUNCTION_TURN_WAIT_TICKS 30U
#define FAULT_IDLE_CHECK_TICKS 30U
#define RESET_FEEDBACK_CASE_COUNT 4U
#define RESET_TEST_TICK_US 10000U
#define JUNCTION_ACTION_CASE_COUNT 4U
#define PENDING_ACTION_TIMEOUT_TICKS 510U
#define PRE_JUNCTION_TRAVEL_TICKS 10U

static uint32_t           g_now;
static uint32_t           g_lease_stamp;
static uint32_t           g_epoch = 1U;
static bool               gb_has_lease;
static bool               gb_should_reject_velocity;
static bool               gb_should_reject_turn;
static bool               gb_should_reject_reset;
static unsigned int       g_polls;
static unsigned int       g_turns;
static unsigned int       g_velocities;
static unsigned int       g_stops;
static uint32_t           g_report_losses;
static motion_manoeuvre_t g_last_turn;
static int32_t            g_counts[2];
static float32_t          g_duty[2];

static motion_report_t g_reports[MOTION_REPORT_QUEUE_LEN];
static unsigned int    g_report_count;

static s3_drive_t g_drive;

static motion_result_t velocity (float32_t left_speed, float32_t right_speed);
static motion_result_t manoeuvre (motion_manoeuvre_t manoeuvre_kind);
static void            brake (void);
static void            post (const motion_report_t * p_report);
static bool            receive (motion_report_t * p_report);
static bool            reset (void);
static void            heartbeat (bool b_is_enabled);
static void            tick (uint16_t barcode, uint16_t left, uint16_t right,
                             bool b_is_valid);
static void hold (unsigned int ticks, uint16_t barcode, uint16_t left,
                  uint16_t right);
static void init (bool b_is_calibrated);
static void start (void);
static void scan_a (void);
static void test_turn_cycle (void);
static void test_faults_and_stops (void);
static void reset_observation (motion_feedback_t feedback);
static void test_reset_acknowledgement (void);
static void test_mapping_and_expiry (void);
static void enter_turn (void);
static void test_new_motion_contract (void);

/** @brief Return the selected test wheel's signed encoder count.
 */
int32_t wheel_encoder_count (wheel_encoder_id_t wheel)
{
    return g_counts[wheel];
}

/** @brief Return the selected encoder count masked to the hardware counter
 * width.
 */
uint32_t wheel_encoder_bits (wheel_encoder_id_t wheel)
{
    return (uint32_t)g_counts[wheel] & WHEEL_COUNT_MASK;
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

/** @brief Save a duty command for the selected test wheel.
 */
void motor_control_set (wheel_encoder_id_t wheel, float32_t value)
{
    g_duty[wheel] = value;
}

/** @brief Set both recorded wheel duties to zero.
 */
void motor_control_brake_all (void)
{
    g_duty[0] = 0.0f;
    g_duty[1] = 0.0f;
}

/** @brief Simulate motion and navigation together through turns, faults,
 * asynchronous resets and mapped-action expiry.
 */
int main (void)
{
    test_turn_cycle();
    test_faults_and_stops();
    test_reset_acknowledgement();
    test_mapping_and_expiry();
    test_new_motion_contract();
    puts("PASS: real Buddy 2 controller/PID + Buddy 3 navigation bridge; "
         "mocked hardware");
    return 0;
}

/** @brief Count the request. Return busy when velocity rejection is
 * injected, or submit both wheel speeds at the simulated timestamp.
 */
static motion_result_t velocity (float32_t left_speed, float32_t right_speed)
{
    g_velocities++;
    return gb_should_reject_velocity
               ? (unsigned)MOTION_ERR_BUSY
               : ctrl_motion_set_velocity(left_speed, right_speed, g_now);
}

/** @brief Record the manoeuvre and submit the corresponding drive or turn to
 * the motion controller unless rejection is injected.
 */
static motion_result_t manoeuvre (motion_manoeuvre_t manoeuvre_kind)
{
    int32_t left  = 1;
    int32_t right = 1;
    g_turns++;
    g_last_turn = manoeuvre_kind;

    if (gb_should_reject_turn)
    {
        return MOTION_ERR_BUSY;
    }

    if (((unsigned)MANOEUVRE_LEFT == manoeuvre_kind) ||
        ((unsigned)MANOEUVRE_U_TURN == manoeuvre_kind))
    {
        left = -1;
    }

    if ((unsigned)MANOEUVRE_RIGHT == manoeuvre_kind)
    {
        right = -1;
    }

    return ctrl_motion_request_move(
        (unsigned)MANOEUVRE_STRAIGHT == manoeuvre_kind ? MOTION_DRIVE
                                                       : MOTION_TURN,
        left, right, 0.02f, 0.10f, g_now);
}

/** @brief Count the stop request and stop the simulated motion controller.
 */
static void brake (void)
{
    g_stops++;
    ctrl_motion_stop();
}

/** @brief Append a copy of a motion report. Fail the fixture assertion if
 * the queue has reached capacity.
 */
static void post (const motion_report_t * p_report)
{
    assert(g_report_count < MOTION_REPORT_QUEUE_LEN);
    g_reports[g_report_count++] = *p_report;
}

/** @brief Return false on an empty queue. Otherwise pop its oldest report
 * into the output and acknowledge a completed move.
 */
static bool receive (motion_report_t * p_report)
{
    if (0U == g_report_count)
    {
        return false;
    }

    *p_report = g_reports[0];

    for (unsigned int report_index = 1U; report_index < g_report_count;
         report_index++)
    {
        g_reports[report_index - 1U] = g_reports[report_index];
    }

    g_report_count--;

    if ((unsigned)MOTION_OUTCOME_DONE == p_report->outcome)
    {
        g_polls++;
        ctrl_motion_ack_complete(p_report->command_id);
    }

    return true;
}

/** @brief Return false when reset rejection is injected. Otherwise return
 * the motion controller's reset result.
 */
static bool reset (void)
{
    return (!gb_should_reject_reset) && (ctrl_motion_reset());
}

/** @brief Record the lease state and refresh its timestamp whenever the
 * lease is enabled.
 */
static void heartbeat (bool b_is_enabled)
{
    gb_has_lease = b_is_enabled;

    if (b_is_enabled)
    {
        g_lease_stamp = g_now;
    }
}

/** @brief Advance simulated time by 10 ms and process the supplied raw
 * barcode and line sensor readings with the indicated validity.
 */
static void tick (uint16_t barcode, uint16_t left, uint16_t right,
                  bool b_is_valid)
{
    unsigned int           index;
    motion_feedback_t      feedback;
    ir_acquisition_frame_t frame = {0};
    g_now                       += 10000U;
    motion_estimation_update(0.01f);

    if (motion_lease_expired(gb_has_lease, g_now, g_lease_stamp))
    {
        ctrl_motion_nav_timeout();
        gb_has_lease = false;
    }

    ctrl_motion_step(0.01f, g_now);
    feedback           = (motion_feedback_t){motion_estimation_robot_dist(),
                                             g_now,
                                             g_epoch,
                                             ctrl_motion_state(),
                                             ctrl_motion_fault(),
                                             true,
                                             ctrl_motion_is_stopped(),
                                             ctrl_motion_command_id(),
                                             g_report_losses};
    frame.timestamp_us = g_now;

    for (index = 0; index < FIXTURE_IR_CHANNEL_COUNT; index++)
    {
        frame.sensor[index] =
            (ir_acquisition_sample_t){g_now, 0U, 0, true, b_is_valid};
    }

    frame.sensor[0].raw = barcode;
    frame.sensor[1].raw = left;
    frame.sensor[2].raw = right;
    assert(s3_drive_process(&g_drive, &frame, &feedback));
}

/** @brief Feed the three fixed raw ADC values as valid readings for the
 * requested number of 10 ms updates.
 */
static void hold (unsigned int ticks, uint16_t barcode, uint16_t left,
                  uint16_t right)
{
    unsigned int index;

    for (index = 0; index < ticks; index++)
    {
        tick(barcode, left, right, true);
    }
}

/** @brief Reset the simulated drive and wheel state. Apply fixture
 * calibration when requested and feed the initial idle readings.
 */
static void init (bool b_is_calibrated)
{
    line_navigation_config_t cfg  = line_navigation_defaults();
    const s3_drive_port_t    port = {
        velocity, manoeuvre, brake, receive, ctrl_motion_command_id,
        reset,    heartbeat};
    ir_calibration_collector_t collector;
    unsigned int               index;
    /* clang-format off */
    g_now                     = 0U;
    gb_has_lease              = false;
    g_epoch                   = 1U;
    gb_should_reject_velocity = false;
    gb_should_reject_turn     = false;
    gb_should_reject_reset    = false;
    g_turns                   = g_polls = g_velocities = g_stops = 0U;
    g_counts[0]               = g_counts[1] = 0;
    g_duty[0]                 = g_duty[1] = 0;
    /* clang-format on */
    motion_estimation_init(0.001f, 0.13f);
    g_report_count  = 0U;
    g_report_losses = 0U;
    ctrl_motion_init(0.30f, 0.0f, 0.0f, post);
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    assert(s3_drive_init(&g_drive, &cfg, &port));

    if (b_is_calibrated)
    {
        ir_calibration_begin(&collector);

        for (index = 0; index < CALIBRATION_SAMPLE_COUNT; index++)
        {
            assert(ir_calibration_add(&collector, IR_SURFACE_WHITE, 150U));
            assert(ir_calibration_add(&collector, IR_SURFACE_BLACK, 280U));
        }

        for (index = 0; index < FIXTURE_IR_CHANNEL_COUNT; index++)
        {
            assert(line_navigation_set_calibration(
                &g_drive.navigation, (ir_acquisition_sensor_id_t)index,
                &collector));
        }
    }

    hold(12U, 150U, 202U, 202U);
}

/** @brief Start the drive and assert that the simulated car reaches line
 * following under velocity control.
 */
static void start (void)
{
    assert(s3_drive_start(&g_drive, g_now));
    hold(5U, 150U, 202U, 202U);
    assert((unsigned)S3_DRIVE_FOLLOW == g_drive.state &&
           ctrl_motion_state() == (unsigned)MOTION_VELOCITY);
}

/** @brief Advance the simulated encoders and feed the start, A and stop
 * symbols with leading and trailing white readings.
 */
static void scan_a (void)
{
    unsigned int       sample_darkness;
    unsigned int       index;
    const unsigned int symbols[3] = {0x094U, 0x109U, 0x094U};
    /* Slow synthetic bar widths; encoder progress prevents a motion stall. */
    unsigned int run;
    unsigned int steps;

    for (run = 0; run < SCAN_LEADING_PAPER_TICKS; run++)
    {
        g_counts[0]++;
        g_counts[1]++;
        tick(150U, 202U, 202U, true);
    }

    for (sample_darkness = 0;
         sample_darkness < (sizeof symbols / sizeof symbols[0]);
         sample_darkness++)
    {
        for (index = 0; index < SCAN_SYMBOL_ELEMENT_COUNT; index++)
        {
            steps =
                (symbols[sample_darkness] & (1U << (8U - index))) ? 12U : 4U;

            for (run = 0; run < steps; run++)
            {
                g_counts[0]++;
                g_counts[1]++;
                tick((index % 2U) ? 150U : 280U, 202U, 202U, true);
            }
        }

        if (sample_darkness < 2U)
        {
            for (run = 0; run < SCAN_SYMBOL_GAP_TICKS; run++)
            {
                g_counts[0]++;
                g_counts[1]++;
                tick(150U, 202U, 202U, true);
            }
        }
    }

    for (run = 0; run < SCAN_TRAILING_PAPER_TICKS; run++)
    {
        g_counts[0]++;
        g_counts[1]++;
        tick(150U, 202U, 202U, true);
    }
}

/** @brief Exercise barcode-triggered braking, turning and return to
 * following, then assert that stopping clears the lease and wheel
 * duties.
 */
static void test_turn_cycle (void)
{
    unsigned int index;
    init(true);
    start();
    scan_a();
    hold(4U, 150U, 280U, 280U);
    assert((unsigned)S3_DRIVE_BRAKING == g_drive.state && 0U == g_turns);
    hold(20U, 150U, 280U, 280U);
    assert((unsigned)S3_DRIVE_MANOEUVRE == g_drive.state && 1U == g_turns &&
           (unsigned)MANOEUVRE_LEFT == g_last_turn);
    assert(S3_MODE_SUPPRESSED == (int)g_drive.navigation.mode);

    for (index = 0; (index < JUNCTION_TURN_WAIT_TICKS) &&
                    ((unsigned)S3_DRIVE_MANOEUVRE == g_drive.state);
         index++)
    {
        g_counts[0] -= 2;
        g_counts[1] += 2;
        tick(150U, 150U, 150U, true);
    }

    assert((unsigned)S3_DRIVE_REACQUIRE == g_drive.state && 1U == g_polls &&
           1U == g_turns);
    hold(25U, 150U, 202U, 202U);
    assert((unsigned)S3_DRIVE_FOLLOW == g_drive.state && 1U == g_polls &&
           g_velocities > 1U);
    assert((unsigned)S3_NAV_NONE == g_drive.pending);
    s3_drive_stop(&g_drive);
    tick(150U, 202U, 202U, true);
    assert((unsigned)S3_DRIVE_OFF == g_drive.state && !gb_has_lease &&
           0.0f == g_duty[0] && 0.0f == g_duty[1]);
}

/** @brief Exercise start rejection, drive faults and stop handling. Check
 * lease expiration across timestamp wraparound.
 */
static void test_faults_and_stops (void)
{
    unsigned int index;
    init(false);
    assert(!s3_drive_start(&g_drive, g_now));
    hold(20U, 150U, 202U, 202U);
    assert(0U == g_velocities && 0U == g_turns && !gb_has_lease);
    init(true);
    start();
    tick(150U, 202U, 202U, false);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state && !gb_has_lease &&
           g_stops > 0U);
    tick(150U, 202U, 202U, true);
    assert(0.0f == g_duty[0] && 0.0f == g_duty[1]);
    assert(s3_drive_reset(&g_drive, g_now));
    assert(s3_drive_reset_result(&g_drive) == (unsigned)S3_RESET_PENDING);
    hold(30U, 150U, 202U, 202U);
    assert(s3_drive_reset_result(&g_drive) == (unsigned)S3_RESET_OK &&
           (unsigned)S3_DRIVE_OFF == g_drive.state);
    start();
    g_epoch++;
    tick(150U, 202U, 202U, true);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state);
    init(true);
    start();
    hold(15U, 150U, 150U, 150U);
    assert((unsigned)S3_DRIVE_HALTED == g_drive.state &&
           (unsigned)S3_DRIVE_LINE_LOST == g_drive.reason);
    hold(10U, 150U, 202U, 202U);
    assert((unsigned)S3_DRIVE_HALTED == g_drive.state);
    init(true);
    start();
    hold(25U, 150U, 280U, 280U);
    assert((unsigned)S3_DRIVE_HALTED == g_drive.state &&
           (unsigned)S3_DRIVE_NO_DIRECTION == g_drive.reason && 0U == g_turns);
    init(true);
    start();
    gb_should_reject_velocity = true;
    tick(150U, 202U, 202U, true);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state &&
           (unsigned)S3_DRIVE_REJECTED == g_drive.reason);
    init(true);
    start();
    scan_a();
    gb_should_reject_turn = true;
    hold(25U, 150U, 280U, 280U);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state && 0U == g_polls);
    /* Independently running motion owner detects a stalled integration task,
     * including during a turn (which the old velocity timeout did not cover).
     */
    init(true);
    start();
    scan_a();
    hold(25U, 150U, 280U, 280U);
    assert(ctrl_motion_state() == (unsigned)MOTION_TURN);
    g_now += 40000U;
    assert(motion_lease_expired(gb_has_lease, g_now, g_lease_stamp));
    ctrl_motion_nav_timeout();
    ctrl_motion_step(0.01f, g_now);
    assert(ctrl_motion_state() == (unsigned)MOTION_FAULT);
    assert(0.0f == g_duty[0] && 0.0f == g_duty[1]);
    tick(150U, 202U, 202U, true);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state);

    for (index = 0; index < FAULT_IDLE_CHECK_TICKS; index++)
    {
        tick(150U, 202U, 202U, true);
    }
    /* Exactly one RESET: Buddy 2 changes state now, but the controller keeps
     * its cached fault until the next published feedback. No second press. */
    assert(g_drive.feedback.fault != (unsigned)MOTION_FAULT_NONE);
    assert(s3_drive_reset(&g_drive, g_now));
    assert(ctrl_motion_state() == (unsigned)MOTION_IDLE);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state &&
           s3_drive_reset_result(&g_drive) == (unsigned)S3_RESET_PENDING);
    tick(150U, 202U, 202U, true);
    assert((unsigned)S3_DRIVE_OFF == g_drive.state &&
           s3_drive_reset_result(&g_drive) == (unsigned)S3_RESET_OK);
    assert(!gb_has_lease && 0.0f == g_duty[0] && 0.0f == g_duty[1]);
    assert(!motion_lease_expired(true, 0x10U, 0xfffffff0U));
    assert(motion_lease_expired(true, 40000U, 0xfffffff0U));
}

/** @brief Deliver a synthetic feedback snapshot without advancing the motion
 * owner. The caller can inject slow, stale or unhealthy replies
 * separately from the real controller reset exercised by tick().
 */
static void reset_observation (motion_feedback_t feedback)
{
    ir_acquisition_frame_t frame = {0};
    g_now                       += 10000U;
    frame.timestamp_us           = g_now;

    for (unsigned int sensor_index = 0;
         sensor_index < (unsigned)S3_SENSOR_COUNT; sensor_index++)
    {
        frame.sensor[sensor_index] = (ir_acquisition_sample_t){
            g_now, sensor_index ? 202U : 150U, 0, true, true};
    }

    assert(s3_drive_process(&g_drive, &frame, &feedback));
}

/** @brief Exercise four reset scenarios and verify the resulting drive
 * state, motion requests and lease.
 */
static void test_reset_acknowledgement (void)
{
    motion_feedback_t feedback;
    unsigned int      before;
    /* Stale, unbraked, still-faulted and invalid snapshots cannot acknowledge
     * a reset. Each attempt terminates within the bounded 500 ms wait. */
    for (unsigned int scenario = 0U; scenario < RESET_FEEDBACK_CASE_COUNT;
         scenario++)
    {
        init(true);
        g_drive.state = S3_DRIVE_FAULT;
        assert(s3_drive_reset(&g_drive, g_now));
        assert(!s3_drive_reset(
            &g_drive, g_now)); /* Reject a second reset while one is pending. */
        assert(!s3_drive_start(&g_drive, g_now));
        feedback = g_drive.feedback;
        before   = g_velocities;

        for (unsigned int tick_index = 0U;
             tick_index < (S3_RESET_WAIT_US / RESET_TEST_TICK_US); tick_index++)
        {
            if (scenario != 0U)
            {
                feedback.timestamp_us = g_now + 10000U;
            }

            if (1U == scenario)
            {
                feedback.b_is_braked = false;
            }
            else if (2U == scenario)
            {
                feedback.state = MOTION_FAULT;
                feedback.fault = MOTION_FAULT_STALL;
            }
            else if (3U == scenario)
            {
                feedback.b_is_feedback_valid = false;
            }
            else
            {
                /* Use the scenario's existing feedback. */
            }

            reset_observation(feedback);
            assert((unsigned)S3_DRIVE_FAULT == g_drive.state);
        }

        assert(s3_drive_reset_result(&g_drive) == (unsigned)S3_RESET_FAILED &&
               !gb_has_lease);
        assert(g_velocities == before && 0U == g_turns);
    }
    /* A port rejection stays faulted and is reported immediately. */
    init(true);
    g_drive.state          = S3_DRIVE_FAULT;
    gb_should_reject_reset = true;
    assert(!s3_drive_reset(&g_drive, g_now));
    assert(s3_drive_reset_result(&g_drive) == (unsigned)S3_RESET_FAILED);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state && !gb_has_lease);
    /* STOP wins over a reset awaiting new feedback; a later healthy snapshot
     * cannot finish the cancelled attempt or start either motor. */
    gb_should_reject_reset = false;
    assert(s3_drive_reset(&g_drive, g_now));
    s3_drive_stop(&g_drive);
    assert(s3_drive_reset_result(&g_drive) == (unsigned)S3_RESET_FAILED);
    hold(30U, 150U, 202U, 202U);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state && 0U == g_velocities &&
           !gb_has_lease);
    /* It is safe to explicitly request RESET again after cancellation. */
    assert(s3_drive_reset(&g_drive, g_now));
    tick(150U, 202U, 202U, true);
    assert(s3_drive_reset_result(&g_drive) == (unsigned)S3_RESET_OK &&
           (unsigned)S3_DRIVE_OFF == g_drive.state && 0U == g_velocities &&
           !gb_has_lease);
    /* Persistent sensor failure may not be cleared by a healthy motor reset. */
    init(true);
    start();
    tick(150U, 202U, 202U, false);
    assert(s3_drive_reset(&g_drive, g_now));
    before = g_velocities;

    for (unsigned int tick_index = 0U;
         tick_index < (S3_RESET_WAIT_US / RESET_TEST_TICK_US); tick_index++)
    {
        tick(150U, 202U, 202U, false);
    }

    assert(s3_drive_reset_result(&g_drive) == (unsigned)S3_RESET_FAILED &&
           (unsigned)S3_DRIVE_FAULT == g_drive.state &&
           g_velocities == before && !gb_has_lease);
    assert(s3_drive_reset_result(0) == (unsigned)S3_RESET_FAILED);
}

/** @brief Check that each configured navigation action requests its
 * manoeuvre and that an expired action produces no turn.
 */
static void test_mapping_and_expiry (void)
{
    const line_navigation_action_t actions[4] = {
        S3_NAV_LEFT,
        S3_NAV_RIGHT,
        S3_NAV_STRAIGHT,
        S3_NAV_UTURN,
    };
    const motion_manoeuvre_t expected[4] = {
        MANOEUVRE_LEFT, MANOEUVRE_RIGHT, MANOEUVRE_STRAIGHT, MANOEUVRE_U_TURN};
    unsigned int index;
    unsigned int other_index;

    for (index = 0U; index < JUNCTION_ACTION_CASE_COUNT; index++)
    {
        line_navigation_action_t map[26] = {0};
        init(true);
        map[0] = actions[index];
        assert(line_navigation_set_mapping(&g_drive.navigation, map));
        start();
        scan_a();
        assert(g_drive.pending == actions[index]);
        hold(25U, 150U, 280U, 280U);
        assert(1U == g_turns && g_last_turn == expected[index]);
        s3_drive_stop(&g_drive);
        tick(150U, 202U, 202U, true);
        assert((unsigned)S3_DRIVE_OFF == g_drive.state && !gb_has_lease &&
               0U == g_polls);
    }

    {
        line_navigation_action_t map[26] = {0};
        init(true);
        assert(line_navigation_set_mapping(&g_drive.navigation, map));
        start();
        scan_a();
        assert((unsigned)S3_DRIVE_HALTED == g_drive.state &&
               (unsigned)S3_DRIVE_UNMAPPED == g_drive.reason);
    }
    init(true);
    start();
    scan_a();
    assert((unsigned)S3_NAV_LEFT == g_drive.pending);

    for (other_index = 0U; other_index < PENDING_ACTION_TIMEOUT_TICKS;
         other_index++)
    {
        g_counts[0]++;
        g_counts[1]++;
        tick(150U, 202U, 202U, true);
    }

    assert((unsigned)S3_DRIVE_HALTED == g_drive.state &&
           (unsigned)S3_DRIVE_TIMEOUT == g_drive.reason);
    assert((unsigned)S3_NAV_NONE == g_drive.pending && 0U == g_turns);
    /* A cancelled partial scan cannot become a command after explicit restart.
     */
    init(true);
    start();
    hold(40U, 150U, 202U, 202U);
    hold(4U, 280U, 202U, 202U);
    s3_drive_stop(&g_drive);
    hold(20U, 150U, 202U, 202U);
    assert(s3_drive_start(&g_drive, g_now));
    hold(20U, 150U, 202U, 202U);
    assert((unsigned)S3_NAV_NONE == g_drive.pending && 0U == g_turns);
}

/** @brief Start a calibrated drive, scan A and supply junction readings
 * until the manoeuvre state is reached.
 */
static void enter_turn (void)
{
    init(true);
    start();
    scan_a();
    hold(25U, 150U, 280U, 280U);
    assert((unsigned)S3_DRIVE_MANOEUVRE == g_drive.state);
}

/** @brief Check motion reports, command ownership and stop handling during
 * an active drive manoeuvre.
 */
static void test_new_motion_contract (void)
{
    unsigned int    sent;
    motion_report_t report;
    /* Pending stop and observed wheel stillness are not immediate. A short
     * ambiguous reading must not fault merely because the brake is pending. */
    init(true);
    start();

    for (unsigned int tick_index = 0U; tick_index < PRE_JUNCTION_TRAVEL_TICKS;
         tick_index++)
    {
        g_counts[0]++;
        g_counts[1]++;
        tick(150U, 202U, 202U, true);
    }

    sent = g_velocities;
    tick(150U, 280U, 280U, true);
    assert(g_drive.b_waits_for_stop && 0U == g_drive.command_id);
    hold(3U, 150U, 202U, 202U);
    assert(g_velocities == sent && (unsigned)S3_DRIVE_FOLLOW == g_drive.state);
    hold(15U, 150U, 202U, 202U);
    assert(g_velocities > sent && (unsigned)S3_DRIVE_FOLLOW == g_drive.state);
    assert(0U == g_report_count && 0U == g_drive.cancel_id);

    enter_turn();
    report = (motion_report_t){(uint8_t)(g_drive.command_id - 1U),
                               MOTION_OUTCOME_DONE, MOTION_FAULT_NONE, false};
    post(&report);
    tick(150U, 280U, 280U, true);
    assert((unsigned)S3_DRIVE_MANOEUVRE ==
           g_drive.state); /* An old ID leaves the turn pending. */
    report.command_id = g_drive.command_id;
    report.outcome    = MOTION_OUTCOME_CANCELLED;
    post(&report);
    tick(150U, 280U, 280U, true);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state &&
           (unsigned)S3_DRIVE_REPORT_FAULT == g_drive.reason);

    enter_turn();
    report = (motion_report_t){g_drive.command_id, MOTION_OUTCOME_DONE,
                               MOTION_FAULT_NONE, true};
    post(&report);
    tick(150U, 280U, 280U, true);
    assert((unsigned)S3_DRIVE_FAULT ==
           g_drive.state); /* Mark the report as lost. */

    enter_turn();
    g_report_losses++;
    tick(150U, 280U, 280U, true);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state &&
           !gb_has_lease); /* Finish without a later report. */

    enter_turn();
    ctrl_motion_stop(); /* A different caller cancelled the manoeuvre. */
    tick(150U, 280U, 280U, true);
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state);

    /* Drain the full bounded queue even when inactive. */
    init(true);
    report = (motion_report_t){100U, MOTION_OUTCOME_CANCELLED,
                               MOTION_FAULT_NONE, false};

    for (unsigned int report_index = 0U; report_index < MOTION_REPORT_QUEUE_LEN;
         report_index++)
    {
        post(&report);
    }

    tick(150U, 202U, 202U, true);
    assert(0U == g_report_count && (unsigned)S3_DRIVE_OFF == g_drive.state);
}

/* End of file. */

