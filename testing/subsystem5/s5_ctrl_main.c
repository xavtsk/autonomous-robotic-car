/** @file s5_ctrl_main.c
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
#include "motor_control.h"
#include "s3_drive.h"
#include "s5_ctrl_main.h"
#if APP_S4
#include "terr_link.h"
#include "../../app_program/subsystem4/buddy4.h"
#include "../../app_program/subsystem1/buddy1.h"
#endif
#include <assert.h>
#include <stdio.h>

#define FIXTURE_SENSOR_COUNT 3U
#define CALIBRATION_SAMPLE_COUNT 10U
#define CONTENTION_CHECK_TICKS 25U
#define BYPASS_START_WAIT_TICKS 500U
#define FAULT_DETECTION_WAIT_TICKS 30U
#define WHEEL_RESPONSE_WAIT_TICKS 10U
#define STOP_CONFIRM_WAIT_TICKS 100U
#define FULL_BYPASS_WAIT_TICKS 5000U

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

static s3_drive_t             g_drive;
static s5_codec_observation_t g_obstacle;
static bool                   gb_is_scanning_requested;
static bool                   gb_is_range_bad;

static unsigned g_scan_calls;
static unsigned g_peek_calls;
static unsigned g_peek_beside; /* Report peeks that still see the obstacle. */
static unsigned g_peek_angle;
static int      g_turn_sign = 1;
static s5_api_snapshot_result_t g_snapshot_result;
static uint32_t                 g_sample_age_us;

static bool gb_is_recovery_tape = true;

#if APP_S4
static uint16_t           g_imu_run;
static terr_data_result_t g_imu_event;

#endif

static motion_result_t manoeuvre (motion_manoeuvre_t manoeuvre_kind);

static void brake (void);

static bool reset (void);

static void tick (uint16_t barcode, uint16_t left, uint16_t right,
                  bool b_is_valid);

static void init (bool b_is_calibrated);

static void start (void);

static void simulate (void);

static void snapshot_contention (void);

#if APP_S4
static void stop_active_bypass (terr_data_result_t event);
#endif

#if APP_S4
/** @brief Write a moving, calibrated terrain sample at the simulated time,
 * the current run ID and a zero hump count.
 */
bool buddy4_read (terr_data_result_t * p_out, uint16_t * p_run,
                  uint16_t * p_humps)
{
    *p_out =
        (terr_data_result_t){.timestamp_us = g_now,
                             .flags = TERRAIN_CALIBRATED | TERRAIN_TILT_VALID,
                             .state = TERRAIN_MOVING};

    *p_run   = g_imu_run;
    *p_humps = 0;
    return true;
}

/** @brief Increment the simulated terrain run ID and return it through the
 * output.
 */
bool buddy4_begin_run (uint16_t * p_run)
{
    *p_run = ++g_imu_run;
    return true;
}

/** @brief Consume one pending impact or safety event. With no event, return
 * false and leave both outputs untouched.
 */
bool buddy4_event (terr_data_result_t * p_out, bool * pb_did_lose)
{
    if ((!g_imu_event.b_has_impact) &&
        ((unsigned)TERRAIN_SAFETY_NONE == g_imu_event.safety))
    {
        return false;
    }

    *p_out              = g_imu_event;
    p_out->timestamp_us = g_now;
    *pb_did_lose        = false;
    g_imu_event         = (terr_data_result_t){0};
    return true;
}

/** @brief Acknowledge fault updates without retaining the changed bits.
 */
ER buddy1_update_faults (UB changed, UB active)
{
    (void)changed;
    (void)active;
    return E_OK;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_hump (UH count, UH height)
{
    (void)count;
    (void)height;
    return E_OK;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_terrain (const telemetry_codec_terrain_t * p_time)
{
    (void)p_time;
    return E_OK;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_avoidance (const telemetry_codec_avoidance_t * p_first)
{
    (void)p_first;
    return E_OK;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_scan (const telemetry_codec_scan_t * p_scan)
{
    (void)p_scan;
    return E_OK;
}

/** @brief Acknowledge telemetry invalidation without changing a cached value.
 */
ER buddy1_invalidate (UB topic)
{
    (void)topic;
    return E_OK;
}
#endif

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

/** @brief Return the injected snapshot status. Only a successful read writes
 * the obstacle data and applies the simulated sample age.
 */
s5_api_snapshot_result_t s5_api_read_snapshot (s5_codec_observation_t * p_out)
{
    if ((unsigned)S5_SNAPSHOT_OK == g_snapshot_result)
    {
        *p_out           = g_obstacle;
        p_out->range_us -= g_sample_age_us;
    }

    return g_snapshot_result;
}

/** @brief Return whether the obstacle snapshot was copied successfully.
 */
bool s5_api_snapshot (s5_codec_observation_t * p_out)
{
    return s5_api_read_snapshot(p_out) == (unsigned)S5_SNAPSHOT_OK;
}

/** @brief Require stopped motion and record a scan request for the supplied
 * ID.
 */
bool s5_api_request_scan (uint16_t requested_scan_id)
{
    assert(ctrl_motion_is_stopped());
    g_scan_calls++;
    gb_is_scanning_requested = true;
    g_obstacle.scan_id       = requested_scan_id;
    return true;
}

/** @brief Require stopped motion and record the requested scan ID and servo
 * angle in degrees.
 */
bool s5_api_request_peek (uint16_t requested_scan_id, unsigned angle)
{
    assert(ctrl_motion_is_stopped());
    g_peek_calls++;
    g_peek_angle       = angle;
    g_obstacle.scan_id = requested_scan_id;
    return true;
}

/** @brief Clear the simulated scan request and reset the remembered peek
 * angle.
 */
void s5_api_cancel_scan (void)
{
    gb_is_scanning_requested = false;
    g_peek_angle             = 0;
}

/** @brief Accept the controller status fields without publishing telemetry.
 */
void s5_api_set_control_status (unsigned state, unsigned fault, unsigned route,
                                unsigned leg, unsigned pass, unsigned swapped)
{
    (void)state;
    (void)fault;
    (void)route;
    (void)leg;
    (void)pass;
    (void)swapped;
}

/** @brief Return a clear obstacle-control status word for this fixture.
 */
uint16_t s5_api_controller_status (void)
{
    return 0;
}

/** @brief Return the fixed servo angle in degrees used by this fixture.
 */
uint8_t s5_api_servo_angle (void)
{
    return 0;
}

/** @brief Submit a forward move with the requested distance in metres and
 * speed in metres per second at the simulated time.
 */
motion_result_t motion_request_drive (float32_t metres, float32_t speed)
{
    return ctrl_motion_request_move(MOTION_DRIVE, 1, 1, metres, speed, g_now);
}

/** @brief Convert degrees to wheel travel using the fixture axle geometry
 * and submit opposing wheel directions at the requested speed.
 */
motion_result_t motion_request_turn (float32_t degrees, float32_t speed)
{
    g_turn_sign = degrees > 0 ? 1 : -1;
    float32_t distance =
        (degrees > 0 ? degrees : -degrees) * 0.13f * 3.14159265f / 360.0f;
    return ctrl_motion_request_move(MOTION_TURN, -g_turn_sign, g_turn_sign,
                                    distance, speed, g_now);
}

/** @brief Run integrated obstacle avoidance on fake range and line
 * observations, including contention, STOP and loss of feedback.
 */
int main (void)
{
    snapshot_contention();
    init(true);
    start();
    g_obstacle.range_mm = 120;
    g_peek_beside       = 2;
    tick(150, 202, 202, true);
    assert(g_drive.b_is_bypassing &&
           S5_WAIT_STOP == (int)g_drive.obstacle.state && 0U == g_scan_calls);
    /* Actual controller applies STOP and produces its matching cancellation. */
    for (unsigned tick_index = 0; (tick_index < FULL_BYPASS_WAIT_TICKS) &&
                                  (g_drive.state != (unsigned)S3_DRIVE_FAULT);
         tick_index++)
    {
        /* Exercise scan and all seven movement handoffs under intermittent
         * contention. */
        g_snapshot_result =
            ((tick_index % 11U) == 0U) ? S5_SNAPSHOT_BUSY : S5_SNAPSHOT_OK;
        simulate();

        if (!g_drive.b_is_bypassing)
        {
            break;
        }
    }

    g_snapshot_result = S5_SNAPSHOT_OK;
    assert((unsigned)S3_DRIVE_FOLLOW == g_drive.state &&
           !g_drive.b_is_bypassing && 1U == g_scan_calls);
    assert(g_drive.obstacle.recovery.b_is_confirmed &&
           S3_MODE_TRACK == (int)g_drive.navigation.mode);
    /* Leg 3 measured the depth: 3 side peeks (2 still beside, 1 clear), 2
     * steps. */
    assert(3U == g_peek_calls &&
           (2U * S5_PEEK_STEP_MM) == g_drive.obstacle.searched_mm);
    /* Completed moves: out turn, out diagonal, turn parallel, approach, 2
     * steps, clearance, turn in, 135 mm onto the line = 9. The return diagonal
     * and the swerve turn are stopped at the tape (not completed). No barcode
     * manoeuvre. */
    assert(9U == g_polls && 0U == g_turns);
    s3_drive_stop(&g_drive);
    assert(!g_drive.b_is_bypassing && !gb_has_lease);
    init(true);
    start();
    gb_is_range_bad = true;
    simulate();
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state && !gb_has_lease);
    init(true);
    start();
    g_obstacle.range_mm = 120;
    simulate();
    s3_drive_stop(&g_drive);
    simulate();
    assert((unsigned)S3_DRIVE_OFF == g_drive.state && !g_drive.b_is_bypassing &&
           0U == g_scan_calls);
    init(true);
    start();
    g_obstacle.range_mm = 120;
    simulate();
    g_report_losses++;
    simulate();
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state &&
           S5_FAULT == (int)g_drive.obstacle.state &&
           S5_INTERRUPTED == (int)g_drive.obstacle.fault);
    init(true);
    start();
    g_obstacle.range_mm = 120;
    simulate();

    for (unsigned tick_index = 0; (tick_index < BYPASS_START_WAIT_TICKS) &&
                                  (!((g_drive.obstacle.b_has_issued) &&
                                     (0 == (int)g_drive.obstacle.leg)));
         tick_index++)
    {
        simulate();
    }

    assert(g_drive.obstacle.b_has_issued);
    gb_is_range_bad = true;
    simulate();
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state);
#if APP_S4
    stop_active_bypass((terr_data_result_t){.b_has_impact = true});
    stop_active_bypass((terr_data_result_t){.safety = TERRAIN_SAFETY_LIFTED});
    stop_active_bypass(
        (terr_data_result_t){.safety = TERRAIN_SAFETY_OVERTURNED});
    puts("PASS: impact, pickup and overturn cancel an active S5 bypass and "
         "brake both Buddy 2 motors");
#endif
    puts("PASS: real motion controller/PID + S5 + S3; brake acknowledgement,"
         " one report owner, bypass with side-peek depth search, real tape "
         "gate, STOP, stale range and report loss");
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
    g_obstacle.range_us          = g_now;
    g_obstacle.b_is_range_valid  = !gb_is_range_bad;

    if (gb_is_range_bad)
    {
        g_obstacle.range_mm      = 0;
        g_obstacle.b_has_no_echo = true;
    }

    if (g_peek_angle)
    {
        /* A 200 mm side reading still sees the obstacle. */
        unsigned side = g_peek_beside ? 200U : 2000U;

        if (g_peek_beside)
        {
            g_peek_beside--;
        }

        g_obstacle.b_is_scan_done = true;
        g_obstacle.scan_us        = g_now;

        if (g_peek_angle < 90U)
        {
            g_obstacle.profile.right_clear_mm = (uint16_t)side;
        }
        else
        {
            g_obstacle.profile.left_clear_mm = (uint16_t)side;
        }

        g_peek_angle = 0;
    }

    if (gb_is_scanning_requested)
    {
        /* clang-format off */
        g_obstacle.b_is_scan_done = true;
        g_obstacle.scan_us        = g_now;
        g_obstacle.profile        =
            (profile_obstacle_t){250, 100, 2000, 2000, 50, -50, 250, true};
        g_obstacle.range_mm       = 2000;
        gb_is_scanning_requested  = false;
        /* clang-format on */
    }

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

    for (index = 0; index < FIXTURE_SENSOR_COUNT; index++)
    {
        frame.sensor[index] =
            (ir_acquisition_sample_t){g_now, 0U, 0, true, b_is_valid};
    }

    frame.sensor[0].raw = barcode;
    frame.sensor[1].raw = left;
    frame.sensor[2].raw = right;
#if APP_S4
    terr_link_poll(&g_drive, g_now);
#endif
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
    g_now             = 0U;
    g_snapshot_result = S5_SNAPSHOT_OK;
    g_sample_age_us   = 0U;
#if APP_S4
    g_imu_run   = 0;
    g_imu_event = (terr_data_result_t){0};
    terr_link_init();
#endif
    /* clang-format off */
    g_obstacle                =
        (s5_codec_observation_t){.range_mm = 2000, .b_is_range_valid = true};
    g_scan_calls              = g_peek_calls = g_peek_beside = g_peek_angle = 0;
    gb_is_scanning_requested  = gb_is_range_bad = false;
    gb_is_recovery_tape       = true;
    gb_has_lease              = false;
    g_epoch                   = 1U;
    gb_should_reject_velocity = false;
    gb_should_reject_turn     = false;
    gb_should_reject_reset    = false;
    /* clang-format on */

    /* clang-format off */
    g_turns     = g_polls = g_velocities = g_stops = 0U;
    g_counts[0] = g_counts[1] = 0;
    g_duty[0]   = g_duty[1] = 0;
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

        for (index = 0; index < FIXTURE_SENSOR_COUNT; index++)
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
#if APP_S4
    assert(terr_link_begin(g_now));
#endif
    hold(5U, 150U, 202U, 202U);
    assert((unsigned)S3_DRIVE_FOLLOW == g_drive.state &&
           ctrl_motion_state() == (unsigned)MOTION_VELOCITY);
}

/** @brief Advance simulated encoder counts, obstacle observations and line
 * sensor inputs for the current simulated motion and avoidance
 * state.
 */
static void simulate (void)
{
    motion_state_t state = ctrl_motion_state();

    if ((unsigned)MOTION_DRIVE == state)
    {
        g_counts[0]++;
        g_counts[1]++;
    }

    if ((unsigned)MOTION_TURN == state)
    {
        g_counts[0] -= g_turn_sign;
        g_counts[1] += g_turn_sign;
    }
    /* Off the line (white) while going round; the tape is crossed on the
     * return diagonal and under the IR again during the swerve and hand-back.
     */
    bool b_is_tape_visible = false;
    bool b_is_off_course   = ((g_drive.b_is_bypassing) &&
                            (S5_MOVING == (int)g_drive.obstacle.state)) &&
                           ((int)g_drive.obstacle.leg >= 1);

    if ((((g_drive.b_is_bypassing) && (5 == (int)g_drive.obstacle.leg)) &&
         (g_drive.obstacle.b_has_issued)) &&
        (!g_drive.obstacle.b_is_after_tape))
    {
        float64_t travelled =
            motion_estimation_robot_dist() - g_drive.obstacle.leg_start_m;
        float64_t goal = (float64_t)g_drive.obstacle.plan.offset_mm / 1000.0;
        b_is_tape_visible =
            (gb_is_recovery_tape && (travelled > (goal - 0.07))) &&
            (travelled < (goal + 0.07));
    }

    if ((g_drive.b_is_bypassing) && ((int)g_drive.obstacle.leg >= 6))
    {
        b_is_tape_visible = gb_is_recovery_tape;
    }

    if ((g_drive.b_is_bypassing) && (S5_FINISH == (int)g_drive.obstacle.state))
    {
        b_is_tape_visible = gb_is_recovery_tape;
        b_is_off_course   = false;
    }

    tick(150, b_is_tape_visible ? 280 : (b_is_off_course ? 150 : 202),
         b_is_tape_visible ? 150 : (b_is_off_course ? 150 : 202), true);
}

/** @brief Inject busy, stale and unavailable observations and check the
 * response while following, avoiding and stopping.
 */
static void snapshot_contention (void)
{
    s5_codec_observation_t cached;
    init(true);
    start();
    uint32_t stamp    = g_now;
    g_snapshot_result = S5_SNAPSHOT_BUSY;
    simulate();
    assert((unsigned)S3_DRIVE_FOLLOW == g_drive.state &&
           g_drive.obstacle_snapshot_us == stamp);
    assert(s5_codec_observation_unpack(g_drive.obstacle_snapshot, &cached) &&
           cached.range_us == stamp);
    g_snapshot_result = S5_SNAPSHOT_OK;
    simulate();
    assert((unsigned)S3_DRIVE_FOLLOW == g_drive.state);
    /* A new valid START sample covers contention on the very first cycle. */
    init(true);
    assert(s3_drive_start(&g_drive, g_now));
#if APP_S4
    assert(terr_link_begin(g_now));
#endif
    g_snapshot_result = S5_SNAPSHOT_BUSY;
    simulate();
    assert((unsigned)S3_DRIVE_FOLLOW == g_drive.state);
    /* Reusing a cache must never extend the source reading's lifetime. */
    init(true);
    start();
    g_snapshot_result = S5_SNAPSHOT_BUSY;

    for (unsigned tick_index = 0; tick_index < CONTENTION_CHECK_TICKS;
         tick_index++)
    {
        simulate();
        assert((unsigned)S3_DRIVE_FOLLOW == g_drive.state);
    }

    simulate();
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state &&
           S5_BAD_RANGE == (int)g_drive.obstacle.fault);
    init(true);
    start();
    g_sample_age_us = 240000U;
    simulate();
    g_snapshot_result = S5_SNAPSHOT_BUSY;
    simulate();
    assert((unsigned)S3_DRIVE_FOLLOW == g_drive.state);
    simulate();
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state);
    /* Explicit negative observations and read/codec failures never use old good
     * data. */
    init(true);
    start();
    g_snapshot_result = S5_SNAPSHOT_BUSY;
    simulate();
    g_snapshot_result = S5_SNAPSHOT_OK;
    gb_is_range_bad   = true;
    simulate();
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state);
    init(true);
    start();
    g_snapshot_result = S5_SNAPSHOT_INVALID;
    simulate();
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state);
    init(true);
    start();
    g_snapshot_result = S5_SNAPSHOT_UNAVAILABLE;
    simulate();
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state);
    init(true);
    start();
    g_drive.b_has_obstacle_snapshot = false;
    g_snapshot_result               = S5_SNAPSHOT_BUSY;
    simulate();
    assert((unsigned)S3_DRIVE_FAULT == g_drive.state);
    /* Active avoidance gets the same grace, without dropping a DONE event. */
    init(true);
    start();
    g_obstacle.range_mm = 120;
    simulate();

    for (unsigned tick_index = 0; (tick_index < BYPASS_START_WAIT_TICKS) &&
                                  (!((g_drive.obstacle.b_has_issued) &&
                                     (0 == (int)g_drive.obstacle.leg)));
         tick_index++)
    {
        simulate();
    }

    assert(g_drive.obstacle.b_has_issued);
    g_snapshot_result = S5_SNAPSHOT_BUSY;
    simulate();
    assert(g_drive.b_is_bypassing &&
           (unsigned)S3_DRIVE_MANOEUVRE == g_drive.state);
    s3_drive_stop(&g_drive);
    assert(!g_drive.b_is_bypassing && !g_drive.b_has_obstacle_snapshot &&
           !gb_has_lease);
    init(true);
    start();
    g_obstacle.range_mm = 120;
    simulate();

    for (unsigned tick_index = 0; (tick_index < BYPASS_START_WAIT_TICKS) &&
                                  (!((g_drive.obstacle.b_has_issued) &&
                                     (0 == (int)g_drive.obstacle.leg)));
         tick_index++)
    {
        simulate();
    }

    g_snapshot_result = S5_SNAPSHOT_BUSY;

    for (unsigned tick_index = 0; (tick_index < FAULT_DETECTION_WAIT_TICKS) &&
                                  (g_drive.state != (unsigned)S3_DRIVE_FAULT);
         tick_index++)
    {
        simulate();
    }

    assert((unsigned)S3_DRIVE_FAULT == g_drive.state &&
           !g_drive.b_is_bypassing);
    puts("PASS: transient BUSY read preserves fresh data; stale, missing, "
         "invalid and unavailable data still fault; active avoidance and STOP "
         "covered");
}
#if APP_S4
/** @brief Inject the supplied terrain event during an active bypass and
 * assert that the drive stops issuing wheel or turn commands.
 */
static void stop_active_bypass (terr_data_result_t event)
{
    init(true);
    start();
    g_obstacle.range_mm = 120;
    simulate();

    for (unsigned tick_index = 0; (tick_index < BYPASS_START_WAIT_TICKS) &&
                                  (!((g_drive.obstacle.b_has_issued) &&
                                     (0 == (int)g_drive.obstacle.leg)));
         tick_index++)
    {
        simulate();
    }

    assert(g_drive.b_is_bypassing && g_drive.obstacle.b_has_issued);

    for (unsigned tick_index = 0;
         ((tick_index < WHEEL_RESPONSE_WAIT_TICKS) && (0.0f == g_duty[0])) &&
         (0.0f == g_duty[1]);
         tick_index++)
    {
        simulate();
    }

    assert(g_duty[0] != 0.0f || g_duty[1] != 0.0f);

    unsigned previous_stops      = g_stops;
    unsigned previous_velocities = g_velocities;
    unsigned previous_turns      = g_turns;
    g_imu_event                  = event;
    simulate();
    assert(g_stops > previous_stops &&
           (unsigned)S3_DRIVE_FAULT == g_drive.state &&
           (unsigned)S3_DRIVE_TERRAIN_FAULT == g_drive.reason);
    assert(!g_drive.b_is_bypassing &&
           S5_INTERRUPTED == (int)g_drive.obstacle.fault && !gb_has_lease);
    simulate();
    assert(0.0f == g_duty[0] && 0.0f == g_duty[1]);

    for (unsigned tick_index = 0;
         (tick_index < STOP_CONFIRM_WAIT_TICKS) && (!ctrl_motion_is_stopped());
         tick_index++)
    {
        simulate();
        assert(0.0f == g_duty[0] && 0.0f == g_duty[1]);
    }

    assert(ctrl_motion_is_stopped() && g_velocities == previous_velocities &&
           g_turns == previous_turns);
}
#endif
/* End of file. */

