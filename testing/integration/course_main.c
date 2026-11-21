/** @file course_main.c
 * @brief Unit checks for the course profile: the "tape between the sensors"
 * line detector mode and the s3_drive course options.
 *
 * Behaviour on a simulated
 * track (curves, crossings, turns) is covered by course_sim_main.c.
 * Unsigned casts spell out the original modulo conversion.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "course_profile.h"
#include "../../app_program/app_types.h"
#include <assert.h>
#include <stdio.h>
#include "course_main.h"

#define REFLECTANCE_CAL_SAMPLE_COUNT 10
#define STRAIGHT_READING_TICKS 20
#define CENTRED_WHITE_TICKS 500U
#define TAPE_WARMUP_TICKS 5U
#define LOSS_OBSERVATION_TICKS 15U

#define CODE39_SYMBOL_ELEMENTS 9

/* The shared fixture also builds against the original course-options header. */
#ifndef COURSE_DRIVE_MEMBER
#define COURSE_DRIVE_MEMBER p_drive
#endif

#if S3_STRAIGHT_TEST
static float32_t g_requested_left;
static float32_t g_requested_right;
static unsigned  g_requested_stops;

/* Exercise production navigation/drive with valid synthetic feedback, not a
 * physics simulation. Extreme IR contrast must not request differential speed.
 */
static s3_drive_t g_straight_drive;
static uint32_t   g_straight_now;

#endif

static line_detection_observation_t step (line_detection_t * p_data,
                                          uint32_t * p_time, uint16_t left,
                                          uint16_t right);

static motion_result_t fake_drive (float32_t distance_m, float32_t speed_mps);

static motion_result_t fake_velocity (float32_t left, float32_t right);

static motion_result_t fake_manoeuvre (motion_manoeuvre_t manoeuvre);

static void fake_stop (void);

static bool fake_report (motion_report_t * p_report);

static uint8_t fake_id (void);

static bool fake_reset (void);

static void fake_heartbeat (bool b_is_on);

#if S3_STRAIGHT_TEST
static void test_straight_requests (void);
#endif

/* Referenced by the course profile when S3_SENSOR_TO_AXLE_MM > 0. */
/** @brief Accept the simulated motion command immediately without starting a
 * motor.
 */
motion_result_t motion_request_drive (float32_t distance_m, float32_t speed_mps)
{
    (void)distance_m;
    (void)speed_mps;
    return MOTION_OK;
}

/** @brief Check the between-sensor tape profile and its steering decisions
 * against the selected course options.
 */
int main (void)
{
    line_detection_config_t      cfg = line_detection_defaults();
    line_detection_t             det;
    line_detection_observation_t observation;
    uint32_t                     start_us = 0U;

    /* Defaults are Buddy 3's original design. */
    assert(!cfg.b_is_tape_between);

    /* Between mode: both white is the normal centred state, forever. */
    cfg.b_is_tape_between = true;
    assert(line_detection_init(&det, &cfg));

    for (unsigned tick_index = 0U; tick_index < CENTRED_WHITE_TICKS;
         tick_index++) /* Feed five seconds of white. */
    {
        observation = step(&det, &start_us, 0U, 0U);
        assert(observation.b_is_valid && observation.b_is_line_present &&
               !observation.b_has_lost_event && 0 == observation.error &&
               !observation.b_is_touching_tape &&
               !observation.b_has_junction_candidate);
    }
    /* One sensor on the tape: steer towards it (sign as in Buddy 3). */
    observation = step(&det, &start_us, 1000U, 0U);
    assert(observation.b_is_line_present && observation.b_is_touching_tape &&
           observation.error == -100);
    observation = step(&det, &start_us, 0U, 1000U);
    assert(observation.b_is_line_present && observation.b_is_touching_tape &&
           100 == observation.error);
    observation = step(&det, &start_us, 150U,
                       0U); /* Set the reading below the tape touch level. */
    assert(!observation.b_is_touching_tape && observation.error == -15);
    /* Both dark for 30 ms: a crossing (unchanged rule). */
    observation = step(&det, &start_us, 900U, 900U);
    assert(observation.b_has_junction_candidate &&
           !observation.b_has_junction_event);
    observation = step(&det, &start_us, 900U, 900U);
    observation = step(&det, &start_us, 900U, 900U);
    observation = step(&det, &start_us, 900U, 900U);
    assert(observation.b_has_junction_event);
    /* Invalid input still invalidates (sensor faults are not hidden). */
    start_us   += 10000U;
    observation = line_detection_update(&det, start_us, false, 0U, 0U);
    assert(!observation.b_is_valid && !observation.b_is_line_present);
    /* The touching level must be a per-sensor value in between mode. */
    cfg.signal_floor = 1500U;
    assert(!line_detection_init(&det, &cfg));

    /* Original mode is untouched: both white -> lost after 100 ms. */
    cfg = line_detection_defaults();
    assert(line_detection_init(&det, &cfg));
    start_us = 0U;

    for (unsigned tick_index = 0U; tick_index < TAPE_WARMUP_TICKS; tick_index++)
    {
        (void)step(&det, &start_us, 500U, 500U);
    }

    {
        bool b_has_lost_line = false;

        for (unsigned tick_index = 0U; tick_index < LOSS_OBSERVATION_TICKS;
             tick_index++)
        {
            observation     = step(&det, &start_us, 0U, 0U);
            b_has_lost_line = b_has_lost_line || (observation.b_has_lost_event);
        }

        assert(b_has_lost_line && !observation.b_is_line_present);
    }

    /* Drive options: defaults reproduce the original controller. */
    {
        const s3_drive_options_t def = s3_drive_default_options();
        assert(!def.b_allows_cross_straight && 0.0f == def.turn_advance_m &&
               5000000U == def.pending_wait_us &&
               0U == def.turn_line_window_us && 0.0f == def.search_deg &&
               0 == def.COURSE_DRIVE_MEMBER);
    }
    /* Course profile values. */
    {
        const s3_drive_options_t drive_options = course_profile_options();
        line_navigation_config_t nav           = line_navigation_defaults();
        course_profile_navigation(&nav);
        assert(nav.line.b_is_tape_between ==
               (S3_STRAIGHT_TEST || S3_LINE_BETWEEN != 0));
        assert(nav.steering.kp_mps ==
               (S3_STRAIGHT_TEST ? 0.0f : S3_COURSE_STEERING_KP));
        /* Barcode cut-offs: barcode sensor only, line sensors unchanged. */
        assert(S3_BARCODE_WHITE_LIMIT ==
                   nav.calibration[S3_SENSOR_BARCODE].white_limit &&
               S3_BARCODE_BLACK_LIMIT ==
                   nav.calibration[S3_SENSOR_BARCODE].black_limit);
        assert(400U == nav.calibration[S3_SENSOR_LEFT].white_limit &&
               600U == nav.calibration[S3_SENSOR_LEFT].black_limit &&
               400U == nav.calibration[S3_SENSOR_RIGHT].white_limit &&
               600U == nav.calibration[S3_SENSOR_RIGHT].black_limit);
        {
            /* These ranges are valid for the reflectance model. */
            ir_reflectance_t chk;
            assert(
                ir_reflectance_init(&chk, &nav.calibration[S3_SENSOR_BARCODE]));
        }
        assert(S3_COURSE_BASE_MPS == nav.steering.base_mps);
        assert(line_controller_config_valid(&nav.steering));
        {
            /* Full error stops the inner wheel and adds gain outside. */
            const line_detection_observation_t full = {
                true, true, false, -100, false, false, false, true};
            const line_controller_request_t request =
                line_controller_compute(&nav.steering, &full);
#if S3_STRAIGHT_TEST
            assert(request.b_is_valid &&
                   S3_COURSE_BASE_MPS == request.left_mps &&
                   S3_COURSE_BASE_MPS == request.right_mps);
#else
            assert(request.b_is_valid && 0.0f == request.left_mps &&
                   request.right_mps >
                       S3_COURSE_BASE_MPS + S3_COURSE_STEERING_KP - 0.001f);
#endif
        }
        assert(drive_options.b_allows_cross_straight &&
               S3_COURSE_PENDING_WAIT_US == drive_options.pending_wait_us &&
               300000U == drive_options.turn_line_window_us);
        /* After-turn search: on, below 45 deg (cannot reach the next arm). */
        assert(S3_TURN_SEARCH_DEG == drive_options.search_deg &&
               drive_options.search_deg > 0.0f &&
               drive_options.search_deg < 45.0f &&
               S3_TURN_SEARCH_MPS == drive_options.search_mps &&
               S3_TRACK_M == drive_options.track_m);
#if S3_SENSOR_TO_AXLE_MM == 0
        assert(0.0f == drive_options.turn_advance_m &&
               0 == drive_options.COURSE_DRIVE_MEMBER);
#else
        assert(drive_options.COURSE_DRIVE_MEMBER == motion_request_drive &&
               drive_options.advance_mps > 0.0f &&
               drive_options.turn_advance_m >
                   0.10f * (float32_t)S3_SENSOR_TO_AXLE_MM / 100.0f);
#endif
    }
    /* set_options validation. */
    {
        s3_drive_t               drive;
        line_navigation_config_t nav = line_navigation_defaults();

        const s3_drive_port_t port = {fake_velocity, fake_manoeuvre, fake_stop,
                                      fake_report,   fake_id,        fake_reset,
                                      fake_heartbeat};
        s3_drive_options_t    test_options = course_profile_options();
        nav.acquisition.enabled_mask       = S3_ALL_SENSORS;
        assert(s3_drive_init(&drive, &nav, &port));
        assert(
            !drive.options.b_allows_cross_straight); /* Initialization installs
                                                        the defaults. */
        test_options.turn_advance_m      = 0.065f;
        test_options.COURSE_DRIVE_MEMBER = 0;
        assert(!s3_drive_set_options(
            &drive, &test_options)); /* Require a drive callback for advance. */
        assert(!drive.options.b_allows_cross_straight); /* Rejection leaves the
                                                           options intact. */
        test_options.COURSE_DRIVE_MEMBER = fake_drive;
        assert(s3_drive_set_options(&drive, &test_options));
        assert(drive.options.b_allows_cross_straight &&
               0.065f == drive.options.turn_advance_m);
        test_options.turn_advance_m =
            0.5f; /* Set a distance beyond the allowed range. */
        assert(!s3_drive_set_options(&drive, &test_options));
        test_options.turn_advance_m = 0.065f;
        test_options.search_deg = 70.0f; /* Set a sweep wider than allowed. */
        assert(!s3_drive_set_options(&drive, &test_options));
        test_options.search_deg = 35.0f;
        test_options.search_mps = 0.0f; /* Require a nonzero sweep speed. */
        assert(!s3_drive_set_options(&drive, &test_options));
        test_options.search_mps = 0.05f;
        test_options.turn_line_window_us =
            0U; /* Search requires the post-turn line check. */
        assert(!s3_drive_set_options(&drive, &test_options));
        test_options.turn_line_window_us = 300000U;
        assert(s3_drive_set_options(&drive, &test_options));
        test_options.turn_advance_m  = 0.065f;
        test_options.pending_wait_us = 0U;
        assert(!s3_drive_set_options(&drive, &test_options));
        /* A halt keeps this cycle's barcode event so telemetry still
         * reports the letter that caused it (e.g. an unmapped one). */
        drive.output.events =
            (unsigned)S3_EVENT_BARCODE | (unsigned)S3_EVENT_JUNCTION;
        s3_drive_stop(&drive);
        assert((unsigned)S3_EVENT_BARCODE == drive.output.events);
        drive.state = S3_DRIVE_FOLLOW; /* Reject this change during a run. */
        test_options.pending_wait_us = 20000000U;
        assert(!s3_drive_set_options(&drive, &test_options));
    }
#if S3_STRAIGHT_TEST
    test_straight_requests();
    puts("PASS: straight-test equal wheel requests for white/extreme "
         "contrast/junctions; STOP, unmapped barcode and sensor/terrain "
         "faults preserved; mapped barcode commands retained");
#else
    puts("PASS: between-sensors detector (centred white, steering sign, "
         "touching, junction, no line lost), original mode unchanged, "
         "course options and validation");
#endif
    return 0;
}

/** @brief Advance the supplied clock by 10 ms and return the detector's
 * update for valid left and right observations.
 */
static line_detection_observation_t step (line_detection_t * p_data,
                                          uint32_t * p_time, uint16_t left,
                                          uint16_t right)
{
    *p_time += 10000U;
    return line_detection_update(p_data, *p_time, true, left, right);
}

/** @brief Accept the simulated motion command immediately without starting a
 * motor.
 */
static motion_result_t fake_drive (float32_t distance_m, float32_t speed_mps)
{
    (void)distance_m;
    (void)speed_mps;
    return MOTION_OK;
}

/** @brief Accept simulated wheel velocities. In the straight-test profile,
 * retain both commands for inspection.
 */
static motion_result_t fake_velocity (float32_t left, float32_t right)
{
#if S3_STRAIGHT_TEST
    g_requested_left  = left;
    g_requested_right = right;
#else
    (void)left;
    (void)right;
#endif
    return MOTION_OK;
}

/** @brief Accept the simulated motion command immediately without starting a
 * motor.
 */
static motion_result_t fake_manoeuvre (motion_manoeuvre_t manoeuvre)
{
    (void)manoeuvre;
    return MOTION_OK;
}

/** @brief In the straight-test profile, zero the recorded wheel requests and
 * count the stop.
 */
static void fake_stop (void)
{
#if S3_STRAIGHT_TEST
    g_requested_left = g_requested_right = 0;
    g_requested_stops++;
#endif
}

/** @brief Report an empty motion queue and leave the output untouched.
 */
static bool fake_report (motion_report_t * p_report)
{
    (void)p_report;
    return false;
}

/** @brief Return the fixed command ID used in simulated motion feedback.
 */
static uint8_t fake_id (void)
{
    return 1U;
}

/** @brief Accept the reset immediately without changing simulated motion
 * state.
 */
static bool fake_reset (void)
{
    return true;
}

/** @brief Accept the navigation heartbeat without simulating a watchdog.
 */
static void fake_heartbeat (bool b_is_on)
{
    (void)b_is_on;
}

#if S3_STRAIGHT_TEST
/** @brief Advance the simulated drive by 10 ms with the supplied raw sensor
 * values and motion feedback.
 */
static void straight_tick (uint16_t barcode, uint16_t left, uint16_t right,
                           bool b_is_valid)
{
    ir_acquisition_frame_t frame    = {0};
    motion_feedback_t      feedback = {0};
    /* clang-format off */
    g_straight_now                 += 10000U;
    frame.timestamp_us              = g_straight_now;
    frame.sensor[S3_SENSOR_BARCODE] =
        (ir_acquisition_sample_t){g_straight_now, barcode, 0, true, b_is_valid};
    frame.sensor[S3_SENSOR_LEFT]    =
        (ir_acquisition_sample_t){g_straight_now, left, 0, true, b_is_valid};
    frame.sensor[S3_SENSOR_RIGHT]   =
        (ir_acquisition_sample_t){g_straight_now, right, 0, true, b_is_valid};
    feedback.timestamp_us           = g_straight_now;
    feedback.epoch                  = 1;
    feedback.b_is_feedback_valid    = true;
    feedback.b_is_braked            = true;
    feedback.state                  = MOTION_IDLE;
    feedback.command_id             = 1;
    /* clang-format on */
    assert(s3_drive_process(&g_straight_drive, &frame, &feedback));
}

/** @brief Supply the three constant raw readings as valid samples for the
 * requested number of ticks.
 */
static void straight_hold (unsigned tick_count, uint16_t barcode, uint16_t left,
                           uint16_t right)
{
    for (unsigned tick_index = 0; tick_index < tick_count; tick_index++)
    {
        straight_tick(barcode, left, right, true);
    }
}

/** @brief Configure, calibrate and start the straight-test fixture after
 * feeding its initial idle readings.
 */
static void straight_start (bool b_should_map_a)
{
    line_navigation_config_t   cfg = line_navigation_defaults();
    ir_calibration_collector_t cal;

    const s3_drive_port_t    port = {fake_velocity, fake_manoeuvre, fake_stop,
                                     fake_report,   fake_id,        fake_reset,
                                     fake_heartbeat};
    const s3_drive_options_t options = course_profile_options();
    course_profile_navigation(&cfg);
    /* clang-format off */
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    cfg.map[0]                   = b_should_map_a ? S3_NAV_LEFT : S3_NAV_NONE;
    g_straight_now               = 0;
    g_requested_left             = g_requested_right = 0;
    g_requested_stops            = 0;
    /* clang-format on */
    assert(s3_drive_init(&g_straight_drive, &cfg, &port));
    assert(s3_drive_set_options(&g_straight_drive, &options));
    ir_calibration_begin(&cal);

    for (unsigned sample_index = 0;
         sample_index < (unsigned)REFLECTANCE_CAL_SAMPLE_COUNT; sample_index++)
    {
        assert(ir_calibration_add(&cal, IR_SURFACE_WHITE, 150));
        assert(ir_calibration_add(&cal, IR_SURFACE_BLACK, 280));
    }

    for (unsigned sensor_index = 0; sensor_index < (unsigned)S3_SENSOR_COUNT;
         sensor_index++)
    {
        assert(line_navigation_set_calibration(
            &g_straight_drive.navigation,
            (ir_acquisition_sensor_id_t)sensor_index, &cal));
    }

    straight_hold(12, 150, 150, 150);
    assert(s3_drive_start(&g_straight_drive, g_straight_now));
}

/** @brief Feed A between two stars, including the inter-character gaps and
 * the leading and trailing white samples.
 */
static void straight_scan_a (void)
{
    const unsigned symbols[3] = {0x094U, 0x109U, 0x094U};
    straight_hold(40, 150, 150, 150);

    for (unsigned symbol_index = 0;
         symbol_index < (sizeof symbols / sizeof symbols[0]); symbol_index++)
    {
        for (unsigned element_index = 0;
             element_index < (unsigned)CODE39_SYMBOL_ELEMENTS; element_index++)
        {
            unsigned ticks =
                (symbols[symbol_index] & (1U << (8U - element_index))) ? 12 : 4;
            straight_hold(ticks, element_index % 2U ? 150 : 280, 150, 150);
        }

        if (symbol_index < 2U)
        {
            straight_hold(4, 150, 150, 150);
        }
    }

    straight_hold(45, 150, 150, 150);
}

/** @brief Check straight-profile wheel requests over successive line
 * readings and confirm braking after the stop condition.
 */
static void test_straight_requests (void)
{
    const uint16_t readings[][2] = {
        {150, 150}, {280, 150}, {150, 280}, {280, 280}, {150, 150}};
    straight_start(false);

    for (unsigned reading_index = 0;
         reading_index < (sizeof(readings) / sizeof(readings[0]));
         reading_index++)
    {
        for (unsigned tick_index = 0;
             tick_index < (unsigned)STRAIGHT_READING_TICKS; tick_index++)
        {
            straight_tick(150, readings[reading_index][0],
                          readings[reading_index][1], true);
            assert((unsigned)S3_DRIVE_FOLLOW == g_straight_drive.state);
            assert(S3_COURSE_BASE_MPS == g_requested_left &&
                   S3_COURSE_BASE_MPS == g_requested_right);
        }
    }
    /* An explicit STOP, an unmapped barcode and invalid sensors still stop. */
    s3_drive_stop(&g_straight_drive);
    assert((unsigned)S3_DRIVE_OFF == g_straight_drive.state &&
           g_requested_stops > 0U && 0 == g_requested_left &&
           0 == g_requested_right);
    straight_start(false);
    straight_scan_a();
    assert((unsigned)S3_DRIVE_HALTED == g_straight_drive.state &&
           (unsigned)S3_DRIVE_UNMAPPED == g_straight_drive.reason);
    assert(0 == g_requested_left && 0 == g_requested_right);
    straight_start(false);
    straight_tick(150, 150, 150, false);
    assert((unsigned)S3_DRIVE_FAULT == g_straight_drive.state &&
           (unsigned)S3_DRIVE_SENSOR_FAULT == g_straight_drive.reason);
    assert(0 == g_requested_left && 0 == g_requested_right);
    straight_start(false);
    straight_tick(150, 150, 150, true);
    s3_drive_terrain_fault(&g_straight_drive);
    assert((unsigned)S3_DRIVE_FAULT == g_straight_drive.state &&
           (unsigned)S3_DRIVE_TERRAIN_FAULT == g_straight_drive.reason);
    assert(0 == g_requested_left && 0 == g_requested_right);
    /* This profile does not erase mapped barcode commands. */
    straight_start(true);
    straight_scan_a();
    assert((unsigned)S3_DRIVE_FOLLOW == g_straight_drive.state &&
           (unsigned)S3_NAV_LEFT == g_straight_drive.pending);
    straight_hold(4, 150, 280, 280);
    assert((unsigned)S3_DRIVE_BRAKING == g_straight_drive.state &&
           0 == g_requested_left && 0 == g_requested_right);
}
#endif

/* End of file. */

