/** @file course_sim_main.c
 * @brief Course simulator for the "tape between the sensors" line-following
 * profile.
 *
 * Runs the REAL code that goes on the car: Buddy 3 navigation (calibration,
 * reflectance, line detection, steering), the s3_drive controller with the
 * course profile (course_profile.h), and Buddy 2's motion controller, PID and
 * estimator. Only the physical world is modelled here:
 *   - two driven wheels (first-order speed lag, static friction / deadband),
 *     quadrature counts from wheel travel (2534 counts per 65.4 mm wheel turn);
 *   - IR sensors as small round spots over 18 mm black tape on white, using
 *     the built-in calibration's raw white/black levels.
 * The physical constants marked GUESS are not measured; results show whether
 * the LOGIC works for a plausible car, not that the real car will.
 *
 * Each scenario prints one line; the program fails if any expectation fails.
 * Unsigned casts spell out the original modulo conversion.
 */
#include "ctrl_motion.h"
#include "../../app_program/app_types.h"
#include "motion_estimation.h"
#include "motion_lease.h"
#include "course_sim_main.h"

#include "course_profile.h"
#include "builtin_calibration.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIM_TICKS_PER_SECOND 100

#define PHYSICS_SUBSTEP_COUNT 10
#define SIM_CALIBRATION_SAMPLE_COUNT 12U
#define SIM_START_SETTLE_TICKS 20
#define CURVE_BEFORE_STOP_TICKS 300

/* ---- Car (measured where stated) --------------------------------------- */
#define TRACK_M 0.116     /* Use the Buddy 2 track width. */
#define WHEEL_D_M 0.0654  /* Use the Buddy 2 wheel diameter. */
#define COUNTS_REV 2534.0 /* Use the 21 October calibration. */
#define VMAX_MPS 0.884    /* Use the full-duty calibration. */
#define TAU_S 0.06        /* The wheel speed time constant is estimated. */
#define BREAKAWAY 0.08    /* Estimate the duty needed to start a wheel. */
#define KEEP_MOVING 0.03  /* Estimate the duty below which a wheel stops. */
#define TAPE_W 0.018      /* The course specifies 18 mm tape. */
#define SPOT_R 0.0025     /* The sensing spot radius is estimated. */
#define MIN_WHEEL 0.06f   /* Use the integration speed floor in m/s. */

/* ---- Tape: straight segments and arcs ----------------------------------- */
typedef struct
{
    int       arc;
    float64_t start_x;
    float64_t start_y;
    float64_t end_x;
    float64_t end_y;
    float64_t centre_x;
    float64_t centre_y;
    float64_t radius_m;
    float64_t start_rad;
    float64_t end_rad;
} piece_t;

#define MAX_PIECES 16
static piece_t   g_track[MAX_PIECES];
static int       g_pieces;
static float64_t g_tape_end_x =
    1e9; /* Remove tape beyond this x in scenario 9. */

/* ---- Wheels and encoders (hardware boundary of Buddy 2) ----------------- */
static float32_t g_duty[2];
static float64_t g_wheel_v[2];
static float64_t g_wheel_pos[2];
static int32_t   g_counts[2];

/* ---- Car pose and Buddy 2 boundary --------------------------------------- */
static float64_t g_axle_x;
static float64_t g_axle_y;
static float64_t g_axle_heading; /* Use axle centre and heading in radians. */
/* Use the measured axle offset from course_profile.h. */
static float64_t g_sensor_ahead =
    (S3_SENSOR_TO_AXLE_MM > 0) ? S3_SENSOR_TO_AXLE_MM / 1000.0 : 0.060;
static float64_t g_sensor_gap = 0.022; /* Measure between sensor centres. */
static float32_t g_steering_kp =
    -1.0f; /* A negative third argument uses the firmware gain. */
static uint32_t        g_now;
static uint32_t        g_lease_stamp;
static bool            gb_has_lease;
static motion_report_t g_reports[MOTION_REPORT_QUEUE_LEN];
static unsigned        g_report_count;
static unsigned        g_turns_done;
static unsigned        g_advances;
static s3_drive_t      g_drive;

static float64_t g_turn_scale =
    1.0; /* Model overshoot or undershoot at turn end. */

static int g_failures;

#ifndef COURSE_BYPASS_TEST
static float64_t g_goal_x;
static float64_t g_goal_y;
#endif

static float64_t       angle_in (float64_t angle_rad, float64_t start_rad,
                                 float64_t end_rad);
static float64_t       coverage (float64_t x_coord, float64_t y_coord);
static void            post (const motion_report_t * p_report);
static bool            receive (motion_report_t * p_report);
static float32_t       floor_wheel (float32_t speed_mps);
static motion_result_t velocity (float32_t left, float32_t right);
static motion_result_t manoeuvre (motion_manoeuvre_t manoeuvre_kind);
static motion_result_t drive_m (float32_t metres, float32_t speed);
static void            brake (void);
static bool            reset (void);
static void            heartbeat (bool b_is_on);
static void            physics_10ms (void);
static uint16_t        raw (int which);
static void            calibrate (void);

#ifndef COURSE_BYPASS_TEST
static float64_t run (float64_t seconds, bool ( * p_until)(void));
static bool      past_x (void);
static bool      below_y (void);
static bool      above_y (void);
static bool      left_of_x (void);
static void      scenario_straight (float64_t offset);
static void      scenario_curve (float64_t bend_radius_m, int dir,
                                 bool b_should_succeed);
static void      scenario_s_bend (float64_t bend_radius_m);
static void      cross_track (void);
static void      scenario_cross_straight (line_navigation_action_t pending);
static void scenario_turn (line_navigation_action_t action, float64_t advance_m,
                           bool b_should_succeed);
static void scenario_turn_error (line_navigation_action_t action,
                                 float64_t advance_m, float64_t scale);
static void scenario_tape_gone (void);
#endif

/** @brief Append a straight piece of tape in the single-threaded fixture.
 * Endpoints are in metres and the caller must leave room in the track array. */
void course_sim_add_line (float64_t start_x, float64_t start_y, float64_t end_x,
                          float64_t end_y)
{
    g_track[g_pieces++] =
        (piece_t){0, start_x, start_y, end_x, end_y, 0, 0, 0, 0, 0};
}

/** @brief Append a circular tape segment. The centre and radius are in metres
 * and the endpoint angles are in radians. The caller must leave room in the
 * track array. */
void course_sim_add_arc (float64_t centre_x, float64_t centre_y,
                         float64_t radius_m, float64_t start_rad,
                         float64_t end_rad)
{
    g_track[g_pieces++] = (piece_t){
        1, 0, 0, 0, 0, centre_x, centre_y, radius_m, start_rad, end_rad};
}

/** @brief Return the minimum centreline distance in metres for the supplied
 * course coordinates using the current simulated track. */
float64_t course_sim_dist_to_tape (float64_t x_coord, float64_t y_coord)
{
    float64_t best = 1e9;

    if (x_coord > g_tape_end_x)
    {
        return best;
    }

    for (int piece_index = 0; piece_index < g_pieces; piece_index++)
    {
        const piece_t * p_piece = &g_track[piece_index];
        float64_t       distance_m;

        if (!p_piece->arc)
        {
            float64_t segment_x_m = p_piece->end_x - p_piece->start_x;
            float64_t segment_y_m = p_piece->end_y - p_piece->start_y;
            float64_t projection =
                (((x_coord - p_piece->start_x) * segment_x_m) +
                 ((y_coord - p_piece->start_y) * segment_y_m)) /
                ((segment_x_m * segment_x_m) + (segment_y_m * segment_y_m));
            projection = projection < 0 ? 0 : (projection > 1 ? 1 : projection);
            distance_m = hypot(
                x_coord - (p_piece->start_x + (projection * segment_x_m)),
                y_coord - (p_piece->start_y + (projection * segment_y_m)));
        }
        else
        {
            float64_t angle_rad =
                atan2(y_coord - p_piece->centre_y, x_coord - p_piece->centre_x);

            if (angle_in(angle_rad, p_piece->start_rad, p_piece->end_rad))
            {
                distance_m = fabs(hypot(x_coord - p_piece->centre_x,
                                        y_coord - p_piece->centre_y) -
                                  p_piece->radius_m);
            }
            else
            {
                float64_t start_gap = hypot(
                    x_coord - (p_piece->centre_x +
                               (p_piece->radius_m * cos(p_piece->start_rad))),
                    y_coord - (p_piece->centre_y +
                               (p_piece->radius_m * sin(p_piece->start_rad))));
                float64_t end_gap = hypot(
                    x_coord - (p_piece->centre_x +
                               (p_piece->radius_m * cos(p_piece->end_rad))),
                    y_coord - (p_piece->centre_y +
                               (p_piece->radius_m * sin(p_piece->end_rad))));
                distance_m = start_gap < end_gap ? start_gap : end_gap;
            }
        }

        if (distance_m < best)
        {
            best = distance_m;
        }
    }

    return best;
}

/** @brief Return the signed simulated encoder count for the selected wheel.
 */
int32_t wheel_encoder_count (wheel_encoder_id_t wheel)
{
    return g_counts[wheel];
}

/** @brief Return the selected wheel count reduced to the encoder's unsigned
 * bit width.
 */
uint32_t wheel_encoder_bits (wheel_encoder_id_t wheel)
{
    return (uint32_t)g_counts[wheel] & WHEEL_COUNT_MASK;
}

/** @brief Return the signed difference between two masked encoder readings,
 * allowing wrap at the encoder width.
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

/** @brief Save the caller's duty command for the selected simulated wheel.
 * The next physics step consumes it.
 */
void motor_control_set (wheel_encoder_id_t wheel, float32_t value)
{
    g_duty[wheel] = value;
}

/** @brief Set both simulated duty commands to zero.
 */
void motor_control_brake_all (void)
{
    g_duty[0] = g_duty[1] = 0.0f;
}

/** @brief Read the simulated wheel clock in microseconds modulo 2^32. */
uint32_t course_sim_time_us (void)
{
    return g_now;
}

/** @brief Submit a turn at the supplied angle in degrees and speed in metres
 * per second. Scale the achieved angle with the fixture setting and return the
 * controller result. */
motion_result_t course_sim_turn_deg (
    float32_t deg, float32_t speed) /* Follow the wheel mapping in motion.c. */
{
    deg           = (float32_t)(deg * g_turn_scale);
    float32_t arc = fabsf(deg) / 360.0f * (float32_t)M_PI * (float32_t)TRACK_M;
    int32_t   left_sign = deg > 0.0f ? -1 : 1;
    g_turns_done++;
    return ctrl_motion_request_move(MOTION_TURN, left_sign, -left_sign, arc,
                                    speed, g_now);
}

/** @brief Forward the course profile's drive request to the same simulated
 * distance driver installed for Buddy 3.
 */
motion_result_t motion_request_drive (float32_t distance_m, float32_t speed)
{
    return drive_m(distance_m, speed);
}

/** @brief Write the selected optical head position in metres to two
 * caller-owned outputs. Index zero selects the left sensor and one selects the
 * right. */
void course_sim_sensor_xy (
    int which, float64_t * p_x_coord,
    float64_t * p_y_coord) /* Use zero for left and one for right. */
{
    float64_t forward_x_m = g_axle_x + (g_sensor_ahead * cos(g_axle_heading));
    float64_t forward_y_m = g_axle_y + (g_sensor_ahead * sin(g_axle_heading));
    float64_t side        = (0 == which ? 1 : -1) * g_sensor_gap / 2;
    *p_x_coord            = forward_x_m - (side * sin(g_axle_heading));
    *p_y_coord            = forward_y_m + (side * cos(g_axle_heading));
}

/** @brief Advance the simulated plant and clock by 10 ms. Update the controller
 * and feed its observations back to navigation in the same tick. */
void course_sim_tick (void)
{
    motion_feedback_t      feedback;
    ir_acquisition_frame_t frame = {0};
    physics_10ms();
    g_now += 10000U;
    motion_estimation_update(0.01f);

    if (motion_lease_expired(gb_has_lease, g_now, g_lease_stamp))
    {
        ctrl_motion_nav_timeout();
        gb_has_lease = false;
    }

    ctrl_motion_step(0.01f, g_now);
    /* clang-format off */
    feedback                        =
        (motion_feedback_t){motion_estimation_robot_dist(),
                                             g_now,
                                             1U,
                                             ctrl_motion_state(),
                                             ctrl_motion_fault(),
                                             true,
                                             ctrl_motion_is_stopped(),
                                             ctrl_motion_command_id(),
                                             0U};
    frame.timestamp_us              = g_now;
    frame.sensor[S3_SENSOR_BARCODE] = (ir_acquisition_sample_t){
        g_now,
        g_drive.navigation.config.calibration[S3_SENSOR_BARCODE].white_raw, 0,
        true, true};
    frame.sensor[S3_SENSOR_LEFT]    =
        (ir_acquisition_sample_t){g_now, raw(0), 0, true, true};
    frame.sensor[S3_SENSOR_RIGHT]   =
        (ir_acquisition_sample_t){g_now, raw(1), 0, true, true};
    /* clang-format on */
    (void)s3_drive_process(&g_drive, &frame, &feedback);
}

/** @brief Reset the pose and controllers for a scenario using metres and
 * radians. Run calibration and start the drive. End the fixture on rejected
 * setup or start. */
void course_sim_setup (float64_t x_coord, float64_t y_coord, float64_t heading,
                       float64_t advance_m)
{
    line_navigation_config_t cfg  = line_navigation_defaults();
    const s3_drive_port_t    port = {
        velocity, manoeuvre, brake, receive, ctrl_motion_command_id,
        reset,    heartbeat};
    s3_drive_options_t opt = course_profile_options();
    /* clang-format off */
    g_now          = 0U;
    gb_has_lease   = false;
    g_report_count = g_turns_done = g_advances = 0U;
    /* clang-format on */
    memset(g_duty, 0, sizeof g_duty);
    memset(g_wheel_v, 0, sizeof g_wheel_v);
    memset(g_wheel_pos, 0, sizeof g_wheel_pos);
    memset(g_counts, 0, sizeof g_counts);
    g_axle_x       = x_coord;
    g_axle_y       = y_coord;
    g_axle_heading = heading;
    motion_estimation_init((float32_t)(M_PI * WHEEL_D_M / COUNTS_REV),
                           (float32_t)TRACK_M);
    ctrl_motion_init((float32_t)VMAX_MPS, 0.0f, 0.0f, post);
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    course_profile_navigation(&cfg);

    if (g_steering_kp >= 0.0f)
    {
        cfg.steering.kp_mps = g_steering_kp;
    }

    if (!s3_drive_init(&g_drive, &cfg, &port))
    {
        puts("FAIL: init");
        exit(1);
    }

    opt.turn_advance_m = (float32_t)advance_m;
    opt.advance_mps    = 0.10f;
    opt.p_drive        = drive_m;

    if (!s3_drive_set_options(&g_drive, &opt))
    {
        puts("FAIL: options");
        exit(1);
    }

    calibrate();

    for (int tick_index = 0; tick_index < SIM_START_SETTLE_TICKS; tick_index++)
    {
        course_sim_tick();
    }

    if (!s3_drive_start(&g_drive, g_now))
    {
        puts("FAIL: START refused");
        exit(1);
    }
}

/** @brief Return the simulated distance from the midpoint of the two sensors
 * to the tape.
 */
float64_t course_sim_centre_error (void)
{
    float64_t left_x_m;
    float64_t left_y_m;
    float64_t right_x_m;
    float64_t right_y_m;
    course_sim_sensor_xy(0, &left_x_m, &left_y_m);
    course_sim_sensor_xy(1, &right_x_m, &right_y_m);
    return course_sim_dist_to_tape((left_x_m + right_x_m) / 2,
                                   (left_y_m + right_y_m) / 2);
}

/** @brief Return the read-only fixture label for a valid drive state. */
const char * course_sim_state_name (s3_drive_state_t state)
{
    static const char * state_labels[] = {"OFF",       "FOLLOW",    "BRAKING",
                                          "MANOEUVRE", "REACQUIRE", "HALTED",
                                          "FAULT"};
    return state_labels[state];
}

/** @brief Return the read-only fixture label for a valid drive reason. */
const char * course_sim_reason_name (s3_drive_reason_t reason)
{
    static const char * reason_labels[] = {
        "ok",           "sensor fault", "motion fault", "rejected",
        "line lost",    "no direction", "unmapped",     "timeout",
        "report fault", "bad input"};
    return reason_labels[reason];
}

/** @brief Return the single-threaded fixture pose in metres and radians. */
course_sim_pose_t course_sim_pose (void)
{
    return (course_sim_pose_t){g_axle_x, g_axle_y, g_axle_heading};
}

/** @brief Borrow the fixture drive state for immediate read-only inspection.
 * The next scenario step can change it. */
const s3_drive_t * course_sim_drive (void)
{
    return &g_drive;
}

/** @brief Empty the simulated track and restore the default tape cut-off. */
void course_sim_clear_track (void)
{
    g_pieces     = 0;
    g_tape_end_x = 1e9;
}

/** @brief Set the dimensionless achieved-to-requested angle ratio used by
 * future simulated turns. */
void course_sim_set_turn_scale (float64_t scale)
{
    g_turn_scale = scale;
}

/** @brief Return the cumulative number of failed course verdicts.
 */
int course_sim_failures (void)
{
    return g_failures;
}

/** @brief Print one scenario verdict and increment the fixture failure count if
 * it failed. The display name and detail are borrowed strings. */
void course_sim_verdict (const char * p_name, bool b_is_ok,
                         const char * p_detail)
{
    printf("%-4s %-52s %s\n", b_is_ok ? "PASS" : "FAIL", p_name, p_detail);

    if (!b_is_ok)
    {
        g_failures++;
    }
}

#ifndef COURSE_BYPASS_TEST
/** @brief Run the standalone course scenarios with optional model overrides. */
int main (int argument_count, char * pp_arguments[])
{
    if (argument_count > 1)
    {
        g_sensor_gap = atof(pp_arguments[1]) / 1000.0;
    }

    if (argument_count > 2)
    {
        g_sensor_ahead = atof(pp_arguments[2]) / 1000.0;
    }

    if (argument_count > 3)
    {
        g_steering_kp = (float32_t)atof(pp_arguments[3]);
    }

    const float64_t adv = g_sensor_ahead + (S3_CROSSING_CENTRE_MM / 1000.0);
    printf("Car model: sensor gap %.0f mm, sensors %.0f mm ahead of the axle, "
           "tape %.0f mm\n",
           g_sensor_gap * 1000, g_sensor_ahead * 1000, TAPE_W * 1000);
    scenario_straight(0.0);
    scenario_straight(0.002);
    scenario_straight(-0.002);
    scenario_curve(0.20, +1, true);
    scenario_curve(0.20, -1, true);
    scenario_curve(0.15, +1, true);
    scenario_curve(0.15, -1, true);
    scenario_curve(0.14, +1,
                   true); /* The 27 October test tiles measure about 14 cm. */
    scenario_curve(0.14, -1, true);
    scenario_curve(0.12, +1, true); /* Keep a margin below the tile width. */
    scenario_curve(0.12, -1, true);
    scenario_s_bend(0.15);
    scenario_s_bend(0.14);
    scenario_cross_straight(S3_NAV_NONE);
    scenario_cross_straight(S3_NAV_STRAIGHT);
    scenario_turn(S3_NAV_LEFT, adv, true);
    scenario_turn(S3_NAV_RIGHT, adv, true);
    scenario_turn(S3_NAV_UTURN, adv, true);
    scenario_turn(S3_NAV_LEFT, 0.0, false);
    scenario_turn(S3_NAV_RIGHT, 0.0, false);
    scenario_turn_error(S3_NAV_LEFT, adv,
                        0.70); /* Simulate undershoot at 63 degrees. */
    scenario_turn_error(S3_NAV_RIGHT, adv, 0.70);
    scenario_turn_error(S3_NAV_LEFT, adv,
                        1.30); /* Simulate overshoot at 117 degrees. */
    scenario_turn_error(S3_NAV_RIGHT, adv, 1.30);
    scenario_tape_gone();
    printf("%s: %d failure(s)\n",
           g_failures ? "COURSE SIM FAIL" : "COURSE SIM PASS", g_failures);
    return g_failures ? 1 : 0;
}
#endif

/** @brief Return whether the angular position lies on the directed arc from
 * lo to hi, accounting for wraparound in radians.
 */
static float64_t angle_in (float64_t angle_rad, float64_t start_rad,
                           float64_t end_rad)
{
    /* is angle a inside the sweep lo..hi (either direction)? */
    float64_t arc_span     = end_rad - start_rad;
    float64_t angle_offset = angle_rad - start_rad;

    while (angle_offset < 0)
    {
        angle_offset += 2 * M_PI;
    }

    while (angle_offset >= (2 * M_PI))
    {
        angle_offset -= 2 * M_PI;
    }

    if (arc_span >= 0)
    {
        return angle_offset <= arc_span;
    }

    angle_offset -= 2 * M_PI;
    return angle_offset >= arc_span;
}

/** @brief Estimate the fraction of the round sensor spot covered by tape at
 * the specified course point using a linear edge approximation.
 */
static float64_t coverage (float64_t x_coord, float64_t y_coord)
{
    float64_t coverage_ratio =
        ((TAPE_W / 2) + SPOT_R - course_sim_dist_to_tape(x_coord, y_coord)) /
        (2 * SPOT_R);
    return coverage_ratio < 0 ? 0 : (coverage_ratio > 1 ? 1 : coverage_ratio);
}

/** @brief Append a copy of a motion report when the simulated queue has
 * space. A full queue discards the new report.
 */
static void post (const motion_report_t * p_report)
{
    if (g_report_count < MOTION_REPORT_QUEUE_LEN)
    {
        g_reports[g_report_count++] = *p_report;
    }
}

/** @brief Pop the oldest pending report into the caller's buffer.
 * Acknowledge completed moves to the simulated controller. An empty
 * queue leaves the buffer untouched.
 */
static bool receive (motion_report_t * p_report)
{
    if (0U == g_report_count)
    {
        return false;
    }

    *p_report = g_reports[0];
    memmove(&g_reports[0], &g_reports[1],
            (g_report_count - 1U) * sizeof g_reports[0]);
    g_report_count--;

    if ((unsigned)MOTION_OUTCOME_DONE == p_report->outcome)
    {
        ctrl_motion_ack_complete(p_report->command_id);
    }

    return true;
}

/** @brief Raise a positive speed below MIN_WHEEL to the configured floor.
 * Zero and negative speeds pass through.
 */
static float32_t floor_wheel (float32_t speed_mps)
{
    return ((speed_mps > 0.0f) && (speed_mps < MIN_WHEEL)) ? MIN_WHEEL
                                                           : speed_mps;
}

/** @brief Apply the positive speed floor to each wheel and return the
 * controller result at the simulated timestamp.
 */
static motion_result_t velocity (float32_t left, float32_t right)
{
    return ctrl_motion_set_velocity(floor_wheel(left), floor_wheel(right),
                                    g_now);
}

/** @brief Translate a supported manoeuvre into the corresponding simulated
 * turn or straight move. Return a bad-argument result for an unknown
 * action.
 */
static motion_result_t manoeuvre (motion_manoeuvre_t manoeuvre_kind)
{
    switch (manoeuvre_kind)
    {
    case MANOEUVRE_LEFT:
        return course_sim_turn_deg(90.0f, 0.15f);
    case MANOEUVRE_RIGHT:
        return course_sim_turn_deg(-90.0f, 0.15f);
    case MANOEUVRE_U_TURN:
        return course_sim_turn_deg(180.0f, 0.15f);
    case MANOEUVRE_STRAIGHT:
        return ctrl_motion_request_move(MOTION_DRIVE, 1, 1, 0.05f, 0.20f,
                                        g_now);
    default:
        return MOTION_ERR_BAD_ARG;
    }
}

/** @brief Count a requested advance and submit the absolute distance in
 * metres with its original direction and speed.
 */
static motion_result_t drive_m (float32_t metres, float32_t speed)
{
    int32_t direction = metres >= 0.0f ? 1 : -1;
    g_advances++;
    return ctrl_motion_request_move(MOTION_DRIVE, direction, direction,
                                    fabsf(metres), speed, g_now);
}

/** @brief Request a stop from the motion controller used by the course
 * simulator.
 */
static void brake (void)
{
    ctrl_motion_stop();
}

/** @brief Reset the simulated motion controller and return its
 * acknowledgement.
 */
static bool reset (void)
{
    return ctrl_motion_reset();
}

/** @brief Set the simulated motion lease and refresh its timestamp whenever
 * the lease is active.
 */
static void heartbeat (bool b_is_on)
{
    gb_has_lease = b_is_on;

    if (b_is_on)
    {
        g_lease_stamp = g_now;
    }
}

/** @brief Advance wheel velocities, encoder counts and the axle pose through
 * ten one-millisecond physics steps using the last commanded duties.
 */
static void physics_10ms (void)
{
    const float64_t mpc = M_PI * WHEEL_D_M / COUNTS_REV;

    for (int substep_index = 0; substep_index < PHYSICS_SUBSTEP_COUNT;
         substep_index++)
    {
        for (int wheel_index = 0; wheel_index < WHEEL_COUNT; wheel_index++)
        {
            float64_t target        = g_duty[wheel_index] * VMAX_MPS;
            float64_t absolute_duty = fabs(g_duty[wheel_index]);

            if ((0.0 == g_wheel_v[wheel_index]) && (absolute_duty < BREAKAWAY))
            {
                target = 0.0; /* static */
            }

            if (absolute_duty < KEEP_MOVING)
            {
                target = 0.0;
            }

            g_wheel_v[wheel_index] +=
                (target - g_wheel_v[wheel_index]) * (0.001 / TAU_S);

            if ((fabs(g_wheel_v[wheel_index]) < 1e-4) && (0.0 == target))
            {
                g_wheel_v[wheel_index] = 0.0;
            }

            /* clang-format off */
            g_wheel_pos[wheel_index] += g_wheel_v[wheel_index] * 0.001;
            g_counts[wheel_index]     =
                (int32_t)floor(g_wheel_pos[wheel_index] / mpc);
            /* clang-format on */
        }

        float64_t centre_speed_mps = (g_wheel_v[0] + g_wheel_v[1]) / 2;
        float64_t angular_speed    = (g_wheel_v[1] - g_wheel_v[0]) / TRACK_M;
        g_axle_x       += centre_speed_mps * cos(g_axle_heading) * 0.001;
        g_axle_y       += centre_speed_mps * sin(g_axle_heading) * 0.001;
        g_axle_heading += angular_speed * 0.001;
    }
}

/** @brief Interpolate the selected IR head's white and black ADC calibration
 * midpoints from the fraction of its spot covered by tape.
 */
static uint16_t raw (int which)
{
    static const uint16_t cal[2][4] = {S3_BUILTIN_CAL_LEFT,
                                       S3_BUILTIN_CAL_RIGHT};
    float64_t             x_coord;
    float64_t             y_coord;
    float64_t             white;
    float64_t             black;
    course_sim_sensor_xy(which, &x_coord, &y_coord);
    white = (cal[which][0] + cal[which][1]) / 2.0;
    black = (cal[which][2] + cal[which][3]) / 2.0;
    return (uint16_t)(white + (coverage(x_coord, y_coord) * (black - white)));
}

/* ---- Scenario harness ---------------------------------------------------- */
/** @brief Feed the built-in white and black calibration readings to every
 * simulated sensor. End the fixture with a failure if navigation
 * rejects a calibration.
 */
static void calibrate (void)
{
    static const uint16_t table[S3_SENSOR_COUNT][4] = {
        S3_BUILTIN_CAL_BARCODE, S3_BUILTIN_CAL_LEFT, S3_BUILTIN_CAL_RIGHT};

    for (unsigned sensor_index = 0U; sensor_index < (unsigned)S3_SENSOR_COUNT;
         sensor_index++)
    {
        ir_calibration_collector_t collector;
        ir_calibration_begin(&collector);

        /* Delta casts preserve the unsigned interpolation arithmetic,
         * including the modulo conversion of negative differences. */
        for (unsigned sample_index = 0U;
             sample_index < SIM_CALIBRATION_SAMPLE_COUNT; sample_index++)
        {
            (void)ir_calibration_add(
                &collector, IR_SURFACE_WHITE,
                (uint16_t)(table[sensor_index][0] +
                           (sample_index % 2U *
                            (unsigned)(table[sensor_index][1] -
                                       table[sensor_index][0]))));
            (void)ir_calibration_add(
                &collector, IR_SURFACE_BLACK,
                (uint16_t)(table[sensor_index][2] +
                           (sample_index % 2U *
                            (unsigned)(table[sensor_index][3] -
                                       table[sensor_index][2]))));
        }

        if (!line_navigation_set_calibration(
                &g_drive.navigation, (ir_acquisition_sensor_id_t)sensor_index,
                &collector))
        {
            puts("FAIL: built-in calibration rejected");
            exit(1);
        }
    }
}

#ifndef COURSE_BYPASS_TEST
/** @brief Drive until the position predicate succeeds or the timeout
 * elapses. Track the worst following error after the first 0.3
 * seconds.
 */
static float64_t run (float64_t seconds, bool ( * p_until)(void))
{
    float64_t worst = 0;

    for (unsigned tick_index = 0;
         tick_index < (unsigned)(seconds * SIM_TICKS_PER_SECOND); tick_index++)
    {
        course_sim_tick();

        if ((tick_index > 30U) && ((unsigned)S3_DRIVE_FOLLOW == g_drive.state))
        {
            float64_t centre_error_m = course_sim_centre_error();

            if (centre_error_m > worst)
            {
                worst = centre_error_m;
            }
        }

        if (((unsigned)S3_DRIVE_HALTED == g_drive.state) ||
            ((unsigned)S3_DRIVE_FAULT == g_drive.state))
        {
            break;
        }

        if (p_until && (p_until()))
        {
            break;
        }
    }

    return worst;
}

/** @brief Return whether the simulated axle has passed the positive-x goal.
 */
static bool past_x (void)
{
    return g_axle_x > g_goal_x;
}

/** @brief Return whether the axle's y coordinate is below the selected
 * target.
 */
static bool below_y (void)
{
    return g_axle_y < g_goal_y;
}

/** @brief Return whether the axle's y coordinate exceeds the selected
 * target.
 */
static bool above_y (void)
{
    return g_axle_y > g_goal_y;
}

/** @brief Return whether the axle's x coordinate is below the selected
 * target.
 */
static bool left_of_x (void)
{
    return g_axle_x < g_goal_x;
}

/** @brief Start at the supplied lateral offset in metres and check forward
 * progress and following error on a straight.
 */
static void scenario_straight (float64_t offset)
{
    char detail[160];
    g_pieces     = 0;
    g_tape_end_x = 1e9;
    course_sim_add_line(-0.2, 0, 2.0, 0);
    course_sim_setup(0, offset, 0, 0);
    g_goal_x        = 1.5;
    float64_t worst = run(10.0 + (1.6 / S3_COURSE_BASE_MPS),
                          past_x); /* Accept any cruise speed in this case. */
    snprintf(detail, sizeof detail,
             "reached x=%.2f m, worst centre error %.1f mm, %s", g_axle_x,
             worst * 1000, course_sim_state_name(g_drive.state));
    char name[64];
    snprintf(name, sizeof name, "straight 1.5 m, start %+.0f mm off centre",
             offset * 1000);
    course_sim_verdict(
        name,
        ((g_axle_x > 1.5) && ((unsigned)S3_DRIVE_FOLLOW == g_drive.state)) &&
            (worst < 0.012),
        detail);
}

/** @brief Exercise a straight followed by a 180-degree arc of radius r and a
 * straight return in the selected direction.
 */
static void scenario_curve (float64_t bend_radius_m, int dir,
                            bool b_should_succeed)
{
    char detail[160];
    char name[64];
    g_pieces     = 0;
    g_tape_end_x = 1e9;
    course_sim_add_line(-0.2, 0, 0.4, 0);

    if (dir > 0)
    {
        course_sim_add_arc(0.4, bend_radius_m, bend_radius_m, -M_PI / 2,
                           M_PI / 2);
        course_sim_add_line(0.4, 2 * bend_radius_m, -0.4, 2 * bend_radius_m);
    }
    else
    {
        course_sim_add_arc(0.4, -bend_radius_m, bend_radius_m, M_PI / 2,
                           -M_PI / 2);
        course_sim_add_line(0.4, -2 * bend_radius_m, -0.4, -2 * bend_radius_m);
    }

    course_sim_setup(0, 0, 0, 0);
    g_goal_x = 0.0;

    for (int tick_index = 0; tick_index < CURVE_BEFORE_STOP_TICKS; tick_index++)
    {
        course_sim_tick(); /* Start moving before testing STOP. */
    }

    float64_t worst = run(40, left_of_x);
    bool      b_is_ok =
        (((g_axle_x < 0.0) && ((unsigned)S3_DRIVE_FOLLOW == g_drive.state)) &&
         (worst < 0.015)) &&
        (fabs(g_axle_y - (dir * 2 * bend_radius_m)) < 0.02);
    snprintf(detail, sizeof detail,
             "end (%.2f, %.2f) m, worst centre error %.1f mm, %s", g_axle_x,
             g_axle_y, worst * 1000, course_sim_state_name(g_drive.state));
    snprintf(name, sizeof name, "%s U-shaped curve, radius %.0f cm%s",
             dir > 0 ? "left" : "right", bend_radius_m * 100,
             b_should_succeed ? "" : " (expected to fail)");
    course_sim_verdict(name, b_is_ok == b_should_succeed, detail);
}

/** @brief Run the car through two consecutive 90-degree curves of radius r
 * metres and report its tracking verdict.
 */
static void scenario_s_bend (float64_t bend_radius_m)
{
    char detail[160];
    char name[64];
    g_pieces     = 0;
    g_tape_end_x = 1e9;
    course_sim_add_line(-0.2, 0, 0.3, 0);
    course_sim_add_arc(0.3, bend_radius_m, bend_radius_m, -M_PI / 2,
                       0); /* left 90 */
    course_sim_add_arc(0.3 + (2 * bend_radius_m), bend_radius_m, bend_radius_m,
                       M_PI, M_PI / 2); /* right 90 */
    course_sim_add_line(0.3 + (2 * bend_radius_m), 2 * bend_radius_m, 1.5,
                        2 * bend_radius_m);
    course_sim_setup(0, 0, 0, 0);
    g_goal_x        = 1.2;
    float64_t worst = run(40, past_x);
    bool      b_is_ok =
        (((g_axle_x > 1.2) && (fabs(g_axle_y - (2 * bend_radius_m)) < 0.012)) &&
         ((unsigned)S3_DRIVE_FOLLOW == g_drive.state)) &&
        (worst < 0.015);
    snprintf(detail, sizeof detail,
             "end (%.2f, %.2f) m, worst centre error %.1f mm, %s", g_axle_x,
             g_axle_y, worst * 1000, course_sim_state_name(g_drive.state));
    snprintf(name, sizeof name,
             "S-bend, two 90 degree curves of radius %.0f cm",
             bend_radius_m * 100);
    course_sim_verdict(name, b_is_ok, detail);
}

/** @brief Lay two perpendicular tape segments across the course at x = 0.6
 * metres.
 */
static void cross_track (void)
{
    g_pieces     = 0;
    g_tape_end_x = 1e9;
    course_sim_add_line(-0.2, 0, 1.6, 0);
    course_sim_add_line(0.6, -0.6, 0.6, 0.6);
}

/** @brief Run the crossing with the supplied pending action and check
 * straight-through tracking.
 */
static void scenario_cross_straight (line_navigation_action_t pending)
{
    char detail[160];
    char name[64];
    cross_track();
    course_sim_setup(0, 0, 0, 0);

    if (pending != (unsigned)S3_NAV_NONE)
    {
        g_drive.pending    = pending;
        g_drive.pending_us = g_now;
    }

    g_goal_x          = 1.2;
    float64_t worst   = run(30, past_x);
    bool      b_is_ok = (((g_axle_x > 1.2) && (fabs(g_axle_y) < 0.008)) &&
                    ((unsigned)S3_DRIVE_FOLLOW == g_drive.state)) &&
                   (worst < 0.012);
    snprintf(detail, sizeof detail,
             "end (%.2f, %.3f) m, worst centre error %.1f mm, %s %s", g_axle_x,
             g_axle_y, worst * 1000, course_sim_state_name(g_drive.state),
             course_sim_reason_name(g_drive.reason));
    snprintf(name, sizeof name, "crossing, %s",
             (unsigned)S3_NAV_NONE == pending ? "no barcode: straight across"
                                              : "STRAIGHT action: across");
    course_sim_verdict(name, b_is_ok, detail);
}

/** @brief Exercise a mapped turn at the cross with the supplied axle advance
 * and report the resulting drive state and line acquisition.
 */
static void scenario_turn (line_navigation_action_t action, float64_t advance_m,
                           bool b_should_succeed)
{
    char detail[200];
    char name[96];
    cross_track();
    course_sim_setup(0, 0, 0, advance_m);
    g_drive.pending    = action;
    g_drive.pending_us = g_now;
    bool b_is_ok;

    if ((unsigned)S3_NAV_UTURN == action)
    {
        g_goal_x = -0.1;
        (void)run(40, left_of_x);
        b_is_ok = (g_axle_x < -0.1) && (fabs(g_axle_y) < 0.01);
    }
    else if ((unsigned)S3_NAV_LEFT == action)
    {
        g_goal_y = 0.4;
        (void)run(40, above_y);
        b_is_ok = (g_axle_y > 0.4) && (fabs(g_axle_x - 0.6) < 0.01);
    }
    else
    {
        g_goal_y = -0.4;
        (void)run(40, below_y);
        b_is_ok = (g_axle_y < -0.4) && (fabs(g_axle_x - 0.6) < 0.01);
    }

    b_is_ok = b_is_ok && ((unsigned)S3_DRIVE_FOLLOW == g_drive.state);
    snprintf(detail, sizeof detail,
             "end (%.2f, %.2f) m heading %.0f deg, %s %s, advances=%u",
             g_axle_x, g_axle_y, fmod((g_axle_heading * 180 / M_PI) + 720, 360),
             course_sim_state_name(g_drive.state),
             course_sim_reason_name(g_drive.reason), g_advances);
    snprintf(name, sizeof name, "%s at crossing, advance %.0f mm%s",
             (unsigned)S3_NAV_LEFT == action ? "LEFT"
             : (unsigned)S3_NAV_RIGHT == action ? "RIGHT"
                                                : "U-TURN",
             advance_m * 1000, b_should_succeed ? "" : " (expected: halt)");

    if (!b_should_succeed)
    {
        b_is_ok = ((unsigned)S3_DRIVE_HALTED == g_drive.state) &&
                  ((unsigned)S3_DRIVE_LINE_LOST == g_drive.reason);
        course_sim_verdict(name, b_is_ok, detail);
    }
    else
    {
        course_sim_verdict(name, b_is_ok, detail);
    }
}

/** @brief Exercise recovery after a simulated 90-degree turn finishes off
 * the line. Check whether the sweep finds the new arm.
 */
static void scenario_turn_error (line_navigation_action_t action,
                                 float64_t advance_m, float64_t scale)
{
    char detail[200];
    char name[96];
    bool b_is_ok;
    g_turn_scale = scale;
    cross_track();
    course_sim_setup(0, 0, 0, advance_m);
    g_drive.pending    = action;
    g_drive.pending_us = g_now;

    if ((unsigned)S3_NAV_LEFT == action)
    {
        g_goal_y = 0.4;
        (void)run(40, above_y);
        b_is_ok = (g_axle_y > 0.4) && (fabs(g_axle_x - 0.6) < 0.01);
    }
    else
    {
        g_goal_y = -0.4;
        (void)run(40, below_y);
        b_is_ok = (g_axle_y < -0.4) && (fabs(g_axle_x - 0.6) < 0.01);
    }

    b_is_ok      = b_is_ok && ((unsigned)S3_DRIVE_FOLLOW == g_drive.state);
    g_turn_scale = 1.0;
    snprintf(detail, sizeof detail,
             "end (%.2f, %.2f) m heading %.0f deg, %s %s", g_axle_x, g_axle_y,
             fmod((g_axle_heading * 180 / M_PI) + 720, 360),
             course_sim_state_name(g_drive.state),
             course_sim_reason_name(g_drive.reason));
    snprintf(name, sizeof name, "%s, spin ends at %.0f deg: search finds line",
             (unsigned)S3_NAV_LEFT == action ? "LEFT" : "RIGHT", 90.0 * scale);
    course_sim_verdict(name, b_is_ok, detail);
}

/** @brief End the simulated tape at 0.6 metres and check that the car keeps
 * moving forward in the follow state.
 */
static void scenario_tape_gone (void)
{
    char detail[160];
    g_pieces     = 0;
    g_tape_end_x = 0.6;
    course_sim_add_line(-0.2, 0, 2.0, 0);
    course_sim_setup(0, 0, 0, 0);
    g_goal_x = 1.2;
    (void)run(30, past_x);
    snprintf(
        detail, sizeof detail,
        "end (%.2f, %.3f) m, %s (no line-lost stop by design; GP21 stops it)",
        g_axle_x, g_axle_y, course_sim_state_name(g_drive.state));
    course_sim_verdict("tape ends at 0.6 m: keeps driving straight",
                       (g_axle_x > 1.2) &&
                           ((unsigned)S3_DRIVE_FOLLOW == g_drive.state),
                       detail);
}

#endif

/* End of file. */

