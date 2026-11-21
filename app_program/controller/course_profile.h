/** @file course_profile.h
 * @brief Line-following profile for the Buddies 1-3 course car (integration
 * build).
 *
 * The course assumes the car starts centred on 18 mm tape, with one sensor
 * on each side. Two white readings mean centred during ordinary following.
 * A sensor on tape produces the corresponding steering correction. This
 * interpretation assumes curves remain within the car's steering range.
 *
 * The default profile permits a straight crossing without a pending map
 * action. A mapped left or right turn advances until the axle reaches the
 * crossing before turning. The turn completes when a front sensor reacquires
 * tape in the configured window or bounded search. Search failure stops the
 * run with a line fault.
 *
 * @see app_program/controller/README.md for the integration profile.
 * @see app_program/subsystem3/INTEGRATION.md for the standalone defaults and
 * the recorded barcode path.
 */
#ifndef S3_COURSE_PROFILE_H
#define S3_COURSE_PROFILE_H
#include "../app_types.h"
#include "s3_drive.h"

/* This optional plain-surface test disables IR steering corrections.
 * Barcode actions, operator STOP and fault handling stay enabled. */
#ifndef S3_STRAIGHT_TEST
#define S3_STRAIGHT_TEST 0
#endif
#if S3_STRAIGHT_TEST != 0 && S3_STRAIGHT_TEST != 1
#error S3_STRAIGHT_TEST must be 0 or 1
#endif
#ifndef S3_LINE_BETWEEN
#define S3_LINE_BETWEEN 1 /* Zero selects the original edge design. */
#endif
#ifndef S3_CROSS_STRAIGHT
#define S3_CROSS_STRAIGHT 1 /* Zero halts at an unmapped crossing. */
#endif
/* Measure from the driven axle to the front IR sensor heads in millimetres.
 * Zero disables the configured junction advance before a turn. */
#ifndef S3_SENSOR_TO_AXLE_MM
#define S3_SENSOR_TO_AXLE_MM 135
#endif
/* The gain is the correction in m/s at full normalized steering error.
 * The 27 October practice tiles had curves of about 14 cm radius. The
 * greater course gain allows the inner wheel to stop at full error. The
 * course simulator in testing/integration/course_sim_main.c exercises the
 * steering model; its result is separate from a physical track measurement. */
#ifndef S3_COURSE_STEERING_KP
#define S3_COURSE_STEERING_KP (0.10f)
#endif
/* This is the requested cruise speed in m/s. Integrated sampling runs every
 * 5 ms (run_timing.h) and the decoder defaults to a 25 ms minimum narrow run
 * (bc_decoder.c). */
#ifndef S3_COURSE_BASE_MPS
#define S3_COURSE_BASE_MPS (0.10f)
#endif
/* The junction is reported a few mm after the sensors reach the crossing tape;
 * the crossing centre is about this much further on. */
#define S3_CROSSING_CENTRE_MM 5
#define S3_ADVANCE_MPS (0.10f)
/* This duration keeps a decoded action pending for its crossing. On the
 * practice course, Y was about 1 to 1.4 m before crossing 2. A 5 s timeout
 * was too short for that placement. */
#ifndef S3_COURSE_PENDING_WAIT_US
#define S3_COURSE_PENDING_WAIT_US (40000000U)
#endif
/* A sensor must have touched the tape this recently when the spin is done. */
#ifndef S3_TURN_LINE_WINDOW_US
#define S3_TURN_LINE_WINDOW_US (300000U)
#endif

/* These are reflectance hysteresis limits for the GP28 barcode channel.
 * Calibrated darkness runs from 0 (white) to 1000 (black). The previous
 * surface state is held between the limits. Raw ADC equivalents depend on
 * the channel calibration in builtin_calibration.h. The recording decoder
 * applies its own edge and prominence checks in bc_decoder.c. */
#ifndef S3_BARCODE_WHITE_LIMIT
#define S3_BARCODE_WHITE_LIMIT (400U)
#endif
#ifndef S3_BARCODE_BLACK_LIMIT
#define S3_BARCODE_BLACK_LIMIT (600U)
#endif

/* Limit the search after a turn that ends without a tape reading. A front
 * sensor reading ends the search. The sweep assumes a limit below 45
 * degrees to reduce the chance of selecting an adjacent crossing arm.
 * Zero disables the search. */
#ifndef S3_TURN_SEARCH_DEG
#define S3_TURN_SEARCH_DEG (35.0f)
#endif
#define S3_TURN_SEARCH_MPS (0.05f) /* Set each wheel's search speed. */
#define S3_TRACK_M (0.116f)        /* Use the Buddy 2 axle track. */

/** @brief Apply the selected track geometry, barcode thresholds and wheel
 * speed settings to the caller's navigation configuration.
 */
static inline void course_profile_navigation (line_navigation_config_t * p_cfg)
{
    p_cfg->calibration[S3_SENSOR_BARCODE].white_limit = S3_BARCODE_WHITE_LIMIT;
    p_cfg->calibration[S3_SENSOR_BARCODE].black_limit = S3_BARCODE_BLACK_LIMIT;

    /* clang-format off */
    p_cfg->line.b_is_tape_between = S3_STRAIGHT_TEST || (S3_LINE_BETWEEN != 0);
    p_cfg->steering.kp_mps        =
        S3_STRAIGHT_TEST ? 0.0f : S3_COURSE_STEERING_KP;
    p_cfg->steering.base_mps      = S3_COURSE_BASE_MPS;
    /* clang-format on */
}

/** @brief Return drive options for this firmware profile, including junction
 * handling, pending barcode timing and turn reacquisition.
 */
static inline s3_drive_options_t course_profile_options (void)
{
    s3_drive_options_t options = s3_drive_default_options();
    options.b_allows_cross_straight =
        S3_STRAIGHT_TEST || (S3_CROSS_STRAIGHT != 0);

    options.pending_wait_us = S3_COURSE_PENDING_WAIT_US;
    /* "touching" is only produced by the between-sensors detector. */
    options.turn_line_window_us =
        (S3_STRAIGHT_TEST || S3_LINE_BETWEEN) ? S3_TURN_LINE_WINDOW_US : 0U;

    if (options.turn_line_window_us != 0U)
    {
        options.search_deg = S3_TURN_SEARCH_DEG;
        options.search_mps = S3_TURN_SEARCH_MPS;
        options.track_m    = S3_TRACK_M;
    }
#if S3_SENSOR_TO_AXLE_MM > 0
    options.turn_advance_m =
        (float32_t)(S3_SENSOR_TO_AXLE_MM + S3_CROSSING_CENTRE_MM) / 1000.0f;

    options.advance_mps = S3_ADVANCE_MPS;
    options.p_drive     = motion_request_drive;
#endif
    return options;
}
#endif

/* End of file. */

