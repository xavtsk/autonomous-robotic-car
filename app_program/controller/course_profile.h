/* Line-following profile for the Buddies 1-3 course car (integration build).
 *
 * Agreed 27 Oct 2026 (hardware session follow-up):
 *  - The two front sensors are further apart than the 18 mm tape, so the tape
 *    runs BETWEEN them. Both white = centred = go straight; one sensor on the
 *    tape = steer towards it (Buddy 3's unchanged R - L error). No "line lost"
 *    while following: the car always starts on the line and the tiles' curve
 *    radius is within the car's tightest steering turn.
 *  - A crossing with no barcode before it (or a STRAIGHT action) is driven
 *    straight across instead of halting.
 *  - LEFT/RIGHT turns pivot on the axle, which is behind the sensors: drive on
 *    so the axle is over the crossing first (needs the measured distance).
 *  - After LEFT/RIGHT/U-TURN a front sensor must have crossed the new line at
 *    the end of the spin; otherwise the run halts with "line lost".
 * Buddy 3's own build/tests keep his original behaviour (all switches off).
 * Every value here is a compile-time override (-D...). */
#ifndef S3_COURSE_PROFILE_H
#define S3_COURSE_PROFILE_H
#include "s3_drive.h"

#ifndef S3_LINE_BETWEEN
#define S3_LINE_BETWEEN 1        /* 0 = Buddy 3's original edge design */
#endif
#ifndef S3_CROSS_STRAIGHT
#define S3_CROSS_STRAIGHT 1      /* 0 = halt at a crossing with no action */
#endif
/* MEASURE on the car: middle of the driven-wheel axle to the front IR sensor
 * heads, in millimetres. 0 = unknown: LEFT/RIGHT turn where the car stops,
 * and the after-turn check will usually halt them (sensors beside the line). */
#ifndef S3_SENSOR_TO_AXLE_MM
#define S3_SENSOR_TO_AXLE_MM 135
#endif
/* Steering gain (m/s of wheel-speed difference per full error). Buddy 3's
 * default 0.06 gives a tightest turn of about 13 cm radius (inner wheel held at
 * the 0.06 m/s minimum); the course tiles measured about 14 cm on 27 Oct, too
 * little margin. 0.10 stops the inner wheel on a full error (0 m/s is not a
 * stall) and the outer runs at 0.20 m/s: simulator follows 10 cm curves. */
#ifndef S3_COURSE_STEERING_KP
#define S3_COURSE_STEERING_KP (0.10f)
#endif
/* Cruise speed. Buddy 3's default is 0.10 m/s; on 27 Oct no barcode was
 * decoded at that speed (the printed bars give the barcode sensor weak
 * contrast, and thin bars pass in ~35 ms against a 30 ms minimum). 0.07 m/s
 * gave each bar ~40 % more time; 0.06 m/s on request (27 Oct, 20:08). 0.06 is
 * also the integration minimum wheel speed (S3_MIN_WHEEL_MPS): going lower
 * needs that floor lowered too, and 0.04 m/s stalled a wheel on 26 Oct. */
#ifndef S3_COURSE_BASE_MPS
#define S3_COURSE_BASE_MPS (0.04f)
#endif
/* The junction is reported a few mm after the sensors reach the crossing tape;
 * the crossing centre is about this much further on. */
#define S3_CROSSING_CENTRE_MM 5
#define S3_ADVANCE_MPS (0.10f)
/* Barcode -> crossing: Y is about 1-1.4 m before crossing 2 on the practice
 * course; the original 5 s limit would time out. */
#ifndef S3_COURSE_PENDING_WAIT_US
#define S3_COURSE_PENDING_WAIT_US (40000000U) /* ~1.4 m at 0.07 m/s = 20 s */
#endif
/* A sensor must have touched the tape this recently when the spin is done. */
#ifndef S3_TURN_LINE_WINDOW_US
#define S3_TURN_LINE_WINDOW_US (300000U)
#endif

/* Barcode sensor (GP28) only: darkness cut-offs, 0 (white) .. 1000 (black).
 * White <= WHITE_LIMIT, black >= BLACK_LIMIT; a reading in between keeps the
 * previous answer (stops flicker). Line sensors always use 400/600.
 * 27 Oct 21:48: barcode recalibrated (builtin_calibration.h, white ~215 raw,
 * bars ~3550 raw), so the normal 400/600 apply again (= raw ~1550 / ~2220).
 * The old 650/700 was a workaround for the stale 156/305 calibration.
 * Tune: white gaps come out too short -> raise WHITE (keep it below every
 * bar); thin bars missed -> lower BLACK (keep it above every white). */
#ifndef S3_BARCODE_WHITE_LIMIT
#define S3_BARCODE_WHITE_LIMIT (400U)
#endif
#ifndef S3_BARCODE_BLACK_LIMIT
#define S3_BARCODE_BLACK_LIMIT (600U)
#endif

/* After a turn that ends off the line: sweep up to +/- this many degrees
 * around where the turn ended, stop as soon as a front sensor finds the tape
 * and follow it; not found -> halt "line lost". 0 = halt at once. Keep < 45
 * so it cannot pick up a neighbouring arm of the crossing. */
#ifndef S3_TURN_SEARCH_DEG
#define S3_TURN_SEARCH_DEG (35.0f)
#endif
#define S3_TURN_SEARCH_MPS (0.05f) /* each wheel, while sweeping */
#define S3_TRACK_M (0.116f)        /* wheel centre-to-centre (Buddy 2) */

static inline void s3_course_navigation(s3_navigation_config_t *cfg)
{
    cfg->calibration[S3_SENSOR_BARCODE].white_limit = S3_BARCODE_WHITE_LIMIT;
    cfg->calibration[S3_SENSOR_BARCODE].black_limit = S3_BARCODE_BLACK_LIMIT;
    cfg->line.between = (S3_LINE_BETWEEN != 0);
    cfg->steering.kp_mps = S3_COURSE_STEERING_KP;
    cfg->steering.base_mps = S3_COURSE_BASE_MPS;
}
static inline s3_drive_options_t s3_course_options(void)
{
    s3_drive_options_t o = s3_drive_default_options();
    o.cross_straight = (S3_CROSS_STRAIGHT != 0);
    o.pending_wait_us = S3_COURSE_PENDING_WAIT_US;
    /* "touching" is only produced by the between-sensors detector. */
    o.turn_line_window_us = S3_LINE_BETWEEN ? S3_TURN_LINE_WINDOW_US : 0U;
    if (o.turn_line_window_us != 0U)
    {
        o.search_deg = S3_TURN_SEARCH_DEG;
        o.search_mps = S3_TURN_SEARCH_MPS;
        o.track_m = S3_TRACK_M;
    }
#if S3_SENSOR_TO_AXLE_MM > 0
    o.turn_advance_m =
        (float)(S3_SENSOR_TO_AXLE_MM + S3_CROSSING_CENTRE_MM) / 1000.0f;
    o.advance_mps = S3_ADVANCE_MPS;
    o.drive = motion_request_drive;
#endif
    return o;
}
#endif
