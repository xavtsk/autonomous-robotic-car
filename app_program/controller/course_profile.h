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
#define S3_SENSOR_TO_AXLE_MM 0
#endif
/* Steering gain (m/s of wheel-speed difference per full error). Buddy 3's
 * default 0.06 gives a tightest turn of about 13 cm radius (inner wheel held at
 * the 0.06 m/s minimum); the course tiles measured about 14 cm on 27 Oct, too
 * little margin. 0.10 stops the inner wheel on a full error (0 m/s is not a
 * stall) and the outer runs at 0.20 m/s: simulator follows 10 cm curves. */
#ifndef S3_COURSE_STEERING_KP
#define S3_COURSE_STEERING_KP (0.10f)
#endif
/* The junction is reported a few mm after the sensors reach the crossing tape;
 * the crossing centre is about this much further on. */
#define S3_CROSSING_CENTRE_MM 5
#define S3_ADVANCE_MPS (0.10f)
/* Barcode -> crossing: Y is about 1 m before crossing 2 on the practice
 * course (about 10 s at 0.10 m/s); the original 5 s limit would time out. */
#ifndef S3_COURSE_PENDING_WAIT_US
#define S3_COURSE_PENDING_WAIT_US (20000000U)
#endif
/* A sensor must have touched the tape this recently when the spin is done. */
#ifndef S3_TURN_LINE_WINDOW_US
#define S3_TURN_LINE_WINDOW_US (300000U)
#endif

static inline void s3_course_navigation(s3_navigation_config_t *cfg)
{
    cfg->line.between = (S3_LINE_BETWEEN != 0);
    cfg->steering.kp_mps = S3_COURSE_STEERING_KP;
}
static inline s3_drive_options_t s3_course_options(void)
{
    s3_drive_options_t o = s3_drive_default_options();
    o.cross_straight = (S3_CROSS_STRAIGHT != 0);
    o.pending_wait_us = S3_COURSE_PENDING_WAIT_US;
    /* "touching" is only produced by the between-sensors detector. */
    o.turn_line_window_us = S3_LINE_BETWEEN ? S3_TURN_LINE_WINDOW_US : 0U;
#if S3_SENSOR_TO_AXLE_MM > 0
    o.turn_advance_m =
        (float)(S3_SENSOR_TO_AXLE_MM + S3_CROSSING_CENTRE_MM) / 1000.0f;
    o.advance_mps = S3_ADVANCE_MPS;
    o.drive = motion_request_drive;
#endif
    return o;
}
#endif
