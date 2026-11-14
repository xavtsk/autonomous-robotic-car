#ifndef S5_CONFIG_H
#define S5_CONFIG_H
#include <stdbool.h>
#include <stdint.h>
#ifndef APP_S5
#define APP_S5 0
#endif
/* User-confirmed HC-SR04: TRIG GP1, ECHO GP0 (27 Oct 2026).
 * User also confirmed servo signal GP12. Activation stays opt-in; pulse
 * limits, ECHO level protection and route geometry need physical validation.
 * Enable after checking the actual sensor, level
 * protection, servo supply/travel and robot geometry. No automatic motor start. */
#ifndef S5_HARDWARE_CONFIRMED
#define S5_HARDWARE_CONFIRMED 0
#endif
#ifndef S5_TRIG_GPIO
#define S5_TRIG_GPIO 1U
#endif
#ifndef S5_ECHO_GPIO
#define S5_ECHO_GPIO 0U
#endif
#ifndef S5_SERVO_GPIO
#define S5_SERVO_GPIO 12U
#endif
#ifndef S5_SERVO_REVERSED
#define S5_SERVO_REVERSED 0
#endif
#define S5_MIN_ANGLE 30U
#define S5_MAX_ANGLE 150U
#define S5_RANGE_MIN_MM 30U
#define S5_RANGE_MAX_MM 4000U
#define S5_RANGE_FRESH_US 250000U
#define S5_PING_PERIOD_US 70000U
#define S5_ECHO_TIMEOUT_US 25000U
#define S5_SCAN_TIMEOUT_US 10000000U
#define S5_SAMPLES_MAX 16U
/* Passing an obstacle of unknown depth (bypass leg 3): stop, look S5_PEEK_DEG
 * toward the obstacle, step S5_PEEK_STEP_MM while it is still beside the car.
 * The servo cannot look 90 deg sideways (30..150 only), hence the diagonal peek. */
/* Bypass turn angle (13 Nov 2026: 45, was 90). Out and back legs are driven
 * diagonally: sideways offset d costs d*sqrt(2) of travel and also moves the
 * car d forward. obstacle_avoidance.c assumes exactly 45 (1.414 / 0.707). */
#define S5_BYPASS_DEG 45
#define S5_PEEK_DEG 60U
/* 25 mm (was 50): stop closer to the obstacle's real rear edge. */
#define S5_PEEK_STEP_MM 25U
/* Car geometry from the team's measurements (Gin, 11 Nov 2026): 125 mm wide
 * (half width 63 in s5_defaults), ultrasonic 75 mm ahead of the wheel axle,
 * body 55 mm behind it. Was 100 mm / half width 90 (placeholders). */
#define S5_SONAR_AXLE_MM 75U
#define S5_REAR_AXLE_MM 55U
/* Finding the line again (13 Nov 2026, run 2): the return diagonal is the
 * same length as the out diagonal, then the car turns back to its heading and
 * sweeps +S5_LINE_SWEEP_DEG / -2x / + back, stopping as soon as tape is
 * confirmed. Tape seen while crossing on the diagonal does not count (the car
 * used to hand back there, already past the line). No straight creeping. */
#define S5_LINE_SWEEP_DEG 45
/* Aim first (13 Nov 2026): on a bend the car stops crooked, so a 45-degree
 * turn from its heading is too shallow relative to the track. After the first
 * scan it turns to face the obstacle centre (the obstacle sits on the line),
 * once, if it is at least S5_AIM_MIN_DEG off, at most S5_AIM_MAX_DEG, then
 * scans again and plans from that square-on view. */
#define S5_AIM_MIN_DEG 5
#define S5_AIM_MAX_DEG 30
/* BEND ONLY (13 Nov run 5). The controller measures how hard the line
 * follower was turning just before the obstacle: (right - left) / (right +
 * left) wheel speed, averaged over ~0.5 s. At or above S5_BEND_RATIO the car
 * is on a bend (0.10 = radius under ~0.6 m with the 116 mm track); below it
 * everything stays exactly as on the straight. On a bend only:
 * - the aim turn is S5_BEND_AIM_PCT of the bearing (the track direction AT the
 *   obstacle is about twice the bearing on a curve), cap S5_BEND_AIM_MAX_DEG;
 *   only when the obstacle is on the same side the car was turning to;
 * - no tape by the end of the return diagonal: up to S5_RETURN_EXTRA_MM more
 *   along it, still watching.
 * ALWAYS (run 7, straight too): tape on the return diagonal stops the car,
 * which then drives only S5_IR_AXLE_MM more so the axle is on the line before
 * turning back. It never drives on past the line for the rest of the planned
 * diagonal (the "goes too far" on the straight when the turns are not exact). */
/* 0.20 (run 7, was 0.10): line-follower wobble on a straight must not count as
 * a bend. The course bends (radius 12..20 cm) give ~0.3..0.5. */
#define S5_BEND_RATIO 0.20
#define S5_BEND_AIM_PCT 200
#define S5_BEND_AIM_MAX_DEG 40
#define S5_IR_AXLE_MM 135U        /* S3_SENSOR_TO_AXLE_MM, course_profile.h */
#define S5_RETURN_EXTRA_MM 100U
/* Worst-case obstacle (13 Nov run 6): up to 20 cm wide or long. The planned
 * sideways offset is never smaller than for a 20 cm wide obstacle centred
 * where the scan saw it, even when the scan reads it narrower. Its depth is
 * measured by the side peeks while passing (max_depth_mm 500 > 200). */
#define S5_OBSTACLE_MAX_MM 200
/* Obstacle too close for the 45-degree out-leg (its front corner would clip
 * the face): reverse by the shortfall first, at most this far. */
#define S5_BACKUP_MAX_MM 100U
/* Scan edge padding, per mille of the farthest return (sin of the padding
 * angle). 173 = sin 10 deg (13 Nov run 3: 87 = sin 5 made the out diagonal
 * too short; 259 = sin 15 made it too long). Lower = shorter out diagonal. */
#define S5_EDGE_INFLATE_PERMIL 173U
/* Straight-leg forward clearance: each straight leg needs a fresh reading
 * beyond this before it drives. Kept at the old 200 mm when the trigger
 * distance (detect_mm) went to 250, so legs do not stop more often. */
#define S5_LEG_CLEAR_MM 200U
/* Bypass side. The ultrasonic cannot see a flat mat edge, so use course
 * knowledge: counter-clockwise laps keep the mat on the LEFT (loop inside).
 * 0 = side with more diagonal clearance, 1 = LEFT first, 2 = RIGHT first.
 * S5_ALLOW_OTHER_SIDE 0 (13 Nov 2026): the preferred side ONLY; if it is
 * blocked the car stops (NO_PATH) instead of going round the other way. */
#define S5_PREFER_ROUTE 1U
#ifndef S5_ALLOW_OTHER_SIDE
#define S5_ALLOW_OTHER_SIDE 0
#endif
/* Application settings, NOT measured tolerances. Units mm unless noted.
 * 12-bit distances preserve 1 mm through 4095; timings retain wrap precision. */
typedef struct {
    unsigned detect_mm:12, emergency_mm:12, profile_mm:12;
    unsigned half_width_mm:12, margin_mm:12, max_depth_mm:12;
    unsigned recovery_gate_mm:12, max_offset_mm:12, sensor_axle_mm:12;
    unsigned speed_mms:9;
} s5_config_t;
static inline s5_config_t s5_defaults(void)
{
    /* detect 250 mm (13 Nov run 3, was 200; 350 before that): more room for the
     * 45-degree out-leg when the car is not square to the obstacle.
     * Emergency must stay well below
     * detect or every avoidance faults at its first leg (obstacle_avoidance.c:66). */
    /* profile 350 mm (was 700): only returns near the obstacle detected at <=200 mm
     * belong to it; cones/walls ~50 cm to the side made it look 501 mm wide (NO_PATH). */
    /* margin 30 mm (was 70): smaller sideways swing, less risk of leaving the mat. */
    /* half width 63 mm (was 90): measured 125 mm car width. */
    return (s5_config_t){250U,80U,350U,63U,30U,500U,100U,700U,0U,80U};
}
#endif
