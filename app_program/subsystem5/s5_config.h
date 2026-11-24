/** @file s5_config.h
 * @brief Define obstacle sensing, scan geometry and avoidance configuration.
 */
#ifndef S5_CONFIG_H
#define S5_CONFIG_H
#ifndef APP_S5
#define APP_S5 0
#endif
/* The 27 October wiring check identified HC-SR04 TRIG on GP1, ECHO on GP0,
 * and the servo signal on GP12. See app_program/subsystem5/README.md for
 * the sensor datasheet and wiring. Hardware validation covers the echo
 * interface, pulse limits, servo travel and course clearance. */
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
 * The servo cannot look 90 deg sideways (30..150 only), hence the diagonal
 * peek. */
/* Bypass turn angle (13 Nov 2026: 45, was 90). Out and back legs are driven
 * diagonally: sideways offset d costs d*sqrt(2) of travel and also moves the
 * car d forward. avoid_obstacle.c assumes exactly 45 (1.414 / 0.707). */
#define S5_BYPASS_DEG 45
#define S5_PEEK_DEG 60U
/* 25 mm (was 50): stop closer to the obstacle's real rear edge. */
#define S5_PEEK_STEP_MM 25U
/* Car geometry from the team's measurements (Gin, 11 Nov 2026): 125 mm wide
 * (half width 63 in s5_config_defaults), ultrasonic 75 mm ahead of the wheel
 * axle, body 55 mm behind it. Was 100 mm / half width 90 (placeholders). */
#define S5_SONAR_AXLE_MM 75U
#define S5_REAR_AXLE_MM 55U
/* Track return was exercised on a straight and on 20 cm and 30 cm bends
 * before and after an obstacle in testing/subsystem5/bypass_course_main.c.
 * The controller uses the following sequence:
 * 1. The line is watched for while passing the obstacle (leg 3, after the
 *    front face) and on the return diagonal (leg 5). On a bend toward the
 *    bypass side the line comes under the car early; on a straight it is
 *    crossed 135 mm before the end of the diagonal.
 * 2. Line seen: stop, then drive S5_IR_AXLE_MM (IR heads to axle) so the axle
 *    is on the line.
 * 3. Swerve: turn toward the bypass side until the IR sees the line again, up
 *    to S5_SWERVE_OUT_DEG past the heading the car had at the obstacle; if not
 *    found, turn the other way up to S5_SWERVE_IN_DEG past it, then back.
 *    Every swerve turn stops the moment the line is confirmed; then line
 *    following continues from there (axle and IR both on the line = car
 *    pointing along it). On a straight the line is found where the old fixed
 *    45-degree turn back ended, so the straight path is unchanged.
 * 4. No line by the end of the return diagonal (bend away from the bypass
 *    side): drive on along it at most S5_RETURN_EXTRA_MM, still watching. */
/* Passing parallel (17 Nov 2026): each side peek also measures how far the
 * obstacle's side is. If the car has drifted closer than planned (it left a
 * bend crooked, or the turns were not exact) it turns away by half the
 * shortfall in degrees (at most S5_SIDE_TURN_MAX_DEG, at most 3 times) and
 * peeks again. Gap from a reading r: the nearest return of a parallel side in
 * the 60 +/- 15 deg cone is at 75 deg, so gap = r x sin 75 = 0.966 r. */
#ifndef S5_SIDE_TOL_MM
#define S5_SIDE_TOL_MM 8U
#endif
#ifndef S5_SIDE_TURN_MAX_DEG
#define S5_SIDE_TURN_MAX_DEG 10U
#endif
/* Hand-back check: the IR must be on the tape when line following resumes.
 * If braking carried it past, turn back up to S5_CENTRE_DEG (then 2x the
 * other way) at S5_CENTRE_SPEED_MMS, stopping on the tape. */
#define S5_CENTRE_DEG 12
#define S5_CENTRE_SPEED_MMS 50U
/* Distance driven after the side peek reads clear, before turning back in.
 * 0 (17 Nov 2026 evening, track run): the car's back is fully past the
 * obstacle's rear edge + margin (~214 mm, the version that ran cleanly on the
 * straight). 1 = the shorter on-the-spot-turn geometry (~139 mm): it hit the
 * obstacle on the real car. */
#ifndef S5_PASS_EXTRA_V2
#define S5_PASS_EXTRA_V2 0
#endif
#ifndef S5_SWERVE_OUT_DEG
#define S5_SWERVE_OUT_DEG 80
#endif
#ifndef S5_SWERVE_IN_DEG
#define S5_SWERVE_IN_DEG 90
#endif
/* Aim first (13 Nov 2026): on a bend the car stops crooked, so a 45-degree
 * turn from its heading is too shallow relative to the track. After the first
 * scan it turns to face the obstacle centre (the obstacle sits on the line),
 * once, if it is at least S5_AIM_MIN_DEG off, at most S5_AIM_MAX_DEG, then
 * scans again and plans from that square-on view. */
#define S5_AIM_MIN_DEG 5
#define S5_AIM_MAX_DEG 30
/* The bend rule was introduced after the 13 November run 5.
 * The controller estimates the pre-obstacle steering from (right - left) /
 * (right + left) averaged over about half a second and expressed as an
 * integer from -100 to 100 percent. S5_BEND_PCT selects the bend route. A
 * bend uses S5_BEND_AIM_PCT of the bearing, currently 100 percent, capped by
 * S5_BEND_AIM_MAX_DEG. Since 17 November, the return-side S5_SWERVE settings
 * are shared because steering before the obstacle cannot show a bend that
 * starts after it.
 */
/* 20 % (run 7, was 10 %): line-follower wobble on a straight must not count as
 * a bend. The course bends (radius 12..20 cm) give ~30..50 %. */
#ifndef S5_BEND_PCT
#define S5_BEND_PCT 20
#endif
/* 100 (17 Nov 2026, was 200 = double): coming out of a bend the double aim
 * turned the car into the obstacle in the course simulator; the side check
 * while passing now corrects a crooked start instead. */
#ifndef S5_BEND_AIM_PCT
#define S5_BEND_AIM_PCT 100
#endif
#define S5_BEND_AIM_MAX_DEG 40
#define S5_IR_AXLE_MM 135U /* Use the sensor offset from course_profile.h. */
/* 0 (17 Nov 2026, was 100): driving on past the end of the diagonal moved
 * the car away from a line bending away; the measured return distance
 * (avoid_obstacle.c return_lat) and the swerve find it instead. */
#ifndef S5_RETURN_EXTRA_MM
#define S5_RETURN_EXTRA_MM 0U
#endif
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
 * S5_ALLOW_OTHER_SIDE 1 (17 Nov 2026): both sides are real routes. The car
 * goes LEFT unless the left is shown to be impossible:
 *   - the scan puts the obstacle's left edge beyond max_offset_mm, or
 *   - after the 45-degree turn the left diagonal reads blocked on
 *     S5_BLOCK_CONFIRM consecutive fresh readings (one bad echo is not enough),
 * or
 *   - the obstacle is too close for the left out-leg even after reversing
 *     S5_BACKUP_MAX_MM.
 * Then it takes the RIGHT (telemetry: side swapped). 0 = left only: stop
 * (NO_PATH) instead (the 13 Nov behaviour). */
#define S5_PREFER_ROUTE 1U
#ifndef S5_ALLOW_OTHER_SIDE
#define S5_ALLOW_OTHER_SIDE 1
#endif
#define S5_BLOCK_CONFIRM 3U
/* Side clearance kept from the obstacle while passing, mm. */
#ifndef S5_MARGIN_MM
#define S5_MARGIN_MM 30U
#endif
/* Outbound diagonal only: allow another 5 mm sideways once the corner
 * has cleared. The obstacle trigger and minimum side-trim gap stay separate. */
#ifndef S5_OUTWARD_EXTRA_MM
#define S5_OUTWARD_EXTRA_MM 5U
#endif
/* Application settings, NOT measured tolerances. Units mm unless noted.
 * 12-bit distances preserve 1 mm through 4095; timings retain wrap precision.
 */
typedef struct
{
    unsigned detect_mm : 12;
    unsigned emergency_mm : 12;
    unsigned profile_mm : 12;
    unsigned half_width_mm : 12;
    unsigned margin_mm : 12;
    unsigned max_depth_mm : 12;
    unsigned recovery_gate_mm : 12;
    unsigned max_offset_mm : 12;
    unsigned sensor_axle_mm : 12;
    unsigned speed_mms : 9;
} s5_config_t;

/** @brief Return the planned axle-to-obstacle side gap in millimetres.
 * @param[in] p_cfg Valid, readable obstacle configuration.
 * @return Vehicle half-width and side margin plus the outbound allowance.
 */
static inline unsigned s5_outward_gap_mm (const s5_config_t * p_cfg)
{
    /* Each 12-bit input widens exactly to unsigned before addition. */
    return ((unsigned)p_cfg->half_width_mm + (unsigned)p_cfg->margin_mm) +
           S5_OUTWARD_EXTRA_MM;
}

/** @brief Return the configured obstacle detection, scan and bypass geometry
 * for the car. Clearance and range distances use millimetres.
 */
static inline s5_config_t s5_config_defaults (void)
{
    /* detect 250 mm (13 Nov run 3, was 200; 350 before that): more room for the
     * 45-degree out-leg when the car is not square to the obstacle.
     * Emergency must stay well below
     * detect or every avoidance faults at its first leg
     * (avoid_obstacle.c:66). */
    /* profile 350 mm (was 700): only returns near the obstacle detected at
     * <=200 mm belong to it; cones/walls ~50 cm to the side made it look 501 mm
     * wide (NO_PATH). */
    /* margin 30 mm (was 70): smaller sideways swing, less risk of leaving the
     * mat. */
    /* half width 63 mm (was 90): measured 125 mm car width. */
    return (s5_config_t){250U, 80U,  350U, 63U, S5_MARGIN_MM,
                         500U, 100U, 700U, 0U,  80U};
}
#endif

/* End of file. */

