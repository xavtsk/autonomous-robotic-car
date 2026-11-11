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
#define S5_SAMPLES_MAX 18U
/* Passing an obstacle of unknown depth (bypass leg 3): stop, look S5_PEEK_DEG
 * toward the obstacle, step S5_PEEK_STEP_MM while it is still beside the car.
 * The servo cannot look 90 deg sideways (30..150 only), hence the diagonal peek. */
#define S5_PEEK_DEG 60U
#define S5_PEEK_STEP_MM 25U
/* Matches the existing course_profile.h IR-to-axle setting. This is inherited
 * configuration, not a new physical measurement; confirm the actual mounting. */
#ifndef S5_IR_AXLE_MM
#define S5_IR_AXLE_MM 135U
#endif
/* Geometry is configured in s5_config_t. Ultrasonic readings cannot establish
 * whether a flat mat/table continues beside the car. A zero side-space budget
 * therefore means UNKNOWN, not unlimited clearance. */
/* Application settings, NOT measured tolerances. Units mm unless noted.
 * 12-bit distances preserve 1 mm through 4095; timings retain wrap precision. */
typedef struct {
    unsigned detect_mm:12, emergency_mm:12, profile_mm:12;
    unsigned half_width_mm:12, margin_mm:12, max_depth_mm:12;
    unsigned recovery_gate_mm:12, max_offset_mm:12, sensor_axle_mm:12;
    unsigned speed_mms:9;
    /* Body extents from the wheel axle; sensor_axle_mm above is the IR sensor.
     * Space is measured from the original axle centreline to the usable edge. */
    unsigned front_axle_mm:12, rear_axle_mm:12, sonar_axle_mm:12;
    unsigned left_space_mm:12, right_space_mm:12;
} s5_config_t;
static inline s5_config_t s5_defaults(void)
{
    /* Compact 45-degree departure, checking the actual swept corners. The
     * 300 mm trigger is the requested upper end of 200..300 mm; a 200 mm
     * approach can still be too close for a wide observed obstacle.
     * User measurements: width 125 mm (rounded up to half-width 63), axle to
     * front 125, rear 55, sonar 75. These explicit axle extents conservatively
     * supersede the inconsistent overall length estimate of 165 mm.
     * Floor budget is 450 mm on each side of the line, confirmed by the user;
     * update it if the course provides less space. IR uses the existing course
     * setting rather than a newly measured value. */
    return (s5_config_t){
        .detect_mm=300U,.emergency_mm=80U,.profile_mm=500U,
        .half_width_mm=63U,.margin_mm=30U,.max_depth_mm=500U,
        .recovery_gate_mm=100U,.max_offset_mm=700U,.sensor_axle_mm=S5_IR_AXLE_MM,
        .speed_mms=80U,.front_axle_mm=125U,.rear_axle_mm=55U,
        .sonar_axle_mm=75U,.left_space_mm=450U,.right_space_mm=450U
    };
}
#endif
