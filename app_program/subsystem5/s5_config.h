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
#define S5_PEEK_DEG 60U
#define S5_PEEK_STEP_MM 50U
/* Ultrasonic sensor ahead of the wheel axle. MEASURE on the car and update. */
#define S5_SONAR_AXLE_MM 100U
/* Bypass side. The ultrasonic cannot see a flat mat edge, so use course
 * knowledge: counter-clockwise laps keep the mat on the LEFT (loop inside).
 * 0 = side with more diagonal clearance, 1 = LEFT first, 2 = RIGHT first.
 * The other side is tried if the first side's path is blocked. */
#define S5_PREFER_ROUTE 1U
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
    /* detect 200 mm (was 350) for the tight course; emergency must stay well below
     * detect or every avoidance faults at its first leg (obstacle_avoidance.c:66). */
    /* profile 350 mm (was 700): only returns near the obstacle detected at <=200 mm
     * belong to it; cones/walls ~50 cm to the side made it look 501 mm wide (NO_PATH). */
    /* margin 30 mm (was 70): smaller sideways swing, less risk of leaving the mat. */
    return (s5_config_t){200U,80U,350U,90U,30U,500U,100U,700U,0U,80U};
}
#endif
