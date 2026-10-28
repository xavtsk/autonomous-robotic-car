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
    return (s5_config_t){350U,150U,700U,90U,70U,500U,100U,700U,0U,80U};
}
#endif
