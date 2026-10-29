#ifndef BUDDY5_ULTRASONIC_SENSOR_H
#define BUDDY5_ULTRASONIC_SENSOR_H
#include "s5_config.h"
bool ultrasonic_sensor_init(void);
bool ultrasonic_sensor_trigger(uint32_t now);
/* Returns true once per result, including timeout/invalid. Never waits for echo.
 * valid && mm<S5_RANGE_MAX_MM : object at mm.
 * valid && mm==S5_RANGE_MAX_MM: path clear - an echo pulse started but nothing
 *                               returned within range (HC-SR04 open-space case).
 * !valid                      : no echo pulse at all, or a pulse shorter than
 *                               S5_RANGE_MIN_MM - a sensor/wiring problem. */
bool ultrasonic_sensor_poll(uint32_t now,uint16_t *mm,bool *valid);
void ultrasonic_sensor_cancel(void);
bool ultrasonic_pulse_mm(uint32_t pulse_us,uint16_t *mm);
/* As ultrasonic_pulse_mm, but a completed pulse beyond range is "clear"
 * (true, mm=S5_RANGE_MAX_MM) instead of invalid. Too-short pulses stay invalid. */
bool ultrasonic_echo_mm(uint32_t pulse_us,uint16_t *mm);
#endif
