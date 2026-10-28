#ifndef BUDDY5_ULTRASONIC_SENSOR_H
#define BUDDY5_ULTRASONIC_SENSOR_H
#include "s5_config.h"
bool ultrasonic_sensor_init(void);
bool ultrasonic_sensor_trigger(uint32_t now);
/* Returns true once per result, including timeout/invalid. Never waits for echo. */
bool ultrasonic_sensor_poll(uint32_t now,uint16_t *mm,bool *valid);
void ultrasonic_sensor_cancel(void);
bool ultrasonic_pulse_mm(uint32_t pulse_us,uint16_t *mm);
#endif
