#ifndef BUDDY4_TILT_ESTIMATION_H
#define BUDDY4_TILT_ESTIMATION_H
#include "../app_types.h"
#include "imu_sensor.h"
bool tilt_estimation (const float32_t p_a[3], float32_t * p_pitch_deg,
                      float32_t * p_roll_deg);
bool tilt_heading (const float32_t p_a[3], const float32_t p_m[3],
                   float32_t * p_heading_deg);
float32_t tilt_angle_delta (float32_t now, float32_t previous);
#endif
