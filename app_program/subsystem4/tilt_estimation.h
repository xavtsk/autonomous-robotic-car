#ifndef BUDDY4_TILT_ESTIMATION_H
#define BUDDY4_TILT_ESTIMATION_H
#include "imu_sensor.h"
bool tilt_estimation (const float p_a[3], float * p_pitch_deg,
                      float * p_roll_deg);
bool tilt_heading (const float p_a[3], const float p_m[3],
                   float * p_heading_deg);
float tilt_angle_delta (float now, float previous);
#endif
