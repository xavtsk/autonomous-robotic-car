#ifndef BUDDY4_TILT_ESTIMATION_H
#define BUDDY4_TILT_ESTIMATION_H
#include "imu_sensor.h"
bool tilt_estimation(const float a[3], float *pitch_deg, float *roll_deg);
bool tilt_heading(const float a[3], const float m[3], float *heading_deg);
float tilt_angle_delta(float now, float previous);
#endif
