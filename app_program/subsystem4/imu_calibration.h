#ifndef BUDDY4_IMU_CALIBRATION_H
#define BUDDY4_IMU_CALIBRATION_H
#include "imu_sensor.h"
#define IMU_CAL_SAMPLES 100U
typedef struct { float bias_g[3], sum[3], min[3], max[3]; unsigned count; bool ready; } imu_calibration_t;
void imu_calibration_reset(imu_calibration_t *c);
/* Level/stationary capture. Restarts on motion, invalid sample, gaps or spread.
 * One-pose offset only, not a six-face scale calibration. Caller confirms stillness. */
bool imu_calibration_feed(imu_calibration_t *c, const imu_sample_t *s, bool stationary);
#endif
