/** @file imu_calibration.h
 * @brief Collect stationary IMU samples for initial sensor calibration.
 */
#ifndef BUDDY4_IMU_CALIBRATION_H
#define BUDDY4_IMU_CALIBRATION_H
#include "../app_types.h"
#include "imu_sensor.h"
#define IMU_CAL_SAMPLES 100U

typedef struct
{
    float32_t bias_g[3];
    float32_t sum[3];
    float32_t min[3];
    float32_t max[3];
    unsigned  count;
    bool      b_is_calibrated;
} imu_calibration_t;

/** @brief Clear the one-pose accelerometer bias and all capture progress.
 * @param[out] p_calibration Non-null caller-owned state; all fields become
 * zero, including count, ready, sums, extrema and bias values.
 * @post The next accepted feed starts a new 100-sample capture.
 * @details No pointer check, allocation or locking occurs. Reset before
 * first use and whenever a completed calibration must be replaced.
 */
void imu_calibration_reset (imu_calibration_t * p_calibration);
/* Level/stationary capture. Restarts on motion, invalid sample, gaps or spread.
 * One-pose offset only, not a six-face scale calibration. Caller confirms
 * stillness. */
/** @brief Accumulate one eligible level/stationary sample or restart capture.
 * @param[in,out] p_calibration Non-null state initialized by reset;
 * caller-owned and exclusively accessed during this call.
 * @param[in] p_sample Stable vehicle-axis sample, X forward/Y left/Z up, in g.
 * A null or invalid sample resets a pending capture; completed state ignores
 * it.
 * @param[in] b_is_stationary Caller-confirmed stillness, required while
 * pending.
 * @return True when calibration is already ready or this accepted sample
 * completes IMU_CAL_SAMPLES; false after accumulating fewer samples or reset.
 * @details Ready state is latched: it returns immediately without inspecting
 * sample or stationary. Otherwise, imu_sensor_sample_is_valid requires valid,
 * finite acceleration and finite magnetics when mag_valid is set. Rejection
 * also occurs if |X| or |Y| exceeds 0.15 g, |Z-1| exceeds 0.15 g, or any
 * axis's running maximum-minus-minimum exceeds 0.04 g. Every rejection
 * clears the complete pending state. Bounds use exact floating comparisons.
 * On the 100th accepted call, each bias is its arithmetic mean in g, with
 * 1 g subtracted from Z; float accumulation is not a scale calibration.
 * Zero X/Y and unit Z are valid level input; NaNs/infinities are rejected
 * through the sample-validity check. Timestamps and overrun are not examined:
 * the gap handling mentioned above belongs to terr_data_process.
 * This function neither retains the sample pointer nor acquires a lock.
 */
bool imu_calibration_feed (imu_calibration_t *         p_calibration,
                           const imu_sensor_sample_t * p_sample,
                           bool                        b_is_stationary);
#endif

/* End of file. */

