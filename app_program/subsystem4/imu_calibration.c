/** @file imu_calibration.c
 * @brief Collect stationary IMU samples for initial sensor calibration.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 */
#include "imu_calibration.h"
#include "../app_types.h"
#include <math.h>

/** @brief Replace calibration state with a fresh zeroed capture.
 * @see imu_calibration.h for state ownership and the non-null precondition.
 */
void imu_calibration_reset (imu_calibration_t * p_calibration)
{
    *p_calibration = (imu_calibration_t){0};
}

/** @brief Validate stationarity/spread and latch the 100-sample mean bias.
 * @see imu_calibration.h for axes, exact thresholds, resets and ready-state
 * short-circuit behavior. Timestamp continuity is enforced by the caller.
 */
bool imu_calibration_feed (imu_calibration_t *         p_calibration,
                           const imu_sensor_sample_t * p_sample,
                           bool                        b_is_stationary)
{

    if (p_calibration->b_is_calibrated)
    {
        return true;
    }
    /* The sensor uses block data update, so a skipped sample does not
     * invalidate the reading. At rest on level ground this mounting
     * should report 0 g on X and Y and +1 g on Z. */
    if (((((!b_is_stationary) || (!imu_sensor_sample_is_valid(p_sample))) ||
          (fabsf(p_sample->accel_g[0]) > 0.15f)) ||
         (fabsf(p_sample->accel_g[1]) > 0.15f)) ||
        (fabsf(p_sample->accel_g[2] - 1.0f) > 0.15f))
    {
        imu_calibration_reset(p_calibration);
        return false;
    }

    /* IMU_AXIS_COUNT is 3. Converting it to unsigned retains all axes. */
    unsigned axis_index;
    for (axis_index = 0; axis_index < (unsigned int)IMU_AXIS_COUNT;
         ++axis_index)
    {
        const float32_t axis_accel_g = p_sample->accel_g[axis_index];

        if (!p_calibration->count)
        {
            p_calibration->min[axis_index] = axis_accel_g;
            p_calibration->max[axis_index] = axis_accel_g;
        }

        if (axis_accel_g < p_calibration->min[axis_index])
        {
            p_calibration->min[axis_index] = axis_accel_g;
        }

        if (axis_accel_g > p_calibration->max[axis_index])
        {
            p_calibration->max[axis_index] = axis_accel_g;
        }

        if ((p_calibration->max[axis_index] - p_calibration->min[axis_index]) >
            0.04f)
        {
            imu_calibration_reset(p_calibration);
            return false;
        }

        p_calibration->sum[axis_index] += axis_accel_g;
    }

    if (IMU_CAL_SAMPLES == ++p_calibration->count)
    {
        for (axis_index = 0; axis_index < (unsigned int)IMU_AXIS_COUNT;
             ++axis_index)
        {
            p_calibration->bias_g[axis_index] =
                p_calibration->sum[axis_index] / IMU_CAL_SAMPLES;
        }

        p_calibration->bias_g[2]      -= 1.0f;
        p_calibration->b_is_calibrated = true;
    }

    return p_calibration->b_is_calibrated;
}

/* End of file. */

