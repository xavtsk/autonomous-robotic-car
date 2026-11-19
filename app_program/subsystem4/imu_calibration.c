/** @file imu_calibration.c
 * @brief Collect stationary IMU samples for initial sensor calibration.
 */
#include "imu_calibration.h"
#include "../app_types.h"
#include <math.h>

void imu_calibration_reset (imu_calibration_t * p_c)
{
    *p_c = (imu_calibration_t){0};
}

bool imu_calibration_feed (imu_calibration_t * p_c, const imu_sample_t * p_s,
                           bool stationary)
{
    unsigned value_i;
    if (p_c->ready)
    {
        return true;
    }
    /* Overrun only means a sample was skipped; values stay valid (BDU). */
    if (!stationary || !imu_sample_is_valid(p_s) ||
        fabsf(p_s->accel_g[0]) > 0.15f || fabsf(p_s->accel_g[1]) > 0.15f ||
        fabsf(p_s->accel_g[2] - 1.0f) > 0.15f)
    {
        imu_calibration_reset(p_c);
        return false;
    }
    for (value_i = 0; value_i < 3; ++value_i)
    {
        float32_t value_a = p_s->accel_g[value_i];
        if (!p_c->count)
        {
            p_c->min[value_i] = value_a;
            p_c->max[value_i] = value_a;
        }
        if (value_a < p_c->min[value_i])
        {
            p_c->min[value_i] = value_a;
        }
        if (value_a > p_c->max[value_i])
        {
            p_c->max[value_i] = value_a;
        }
        if (p_c->max[value_i] - p_c->min[value_i] > 0.04f)
        {
            imu_calibration_reset(p_c);
            return false;
        }
        p_c->sum[value_i] += value_a;
    }
    if (++p_c->count == IMU_CAL_SAMPLES)
    {
        for (value_i = 0; value_i < 3; ++value_i)
        {
            p_c->bias_g[value_i] = p_c->sum[value_i] / IMU_CAL_SAMPLES;
        }
        p_c->bias_g[2] -= 1.0f;
        p_c->ready = true;
    }
    return p_c->ready;
}

/* End of file. */

