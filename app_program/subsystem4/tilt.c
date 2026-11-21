/** @file tilt.c
 * @brief Estimate tilt and heading and calculate wrapped angle differences.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 */
#include "tilt.h"
#include "../app_types.h"
#include <math.h>
#define DEG 57.295779513f

/** @brief Reject unusable acceleration before assigning pitch and roll.
 * @param[in] p_accel Stable vehicle-axis acceleration in g.
 * @param[out] p_pitch_deg Nose-up-positive degrees; unchanged on rejection.
 * @param[out] p_roll_deg Right-side-up-positive degrees; unchanged on
 * rejection.
 * @return True for an accepted near-1 g vector with positive Z, else false.
 * @see tilt.h for exact bounds, pointer ownership and signed zero.
 */
bool tilt_estimation (const float32_t p_accel[3], float32_t * p_pitch_deg,
                      float32_t * p_roll_deg)
{
    const float32_t accel_norm_sq = (p_accel[0] * p_accel[0]) +
                                    (p_accel[1] * p_accel[1]) +
                                    (p_accel[2] * p_accel[2]);

    if ((((!isfinite(accel_norm_sq)) || (accel_norm_sq < 0.81f)) ||
         (accel_norm_sq > 1.21f)) ||
        (p_accel[2] <= 0.0f))
    {
        return false;
    }
    /* Pitch is positive nose-up. Roll is positive right-side-up. In the
     * vehicle frame defined in imu_sensor.h, lifting the right side makes
     * the measured acceleration along leftward Y negative. */
    /* clang-format off */
    *p_pitch_deg =
        atan2f(p_accel[0],
               sqrtf((p_accel[1] * p_accel[1]) +
                     (p_accel[2] * p_accel[2]))) * DEG;
    *p_roll_deg  = atan2f(-p_accel[1], p_accel[2]) * DEG;
    /* clang-format on */
    return true;
}

/** @brief Project magnetic north and vehicle forward onto the level plane.
 * @param[in] p_accel Stable vehicle-axis acceleration in g.
 * @param[in] p_mag Stable vehicle-axis magnetic field in gauss.
 * @param[out] p_heading_deg Anticlockwise degrees; unchanged on rejected
 * geometry.
 * @return True after assigning a heading, false when a norm/projection fails.
 * @see tilt.h for thresholds, caller calibration and zero handling.
 */
bool tilt_heading (const float32_t p_accel[3], const float32_t p_mag[3],
                   float32_t * p_heading_deg)
{
    /* The near-1 g acceleration defines up. Project the vehicle X axis
     * onto its normal plane, form the left vector by cross product, and
     * measure calibrated magnetic heading in that basis. The angle
     * increases anticlockwise, matching Buddy 2. */
    const float32_t accel_norm =
        sqrtf((p_accel[0] * p_accel[0]) + (p_accel[1] * p_accel[1]) +
              (p_accel[2] * p_accel[2]));
    const float32_t mag_norm_sq =
        (p_mag[0] * p_mag[0]) + (p_mag[1] * p_mag[1]) + (p_mag[2] * p_mag[2]);
    float32_t up_axis[3];
    float32_t forward[3];
    float32_t left[3];

    if ((((((!isfinite(accel_norm)) || (accel_norm < 0.9f)) ||
           (accel_norm > 1.1f)) ||
          (!isfinite(mag_norm_sq))) ||
         (mag_norm_sq < 0.01f)) ||
        (mag_norm_sq > 0.64f))
    {
        return false;
    }

    /* Both loops visit the three axes. The positive axis count fits in
     * unsigned int without changing the bound. */
    unsigned axis_index;
    for (axis_index = 0; axis_index < (unsigned int)IMU_AXIS_COUNT;
         ++axis_index)
    {
        /* clang-format off */
        up_axis[axis_index] = p_accel[axis_index] / accel_norm;
        forward[axis_index] =
            (0U == axis_index ? 1.0f : 0.0f) -
            (up_axis[0] * up_axis[axis_index]);
        /* clang-format on */
    }

    const float32_t forward_norm =
        sqrtf((forward[0] * forward[0]) + (forward[1] * forward[1]) +
              (forward[2] * forward[2]));

    if (forward_norm < 0.2f)
    {
        return false;
    }

    for (axis_index = 0; axis_index < (unsigned int)IMU_AXIS_COUNT;
         ++axis_index)
    {
        forward[axis_index] /= forward_norm;
    }

    left[0] = (up_axis[1] * forward[2]) - (up_axis[2] * forward[1]);
    left[1] = (up_axis[2] * forward[0]) - (up_axis[0] * forward[2]);
    left[2] = (up_axis[0] * forward[1]) - (up_axis[1] * forward[0]);
    const float32_t mag_forward = (p_mag[0] * forward[0]) +
                                  (p_mag[1] * forward[1]) +
                                  (p_mag[2] * forward[2]);
    const float32_t mag_left =
        (p_mag[0] * left[0]) + (p_mag[1] * left[1]) + (p_mag[2] * left[2]);

    if (((mag_forward * mag_forward) + (mag_left * mag_left)) < 0.0025f)
    {
        return false;
    }

    *p_heading_deg = -atan2f(mag_left, mag_forward) * DEG;

    if (*p_heading_deg < 0)
    {
        *p_heading_deg += 360.0f;
    }

    return true;
}

/** @brief Apply one strict +/-180-degree correction to a heading difference.
 * @see tilt.h for supported heading domains and nonfinite inputs.
 */
float32_t tilt_angle_delta (float32_t now, float32_t previous)
{
    float32_t delta_deg = now - previous;

    if (delta_deg > 180.0f)
    {
        delta_deg -= 360.0f;
    }
    else if (delta_deg < -180.0f)
    {
        delta_deg += 360.0f;
    }
    else
    {
        /* The angular difference is already within the wrap limits. */
    }

    return delta_deg;
}

/* End of file. */

