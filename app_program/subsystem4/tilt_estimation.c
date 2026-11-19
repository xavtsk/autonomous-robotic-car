/** @file tilt_estimation.c
 * @brief Estimate tilt and heading and calculate wrapped angle differences.
 */
#include "tilt_estimation.h"
#include "../app_types.h"
#include <math.h>
#define DEG 57.295779513f

/** @brief Reject unusable acceleration before assigning pitch and roll.
 * @param[in] p_a Stable vehicle-axis acceleration in g.
 * @param[out] p_pitch Nose-up-positive degrees; unchanged on rejection.
 * @param[out] p_roll Right-side-up-positive degrees; unchanged on rejection.
 * @return True for an accepted near-1 g vector with positive Z, else false.
 * @see tilt_estimation.h for exact bounds, pointer ownership and signed zero.
 */
bool tilt_estimation (const float32_t p_a[3], float32_t * p_pitch,
                      float32_t * p_roll)
{
    float32_t value_n = p_a[0] * p_a[0] + p_a[1] * p_a[1] + p_a[2] * p_a[2];
    if (!isfinite(value_n) || value_n < 0.81f || value_n > 1.21f ||
        p_a[2] <= 0.0f)
    {
        return false;
    }
    /* Positive pitch = nose up; positive roll = right side up (team choice,
     * 3 Nov 2026). Accelerometer measures support against gravity; a[1] is
     * the vehicle-left axis, so right side up makes it negative. */
    *p_pitch = atan2f(p_a[0], sqrtf(p_a[1] * p_a[1] + p_a[2] * p_a[2])) * DEG;
    *p_roll = atan2f(-p_a[1], p_a[2]) * DEG;
    return true;
}

/** @brief Project magnetic north and vehicle forward onto the level plane.
 * @param[in] p_a Stable vehicle-axis acceleration in g.
 * @param[in] p_m Stable vehicle-axis magnetic field in gauss.
 * @param[out] p_heading Anticlockwise degrees; unchanged on rejected geometry.
 * @return True after assigning a heading, false when a norm/projection fails.
 * @see tilt_estimation.h for thresholds, caller calibration and zero handling.
 */
bool tilt_heading (const float32_t p_a[3], const float32_t p_m[3],
                   float32_t * p_heading)
{
    /* Project vehicle forward and magnetic north onto the gravity-normal plane.
     * Heading increases anticlockwise, matching Buddy 2. */
    float32_t value_n =
        sqrtf(p_a[0] * p_a[0] + p_a[1] * p_a[1] + p_a[2] * p_a[2]);
    float32_t norm = p_m[0] * p_m[0] + p_m[1] * p_m[1] + p_m[2] * p_m[2];
    float32_t value_u[3], value_f[3], left[3], value_fn, value_x, value_y;
    unsigned value_i;
    if (!isfinite(value_n) || value_n < 0.9f || value_n > 1.1f ||
        !isfinite(norm) || norm < 0.01f || norm > 0.64f)
    {
        return false;
    }
    for (value_i = 0; value_i < 3; ++value_i)
    {
        value_u[value_i] = p_a[value_i] / value_n;
        value_f[value_i] =
            (value_i == 0 ? 1.0f : 0.0f) - value_u[0] * value_u[value_i];
    }
    value_fn = sqrtf(value_f[0] * value_f[0] + value_f[1] * value_f[1] +
                     value_f[2] * value_f[2]);
    if (value_fn < 0.2f)
    {
        return false;
    }
    for (value_i = 0; value_i < 3; ++value_i)
    {
        value_f[value_i] /= value_fn;
    }
    left[0] = value_u[1] * value_f[2] - value_u[2] * value_f[1];
    left[1] = value_u[2] * value_f[0] - value_u[0] * value_f[2];
    left[2] = value_u[0] * value_f[1] - value_u[1] * value_f[0];
    value_x = p_m[0] * value_f[0] + p_m[1] * value_f[1] + p_m[2] * value_f[2];
    value_y = p_m[0] * left[0] + p_m[1] * left[1] + p_m[2] * left[2];
    if (value_x * value_x + value_y * value_y < 0.0025f)
    {
        return false;
    }
    *p_heading = -atan2f(value_y, value_x) * DEG;
    if (*p_heading < 0)
    {
        *p_heading += 360.0f;
    }
    return true;
}

/** @brief Apply one strict +/-180-degree correction to a heading difference.
 * @see tilt_estimation.h for supported heading domains and nonfinite inputs.
 */
float32_t tilt_angle_delta (float32_t now, float32_t previous)
{
    float32_t value_d = now - previous;
    if (value_d > 180.0f)
    {
        value_d -= 360.0f;
    }
    else if (value_d < -180.0f)
    {
        value_d += 360.0f;
    }
    return value_d;
}

/* End of file. */

