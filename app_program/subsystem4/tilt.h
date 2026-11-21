/** @file tilt.h
 * @brief Estimate tilt and heading and calculate wrapped angle differences.
 */
#ifndef BUDDY4_TILT_ESTIMATION_H
#define BUDDY4_TILT_ESTIMATION_H
#include "../app_types.h"
#include "imu_sensor.h"
/** @brief Estimate pitch and roll from a near-1 g acceleration vector.
 * @param[in] p_accel Stable three-element acceleration in g, ordered vehicle
 * X forward, Y left, Z up; use bias-corrected values for calibrated tilt.
 * @param[out] p_pitch_deg Pitch in degrees, positive nose up.
 * @param[out] p_roll_deg Roll in degrees, positive right side up.
 * @return False, leaving both outputs unchanged, if the squared norm is
 * nonfinite, below 0.81 or above 1.21, or Z is zero/negative. Otherwise true.
 * @pre All pointers are non-null; output locations are distinct and do not
 * overlap the input. Pointers are not checked or retained; no lock is taken.
 * @details Rejection includes NaN/infinite inputs and overflowing squared
 * norms. The exact floating comparisons are inclusive at the norm limits,
 * with no tolerance. atan2f/sqrtf determine precision and signed-zero results;
 * accepted vectors produce nominal pitch/roll between -90 and +90 degrees.
 */
bool tilt_estimation (const float32_t p_accel[3], float32_t * p_pitch_deg,
                      float32_t * p_roll_deg);
/** @brief Estimate magnetic heading in the plane normal to acceleration.
 * @param[in] p_accel Stable acceleration in g on vehicle X-forward/Y-left/Z-up.
 * @param[in] p_mag Stable magnetic field in gauss on the same axes; calibration
 * offsets/scales and any sample-validity checks are the caller's work.
 * @param[out] p_heading_deg Caller-owned heading in degrees, increasing
 * anticlockwise; unchanged on failure.
 * @return False if acceleration norm is nonfinite or outside 0.9..1.1 g,
 * magnetic squared norm is nonfinite or outside 0.01..0.64 gauss squared,
 * projected forward length is below 0.2, or horizontal magnetic squared
 * magnitude is below 0.0025. Otherwise true after assigning the heading.
 * @pre Non-null three-element input arrays and writable output; inputs stay
 * stable during the call. No pointer is retained and no locking occurs.
 * @details NaN/infinite inputs are rejected by the norm checks. There is no
 * positive-Z check in this function. Heading uses -atan2f and adds 360 only
 * for a strictly negative result. Its nominal range is one revolution;
 * signed zero and a value rounded to 360 are not normalized further.
 */
bool tilt_heading (const float32_t p_accel[3], const float32_t p_mag[3],
                   float32_t * p_heading_deg);
/** @brief Subtract two headings and apply at most one revolution correction.
 * @param[in] now Current heading in degrees.
 * @param[in] previous Previous heading in degrees.
 * @return now minus previous, subtracting 360 when above +180 or adding 360
 * when below -180. Exact +180/-180 remain unchanged.
 * @details For headings in one common 0..360 revolution, this is the signed
 * shortest delta. Arbitrary multi-turn inputs are not fully normalized.
 * There is no finite-value check or epsilon: NaNs/infinities propagate through
 * the arithmetic, and zero retains the subtraction's floating-point sign.
 */
float32_t tilt_angle_delta (float32_t now, float32_t previous);
#endif

/* End of file. */

