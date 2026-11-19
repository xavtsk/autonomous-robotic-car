/** @file motion_estimation.h
 * @brief Encoder-based speed and distance estimation.
 */
/*
 * File: motion_estimation.h
 * Owner: Buddy 2 - Motion Control
 * Purpose: Encoder-based speed and distance estimation.
 *
 *   distance = encoder counts x metres per count
 *   speed    = change in distance over each control period, lightly smoothed
 *   heading  = (right distance - left distance) / track width
 *
 * Units: metres, metres per second, degrees. + = forward / anticlockwise.
 * Updated only by the motion task; the getters are safe from any task.
 */

#ifndef BUDDY2_MOTION_ESTIMATION_H
#define BUDDY2_MOTION_ESTIMATION_H

#include "../app_types.h"
#include "wheel_encoder.h" /* wheel_id_t */

/* m_per_count = pi x wheel diameter / encoder counts per wheel turn
 * (0 = not calibrated: all estimates read 0) */
/** @brief Reset the estimator using the current encoder counts.
 * @param m_per_count Metres per encoder count; zero represents no calibration.
 * @param track_width_m Wheel separation in metres, used by heading estimation.
 * @note Resets both stored wheel speeds and accelerations to zero. The motion
 * owner must serialize initialization with updates and calibration changes.
 */
void motion_estimation_init (float32_t m_per_count, float32_t track_width_m);
/** @brief Replace the metres-per-count scale without resetting history.
 * @param m_per_count New encoder distance scale in metres per count.
 * @note Stores the value without validation. Retained filtered speeds and
 * accelerations are unchanged until later updates by the motion owner.
 */
void motion_estimation_set_scale (float32_t m_per_count);
/** @brief Read the currently stored encoder distance scale.
 * @return Metres per count, including zero before calibration.
 * @note Performs one volatile read and does not acquire a lock.
 */
float32_t motion_estimation_scale (void);

/* Read the encoders and update the speeds; once per control period */
/** @brief Update filtered wheel speed and acceleration from encoder deltas.
 * @param dt_s Positive, finite elapsed control time in seconds.
 * @pre Initialize the estimator and supply a nonzero dt_s; no validation is
 * performed here. Only the motion owner may update the retained history.
 * @note Uses modular encoder deltas, a 0.4 speed filter coefficient, and the
 * change in filtered speed divided by dt_s for acceleration. No lock is taken.
 */
void motion_estimation_update (float32_t dt_s);

/** @brief Read one wheel's retained filtered speed.
 * @param wheel Valid WHEEL_LEFT or WHEEL_RIGHT index; it is not range-checked.
 * @return Signed wheel speed in metres per second, positive forward.
 * @note Reads the live stored value without locking or taking a pair snapshot.
 */
float32_t motion_estimation_wheel_speed (wheel_id_t wheel); /* m/s   */
/** @brief Read one wheel's retained acceleration estimate.
 * @param wheel Valid WHEEL_LEFT or WHEEL_RIGHT index; it is not range-checked.
 * @return Change in filtered speed per second, in metres per second squared.
 * @note Reads the live stored value without locking.
 */
float32_t motion_estimation_wheel_accel (wheel_id_t wheel); /* m/s^2 */
/** @brief Convert one wheel's current signed encoder count to distance.
 * @param wheel Valid WHEEL_LEFT or WHEEL_RIGHT encoder index.
 * @return Signed count multiplied by the current metres-per-count scale.
 * @note The count and scale are separate reads; no coherent snapshot is taken.
 */
float32_t motion_estimation_wheel_dist (wheel_id_t wheel);  /* m     */
/** @brief Average the two retained wheel-speed estimates.
 * @return Signed mean wheel speed in metres per second.
 * @note Reads each wheel separately without locking against an update.
 */
float32_t motion_estimation_robot_speed (void);             /* m/s   */
/** @brief Average the two current encoder-derived wheel distances.
 * @return Signed centre distance in metres under the current encoder scale.
 * @note Reads both wheel distances separately without taking a snapshot.
 */
float32_t motion_estimation_robot_dist (void);              /* m     */
/** @brief Calculate heading from the wheel-distance difference.
 * @return Unwrapped degrees, positive anticlockwise; zero when the configured
 * track width is nonpositive. Otherwise no finite-value validation is done.
 * @note Reads wheel distances separately and divides right minus left distance
 * by track width. Callers needing a coherent sample must serialize the reads.
 */
float32_t motion_estimation_heading_deg (void);             /* deg   */

#endif /* BUDDY2_MOTION_ESTIMATION_H */

/* End of file. */

