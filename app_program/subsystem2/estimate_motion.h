/** @file estimate_motion.h
 * @brief Encoder-based speed and distance estimation.
 */
/*
 * File: estimate_motion.h
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
void motion_estimation_init (float32_t m_per_count, float32_t track_width_m);
void motion_estimation_set_scale (float32_t m_per_count);
float32_t motion_estimation_scale (void);

/* Read the encoders and update the speeds; once per control period */
void motion_estimation_update (float32_t dt_s);

float32_t motion_estimation_wheel_speed (wheel_id_t wheel); /* m/s   */
float32_t motion_estimation_wheel_accel (wheel_id_t wheel); /* m/s^2 */
float32_t motion_estimation_wheel_dist (wheel_id_t wheel);  /* m     */
float32_t motion_estimation_robot_speed (void);             /* m/s   */
float32_t motion_estimation_robot_dist (void);              /* m     */
float32_t motion_estimation_heading_deg (void);             /* deg   */

#endif /* BUDDY2_MOTION_ESTIMATION_H */

/* End of file. */

