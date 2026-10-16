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

#include "wheel_encoder.h"      /* wheel_id_t */

/* m_per_count = pi x wheel diameter / encoder counts per wheel turn
 * (0 = not calibrated: all estimates read 0) */
void  motion_estimation_init(float m_per_count, float track_width_m);
void  motion_estimation_set_scale(float m_per_count);
float motion_estimation_scale(void);

/* Read the encoders and update the speeds; once per control period */
void  motion_estimation_update(float dt_s);

float motion_estimation_wheel_speed(wheel_id_t wheel);      /* m/s   */
float motion_estimation_wheel_accel(wheel_id_t wheel);      /* m/s^2 */
float motion_estimation_wheel_distance(wheel_id_t wheel);   /* m     */
float motion_estimation_robot_speed(void);                  /* m/s   */
float motion_estimation_robot_distance(void);               /* m     */
float motion_estimation_robot_heading_deg(void);            /* deg   */

#endif /* BUDDY2_MOTION_ESTIMATION_H */
