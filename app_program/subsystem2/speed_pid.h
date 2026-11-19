/*
 * File: speed_pid.h
 * Owner: Buddy 2 - Motion Control
 * Purpose: Motor speed PID control (one controller per wheel).
 *
 * Output duty = feed-forward + P + I + D, limited to -1.0 .. +1.0.
 * The setpoint is ramped at a fixed acceleration so the wheels never get a
 * sudden step (stops wheel slip). D acts on the measured speed, so a change
 * of target does not cause a kick.
 */

#ifndef BUDDY2_SPEED_PID_H
#define BUDDY2_SPEED_PID_H

#include "../app_types.h"
typedef struct
{
    float32_t kp;           /* duty per m/s of speed error              */
    float32_t ki;           /* duty per m of accumulated speed error    */
    float32_t kd;           /* duty per m/s^2 of measured acceleration  */
    float32_t kff;          /* duty per m/s of setpoint (1 / top speed) */
    float32_t accel_mps2;   /* setpoint ramp rate                       */
    float32_t setpoint_mps; /* ramped setpoint                          */
    float32_t integral;     /* integral term, duty                      */
} speed_pid_t;

void speed_pid_init (speed_pid_t * p_pid, float32_t value_kp, float32_t value_ki,
                     float32_t value_kd, float32_t kff, float32_t accel_mps2);

/* Clear the setpoint and integral (used whenever the motor is braked) */
void speed_pid_reset (speed_pid_t * p_pid);

/* One control step. Returns the motor duty; 0.0 (brake) once both the
 * target and the ramped setpoint are zero. */
float32_t speed_pid_update (speed_pid_t * p_pid, float32_t target_mps, float32_t speed_mps,
                        float32_t accel_mps2, float32_t dt_s);

/* Current ramped setpoint, m/s */
float32_t speed_pid_setpoint (const speed_pid_t * p_pid);

#endif /* BUDDY2_SPEED_PID_H */
