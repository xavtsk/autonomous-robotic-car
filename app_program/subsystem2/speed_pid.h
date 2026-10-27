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

typedef struct
{
    float kp;               /* duty per m/s of speed error              */
    float ki;               /* duty per m of accumulated speed error    */
    float kd;               /* duty per m/s^2 of measured acceleration  */
    float kff;              /* duty per m/s of setpoint (1 / top speed) */
    float accel_mps2;       /* setpoint ramp rate                       */
    float setpoint_mps;     /* ramped setpoint                          */
    float integral;         /* integral term, duty                      */
} speed_pid_t;

void  speed_pid_init(speed_pid_t *p_pid, float kp, float ki, float kd,
                     float kff, float accel_mps2);

/* Clear the setpoint and integral (used whenever the motor is braked) */
void  speed_pid_reset(speed_pid_t *p_pid);

/* One control step. Returns the motor duty; 0.0 (brake) once both the
 * target and the ramped setpoint are zero. */
float speed_pid_update(speed_pid_t *p_pid, float target_mps,
                       float speed_mps, float accel_mps2, float dt_s);

/* Current ramped setpoint, m/s */
float speed_pid_setpoint(const speed_pid_t *p_pid);

#endif /* BUDDY2_SPEED_PID_H */
