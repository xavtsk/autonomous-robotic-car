/*
 * File: speed_pid.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Motor speed PID control (one controller per wheel).
 */

#include "speed_pid.h"

#define DUTY_MAX                (1.0f)

void speed_pid_init(speed_pid_t *p_pid, float kp, float ki, float kd,
                    float kff, float accel_mps2)
{
    p_pid->kp = kp;
    p_pid->ki = ki;
    p_pid->kd = kd;
    p_pid->kff = kff;
    p_pid->accel_mps2 = accel_mps2;
    speed_pid_reset(p_pid);
}

void speed_pid_reset(speed_pid_t *p_pid)
{
    p_pid->setpoint_mps = 0.0f;
    p_pid->integral = 0.0f;
}

float speed_pid_update(speed_pid_t *p_pid, float target_mps,
                       float speed_mps, float accel_mps2, float dt_s)
{
    float step = p_pid->accel_mps2 * dt_s;
    float error;
    float ff;
    float deriv;
    float integral;
    float duty;

    /* Ramp the setpoint toward the target */
    if (target_mps > p_pid->setpoint_mps + step)
    {
        p_pid->setpoint_mps += step;
    }
    else if (target_mps < p_pid->setpoint_mps - step)
    {
        p_pid->setpoint_mps -= step;
    }
    else
    {
        p_pid->setpoint_mps = target_mps;
    }

    if ((p_pid->setpoint_mps == 0.0f) && (target_mps == 0.0f))
    {
        speed_pid_reset(p_pid);

        return 0.0f;
    }

    error    = p_pid->setpoint_mps - speed_mps;
    ff       = p_pid->kff * p_pid->setpoint_mps;
    deriv    = -p_pid->kd * accel_mps2;
    integral = p_pid->integral + (p_pid->ki * error * dt_s);
    duty     = ff + (p_pid->kp * error) + integral + deriv;

    /* Anti-windup: do not grow the integral while the output is saturated */
    if (((duty > DUTY_MAX) && (error > 0.0f))
        || ((duty < -DUTY_MAX) && (error < 0.0f)))
    {
        duty = ff + (p_pid->kp * error) + p_pid->integral + deriv;
    }
    else
    {
        p_pid->integral = integral;
    }

    if (duty > DUTY_MAX)
    {
        duty = DUTY_MAX;
    }
    if (duty < -DUTY_MAX)
    {
        duty = -DUTY_MAX;
    }

    return duty;
}

float speed_pid_setpoint(const speed_pid_t *p_pid)
{
    return p_pid->setpoint_mps;
}
