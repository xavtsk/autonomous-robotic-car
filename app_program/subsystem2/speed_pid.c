/*
 * File: speed_pid.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Motor speed PID control (one controller per wheel).
 */

#include "speed_pid.h"
#include "../app_types.h"

#define DUTY_MAX (1.0f)

void speed_pid_init (speed_pid_t * p_pid, float32_t value_kp, float32_t value_ki,
                     float32_t value_kd, float32_t kff, float32_t accel_mps2)
{
    p_pid->kp = value_kp;
    p_pid->ki = value_ki;
    p_pid->kd = value_kd;
    p_pid->kff = kff;
    p_pid->accel_mps2 = accel_mps2;
    speed_pid_reset(p_pid);
}

void speed_pid_reset (speed_pid_t * p_pid)
{
    p_pid->setpoint_mps = 0.0f;
    p_pid->integral = 0.0f;
}

float32_t speed_pid_update (speed_pid_t * p_pid, float32_t target_mps, float32_t speed_mps,
                        float32_t accel_mps2, float32_t dt_s)
{
    float32_t step = p_pid->accel_mps2 * dt_s;
    float32_t error;
    float32_t value_ff;
    float32_t deriv;
    float32_t integral;
    float32_t duty;

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

    error = p_pid->setpoint_mps - speed_mps;
    value_ff = p_pid->kff * p_pid->setpoint_mps;
    deriv = -p_pid->kd * accel_mps2;
    integral = p_pid->integral + (p_pid->ki * error * dt_s);
    duty = value_ff + (p_pid->kp * error) + integral + deriv;

    /* Anti-windup: do not grow the integral while the output is saturated */
    if (((duty > DUTY_MAX) && (error > 0.0f)) ||
        ((duty < -DUTY_MAX) && (error < 0.0f)))
    {
        duty = value_ff + (p_pid->kp * error) + p_pid->integral + deriv;
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

float32_t speed_pid_setpoint (const speed_pid_t * p_pid)
{
    return p_pid->setpoint_mps;
}
