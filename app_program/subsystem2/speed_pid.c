/** @file speed_pid.c
 * @brief Motor speed PID control (one controller per wheel).
 *
 * @par Ownership
 * Buddy 2 owns both wheel controllers.
 */

#include "speed_pid.h"
#include "../app_types.h"

#define DUTY_MAX (1.0f)

/** @brief Initialize one wheel controller with the supplied gains.
 * @see speed_pid.h for the units, validation and ownership contract.
 */
void speed_pid_init (speed_pid_t * p_pid, float32_t proportional_gain,
                     float32_t integral_gain, float32_t derivative_gain,
                     float32_t feedforward_gain, float32_t accel_mps2)
{
    p_pid->proportional_gain = proportional_gain;
    p_pid->integral_gain     = integral_gain;
    p_pid->derivative_gain   = derivative_gain;
    p_pid->feedforward_gain  = feedforward_gain;
    p_pid->accel_mps2        = accel_mps2;
    speed_pid_reset(p_pid);
}

/** @brief Clear the ramped setpoint and integral without changing gains.
 * @see speed_pid.h for the units, validation and ownership contract.
 */
void speed_pid_reset (speed_pid_t * p_pid)
{
    p_pid->setpoint_mps = 0.0f;
    p_pid->integral     = 0.0f;
}

/** @brief Advance the ramp and calculate one wheel's signed motor duty.
 * @see speed_pid.h for the units, validation and ownership contract.
 */
float32_t speed_pid_update (speed_pid_t * p_pid, float32_t target_mps,
                            float32_t speed_mps, float32_t accel_mps2,
                            float32_t dt_s)
{
    const float32_t step = p_pid->accel_mps2 * dt_s;

    /* Limit the setpoint change to one acceleration step. */
    if (target_mps > (p_pid->setpoint_mps + step))
    {
        p_pid->setpoint_mps += step;
    }
    else if (target_mps < (p_pid->setpoint_mps - step))
    {
        p_pid->setpoint_mps -= step;
    }
    else
    {
        p_pid->setpoint_mps = target_mps;
    }

    if ((0.0f == p_pid->setpoint_mps) && (0.0f == target_mps))
    {
        speed_pid_reset(p_pid);

        return 0.0f;
    }

    const float32_t error    = p_pid->setpoint_mps - speed_mps;
    const float32_t value_ff = p_pid->feedforward_gain * p_pid->setpoint_mps;
    const float32_t deriv    = -p_pid->derivative_gain * accel_mps2;
    float32_t       integral;
    integral = p_pid->integral + (p_pid->integral_gain * error * dt_s);
    float32_t duty =
        value_ff + (p_pid->proportional_gain * error) + integral + deriv;

    /* Hold the integral when it would drive the output further into saturation.
     */
    if (((duty > DUTY_MAX) && (error > 0.0f)) ||
        ((duty < -DUTY_MAX) && (error < 0.0f)))
    {
        duty = value_ff + (p_pid->proportional_gain * error) + p_pid->integral +
               deriv;
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

/** @brief Read the current ramped wheel-speed setpoint.
 * @see speed_pid.h for the units, validation and ownership contract.
 */
float32_t speed_pid_setpoint (const speed_pid_t * p_pid)
{
    return p_pid->setpoint_mps;
}

/* End of file. */

