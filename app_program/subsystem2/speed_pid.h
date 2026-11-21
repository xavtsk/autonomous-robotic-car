/** @file speed_pid.h
 * @brief Motor speed PID control (one controller per wheel).
 *
 * @par Ownership
 * Buddy 2 - Motion Control
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
    float32_t proportional_gain; /* Scale proportional duty per m/s of error. */
    float32_t integral_gain;     /* Scale integral duty per metre of error. */
    float32_t derivative_gain;   /* Scale derivative duty per m/s^2. */
    float32_t feedforward_gain; /* Scale feedforward by reciprocal top speed. */
    float32_t accel_mps2;       /* Limit the setpoint's rate of change. */
    float32_t setpoint_mps;     /* Retain the current ramped setpoint. */
    float32_t integral;         /* Retain the integral term as duty. */
} speed_pid_t;

/** @brief Initialize one wheel controller with the supplied gains.
 * @param p_pid Non-null writable controller, exclusively owned by the caller.
 * @param proportional_gain Proportional gain in duty per metre-per-second
 * error.
 * @param integral_gain Integral gain in duty per metre of accumulated speed
 * error.
 * @param derivative_gain Gain in duty per metre-per-second-squared
 * acceleration.
 * @param feedforward_gain Feed-forward duty per metre-per-second setpoint.
 * @param accel_mps2 Setpoint ramp rate in metres per second squared.
 * @post Stores every gain verbatim and clears setpoint and integral to zero.
 * @note No range/finite checks or locking; the motion controller calls this
 * under its ownership boundary, with a separate instance for each wheel.
 */
void speed_pid_init (speed_pid_t * p_pid, float32_t proportional_gain,
                     float32_t integral_gain, float32_t derivative_gain,
                     float32_t feedforward_gain, float32_t accel_mps2);

/* Clear the setpoint and integral when applying the brake. */
/** @brief Clear the ramped setpoint and integral without changing gains.
 * @param p_pid Non-null writable controller, exclusively owned by the caller.
 * @post Setpoint and integral become positive zero; all tuning fields remain.
 * @note The caller owns the separate hardware brake operation.
 */
void speed_pid_reset (speed_pid_t * p_pid);

/* One control step. Returns the motor duty; 0.0 (brake) once both the
 * target and the ramped setpoint are zero. */
/** @brief Advance the ramp and calculate one wheel's signed motor duty.
 * @param p_pid Non-null initialized controller; caller serializes its access.
 * @param target_mps Target wheel speed in metres per second, forward positive.
 * @param speed_mps Measured wheel speed in metres per second, forward
 * positive.
 * @param accel_mps2 Measured wheel acceleration in metres per second squared.
 * @param dt_s Elapsed seconds, used as supplied; normally positive and finite.
 * @return Signed duty, with comparable results clipped to -1 through +1.
 * NaN is not rejected and can propagate. Exact zero target and ramped setpoint
 * clear the integral and return positive zero, including signed-zero inputs.
 * @post For finite target/setpoint and a nonnegative finite ramp step,
 * advances setpoint by at most ramp-rate times dt_s. Adds the integral
 * increment unless saturation and error have the same sign; in that case
 * the previous integral is retained.
 * @note Derivative action is negative gain times measured acceleration.
 * No timing, sign or finite-value validation, locks, allocation or hardware
 * access occurs here. The surrounding motion controller owns scheduling.
 */
float32_t speed_pid_update (speed_pid_t * p_pid, float32_t target_mps,
                            float32_t speed_mps, float32_t accel_mps2,
                            float32_t dt_s);

/* Return the current ramped setpoint in m/s. */
/** @brief Read the current ramped wheel-speed setpoint.
 * @param p_pid Non-null readable controller with caller-synchronized access.
 * @return Stored setpoint in metres per second, forward positive.
 * @note Does not alter state or obtain a coherent snapshot by itself.
 */
float32_t speed_pid_setpoint (const speed_pid_t * p_pid);

#endif /* BUDDY2_SPEED_PID_H */

/* End of file. */

