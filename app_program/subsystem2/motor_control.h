/*
 * File: motor_control.h
 * Owner: Buddy 2 - Motion Control
 * Purpose: Motor actuation through the Robo Pico motor driver (PWM).
 *
 * Driver truth table (Robo Pico datasheet): A low + B low = brake (the safe
 * output state), A PWM + B low = forward, A low + B PWM = backward.
 * Only the motion task may call these functions (single PWM owner).
 */

#ifndef BUDDY2_MOTOR_CONTROL_H
#define BUDDY2_MOTOR_CONTROL_H

#include <stdbool.h>
#include "wheel_encoder.h"      /* wheel_id_t */

/* Set up M1A/M1B (left) and M2A/M2B (right) for 10 kHz PWM, both braked */
void  motor_control_init(bool b_m1_inverted, bool b_m2_inverted);

/* duty: -1.0 (full backward) .. 0.0 (brake) .. +1.0 (full forward) */
void  motor_control_set(wheel_id_t wheel, float duty);

/* Safe output state: both driver inputs low (brake) on both motors */
void  motor_control_brake_all(void);

/* Last duty sent to a motor (after inversion is undone), for diagnostics */
float motor_control_duty(wheel_id_t wheel);

#endif /* BUDDY2_MOTOR_CONTROL_H */
