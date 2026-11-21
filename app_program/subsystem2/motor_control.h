/** @file motor_control.h
 * @brief Motor actuation through the Robo Pico motor driver (PWM).
 *
 * @par Ownership
 * Buddy 2 - Motion Control
 *
 * The course reference "Robo Pico Board Datasheet.pdf" supplies the driver
 * truth table. A low + B low = brake (the safe
 * output state), A PWM + B low = forward, A low + B PWM = backward.
 * Only the motion task may call these functions (single PWM owner).
 */

#ifndef BUDDY2_MOTOR_CONTROL_H
#define BUDDY2_MOTOR_CONTROL_H

#include "../app_types.h"
#include <stdbool.h>
#include "wheel_encoder.h" /* wheel_encoder_id_t */

/* Set up both motor channels at 10 kHz and apply the brakes. */
void motor_control_init (bool b_is_left_motor_inverted,
                         bool b_is_right_motor_inverted);

/* Accept duty from full reverse (-1) to full forward (+1). Zero brakes. */
void motor_control_set (wheel_encoder_id_t wheel, float32_t duty);

/* Brake both motors by clearing both channel inputs. */
void motor_control_brake_all (void);

/* Return the last duty before applying hardware polarity. */
float32_t motor_control_duty (wheel_encoder_id_t wheel);

#endif /* BUDDY2_MOTOR_CONTROL_H */

/* End of file. */

