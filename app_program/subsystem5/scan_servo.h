/** @file scan_servo.h
 * @brief Configure and position the ultrasonic scan servo through PWM.
 */
#ifndef BUDDY5_SCAN_SERVO_H
#define BUDDY5_SCAN_SERVO_H
#include "s5_config.h"
bool scan_servo_init (void);
bool scan_servo_position (unsigned degrees);
void scan_servo_disable (void);
#endif

/* End of file. */

