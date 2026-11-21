/** @file scan_servo.h
 * @brief Configure and position the ultrasonic scan servo through PWM.
 */
#ifndef BUDDY5_SCAN_SERVO_H
#define BUDDY5_SCAN_SERVO_H
#include "s5_config.h"
#include <stdbool.h>
/** @brief Configure GP12/PWM6A and enable the centred scan servo.
 * @return True if configuration and enabling succeed; false if hardware is
 * unconfirmed, pulse bounds are invalid, reset is incomplete or a driver fails.
 * @details With APP_S5 enabled, accepts pulse bounds within 500..2500 us and
 * minimum strictly below maximum. Deasserts PWM reset, selects the pin and
 * wrap 19999, sets the slice-6 divider for 1 us ticks, centres at 90 degrees,
 * then enables PWM. Failures can leave earlier writes applied; no rollback
 * occurs. APP_S5=0 returns false without hardware access.
 * @pre The obstacle task exclusively owns PWM6A/GP12; calls are serialized.
 * Other slices' configuration is retained. No mutex or resource claim is made.
 */
bool scan_servo_init (void);
/** @brief Write the pulse width for a requested servo direction.
 * @param[in] degrees Servo degrees, S5_MIN_ANGLE..S5_MAX_ANGLE inclusive
 * (30..150); 90 is forward. Larger angles point left before mount reversal.
 * @return True if the PWM compare write succeeds; false on invalid angle,
 * driver failure, or APP_S5=0. Invalid angles perform no driver write.
 * @details S5_SERVO_REVERSED maps the accepted angle to 180 minus degrees.
 * Pulse width is MIN + (MAX - MIN) * angle / 180, truncated to whole us.
 * This does not enable PWM, wait for settling or confirm physical position.
 * @pre Initialize successfully first and serialize access with init/disable;
 * readiness is a caller obligation and is not stored or checked here.
 */
bool scan_servo_position (unsigned degrees);
/** @brief Request PWM disable without releasing the servo's configuration.
 * @details With APP_S5 enabled, calls the PWM driver and ignores its result.
 * Otherwise does nothing. No lock, pin release or pulse reset is performed.
 * @pre Serialize with other servo operations under the obstacle task's owner.
 */
void scan_servo_disable (void);
#endif

/* End of file. */

