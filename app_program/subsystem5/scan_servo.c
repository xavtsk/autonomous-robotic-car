/** @file scan_servo.c
 * @brief Configure and position the ultrasonic scan servo through PWM.
 * The hardware masks fit in unsigned. Convert them before combining
 * register or packet bits.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 */
#include "scan_servo.h"
#if APP_S5
#include "../app.h"
#include <sys/sysdef.h>
#include <stdint.h>
/* The servo counter uses one-microsecond ticks at 50 Hz. Its default
 * pulse map spans 1000..2000 us. Configure only slice 6 because the motor
 * driver owns PWM slices 4 and 5. */
#ifndef S5_SERVO_MIN_US
#define S5_SERVO_MIN_US 1000U
#endif
#ifndef S5_SERVO_MAX_US
#define S5_SERVO_MAX_US 2000U
#endif
#if S5_SERVO_GPIO != 12U
#error "Buddy 5 claims GP12/PWM6A; update the resource contract before moving."
#endif
/** @brief Validate and map an angle before writing its PWM compare value.
 * @see scan_servo.h for units, reversal, rounding and ownership.
 */
bool scan_servo_position (unsigned degrees)
{
    if ((degrees < S5_MIN_ANGLE) || (degrees > S5_MAX_ANGLE))
    {
        return false;
    }

    if (S5_SERVO_REVERSED)
    {
        degrees = 180U - degrees;
    }

    return pwm_set_cc(S5_SERVO_GPIO,
                      S5_SERVO_MIN_US +
                          (((S5_SERVO_MAX_US - S5_SERVO_MIN_US) * degrees) /
                           180U)) == E_OK;
}

/** @brief Configure only the owned PWM slice and enable its centred output.
 * @see scan_servo.h for ordered setup and partial-failure effects.
 */
bool scan_servo_init (void)
{
    if ((((!S5_HARDWARE_CONFIRMED) || (S5_SERVO_MIN_US < 500U)) ||
         (S5_SERVO_MAX_US > 2500U)) ||
        (S5_SERVO_MIN_US >= S5_SERVO_MAX_US))
    {
        return false;
    }

    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    if ((in_w(RESETS_RESET_DONE) & (unsigned)RESETS_RESET_PWM) == 0U)
    {
        return false;
    }

    if ((pwm_set_pin(S5_SERVO_GPIO) != E_OK) ||
        (pwm_set_wrap(S5_SERVO_GPIO, 19999U) != E_OK))
    {
        return false;
    }
    /* SYSCLK is 125 MHz on this target. Shift the unsigned value into the
     * divider's integer field, which leaves its four fraction bits zero.
     * The RP2040 peripheral base and offset constants fit in unsigned int.
     * See the PWM chapter of the RP2040 Datasheet for the register map. */
    out_w((unsigned int)PWM_BASE + (6U * 0x14U) + (unsigned int)PWM_CHx_DIV,
          (uint32_t)SYSCLK << 4);

    if (!scan_servo_position(90U))
    {
        return false;
    }

    return pwm_set_enabled(S5_SERVO_GPIO, TRUE) == E_OK;
}

/** @brief Request disable of the owned slice, ignoring the driver result.
 * @see scan_servo.h for retained configuration and serialization.
 */
void scan_servo_disable (void)
{
    (void)pwm_set_enabled(S5_SERVO_GPIO, FALSE);
}
#else
/** @brief Report that the servo is unavailable in an APP_S5=0 build.
 * @return Always false; no hardware or state is changed.
 */
bool scan_servo_init (void)
{
    return false;
}

/** @brief Reject every positioning request in an APP_S5=0 build.
 * @param[in] angle Ignored servo angle in degrees.
 * @return Always false; no hardware or state is changed.
 */
bool scan_servo_position (unsigned angle)
{
    (void)angle;
    return false;
}

/** @brief Do nothing when the servo subsystem is excluded from the build. */
void scan_servo_disable (void)
{
}
#endif

/* End of file. */

