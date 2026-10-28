#include "scan_servo.h"
#if APP_S5
#include "../app.h"
#include <sys/sysdef.h>
/* 50 Hz, 1 us counter; configurable 1000..2000 us nominal servo mapping.
 * Only slice 6 is used; never reset all PWM while motors own slices 4/5. */
#ifndef S5_SERVO_MIN_US
#define S5_SERVO_MIN_US 1000U
#endif
#ifndef S5_SERVO_MAX_US
#define S5_SERVO_MAX_US 2000U
#endif
#if S5_SERVO_GPIO != 12U
#error "Buddy 5 currently claims GP12 / PWM6A; update the resource contract before moving it"
#endif
bool scan_servo_position(unsigned angle)
{
    if(angle<S5_MIN_ANGLE || angle>S5_MAX_ANGLE) { return false; }
    if(S5_SERVO_REVERSED) { angle=180U-angle; }
    return pwm_set_cc(S5_SERVO_GPIO,S5_SERVO_MIN_US+(S5_SERVO_MAX_US-S5_SERVO_MIN_US)*angle/180U)==E_OK;
}
bool scan_servo_init(void)
{
    if(!S5_HARDWARE_CONFIRMED || S5_SERVO_MIN_US<500U || S5_SERVO_MAX_US>2500U || S5_SERVO_MIN_US>=S5_SERVO_MAX_US) { return false; }
    clr_w(RESETS_RESET,RESETS_RESET_PWM);
    if((in_w(RESETS_RESET_DONE)&RESETS_RESET_PWM)==0U) { return false; }
    if(pwm_set_pin(S5_SERVO_GPIO)!=E_OK || pwm_set_wrap(S5_SERVO_GPIO,19999U)!=E_OK) { return false; }
    out_w(PWM_BASE+6U*0x14U+PWM_CHx_DIV,(uint32_t)SYSCLK<<4);
    if(!scan_servo_position(90U)) { return false; }
    return pwm_set_enabled(S5_SERVO_GPIO,TRUE)==E_OK;
}
void scan_servo_disable(void) { (void)pwm_set_enabled(S5_SERVO_GPIO,FALSE); }
#else
bool scan_servo_init(void) { return false; }
bool scan_servo_position(unsigned angle) { (void)angle;return false; }
void scan_servo_disable(void) { }
#endif
