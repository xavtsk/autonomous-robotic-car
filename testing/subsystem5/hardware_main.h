/** @file hardware_main.h
 * @brief Register access interface for the ultrasonic and servo fixture.
 *
 * The fixture implements fake_read and fake_write. The fake register header
 * selects the timer, PIO and PWM addresses used by the drivers.
 */
#ifndef TESTING_SUBSYSTEM5_HARDWARE_MAIN_H
#define TESTING_SUBSYSTEM5_HARDWARE_MAIN_H

#include <stdbool.h>
#include "sys/sysdef.h"

#define E_OK 0
#define TRUE 1
#define FALSE 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_IN 0

int      gpio_set_pin (unsigned pin, unsigned mode);
int      gpio_set_val (unsigned pin, unsigned value);
unsigned gpio_get_val (unsigned pin);
int      pwm_set_pin (unsigned pin);
int      pwm_set_wrap (unsigned pin, uint32_t value);
int      pwm_set_cc (unsigned pin, uint32_t value);
int      pwm_set_enabled (unsigned pin, bool b_is_enabled);

#endif /* TESTING_SUBSYSTEM5_HARDWARE_MAIN_H */

/* End of file. */

