/** @file wheel_smoke_main.h
 * @brief Motion interface used by the smoke runner fixture.
 *
 * The fixture drives button and clock inputs and observes the smoke runner
 * through the motion request and polling API.
 */
#ifndef TESTING_DIAGNOSTICS_WHEEL_SMOKE_MAIN_H
#define TESTING_DIAGNOSTICS_WHEEL_SMOKE_MAIN_H

#include <stdint.h>

#include "../../app_program/subsystem2/motion.h"

#define E_OK 0
#define TSK_SELF 0
#define GPIO_MODE_IN 0
#define GPIO(pin) (pin)
#define GPIO_PDE 1U
#define GPIO_PUE 2U

/** @brief Accept a GPIO pull-down clear request in the button fixture. */
static inline void clr_w (uint32_t address, uint32_t bits)
{
    (void)address;
    (void)bits;
}

/** @brief Accept a GPIO pull-up set request in the button fixture. */
static inline void set_w (uint32_t address, uint32_t bits)
{
    (void)address;
    (void)bits;
}

typedef int     ER;
typedef int     INT;
typedef uint8_t UB;

typedef struct
{
    int32_t  hi;
    uint32_t lo;
} SYSTIM;

ER       tk_chg_pri (INT task, INT priority);
ER       gpio_set_pin (unsigned pin, unsigned mode);
ER       tk_get_otm (SYSTIM * p_value);
ER       tk_dly_tsk (unsigned delay);
unsigned gpio_get_val (unsigned pin);
INT      tm_printf (const UB * p_format, ...);

#endif /* TESTING_DIAGNOSTICS_WHEEL_SMOKE_MAIN_H */

/* End of file. */

