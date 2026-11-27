/** @file hardware_main.c
 * @brief Exercise ultrasonic and servo driver logic through fake hardware.
 * The selected narrow integer operands fit in int.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "hardware_main.h"
#include "../../app_program/subsystem5/scan_servo.h"
#include "../../app_program/subsystem5/ultrasonic_sensor.h"

static uint32_t g_timer;
static uint32_t g_control   = 3U;
static uint32_t g_fifo      = 1000U;
static bool     gb_is_empty = true;
static bool     gb_is_echo_high;
static unsigned g_program_writes;
static unsigned g_triggers;
static unsigned g_compare_register;
static unsigned g_wrap;
static bool     gb_is_pwm_on;

/** @brief Return simulated RP2040 timer, reset and PIO register values,
 * including the configured FIFO word or empty flag.
 */
uint32_t fake_read (uint32_t address)
{
    if (TIMER_TIMERAWL == address)
    {
        return g_timer++;
    }

    if (RESETS_RESET == address)
    {
        return 0;
    }

    if (RESETS_RESET_DONE == address)
    {
        return RESETS_RESET_PWM | RESETS_RESET_PIO0;
    }

    if (0x50200000U == address)
    {
        return g_control;
    }

    if (0x50200004U == address)
    {
        return gb_is_empty ? 1U << 10 : 0U;
    }

    if (0x50200028U == address)
    {
        return g_fifo;
    }

    return 0;
}

/** @brief Apply the simulated PIO control write and check that ultrasonic
 * program writes stay within the permitted region.
 */
void fake_write (uint32_t address, uint32_t value)
{
    if (0x50202000U == address)
    {
        g_control |= value & 15U;
    }

    if (0x50203000U == address)
    {
        g_control &= ~value;
    }

    if ((address >= 0x50200048U) && (address < 0x502000c8U))
    {
        assert(address >= 0x502000a8U);
        g_program_writes++;
    }

    assert((g_control & 3U) == 3U); /* Encoder SM0 and SM1 remain running. */
}

/** @brief Check that the trigger pin is configured for output or the echo
 * pin for input.
 */
int gpio_set_pin (unsigned pin, unsigned mode)
{
    assert((1U == pin && GPIO_MODE_OUT == mode) ||
           (0U == pin && GPIO_MODE_IN == mode));
    return 0;
}

/** @brief Require the trigger pin and count requests to drive it high.
 */
int gpio_set_val (unsigned pin, unsigned value)
{
    assert(1U == pin);

    if (value)
    {
        g_triggers++;
    }

    return 0;
}

/** @brief Return the simulated echo level for the expected input pin.
 */
unsigned gpio_get_val (unsigned pin)
{
    assert(0U == pin);
    return gb_is_echo_high;
}

/** @brief Check that the ultrasonic servo selects PWM pin 12.
 */
int pwm_set_pin (unsigned pin)
{
    assert(12U == pin);
    return 0;
}

/** @brief Record the servo PWM wrap count on the expected pin.
 */
int pwm_set_wrap (unsigned pin, uint32_t value)
{
    assert(12U == pin);
    g_wrap = value;
    return 0;
}

/** @brief Record the servo PWM compare count on the expected pin.
 */
int pwm_set_cc (unsigned pin, uint32_t value)
{
    assert(12U == pin);
    g_compare_register = value;
    return 0;
}

/** @brief Record the servo PWM enable state on the expected pin.
 */
int pwm_set_enabled (unsigned pin, bool b_is_enabled)
{
    assert(12U == pin);
    gb_is_pwm_on = b_is_enabled;
    return 0;
}

/** @brief Simulate echo timing and servo register writes and check range
 * classification, servo limits and state-machine ownership.
 */
int main (void)
{
    uint16_t millimetres;
    bool     b_is_valid;
    assert(ultrasonic_sensor_init() && 8U == g_program_writes);
    assert(ultrasonic_sensor_trigger(0) && 1U == g_triggers &&
           (g_control & 4U));
    assert(!ultrasonic_sensor_poll(100, &millimetres, &b_is_valid));
    gb_is_empty = false;
    assert(ultrasonic_sensor_poll(3000, &millimetres, &b_is_valid) &&
           b_is_valid && 343 == (int)millimetres && !(g_control & 4U));
    gb_is_empty = true;
    assert(ultrasonic_sensor_trigger(70000));
    assert(ultrasonic_sensor_poll(95000, &millimetres, &b_is_valid) &&
           !b_is_valid);
    gb_is_echo_high = true;
    assert(!ultrasonic_sensor_trigger(140000));
    gb_is_echo_high = false;
    /* A normal long pulse and a wire stuck HIGH after a valid trigger give
     * the same first 4,000 mm result. HIGH blocks the following trigger. */
    assert(ultrasonic_sensor_trigger(210000));
    gb_is_echo_high = true;
    assert(!ultrasonic_sensor_poll(220000, &millimetres, &b_is_valid));
    assert(ultrasonic_sensor_poll(236000, &millimetres, &b_is_valid) &&
           b_is_valid && S5_RANGE_MAX_MM == millimetres && !(g_control & 4U));
    assert(!ultrasonic_sensor_trigger(280000));
    gb_is_echo_high = false;
    /* Completed pulse longer than the range window is also clear. */
    assert(ultrasonic_sensor_trigger(280000));
    gb_is_empty = false;
    g_fifo      = 13000U;
    assert(ultrasonic_sensor_poll(283000, &millimetres, &b_is_valid) &&
           b_is_valid && S5_RANGE_MAX_MM == millimetres);
    /* Too-short pulse (<30 mm) stays invalid. */
    gb_is_empty = true;
    assert(ultrasonic_sensor_trigger(350000));
    gb_is_empty = false;
    g_fifo      = 5U;
    assert(ultrasonic_sensor_poll(353000, &millimetres, &b_is_valid) &&
           !b_is_valid && 0 == (int)millimetres);
    gb_is_empty = true;
    g_fifo      = 1000U;
    assert(scan_servo_init() && gb_is_pwm_on && 19999U == g_wrap &&
           1500U == g_compare_register);
    assert(!scan_servo_position(0) && !scan_servo_position(180));
    assert(scan_servo_position(30) && 1166U == g_compare_register);
    scan_servo_disable();
    assert(!gb_is_pwm_on);
    puts("PASS: production PIO/PWM drivers with fake registers; confirmed "
         "pins, echo/timeout/stuck-high, clear-path vs no-pulse, servo "
         "limits, encoder SM ownership");
}

/* End of file. */

