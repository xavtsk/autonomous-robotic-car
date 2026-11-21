/** @file ultrasonic_sensor.c
 * @brief Trigger and poll PIO ultrasonic echo capture and convert pulses to
 * range.
 * The hardware masks fit in unsigned. Convert them before combining
 * register or packet bits.
 * @see RP2040 Datasheet, section 3.7, for the PIO register map.
 * @see ultrasonic_echo.pio for the echo timing instruction sequence.
 * @see app_program/subsystem5/README.md for the HC-SR04 datasheet and wiring.
 */
#include "ultrasonic_sensor.h"

#if APP_S5
#include "../app.h"
#include <sys/sysdef.h>
#define S5_PIO 0x50200000U
#define S5_SM (S5_PIO + 0x0C8U + 2U * 0x18U)
#define S5_ENABLE (1U << 2)
#define S5_TRIGGER_PULSE_US (12U)
/* PIO0 state machine 2 uses instruction slots 24 through 31. It runs at
 * 1 MHz, and each count takes two instructions in ultrasonic_echo.pio.
 * Its instruction slots and state machine are separate from the encoders. */
static const uint16_t g_program[8] = {0x20A0, 0xA02B, 0x00DC, 0x001E,
                                      0x005A, 0x001E, 0xA0C9, 0x8020};
static uint32_t       g_started;
static bool           gb_is_echo_pending;
static bool           gb_is_ready;
#if S5_TRIG_GPIO != 1U || S5_ECHO_GPIO != 0U
#error "Buddy 5 claims GP1 TRIG/GP0 ECHO; update resource claims before moving."
#endif
#endif

/** @brief Convert a bounded pulse, leaving output unchanged on rejection.
 * @see ultrasonic_sensor.h for units, rounding and inclusive range limits.
 */
bool ultrasonic_sensor_pulse_mm (uint32_t pulse_us, uint16_t * p_mm)
{

    if ((0 == p_mm) || (pulse_us > 24000U))
    {
        return false;
    }
    /* Treat the sound speed as 343 m/s. Divide the round trip by two
     * and round the one-way distance to whole millimetres. */
    const uint32_t range_mm = ((pulse_us * 343U) + 1000U) / 2000U;

    if ((range_mm < S5_RANGE_MIN_MM) || (range_mm > S5_RANGE_MAX_MM))
    {
        return false;
    }
    /* The accepted echo range 30..4000 mm fits in the output field. */
    *p_mm = (uint16_t)range_mm;
    return true;
}

/** @brief Distinguish usable, clear over-range and invalid too-short echoes.
 * @see ultrasonic_sensor.h for output writes on each return path.
 */
bool ultrasonic_sensor_echo_mm (uint32_t pulse_us, uint16_t * p_mm)
{
    if (0 == p_mm)
    {
        return false;
    }

    if (ultrasonic_sensor_pulse_mm(pulse_us, p_mm))
    {
        return true;
    }
    /* Nothing reflected within range: report clear, not a fault. */
    if ((pulse_us > 24000U) ||
        ((((pulse_us * 343U) + 1000U) / 2000U) > S5_RANGE_MAX_MM))
    {
        *p_mm = S5_RANGE_MAX_MM;
        return true;
    }

    *p_mm = 0;
    return false;
}

#if APP_S5
/** @brief Clear only SM2's enable bit and the active-capture flag.
 * @see ultrasonic_sensor.h for ownership and retained module state.
 */
void ultrasonic_sensor_cancel (void)
{
    out_w(S5_PIO + 0x3000U, S5_ENABLE);
    gb_is_echo_pending = false;
}

/** @brief Configure the reserved PIO program, state machine and GPIO pins.
 * @see ultrasonic_sensor.h for resource prerequisites and failure effects.
 */
bool ultrasonic_sensor_init (void)
{
    if (((!S5_HARDWARE_CONFIRMED) ||
         (in_w(RESETS_RESET) & (unsigned)RESETS_RESET_PIO0)) ||
        (in_w(S5_PIO) & S5_ENABLE))
    {
        return false;
    }

    if ((gpio_set_pin(S5_TRIG_GPIO, GPIO_MODE_OUT) != E_OK) ||
        (gpio_set_pin(S5_ECHO_GPIO, GPIO_MODE_IN) != E_OK))
    {
        return false;
    }

    (void)gpio_set_val(S5_TRIG_GPIO, 0U);

    for (unsigned word_index = 0;
         word_index < (sizeof g_program / sizeof g_program[0]); word_index++)
    {
        out_w(S5_PIO + 0x48U + (4U * (24U + word_index)),
              g_program[word_index]);
    }

    /* SYSCLK is 125 MHz on this target. Convert it to uint32_t so the
     * divider shift is unsigned and leaves the low 16 bits zero. */
    out_w(S5_SM, (uint32_t)SYSCLK << 16);
    out_w(S5_SM + 4U, (S5_ECHO_GPIO << 24) | (31U << 12) | (24U << 7));
    out_w(S5_SM + 8U, 0U);
    out_w(S5_SM + 20U, S5_ECHO_GPIO << 15);
    gb_is_ready        = true;
    gb_is_echo_pending = false;
    return true;
}

/** @brief Restart SM2, mark the capture active and emit the trigger pulse.
 * @see ultrasonic_sensor.h for timestamp units, rejection and blocking time.
 */
bool ultrasonic_sensor_trigger (uint32_t now)
{
    if (((!gb_is_ready) || gb_is_echo_pending) || (gpio_get_val(S5_ECHO_GPIO)))
    {
        return false;
    }

    ultrasonic_sensor_cancel();
    /* Toggle FIFO join to clear SM2 FIFOs, restart SM2 only. Atomic aliases
     * avoid overwriting Buddy 2's enable bits on the same PIO block. */
    out_w(S5_SM + 8U, 1UL << 31);
    out_w(S5_SM + 8U, 0U);
    out_w(S5_PIO + 0x2000U, (1U << 6) | (1U << 10));
    out_w(S5_SM + 16U, 24U); /* Inject a jump to instruction 24. */
    out_w(S5_PIO + 0x2000U, S5_ENABLE);
    g_started          = now;
    gb_is_echo_pending = true;
    (void)gpio_set_val(S5_TRIG_GPIO, 1U);
    const uint32_t trigger_us = in_w(TIMER_TIMERAWL);
    /* Keep elapsed ticks modulo 2^32 when the raw timer wraps. */
    while ((in_w(TIMER_TIMERAWL) - trigger_us) < S5_TRIGGER_PULSE_US)
    {
        /* The busy wait times only the trigger pulse. */
    }

    (void)gpio_set_val(S5_TRIG_GPIO, 0U);
    return true;
}

/** @brief Prefer a captured FIFO value to timeout and publish one result.
 * @see ultrasonic_sensor.h for nulls, output mutation and timeout semantics.
 */
bool ultrasonic_sensor_poll (uint32_t now, uint16_t * p_mm, bool * pb_is_valid)
{

    if (((!gb_is_echo_pending) || (0 == p_mm)) || (0 == pb_is_valid))
    {
        return false;
    }

    uint32_t cycles;
    if ((in_w(S5_PIO + 4U) & (1U << (8U + 2U))) == 0U)
    {
        cycles = in_w(S5_PIO + 0x20U + (2U * 4U));
        ultrasonic_sensor_cancel();
        *p_mm        = 0;
        *pb_is_valid = ultrasonic_sensor_echo_mm(
            cycles > 12000U ? 24001U : cycles * 2U, p_mm);
        return true;
    }

    /* Elapsed ticks are modulo 2^32. The caller uses the same clock
     * as at trigger time and polls within one wrap. */
    if ((now - g_started) >= S5_ECHO_TIMEOUT_US)
    {
        /* ECHO still high: the pulse started and is longer than the range
         * window, i.e. nothing within range (clear). ECHO low: no pulse ever
         * started - unpowered, unplugged or miswired sensor (invalid). */
        const bool b_is_echo_high = gpio_get_val(S5_ECHO_GPIO) != 0U;
        ultrasonic_sensor_cancel();
        /* The 4000 mm sentinel fits in uint16_t. */
        *p_mm        = b_is_echo_high ? (uint16_t)S5_RANGE_MAX_MM : 0U;
        *pb_is_valid = b_is_echo_high;
        return true;
    }

    return false;
}
#else
/** @brief Report unavailable hardware in an APP_S5=0 build.
 * @return Always false, without changing hardware or module state.
 */
bool ultrasonic_sensor_init (void)
{
    return false;
}

/** @brief Reject capture requests when APP_S5=0.
 * @param[in] now Ignored timestamp in microseconds.
 * @return Always false; no capture or trigger is started.
 */
bool ultrasonic_sensor_trigger (uint32_t now)
{
    (void)now;
    return false;
}

/** @brief Return no result when APP_S5=0, preserving caller outputs.
 * @param[in] now Ignored timestamp in microseconds.
 * @param[out] p_mm Ignored output pointer.
 * @param[out] pb_is_valid Ignored usability pointer.
 * @return Always false, including for null outputs.
 */
bool ultrasonic_sensor_poll (uint32_t now, uint16_t * p_mm, bool * pb_is_valid)
{
    (void)now;
    (void)p_mm;
    (void)pb_is_valid;
    return false;
}

/** @brief Do nothing when capture hardware is excluded from the build. */
void ultrasonic_sensor_cancel (void)
{
}
#endif

/* End of file. */

