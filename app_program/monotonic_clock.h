/** @file monotonic_clock.h
 * @brief Start the RP2040 timer clock used by timed application tasks.
 * The hardware masks fit in unsigned. Convert them before combining
 * register or packet bits.
 */
/* Call on core 0 before starting timed application tasks. Does not reset an
 * already-running counter or change the kernel tick. Registers from sysdef.h.
 */
#ifndef APP_MONOTONIC_CLOCK_H
#define APP_MONOTONIC_CLOCK_H
/* The RP2040 watchdog tick drives TIMER in 1 us steps from clk_ref. */
#define WATCHDOG_TICK (0x40058000U + 0x2CU)
#define WATCHDOG_TICK_CYCLES (0x1FFU)
#define WATCHDOG_TICK_ENABLE (1UL << 9)
#define WATCHDOG_TICK_RUNNING (1UL << 10)

/* The kernel leaves the timer used by time_now_us() in reset. Release it and
 * start its tick when the Pico SDK startup has not already done so.
 */
/** @brief Release the RP2040 TIMER from reset when needed and start or correct
 * its microsecond watchdog tick.
 */
static inline void monotonic_clock_start (void)
{
    uint32_t tick = in_w(WATCHDOG_TICK);

    if ((in_w(RESETS_RESET) & (unsigned)RESETS_RESET_TIMER) != 0U)
    {
        clr_w(RESETS_RESET, RESETS_RESET_TIMER);

        while ((in_w(RESETS_RESET_DONE) & (unsigned)RESETS_RESET_TIMER) == 0U)
        {
            /* Wait until TIMER leaves reset. */
        }
    }

    if (((tick & WATCHDOG_TICK_RUNNING) == 0U) ||
        ((tick & WATCHDOG_TICK_CYCLES) != (uint32_t)XOSC_MHz))
    {
        out_w(WATCHDOG_TICK, WATCHDOG_TICK_ENABLE | (uint32_t)XOSC_MHz);
    }
}

#endif

/* End of file. */

