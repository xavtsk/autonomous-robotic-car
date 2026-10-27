/* Call on core 0 before starting timed application tasks. Does not reset an
 * already-running counter or change the kernel tick. Registers from sysdef.h.
 */
#ifndef APP_MONOTONIC_CLOCK_H
#define APP_MONOTONIC_CLOCK_H
/* RP2040 watchdog tick: drives the TIMER at 1 us from clk_ref (XOSC) */
#define WATCHDOG_TICK (0x40058000U + 0x2CU)
#define WATCHDOG_TICK_CYCLES (0x1FFU)
#define WATCHDOG_TICK_ENABLE (1UL << 9)
#define WATCHDOG_TICK_RUNNING (1UL << 10)

/* The kernel does not start the TIMER that time_now_us() reads (the Pico
 * SDK runtime normally does): take it out of reset and start its tick */
static inline void app_clock_start(void)
{
    uint32_t tick = in_w(WATCHDOG_TICK);

    if ((in_w(RESETS_RESET) & RESETS_RESET_TIMER) != 0U)
    {
        clr_w(RESETS_RESET, RESETS_RESET_TIMER);
        while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER) == 0U)
        {
            /* TIMER reset release takes a few cycles */
        }
    }
    if (((tick & WATCHDOG_TICK_RUNNING) == 0U) ||
        ((tick & WATCHDOG_TICK_CYCLES) != (uint32_t)XOSC_MHz))
    {
        out_w(WATCHDOG_TICK, WATCHDOG_TICK_ENABLE | (uint32_t)XOSC_MHz);
    }
}

#endif
