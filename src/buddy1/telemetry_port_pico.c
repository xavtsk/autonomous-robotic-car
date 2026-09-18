/* Pico-specific clock and synchronization seam for telemetry.c. */

#include "buddy1/telemetry.h"

#include "pico/stdlib.h"

uint32_t tele_now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void tele_lock(void)
{
    /* Correct only while the current bare-metal integration has one caller.
     * Replace with the lecturer port's measured critical section before
     * several μT-Kernel tasks call tele_report_* concurrently. */
}

void tele_unlock(void)
{
    /* See tele_lock(). */
}
