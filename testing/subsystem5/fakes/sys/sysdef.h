/** @file sysdef.h
 * @brief Provide controlled RP2040 register reads and writes for obstacle
 * tests.
 */
#ifndef S5_FAKE_SYSDEF_H
#define S5_FAKE_SYSDEF_H
#include <stdint.h>
#define TIMER_TIMERAWL 0x40054028U
#define RESETS_RESET 0x4000c000U
#define RESETS_RESET_DONE 0x4000c008U
#define RESETS_RESET_PIO0 (1U << 10)
#define RESETS_RESET_PWM (1U << 14)
#define PWM_BASE 0x40050000U
/* The servo driver uses the imported RP2040 name for this 0x04 offset. */
#define PWM_CHx_DIV 4U
#define SYSCLK 125U
uint32_t fake_read (uint32_t address);
void     fake_write (uint32_t address, uint32_t value);

/** @brief Read a 32-bit register from the obstacle hardware fixture.
 * @param address RP2040 register address to query.
 * @return The word supplied by the simulated register bank.
 */
static inline uint32_t in_w (uint32_t address)
{
    return fake_read(address);
}

/** @brief Forward one register write to the obstacle hardware fixture.
 * @param address RP2040 register address to update.
 * @param value Complete 32-bit register value.
 */
static inline void out_w (uint32_t address, uint32_t value)
{
    fake_write(address, value);
}

/** @brief Read the fake register once and clear the requested mask bits. */
static inline void clr_w (uint32_t address, uint32_t bits)
{
    fake_write(address, fake_read(address) & ~bits);
}

#endif

/* End of file. */

