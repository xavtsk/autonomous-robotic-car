/** @file quiet_zone_main.c
 * @brief Exercise strict and tolerant trailing barcode quiet intervals.
 */
#include "quiet_zone_main.h"
#include <assert.h>
#include <stdio.h>
#include "bc_quiet.h"

#define PERIOD_CASE_STEP_US 5000U
#define LAST_PERIOD_CASE_US 10000U

#define BRIDGED_PULSE_LIMIT_US 40000U
#define FRAME_CONTRAST_CASE_COUNT 2U

/** @brief Exercise strict and tolerant barcode quiet clocks across pulse,
 * cancellation and saturation boundaries.
 */
int main (void)
{
    bc_quiet_t quiet_clock = {0};
    for (unsigned int period                    = PERIOD_CASE_STEP_US;
         period <= LAST_PERIOD_CASE_US; period += PERIOD_CASE_STEP_US)
    {
        for (unsigned int duration = period; duration <= BRIDGED_PULSE_LIMIT_US;
             duration             += period)
        {
            bc_quiet_reset(&quiet_clock);
            assert(bc_quiet_update(&quiet_clock, 200000U, 0U, 0U, 5U, true) ==
                   (unsigned int)S3_QUIET_LOW);
            for (unsigned int elapsed = 0U; elapsed < duration;
                 elapsed             += period)
            {
                assert(bc_quiet_update(&quiet_clock, period, 14U, 0U, 5U,
                                       true) == (unsigned int)S3_QUIET_PENDING);
                assert(0U == quiet_clock.strict_us &&
                       200000U == quiet_clock.tolerant_us);
            }
            assert(bc_quiet_update(&quiet_clock, period, 5U, 0U, 5U, true) ==
                   (unsigned int)S3_QUIET_LOW);
            assert(quiet_clock.strict_us == period &&
                   quiet_clock.tolerant_us == 200000U + duration + period);
            assert(0U == quiet_clock.pulse_us &&
                   !quiet_clock.b_is_pulse_invalid);
        }
    }
    /* A long or stronger excursion loses all earlier quiet. Later real paper
     * starts a new interval, rather than retroactively restoring that margin.
     */
    for (unsigned int strong = 0U; strong < FRAME_CONTRAST_CASE_COUNT; strong++)
    {
        bc_quiet_reset(&quiet_clock);
        (void)bc_quiet_update(&quiet_clock, 1000000U, 0U, 0U, 5U, true);
        const unsigned int flags =
            bc_quiet_update(&quiet_clock, strong ? 10000U : 40500U,
                            strong ? 15U : 14U, 0U, 5U, true);
        assert(flags ==
               ((unsigned int)S3_QUIET_PENDING | (unsigned int)S3_QUIET_BREAK));
        assert(0U == quiet_clock.strict_us && 0U == quiet_clock.tolerant_us);
        (void)bc_quiet_update(&quiet_clock, 10000U, 0U, 0U, 5U, true);
        assert(10000U == quiet_clock.strict_us &&
               10000U == quiet_clock.tolerant_us);
    }
    /* Without permission there is no relaxed interpretation at all. */
    bc_quiet_reset(&quiet_clock);
    (void)bc_quiet_update(&quiet_clock, 100000U, 100U, 100U, 5U, false);
    assert(bc_quiet_update(&quiet_clock, 10000U, 106U, 100U, 5U, false) ==
           ((unsigned int)S3_QUIET_PENDING | (unsigned int)S3_QUIET_BREAK));
    assert(0U == quiet_clock.strict_us && 0U == quiet_clock.tolerant_us);
    /* Wider existing bands retain their own strict semantics. */
    bc_quiet_reset(&quiet_clock);
    (void)bc_quiet_update(&quiet_clock, 100000U, 75U, 0U, 75U, true);
    assert(100000U == quiet_clock.strict_us &&
           100000U == quiet_clock.tolerant_us);
    assert(bc_quiet_update(&quiet_clock, 10000U, 76U, 0U, 75U, true) &
           (unsigned int)S3_QUIET_BREAK);
    /* No leading interval is manufactured from a pulse before any paper. */
    bc_quiet_reset(&quiet_clock);
    (void)bc_quiet_update(&quiet_clock, 10000U, 10U, 0U, 5U, true);
    (void)bc_quiet_update(&quiet_clock, 10000U, 0U, 0U, 5U, true);
    assert(10000U == quiet_clock.tolerant_us);
    quiet_clock.strict_us = quiet_clock.tolerant_us = UINT32_MAX - 5000U;
    (void)bc_quiet_update(&quiet_clock, 10000U, 0U, 0U, 5U, true);
    assert(UINT32_MAX == quiet_clock.strict_us &&
           UINT32_MAX == quiet_clock.tolerant_us);
    puts("PASS: strict/tolerant quiet clocks, pulse boundaries, cancellation "
         "and saturation");
    return 0;
}

/* End of file. */

