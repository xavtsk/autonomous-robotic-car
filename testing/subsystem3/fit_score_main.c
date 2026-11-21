/** @file fit_score_main.c
 * @brief Exact-equivalence regression for the RP2040 fit-score optimization.
 *
 * Link the decoder's host-only test entry points to exercise the same fit
 * arithmetic and timing validation as the production module.
 */
#include "fit_score_main.h"
#include <assert.h>
#include <stdio.h>

#define FIT_ELEMENT_COUNT 9U
#define ORIENTATION_CASE_COUNT 2U
#define ERROR_SIDE_COUNT 2U
#define TIMING_DOMAIN_COUNT 2U
#define TRIAL_COUNT_PER_DOMAIN 100000U

static uint32_t random_state = 0x8421937U;

/** @brief Advance the deterministic 32-bit linear congruential sequence and
 * return its next word.
 */
static uint32_t random_word (void)
{
    random_state = random_state * 1664525U + 1013904223U;
    return random_state;
}

/** @brief Calculate the error with the previous production formula as an
 * independent reference for the candidate.
 */
static uint32_t original_error (const uint32_t adjusted[9], uint16_t star,
                                uint32_t narrow, uint32_t wide)
{
    uint32_t score = 0U;
    for (unsigned int element_index = 0U; element_index < FIT_ELEMENT_COUNT;
         element_index++)
    {
        const uint32_t reference =
            (star & (1U << (8U - element_index))) ? wide : narrow;
        const uint32_t delta = (adjusted[element_index] > reference)
                                   ? adjusted[element_index] - reference
                                   : reference - adjusted[element_index];
        const uint32_t error =
            (uint32_t)((uint64_t)delta * 1000000U / reference);
        if (error > score)
        {
            score = error;
        }
    }
    return score;
}

/** @brief Require nonzero narrow and wide references, compare the optimized
 * score with the arithmetic oracle and check the 350000 upper bound.
 */
static void check (const uint32_t adjusted[9], uint16_t star, uint32_t narrow,
                   uint32_t wide)
{
    assert(narrow != 0U && wide != 0U);
    assert(bc_decoder_fit_test_score(adjusted, star, narrow, wide) ==
           original_error(adjusted, star, narrow, wide));
    assert(bc_decoder_fit_test_score(adjusted, star, narrow, wide) <= 350000U);
}

/** @brief Check 432 cases covering both sentinel directions, extreme run
 * widths and values at the deviation limit.
 */
static void exact_boundaries (void)
{
    /* Includes minimum operational widths, non-divisible references,
     * and large widths whose 35% deviations still fit the legal max_run bound.
     */
    const uint32_t narrow[] = {30000U, 30001U, 100003U, 50000000U};
    unsigned int   checks   = 0U;
    for (unsigned int reverse = 0U; reverse < ORIENTATION_CASE_COUNT; reverse++)
    {
        const uint16_t star = reverse ? 0x052U : 0x094U;
        for (unsigned int width_index = 0U;
             width_index < sizeof(narrow) / sizeof(narrow[0]); width_index++)
        {
            const uint32_t wide = narrow[width_index] * 3U;
            for (unsigned int winner = 0U; winner < FIT_ELEMENT_COUNT; winner++)
            {
                for (unsigned int side = 0U; side < ERROR_SIDE_COUNT; side++)
                {
                    uint32_t adjusted[9];
                    for (unsigned int element_index = 0U;
                         element_index < FIT_ELEMENT_COUNT; element_index++)
                    {
                        adjusted[element_index] =
                            (star & (1U << (8U - element_index)))
                                ? wide
                                : narrow[width_index];
                    }
                    /* Exact zero, tiny deviations, and the strict 35%
                     * endpoint cover ties and integer floor boundaries. */
                    check(adjusted, star, narrow[width_index], wide);
                    checks++;
                    if (side)
                    {
                        adjusted[winner]++;
                    }
                    else
                    {
                        adjusted[winner]--;
                    }
                    check(adjusted, star, narrow[width_index], wide);
                    checks++;
                    const uint32_t reference = (star & (1U << (8U - winner)))
                                                   ? wide
                                                   : narrow[width_index];
                    const uint32_t delta =
                        (uint32_t)((uint64_t)reference * 35U / 100U);
                    adjusted[winner] =
                        side ? reference + delta : reference - delta;
                    check(adjusted, star, narrow[width_index], wide);
                    checks++;
                }
            }
        }
    }
    assert(432U == checks);
}

/** @brief Generate deterministic valid fits under both configured domains.
 * Compare each score with the independent oracle and return the
 * number checked.
 */
static unsigned int validated_random_fits (void)
{
    unsigned int checked = 0U;
    for (unsigned int domain = 0U; domain < TIMING_DOMAIN_COUNT; domain++)
    {
        bc_decoder_config_t config = bc_decoder_defaults();
        if (domain)
        {
            config.max_run_us   = 0x7FFFFFFFU / config.quiet_modules;
            config.max_frame_us = 0x7FFFFFFFU;
        }
        assert(bc_decoder_fit_test_config_valid(&config));
        for (unsigned int trial = 0U; trial < TRIAL_COUNT_PER_DOMAIN; trial++)
        {
            const uint16_t star = (trial & 1U) ? 0x052U : 0x094U;
            const uint32_t base = config.min_narrow_us +
                                  random_word() % (config.max_run_us / 5U -
                                                   config.min_narrow_us + 1U);
            const uint32_t broad =
                (uint32_t)((uint64_t)base * (200U + random_word() % 151U) /
                           100U);
            uint32_t adjusted[9];
            uint32_t narrow  = 0U;
            uint32_t wide    = 0U;
            uint16_t pattern = 0U;
            for (unsigned int element_index = 0U;
                 element_index < FIT_ELEMENT_COUNT; element_index++)
            {
                const uint32_t reference =
                    (star & (1U << (8U - element_index))) ? broad : base;
                adjusted[element_index] =
                    (uint32_t)((uint64_t)reference *
                               (65U + random_word() % 71U) / 100U);
                if (star & (1U << (8U - element_index)))
                {
                    wide += adjusted[element_index];
                }
            }
            if (!bc_decoder_fit_test_character(adjusted, 10000U, &config,
                                               &pattern, &narrow) ||
                pattern != star)
            {
                continue;
            }
            wide /= 3U;
            check(adjusted, star, narrow, wide);
            checked++;
        }
    }
    assert(checked > 50000U);
    return checked;
}

/** @brief Compare fitted barcode scores against the original arithmetic on
 * boundary widths and validated pseudorandom inputs.
 */
int main (void)
{
    exact_boundaries();
    const unsigned int random_checks = validated_random_fits();
    printf("PASS: exact fit-score equivalence, 432 boundary cases and %u "
           "validated random fits across default/maximal duration limits\n",
           random_checks);
    return 0;
}

/* End of file. */

