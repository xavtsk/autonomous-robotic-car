/** @file bc_quiet.h
 * @brief Classify trailing quiet intervals after strongly contrasted barcode
 * frames.
 */
#ifndef BUDDY3_BARCODE_QUIET_H
#define BUDDY3_BARCODE_QUIET_H
#include <stdbool.h>
#include <stdint.h>

/* These limits classify only trailing background after a complete, strongly
 * contrasted frame. They must never filter samples or qualify faint frames. */
#define S3_QUIET_NOISE_BAND                                                    \
    (14U) /* This lies below the 15-unit primary peak floor. */
#define S3_QUIET_PULSE_US (40000U)
#define S3_QUIET_STRONG_PROMINENCE (60U)

typedef struct
{
    uint32_t strict_us;
    uint32_t tolerant_us;
    uint32_t pulse_us;
    bool     b_is_pulse_invalid;
} bc_quiet_t;

typedef enum
{
    S3_QUIET_LOW     = 1U, /* The sample is below the strict band. */
    S3_QUIET_BREAK   = 2U, /* Sustained ink breaks the tolerant interval. */
    S3_QUIET_PENDING = 4U /* Wait for the excursion to end before publishing. */
} bc_quiet_flag_t;

/** @brief Add elapsed microseconds with saturation.
 * @param elapsed_us Current timer value in microseconds.
 * @param delta_us Elapsed microseconds to add, up to UINT32_MAX.
 * @return The sum, limited to UINT32_MAX.
 */
static inline uint32_t bc_quiet_add (uint32_t elapsed_us, uint32_t delta_us)
{
    return ((UINT32_MAX - elapsed_us) < delta_us) ? UINT32_MAX
                                                  : (elapsed_us + delta_us);
}

/** @brief Clear all interval clocks and the pulse rejection flag.
 * @param p_quiet Nonnull writable state owned by the caller.
 */
static inline void bc_quiet_reset (bc_quiet_t * p_quiet)
{
    *p_quiet = (bc_quiet_t){0};
}

/** @brief Account for one sample of the trailing barcode quiet interval.
 * @param p_quiet Nonnull state initialized by reset. One sampling owner updates
 * it.
 * @param step_us Elapsed microseconds to accumulate in the relevant clocks.
 * @param dark Sample darkness on the recorder's normalized 0..1000 scale.
 * @param paper Frozen paper reference on the same scale.
 * @param strict_band Maximum rise above paper accepted as quiet.
 * @param b_allows_noise Allow short excursions into the wider noise band.
 * @return LOW at or below the strict limit. Above it, return PENDING and
 * additionally BREAK once the pulse exceeds its amplitude or time limit.
 * @note A qualifying pulse counts toward the tolerant interval only on return
 * to quiet. Releasing a decode still requires validation of all 15 strong bars.
 */
static inline unsigned int bc_quiet_update (bc_quiet_t * p_quiet,
                                            uint32_t step_us, unsigned int dark,
                                            unsigned int paper,
                                            unsigned int strict_band,
                                            bool         b_allows_noise)
{
    const unsigned int noise_band =
        (b_allows_noise && (strict_band < S3_QUIET_NOISE_BAND))
            ? S3_QUIET_NOISE_BAND
            : strict_band;

    if (dark <= (paper + strict_band))
    {
        p_quiet->strict_us = bc_quiet_add(p_quiet->strict_us, step_us);

        if (((0U != p_quiet->pulse_us) && (!p_quiet->b_is_pulse_invalid)) &&
            (0U != p_quiet->tolerant_us))
        {
            p_quiet->tolerant_us =
                bc_quiet_add(p_quiet->tolerant_us, p_quiet->pulse_us);
        }

        /* clang-format off */
        p_quiet->tolerant_us        =
            bc_quiet_add(p_quiet->tolerant_us, step_us);
        p_quiet->pulse_us           = 0U;
        p_quiet->b_is_pulse_invalid = false;
        /* clang-format on */
        return S3_QUIET_LOW;
    }

    p_quiet->strict_us = 0U;
    p_quiet->pulse_us  = bc_quiet_add(p_quiet->pulse_us, step_us);

    if ((dark > (paper + noise_band)) ||
        (p_quiet->pulse_us > S3_QUIET_PULSE_US))
    {
        p_quiet->tolerant_us        = 0U;
        p_quiet->b_is_pulse_invalid = true;
    }

    /* Both enum constants fit in unsigned int. Combine the masks there. */
    return (unsigned int)S3_QUIET_PENDING |
           (p_quiet->b_is_pulse_invalid ? (unsigned int)S3_QUIET_BREAK : 0U);
}
#endif

/* End of file. */

