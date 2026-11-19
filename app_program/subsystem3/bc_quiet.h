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
#define S3_QUIET_NOISE_BAND (14U) /* below the 15-unit primary peak floor */
#define S3_QUIET_PULSE_US (40000U)
#define S3_QUIET_STRONG_PROMINENCE (60U)

typedef struct
{
    uint32_t strict_us;
    uint32_t tolerant_us;
    uint32_t pulse_us;
    bool pulse_bad;
} s3_barcode_quiet_t;

enum
{
    S3_QUIET_LOW = 1U,    /* current reading is below the strict band */
    S3_QUIET_BREAK = 2U,  /* real/long ink; tolerant interval is broken */
    S3_QUIET_PENDING = 4U /* do not publish until this excursion ends */
};

static inline uint32_t s3_quiet_add (uint32_t value_a, uint32_t value_b)
{
    return UINT32_MAX - value_a < value_b ? UINT32_MAX : value_a + value_b;
}

static inline void s3_barcode_quiet_reset (s3_barcode_quiet_t * p_q)
{
    *p_q = (s3_barcode_quiet_t){0};
}

/* strict_band is the existing rec_white band, relative to frozen paper.
 * permit_noise only enables the provisional clock: a caller must additionally
 * validate all 15 strong bars before using it to release a decoded result.
 * Pending pulses contribute elapsed time only once they return below the band.
 * Thus a pulse still in progress can neither trigger analysis nor publish. */
static inline unsigned int
s3_barcode_quiet_update (s3_barcode_quiet_t * p_q, uint32_t step_us,
                         unsigned int dark, unsigned int paper,
                         unsigned int strict_band, bool permit_noise)
{
    const unsigned int noise_band =
        (permit_noise && strict_band < S3_QUIET_NOISE_BAND)
            ? S3_QUIET_NOISE_BAND
            : strict_band;
    if (dark <= paper + strict_band)
    {
        p_q->strict_us = s3_quiet_add(p_q->strict_us, step_us);
        if (p_q->pulse_us && !p_q->pulse_bad && p_q->tolerant_us)
        {
            p_q->tolerant_us = s3_quiet_add(p_q->tolerant_us, p_q->pulse_us);
        }
        p_q->tolerant_us = s3_quiet_add(p_q->tolerant_us, step_us);
        p_q->pulse_us = 0U;
        p_q->pulse_bad = false;
        return S3_QUIET_LOW;
    }
    p_q->strict_us = 0U;
    p_q->pulse_us = s3_quiet_add(p_q->pulse_us, step_us);
    if (dark > paper + noise_band || p_q->pulse_us > S3_QUIET_PULSE_US)
    {
        p_q->tolerant_us = 0U;
        p_q->pulse_bad = true;
    }
    return S3_QUIET_PENDING | (p_q->pulse_bad ? S3_QUIET_BREAK : 0U);
}
#endif

/* End of file. */

