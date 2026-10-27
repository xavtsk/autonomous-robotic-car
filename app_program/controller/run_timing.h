/* Observation only: one sampling owner updates this; packed status carries a
 * copied view. No scheduling, control, threshold or motion-policy changes. */
#ifndef S3_RUN_TIMING_H
#define S3_RUN_TIMING_H
#include <stdint.h>

#define S3_SAMPLE_PERIOD_MS (10U)
#define S3_SAMPLE_PERIOD_US (S3_SAMPLE_PERIOD_MS * 1000U)

/* Monotonic hardware timestamps wrap modulo 2^32. Counters saturate rather
 * than wrap; 32 bits retain microsecond gaps and >1 year of 100 Hz samples.
 * All six fields are encoded explicitly in the internal status packet. */
typedef struct
{
    uint32_t started_us, samples, max_gap_us;
    uint32_t above_target, above_safety, last_safety_us;
} s3_run_timing_t;

static inline void s3_timing_increment(uint32_t *value)
{
    if (*value != UINT32_MAX) { ++*value; }
}
static inline void s3_run_timing_begin(s3_run_timing_t *timing, uint32_t now)
{
    *timing = (s3_run_timing_t){.started_us = now};
}
static inline void s3_run_timing_record(s3_run_timing_t *timing,
                                        uint32_t gap_us, uint32_t stamp_us,
                                        uint32_t safety_limit_us)
{
    s3_timing_increment(&timing->samples);
    if (gap_us > timing->max_gap_us) { timing->max_gap_us = gap_us; }
    /* Above-target intervals include ordinary jitter: not a missed-deadline
     * count. The separate safety counter uses the existing navigation limit. */
    if (gap_us > S3_SAMPLE_PERIOD_US)
    {
        s3_timing_increment(&timing->above_target);
    }
    if (gap_us > safety_limit_us)
    {
        s3_timing_increment(&timing->above_safety);
        timing->last_safety_us = stamp_us;
    }
}
#endif
