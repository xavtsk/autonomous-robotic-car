/** @file run_timing.h
 * @brief Observation only: one sampling owner updates this; packed status
 * carries a copied view.
 *
 * No scheduling, control, threshold or motion-policy changes.
 */
#ifndef S3_RUN_TIMING_H
#define S3_RUN_TIMING_H
#include <stdint.h>

#define S3_SAMPLE_PERIOD_MS (5U) /* Allow 30 ms modules at 0.10 m/s. */
#define S3_SAMPLE_PERIOD_US (S3_SAMPLE_PERIOD_MS * 1000U)

/* Monotonic hardware timestamps wrap modulo 2^32. Counters saturate rather
 * than wrap; 32 bits retain microsecond gaps and >248 days of 200 Hz samples.
 * All six fields are encoded explicitly in the internal status packet. */
typedef struct
{
    uint32_t started_us;
    uint32_t samples;
    uint32_t max_gap_us;
    uint32_t above_target;
    uint32_t above_safety;
    uint32_t last_safety_us;
} run_timing_t;

/** @brief Increment an observation counter without wrapping at UINT32_MAX.
 * @param p_value Nonnull counter owned exclusively by the caller.
 * @note Leaves an already saturated counter unchanged; takes no lock.
 */
static inline void run_timing_increment (uint32_t * p_value)
{
    if (*p_value != UINT32_MAX)
    {
        ++*p_value;
    }
}

/** @brief Start a timing record and clear all previous observations.
 * @param p_timing Nonnull record owned by the sampling task.
 * @param now Wrapping hardware timestamp in microseconds for accepted START.
 * @note Overwrites the whole record, sets started_us, and leaves counters and
 * gap/safety observations zero. The caller provides exclusive access.
 */
static inline void run_timing_begin (run_timing_t * p_timing, uint32_t now)
{
    *p_timing = (run_timing_t){.started_us = now};
}

/** @brief Add one sample interval to an existing observation record.
 * @param p_timing Nonnull record owned exclusively by the sampling task.
 * @param gap_us Already-computed elapsed sample interval in microseconds.
 * @param stamp_us Wrapping microsecond timestamp associated with this sample.
 * @param safety_limit_us Existing navigation safety limit in microseconds.
 * @note Saturates sample/threshold counters, retains the largest gap, and
 * records stamp_us only when gap_us strictly exceeds safety_limit_us. The
 * target counter also uses a strict comparison with S3_SAMPLE_PERIOD_US.
 * No deadline policy, scheduling or controller state is changed here.
 */
static inline void run_timing_record (run_timing_t * p_timing, uint32_t gap_us,
                                      uint32_t stamp_us,
                                      uint32_t safety_limit_us)
{
    run_timing_increment(&p_timing->samples);

    if (gap_us > p_timing->max_gap_us)
    {
        p_timing->max_gap_us = gap_us;
    }
    /* Above-target intervals include ordinary jitter: not a missed-deadline
     * count. The separate safety counter uses the existing navigation limit. */
    if (gap_us > S3_SAMPLE_PERIOD_US)
    {
        run_timing_increment(&p_timing->above_target);
    }

    if (gap_us > safety_limit_us)
    {
        run_timing_increment(&p_timing->above_safety);
        p_timing->last_safety_us = stamp_us;
    }
}
#endif

/* End of file. */

