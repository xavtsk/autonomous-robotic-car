/** @file timing_main.c
 * @brief Exercise run timing accumulation and diagnostic counters.
 */
#include "timing_main.h"
#include "../../app_program/controller/run_timing.h"
#include <assert.h>
#include <stdio.h>

/** @brief Exercise the timing monitor through timestamp wrap, counter
 * saturation, per-run reset and nominal sample boundaries.
 */
int main (void)
{
    run_timing_t run;
    uint32_t     now = UINT32_MAX - (S3_SAMPLE_PERIOD_US / 2U);
    run_timing_begin(&run, now);
    assert(run.started_us == now && 0U == run.samples);
    now += S3_SAMPLE_PERIOD_US; /* Preserve the target gap across rollover. */
    run_timing_record(&run, S3_SAMPLE_PERIOD_US, now, 30000U);
    assert(1U == run.samples && 0U == run.above_target &&
           0U == run.above_safety);
    now += S3_SAMPLE_PERIOD_US + 1U;
    run_timing_record(&run, S3_SAMPLE_PERIOD_US + 1U, now, 30000U);
    now += 30000U;
    run_timing_record(&run, 30000U, now, 30000U);
    assert(2U == run.above_target && 0U == run.above_safety);
    now += 130000U;
    run_timing_record(&run, 130000U, now, 30000U);
    assert(4U == run.samples && 130000U == run.max_gap_us);
    assert(1U == run.above_safety && run.last_safety_us == now);
    /* Counters must remain interpretable rather than rolling over. */
    run.samples = run.above_target = run.above_safety = UINT32_MAX;

    now += 30001U;
    run_timing_record(&run, 30001U, now, 30000U);
    assert(UINT32_MAX == run.samples && UINT32_MAX == run.above_target &&
           UINT32_MAX == run.above_safety && 130000U == run.max_gap_us);
    now += S3_SAMPLE_PERIOD_US;
    run_timing_begin(&run, now);
    assert(run.started_us == now && 0U == run.samples && 0U == run.max_gap_us &&
           0U == run.last_safety_us);
    puts("PASS: run timing target/safety boundaries, timestamp wrap, "
         "saturation and per-run reset");
    return 0;
}

/* End of file. */

