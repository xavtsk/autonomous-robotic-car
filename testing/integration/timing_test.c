#include "../../app_program/controller/run_timing.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    s3_run_timing_t run;
    s3_run_timing_begin(&run, UINT32_MAX - 5000U);
    assert(run.started_us == UINT32_MAX - 5000U && run.samples == 0U);
    s3_run_timing_record(&run, 10000U, 4999U, 30000U);
    assert(run.samples == 1U && run.above_target == 0U && run.above_safety == 0U);
    s3_run_timing_record(&run, 10001U, 15000U, 30000U);
    s3_run_timing_record(&run, 30000U, 45000U, 30000U);
    assert(run.above_target == 2U && run.above_safety == 0U);
    s3_run_timing_record(&run, 130000U, 175000U, 30000U);
    assert(run.samples == 4U && run.max_gap_us == 130000U);
    assert(run.above_safety == 1U && run.last_safety_us == 175000U);
    /* Counters must remain interpretable rather than rolling over. */
    run.samples = run.above_target = run.above_safety = UINT32_MAX;
    s3_run_timing_record(&run, 30001U, 205001U, 30000U);
    assert(run.samples == UINT32_MAX && run.above_target == UINT32_MAX &&
           run.above_safety == UINT32_MAX && run.max_gap_us == 130000U);
    s3_run_timing_begin(&run, 215001U);
    assert(run.started_us == 215001U && run.samples == 0U &&
           run.max_gap_us == 0U && run.last_safety_us == 0U);
    puts("PASS: run timing target/safety boundaries, timestamp wrap, saturation and per-run reset");
    return 0;
}
