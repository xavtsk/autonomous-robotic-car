/* Shared harness: observe actual production feedback without consuming extra reports. */
#include "motion.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
static void trace_process(void);
static bool trace_report(motion_report_t *report);
#define motion_process trace_process
#define motion_get_report trace_report
#include "test_motion_feedback.c"
#undef motion_process
#undef motion_get_report

static uint32_t trace_float(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static void trace_process(void)
{
    motion_feedback_t feedback = {0};
    motion_debug_snapshot_t debug = {0};
    uint64_t distance_bits;
    bool feedback_ok, debug_ok;
    motion_process();
    feedback_ok = motion_get_feedback(&feedback);
    debug_ok = motion_try_debug_snapshot(&debug);
    memcpy(&distance_bits, &feedback.centre_m, sizeof(distance_bits));
    printf("M %u %u %" PRIu32 " %" PRIu32 " %u %u %u %u %u %" PRIu32 " %016" PRIx64,
           feedback_ok, debug_ok, feedback.timestamp_us, feedback.epoch,
           feedback.state, feedback.fault, feedback.valid, feedback.braked,
           feedback.command_id, feedback.report_loss_count, distance_bits);
    for (unsigned index = 0; index < 2; ++index)
    {
        printf(" %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %" PRId32,
               trace_float(debug.setpoint_mps[index]), trace_float(debug.measured_mps[index]),
               trace_float(debug.duty[index]), debug.encoder[index]);
    }
    printf(" %u %u %u\n", debug.state, debug.fault, debug.busy);
}

static bool trace_report(motion_report_t *report)
{
    bool okay = motion_get_report(report);
    printf("REPORT %u", okay);
    if (okay)
    {
        printf(" %u %u %u %u", report->command_id, report->outcome, report->fault, report->b_lost);
    }
    putchar('\n');
    return okay;
}
