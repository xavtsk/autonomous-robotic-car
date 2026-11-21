/** @file motion_trace.c
 * @brief Shared harness: observe actual production feedback without consuming
 * extra reports.
 */
#include "motion_trace.h"
#include "../../../app_program/app_types.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

/** @brief Read the exact floating-point representation for comparison. */
static uint32_t trace_float (float32_t value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

/** @brief Print feedback and debug fields after the observed tick. */
void motion_trace_feedback (void)
{
    motion_feedback_t       feedback = {0};
    motion_debug_snapshot_t debug    = {0};
    uint64_t                distance_bits;
    bool                    feedback_ok;
    bool                    debug_ok;
    feedback_ok = motion_get_feedback(&feedback);
    debug_ok    = motion_try_debug_snapshot(&debug);
    memcpy(&distance_bits, &feedback.centre_m, sizeof(distance_bits));
    printf("M %u %u %" PRIu32 " %" PRIu32 " %u %u %u %u %u %" PRIu32
           " %016" PRIx64,
           feedback_ok, debug_ok, feedback.timestamp_us, feedback.epoch,
           feedback.state, feedback.fault, feedback.b_is_feedback_valid,
           feedback.b_is_braked, feedback.command_id,
           feedback.report_loss_count, distance_bits);
    for (unsigned index = 0;
         index < (sizeof debug.setpoint_mps / sizeof debug.setpoint_mps[0]);
         ++index)
    {
        printf(" %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %" PRId32,
               trace_float(debug.setpoint_mps[index]),
               trace_float(debug.measured_mps[index]),
               trace_float(debug.duty[index]), debug.encoder[index]);
    }
    printf(" %u %u %u\n", debug.state, debug.fault, debug.b_is_debug_busy);
}

/** @brief Print report fields only when the observed read succeeded. */
void motion_trace_report (bool received, const motion_report_t * p_report)
{
    printf("REPORT %u", received);
    if (received)
    {
        printf(" %u %u %u %u", p_report->command_id, p_report->outcome,
               p_report->fault, p_report->b_was_lost);
    }
    putchar('\n');
}

/* End of file. */

