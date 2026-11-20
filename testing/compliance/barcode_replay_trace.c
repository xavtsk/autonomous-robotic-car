/**
 * @file barcode_replay_trace.c
 * @brief Replay observed samples and expose every decoder work/state update.
 *
 * Host-only inputs are elapsed microseconds and normalized darkness 0..1000.
 * Optional arguments explicitly add synthetic leading/trailing paper in ms.
 * Stdout preserves the original replay protocol. Stderr retains per-update
 * state, quiet requirements, work budgets and exact decision timestamps.
 */
#include "barcode_replay_trace.h"
#include "app_program/subsystem3/bc_decoder.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

static s3_barcode_record_t g_record;
static uint32_t g_now;
static uint32_t g_decoded;
static uint32_t g_rejected;
static uint32_t g_timing;
static uint32_t g_overflow;

/** Advance one real or explicitly synthetic sample without draining work. */
static void barcode_trace_feed (uint32_t elapsed_us, uint16_t darkness)
{
    const s3_barcode_config_t config = s3_barcode_defaults();
    s3_barcode_result_t result;
    s3_barcode_status_t status;
    g_now += elapsed_us;
    status = s3_barcode_record_update(&g_record, &config, g_now, true, darkness,
                                      &result);
    if (S3_BARCODE_DECODED == status)
    {
        ++g_decoded;
        (void)printf("letter=%c reversed=%u timestamp_us=%" PRIu32 "\n",
                     result.letter, (unsigned int)result.reversed, g_now);
    }
    else if (S3_BARCODE_REJECTED == status)
    {
        ++g_rejected;
    }
    else if (S3_BARCODE_TIMING_ERROR == status)
    {
        ++g_timing;
    }
    else if (S3_BARCODE_OVERFLOW == status)
    {
        ++g_overflow;
    }
    else
    {
        /* NONE retains pending recording/processing; no synthetic drain. */
    }
    (void)fprintf(
        stderr,
        "t=%" PRIu32 " dt=%" PRIu32 " dark=%u status=%u "
        "state=%u count=%u phase=%u cursor=%u frozen=%u "
        "work=%u fit=%u frame=%" PRIu32 " required=%" PRIu32 " strict=%" PRIu32
        " tolerant=%" PRIu32 " quiet_required=%" PRIu32
        " letter_code=%u reverse=%u\n",
        g_now, elapsed_us, (unsigned int)darkness, (unsigned int)status,
        (unsigned int)g_record.state, (unsigned int)g_record.count,
        (unsigned int)g_record.work.phase, (unsigned int)g_record.work.cursor,
        (unsigned int)g_record.work.frozen_count,
        (unsigned int)g_record.work_units, (unsigned int)g_record.fit_ticks,
        g_record.frame_us, g_record.required_us, g_record.quiet.strict_us,
        g_record.quiet.tolerant_us, g_record.quiet_required_us,
        (unsigned int)g_record.candidate.letter_code,
        (unsigned int)g_record.candidate.reversed);
}

/** Parse bounded padding and samples; invalid inputs return 2 without replay.
 */
int main (int argc, char ** pp_argv)
{
    uint32_t elapsed_us = 0U;
    uint32_t darkness = 0U;
    uint32_t index = 0U;
    const uintmax_t leading_ms =
        (argc > 1) ? strtoumax(pp_argv[1], NULL, 10) : 0U;
    const uintmax_t trailing_ms =
        (argc > 2) ? strtoumax(pp_argv[2], NULL, 10) : 0U;
    if ((leading_ms > 10000U) || (trailing_ms > 10000U))
    {
        return 2;
    }
    (void)printf("synthetic_leading_ms=%" PRIuMAX
                 " synthetic_trailing_ms=%" PRIuMAX "\n",
                 leading_ms, trailing_ms);
    for (index = 0U; index < (leading_ms / 10U); ++index)
    {
        barcode_trace_feed(10000U, 0U);
    }
    while (2 == scanf("%" SCNu32 " %" SCNu32, &elapsed_us, &darkness))
    {
        if ((darkness > 1000U) || (0U == elapsed_us) || (elapsed_us > 1000000U))
        {
            return 2;
        }
        /* The preceding bound proves this narrowing is exact. */
        barcode_trace_feed(elapsed_us, (uint16_t)darkness);
    }
    for (index = 0U; index < (trailing_ms / 10U); ++index)
    {
        barcode_trace_feed(10000U, 0U);
    }
    (void)printf("decoded=%" PRIu32 " rejected=%" PRIu32 " timing=%" PRIu32
                 " overflow=%" PRIu32 " state=%u count=%u\n",
                 g_decoded, g_rejected, g_timing, g_overflow,
                 (unsigned int)g_record.state, (unsigned int)g_record.count);
    return 0;
}

/* End of file. */

