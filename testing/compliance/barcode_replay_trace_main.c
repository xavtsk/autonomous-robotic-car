/**
 * @file barcode_replay_trace_main.c
 * @brief Replay observed samples and expose every decoder work/state update.
 *
 * Host-only inputs are elapsed microseconds and normalized darkness 0..1000.
 * Optional arguments explicitly add synthetic leading/trailing paper in ms.
 * Stdout preserves the original replay protocol. Stderr retains per-update
 * state, quiet requirements, work budgets and exact decision timestamps.
 */
#include "barcode_replay_trace_main.h"
#include "app_program/subsystem3/bc_decoder.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#define TRACE_QUIET_SAMPLE_MS (10U)
#define TRACE_QUIET_SAMPLE_US (10000U)
#define TRACE_READING_FIELDS (2)

static bc_decoder_record_t g_record;
static uint32_t            g_now;
static uint32_t            g_decoded;
static uint32_t            g_rejected;
static uint32_t            g_timing;
static uint32_t            g_overflow;

/** Advance one real or explicitly synthetic sample without draining work. */
static void barcode_trace_feed (uint32_t elapsed_us, uint16_t darkness)
{
    const bc_decoder_config_t config = bc_decoder_defaults();
    bc_decoder_result_t       result;
    bc_decoder_status_t       status;
    g_now += elapsed_us;
    status = bc_decoder_record_update(&g_record, &config, g_now, true, darkness,
                                      &result);
    if ((bc_decoder_status_t)S3_BARCODE_DECODED == status)
    {
        ++g_decoded;
        (void)printf("letter=%c reversed=%u timestamp_us=%" PRIu32 "\n",
                     result.letter, (unsigned int)result.b_is_reversed, g_now);
    }
    else if ((bc_decoder_status_t)S3_BARCODE_REJECTED == status)
    {
        ++g_rejected;
    }
    else if ((bc_decoder_status_t)S3_BARCODE_TIMING_ERROR == status)
    {
        ++g_timing;
    }
    else if ((bc_decoder_status_t)S3_BARCODE_OVERFLOW == status)
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
        (unsigned int)g_record.candidate.b_is_reversed);
}

/** Parse bounded padding and samples; invalid inputs return 2 without replay.
 */
int main (int argc, char * pp_argv[])
{
    uint32_t        elapsed_us = 0U;
    uint32_t        darkness   = 0U;
    uint32_t        index      = 0U;
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
    for (index = 0U; index < (leading_ms / TRACE_QUIET_SAMPLE_MS); ++index)
    {
        barcode_trace_feed(TRACE_QUIET_SAMPLE_US, 0U);
    }
    while (TRACE_READING_FIELDS ==
           scanf("%" SCNu32 " %" SCNu32, &elapsed_us, &darkness))
    {
        if ((darkness > 1000U) || (0U == elapsed_us) || (elapsed_us > 1000000U))
        {
            return 2;
        }
        /* The preceding bound proves this narrowing is exact. */
        barcode_trace_feed(elapsed_us, (uint16_t)darkness);
    }
    for (index = 0U; index < (trailing_ms / TRACE_QUIET_SAMPLE_MS); ++index)
    {
        barcode_trace_feed(TRACE_QUIET_SAMPLE_US, 0U);
    }
    (void)printf("decoded=%" PRIu32 " rejected=%" PRIu32 " timing=%" PRIu32
                 " overflow=%" PRIu32 " state=%u count=%u\n",
                 g_decoded, g_rejected, g_timing, g_overflow,
                 (unsigned int)g_record.state, (unsigned int)g_record.count);
    return 0;
}

/* End of file. */

