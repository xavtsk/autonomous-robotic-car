/** @file scan_record_main.c
 * @brief Host tests for the record-then-decide barcode reader
 * (s3_barcode_record_*).
 *
 * Synthetic darkness traces only; real-car evidence is in the BCAP traces.
 * Each trace: paper, a Code 39 "*X*" frame (bars/gaps at chosen darkness
 * levels, optional blur = moving average over an aperture, optional noise),
 * then paper again.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 */
#include "scan_record_main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bc_decoder.h"

#define BLUR_INTEGRATION_STEP_US 1000U

#define GAP_TEST_LEADING_PAPER_SAMPLES 100U

#define REVERSED_RUN_PAIRS 14U
#define WIDE_BAR_RUN_INDEX 14U
#define PAYLOAD_FIRST_RUN 10U
#define PAYLOAD_END_RUN 19U
#define OVERFLOW_EXTRA_SAMPLES 100U

#define CODE39_ELEMENT_COUNT 9U
#define BASE_SCAN_SHAPE_COUNT 3U
#define FIXTURE_ALPHABET_COUNT 26U
#define SCAN_DIRECTION_COUNT 2U
#define SAMPLING_RATE_CASE_COUNT 2U
#define FIXTURE_FRAME_RUN_COUNT 29U
#define LAST_INTERRUPTION_CASE 2U
#define LOW_CONTRAST_SAMPLE_COUNT 700U
#define PATCH_SIDE_COUNT 2U
#define MIN_PATCH_OFFSET_US 10000U
#define MAX_PATCH_OFFSET_US 50000U
#define RANDOM_FRAME_TRIAL_COUNT 3000U
#define QUANTIZATION_CASE_COUNT 2U
#define TIMEOUT_SAMPLE_COUNT 360U
#define FAST_LEAD_SAMPLE_COUNT 400U
#define CLAMP_LEAD_SAMPLE_COUNT 50U
#define WIDTH_BIAS_DIRECTION_COUNT 2U
#define ADAPTIVE_QUIET_CASE_COUNT 6U

typedef struct
{
    uint32_t narrow_us;
    uint32_t wide_us;   /* Set the barcode module widths. */
    uint32_t period_us; /* Set the interval between samples. */
    uint32_t blur_us;   /* Zero aperture gives a sharp edge. */
    uint16_t paper;
    uint16_t bar;
    uint16_t gap;   /* Set the synthetic darkness levels. */
    uint16_t noise; /* Set the peak noise in either direction. */
    uint32_t lead_us;
    uint32_t trail_us; /* Provide paper margins before and after the frame. */
} shape_t;

/** @brief Counts and last decoded result for a synthetic scan. */
typedef struct
{
    unsigned int        decoded;
    unsigned int        rejected;
    unsigned int        timing;
    unsigned int        overflow;
    unsigned int        max_recorded;
    bc_decoder_result_t last;
} tally_t;

typedef enum
{
    RIPPLE_INTACT,
    RIPPLE_BAD_START,
    RIPPLE_BAD_STOP,
    RIPPLE_MISSING_BAR,
    RIPPLE_SHORT_LEAD,
    RIPPLE_BROKEN_TRAIL
} scan_record_ripple_case_t;

static const uint16_t g_letters[26] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
    0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
    0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0};

static uint32_t g_rng = 12345U;

static const shape_t g_sharp = {40000U, 120000U, 10000U, 0U,       10U,
                                1000U,  10U,     0U,     1500000U, 1500000U};

static void test_all_letters (void);

static void test_200hz_long_frame (void);

static void test_faint_y_beats_fixed_limits (void);

static void test_faint_recording_start (void);

static void test_real_pretrigger_history (void);

static void test_faint_rearm_on_paper (void);

static void test_scratch_inside_wide_bar (void);

static void test_grey_shelf_in_wide_bar (void);

static void test_merged_wide_bar_dip (void);

static void test_gap_spike_letter (void);

static void test_never_a_wrong_letter (void);

static void test_dense_wide_bar_ripples (void);

static void test_cut_pass_is_not_read (void);

static void test_short_quiet_zones (void);

static void test_line_crossing (void);

static void test_limits_and_faults (void);

static void test_shared_width_bias (void);

static void test_adaptive_peaks_and_quiet (void);

/** @brief Feed recorded-style waveforms through the barcode reader and check
 * letters, timing bounds, reset and quiet-zone handling.
 */
int main (void)
{
    assert(sizeof(((bc_decoder_record_t *)0)->sample[0]) == 2U);
    test_all_letters();
    test_200hz_long_frame();
    test_faint_y_beats_fixed_limits();
    test_faint_recording_start();
    test_real_pretrigger_history();
    test_faint_rearm_on_paper();
    test_scratch_inside_wide_bar();
    test_grey_shelf_in_wide_bar();
    test_merged_wide_bar_dip();
    test_gap_spike_letter();
    test_never_a_wrong_letter();
    test_dense_wide_bar_ripples();
    test_cut_pass_is_not_read();
    test_short_quiet_zones();
    test_line_crossing();
    test_limits_and_faults();
    test_shared_width_bias();
    test_adaptive_peaks_and_quiet();
    puts("PASS: record-then-decide barcode reader (26 letters x 2 "
         "directions x sharp/blurred/faint at 100/200 Hz, "
         "200 Hz long frames beyond 1024 samples, unchanged minimum widths "
         "and complete-frame requirement, "
         "scratch trim, grey shelf, 3000 frames with +/-25 % width jitter "
         "and no wrong letter, cut pass, quiet zones, line "
         "crossing, overflow, timing, invalid input, sentinel-only bias "
         "both signs/all letters/directions, "
         "invalid payload, inconsistent stars, corrected quiet, adaptive "
         "faint peaks/noise, faint starts, real pre-trigger history, "
         "weak opening stars, history wrap/reset, no invented quiet and "
         "faint-patch rearming)");
    return 0;
}

/** @brief Advance the fixture generator and return a reproducible 16-bit
 * portion of its state.
 */
static uint32_t next_rand (void)
{
    g_rng = (g_rng * 1103515245U) + 12345U;
    return (g_rng >> 8) & 0xFFFFU;
}

/** @brief Write the 29 Code 39 run durations for a framed uppercase letter
 * using the shape's narrow and wide durations.
 */
static void make_runs (char letter, const shape_t * p_shape, uint32_t runs[29])
{
    uint16_t sym[3] = {0x094U, g_letters[(unsigned int)(letter - 'A')], 0x094U};

    unsigned int symbol_index;
    unsigned int index;

    for (symbol_index = 0U; symbol_index < (sizeof sym / sizeof sym[0]);
         symbol_index++)
    {
        for (index = 0U; index < CODE39_ELEMENT_COUNT; index++)
        {
            runs[(symbol_index * 10U) + index] =
                (sym[symbol_index] & (1U << (8U - index))) ? p_shape->wide_us
                                                           : p_shape->narrow_us;
        }

        if (symbol_index < 2U)
        {
            runs[(symbol_index * 10U) + 9U] = p_shape->narrow_us;
        }
    }
}

/** @brief Return the ideal reflectance at time t in microseconds from the
 * trace start. Odd-numbered stripes are gaps.
 */
static uint16_t ideal (const shape_t * p_shape, const uint32_t * p_runs,
                       unsigned int run_count, uint32_t elapsed_us)
{
    uint32_t     edge = p_shape->lead_us;
    unsigned int index;

    if (elapsed_us < edge)
    {
        return p_shape->paper;
    }

    for (index = 0U; index < run_count; index++)
    {
        edge += p_runs[index];

        if (elapsed_us < edge)
        {
            return (index % 2U) == 0U ? p_shape->bar : p_shape->gap;
        }
    }

    return p_shape->paper;
}

/** @brief Sample the supplied run sequence using the selected shape and
 * accumulate decoder results while advancing the caller's clock.
 */
static tally_t feed (bc_decoder_record_t * p_report, uint32_t * p_clock,
                     const shape_t * p_shape, const uint32_t * p_runs,
                     unsigned int run_count)
{
    const bc_decoder_config_t cfg      = bc_decoder_defaults();
    tally_t                   outcomes = {0};
    uint32_t                  total    = p_shape->lead_us + p_shape->trail_us;
    uint32_t                  sample_time_us;
    unsigned int              index;

    for (index = 0U; index < run_count; index++)
    {
        total += p_runs[index];
    }

    for (sample_time_us  = 0U; sample_time_us < total;
         sample_time_us += p_shape->period_us)
    {
        uint32_t            sum = 0U;
        uint32_t            cnt = 0U;
        uint32_t            blur_sample_us;
        int32_t             signed_darkness;
        bc_decoder_result_t res;
        bc_decoder_status_t status;

        if (0U == p_shape->blur_us)
        {
            sum = ideal(p_shape, p_runs, run_count, sample_time_us);
            cnt = 1U;
        }
        else
        {
            for (blur_sample_us = (sample_time_us > (p_shape->blur_us / 2U))
                                      ? sample_time_us - (p_shape->blur_us / 2U)
                                      : 0U;
                 blur_sample_us <= (sample_time_us + (p_shape->blur_us / 2U));
                 blur_sample_us += BLUR_INTEGRATION_STEP_US)
            {
                sum += ideal(p_shape, p_runs, run_count, blur_sample_us);
                cnt++;
            }
        }

        signed_darkness = (int32_t)(sum / cnt);

        if (p_shape->noise != 0U)
        {
            signed_darkness +=
                (int32_t)(next_rand() % ((2U * p_shape->noise) + 1U)) -
                (int32_t)p_shape->noise;
        }

        if (signed_darkness < 0)
        {
            signed_darkness = 0;
        }

        if (signed_darkness > 1000)
        {
            signed_darkness = 1000;
        }

        *p_clock += p_shape->period_us;
        status    = bc_decoder_record_update(p_report, &cfg, *p_clock, true,
                                             (uint16_t)signed_darkness, &res);

        if (p_report->count > outcomes.max_recorded)
        {
            outcomes.max_recorded = p_report->count;
        }

        if ((unsigned)S3_BARCODE_DECODED == status)
        {
            outcomes.decoded++;
            outcomes.last = res;
        }
        else if ((unsigned)S3_BARCODE_REJECTED == status)
        {
            outcomes.rejected++;
        }
        else if ((unsigned)S3_BARCODE_TIMING_ERROR == status)
        {
            outcomes.timing++;
        }
        else if ((unsigned)S3_BARCODE_OVERFLOW == status)
        {
            outcomes.overflow++;
        }
        else
        {
            /* This status does not add to the scan totals. */
        }
    }

    return outcomes;
}

/** @brief Reverse the 29 widths in place. The middle width stays at index
 * 14.
 */
static void reverse_runs (uint32_t runs[29])
{
    unsigned int index;

    for (index = 0U; index < REVERSED_RUN_PAIRS; index++)
    {
        uint32_t saved_width_us = runs[index];
        runs[index]             = runs[28U - index];
        runs[28U - index]       = saved_width_us;
    }
}

/** @brief Feed six optical shapes for every uppercase letter in both
 * directions and check the result and the final capture state.
 */
static void test_all_letters (void)
{
    /* Sharp, blurred (thin bars only reach ~half), and faint grey (Y-like:
     * bars 550, gaps 300 - never black/white for fixed 400/600 limits). */
    shape_t      shapes[6];
    unsigned int shape_index;
    unsigned int letter_index;
    unsigned int dir;
    shapes[0]           = g_sharp;
    shapes[1]           = g_sharp;
    shapes[1].narrow_us = 60000U;
    shapes[1].wide_us   = 150000U;
    shapes[1].blur_us   = 60000U;
    shapes[1].noise     = 15U;
    shapes[2]           = g_sharp;
    shapes[2].narrow_us = 50000U;
    shapes[2].wide_us   = 130000U;
    shapes[2].bar       = 550U;
    shapes[2].gap       = 300U;
    shapes[2].blur_us   = 20000U;
    shapes[2].noise     = 10U;

    for (shape_index = 0U; shape_index < BASE_SCAN_SHAPE_COUNT; shape_index++)
    {
        shapes[shape_index + 3U]           = shapes[shape_index];
        shapes[shape_index + 3U].period_us = 5000U;
    }

    for (shape_index = 0U; shape_index < (sizeof shapes / sizeof shapes[0]);
         shape_index++)
    {
        for (letter_index = 0U; letter_index < FIXTURE_ALPHABET_COUNT;
             letter_index++)
        {
            for (dir = 0U; dir < SCAN_DIRECTION_COUNT; dir++)
            {
                static bc_decoder_record_t record;
                uint32_t                   runs[29];

                uint32_t clock =
                    0xFFFF0000U; /* Cross the 32-bit timer rollover. */
                tally_t outcomes;
                make_runs((char)((unsigned)('A') + letter_index),
                          &shapes[shape_index], runs);

                if (dir)
                {
                    reverse_runs(runs);
                }

                bc_decoder_record_reset(&record);
                outcomes =
                    feed(&record, &clock, &shapes[shape_index], runs, 29U);

                if (!((((1U == outcomes.decoded) &&
                        (outcomes.last.letter ==
                         (char)((unsigned)('A') + letter_index))) &&
                       (outcomes.last.b_is_reversed == (dir != 0U))) &&
                      (0U == outcomes.rejected)))
                {
                    printf("shape %u letter %c dir %u: decoded %u (%c) "
                           "rejected %u\n",
                           shape_index, (unsigned)('A') + letter_index, dir,
                           outcomes.decoded,
                           outcomes.last.letter ? outcomes.last.letter : '-',
                           outcomes.rejected);
                    assert(0);
                }

                assert(!bc_decoder_record_active(&record));
            }
        }
    }
}

/** @brief Exercise long frames sampled every five milliseconds and check the
 * expected payload and rejection count.
 */
static void test_200hz_long_frame (void)
{
    /* A 5.95-second frame plus its real quiet zone exceeds the old 1024
     * readings at 200 Hz. It must survive without wrapping the count or
     * decoding early, in either travel direction. */
    static bc_decoder_record_t record;
    shape_t                    shape      = g_sharp;
    const char                 payloads[] = {'A', 'Z'};
    unsigned int               letter_index;
    unsigned int               dir;
    unsigned int               index;
    shape.period_us = 5000U;
    shape.narrow_us = 140000U;
    shape.wide_us   = 350000U;
    shape.trail_us  = 1800000U;

    for (letter_index = 0U; letter_index < sizeof payloads; letter_index++)
    {
        for (dir = 0U; dir < SCAN_DIRECTION_COUNT; dir++)
        {
            uint32_t runs[29];
            uint32_t clock    = 0U;
            uint32_t frame_us = 0U;
            tally_t  outcomes;
            make_runs(payloads[letter_index], &shape, runs);

            if (dir)
            {
                reverse_runs(runs);
            }

            for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
            {
                frame_us += runs[index];
            }

            bc_decoder_record_reset(&record);
            outcomes = feed(&record, &clock, &shape, runs, 29U);
            assert(outcomes.max_recorded > 1024U);
            assert(1U == outcomes.decoded &&
                   outcomes.last.letter == payloads[letter_index]);
            assert(outcomes.last.b_is_reversed == (dir != 0U));
            assert(0U == outcomes.rejected && 0U == outcomes.timing &&
                   0U == outcomes.overflow);
            assert(outcomes.last.timestamp_us >=
                   shape.lead_us + frame_us + 10U * shape.narrow_us);
            assert(!bc_decoder_record_active(&record));
        }
    }
    /* More readings must not relax the existing physical-width rules or
     * invent the opening sentinel in a late-started capture. */
    {
        uint32_t runs[29];
        uint32_t clock = 0U;
        tally_t  outcomes;
        shape           = g_sharp;
        shape.period_us = 5000U;
        shape.narrow_us = 20000U;
        shape.wide_us   = 60000U;
        make_runs('A', &shape, runs);
        bc_decoder_record_reset(&record);
        outcomes = feed(&record, &clock, &shape, runs, 29U);
        assert(0U == outcomes.decoded && 1U == outcomes.rejected &&
               0U == outcomes.overflow);
        shape           = g_sharp;
        shape.period_us = 5000U;
        make_runs('A', &shape, runs);
        bc_decoder_record_reset(&record);
        clock    = 0U;
        outcomes = feed(&record, &clock, &shape, &runs[10], 19U);
        assert(0U == outcomes.decoded && 1U == outcomes.rejected &&
               0U == outcomes.overflow);
        /* Faint bars below the former 150-unit start gate now start a full
         * recording; the same physical trace works at either sample rate. */
        shape.bar = shape.paper + 100U;
        bc_decoder_record_reset(&record);
        clock    = 0U;
        outcomes = feed(&record, &clock, &shape, runs, 29U);
        assert(1U == outcomes.decoded && 'A' == outcomes.last.letter &&
               0U == outcomes.rejected);
    }
}

/** @brief Compare fixed-level streaming detection with the recorded reader
 * on a synthetic faint Y and require the recorded decode.
 */
static void test_faint_y_beats_fixed_limits (void)
{
    /* The fixed 400/600 reader sees no black at all in the faint trace. */
    shape_t                    shape = g_sharp;
    bc_decoder_t               live;
    bc_decoder_config_t        cfg = bc_decoder_defaults();
    bc_decoder_result_t        res;
    uint32_t                   runs[29];
    uint32_t                   time_offset_us = 0U;
    uint32_t                   clock          = 0U;
    uint32_t                   total          = 0U;
    unsigned int               index;
    unsigned int               live_decoded = 0U;
    static bc_decoder_record_t record;
    shape.bar = 550U;
    shape.gap = 300U;
    make_runs('Y', &shape, runs);
    assert(bc_decoder_init(&live, &cfg));

    for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
    {
        total += runs[index];
    }

    total += shape.lead_us + shape.trail_us;

    for (time_offset_us  = 0U; time_offset_us < total;
         time_offset_us += shape.period_us)
    {
        uint16_t    darkness   = ideal(&shape, runs, 29U, time_offset_us);
        static bool b_is_black = false;

        if (darkness >= 600U)
        {
            b_is_black = true;
        }
        else if (darkness <= 400U)
        {
            b_is_black = false;
        }
        else
        {
            /* Retain the previous black or white classification in the
             * dead band. */
        }

        if (bc_decoder_update(&live, 1000U + time_offset_us, true, b_is_black,
                              &res) == (unsigned)S3_BARCODE_DECODED)
        {
            live_decoded++;
        }
    }

    assert(0U == live_decoded);
    bc_decoder_record_reset(&record);
    {
        tally_t outcomes = feed(&record, &clock, &shape, runs, 29U);
        assert(1U == outcomes.decoded && 'Y' == outcomes.last.letter);
    }
}

/** @brief Exercise faint A and Z starts at the tested cadences in both
 * directions and check the letter, orientation and error counts.
 */
static void test_faint_recording_start (void)
{
    /* The complete frame is visible but every bar is below the old start
     * threshold. Long real paper exercises history wrap before the frame. */
    static bc_decoder_record_t record;
    const char                 payloads[] = {'A', 'Z'};
    unsigned int               rate;
    unsigned int               letter_index;
    unsigned int               dir;

    for (rate = 0U; rate < SAMPLING_RATE_CASE_COUNT; rate++)
    {
        for (letter_index = 0U; letter_index < sizeof payloads; letter_index++)
        {
            for (dir = 0U; dir < SCAN_DIRECTION_COUNT; dir++)
            {
                shape_t  shape = g_sharp;
                uint32_t runs[29];
                uint32_t clock = 0xFFFF0000U;
                tally_t  outcomes;
                shape.period_us = rate ? 5000U : 10000U;
                shape.narrow_us = 80000U;
                shape.wide_us   = 200000U;
                shape.bar       = 110U;
                shape.blur_us   = 40000U;
                shape.lead_us   = 4000000U;
                make_runs(payloads[letter_index], &shape, runs);

                if (dir)
                {
                    reverse_runs(runs);
                }

                bc_decoder_record_reset(&record);
                outcomes = feed(&record, &clock, &shape, runs, 29U);
                assert(1U == outcomes.decoded &&
                       outcomes.last.letter == payloads[letter_index]);
                assert(outcomes.last.b_is_reversed == (dir != 0U));
                assert(0U == outcomes.rejected && 0U == outcomes.timing &&
                       0U == outcomes.overflow);
            }
        }
    }
}

/** @brief Feed a faint opening star 40 units above paper followed by a
 * stronger payload bar. Fault 1 injects an invalid sample and fault
 * 2 omits a sample before recording starts.
 */
static tally_t feed_weak_opening (bc_decoder_record_t * p_report,
                                  uint32_t * p_clock, const shape_t * p_shape,
                                  const uint32_t runs[29], unsigned int fault)
{
    const bc_decoder_config_t cfg         = bc_decoder_defaults();
    tally_t                   outcomes    = {0};
    uint32_t                  opening_end = p_shape->lead_us;
    uint32_t                  payload_start;
    uint32_t                  total;
    uint32_t                  sample_time_us;
    unsigned int              index;

    for (index = 0U; index < CODE39_ELEMENT_COUNT; index++)
    {
        opening_end += runs[index];
    }

    payload_start = opening_end + runs[9];
    total         = p_shape->lead_us + p_shape->trail_us;

    for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index++)
    {
        total += runs[index];
    }

    for (sample_time_us  = 0U; sample_time_us < total;
         sample_time_us += p_shape->period_us)
    {
        uint16_t   dark = ideal(p_shape, runs, 29U, sample_time_us);
        const bool b_is_fault_sample =
            (sample_time_us + p_shape->period_us) == payload_start;
        bc_decoder_result_t result;
        bc_decoder_status_t status;

        if ((sample_time_us < opening_end) && (dark == p_shape->bar))
        {
            dark = p_shape->paper + 40U;
        }

        /* clang-format off */
        *p_clock +=
            ((2U == fault) && b_is_fault_sample) ? 40000U : p_shape->period_us;
        status    = bc_decoder_record_update(p_report, &cfg, *p_clock,
                                          !((1U == fault) && b_is_fault_sample),
                                          dark, &result);
        /* clang-format on */

        if (p_report->count > outcomes.max_recorded)
        {
            outcomes.max_recorded = p_report->count;
        }

        if ((unsigned)S3_BARCODE_DECODED == status)
        {
            outcomes.decoded++;
            outcomes.last = result;
        }
        else if ((unsigned)S3_BARCODE_REJECTED == status)
        {
            outcomes.rejected++;
        }
        else if ((unsigned)S3_BARCODE_TIMING_ERROR == status)
        {
            outcomes.timing++;
        }
        else if ((unsigned)S3_BARCODE_OVERFLOW == status)
        {
            outcomes.overflow++;
        }
        else
        {
            /* This status does not add to the scan totals. */
        }
    }

    return outcomes;
}

/** @brief Exercise delayed starts with recorded leading samples. Check
 * decoding and the effect of missing or invalid pre-trigger
 * readings.
 */
static void test_real_pretrigger_history (void)
{
    static bc_decoder_record_t record;
    const char                 payloads[] = {'A', 'Z'};
    unsigned int               rate;
    unsigned int               letter_index;
    unsigned int               dir;
    unsigned int               fault;

    for (rate = 0U; rate < SAMPLING_RATE_CASE_COUNT; rate++)
    {
        for (letter_index = 0U; letter_index < sizeof payloads; letter_index++)
        {
            for (dir = 0U; dir < SCAN_DIRECTION_COUNT; dir++)
            {
                shape_t  frame_shape = g_sharp;
                uint32_t runs[29];
                uint32_t clock = 0xFFFF0000U;
                tally_t  outcomes;
                /* clang-format off */
                frame_shape.period_us = rate ? 5000U : 10000U;
                frame_shape.narrow_us = 80000U;
                frame_shape.wide_us   = 200000U;
                frame_shape.lead_us   =
                    4000000U; /* Wrap around real quiet. */
                /* clang-format on */
                make_runs(payloads[letter_index], &frame_shape, runs);

                if (dir)
                {
                    reverse_runs(runs);
                }

                bc_decoder_record_reset(&record);
                outcomes =
                    feed_weak_opening(&record, &clock, &frame_shape, runs, 0U);

                if (!((1U == outcomes.decoded) &&
                      (outcomes.last.letter == payloads[letter_index])))
                {
                    printf("weak opening rate=%u letter=%c dir=%u: decoded=%u "
                           "got=%c rejected=%u\n",
                           rate, payloads[letter_index], dir, outcomes.decoded,
                           outcomes.last.letter ? outcomes.last.letter : '-',
                           outcomes.rejected);
                    assert(0);
                }

                assert(outcomes.last.b_is_reversed == (dir != 0U));
                assert(0U == outcomes.rejected && 0U == outcomes.timing &&
                       0U == outcomes.overflow);
                /* The opening star is not extra leading white. Only 600 ms
                 * of real paper precedes it: below ten 80 ms modules. */
                frame_shape.lead_us = 600000U;
                clock               = 0U;
                bc_decoder_record_reset(&record);
                outcomes =
                    feed_weak_opening(&record, &clock, &frame_shape, runs, 0U);
                assert(0U == outcomes.decoded && 1U == outcomes.rejected);
            }
        }
        /* Neither invalid data nor a sampling gap may join old buffered
         * opening bars onto a later payload to invent a complete frame. */
        for (fault = 1U; fault <= LAST_INTERRUPTION_CASE; fault++)
        {
            shape_t  frame_shape = g_sharp;
            uint32_t runs[29];
            uint32_t clock = 0U;
            tally_t  outcomes;
            frame_shape.period_us = rate ? 5000U : 10000U;
            frame_shape.narrow_us = 80000U;
            frame_shape.wide_us   = 200000U;
            frame_shape.lead_us   = 2000000U;
            make_runs('A', &frame_shape, runs);
            bc_decoder_record_reset(&record);
            outcomes =
                feed_weak_opening(&record, &clock, &frame_shape, runs, fault);
            assert(0U == outcomes.decoded && 1U == outcomes.rejected &&
                   0U == outcomes.overflow);
        }
    }
    /* A delayed trigger on the top of a blurred, faint first bar must still
     * retain both sides of its edge. Check measured widths before decoding
     * so sentinel correction cannot conceal a clipped opening edge. */
    {
        shape_t  frame_shape = g_sharp;
        uint32_t runs[29];
        uint32_t measured[29];
        uint32_t first;
        uint32_t last;
        uint32_t end;
        uint32_t gap;
        uint32_t clock = 0U;
        tally_t  outcomes;
        frame_shape.narrow_us = 120000U;
        frame_shape.wide_us   = 300000U;
        frame_shape.bar       = 70U;
        frame_shape.blur_us   = 120000U;
        frame_shape.trail_us  = 200000U;
        make_runs('A', &frame_shape, runs);
        bc_decoder_record_reset(&record);
        outcomes = feed(&record, &clock, &frame_shape, runs, 29U);
        assert(0U == outcomes.decoded && 0U == outcomes.rejected &&
               bc_decoder_record_active(&record));
        assert(bc_decoder_record_runs(&record, measured, &first, &last, &end,
                                      &gap) == 29U);
        assert(measured[0] >= 110000U && measured[0] <= 130000U);
        assert(first > 0U && last < end);
    }
    /* Saved paper before the trigger must not consume the existing frame
     * deadline. This 8.1 s barcode needs another 1.8 s of actual white;
     * adding the buffered paper would incorrectly push it over 10 s. */
    {
        shape_t  frame_shape = g_sharp;
        uint32_t runs[29];
        uint32_t clock = 0U;
        tally_t  outcomes;
        frame_shape.narrow_us = 180000U;
        frame_shape.wide_us   = 500000U;
        frame_shape.lead_us   = 2500000U;
        frame_shape.trail_us  = 2100000U;
        make_runs('A', &frame_shape, runs);
        bc_decoder_record_reset(&record);
        outcomes = feed(&record, &clock, &frame_shape, runs, 29U);
        assert(1U == outcomes.decoded && 'A' == outcomes.last.letter);
        assert(0U == outcomes.rejected && 0U == outcomes.timing &&
               0U == outcomes.overflow);
    }
}

/** @brief Return the reader to paper after faint activity and check that it
 * accepts the following A scan once.
 */
static void test_faint_rearm_on_paper (void)
{
    static bc_decoder_record_t record;
    const bc_decoder_config_t  cfg   = bc_decoder_defaults();
    uint32_t                   clock = 0U;
    unsigned int               index;
    unsigned int               rejected = 0U;
    bc_decoder_record_reset(&record);

    for (index = 0U; index < LOW_CONTRAST_SAMPLE_COUNT; index++)
    {
        const uint16_t dark = (index < 200U) || (index >= 600U) ? 10U : 70U;
        bc_decoder_result_t result;
        bc_decoder_status_t status;
        clock += 10000U;
        status =
            bc_decoder_record_update(&record, &cfg, clock, true, dark, &result);
        assert(status != (unsigned)S3_BARCODE_DECODED &&
               status != (unsigned)S3_BARCODE_OVERFLOW);

        if ((unsigned)S3_BARCODE_REJECTED == status)
        {
            rejected++;
        }
    }

    assert(1U == rejected && !bc_decoder_record_active(&record));
    /* A real subsequent frame is still allowed after actual white paper. */
    {
        uint32_t runs[29];
        tally_t  outcomes;
        make_runs('A', &g_sharp, runs);
        outcomes = feed(&record, &clock, &g_sharp, runs, 29U);
        assert(1U == outcomes.decoded && 'A' == outcomes.last.letter &&
               0U == outcomes.rejected);
    }
}

/** @brief Inject a scratch into a wide bar of a Z waveform and check that
 * the reader still publishes exactly one Z.
 */
static void test_scratch_inside_wide_bar (void)
{
    /* A 20 ms scratch to darkness 450 in the middle of a wide bar adds a
     * false dip; the reader trims it and still reads the letter. */
    static bc_decoder_record_t record;
    const bc_decoder_config_t  cfg = bc_decoder_defaults();
    uint32_t                   runs[29];
    uint32_t                   clock = 0U;
    uint32_t                   time_offset_us;
    uint32_t                   total = 0U;
    uint32_t                   edge;
    unsigned int               index;
    unsigned int               decoded = 0U;
    char                       got     = 0;
    shape_t                    shape   = g_sharp;
    shape.narrow_us                    = 140000U;
    shape.wide_us = 420000U; /* Scan a large barcode slowly. */
    make_runs('Z', &shape, runs);

    for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
    {
        total += runs[index];
    }

    total += shape.lead_us + shape.trail_us;
    edge   = shape.lead_us;

    for (index = 0U; index < WIDE_BAR_RUN_INDEX; index++)
    {
        edge += runs[index];
    }

    edge += runs[14] / 2U; /* Use the centre of wide bar 14 in Z. */
    assert(runs[14] == shape.wide_us);
    bc_decoder_record_reset(&record);

    for (time_offset_us  = 0U; time_offset_us < total;
         time_offset_us += shape.period_us)
    {
        bc_decoder_result_t res;
        uint16_t            darkness = ideal(&shape, runs, 29U, time_offset_us);

        if ((time_offset_us >= edge) && (time_offset_us < (edge + 20000U)))
        {
            darkness = 450U;
        }

        clock += shape.period_us;

        if (bc_decoder_record_update(&record, &cfg, clock, true, darkness,
                                     &res) == (unsigned)S3_BARCODE_DECODED)
        {
            decoded++;
            got = res.letter;
        }
    }

    assert(1U == decoded && 'Z' == got);
}

/** @brief Feed a wide Z bar with a grey shelf and require a single correct
 * result.
 */
static void test_grey_shelf_in_wide_bar (void)
{
    /* Seen on the car (3 Nov, big Z): a wide bar whose first half reads grey
     * (~550) and second half black. Its edge is where it leaves the paper,
     * not the middle of the grey shelf. */
    static bc_decoder_record_t record;
    const bc_decoder_config_t  cfg = bc_decoder_defaults();
    uint32_t                   runs[29];
    uint32_t                   clock = 0U;
    uint32_t                   time_offset_us;
    uint32_t                   total = 0U;
    uint32_t                   start;
    unsigned int               index;
    unsigned int               decoded = 0U;
    unsigned int               bar;
    char                       got   = 0;
    shape_t                    shape = g_sharp;
    shape.narrow_us                  = 140000U;
    shape.wide_us                    = 420000U;
    make_runs('Z', &shape, runs);

    for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
    {
        total += runs[index];
    }

    total += shape.lead_us + shape.trail_us;

    for (bar = 0U; bar < (sizeof runs / sizeof runs[0]); bar += 2U)
    {
        if (runs[bar] != shape.wide_us)
        {
            continue;
        }

        start = shape.lead_us;

        for (index = 0U; index < bar; index++)
        {
            start += runs[index];
        }

        bc_decoder_record_reset(&record);
        clock   = 0U;
        decoded = 0U;

        for (time_offset_us  = 0U; time_offset_us < total;
             time_offset_us += shape.period_us)
        {
            bc_decoder_result_t res;
            uint16_t darkness = ideal(&shape, runs, 29U, time_offset_us);

            if ((time_offset_us >= start) &&
                (time_offset_us < (start + (runs[bar] / 2U))))
            {
                darkness = 550U;
            }

            clock += shape.period_us;

            if (bc_decoder_record_update(&record, &cfg, clock, true, darkness,
                                         &res) == (unsigned)S3_BARCODE_DECODED)
            {
                decoded++;
                got = res.letter;
            }
        }

        assert(1U == decoded && 'Z' == got);
    }
}

/** @brief Feed an ideal printed frame with one rectangular reflectance
 * defect. Reverse the complete waveform to mirror both the defect
 * and the barcode.
 */
static tally_t feed_patched_frame (const shape_t * p_shape,
                                   const uint32_t  runs[29],
                                   uint32_t patch_start, uint32_t patch_us,
                                   uint16_t patch_dark, bool b_is_reversed)
{
    static bc_decoder_record_t record;
    const bc_decoder_config_t  cfg      = bc_decoder_defaults();
    tally_t                    outcomes = {0};
    uint32_t                   total    = p_shape->lead_us + p_shape->trail_us;
    uint32_t                   sample_time_us;
    uint32_t                   clock = 0U;
    unsigned int               index;

    for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index++)
    {
        total += runs[index];
    }

    assert(total % p_shape->period_us == 0U);
    bc_decoder_record_reset(&record);

    for (sample_time_us  = 0U; sample_time_us < total;
         sample_time_us += p_shape->period_us)
    {
        const uint32_t physical_t =
            b_is_reversed ? total - p_shape->period_us - sample_time_us
                          : sample_time_us;
        uint16_t            dark = ideal(p_shape, runs, 29U, physical_t);
        bc_decoder_result_t result;
        bc_decoder_status_t status;

        if ((physical_t >= patch_start) &&
            (physical_t < (patch_start + patch_us)))
        {
            dark = patch_dark;
        }

        clock += p_shape->period_us;
        status =
            bc_decoder_record_update(&record, &cfg, clock, true, dark, &result);

        if (record.count > outcomes.max_recorded)
        {
            outcomes.max_recorded = record.count;
        }

        if ((unsigned)S3_BARCODE_DECODED == status)
        {
            outcomes.decoded++;
            outcomes.last = result;
        }
        else if ((unsigned)S3_BARCODE_REJECTED == status)
        {
            outcomes.rejected++;
        }
        else if ((unsigned)S3_BARCODE_TIMING_ERROR == status)
        {
            outcomes.timing++;
        }
        else if ((unsigned)S3_BARCODE_OVERFLOW == status)
        {
            outcomes.overflow++;
        }
        else
        {
            /* This status does not add to the scan totals. */
        }
    }

    return outcomes;
}

/** @brief Inject a midpoint dip and rebound into wide bars and check the
 * decoder across letters and orientations.
 */
static void test_merged_wide_bar_dip (void)
{
    /* A broad, pale patch inside one wide black bar falls below its local
     * midpoint, then rebounds before the actual white gap. Pruning merges
     * this extra dip/bump pair; edge selection must retain the whole bar.
     * The old crossing nearest the weighted estimate selected the internal
     * rebound and clipped the bar. This shape and expectation do not depend
     * on the payload letter. Test each wide payload bar independently. */
    shape_t      shape = g_sharp;
    unsigned int letter_index;
    unsigned int bar;
    unsigned int dir;
    unsigned int cases = 0U;
    shape.narrow_us    = 100000U;
    shape.wide_us      = 300000U;

    for (letter_index = 0U; letter_index < FIXTURE_ALPHABET_COUNT;
         letter_index++)
    {
        uint32_t runs[29];
        uint32_t start = shape.lead_us;
        make_runs((char)((unsigned)('A') + letter_index), &shape, runs);

        for (bar = 0U; bar < (sizeof runs / sizeof runs[0]); bar++)
        {
            if ((((bar >= 10U) && (bar <= 18U)) && ((bar % 2U) == 0U)) &&
                (runs[bar] == shape.wide_us))
            {
                for (dir = 0U; dir < SCAN_DIRECTION_COUNT; dir++)
                {
                    const tally_t outcomes = feed_patched_frame(
                        &shape, runs, start + 60000U, 180000U, 100U, dir != 0U);

                    if (!((((((1U == outcomes.decoded) &&
                              (outcomes.last.letter ==
                               (char)((unsigned)('A') + letter_index))) &&
                             (outcomes.last.b_is_reversed == (dir != 0U))) &&
                            (0U == outcomes.rejected)) &&
                           (0U == outcomes.timing)) &&
                          (0U == outcomes.overflow)))
                    {
                        printf("merged wide-bar dip: letter=%c bar=%u dir=%u "
                               "decoded=%u got=%c rejected=%u\n",
                               (unsigned)('A') + letter_index, bar, dir,
                               outcomes.decoded,
                               outcomes.last.letter ? outcomes.last.letter
                                                    : '-',
                               outcomes.rejected);
                        fflush(stdout);
                        assert(0);
                    }

                    cases++;
                }
            }

            start += runs[bar];
        }
    }

    printf("PASS: merged wide-bar midpoint dip and rebound, %u "
           "A-Z/both-direction scans\n",
           cases);
}

/** @brief Place narrow spikes in gaps and check that accepted scans report
 * the correct letter. Damaged scans may be rejected.
 */
static void test_gap_spike_letter (void)
{
    /* The outer-crossing rule must not turn a brief dark smudge in white
     * paper into a different letter. A 20 ms spike is above the local
     * midpoint, near either side of every narrow payload bar. Refusal is
     * allowed: Code 39 has no checksum and the pristine frame was altered. */
    shape_t      shape = g_sharp;
    unsigned int letter_index;
    unsigned int bar;
    unsigned int dir;
    unsigned int side;
    unsigned int offset;
    unsigned int cases   = 0U;
    unsigned int decoded = 0U;
    shape.narrow_us      = 100000U;
    shape.wide_us        = 300000U;

    for (letter_index = 0U; letter_index < FIXTURE_ALPHABET_COUNT;
         letter_index++)
    {
        uint32_t runs[29];
        uint32_t start = shape.lead_us;
        make_runs((char)((unsigned)('A') + letter_index), &shape, runs);

        for (bar = 0U; bar < (sizeof runs / sizeof runs[0]); bar++)
        {
            if ((((bar >= 10U) && (bar <= 18U)) && ((bar % 2U) == 0U)) &&
                (runs[bar] == shape.narrow_us))
            {
                for (side = 0U; side < PATCH_SIDE_COUNT; side++)
                {
                    for (offset = MIN_PATCH_OFFSET_US;
                         offset <= MAX_PATCH_OFFSET_US; offset += 40000U)
                    {
                        const uint32_t patch_start =
                            side ? start + runs[bar] + offset
                                 : start - offset - 20000U;

                        for (dir = 0U; dir < SCAN_DIRECTION_COUNT; dir++)
                        {
                            const tally_t outcomes =
                                feed_patched_frame(&shape, runs, patch_start,
                                                   20000U, 700U, dir != 0U);
                            assert(outcomes.decoded <= 1U &&
                                   0U == outcomes.timing &&
                                   0U == outcomes.overflow);
                            assert(outcomes.decoded + outcomes.rejected == 1U);

                            if (outcomes.decoded != 0U)
                            {
                                assert(outcomes.last.letter ==
                                       (char)((unsigned)('A') + letter_index));
                                assert(outcomes.last.b_is_reversed ==
                                       (dir != 0U));
                                decoded++;
                            }

                            cases++;
                        }
                    }
                }
            }

            start += runs[bar];
        }
    }

    printf("PASS: isolated 20 ms white-gap spikes, %u scans (%u correct, "
           "remainder rejected; no wrong letter)\n",
           cases, decoded);
}

/** @brief Run 3000 deterministic damaged scans. Require at least one correct
 * decode and no incorrect letter.
 */
static void test_never_a_wrong_letter (void)
{
    /* Vary stripe widths by up to 25 percent in either direction and apply
     * randomized levels, blur and noise. A test may reject a frame but may
     * only decode the intended letter. Earlier offline testing saw no wrong
     * letters in 30,000 frames at 25 percent and three wrong letters in
     * 30,000 frames at 35 percent. Code 39 has no check digit, so sufficient
     * width changes can form another valid letter.
     */
    unsigned int trial;
    unsigned int wrong = 0U;
    unsigned int right = 0U;

    for (trial = 0U; trial < RANDOM_FRAME_TRIAL_COUNT; trial++)
    {
        static bc_decoder_record_t record;
        shape_t                    shape = g_sharp;
        uint32_t                   runs[29];
        uint32_t                   clock = 0U;
        unsigned int               index;
        char    letter = (char)((unsigned)('A') + (next_rand() % 26U));
        tally_t outcomes;
        shape.narrow_us = 40000U + ((next_rand() % 40U) * 1000U);
        shape.wide_us   = shape.narrow_us * (20U + (next_rand() % 15U)) / 10U;
        shape.bar       = (uint16_t)(450U + (next_rand() % 550U));
        shape.gap       = (uint16_t)(next_rand() % 350U);
        shape.blur_us   = (next_rand() % 50U) * 1000U;
        shape.noise     = (uint16_t)(next_rand() % 40U);
        make_runs(letter, &shape, runs);

        for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
        {
            runs[index] = runs[index] * (75U + (next_rand() % 51U)) / 100U;
        }

        bc_decoder_record_reset(&record);
        outcomes = feed(&record, &clock, &shape, runs, 29U);

        if (outcomes.decoded)
        {
            if (outcomes.last.letter == letter)
            {
                right++;
            }
            else
            {
                wrong++;
            }
        }
    }

    assert(0U == wrong);
    assert(right > 0U);
}

/** @brief Compute the darkness of one synthetic bar sample.
 * @param p_shape Borrowed fixture shape with the bar width and sample period.
 * @param index Index of the bar run in this frame.
 * @param dips Number of shallow dips per wide bar, possibly zero.
 * @param start_us Bar start time in microseconds.
 * @param run_width_us Duration of the current bar in microseconds.
 * @param sample_us Physical sample time in microseconds.
 * @return Darkness on the decoder's normalized scale, before quantization.
 */
static uint16_t rippled_bar_darkness (const shape_t * p_shape,
                                      unsigned int index, unsigned int dips,
                                      uint32_t start_us, uint32_t run_width_us,
                                      uint32_t sample_us)
{
    uint16_t dark = (uint16_t)(880U + ((index % 5U) * 7U));

    if (run_width_us == p_shape->wide_us)
    {
        for (unsigned int dip = 1U; dip <= dips; dip++)
        {
            const uint32_t dip_time_us =
                start_us + (dip * p_shape->wide_us / (dips + 1U));

            if ((sample_us >= dip_time_us) &&
                (sample_us < (dip_time_us + p_shape->period_us)))
            {
                dark = (uint16_t)(740U + ((dip % 3U) * 9U));
            }
        }
    }

    return dark;
}

/** @brief Feed a synthetic frame with wide-bar dips and count decoder outcomes.
 * @param letter Encoded Code 39 symbol.
 * @param b_is_reverse Scan direction.
 * @param dips Number of simulated dips per wide bar.
 * @param damage Scenario controlling which quiet zone or bar is disturbed.
 * @param b_is_quantized Whether to quantize the synthetic ADC darkness by four.
 * @return Decoder outcome counts for this frame.
 * @note Four dips per wide bar add 48 extrema to the original 31. Nine dips
 * exceed the cleanup workspace. A paper level of 100 exercises primary
 * extraction. The dips stay above the midpoint when the waveform is reversed.
 */
static tally_t feed_rippled_frame (char letter, bool b_is_reverse,
                                   unsigned int dips, unsigned int damage,
                                   bool b_is_quantized)
{
    static bc_decoder_record_t record;
    const bc_decoder_config_t  cfg   = bc_decoder_defaults();
    shape_t                    shape = g_sharp;
    uint32_t                   runs[29];
    uint32_t                   total;
    uint32_t                   time_offset_us;
    uint32_t                   clock   = 0xffff0000U;
    unsigned int               missing = 29U;
    unsigned int               index;
    tally_t                    outcomes = {0};
    shape.narrow_us                     = 100000U;
    shape.wide_us                       = 300000U;
    shape.paper                         = 100U;
    shape.gap                           = 100U;
    shape.bar                           = 900U;

    if ((unsigned)RIPPLE_SHORT_LEAD == damage)
    {
        shape.lead_us = 200000U;
    }

    make_runs(letter, &shape, runs);

    if ((unsigned)RIPPLE_BAD_START == damage)
    {
        runs[0] = shape.wide_us;
    }

    if ((unsigned)RIPPLE_BAD_STOP == damage)
    {
        runs[28] = shape.wide_us;
    }

    for (index = PAYLOAD_FIRST_RUN; index < PAYLOAD_END_RUN; index += 2U)
    {
        if (runs[index] == shape.narrow_us)
        {
            missing = index;
            break;
        }
    }

    assert(missing < 29U);
    total = shape.lead_us + shape.trail_us;

    for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
    {
        total += runs[index];
    }

    bc_decoder_record_reset(&record);

    for (time_offset_us  = 0U; time_offset_us < total;
         time_offset_us += shape.period_us)
    {
        const uint32_t physical =
            (b_is_reverse ? total - shape.period_us - time_offset_us
                          : time_offset_us);
        uint32_t            start = shape.lead_us;
        uint16_t            dark  = shape.paper;
        bc_decoder_result_t result;
        bc_decoder_status_t status;

        for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
        {
            if ((physical >= start) && (physical < (start + runs[index])))
            {
                if (((index % 2U) == 0U) &&
                    (!(((unsigned)RIPPLE_MISSING_BAR == damage) &&
                       (index == missing))))
                {
                    dark = rippled_bar_darkness(&shape, index, dips, start,
                                                runs[index], physical);
                }

                break;
            }

            start += runs[index];
        }

        if ((((unsigned)RIPPLE_BROKEN_TRAIL == damage) &&
             (physical >= (total - shape.trail_us + 200000U))) &&
            (physical < (total - shape.trail_us + 280000U)))
        {
            dark = 900U;
        }

        if (b_is_quantized)
        {
            dark = (uint16_t)((dark / 4U) * 4U);
        }

        clock += shape.period_us;
        status =
            bc_decoder_record_update(&record, &cfg, clock, true, dark, &result);

        if ((unsigned)S3_BARCODE_DECODED == status)
        {
            outcomes.decoded++;
            outcomes.last = result;
        }
        else if ((unsigned)S3_BARCODE_REJECTED == status)
        {
            outcomes.rejected++;
        }
        else if ((unsigned)S3_BARCODE_TIMING_ERROR == status)
        {
            outcomes.timing++;
        }
        else if ((unsigned)S3_BARCODE_OVERFLOW == status)
        {
            outcomes.overflow++;
        }
        else
        {
            /* This status does not add to the scan totals. */
        }
    }

    return outcomes;
}

/** @brief Exercise dense ripple, quantization and damaged frames in both
 * directions. Check the marker, quiet-zone and workspace limits.
 */
static void test_dense_wide_bar_ripples (void)
{
    unsigned int letter_index;
    unsigned int dir;
    unsigned int quantized;
    unsigned int damage;
    unsigned int checks = 0U;

    for (letter_index = 0U; letter_index < FIXTURE_ALPHABET_COUNT;
         letter_index++)
    {
        for (dir = 0U; dir < SCAN_DIRECTION_COUNT; dir++)
        {
            for (quantized = 0U; quantized < QUANTIZATION_CASE_COUNT;
                 quantized++)
            {
                for (damage = RIPPLE_INTACT;
                     damage <= (unsigned)RIPPLE_BROKEN_TRAIL; damage++)
                {
                    const tally_t outcomes = feed_rippled_frame(
                        (char)((unsigned)('A') + letter_index), dir != 0U, 4U,
                        damage, quantized != 0U);
                    assert(0U == outcomes.timing && 0U == outcomes.overflow);

                    if ((unsigned)RIPPLE_INTACT == damage)
                    {
                        if (!((((1U == outcomes.decoded) &&
                                (outcomes.last.letter ==
                                 (char)((unsigned)('A') + letter_index))) &&
                               (outcomes.last.b_is_reversed == (dir != 0U))) &&
                              (0U == outcomes.rejected)))
                        {
                            printf(
                                "dense ripple: letter=%c dir=%u quantized=%u "
                                "decoded=%u got=%c rejected=%u\n",
                                (unsigned)('A') + letter_index, dir, quantized,
                                outcomes.decoded,
                                outcomes.last.letter ? outcomes.last.letter
                                                     : '-',
                                outcomes.rejected);
                            fflush(stdout);
                            assert(0);
                        }
                    }
                    else
                    {
                        assert(0U == outcomes.decoded);
                    }

                    checks++;
                }
                /* Excessive ripple must fail closed; no data-dependent heap
                 * growth, silent truncation or selection of a known letter. */
                {
                    const tally_t outcomes = feed_rippled_frame(
                        (char)((unsigned)('A') + letter_index), dir != 0U, 9U,
                        RIPPLE_INTACT, quantized != 0U);
                    assert(0U == outcomes.decoded && 1U == outcomes.rejected &&
                           0U == outcomes.timing && 0U == outcomes.overflow);
                    checks++;
                }
            }
        }
    }

    printf("PASS: %u dense-ripple A-Z/direction/quantization and strict "
           "marker/bar/quiet/capacity checks\n",
           checks);
}

/** @brief Remove the first two runs of a Y and require one rejection with no
 * decode.
 */
static void test_cut_pass_is_not_read (void)
{
    /* The first two stripes were never seen: 27 stripes cannot decode. */
    static bc_decoder_record_t record;
    uint32_t                   runs[29];
    uint32_t                   clock = 0U;
    tally_t                    outcomes;
    make_runs('Y', &g_sharp, runs);
    bc_decoder_record_reset(&record);
    outcomes = feed(&record, &clock, &g_sharp, &runs[2], 27U);
    assert(0U == outcomes.decoded && 1U == outcomes.rejected);
}

/** @brief Exercise insufficient white paper and additional runs at the frame
 * boundary and check that invalid scans do not decode.
 */
static void test_short_quiet_zones (void)
{
    static bc_decoder_record_t record;
    uint32_t                   runs[29];
    uint32_t                   clock = 0U;
    tally_t                    outcomes;
    shape_t                    shape = g_sharp;
    make_runs('A', &shape, runs);
    shape.lead_us =
        200000U; /* Use less than ten narrow modules of leading quiet. */
    bc_decoder_record_reset(&record);
    outcomes = feed(&record, &clock, &shape, runs, 29U);
    assert(0U == outcomes.decoded);
    /* Trailing quiet broken by a dark patch 200 ms after the frame. */
    {
        uint32_t     ext[31];
        unsigned int index;

        for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
        {
            ext[index] = runs[index];
        }

        ext[29] = 200000U;
        ext[30] = 80000U; /* Follow the gap with another bar. */
        shape   = g_sharp;
        bc_decoder_record_reset(&record);
        outcomes = feed(&record, &clock, &shape, ext, 31U);
        assert(0U == outcomes.decoded);
    }
}

/** @brief Pass a lone simulated line across the reader and require one
 * rejection, no other terminal event and an inactive recorder.
 */
static void test_line_crossing (void)
{
    /* 1.2 s of black (a track line): rejected once, like the fixed reader,
     * and not reported again while it is still black or afterwards. */
    static bc_decoder_record_t record;
    const bc_decoder_config_t  cfg = bc_decoder_defaults();
    bc_decoder_result_t        res;
    uint32_t                   sample_time_us = 0U;
    unsigned int               index;
    unsigned int               rejected = 0U;
    unsigned int               other    = 0U;
    bc_decoder_record_reset(&record);

    for (index = 0U; index < TIMEOUT_SAMPLE_COUNT; index++)
    {
        uint16_t darkness = ((index < 100U) || (index >= 220U)) ? 10U : 1000U;
        bc_decoder_status_t status;
        /* clang-format off */
        sample_time_us += 10000U;
        status          =
            bc_decoder_record_update(&record, &cfg, sample_time_us, true,
                                          darkness, &res);
        /* clang-format on */

        if ((unsigned)S3_BARCODE_REJECTED == status)
        {
            rejected++;
            assert(index < 220U);
        }
        else if (status != (unsigned)S3_BARCODE_NONE)
        {
            other++;
        }
        else
        {
            /* No barcode outcome was reported on this update. */
        }
    }

    assert(1U == rejected && 0U == other && !bc_decoder_record_active(&record));
}

/** @brief Exercise invalid parameters, timing and storage bounds, including
 * saturation of out-of-range darkness to 1000.
 */
static void test_limits_and_faults (void)
{
    static bc_decoder_record_t record;
    const bc_decoder_config_t  cfg = bc_decoder_defaults();
    bc_decoder_config_t        bad = cfg;
    bc_decoder_result_t        res;
    uint32_t                   sample_time_us = 0U;
    unsigned int               index;
    unsigned int               overflow     = 0U;
    unsigned int               max_recorded = 0U;
    /* Recording memory full before the frame-duration timeout: 30 ms stripes
     * sampled every 2.5 ms, including after the 200 Hz capacity increase. */
    bc_decoder_record_reset(&record);

    for (index = 0U; index < FAST_LEAD_SAMPLE_COUNT; index++)
    {
        sample_time_us += 2500U;
        (void)bc_decoder_record_update(&record, &cfg, sample_time_us, true, 10U,
                                       &res);
    }

    for (index = 0U; index < (S3_REC_MAX + OVERFLOW_EXTRA_SAMPLES); index++)
    {
        sample_time_us += 2500U;

        if (bc_decoder_record_update(&record, &cfg, sample_time_us, true,
                                     ((index / 12U) % 2U) ? 50U : 900U,
                                     &res) == (unsigned)S3_BARCODE_OVERFLOW)
        {
            overflow++;
        }

        if (record.count > max_recorded)
        {
            max_recorded = record.count;
        }
    }
    /* The count must represent the full 4096-entry state before refusing the
     * next sample; a too-small count bitfield must not silently wrap to 0. */
    assert(S3_REC_MAX == max_recorded);
    assert(overflow >= 1U);
    /* A missing reading (gap > max_gap_us) while recording. */
    bc_decoder_record_reset(&record);
    sample_time_us = 0U;

    for (index = 0U; index < GAP_TEST_LEADING_PAPER_SAMPLES; index++)
    {
        sample_time_us += 10000U;
        assert(bc_decoder_record_update(&record, &cfg, sample_time_us, true,
                                        10U,
                                        &res) == (unsigned)S3_BARCODE_NONE);
    }

    sample_time_us += 10000U;
    (void)bc_decoder_record_update(&record, &cfg, sample_time_us, true, 900U,
                                   &res);
    assert(bc_decoder_record_active(&record));
    sample_time_us += 40000U;
    assert(bc_decoder_record_update(&record, &cfg, sample_time_us, true, 900U,
                                    &res) == (unsigned)S3_BARCODE_TIMING_ERROR);
    assert(!bc_decoder_record_active(&record));
    /* ... but only reported while a barcode was being read. */
    sample_time_us += 40000U;
    assert(bc_decoder_record_update(&record, &cfg, sample_time_us, true, 10U,
                                    &res) == (unsigned)S3_BARCODE_NONE);
    /* Invalid reading: start over silently. */
    sample_time_us += 10000U;
    (void)bc_decoder_record_update(&record, &cfg, sample_time_us, true, 900U,
                                   &res);
    assert(bc_decoder_record_active(&record));
    sample_time_us += 10000U;
    assert(bc_decoder_record_update(&record, &cfg, sample_time_us, false, 900U,
                                    &res) == (unsigned)S3_BARCODE_NONE);
    assert(!bc_decoder_record_active(&record) && !record.b_has_sample);
    /* Bad arguments. */
    bad.quiet_modules = 0U;
    assert(bc_decoder_record_update(&record, &bad, sample_time_us, true, 10U,
                                    &res) == (unsigned)S3_BARCODE_NONE);
    assert(bc_decoder_record_update(&record, &cfg, sample_time_us, true, 10U,
                                    0) == (unsigned)S3_BARCODE_NONE);
    assert(bc_decoder_record_update(0, &cfg, sample_time_us, true, 10U, &res) ==
           (unsigned)S3_BARCODE_NONE);
    assert(!bc_decoder_record_active(0));
    bc_decoder_record_reset(0);
    /* Darkness above 1000 is clamped, never stored as a wrapped value. */
    bc_decoder_record_reset(&record);
    sample_time_us = 0U;

    for (index = 0U; index < CLAMP_LEAD_SAMPLE_COUNT; index++)
    {
        sample_time_us += 10000U;
        (void)bc_decoder_record_update(&record, &cfg, sample_time_us, true, 10U,
                                       &res);
    }

    sample_time_us += 10000U;
    (void)bc_decoder_record_update(&record, &cfg, sample_time_us, true, 0xFFFFU,
                                   &res);
    assert(record.count >= 2U &&
           (record.sample[record.count - 1U] & 0x3FFU) == 1000U);
}

/** @brief Apply shared bar and gap width bias in both scan directions and
 * check rejection of invalid sentinel fits.
 */
static void test_shared_width_bias (void)
{
    unsigned int              letter_index;
    unsigned int              dir;
    unsigned int              sign;
    unsigned int              index;
    const bc_decoder_config_t cfg   = bc_decoder_defaults();
    shape_t                   shape = g_sharp;
    shape.narrow_us                 = 80000U;
    shape.wide_us                   = 200000U;

    for (letter_index = 0U; letter_index < FIXTURE_ALPHABET_COUNT;
         letter_index++)
    {
        for (dir = 0U; dir < SCAN_DIRECTION_COUNT; dir++)
        {
            for (sign = 0U; sign < WIDTH_BIAS_DIRECTION_COUNT; sign++)
            {
                uint32_t                   runs[29];
                uint32_t                   corrected[29];
                uint32_t                   clock = 0U;
                int32_t                    bias  = sign ? -40000 : 40000;
                int32_t                    fitted;
                static bc_decoder_record_t decoder_record;
                bc_decoder_result_t        result;
                tally_t                    outcomes;
                make_runs((char)((unsigned)('A') + letter_index), &shape, runs);

                if (dir)
                {
                    reverse_runs(runs);
                }

                for (index = 0U; index < (sizeof runs / sizeof runs[0]);
                     index++)
                {
                    runs[index] = (uint32_t)((int32_t)runs[index] +
                                             ((index % 2U) ? bias : -bias));
                }

                assert(!bc_decoder_decode_runs(runs, 29U, shape.lead_us,
                                               shape.trail_us, shape.period_us,
                                               &cfg, &result));
                memcpy(corrected, runs, sizeof(runs));
                assert(bc_decoder_record_correct_runs(
                    corrected, shape.period_us, &cfg, &fitted));
                assert(fitted >= bias - 1000 && fitted <= bias + 1000);
                bc_decoder_record_reset(&decoder_record);
                outcomes = feed(&decoder_record, &clock, &shape, runs, 29U);
                assert(1U == outcomes.decoded &&
                       outcomes.last.letter ==
                           (char)((unsigned)('A') + letter_index));
                assert(outcomes.last.b_is_reversed == (dir != 0U) &&
                       0U == outcomes.rejected);
            }
        }
    }
    /* Identical sentinels give an identical fit for every payload, including
     * an invalid one. A fit alone must never authorise a letter. */
    {
        uint32_t                   runs[29];
        uint32_t                   corrected[29];
        uint32_t                   clock = 0U;
        int32_t                    fitted;
        static bc_decoder_record_t decoder_record;
        tally_t                    outcomes;
        make_runs('A', &shape, runs);

        for (index = PAYLOAD_FIRST_RUN; index < PAYLOAD_END_RUN; index++)
        {
            runs[index] = shape.narrow_us;
        }

        for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
        {
            runs[index] =
                (index % 2U) ? runs[index] + 40000U : runs[index] - 40000U;
        }

        memcpy(corrected, runs, sizeof(runs));
        assert(bc_decoder_record_correct_runs(corrected, shape.period_us, &cfg,
                                              &fitted));
        assert(fitted >= 39000 && fitted <= 41000);
        bc_decoder_record_reset(&decoder_record);
        outcomes = feed(&decoder_record, &clock, &shape, runs, 29U);
        assert(0U == outcomes.decoded && 1U == outcomes.rejected);
        /* Opposite bias at the end cannot be explained by one shared fit. */
        make_runs('A', &shape, runs);

        for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
        {
            int32_t bias = (index < 20U) ? 40000 : -40000;
            runs[index]  = (uint32_t)((int32_t)runs[index] +
                                     ((index % 2U) ? bias : -bias));
        }

        memcpy(corrected, runs, sizeof(runs));
        assert(!bc_decoder_record_correct_runs(corrected, shape.period_us, &cfg,
                                               &fitted));
        assert(memcmp(corrected, runs, sizeof(runs)) == 0);
        /* Bar expansion consumes the leading margin as well. */
        make_runs('A', &shape, runs);

        for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
        {
            runs[index] =
                (index % 2U) ? runs[index] + 40000U : runs[index] - 40000U;
        }

        shape.lead_us =
            810000U; /* The uncorrected edge appears ten modules away. */
        bc_decoder_record_reset(&decoder_record);
        outcomes = feed(&decoder_record, &clock, &shape, runs, 29U);
        assert(0U == outcomes.decoded && 1U == outcomes.rejected);
    }
}

/** @brief Exercise adaptive peak extraction and quiet-zone decisions,
 * including scans whose validation remains pending.
 */
static void test_adaptive_peaks_and_quiet (void)
{
    const bc_decoder_config_t cfg = bc_decoder_defaults();
    unsigned int              mode;

    for (mode = 0U; mode < ADAPTIVE_QUIET_CASE_COUNT; mode++)
    {
        static bc_decoder_record_t record;
        shape_t                    shape    = g_sharp;
        const unsigned int         scenario = mode % 3U;
        uint32_t                   runs[29];
        uint32_t                   time_offset_us;
        uint32_t                   total;
        uint32_t                   finish;
        uint32_t                   clock = 0U;
        unsigned int               index;
        unsigned int               decoded      = 0U;
        bool                       b_is_pending = false;

        if (mode >= 3U)
        {
            shape.period_us = 5000U;
        }

        shape.narrow_us = 140000U;
        shape.wide_us   = 350000U;
        shape.trail_us  = 1800000U;
        make_runs('Y', &shape, runs);
        finish = shape.lead_us;

        for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
        {
            finish += runs[index];
        }

        total = finish + shape.trail_us;
        bc_decoder_record_reset(&record);

        for (time_offset_us  = 0U; time_offset_us < total;
             time_offset_us += shape.period_us)
        {
            uint16_t            dark = ideal(&shape, runs, 29U, time_offset_us);
            uint32_t            edge = shape.lead_us;
            bc_decoder_result_t result;
            bc_decoder_status_t status;
            /* Two genuinely faint bars, including the final bar: the fixed
             * 100-unit prominence loses them, the quiet-paper gate sees them.
             */
            for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
            {
                if (((time_offset_us >= edge) &&
                     (time_offset_us < (edge + runs[index]))) &&
                    ((10U == index) || (28U == index)))
                {
                    dark = 70U;
                }

                edge += runs[index];
            }

            if (((1U == scenario) && (time_offset_us >= (finish + 1100000U))) &&
                (time_offset_us < (finish + 1180000U)))
            {
                dark = 70U;
            }
            /* Same physical noise waveform at both sample rates. */
            if ((2U == scenario) && (time_offset_us < shape.lead_us))
            {
                dark = (time_offset_us / 10000U) % 2U ? 65U : 10U;
            }

            clock += shape.period_us;
            status = bc_decoder_record_update(&record, &cfg, clock, true, dark,
                                              &result);

            if ((S3_REC_PROCESSING == (int)record.state) ||
                (S3_REC_QUIET == (int)record.state))
            {
                b_is_pending = true;
            }

            if ((unsigned)S3_BARCODE_DECODED == status)
            {
                decoded++;
                assert('Y' == result.letter);
                assert(time_offset_us >=
                       finish + cfg.quiet_modules * shape.narrow_us -
                           shape.period_us);
            }
        }

        if (0U == scenario)
        {
            assert(1U == decoded && b_is_pending);
        }
        else
        {
            /* Break quiet and keep peaks below learned noise. */
            assert(0U == decoded);
        }

        if (1U == scenario)
        {
            assert(b_is_pending);
        }
    }
}

/* End of file. */

