/** @file wide_bar_main.c
 * @brief Synthetic physical-size coverage at 40 mm/s.
 *
 * These tests check the decoder's
 * permitted geometry/timing, not the dimensions or success of an unmeasured
 * printed barcode. One measured Z payload below has synthetic markers/quiet.
 * Unsigned casts spell out the original modulo conversion.
 */
#include "wide_bar_main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bc_decoder.h"

#define CODE39_ELEMENT_COUNT 9U
#define REVERSED_RUN_PAIRS 14U
#define FIXTURE_ALPHABET_COUNT 26U
#define SCAN_DIRECTION_COUNT 2U
#define TRAILING_SAMPLE_COUNT 100U

typedef struct
{
    unsigned int        decoded;
    unsigned int        rejected;
    unsigned int        timing;
    unsigned int        overflow;
    bc_decoder_result_t result;
} tally_t;

static const uint16_t g_letters[26] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
    0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
    0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0};

static bc_decoder_record_t g_record;
static uint32_t            g_clock_us;

static void    make_runs (unsigned int letter, uint32_t narrow, uint32_t wide,
                          uint32_t runs[29]);
static void    reverse_runs (uint32_t runs[29]);
static void    feed (const bc_decoder_config_t * p_config, uint16_t darkness,
                     tally_t * p_out);
static tally_t trace (const bc_decoder_config_t * p_config,
                      const uint32_t runs[29], unsigned int count,
                      uint32_t leading, uint32_t trailing);
static void    test_physical_wide_gaps (void);
static void    test_long_frame_quiet (void);
static bool    batch (const uint32_t runs[29], bc_decoder_result_t * p_result);
static void    test_extended_narrow_gap (void);
static void    test_measured_z_payload (void);

/** @brief Feed wide gaps and a measured Z payload to the barcode reader and
 * check the quiet-zone and marker constraints.
 */
int main (void)
{
    test_physical_wide_gaps();
    test_long_frame_quiet();
    test_extended_narrow_gap();
    test_measured_z_payload();
    puts("PASS: 468 synthetic A-Z scans at 8/14/19 mm wide gaps and 40 mm/s,"
         " real large-frame quiet, "
         "truncation/absolute bounds, one extended narrow payload white gap,"
         " strict markers/bars and measured Z payload");
    return 0;
}

/** @brief Build the 29 alternating durations for a star, the selected
 * payload and a terminating star using the supplied narrow and wide
 * widths.
 */
static void make_runs (unsigned int letter, uint32_t narrow, uint32_t wide,
                       uint32_t runs[29])
{
    const uint16_t symbols[3] = {0x094U, g_letters[letter], 0x094U};

    for (unsigned int symbol_index = 0U;
         symbol_index < (sizeof symbols / sizeof symbols[0]); symbol_index++)
    {
        for (unsigned int element_index = 0U;
             element_index < CODE39_ELEMENT_COUNT; element_index++)
        {
            runs[(symbol_index * 10U) + element_index] =
                (symbols[symbol_index] & (1U << (8U - element_index))) ? wide
                                                                       : narrow;
        }

        if (symbol_index < 2U)
        {
            runs[(symbol_index * 10U) + 9U] = narrow;
        }
    }
}

/** @brief Reverse all 29 run durations in place without changing their
 * widths.
 */
static void reverse_runs (uint32_t runs[29])
{
    for (unsigned int run_index = 0U; run_index < REVERSED_RUN_PAIRS;
         run_index++)
    {
        uint32_t value        = runs[run_index];
        runs[run_index]       = runs[28U - run_index];
        runs[28U - run_index] = value;
    }
}

/** @brief Advance the test clock by 10 ms and accumulate decoded, rejected,
 * timing and overflow results for one sample.
 */
static void feed (const bc_decoder_config_t * p_config, uint16_t darkness,
                  tally_t * p_out)
{
    bc_decoder_result_t result;
    g_clock_us += 10000U;

    switch (bc_decoder_record_update(&g_record, p_config, g_clock_us, true,
                                     darkness, &result))
    {
    case S3_BARCODE_DECODED:
        p_out->decoded++;
        p_out->result = result;
    break;
    case S3_BARCODE_REJECTED:
        p_out->rejected++;
    break;
    case S3_BARCODE_TIMING_ERROR:
        p_out->timing++;
    break;
    case S3_BARCODE_OVERFLOW:
        p_out->overflow++;
    break;
    default:
    break;
    }
}

/** @brief Generate dark and light samples for the supplied run durations and
 * quiet margins, returning the recorded outcome counts.
 */
static tally_t trace (const bc_decoder_config_t * p_config,
                      const uint32_t runs[29], unsigned int count,
                      uint32_t leading, uint32_t trailing)
{
    tally_t  out = {0};
    uint32_t end = leading + trailing;

    for (unsigned int run_index = 0U; run_index < count; run_index++)
    {
        end += runs[run_index];
    }

    bc_decoder_record_reset(&g_record);
    g_clock_us = UINT32_MAX - 1000000U;

    for (uint32_t elapsed_us = 0U; elapsed_us < end; elapsed_us += 10000U)
    {
        uint32_t edge = leading;
        uint16_t dark = 10U;

        if (elapsed_us >= leading)
        {
            for (unsigned int run_index = 0U; run_index < count; run_index++)
            {
                edge += runs[run_index];

                if (elapsed_us < edge)
                {
                    dark = (run_index % 2U) ? 10U : 900U;
                    break;
                }
            }
        }

        feed(p_config, dark, &out);
    }

    return out;
}

/** @brief Check 468 synthetic cases spanning the 8, 14 and 19 mm fixture
 * widths and the tested orientations.
 */
static void test_physical_wide_gaps (void)
{
    const bc_decoder_config_t cfg            = bc_decoder_defaults();
    const unsigned int        wide_mm[]      = {8U, 14U, 19U};
    const unsigned int        ratio_tenths[] = {20U, 25U, 30U};
    unsigned int              cases          = 0U;

    for (unsigned int size = 0U; size < (sizeof wide_mm / sizeof wide_mm[0]);
         size++)
    {
        /* time = distance / speed, at the configured normal 40 mm/s. */
        const uint32_t wide = wide_mm[size] * 1000000U / 40U;

        for (unsigned int ratio = 0U;
             ratio < (sizeof ratio_tenths / sizeof ratio_tenths[0]); ratio++)
        {
            const uint32_t narrow = wide * 10U / ratio_tenths[ratio];
            /* Reader waits at least its one-second end-of-barcode window. */
            const uint32_t margin = (12U * narrow) + 50000U;
            const uint32_t quiet  = margin > 1200000U ? margin : 1200000U;

            for (unsigned int letter = 0U; letter < FIXTURE_ALPHABET_COUNT;
                 letter++)
            {
                for (unsigned int direction = 0U;
                     direction < SCAN_DIRECTION_COUNT; direction++)
                {
                    uint32_t runs[29];
                    make_runs(letter, narrow, wide, runs);

                    if (direction)
                    {
                        reverse_runs(runs);
                    }

                    tally_t out = trace(&cfg, runs, 29U, quiet, quiet);

                    if ((((((out.decoded != 1U) ||
                            (out.result.letter !=
                             (char)((unsigned)('A') + letter))) ||
                           (out.result.b_is_reversed != (direction != 0U))) ||
                          (out.rejected)) ||
                         (out.timing)) ||
                        (out.overflow))
                    {
                        fprintf(stderr,
                                "wide=%umm ratio=%u/10 letter=%c direction=%u "
                                "decoded=%u (%c) rejected=%u timing=%u "
                                "overflow=%u\n",
                                wide_mm[size], ratio_tenths[ratio],
                                (char)((unsigned)('A') + letter), direction,
                                out.decoded,
                                out.result.letter ? out.result.letter : '-',
                                out.rejected, out.timing, out.overflow);
                        assert(0);
                    }

                    cases++;
                }
            }
        }
    }

    assert(468U == cases);
}

/** @brief Check acceptance and rejection of long frames as their quiet
 * regions and decoder configuration change.
 */
static void test_long_frame_quiet (void)
{
    bc_decoder_config_t cfg    = bc_decoder_defaults();
    const uint32_t      narrow = 237500U;
    const uint32_t wide = 475000U; /* Use a 19 mm wide bar and a 2:1 ratio. */
    uint32_t       runs[29];
    bc_decoder_result_t result;
    make_runs(25U, narrow, wide, runs);

    /* Geometry fits the old ten-second frame limit, but the actual complete
     * recording also needs 2.375 seconds of trailing quiet. The old wall
     * timeout was too short even though its batch width check passed. */
    bc_decoder_config_t old = cfg;
    old.max_frame_us        = 10000000U;
    assert(bc_decoder_decode_runs(runs, 29U, 3000000U, 3000000U, 10000U, &old,
                                  &result));
    tally_t out = trace(&old, runs, 29U, 3000000U, 3000000U);
    assert(0U == out.decoded && out.rejected != 0U && !out.timing &&
           !out.overflow);

    out = trace(&cfg, runs, 29U, 3000000U, 8U * narrow);
    assert(0U == out.decoded && !out.rejected && !out.timing && !out.overflow);

    for (unsigned int sample_index = 0U; sample_index < TRAILING_SAMPLE_COUNT;
         sample_index++)
    {
        feed(&cfg, 10U, &out);
    }

    assert(1U == out.decoded && 'Z' == out.result.letter && !out.rejected);

    out = trace(&cfg, runs, 29U, 8U * narrow, 3000000U);
    assert(0U == out.decoded); /* Require real leading quiet. */
    out = trace(&cfg, runs, 27U, 3000000U, 3000000U);
    assert(0U == out.decoded); /* Leave the closing marker incomplete. */
    runs[13] = 1100000U;
    out      = trace(&cfg, runs, 29U, 3000000U, 3000000U);
    assert(0U == out.decoded &&
           out.rejected != 0U); /* Reach the absolute single-gap limit. */
}

/** @brief Attempt to decode exactly 29 widths with three-second quiet
 * margins and a 10 ms sampling interval.
 */
static bool batch (const uint32_t runs[29], bc_decoder_result_t * p_result)
{
    const bc_decoder_config_t cfg = bc_decoder_defaults();
    return bc_decoder_decode_runs(runs, 29U, 3000000U, 3000000U, 10000U, &cfg,
                                  p_result);
}

/** @brief Check extended-gap acceptance in both directions and reject a
 * distorted payload.
 */
static void test_extended_narrow_gap (void)
{
    uint32_t            runs[29];
    bc_decoder_result_t result;

    for (unsigned int direction = 0U; direction < SCAN_DIRECTION_COUNT;
         direction++)
    {
        make_runs(25U, 100000U, 250000U, runs);
        runs[13] = 166000U; /* 149.55% of the resulting narrow mean (111000). */
        if (direction)
        {
            reverse_runs(runs);
        }

        assert(batch(runs, &result) && 'Z' == result.letter &&
               result.b_is_reversed == (direction != 0U));
    }

    make_runs(25U, 100000U, 250000U, runs);
    runs[13] = 168000U; /* 150.90% of the resulting narrow mean: too wide. */
    assert(!batch(runs, &result));

    make_runs(25U, 100000U, 250000U, runs);
    runs[13] = runs[15] =
        180000U; /* each 142.11%, but two outliers are refused. */
    assert(!batch(runs, &result));

    make_runs(25U, 100000U, 250000U, runs);
    runs[10] = 166000U; /* a narrow black bar keeps the strict 135% limit. */
    assert(!batch(runs, &result));
    make_runs(25U, 100000U, 250000U, runs);
    runs[3] = 166000U; /* opening star remains strict. */
    assert(!batch(runs, &result));
    make_runs(25U, 100000U, 250000U, runs);
    runs[23] = 166000U; /* closing star remains strict. */
    assert(!batch(runs, &result));
    make_runs(25U, 100000U, 250000U, runs);
    runs[11] = 500000U; /* wide white gaps keep the strict 135% limit. */
    assert(!batch(runs, &result));
}

/** @brief Decode the measured Z payload forward and backward and verify the
 * returned orientation.
 */
static void test_measured_z_payload (void)
{
    /* From the 3 Nov 20:38:55 waveform after sentinel correction. The markers
     * and margins here are synthetic: this isolates payload validation and
     * does not claim that the faulted hardware run decoded Z. */
    const uint32_t payload[9] = {109370U, 322217U, 282249U, 185373U, 300880U,
                                 121629U, 106343U, 105846U, 115817U};
    uint32_t       runs[29];

    bc_decoder_result_t result;
    make_runs(25U, 124000U, 302000U, runs);
    memcpy(runs + 10, payload, sizeof(payload));
    assert(batch(runs, &result) && 'Z' == result.letter);
    reverse_runs(runs);
    assert(batch(runs, &result) && 'Z' == result.letter &&
           result.b_is_reversed);
}

/* End of file. */

