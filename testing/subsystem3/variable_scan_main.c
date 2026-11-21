/** @file variable_scan_main.c
 * @brief Host evidence for a width bias that changes between the two known
 * stars.
 *
 * Synthetic edges and one measured-width fixture; not hardware proof.
 * Unsigned casts spell out the original modulo conversion.
 */
#include "variable_scan_main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bc_decoder.h"

#define REPLAY_FRAME_RUN_LIMIT 30U
#define STAR_REVERSE_PAIR_COUNT 4U

#define CODE39_ELEMENT_COUNT 9U
#define REVERSED_RUN_PAIRS 14U
#define FIXTURE_FRAME_RUN_COUNT 29U
#define WIDTH_BIAS_DIRECTION_COUNT 2U
#define SCAN_DIRECTION_COUNT 2U
#define BASELINE_SLOPE_CASES 2U
#define FRAME_CASE_COUNT 27U
#define PAYLOAD_FIRST_RUN 10U
#define PAYLOAD_END_RUN 19U

static const uint16_t g_letters[26] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
    0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
    0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0};

/* Extracted from the 18:26:59 BCAP trace at dt_ms*1000+500 us. Only the
 * widths are measured here; this test explicitly synthesises sharp levels
 * and white margins to exercise the streaming candidate/quiet state machine. */
static const uint32_t g_measured_a[29] = {
    90168,  532275, 39187,  286517, 303129, 353667, 284937, 300000,
    43563,  270000, 323734, 304891, 45700,  240328, 55231,  565541,
    70736,  241099, 372331, 211134, 104314, 481392, 133752, 189043,
    410586, 202229, 360209, 268500, 54666};

static void make_runs (unsigned int letter, uint32_t runs[29]);
static void reverse (uint32_t runs[29]);
static void distort (uint32_t runs[29], int32_t first, int32_t last);
static void test_letter_payload_patterns (void);
static void refuses_unchanged (const uint32_t original[29], uint32_t gap,
                               const bc_decoder_config_t * p_config);
static void test_bad_fits (void);
static unsigned int feed_measured (const uint32_t runs[29], uint32_t lead,
                                   uint32_t   trail,
                                   bool       b_should_interrupt_quiet,
                                   uint32_t * p_decoded_after);
static void         test_measured_stars_and_quiet (void);

/** @brief Test changing width bias against letter patterns, invalid fits and
 * measured star widths in both scan directions.
 */
int main (void)
{
    test_letter_payload_patterns();
    test_bad_fits();
    test_measured_stars_and_quiet();
    puts("PASS: varying sentinel correction, all "
         "letters/signs/directions/slopes, payload independence, "
         "invalid payload, inconsistent stars, bounded drift, unchanged "
         "failure inputs, measured A and actual quiet waiting");
    return 0;
}

/** @brief Encode the indexed letter between two sentinels using 100 ms
 * narrow elements and 250 ms wide elements.
 */
static void make_runs (unsigned int letter, uint32_t runs[29])
{
    uint16_t     patterns[3] = {0x094U, g_letters[letter], 0x094U};
    unsigned int symbol_index;
    unsigned int index;

    for (symbol_index = 0U;
         symbol_index < (sizeof patterns / sizeof patterns[0]); symbol_index++)
    {
        for (index = 0U; index < CODE39_ELEMENT_COUNT; index++)
        {
            runs[(symbol_index * 10U) + index] =
                (patterns[symbol_index] & (1U << (8U - index))) ? 250000U
                                                                : 100000U;
        }

        if (symbol_index < 2U)
        {
            runs[(symbol_index * 10U) + 9U] = 100000U;
        }
    }
}

/** @brief Reverse the 29 supplied barcode widths in place.
 */
static void reverse (uint32_t runs[29])
{
    unsigned int index;

    for (index = 0U; index < REVERSED_RUN_PAIRS; index++)
    {
        uint32_t saved_width_us = runs[index];
        runs[index]             = runs[28U - index];
        runs[28U - index]       = saved_width_us;
    }
}

/** @brief Shift printed boundaries in elapsed time to simulate bar erosion
 * or dilation. The decoder must infer the changing width bias from
 * measured intervals.
 */
static void distort (uint32_t runs[29], int32_t first, int32_t last)
{
    uint32_t     edges[30] = {0U};
    int64_t      shifted[30];
    unsigned int index;

    for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index++)
    {
        edges[index + 1U] = edges[index] + runs[index];
    }

    for (index = 0U; index < REPLAY_FRAME_RUN_LIMIT; index++)
    {
        int32_t bias;

        if (index <= 9U)
        {
            bias = first;
        }
        else if (index >= 20U)
        {
            bias = last;
        }
        else
        {
            /* Each unsigned edge difference fits before widening. */
            bias = first + (int32_t)((int64_t)(last - first) *
                                     (int64_t)(edges[index] - edges[9]) /
                                     (int64_t)(edges[20] - edges[9]));
        }

        shifted[index] =
            (int64_t)edges[index] +
            ((index % 2U) ? -(int64_t)bias / 2 : (int64_t)bias / 2);
    }

    for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index++)
    {
        assert(shifted[index + 1U] > shifted[index]);
        runs[index] = (uint32_t)(shifted[index + 1U] - shifted[index]);
    }
}

/** @brief Exercise payload-dependent bias corrections and check the direct
 * decoder's results in both orientations.
 */
static void test_letter_payload_patterns (void)
{
    const bc_decoder_config_t cfg = bc_decoder_defaults();
    unsigned int              case_index;
    unsigned int              sign;
    unsigned int              dir;
    unsigned int              slope;
    unsigned int              index;

    for (sign = 0U; sign < WIDTH_BIAS_DIRECTION_COUNT; sign++)
    {
        for (dir = 0U; dir < SCAN_DIRECTION_COUNT; dir++)
        {
            for (slope = 0U; slope < BASELINE_SLOPE_CASES; slope++)
            {
                int32_t reference[2] = {0, 0};

                for (case_index = 0U; case_index < FRAME_CASE_COUNT;
                     case_index++)
                {
                    uint32_t            runs[29];
                    int32_t             first = slope ? 20000 : 60000;
                    int32_t             last  = slope ? 60000 : 20000;
                    int32_t             bias[2];
                    bc_decoder_result_t result;
                    make_runs(case_index % 26U, runs);

                    if (26U == case_index)
                    {
                        for (index = PAYLOAD_FIRST_RUN; index < PAYLOAD_END_RUN;
                             index++)
                        {
                            runs[index] = 100000U;
                        }
                    }

                    if (sign)
                    {
                        first = -first;
                        last  = -last;
                    }

                    distort(runs, first, last);

                    if (dir)
                    {
                        int32_t fit_bias_us = first;
                        reverse(runs);
                        first = last;
                        last  = fit_bias_us;
                    }

                    assert(bc_decoder_record_fix_varying(runs, 10000U, &cfg,
                                                         bias));
                    assert(bias[0] > first - 1500 && bias[0] < first + 1500);
                    assert(bias[1] > last - 1500 && bias[1] < last + 1500);

                    if (0U == case_index)
                    {
                        reference[0] = bias[0];
                        reference[1] = bias[1];
                    }
                    else
                    {
                        assert(bias[0] == reference[0] &&
                               bias[1] == reference[1]);
                    }

                    if (case_index < 26U)
                    {
                        assert(bc_decoder_decode_runs(runs, 29U, 2000000U,
                                                      2000000U, 10000U, &cfg,
                                                      &result));
                        assert(result.letter ==
                                   (char)((unsigned)('A') + case_index) &&
                               result.b_is_reversed == (dir != 0U));
                    }
                    else
                    {
                        assert(!bc_decoder_decode_runs(runs, 29U, 2000000U,
                                                       2000000U, 10000U, &cfg,
                                                       &result));
                    }
                }
            }
        }
    }
}

/** @brief Require rejection of the varying-bias fit and compare every run
 * width with the input copy afterward.
 */
static void refuses_unchanged (const uint32_t original[29], uint32_t gap,
                               const bc_decoder_config_t * p_config)
{
    uint32_t runs[29];
    int32_t  bias[2] = {123, 456};
    memcpy(runs, original, sizeof(runs));
    assert(!bc_decoder_record_fix_varying(runs, gap, p_config, bias));
    assert(memcmp(runs, original, sizeof(runs)) == 0);
}

/** @brief Check that malformed durations, excessive distortions and invalid
 * configuration reject a fit without modifying its widths.
 */
static void test_bad_fits (void)
{
    bc_decoder_config_t cfg = bc_decoder_defaults();
    bc_decoder_config_t bad = cfg;
    uint32_t            runs[29];
    unsigned int        index;
    make_runs(0U, runs);
    distort(runs, 20000, -20000);
    refuses_unchanged(runs, 10000U, &cfg); /* Reverse the bias sign. */
    make_runs(0U, runs);
    distort(runs, 70000, 5000);
    refuses_unchanged(runs, 10000U,
                      &cfg); /* Change by more than half a narrow module. */
    make_runs(0U, runs);

    for (index = 0U; index < STAR_REVERSE_PAIR_COUNT; index++)
    {
        uint32_t saved_width_us = runs[20U + index];
        runs[20U + index]       = runs[28U - index];
        runs[28U - index]       = saved_width_us;
    }

    refuses_unchanged(runs, 10000U,
                      &cfg); /* Use opposite sentinel orientations. */
    make_runs(0U, runs);
    runs[20] = 0U;
    refuses_unchanged(runs, 10000U, &cfg);
    make_runs(0U, runs);
    distort(runs, 60000, 20000);
    refuses_unchanged(runs, 40000U, &cfg); /* Insert a gap between samples. */
    bad.quiet_modules = 0U;
    refuses_unchanged(runs, 10000U, &bad);
}

/** @brief Sample the supplied run durations and quiet regions. Return the
 * count of decoded A results and record decode delay on success.
 */
static unsigned int feed_measured (const uint32_t runs[29], uint32_t lead,
                                   uint32_t   trail,
                                   bool       b_should_interrupt_quiet,
                                   uint32_t * p_decoded_after)
{
    static bc_decoder_record_t record;
    const bc_decoder_config_t  cfg    = bc_decoder_defaults();
    uint32_t                   finish = lead;
    uint32_t                   sample_time_us;
    unsigned int               index;
    unsigned int               decoded = 0U;

    for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index++)
    {
        finish += runs[index];
    }

    bc_decoder_record_reset(&record);
    *p_decoded_after = 0U;

    for (sample_time_us  = 0U; sample_time_us < (finish + trail);
         sample_time_us += 10000U)
    {
        uint32_t            edge = lead;
        uint16_t            dark = 10U;
        bc_decoder_result_t result;
        bc_decoder_status_t status;

        for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index++)
        {
            if ((sample_time_us >= edge) &&
                (sample_time_us < (edge + runs[index])))
            {
                dark = (index % 2U) ? 10U : 900U;
                break;
            }

            edge += runs[index];
        }

        if ((b_should_interrupt_quiet &&
             (sample_time_us >= (finish + 1500000U))) &&
            (sample_time_us < (finish + 1570000U)))
        {
            dark = 70U;
        }

        status = bc_decoder_record_update(
            &record, &cfg, sample_time_us + 10000U, true, dark, &result);
        assert(status != (unsigned)S3_BARCODE_TIMING_ERROR &&
               status != (unsigned)S3_BARCODE_OVERFLOW);

        if ((unsigned)S3_BARCODE_DECODED == status)
        {
            decoded++;
            assert('A' == result.letter);
            *p_decoded_after = sample_time_us - finish;
        }
    }

    return decoded;
}

/** @brief Check fits against measured sentinel widths and verify acceptance
 * or rejection as the quiet regions and payload change.
 */
static void test_measured_stars_and_quiet (void)
{
    const bc_decoder_config_t cfg = bc_decoder_defaults();
    uint32_t                  runs[29];
    uint32_t                  elapsed;
    int32_t                   shared;
    int32_t                   bias[2];
    bc_decoder_result_t       result;
    unsigned int              index;
    memcpy(runs, g_measured_a, sizeof(runs));
    assert(!bc_decoder_record_correct_runs(runs, 12500U, &cfg, &shared));
    assert(bc_decoder_record_fix_varying(runs, 12500U, &cfg, bias));
    assert(bias[0] > 120000 && bias[0] < 140000 && bias[1] > 60000 &&
           bias[1] < 75000);
    assert(bc_decoder_decode_runs(runs, 29U, 2200000U, 2000000U, 12500U, &cfg,
                                  &result));
    assert('A' == result.letter);
    assert(feed_measured(g_measured_a, 2200000U, 2000000U, false, &elapsed) ==
           1U);
    /* End star's narrow mean is ~159 ms. Its expansion consumes additional
     * trailing white; a candidate must not emit as soon as one second passes.
     */
    assert(elapsed >= 1600000U);
    /* These margins exceed ten observed narrow modules, but not ten modules
     * plus the outward movement of the corresponding outer edge. */
    assert(feed_measured(g_measured_a, 1890000U, 2000000U, false, &elapsed) ==
           0U);
    assert(feed_measured(g_measured_a, 2200000U, 1600000U, false, &elapsed) ==
           0U);
    assert(feed_measured(g_measured_a, 2200000U, 2000000U, true, &elapsed) ==
           0U);
    memcpy(runs, g_measured_a, sizeof(runs));

    for (index = PAYLOAD_FIRST_RUN; index < PAYLOAD_END_RUN; index++)
    {
        runs[index] = 170000U;
    }

    assert(feed_measured(runs, 2200000U, 2000000U, false, &elapsed) == 0U);
}

/* End of file. */

