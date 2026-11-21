/** @file fast_scan_main.c
 * @brief Synthetic fast-scan regressions, not claims about physical test
 * success.
 *
 * F and P share their wide-bar centres: the narrow bar between them must
 * remain observed and distinguish the letters in either travel direction.
 * Unsigned casts spell out the original modulo conversion.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "fast_scan_main.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "bc_decoder.h"

#define SYMBOL_ELEMENT_COUNT 9U
#define SCAN_DIRECTION_COUNT 2U
#define OPTICAL_CASE_COUNT 2U
#define MIN_WIDTH_RATIO 2U
#define MAX_WIDTH_RATIO 3U
#define INCOMPLETE_FRAME_END_US 1500000U

typedef enum
{
    STEP_US   = 5000U,
    PAPER     = 100U,
    INK       = 900U,
    MARGIN_US = 1500000U
} fast_scan_fixture_constant_t;

typedef enum
{
    INTACT,
    SHORT_LEAD,
    SHORT_TRAIL,
    MISSING_NARROW,
    WEAK_FRAME
} fast_scan_scenario_t;

typedef struct
{
    unsigned int        decoded;
    unsigned int        rejected;
    unsigned int        timing;
    unsigned int        overflow;
    uint32_t            decode_delay_us;
    uint32_t            rejection_delay_us;
    bc_decoder_result_t last;
} tally_t;

static void    make_runs (char letter, uint32_t narrow, unsigned int ratio,
                          bool b_is_reverse, uint32_t runs[S3_BARCODE_RUNS]);
static tally_t scan (char letter, bool b_is_reverse, uint32_t narrow,
                     unsigned int ratio, uint32_t erosion, uint32_t phase,
                     unsigned int damage);
static void    test_fast_frames (void);
static void    test_fast_refusals (void);
static void    test_early_quiet_and_long_frame (void);
static void    test_width_and_sample_limits (void);

/** @brief Exercise fast barcode scans at the sampling limit, including frame
 * refusals and malformed gaps.
 */
int main (void)
{
    test_fast_frames();
    test_fast_refusals();
    test_width_and_sample_limits();
    test_early_quiet_and_long_frame();
    puts("PASS: fast F/P quiet/missing-bar refusals, 25 ms floor, "
         "3-times-sample-gap guard and malformed payload gap");
    return 0;
}

/** @brief Generate an F or P frame with the given narrow duration and width
 * ratio, reversing the width sequence when requested.
 */
static void make_runs (char letter, uint32_t narrow, unsigned int ratio,
                       bool b_is_reverse, uint32_t runs[S3_BARCODE_RUNS])
{
    const uint16_t payload    = 'F' == letter ? 0x058U : 0x052U;
    const uint16_t symbols[3] = {0x094U, payload, 0x094U};
    assert('F' == letter || 'P' == letter);

    for (unsigned int symbol_index = 0U;
         symbol_index < (sizeof symbols / sizeof symbols[0]); symbol_index++)
    {
        for (unsigned int element_index = 0U;
             element_index < SYMBOL_ELEMENT_COUNT; element_index++)
        {
            runs[(symbol_index * 10U) + element_index] =
                (symbols[symbol_index] & (1U << (8U - element_index)))
                    ? ratio * narrow
                    : narrow;
        }

        if (symbol_index < 2U)
        {
            runs[(symbol_index * 10U) + 9U] = narrow;
        }
    }

    if (b_is_reverse)
    {
        for (unsigned int run_index = 0U; run_index < (S3_BARCODE_RUNS / 2U);
             run_index++)
        {
            const uint32_t saved = runs[run_index];
            /* clang-format off */
            runs[run_index]                        =
                runs[S3_BARCODE_RUNS - 1U - run_index];
            runs[S3_BARCODE_RUNS - 1U - run_index] = saved;
            /* clang-format on */
        }
    }
}

/** @brief Sample the selected frame and defect scenario and return the
 * decoder outcomes with their delays.
 */
static tally_t scan (char letter, bool b_is_reverse, uint32_t narrow,
                     unsigned int ratio, uint32_t erosion, uint32_t phase,
                     unsigned int damage)
{
    static bc_decoder_record_t record;
    const bc_decoder_config_t  config = bc_decoder_defaults();

    const uint32_t margin = (12U * narrow) > (unsigned)MARGIN_US
                                ? 12U * narrow
                                : (unsigned)MARGIN_US;
    const uint32_t lead =
        ((unsigned)SHORT_LEAD == damage ? 8U * narrow : margin) + phase;
    const uint32_t trail =
        (unsigned)SHORT_TRAIL == damage ? 8U * narrow : margin;
    uint32_t runs[S3_BARCODE_RUNS];
    uint32_t begin[S3_BARCODE_RUNS];
    uint32_t end[S3_BARCODE_RUNS];
    uint32_t total = lead;
    tally_t  tally = {0};
    /* Canonical stripe 16 is the narrow black bar separating F from P.
     * Delete it geometrically, without substituting a different payload. */
    const unsigned int missing =
        b_is_reverse ? S3_BARCODE_RUNS - 1U - 16U : 16U;
    make_runs(letter, narrow, ratio, b_is_reverse, runs);
    assert(erosion < narrow && phase < (unsigned)STEP_US);

    for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS; run_index++)
    {
        /* Fixed printed widths first; the optics shrink every black bar by
         * the same duration and enlarge each internal white space equally.
         * Both outer margins gain half the erosion as well. */
        /* clang-format off */
        begin[run_index] = total + ((run_index % 2U) ? 0U : erosion / 2U);
        end[run_index]   =
            total + runs[run_index] - ((run_index % 2U) ? 0U : erosion / 2U);
        total           += runs[run_index];
        /* clang-format on */
    }

    total += trail;
    bc_decoder_record_reset(&record);

    for (uint32_t elapsed_us = 0U; elapsed_us < total;
         elapsed_us         += (unsigned)STEP_US)
    {
        uint16_t            darkness = PAPER;
        bc_decoder_result_t result;

        for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS;
             run_index             += 2U)
        {
            if (((unsigned)MISSING_NARROW == damage) && (run_index == missing))
            {
                continue;
            }

            if ((elapsed_us >= begin[run_index]) &&
                (elapsed_us < end[run_index]))
            {
                darkness =
                    (unsigned)WEAK_FRAME == damage
                        ? (unsigned)PAPER + S3_QUIET_STRONG_PROMINENCE - 5U
                        : (unsigned)INK;
                break;
            }
        }

        const bc_decoder_status_t status =
            bc_decoder_record_update(&record, &config, 0xFFFF0000U + elapsed_us,
                                     true, darkness, &result);
        assert(record.work_units <= S3_REC_WORK_UNITS);
        assert(record.fit_ticks <= S3_REC_FIT_TICKS);

        if ((unsigned)S3_BARCODE_DECODED == status)
        {
            assert(elapsed_us >= end[S3_BARCODE_RUNS - 1U]);
            tally.decoded++;
            tally.last            = result;
            tally.decode_delay_us = elapsed_us - end[S3_BARCODE_RUNS - 1U];
        }
        else if ((unsigned)S3_BARCODE_REJECTED == status)
        {
            if (!tally.rejected)
            {
                tally.rejection_delay_us =
                    elapsed_us >= end[S3_BARCODE_RUNS - 1U]
                        ? elapsed_us - end[S3_BARCODE_RUNS - 1U]
                        : UINT32_MAX;
            }

            tally.rejected++;
        }
        else if ((unsigned)S3_BARCODE_TIMING_ERROR == status)
        {
            tally.timing++;
        }
        else if ((unsigned)S3_BARCODE_OVERFLOW == status)
        {
            tally.overflow++;
        }
        else
        {
            /* This status does not add to the scan totals. */
        }
    }

    return tally;
}

/** @brief Check fast F and P scans in both directions at five sampling
 * phases and multiple narrow widths.
 */
static void test_fast_frames (void)
{
    const char letters[] = {'F', 'P'};
    /* Half-period module widths also vary the individual sample counts;
     * shifting only period-aligned widths would preserve every measured run. */
    const uint32_t narrow[] = {25000U, 27500U, 30000U, 32500U, 35000U};
    unsigned int   checks   = 0U;

    for (unsigned int letter_index = 0U; letter_index < sizeof letters;
         letter_index++)
    {
        for (unsigned int reverse = 0U; reverse < SCAN_DIRECTION_COUNT;
             reverse++)
        {
            for (unsigned int width_index = 0U;
                 width_index < (sizeof narrow / sizeof narrow[0]);
                 width_index++)
            {
                for (unsigned int ratio = MIN_WIDTH_RATIO;
                     ratio <= MAX_WIDTH_RATIO; ratio++)
                {
                    for (unsigned int optical = 0U;
                         optical < OPTICAL_CASE_COUNT; optical++)
                    {
                        /* For a 25 ms module, 20 ms of erosion exceeds the
                         * 75% bias bound. A 15 ms erosion stays within it.
                         * For a 30 ms module, 20 ms leaves a 10 ms core.
                         */
                        const uint32_t erosion =
                            optical ? (narrow[width_index] < 30000U
                                           ? narrow[width_index] - 10000U
                                           : 20000U)
                                    : 0U;

                        for (uint32_t phase = 0U; phase < (unsigned)STEP_US;
                             phase         += 1000U)
                        {
                            const tally_t got =
                                scan(letters[letter_index], reverse != 0U,
                                     narrow[width_index], ratio, erosion, phase,
                                     INTACT);

                            if ((((got.decoded != 1U) ||
                                  (got.last.letter != letters[letter_index])) ||
                                 (got.last.b_is_reversed != (reverse != 0U))) ||
                                (got.rejected))
                            {
                                printf("fast %c reverse=%u narrow=%u ratio=%u "
                                       "erosion=%u phase=%u: decoded=%u "
                                       "letter=%c rejected=%u\n",
                                       letters[letter_index], reverse,
                                       narrow[width_index], ratio, erosion,
                                       phase, got.decoded,
                                       got.last.letter ? got.last.letter : '-',
                                       got.rejected);
                            }

                            assert(1U == got.decoded &&
                                   got.last.letter == letters[letter_index]);
                            assert(got.last.b_is_reversed == (reverse != 0U) &&
                                   0U == got.rejected);
                            assert(0U == got.timing && 0U == got.overflow);
                            assert(got.decode_delay_us < 1000000U);
                            checks++;
                        }
                    }
                }
            }
        }
    }

    printf("PASS: %u synthetic fast F/P scans, both directions, 5 sample "
           "phases, sharp/eroded 25-35 ms modules\n",
           checks);
}

/** @brief Check that the malformed or weak fast frames do not decode and
 * inspect the associated rejection status.
 */
static void test_fast_refusals (void)
{
    const char letters[] = {'F', 'P'};

    for (unsigned int letter_index = 0U; letter_index < sizeof letters;
         letter_index++)
    {
        for (unsigned int reverse = 0U; reverse < SCAN_DIRECTION_COUNT;
             reverse++)
        {
            for (unsigned int damage = SHORT_LEAD;
                 damage <= (unsigned)MISSING_NARROW; damage++)
            {
                for (uint32_t phase = 0U; phase < (unsigned)STEP_US;
                     phase         += 1000U)
                {
                    const tally_t got =
                        scan(letters[letter_index], reverse != 0U, 30000U, 3U,
                             20000U, phase, damage);
                    assert(0U == got.decoded && 0U == got.timing &&
                           0U == got.overflow);

                    if ((unsigned)MISSING_NARROW == damage)
                    {
                        /* The 250 ms attempt cannot end an incomplete frame.
                         * Its unchanged final 1 s attempt still rejects it. */
                        assert(1U == got.rejected);
                        assert(got.rejection_delay_us >= 1000000U &&
                               got.rejection_delay_us < 1500000U);
                    }
                }
            }
            /* More frequent readings do not make sub-minimum modules valid. */
            const tally_t small = scan(letters[letter_index], reverse != 0U,
                                       20000U, 3U, 0U, 0U, INTACT);
            assert(0U == small.decoded && small.rejected != 0U);
            assert(0U == small.timing && 0U == small.overflow);
        }
    }
}

/** @brief Exercise early quiet-zone handling and verify that the recorder
 * can stay active during a longer frame.
 */
static void test_early_quiet_and_long_frame (void)
{
    const char letters[] = {'F', 'P'};

    for (unsigned int letter_index = 0U; letter_index < sizeof letters;
         letter_index++)
    {
        for (unsigned int reverse = 0U; reverse < SCAN_DIRECTION_COUNT;
             reverse++)
        {
            const tally_t small = scan(letters[letter_index], reverse != 0U,
                                       30000U, 3U, 20000U, 0U, INTACT);
            assert(1U == small.decoded && 0U == small.rejected);
            /* Do not confuse beginning computation at 250 ms with permission
             * to stop: this frame still needs its full 300 ms quiet margin. */
            assert(small.decode_delay_us >= 300000U &&
                   small.decode_delay_us < 1000000U);
            const tally_t weak = scan(letters[letter_index], reverse != 0U,
                                      30000U, 3U, 20000U, 0U, WEAK_FRAME);
            /* Below the strong-frame threshold: only actual strict quiet,
             * not the tolerant noise clock, can authorize this early fit. */
            assert(1U == weak.decoded &&
                   weak.last.letter == letters[letter_index] &&
                   0U == weak.rejected);
            assert(weak.last.b_is_reversed == (reverse != 0U));
            assert(weak.decode_delay_us >= 300000U &&
                   weak.decode_delay_us < 1000000U);
            /* 450 ms internal white elements exceed the early-work trigger.
             * Neither a failed partial fit nor new ink may discard the frame.
             */
            const tally_t large = scan(letters[letter_index], reverse != 0U,
                                       150000U, 3U, 60000U, 0U, INTACT);
            assert(1U == large.decoded &&
                   large.last.letter == letters[letter_index]);
            assert(large.last.b_is_reversed == (reverse != 0U));
            assert(0U == large.rejected && 0U == large.timing &&
                   0U == large.overflow);
            assert(large.decode_delay_us >= 1500000U);
        }
    }
    /* A short incomplete fragment followed by 500 ms white must remain open.
     * Its early attempt may fail, but cannot publish a letter or rejection. */
    static bc_decoder_record_t record;
    const bc_decoder_config_t  config = bc_decoder_defaults();
    bc_decoder_result_t        result;
    bc_decoder_record_reset(&record);

    for (uint32_t elapsed_us = 0U; elapsed_us < INCOMPLETE_FRAME_END_US;
         elapsed_us         += (unsigned)STEP_US)
    {
        const uint16_t dark =
            (elapsed_us >= 900000U) && (elapsed_us < 1000000U) ? INK : PAPER;
        assert(bc_decoder_record_update(&record, &config, elapsed_us, true,
                                        dark,
                                        &result) == (unsigned)S3_BARCODE_NONE);
    }

    assert(bc_decoder_record_active(&record));
    assert(record.b_did_try_early_fit);
}

/** @brief Check the configured minimum narrow width and the relationship
 * between element duration and sample spacing.
 */
static void test_width_and_sample_limits (void)
{
    const bc_decoder_config_t config    = bc_decoder_defaults();
    const char                letters[] = {'F', 'P'};
    assert(25000U == config.min_narrow_us);

    for (unsigned int letter_index = 0U; letter_index < sizeof letters;
         letter_index++)
    {
        for (unsigned int reverse = 0U; reverse < SCAN_DIRECTION_COUNT;
             reverse++)
        {
            uint32_t            runs[S3_BARCODE_RUNS];
            bc_decoder_result_t result;
            make_runs(letters[letter_index], 24999U, 3U, reverse != 0U, runs);
            assert(!bc_decoder_decode_runs(runs, S3_BARCODE_RUNS, MARGIN_US,
                                           MARGIN_US, STEP_US, &config,
                                           &result));
            make_runs(letters[letter_index], 25000U, 3U, reverse != 0U, runs);
            assert(bc_decoder_decode_runs(runs, S3_BARCODE_RUNS, MARGIN_US,
                                          MARGIN_US, STEP_US, &config,
                                          &result));
            assert(result.letter == letters[letter_index] &&
                   result.b_is_reversed == (reverse != 0U));
            make_runs(letters[letter_index], 30000U, 3U, reverse != 0U, runs);
            assert(bc_decoder_decode_runs(runs, S3_BARCODE_RUNS, MARGIN_US,
                                          MARGIN_US, 10000U, &config, &result));
            assert(!bc_decoder_decode_runs(runs, S3_BARCODE_RUNS, MARGIN_US,
                                           MARGIN_US, 10001U, &config,
                                           &result));
            /* A physically too-short payload white gap remains invalid.
             * Use exact durations here: 5 ms sample rounding could turn a
             * 16.2 ms gap into 20 ms and obscure the intended 65% boundary. */
            const unsigned int gap = reverse ? S3_BARCODE_RUNS - 1U - 11U : 11U;
            runs[gap]              = 30000U * 54U / 100U;
            assert(!bc_decoder_decode_runs(runs, S3_BARCODE_RUNS, MARGIN_US,
                                           MARGIN_US, STEP_US, &config,
                                           &result));
        }
    }
}

/* End of file. */

