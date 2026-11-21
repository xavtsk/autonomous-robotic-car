/** @file work_quota_main.c
 * @brief Bounded-work and cancellation tests.
 *
 * Link the synchronous reference to compare candidate semantics with the
 * same sampled input. The fixture tracks each update's work usage.
 */
#include "work_quota_main.h"
#include <assert.h>
#include <stdio.h>
#include "../../app_program/build_assert.h"

#define LEADING_PAPER_SAMPLE_COUNT 240U
#define TRAILING_PAPER_SAMPLE_COUNT 280U
#define WAVEFORM_CASE_COUNT 3U
#define CANCELLATION_CASE_COUNT 4U
#define AFTER_CANCEL_SAMPLE_COUNT 300U
#define SHORTENED_WIDE_BAR_SAMPLES 2U

#define FRAME_SYMBOL_COUNT 3U
#define CODE39_ELEMENT_COUNT 9U
#define FIXTURE_FRAME_RUN_COUNT 29U
#define FIXTURE_ALPHABET_COUNT 26U
#define SCAN_DIRECTION_COUNT 2U
#define FINISH_MODE_CASE_COUNT 2U

/* Select the former field only when the shared fixture builds the baseline. */
#ifndef TEST_EXTREMA_COUNT_MEMBER
#define TEST_EXTREMA_COUNT_MEMBER extrema_count
#endif

static bc_decoder_record_t record;
static uint32_t            clock_us;
static unsigned int        max_units;
static unsigned int        max_ticks;
static unsigned int        max_extrema;
static unsigned int        processing_updates;
static const uint16_t      codes[26] = {
    0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U, 0x00DU, 0x10CU, 0x04CU,
    0x01CU, 0x103U, 0x043U, 0x142U, 0x013U, 0x112U, 0x052U, 0x007U, 0x106U,
    0x046U, 0x016U, 0x181U, 0x0C1U, 0x1C0U, 0x091U, 0x190U, 0x0D0U};
static uint16_t samples[2000];

/** @brief Advance simulated time by the supplied step, feed one sample and
 * track the incremental decoder's work and processing updates.
 */
static bc_decoder_status_t feed (uint16_t dark, bool b_is_valid, uint32_t step,
                                 bc_decoder_result_t * p_result)
{
    const bc_decoder_config_t config = bc_decoder_defaults();
    clock_us                        += step;
    const bc_decoder_status_t status = bc_decoder_record_update(
        &record, &config, clock_us, b_is_valid, dark, p_result);
    assert(record.work_units <= S3_REC_WORK_UNITS);
    assert(record.fit_ticks <= S3_REC_FIT_TICKS);
    if (record.work_units > max_units)
    {
        max_units = record.work_units;
    }
    if (record.fit_ticks > max_ticks)
    {
        max_ticks = record.fit_ticks;
    }
    if (record.work.TEST_EXTREMA_COUNT_MEMBER > max_extrema)
    {
        max_extrema = record.work.TEST_EXTREMA_COUNT_MEMBER;
    }
    if (S3_REC_PROCESSING == (int)record.state)
    {
        processing_updates++;
    }
    return status;
}

/** @brief Build a sampled barcode for the indexed letter, direction and
 * width scenario with its white margins.
 */
static unsigned int wave (unsigned int letter, unsigned int mode)
{
    const uint16_t patterns[3] = {0x094U, codes[letter], 0x094U};
    const uint32_t narrow      = 2U == mode ? 200000U : 100000U;
    const uint32_t wide        = 2U == mode ? 600000U : 250000U;
    uint32_t       widths[29];
    unsigned int   count = 0U;
    for (unsigned int symbol_index = 0U; symbol_index < FRAME_SYMBOL_COUNT;
         symbol_index++)
    {
        for (unsigned int element_index = 0U;
             element_index < CODE39_ELEMENT_COUNT; element_index++)
        {
            widths[symbol_index * 10U + element_index] =
                (patterns[symbol_index] & (1U << (8U - element_index)))
                    ? wide
                    : narrow;
        }
        if (symbol_index < 2U)
        {
            widths[symbol_index * 10U + 9U] = narrow;
        }
    }
    for (unsigned int sample_index = 0U;
         sample_index < LEADING_PAPER_SAMPLE_COUNT; sample_index++)
    {
        samples[count++] = 0U;
    }
    for (unsigned int run_index = 0U; run_index < FIXTURE_FRAME_RUN_COUNT;
         run_index++)
    {
        const uint32_t observed = (run_index % 2U) ? widths[run_index] + 60000U
                                                   : widths[run_index] - 60000U;
        for (uint32_t elapsed_us = 0U; elapsed_us < observed;
             elapsed_us         += 10000U)
        {
            uint16_t dark =
                (run_index % 2U)
                    ? 0U
                    : (1U == mode && widths[run_index] == narrow ? 12U : 400U);
            if (2U == mode && !(run_index % 2U) && widths[run_index] == wide &&
                elapsed_us >= 50000U && elapsed_us < 400000U &&
                (elapsed_us % 50000U) < 20000U)
            {
                dark = 360U;
            }
            assert(count < sizeof(samples) / sizeof(samples[0]));
            samples[count++] = dark;
        }
    }
    for (unsigned int sample_index = 0U;
         sample_index < TRAILING_PAPER_SAMPLE_COUNT; sample_index++)
    {
        samples[count++] = 0U;
    }
    return count;
}

/** @brief Compare a copied incremental decoder candidate and its required
 * quiet time against the reference calculation.
 */
static void compare_frozen_candidate (void)
{
    bc_decoder_record_t       snapshot       = record;
    const bc_decoder_config_t config         = bc_decoder_defaults();
    bc_decoder_result_t       expected       = {0};
    bc_decoder_result_t       other          = {0};
    uint32_t                  required       = 0U;
    uint32_t                  other_required = 0U;
    snapshot.count                           = snapshot.work.frozen_count;
    bool b_has_candidate                     = bc_decoder_work_test_candidate(
        &snapshot, &config, false, &expected, &required);
    bool b_is_ambiguous = false;
    if (bc_decoder_work_test_faint_eligible(&snapshot) &&
        bc_decoder_work_test_candidate(&snapshot, &config, true, &other,
                                       &other_required))
    {
        b_is_ambiguous =
            b_has_candidate && (expected.letter != other.letter ||
                                expected.b_is_reversed != other.b_is_reversed);
        expected        = other;
        b_has_candidate = true;
        if (other_required > required)
        {
            required = other_required;
        }
    }
    /* Advance only a diagnostic copy, with its frozen physical observations.
     * This comparison is not a live decode or a substitute for fresh samples.
     */
    while (!bc_decoder_work_test_is_done(&snapshot))
    {
        snapshot.work_units = 0U;
        snapshot.fit_ticks  = 0U;
        bc_decoder_work_test_advance(&snapshot, &config);
        assert(snapshot.work_units <= S3_REC_WORK_UNITS);
        assert(snapshot.fit_ticks <= S3_REC_FIT_TICKS);
    }
    assert(snapshot.work.b_has_candidate == b_has_candidate);
    assert(snapshot.work.b_is_ambiguous == b_is_ambiguous);
    if (b_has_candidate && !b_is_ambiguous)
    {
        assert(snapshot.work.candidate.letter_code ==
               (unsigned int)(expected.letter - 'A' + 1));
        assert(snapshot.work.candidate.b_is_reversed == expected.b_is_reversed);
        assert(snapshot.work.required_us == required);
    }
}

/** @brief Check 156 letter, direction and sampling scenarios, including a
 * frozen-candidate comparison and one successful decode per case.
 */
static void all_letters (void)
{
    unsigned int checks = 0U;
    for (unsigned int letter = 0U; letter < FIXTURE_ALPHABET_COUNT; letter++)
    {
        for (unsigned int reverse = 0U; reverse < SCAN_DIRECTION_COUNT;
             reverse++)
        {
            for (unsigned int mode = 0U; mode < WAVEFORM_CASE_COUNT; mode++)
            {
                const unsigned int count         = wave(letter, mode);
                unsigned int       decoded       = 0U;
                bool               b_did_compare = false;
                bc_decoder_record_reset(&record);
                clock_us = 0xFFFF0000U;
                for (unsigned int sample_index = 0U; sample_index < count;
                     sample_index++)
                {
                    bc_decoder_result_t       result;
                    const bc_decoder_status_t status =
                        feed(samples[reverse ? count - 1U - sample_index
                                             : sample_index],
                             true, 10000U, &result);
                    if (!b_did_compare &&
                        S3_REC_PROCESSING == (int)record.state)
                    {
                        compare_frozen_candidate();
                        b_did_compare = true;
                    }
                    if ((bc_decoder_status_t)S3_BARCODE_DECODED == status)
                    {
                        decoded++;
                        assert(result.letter == (char)('A' + (int)letter));
                        assert(result.b_is_reversed == (reverse != 0U));
                    }
                    assert(status !=
                               (bc_decoder_status_t)S3_BARCODE_TIMING_ERROR &&
                           status != (bc_decoder_status_t)S3_BARCODE_OVERFLOW);
                }
                assert(b_did_compare && 1U == decoded);
                checks++;
            }
        }
    }
    assert(156U == checks);
}

/** @brief Interrupt a pending decode at the selected processing stages and
 * verify that later white samples never publish the cancelled
 * result.
 */
static void cancel_pending_jobs (void)
{
    for (unsigned int finish_pass = 0U; finish_pass < FINISH_MODE_CASE_COUNT;
         finish_pass++)
    {
        for (unsigned int mode = 0U; mode < CANCELLATION_CASE_COUNT; mode++)
        {
            const unsigned int  count = wave(25U, 0U);
            bc_decoder_result_t result;
            bool                b_is_pending = false;
            if (finish_pass)
            {
                /* Retain 15 bars and both stars, but shorten the middle wide
                 * payload bar to 20 ms. The early fit must refuse it; its
                 * unchanged 1 s final attempt still has real work to cancel. */
                unsigned int bars = 0U;
                for (unsigned int sample_index = 0U; sample_index < count;)
                {
                    if (!samples[sample_index])
                    {
                        sample_index++;
                        continue;
                    }
                    const unsigned int begin = sample_index;
                    while (sample_index < count && samples[sample_index])
                    {
                        sample_index++;
                    }
                    if (bars++ == 7U)
                    {
                        assert(sample_index - begin > 2U);
                        for (unsigned int tail_index =
                                 begin + SHORTENED_WIDE_BAR_SAMPLES;
                             tail_index < sample_index; tail_index++)
                        {
                            samples[tail_index] = 0U;
                        }
                        break;
                    }
                }
                assert(8U == bars);
            }
            bc_decoder_record_reset(&record);
            clock_us = 0U;
            for (unsigned int sample_index = 0U; sample_index < count;
                 sample_index++)
            {
                assert(feed(samples[sample_index], true, 10000U, &result) !=
                       (bc_decoder_status_t)S3_BARCODE_DECODED);
                if (S3_REC_PROCESSING == (int)record.state &&
                    record.work.b_is_early == (0U == finish_pass))
                {
                    assert(!record.work.b_is_provisional);
                    b_is_pending = true;
                    break;
                }
            }
            assert(b_is_pending);
            const unsigned int retained = record.count;
            if (0U == mode)
            {
                assert(feed(0U, false, 10000U, &result) ==
                       (bc_decoder_status_t)S3_BARCODE_NONE);
            }
            if (1U == mode)
            {
                assert(feed(0U, true, 40000U, &result) ==
                       (bc_decoder_status_t)S3_BARCODE_TIMING_ERROR);
            }
            if (2U == mode)
            {
                const bc_decoder_status_t expected =
                    finish_pass ? S3_BARCODE_REJECTED : S3_BARCODE_NONE;
                assert(feed(400U, true, 10000U, &result) == expected);
            }
            if (3U == mode)
            {
                bc_decoder_record_reset(&record);
            }
            if (2U == mode && !finish_pass)
            {
                assert(S3_REC_RECORDING == (int)record.state &&
                       0U == record.work.phase);
                assert(record.count ==
                       retained +
                           1U); /* Preserve new ink and the earlier history. */
            }
            else
            {
                assert(S3_REC_WAITING == (int)record.state &&
                       0U == record.work.phase);
            }
            for (unsigned int sample_index = 0U;
                 sample_index < AFTER_CANCEL_SAMPLE_COUNT; sample_index++)
            {
                assert(feed(0U, true, 10000U, &result) !=
                       (bc_decoder_status_t)S3_BARCODE_DECODED);
            }
        }
    }
}

BUILD_ASSERT(sizeof(bc_decoder_work_t) <= 1200U, workspace_budget_t);

/** @brief Decode letters and cancel work at phase boundaries. Check and
 * print the maximum per-update work observed.
 */
int main (void)
{
    all_letters();
    cancel_pending_jobs();
    assert(S3_REC_WORK_UNITS == max_units && S3_REC_FIT_TICKS == max_ticks);
    assert(max_extrema >= 100U && processing_updates > 100U);
    printf("PASS: 156 incremental A-Z/direction/sharp-faint-ripple "
           "equivalence scans; invalid/timing/new-ink/reset cancellation; "
           "maximum work=%u ticks=%u extrema=%u workspace=%u record=%u bytes\n",
           max_units, max_ticks, max_extrema,
           (unsigned int)sizeof(bc_decoder_work_t),
           (unsigned int)sizeof(bc_decoder_record_t));
    return 0;
}

/* End of file. */

