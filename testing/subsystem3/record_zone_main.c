/** @file record_zone_main.c
 * @brief Public-API regressions for noisy trailing paper in the recording
 * reader.
 *
 * Synthetic Code 39 waveforms, not observed hardware successes. A valid
 * strong frame may bridge bounded paper excursions; faint bars and genuine
 * quiet-zone interruptions must remain visible.
 */
#include "record_zone_main.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "bc_decoder.h"

#define REVERSED_RUN_PAIRS 14U

#define FRAME_SYMBOL_COUNT 3U
#define CODE39_ELEMENT_COUNT 9U
#define FIXTURE_FRAME_RUN_COUNT 29U
#define FIXTURE_ALPHABET_COUNT 26U
#define SCAN_DIRECTION_COUNT 2U
#define QUIET_BOUNDARY_CASE_COUNT 2U
#define FAINT_CONTRAST_CASE_COUNT 2U
#define FRAME_CUT_CASE_COUNT 2U
#define INTERRUPTION_CASE_COUNT 2U

static const uint16_t letters[26] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
    0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
    0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0};

#define STEP 10000U
#define NARROW 150000U
#define WIDE 450000U
#define LEAD 2000000U

static bc_decoder_record_t record;
static uint32_t            clock_us;
static uint32_t            elapsed_us;

typedef struct
{
    unsigned int decoded;
    unsigned int rejected;
    unsigned int timing;
    unsigned int overflow;
    char         letter;
    bool         b_is_reversed;
    uint32_t     first_decode_us;
    uint16_t     decode_dark;
} tally_t;

static tally_t      tally;
static unsigned int case_boundary;

/** @brief Reset the recorded barcode state and counters and start the
 * simulated clock just below unsigned wraparound.
 */
static void reset (void)
{
    bc_decoder_record_reset(&record);
    clock_us   = 0xFFFF0000U;
    elapsed_us = 0U;
    tally      = (tally_t){0};
}

/** @brief Advance by one sample period, feed the given darkness and
 * accumulate the recorder outcome and decode timing.
 */
static void sample (uint16_t darkness)
{
    const bc_decoder_config_t config = bc_decoder_defaults();
    bc_decoder_result_t       result;
    clock_us                        += STEP;
    elapsed_us                      += STEP;
    const bc_decoder_status_t status = bc_decoder_record_update(
        &record, &config, clock_us, true, darkness, &result);
    if ((bc_decoder_status_t)S3_BARCODE_DECODED == status)
    {
        if (0U == tally.decoded)
        {
            tally.first_decode_us = elapsed_us;
        }
        tally.decoded++;
        tally.letter        = result.letter;
        tally.b_is_reversed = result.b_is_reversed;
        tally.decode_dark   = darkness;
    }
    if ((bc_decoder_status_t)S3_BARCODE_REJECTED == status)
    {
        tally.rejected++;
    }
    if ((bc_decoder_status_t)S3_BARCODE_TIMING_ERROR == status)
    {
        tally.timing++;
    }
    if ((bc_decoder_status_t)S3_BARCODE_OVERFLOW == status)
    {
        tally.overflow++;
    }
}

/** @brief Feed a constant darkness for a duration in microseconds that must
 * be an exact multiple of the fixture sample period.
 */
static void level (uint32_t duration, uint16_t darkness)
{
    assert(duration % STEP == 0U);
    for (uint32_t elapsed_us = 0U; elapsed_us < duration; elapsed_us += STEP)
    {
        sample(darkness);
    }
}

/** @brief Build start, letter and stop run widths and optionally reverse all
 * 29 elements for the opposite scan direction.
 */
static void make_widths (char letter, bool b_is_reversed, uint32_t p_widths[29])
{
    const uint16_t symbols[3] = {0x094U, letters[(unsigned int)(letter - 'A')],
                                 0x094U};
    for (unsigned int symbol_index = 0U; symbol_index < FRAME_SYMBOL_COUNT;
         symbol_index++)
    {
        for (unsigned int element_index = 0U;
             element_index < CODE39_ELEMENT_COUNT; element_index++)
        {
            p_widths[symbol_index * 10U + element_index] =
                (symbols[symbol_index] & (1U << (8U - element_index))) ? WIDE
                                                                       : NARROW;
        }
        if (symbol_index < 2U)
        {
            p_widths[symbol_index * 10U + 9U] = NARROW;
        }
    }
    if (b_is_reversed)
    {
        for (unsigned int run_index = 0U; run_index < REVERSED_RUN_PAIRS;
             run_index++)
        {
            uint32_t swap             = p_widths[run_index];
            p_widths[run_index]       = p_widths[28U - run_index];
            p_widths[28U - run_index] = swap;
        }
    }
}

/** @brief Generate a barcode with a 60 ms optical width bias. A zero weak
 * value selects strong bars throughout. Otherwise only narrow bars
 * use the weaker contrast.
 */
static uint32_t frame (char letter, bool b_is_reversed, uint16_t strong,
                       uint16_t weak, int missing_bar, unsigned int first_run)
{
    uint32_t widths[29];
    make_widths(letter, b_is_reversed, widths);
    level(LEAD, 0U);
    for (unsigned int run_index = first_run;
         run_index < FIXTURE_FRAME_RUN_COUNT; run_index++)
    {
        const bool b_is_black = run_index % 2U == 0U;
        uint32_t   duration   = b_is_black ? widths[run_index] - 60000U
                                           : widths[run_index] + 60000U;
        uint16_t   dark =
            b_is_black ? (uint16_t)((weak != 0U && NARROW == widths[run_index])
                                          ? weak
                                          : strong)
                         : (uint16_t)0U;
        if ((int)run_index == missing_bar)
        {
            dark = 0U;
        }
        level(duration, dark);
    }
    return elapsed_us;
}

/** @brief Feed a trailing region with a repeating ten-sample noise pattern
 * at the supplied amplitude.
 */
static void noisy_tail (uint32_t duration, uint16_t amplitude,
                        unsigned int high_samples)
{
    /* Every 100 ms contains a bounded excursion and a return to low paper. */
    for (unsigned int sample_index = 0U; sample_index < duration / STEP;
         sample_index++)
    {
        sample(sample_index % 10U >= 10U - high_samples ? amplitude : 0U);
    }
}

/** @brief Require the recorder's timing-error and overflow counters to
 * remain zero.
 */
static void check_no_transport_failure (void)
{
    assert(0U == tally.timing && 0U == tally.overflow);
}

/** @brief Require one decode of the expected letter and orientation and
 * verify trailing quiet, latency and transport status.
 */
static void check_success (char letter, bool b_is_reversed, uint32_t end)
{
    if (tally.decoded != 1U || tally.letter != letter ||
        tally.b_is_reversed != b_is_reversed)
    {
        printf("quiet record %c reverse=%u boundary=%u decoded=%u letter=%c "
               "rejected=%u state=%u\n",
               letter, b_is_reversed, case_boundary, tally.decoded,
               tally.letter ? tally.letter : '-', tally.rejected, record.state);
        printf("paper=%u prominence=%u phase=%u good=%u allow_noise=%u "
               "provisional=%u "
               "strong=%u frozen=%u leading=%u quiet=%u/%u retry=%u\n",
               record.paper, record.prominence, record.work.phase,
               record.work.b_has_candidate, record.work.b_allows_noise,
               record.work.b_is_provisional, record.work.b_is_strong,
               record.work.frozen_count, record.leading_us,
               record.quiet.strict_us, record.quiet.tolerant_us,
               record.b_blocks_retry);
        fflush(stdout);
        assert(0);
    }
    assert(tally.first_decode_us >= end + 10U * NARROW - STEP);
    assert(tally.decode_dark <=
           5U); /* Never release during a bridged excursion. */
    check_no_transport_failure();
}

/** @brief Exercise both scan directions for every payload letter and return
 * the number of quiet-zone cases checked.
 */
static unsigned int all_letters_both_directions (void)
{
    unsigned int checks = 0U;
    for (unsigned int letter = 0U; letter < FIXTURE_ALPHABET_COUNT; letter++)
    {
        for (unsigned int reverse = 0U; reverse < SCAN_DIRECTION_COUNT;
             reverse++)
        {
            for (unsigned int boundary = 0U;
                 boundary < QUIET_BOUNDARY_CASE_COUNT; boundary++)
            {
                case_boundary = boundary;
                reset();
                const uint32_t end =
                    frame((char)('A' + (int)letter), reverse != 0U,
                          boundary ? 60U : 400U, 0U, -1, 0U);
                /* Tolerated paper must stay strictly below the primary peak
                 * prominence (15), otherwise it creates real extractor peaks.
                 */
                noisy_tail(3000000U, boundary ? 14U : 6U, boundary ? 4U : 1U);
                check_success((char)('A' + (int)letter), reverse != 0U, end);
                assert(0U == tally.rejected);
                checks++;
            }
        }
    }
    return checks;
}

/** @brief Check quiet-zone handling for weak frames in both directions and
 * return the number of cases examined.
 */
static unsigned int weak_frames_keep_strict_paper (void)
{
    unsigned int checks = 0U;
    for (unsigned int reverse = 0U; reverse < SCAN_DIRECTION_COUNT; reverse++)
    {
        for (unsigned int faint = 0U; faint < FAINT_CONTRAST_CASE_COUNT;
             faint++)
        {
            reset();
            const uint32_t end = frame('Z', reverse != 0U, faint ? 400U : 55U,
                                       faint ? 12U : 0U, -1, 0U);
            noisy_tail(2000000U, 12U, 2U);
            /* A tentative strong-frame check must neither accept nor discard
             * a faint scan; it still needs a genuinely clean trailing margin.
             */
            assert(0U == tally.decoded);
            assert(bc_decoder_record_active(&record));
            level(2500000U, 0U);
            if (!faint)
            {
                check_success('Z', reverse != 0U, end);
            }
            else
            {
                /* Twelve-unit marks are as dark as the faint barcode's
                 * narrow bars. They are genuine ambiguous extra ink, not
                 * something a strong-frame noise allowance may erase. */
                assert(0U == tally.decoded);
                check_no_transport_failure();
            }
            checks++;
        }
    }
    return checks;
}

/** @brief Check that the malformed frame shapes produce no decode and retain
 * valid transport timing in both directions.
 */
static unsigned int invalid_shapes_still_refused (void)
{
    unsigned int checks = 0U;
    for (unsigned int reverse = 0U; reverse < SCAN_DIRECTION_COUNT; reverse++)
    {
        for (unsigned int cut = 0U; cut < FRAME_CUT_CASE_COUNT; cut++)
        {
            reset();
            (void)frame('Z', reverse != 0U, 400U, 0U, cut ? -1 : 12,
                        cut ? 10U : 0U);
            noisy_tail(2000000U, 12U, 2U);
            level(2500000U, 0U);
            assert(0U == tally.decoded);
            check_no_transport_failure();
            checks++;
        }
    }
    return checks;
}

/** @brief Exercise interrupted quiet regions in both directions and check
 * decode refusal and transport status.
 */
static unsigned int check_quiet_interruption (void)
{
    unsigned int checks = 0U;
    for (unsigned int reverse = 0U; reverse < SCAN_DIRECTION_COUNT; reverse++)
    {
        for (unsigned int kind = 0U; kind < INTERRUPTION_CASE_COUNT; kind++)
        {
            reset();
            (void)frame('Z', reverse != 0U, 400U, 0U, -1, 0U);
            level(1400000U, 0U);
            /* Above the amplitude bound, or longer than the duration bound.
             * There is insufficient fresh quiet before this test's endpoint. */
            level(kind ? 50000U : 40000U, kind ? 12U : 15U);
            level(400000U, 0U);
            assert(0U == tally.decoded);
            check_no_transport_failure();
            checks++;
        }
    }
    return checks;
}

/** @brief Check quiet-zone acceptance across letters and directions and
 * verify weak and invalid barcode frames.
 */
int main (void)
{
    unsigned int checks = all_letters_both_directions();
    checks             += weak_frames_keep_strict_paper();
    checks             += invalid_shapes_still_refused();
    checks             += check_quiet_interruption();
    printf("PASS: %u public recording-reader noisy-paper, weak-bar and "
           "invalid-frame checks\n",
           checks);
    return 0;
}

/* End of file. */

