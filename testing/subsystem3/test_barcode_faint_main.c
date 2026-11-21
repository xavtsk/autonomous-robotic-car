/** @file test_barcode_faint_main.c
 * @brief Synthetic low-contrast printed-barcode regressions.
 *
 * These are generated
 * sensor waveforms, not additional observed hardware successes. Physical
 * Code 39 widths are fixed first; a common optical bias shortens every black
 * bar and lengthens every white space. Narrow bars have the weak contrast
 * seen in the November captures, independently of their payload letter.
 * Unsigned casts spell out the original modulo conversion.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "test_barcode_faint_main.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "bc_decoder.h"

#define FIRST_WEAK_BAR_RUN 12U
#define PAYLOAD_END_RUN 19U
#define PAPER_NOISE_SAMPLE_COUNT 160U

#define FIXTURE_ALPHABET_COUNT 26U
#define SCAN_DIRECTION_COUNT 2U
#define CODE39_ELEMENT_COUNT 9U
#define FIXTURE_FRAME_RUN_COUNT 29U

typedef enum
{
    STEP        = 10000,
    NARROW      = 100000,
    WIDE        = 250000,
    BIAS        = 60000,
    MARGIN      = 1600000,
    MAX_SAMPLES = 1200
} faint_scan_fixture_constant_t;

typedef enum
{
    WIGGLE              = 1,
    SHOULDER            = 2,
    SHORT_LEAD          = 4,
    BROKEN_TRAIL        = 8,
    CUT_START           = 16,
    EXTRA_PULSE         = 32,
    PAPER_NOISE         = 64,
    MARK_BEFORE_HISTORY = 128,
    EARLY_MARK          = 256,
    PRETRIGGER_RAMP     = 512,
    WRONG_PRIMARY       = 1024
} faint_scan_feature_t;

typedef struct
{
    unsigned int decoded;
    unsigned int rejected;
    unsigned int timing;
    unsigned int overflow;
    char         letter;
    bool         b_is_reversed;
} tally_t;

static const uint16_t g_letters[26] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
    0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
    0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0};

static unsigned int waveform (char letter, unsigned int flags,
                              unsigned int weak, uint16_t values[MAX_SAMPLES]);
static tally_t      scan (char letter, bool b_is_reverse, unsigned int flags,
                          unsigned int weak);
static void         test_noisy_paper_prefix (void);

/** @brief Test low-contrast barcode waveforms in both directions, including
 * missing quiet zones, truncated frames and extra pulses.
 */
int main (void)
{
    unsigned int letter_index;
    unsigned int dir;
    unsigned int mode;
    unsigned int checks = 0U;

    typedef struct
    {
        unsigned int flags;
        unsigned int weak;
    } faint_scan_case_t;

    const faint_scan_case_t positive[] = {
        {0U, 10U},
        {0U, 12U},
        {0U, 25U},
        {WIGGLE, 12U},
        {SHOULDER, 12U},
        {(unsigned)WIGGLE | (unsigned)SHOULDER, 12U},
        {(unsigned)WIGGLE | (unsigned)SHOULDER | (unsigned)PAPER_NOISE, 12U},
        {EARLY_MARK, 12U},
        {PRETRIGGER_RAMP, 12U},
        {WRONG_PRIMARY, 25U}};

    const unsigned int negative[] = {SHORT_LEAD, BROKEN_TRAIL, CUT_START,
                                     EXTRA_PULSE, MARK_BEFORE_HISTORY};

    for (letter_index = 0U; letter_index < FIXTURE_ALPHABET_COUNT;
         letter_index++)
    {
        for (dir = 0U; dir < SCAN_DIRECTION_COUNT; dir++)
        {
            for (mode = 0U; mode < (sizeof positive / sizeof positive[0]);
                 mode++)
            {
                const tally_t got =
                    scan((char)((unsigned)('A') + letter_index), dir != 0U,
                         positive[mode].flags, positive[mode].weak);

                if (((got.decoded != 1U) ||
                     (got.letter != (char)((unsigned)('A') + letter_index))) ||
                    (got.b_is_reversed != (dir != 0U)))
                {
                    printf("faint recovery letter=%c direction=%u mode=%u: "
                           "decoded=%u letter=%c rejected=%u\n",
                           (unsigned)('A') + letter_index, dir,
                           positive[mode].flags, got.decoded,
                           got.letter ? got.letter : '-', got.rejected);
                    fflush(stdout);
                    assert(0);
                }

                assert(0U == got.timing && 0U == got.overflow);
                checks++;
            }

            for (mode = 0U; mode < (sizeof negative / sizeof negative[0]);
                 mode++)
            {
                const tally_t got = scan((char)((unsigned)('A') + letter_index),
                                         dir != 0U, negative[mode], 12U);

                if (got.decoded != 0U)
                {
                    printf("faint refusal letter=%c direction=%u mode=%u: "
                           "decoded=%u letter=%c\n",
                           (unsigned)('A') + letter_index, dir, negative[mode],
                           got.decoded, got.letter ? got.letter : '-');
                    fflush(stdout);
                }

                assert(0U == got.decoded && 0U == got.timing &&
                       0U == got.overflow);
                checks++;
            }
        }
    }

    test_noisy_paper_prefix();
    checks++;
    printf("PASS: %u faint-bar A-Z/both-direction recovery and "
           "real-quiet/truncation/extra-pulse checks\n",
           checks);
    return 0;
}

/** @brief Synthesize a barcode waveform with the requested weak-stripe and
 * defect settings and return its number of samples.
 */
static unsigned int waveform (char letter, unsigned int flags,
                              unsigned int weak, uint16_t values[MAX_SAMPLES])
{
    const uint16_t symbols[3] = {
        0x094U, g_letters[(unsigned int)(letter - 'A')], 0x094U};
    uint32_t       physical[29];
    uint32_t       width[29];
    uint32_t       start[29];
    uint32_t       total;
    const uint32_t lead =
        (flags & (unsigned)SHORT_LEAD) ? 200000U : (unsigned)MARGIN;
    unsigned int index;
    unsigned int symbol_index;
    unsigned int sample_time_us;
    unsigned int shoulder = 29U;
    unsigned int wide_bar = 29U;
    unsigned int weak_bar = 29U;

    for (symbol_index = 0U; symbol_index < (sizeof symbols / sizeof symbols[0]);
         symbol_index++)
    {
        for (index = 0U; index < CODE39_ELEMENT_COUNT; index++)
        {
            physical[(symbol_index * 10U) + index] =
                (symbols[symbol_index] & (1U << (8U - index))) ? WIDE : NARROW;
        }

        if (symbol_index < 2U)
        {
            physical[(symbol_index * 10U) + 9U] = NARROW;
        }
    }
    /* A legitimately slightly short payload space leaves room to expose an
     * early edge caused by a leading shoulder. Choose by stripe geometry,
     * not by a payload bit pattern or expected decoded letter. */
    for (index = FIRST_WEAK_BAR_RUN; index < PAYLOAD_END_RUN; index += 2U)
    {
        if (((unsigned)NARROW == physical[index]) &&
            ((unsigned)NARROW == physical[index - 1U]))
        {
            shoulder = index;

            if (flags & (unsigned)SHOULDER)
            {
                physical[index - 1U] = 80000U;
            }

            break;
        }
    }

    {
        const bc_decoder_config_t cfg = bc_decoder_defaults();
        bc_decoder_result_t       check;
        assert(bc_decoder_decode_runs(physical, 29U, MARGIN, MARGIN, STEP, &cfg,
                                      &check));
        assert(check.letter == letter && !check.b_is_reversed);
    }
    total = lead;

    for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index++)
    {
        width[index] = (index % 2U) ? physical[index] + (unsigned)BIAS
                                    : physical[index] - (unsigned)BIAS;
        start[index] = total;
        total       += width[index];

        if ((((index % 2U) == 0U) && ((unsigned)WIDE == physical[index])) &&
            (29U == wide_bar))
        {
            wide_bar = index;
        }

        if (((((index >= 10U) && (index <= 18U)) && ((index % 2U) == 0U)) &&
             ((unsigned)NARROW == physical[index])) &&
            (29U == weak_bar))
        {
            weak_bar = index;
        }
    }

    assert(total + (unsigned)MARGIN < (unsigned)MAX_SAMPLES * (unsigned)STEP);

    for (sample_time_us  = 0U; sample_time_us < (total + (unsigned)MARGIN);
         sample_time_us += (unsigned)STEP)
    {
        uint16_t dark = 0U;

        for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index += 2U)
        {
            if ((sample_time_us >= start[index]) &&
                (sample_time_us < (start[index] + width[index])))
            {
                dark = (uint16_t)((unsigned)WIDE == physical[index]
                                      ? 220U + ((index * 17U) % 180U)
                                      : weak);

                if ((flags & (unsigned)WRONG_PRIMARY) && (index == weak_bar))
                {
                    dark = 12U;
                }

                if ((((flags & ((unsigned)WIGGLE | (unsigned)WRONG_PRIMARY)) &&
                      (index == wide_bar)) &&
                     (sample_time_us >= (start[index] + 80000U))) &&
                    (sample_time_us < (start[index] + 120000U)))
                {
                    dark = (uint16_t)(dark - 40U);
                }
            }
        }
        /* An 8 -> 6 -> 12 shoulder immediately before a weak core. */
        if ((flags & (unsigned)SHOULDER) && (shoulder < 29U))
        {
            if (sample_time_us == (start[shoulder] - 20000U))
            {
                dark = 8U;
            }

            if (sample_time_us == (start[shoulder] - 10000U))
            {
                dark = 6U;
            }
        }

        if ((flags & (unsigned)CUT_START) && (sample_time_us < start[2U]))
        {
            dark = 0U;
        }

        if (((flags & (unsigned)EXTRA_PULSE) &&
             (sample_time_us >= (start[1U] + 80000U))) &&
            (sample_time_us < (start[1U] + 120000U)))
        {
            dark = 12U;
        }

        if (((flags & (unsigned)BROKEN_TRAIL) &&
             (sample_time_us >= (total + 500000U))) &&
            (sample_time_us < (total + 550000U)))
        {
            dark = 12U;
        }

        if ((flags & (unsigned)PAPER_NOISE) && (0U == dark))
        {
            dark = (sample_time_us / (unsigned)STEP) % 2U ? 3U : 0U;
        }
        /* This 100 ms faint mark leaves only 960 ms of observed white before
         * the first bar. By the first strong bar's trigger it is outside the
         * retained 1.5 s history; it must still invalidate the leading quiet
         * credit. Reversing the same waveform tests a trailing interruption.
         * The earlier control mark leaves enough real white on either side. */
        if (((flags & (unsigned)MARK_BEFORE_HISTORY) &&
             (sample_time_us >= 540000U)) &&
            (sample_time_us < 640000U))
        {
            dark = 12U;
        }

        if (((flags & (unsigned)EARLY_MARK) && (sample_time_us >= 200000U)) &&
            (sample_time_us < 300000U))
        {
            dark = 12U;
        }
        /* A genuine rising bar shoulder below the recording-start rise must
         * not be learned as quiet-paper noise and disable weak-bar recovery. */
        if ((flags & (unsigned)PRETRIGGER_RAMP) &&
            (sample_time_us == (start[wide_bar] - (unsigned)STEP)))
        {
            dark = 45U;
        }

        values[sample_time_us / (unsigned)STEP] = dark;
    }

    return (total + (unsigned)MARGIN) / (unsigned)STEP;
}

/** @brief Run the generated waveform through the recorder and return counts
 * of successful decodes and each failure status.
 */
static tally_t scan (char letter, bool b_is_reverse, unsigned int flags,
                     unsigned int weak)
{
    static uint16_t            values[MAX_SAMPLES];
    static bc_decoder_record_t record;
    const bc_decoder_config_t  cfg   = bc_decoder_defaults();
    const unsigned int         count = waveform(letter, flags, weak, values);
    tally_t                    out   = {0};
    uint32_t                   clock = 0xFFFF0000U;
    unsigned int               index;
    bc_decoder_record_reset(&record);

    for (index = 0U; index < count; index++)
    {
        bc_decoder_result_t result;
        bc_decoder_status_t status;
        clock += (unsigned)STEP;
        status = bc_decoder_record_update(
            &record, &cfg, clock, true,
            values[b_is_reverse ? count - 1U - index : index], &result);

        if ((unsigned)S3_BARCODE_DECODED == status)
        {
            out.decoded++;
            out.letter        = result.letter;
            out.b_is_reversed = result.b_is_reversed;
        }

        if ((unsigned)S3_BARCODE_REJECTED == status)
        {
            out.rejected++;
        }

        if ((unsigned)S3_BARCODE_TIMING_ERROR == status)
        {
            out.timing++;
        }

        if ((unsigned)S3_BARCODE_OVERFLOW == status)
        {
            out.overflow++;
        }
    }

    return out;
}

/** @brief Feed noisy paper readings ahead of a recorded frame and check the
 * active recorder's prominence selection.
 */
static void test_noisy_paper_prefix (void)
{
    static bc_decoder_record_t record;
    const bc_decoder_config_t  cfg = bc_decoder_defaults();
    bc_decoder_result_t        result;
    uint32_t                   clock = 0U;
    unsigned int               index;
    bc_decoder_record_reset(&record);
    /* Repeated 12-unit paper excursions provide no stable low-band prefix.
     * A later strong bar must not cause that measured noise to be replaced
     * by a clean-paper estimate merely because some individual readings are
     * zero. Do not require a particular estimator formula, just preserve a
     * noise gate above the low-noise recovery eligibility limit. */
    for (index = 0U; index < PAPER_NOISE_SAMPLE_COUNT; index++)
    {
        clock += (unsigned)STEP;
        assert(bc_decoder_record_update(&record, &cfg, clock, true,
                                        (index % 2U) ? 12U : 0U,
                                        &result) == (unsigned)S3_BARCODE_NONE);
    }

    clock += (unsigned)STEP;
    assert(bc_decoder_record_update(&record, &cfg, clock, true, 500U,
                                    &result) == (unsigned)S3_BARCODE_NONE);
    assert(bc_decoder_record_active(&record));
    assert(record.prominence > S3_REC_FAINT_MAX_NOISE);
}

/* End of file. */

