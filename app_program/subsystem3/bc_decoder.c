/** @file bc_decoder.c
 * @brief Decode Code 39 letters from bounded streaming and recorded reflectance
 * samples.
 * @details Streaming decoding consumes transitions in time order. The
 * recorded decoder retains pre-trigger readings and advances fitting by a
 * fixed work budget on each subsequent sample.
 * @note The target uses contiguous A..Z character codes. Keep masks,
 * validated state limits and letter offsets in their unsigned domains.
 * The target's int represents 8- and 16-bit operands, and int64_t
 * represents every uint32_t width before signed fit arithmetic.
 * @see app_program/subsystem3/INTEGRATION.md for the recorded decode path.
 */
/* Code 39 symbol facts cross-checked against ZXing Code39Reader encodings:
 * Repository: github.com/zxing/zxing
 * Source: core/src/main/java/com/google/zxing/oned/Code39Reader.java
 * This decoder uses its own bounded duration/sampling validation algorithm. */
#include "bc_decoder.h"

#if defined(BC_DECODER_FIT_TEST)
#include "fit_score_main.h"
#endif
#if defined(BC_DECODER_WORK_TEST)
#include "work_quota_main.h"
#endif
#if defined(BC_DECODER_BOUNDARY_TEST)
#include "decode_work_main.h"
#endif

#define CODE39_ELEMENTS (9U)
#define CODE39_NARROW_ELEMENTS (6U)
#define CODE39_WIDE_ELEMENTS (3U)
#define CODE39_FRAME_CHARACTERS (3U)
#define CODE39_INTERCHARACTER_GAPS (2U)
#define CODE39_SENTINELS (2U)
#define CODE39_ORIENTATIONS (2U)
#define RECORD_MIN_LEAD_SAMPLES (1U)
#define RECORD_EDGE_ENDPOINTS (2U)
#define RECORD_NOISE_PAIR (2U)
/* Signed coordinates on either side of the zero-bias fit. */
#define BIAS_GRID_RADIUS (64)

typedef enum
{
    RW_PRIMARY = 1,
    RW_FAINT_WHITE,
    RW_FAINT_INK,
    RW_FAINT_VALLEY,
    RW_PRUNE_BEGIN,
    RW_PRUNE_SCAN,
    RW_PRUNE_COPY,
    RW_EDGES_BEGIN,
    RW_EDGE_BEGIN,
    RW_EDGE_SCAN,
    RW_DIRECT,
    RW_FIT_BEGIN,
    RW_FIT_TICK,
    RW_FIT_END,
    RW_APPLY,
    RW_VALIDATE,
    RW_PATH_END,
    RW_DONE
} bc_decoder_record_phase_t;

/* A stored sample contains ten darkness bits and six timing-step bits.
 * Widen before shifting. The largest recovered step is 63 times 500 us. */
#define REC_DARK(r, i) ((unsigned int)((r)->sample[(i)] & 0x3FFU))

#define REC_STEP_US(r, i) (((uint32_t)(r)->sample[(i)] >> 10U) * S3_REC_STEP_US)

static const uint16_t g_letters[S3_BARCODE_LETTERS] = {
    0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U, 0x00DU, 0x10CU, 0x04CU,
    0x01CU, 0x103U, 0x043U, 0x142U, 0x013U, 0x112U, 0x052U, 0x007U, 0x106U,
    0x046U, 0x016U, 0x181U, 0x0C1U, 0x1C0U, 0x091U, 0x190U, 0x0D0U};

static bool config_valid (const bc_decoder_config_t * p_config);

static bool character_widths (const uint32_t * p_runs, uint32_t gap,
                              const bc_decoder_config_t * p_config,
                              uint16_t * p_pattern, uint32_t * p_narrow,
                              bool b_is_payload);

static bool character (const uint32_t * p_runs, uint32_t gap,
                       const bc_decoder_config_t * p_config,
                       uint16_t * p_pattern, uint32_t * p_narrow);

static void seed_decoder (bc_decoder_t * p_decoder, uint32_t now,
                          bool b_is_black);

static bool rec_step (bc_decoder_record_t * p_record, uint32_t gap_us,
                      uint16_t * p_step);

static bool rec_faint_eligible (const bc_decoder_record_t * p_record);

static unsigned int rec_monitor_band (const bc_decoder_record_t * p_record);

static unsigned int rec_white_band (const bc_decoder_record_t * p_record);

static bool rec_white (const bc_decoder_record_t * p_record, unsigned int dark);

static uint32_t sat_add (uint32_t elapsed_us, uint32_t delta_us);

static void rec_pre_drop (bc_decoder_record_t * p_record);

static void rec_pre_push (bc_decoder_record_t * p_record, unsigned int dark,
                          uint16_t step);

static unsigned int rec_paper_prominence (const bc_decoder_record_t * p_record,
                                          const bc_decoder_config_t * p_config);

static uint32_t rec_frac (unsigned int darkness, unsigned int value_lo,
                          unsigned int value_hi);

static unsigned int rec_faint_extrema (const bc_decoder_record_t * p_recorder,
                                       uint16_t p_ext[S3_REC_EXT_MAX]);

static unsigned int
rec_extract_runs (const bc_decoder_record_t * p_record,
                  uint32_t p_runs[S3_BARCODE_RUNS], uint32_t * p_first_edge_us,
                  uint32_t * p_last_edge_us, uint32_t * p_end_us,
                  uint32_t * p_max_step_us, bool b_is_faint);

static uint32_t rec_narrow (const uint32_t * p_runs);

static uint32_t rec_fit_error (const uint32_t p_adjusted[CODE39_ELEMENTS],
                               uint16_t star, uint32_t narrow, uint32_t wide);

static bool rec_correct_shared_runs (uint32_t p_runs[S3_BARCODE_RUNS],
                                     uint32_t gap,
                                     const bc_decoder_config_t * p_config,
                                     int32_t *                   p_bias_us,
                                     bool * pb_did_match_sentinels);

static bool rec_fit_star (const uint32_t * p_runs, uint16_t star, uint32_t gap,
                          const bc_decoder_config_t * p_config,
                          int32_t * p_bias, uint32_t * p_narrow);

#ifdef S3_BARCODE_SYNC_REFERENCE
static bool rec_candidate (const bc_decoder_record_t * p_record,
                           const bc_decoder_config_t * p_config,
                           bool b_is_faint, bc_decoder_result_t * p_candidate,
                           uint32_t * p_required_us);

#endif
static void work_next (bc_decoder_record_t * p_recorder);

static void work_path_begin (bc_decoder_record_t * p_record, bool b_is_faint);

static void work_begin (bc_decoder_record_t * p_record, bool b_is_provisional,
                        bool b_is_early);

static void work_bad_path (bc_decoder_record_t * p_record);

static void work_keep_candidate (bc_decoder_record_t *       p_record,
                                 const bc_decoder_config_t * p_config,
                                 const bc_decoder_result_t * p_candidate);

static uint32_t work_fit_trial (bc_decoder_record_t *       p_record,
                                const bc_decoder_config_t * p_config);

static bool work_apply (bc_decoder_work_t *         p_work,
                        const bc_decoder_config_t * p_config);

static void work_primary (bc_decoder_record_t * p_recorder);

static void work_faint_white (bc_decoder_record_t * p_recorder);

static void work_faint_ink (bc_decoder_record_t * p_recorder);

static void work_faint_valley (bc_decoder_record_t * p_recorder);

static void work_prune_begin (bc_decoder_record_t * p_recorder);

static void work_prune_scan (bc_decoder_record_t * p_recorder);

static void work_prune_copy (bc_decoder_record_t * p_recorder);

static void work_edges_begin (bc_decoder_record_t * p_recorder);

static void work_edge_begin (bc_decoder_record_t * p_recorder);

static void work_edge_scan (bc_decoder_record_t * p_recorder);

static void work_validate (bc_decoder_record_t *       p_recorder,
                           const bc_decoder_config_t * p_config);

static void work_advance (bc_decoder_record_t *       p_recorder,
                          const bc_decoder_config_t * p_config);

static bc_decoder_status_t rec_finish (bc_decoder_record_t * p_record,
                                       bc_decoder_status_t   status);

/** @brief Return the integrated reader's default timing bounds in microseconds
 * and its quiet-zone length in narrow modules.
 */
bc_decoder_config_t bc_decoder_defaults (void)
{
    /* At 0.04 m/s a 19 mm wide element lasts 475 ms. A 2:1 symbol can
     * require ~11.4 s including trailing quiet; allow bounded extra time. */
    /* At 0.10 m/s the small F's measured narrow means are about 29..35 ms.
     * Permit 25 ms modules, retaining character()'s independent requirement
     * of at least three observed sample intervals per narrow mean. */
    return (bc_decoder_config_t){30000U, 25000U, 1000000U, 15000000U, 10U};
}

/** @brief Clear streaming history while retaining the caller's stored
 * configuration and configured flag. A null decoder is ignored.
 */
void bc_decoder_reset (bc_decoder_t * p_decoder)
{
    if (p_decoder != 0)
    {
        bc_decoder_stored_config_t config = p_decoder->config;
        const bool b_was_configured       = p_decoder->b_is_configured;
        *p_decoder                        = (bc_decoder_t){0};
        p_decoder->config                 = config;
        p_decoder->b_is_configured        = b_was_configured;
    }
}

/** @brief Clear the supplied decoder, validate the timing configuration and
 * install it on success. Rejected configuration leaves a non-null
 * decoder cleared.
 */
bool bc_decoder_init (bc_decoder_t *              p_decoder,
                      const bc_decoder_config_t * p_config)
{
    if (0 == p_decoder)
    {
        return false;
    }

    *p_decoder = (bc_decoder_t){0};

    if (!config_valid(p_config))
    {
        return false;
    }

    /* clang-format off */
    p_decoder->config          = (bc_decoder_stored_config_t){
        p_config->max_gap_us, p_config->min_narrow_us, p_config->max_run_us,
        p_config->max_frame_us, p_config->quiet_modules};
    p_decoder->b_is_configured = true;
    /* clang-format on */
    return true;
}

/** @brief Validate 29 recorded run widths in microseconds against both quiet
 * margins and the observed sampling gap. Try both scan directions.
 * Clear a non-null result before validation and populate it for an
 * accepted letter.
 * @details Divide the frame into opening sentinel, payload and closing
 * sentinel. Check narrow and wide elements and both inter-character gaps
 * before matching the letter. The recorded quiet time and maximum sampling
 * step must also satisfy the candidate's width limits.
 */
bool bc_decoder_decode_runs (const uint32_t * p_runs, unsigned int count,
                             uint32_t leading_us, uint32_t trailing_us,
                             uint32_t                    observed_gap_us,
                             const bc_decoder_config_t * p_config,
                             bc_decoder_result_t *       p_result)
{
    unsigned int orientation;
    unsigned int index;
    unsigned int letter_index;
    uint64_t     total = 0U;

    if (0 == p_result)
    {
        return false;
    }

    *p_result = (bc_decoder_result_t){0};

    if ((((0 == p_runs) || (count != S3_BARCODE_RUNS)) ||
         (!config_valid(p_config))) ||
        (observed_gap_us > p_config->max_gap_us))
    {
        return false;
    }

    for (index = 0U; index < count; index++)
    {
        if ((0U == p_runs[index]) || (p_runs[index] > p_config->max_run_us))
        {
            return false;
        }

        total += p_runs[index];
    }

    if (total > p_config->max_frame_us)
    {
        return false;
    }

    for (orientation = 0U; orientation < CODE39_ORIENTATIONS; orientation++)
    {
        uint32_t ordered[S3_BARCODE_RUNS];
        uint32_t narrow[CODE39_FRAME_CHARACTERS];
        uint16_t pattern[CODE39_FRAME_CHARACTERS];
        bool     b_is_valid = true;

        for (index = 0U; index < count; index++)
        {
            ordered[index] = p_runs[orientation ? count - 1U - index : index];
        }

        for (index = 0U; index < CODE39_FRAME_CHARACTERS; index++)
        {
            if (!character_widths(&ordered[index * 10U], observed_gap_us,
                                  p_config, &pattern[index], &narrow[index],
                                  1U == index))
            {
                b_is_valid = false;
                break;
            }
        }

        if (((!b_is_valid) || (pattern[0] != 0x094U)) || (pattern[2] != 0x094U))
        {
            continue;
        }

        /* Widen the two 32-bit widths before adding. The doubling and
         * tripling in the gap check also stay in the widened domain. */
        for (index = 0U; index < CODE39_INTERCHARACTER_GAPS; index++)
        {
            const uint64_t reference =
                ((uint64_t)narrow[index] + narrow[index + 1U]) / 2U;
            const uint64_t space = ordered[9U + (index * 10U)];

            if (((space * 2U) < reference) || ((space * 2U) > (reference * 3U)))
            {
                b_is_valid = false;
            }
        }

        /* Keep the full 32-bit quiet times and form each module product
         * in 64 bits before comparing either end of the frame. */
        if (((!b_is_valid) ||
             ((uint64_t)(orientation ? trailing_us : leading_us) <
              ((uint64_t)p_config->quiet_modules * narrow[0]))) ||
            ((uint64_t)(orientation ? leading_us : trailing_us) <
             ((uint64_t)p_config->quiet_modules * narrow[2])))
        {
            continue;
        }

        for (letter_index = 0U;
             letter_index < (sizeof g_letters / sizeof g_letters[0]);
             letter_index++)
        {
            if (pattern[1] == g_letters[letter_index])
            {
                /* The target's contiguous A..Z codes fit char. The loop
                 * limits the unsigned offset to one of the 26 entries. */
                /* clang-format off */
                p_result->letter        =
                    (char)((unsigned int)'A' + letter_index);
                p_result->b_is_reversed = orientation != 0U;
                /* clang-format on */
                return true;
            }
        }
    }

    return false;
}

/** @brief Consume one timestamped black or white reading in the caller's
 * streaming decoder. Publish a letter only after the trailing quiet
 * interval. Return a distinct status for an invalid frame, missed
 * timing or excess runs.
 * @details Use unsigned elapsed microseconds through clock rollover.
 * After the first valid reading, a repeated timestamp or excessive gap
 * starts a new history and reports a timing error. A frame starts on black
 * and measures each transition until 29 runs are present. A fitted letter
 * remains provisional until a white reading proves the trailing interval.
 * Invalid samples clear streaming history. Rejected, mistimed or oversized
 * frames may seed the next history with the current valid reading.
 * @note A non-null result is cleared before processing, and only a
 * confirmed decode fills it. A null result leaves the decoder untouched.
 */
bc_decoder_status_t bc_decoder_update (bc_decoder_t * p_decoder,
                                       uint32_t timestamp_us, bool b_is_valid,
                                       bool                  b_is_black,
                                       bc_decoder_result_t * p_result)
{

    if (0 == p_result)
    {
        return S3_BARCODE_REJECTED;
    }

    *p_result = (bc_decoder_result_t){0};

    if ((0 == p_decoder) || (!p_decoder->b_is_configured))
    {
        return S3_BARCODE_REJECTED;
    }

    if (!b_is_valid)
    {
        bc_decoder_reset(p_decoder);
        return S3_BARCODE_REJECTED;
    }

    if (!p_decoder->b_has_sample)
    {
        seed_decoder(p_decoder, timestamp_us, b_is_black);
        return S3_BARCODE_NONE;
    }

    const uint32_t gap = timestamp_us - p_decoder->last_us;

    if ((0U == gap) || (gap > p_decoder->config.max_gap_us))
    {
        seed_decoder(p_decoder, timestamp_us, b_is_black);
        return S3_BARCODE_TIMING_ERROR;
    }

    p_decoder->last_us     = timestamp_us;
    const uint32_t elapsed = timestamp_us - p_decoder->run_start_us;

    if (p_decoder->b_is_collecting)
    {
        if (gap > p_decoder->observed_gap_us)
        {
            p_decoder->observed_gap_us = gap;
        }

        if ((timestamp_us - p_decoder->frame_start_us) >
            p_decoder->config.max_frame_us)
        {
            seed_decoder(p_decoder, timestamp_us, b_is_black);
            return S3_BARCODE_REJECTED;
        }
    }

    if (p_decoder->b_is_pending)
    {
        if (b_is_black)
        {
            seed_decoder(p_decoder, timestamp_us, b_is_black);
            return S3_BARCODE_REJECTED;
        }

        if (elapsed >= p_decoder->trailing_required_us)
        {
            /* Code zero denotes no letter. Codes 1..26 become A..Z,
             * which fit char on the target. */
            /* clang-format off */
            p_result->letter           =
                0U == p_decoder->candidate.letter_code
                    ? (char)0
                    : (char)((unsigned int)('A' + (int)p_decoder->candidate
                                                      .letter_code) -
                             1U);
            p_result->b_is_reversed    = p_decoder->candidate.b_is_reversed;
            p_result->timestamp_us     = timestamp_us;
            p_decoder->b_is_collecting = false;
            p_decoder->b_is_pending    = false;
            p_decoder->count           = 0U;
            /* clang-format on */
            return S3_BARCODE_DECODED;
        }

        return S3_BARCODE_NONE;
    }

    if (b_is_black != p_decoder->b_is_black)
    {
        if (p_decoder->b_is_collecting)
        {
            if (p_decoder->count >= S3_BARCODE_RUNS)
            {
                seed_decoder(p_decoder, timestamp_us, b_is_black);
                return S3_BARCODE_OVERFLOW;
            }

            p_decoder->runs[p_decoder->count++] = elapsed;

            if (S3_BARCODE_RUNS == p_decoder->count)
            {
                uint16_t                  pattern;
                uint32_t                  narrow;
                bc_decoder_result_t       candidate;
                const bc_decoder_config_t config = {
                    p_decoder->config.max_gap_us,
                    p_decoder->config.min_narrow_us,
                    p_decoder->config.max_run_us,
                    p_decoder->config.max_frame_us,
                    p_decoder->config.quiet_modules};

                if ((b_is_black ||
                     (!bc_decoder_decode_runs(
                         p_decoder->runs, p_decoder->count,
                         p_decoder->leading_us, 0x7FFFFFFFU,
                         p_decoder->observed_gap_us, &config, &candidate))) ||
                    (!character(&p_decoder->runs[20],
                                p_decoder->observed_gap_us, &config, &pattern,
                                &narrow)))
                {
                    seed_decoder(p_decoder, timestamp_us, b_is_black);
                    return S3_BARCODE_REJECTED;
                }

                /* A successful frame returns A..Z. Subtracting 'A' and
                 * adding one yields the retained code in 1..26. */
                /* clang-format off */
                p_decoder->candidate            = (bc_decoder_stored_result_t){
                    (uint8_t)((int)candidate.letter - 'A' + 1),
                    candidate.b_is_reversed};
                p_decoder->trailing_required_us =
                    narrow * p_decoder->config.quiet_modules;
                p_decoder->b_is_pending         = true;
                /* clang-format on */
            }
        }
        else if (b_is_black)
        {
            p_decoder->leading_us      = elapsed;
            p_decoder->frame_start_us  = timestamp_us;
            p_decoder->b_is_collecting = true;
            p_decoder->count           = 0U;
            p_decoder->observed_gap_us = gap;
        }
        else
        {
            /* Only a black transition starts collection outside a frame. */
        }

        p_decoder->b_is_black   = b_is_black;
        p_decoder->run_start_us = timestamp_us;
    }
    else if ((p_decoder->b_is_collecting) &&
             (elapsed > p_decoder->config.max_run_us))
    {
        seed_decoder(p_decoder, timestamp_us, b_is_black);
        return S3_BARCODE_REJECTED;
    }
    else
    {
        /* No transition or timeout needs processing on this sample. */
    }

    return S3_BARCODE_NONE;
}

/** @brief Clear the full recording and work state in caller storage. Accept a
 * null pointer as a no-op.
 */
void bc_decoder_record_reset (bc_decoder_record_t * p_record)
{
    if (p_record != 0)
    {
        *p_record = (bc_decoder_record_t){0};
    }
}

/** @brief Return whether a non-null recorder has left its waiting state,
 * including processing and quiet-zone confirmation.
 */
bool bc_decoder_record_active (const bc_decoder_record_t * p_record)
{
    return (p_record != 0) && (p_record->state != (uint16_t)S3_REC_WAITING);
}

/** @brief Extract primary widths first. If the count differs from the full
 * frame and the paper permits faint recovery, retry that path. Return
 * the extracted count and the corresponding timing outputs.
 */
unsigned int bc_decoder_record_runs (const bc_decoder_record_t * p_record,
                                     uint32_t   p_runs[S3_BARCODE_RUNS],
                                     uint32_t * p_first_edge_us,
                                     uint32_t * p_last_edge_us,
                                     uint32_t * p_end_us,
                                     uint32_t * p_max_step_us)
{
    unsigned int run_count =
        rec_extract_runs(p_record, p_runs, p_first_edge_us, p_last_edge_us,
                         p_end_us, p_max_step_us, false);

    if ((run_count != S3_BARCODE_RUNS) && (rec_faint_eligible(p_record)))
    {
        run_count =
            rec_extract_runs(p_record, p_runs, p_first_edge_us, p_last_edge_us,
                             p_end_us, p_max_step_us, true);
    }

    return run_count;
}

/** @brief Fit one bias to both sentinels of a 29-run frame. Return the signed
 * correction in microseconds on success. The caller still checks both
 * quiet zones and the payload.
 */
bool bc_decoder_record_correct_runs (uint32_t p_runs[S3_BARCODE_RUNS],
                                     uint32_t observed_gap_us,
                                     const bc_decoder_config_t * p_config,
                                     int32_t *                   p_bias_us)
{
    return rec_correct_shared_runs(p_runs, observed_gap_us, p_config, p_bias_us,
                                   0);
}

/** @brief Fit the two sentinels independently and interpolate their edge
 * offsets across the frame. Return both corrections on success. Reject
 * inconsistent fits before committing corrected widths.
 */
bool bc_decoder_record_fix_varying (uint32_t p_runs[S3_BARCODE_RUNS],
                                    uint32_t observed_gap_us,
                                    const bc_decoder_config_t * p_config,
                                    int32_t                     p_bias_us[2])
{
    uint32_t     adjusted[S3_BARCODE_RUNS];
    uint64_t     total      = 0U;
    uint64_t     start      = 0U;
    uint64_t     finish     = 0U;
    uint64_t     elapsed_us = 0U;
    int32_t      chosen[2]  = {0, 0};
    unsigned int run_index;
    unsigned int orientation;
    unsigned int found = 0U;

    if ((((0 == p_runs) || (0 == p_bias_us)) || (!config_valid(p_config))) ||
        (observed_gap_us > p_config->max_gap_us))
    {
        return false;
    }

    p_bias_us[0] = 0;
    p_bias_us[1] = 0;

    for (run_index = 0U; run_index < S3_BARCODE_RUNS; run_index++)
    {
        if ((0U == p_runs[run_index]) ||
            (p_runs[run_index] > p_config->max_run_us))
        {
            return false;
        }

        total += p_runs[run_index];

        if (8U == run_index)
        {
            /* Edge 9 ends the opening sentinel. */
            start = total;
        }

        if (19U == run_index)
        {
            /* Edge 20 begins the closing sentinel. */
            finish = total;
        }
    }

    if (total > p_config->max_frame_us)
    {
        return false;
    }

    for (orientation = 0U; orientation < CODE39_ORIENTATIONS; orientation++)
    {
        const uint16_t star = orientation ? 0x052U : 0x094U;
        uint32_t       narrow[2];
        int32_t        bias[2];

        if ((!rec_fit_star(p_runs, star, observed_gap_us, p_config, &bias[0],
                           &narrow[0])) ||
            (!rec_fit_star(&p_runs[20], star, observed_gap_us, p_config,
                           &bias[1], &narrow[1])))
        {
            continue;
        }
        /* Two endpoints do not justify arbitrary distortion between them. */
        if (((bias[0] < 0) && (bias[1] > 0)) ||
            ((bias[0] > 0) && (bias[1] < 0)))
        {
            continue;
        }

        const uint32_t minimum =
            (narrow[0] < narrow[1]) ? narrow[0] : narrow[1];
        const int32_t delta = bias[1] - bias[0];

        /* The accepted biases share a sign or include zero. Their
         * bounded difference is safe to take to an unsigned magnitude. */
        if ((uint32_t)(delta < 0 ? -delta : delta) > (minimum / 2U))
        {
            continue;
        }

        chosen[0] = bias[0];
        chosen[1] = bias[1];
        found++;
    }

    if (found != 1U)
    {
        return false;
    }

    int32_t previous_shift = -chosen[0] / 2;

    for (run_index = 0U; run_index < S3_BARCODE_RUNS; run_index++)
    {
        int32_t bias;
        elapsed_us += p_runs[run_index];

        if (elapsed_us <= start)
        {
            bias = chosen[0];
        }
        else if (elapsed_us >= finish)
        {
            bias = chosen[1];
        }
        else
        {
            /* Form the weighted sum in 64 bits. The quotient lies between
             * the two signed biases, so int32_t preserves it. Interpolate
             * one shift per edge so neighbouring widths stay adjacent. */
            bias = (int32_t)((((int64_t)chosen[0] *
                               (int64_t)(finish - elapsed_us)) +
                              ((int64_t)chosen[1] *
                               (int64_t)(elapsed_us - start))) /
                             (int64_t)(finish - start));
        }

        /* Widen the 32-bit width before applying the two signed shifts.
         * Only positive widths through max_run_us narrow back to uint32_t. */
        const int32_t shift = (run_index % 2U) ? -bias / 2 : bias / 2;
        const int64_t width =
            (int64_t)p_runs[run_index] + shift - previous_shift;

        if ((width <= 0) || (width > (int64_t)p_config->max_run_us))
        {
            return false;
        }

        adjusted[run_index] = (uint32_t)width;
        previous_shift      = shift;
    }

    for (run_index = 0U; run_index < S3_BARCODE_RUNS; run_index++)
    {
        p_runs[run_index] = adjusted[run_index];
    }

    p_bias_us[0] = chosen[0];
    p_bias_us[1] = chosen[1];
    return true;
}

/** @brief Consume one timestamped normalized darkness sample in caller-owned
 * storage. Clamp darkness to 1000 and advance analysis within the
 * per-call work limits. A decoded result includes the letter,
 * direction and decision time after the required quiet interval.
 */
bc_decoder_status_t
bc_decoder_record_update (bc_decoder_record_t *       p_record,
                          const bc_decoder_config_t * p_config,
                          uint32_t timestamp_us, bool b_is_valid,
                          uint16_t darkness, bc_decoder_result_t * p_result)
{
    /* Clamp darkness to 0..1000 before retaining the 10-bit samples
     * and updating the paper and previous-sample fields. */
    const unsigned int dark = (darkness > 1000U) ? 1000U : darkness;
    uint16_t           step = 0U;

    if (((0 == p_result) || (0 == p_record)) || (!config_valid(p_config)))
    {
        return S3_BARCODE_NONE;
    }

    p_record->work_units = 0U;
    p_record->fit_ticks  = 0U;
    *p_result            = (bc_decoder_result_t){0};

    if (!b_is_valid)
    {
        bc_decoder_record_reset(p_record);
        return S3_BARCODE_NONE;
    }

    if (!p_record->b_has_sample)
    {
        p_record->b_has_sample = 1U;
        p_record->paper        = (uint16_t)dark;
        p_record->last_us      = timestamp_us;
        p_record->prev         = (uint16_t)dark;
        rec_pre_push(p_record, dark, 0U);
        return S3_BARCODE_NONE;
    }

    if ((!rec_step(p_record, timestamp_us - p_record->last_us, &step)) ||
        (((uint32_t)step * S3_REC_STEP_US) >
         (p_config->max_gap_us + S3_REC_STEP_US)))
    {
        const bool b_was_recording = bc_decoder_record_active(p_record);
        bc_decoder_record_reset(p_record);
        p_record->b_has_sample = 1U;
        p_record->paper        = (uint16_t)dark;
        p_record->last_us      = timestamp_us;
        p_record->prev         = (uint16_t)dark;
        rec_pre_push(p_record, dark, 0U);
        return b_was_recording ? S3_BARCODE_TIMING_ERROR : S3_BARCODE_NONE;
    }

    p_record->last_us = timestamp_us;

    if ((uint16_t)S3_REC_WAITING == p_record->state)
    {
        /* Learn the paper: follow lighter readings at once, darker ones slowly.
         */
        if (dark < p_record->paper)
        {
            p_record->paper = (uint16_t)dark;
        }
        else if (dark < ((unsigned int)p_record->paper + S3_REC_RISE))
        {
            /* Move at most to the clamped darkness. The updated paper
             * stays within 0..1000 when initialized by this recorder. */
            p_record->paper =
                (uint16_t)(p_record->paper + ((dark - p_record->paper) >> 5));
        }
        else
        {
            /* A starting bar must not raise the paper reference. */
        }

        if ((p_record->b_is_blocked) &&
            ((!rec_white(p_record, dark)) ||
             (dark >= ((unsigned int)p_record->paper + S3_REC_RISE))))
        {
            p_record->white_us = 0U;
            return S3_BARCODE_NONE;
        }

        if (p_record->b_is_blocked)
        {
            p_record->b_is_blocked = 0U;
        }

        if (dark < ((unsigned int)p_record->paper + S3_REC_RISE))
        {
            /* Learn noise only near paper, not from the rising bar. Freeze
             * it once recording starts so bars cannot raise their own gate. */
            if ((dark <= ((unsigned int)p_record->paper + S3_REC_WHITE_BAND)) &&
                (p_record->prev <=
                 ((unsigned int)p_record->paper + S3_REC_WHITE_BAND)))
            {
                const unsigned int delta = (dark > p_record->prev)
                                               ? dark - p_record->prev
                                               : p_record->prev - dark;
                /* Starting at zero, delta <= 1000 keeps this recurrence
                 * at or below 16000. The cast preserves the estimate. */
                p_record->noise_estimate =
                    (uint16_t)(p_record->noise_estimate -
                               (p_record->noise_estimate / 16U) + delta);
            }
            /* Margin before a barcode: everything short of a bar counts, as
             * in bc_decoder_update (the rising slope is never a bar). */
            /* clang-format off */
            p_record->white_us =
                sat_add(p_record->white_us, (uint32_t)step * S3_REC_STEP_US);
            p_record->prev     = (uint16_t)dark;
            /* clang-format on */
            rec_pre_push(p_record, dark, step);
            return S3_BARCODE_NONE;
        }
        /* Trigger on a stronger bar, retaining real earlier readings so a
         * faint opening star/slow edge is not discarded. The history is part
         * of the already-measured leading interval, never extra quiet time. */
        /* The recording state fits the narrow code. Prominence clamps
         * the estimator between its configured minimum and maximum. */
        /* clang-format off */
        p_record->state      = (uint16_t)S3_REC_RECORDING;
        p_record->prominence =
            (uint16_t)rec_paper_prominence(p_record, p_config);
        p_record->prefix_us  = p_record->pre_us;
        /* clang-format on */

        if (!rec_faint_eligible(p_record))
        {
            p_record->leading_us =
                (p_record->white_us >= p_record->prefix_us)
                    ? p_record->white_us - p_record->prefix_us
                    : 0U;
        }

        p_record->white_us    = 0U;
        p_record->frame_us    = p_record->prefix_us;
        p_record->required_us = 0U;
        bc_quiet_reset(&p_record->quiet);
        p_record->b_blocks_retry        = false;
        p_record->b_accepts_faint_quiet = false;
        p_record->b_did_try_early_fit   = false;
        p_record->b_has_early_candidate = false;
        p_record->count                 = p_record->pre_count;

        /* The ring head is below 320 and the copy index stays below its
         * retained count. Unsigned addition and modulo keep the index in
         * the ring as the copy crosses its end. */
        for (unsigned int history_index = 0U;
             history_index < p_record->pre_count; history_index++)
        {
            p_record->sample[history_index] =
                p_record
                    ->pre[((unsigned int)p_record->pre_head + history_index) %
                          S3_REC_PRE_MAX];
        }

        if (0U == p_record->count)
        {
            p_record->sample[0] = p_record->prev;
            p_record->count     = 1U;
        }

        /* The previous clamped darkness fits the wider extremum field. */
        p_record->best         = (uint16_t)p_record->prev;
        p_record->b_is_falling = 0U;
        p_record->turn_us      = 0U;
    }

    if (p_record->count >= S3_REC_MAX)
    {
        return rec_finish(p_record, S3_BARCODE_OVERFLOW);
    }

    /* Normalized darkness uses ten bits and the accepted step uses six.
     * The packed sample occupies all sixteen bits of the destination. */
    /* clang-format off */
    p_record->sample[p_record->count++] =
        (uint16_t)(dark | ((unsigned int)step << 10));
    p_record->frame_us                  =
        sat_add(p_record->frame_us, (uint32_t)step * S3_REC_STEP_US);
    /* clang-format on */
    /* The batch validator still limits the actual 29 widths to max_frame_us.
     * Leading history must not shorten the existing post-trigger timeout. */
    if ((p_record->frame_us - p_record->prefix_us) > p_config->max_frame_us)
    {
        return rec_finish(p_record, S3_BARCODE_REJECTED);
    }
    /* Follow bumps and dips as they come: no stripe is longer than
     * max_run_us, so this long without a turn while dark is a line or a
     * patch, not a barcode (bc_decoder_update rejects the same case). */
    p_record->turn_us =
        sat_add(p_record->turn_us, (uint32_t)step * S3_REC_STEP_US);

    if (p_record->b_is_falling ? (dark < p_record->best)
                               : (dark > p_record->best))
    {
        p_record->best = (uint16_t)dark;
    }
    else if ((p_record->b_is_falling
                  ? dark - p_record->best
                  : p_record->best - dark) >= rec_monitor_band(p_record))
    {
        p_record->best         = (uint16_t)dark;
        p_record->b_is_falling = p_record->b_is_falling ? 0U : 1U;
        p_record->turn_us      = 0U;
    }
    else
    {
        /* Retain the extremum and direction inside the reversal band. */
    }

    if ((p_record->turn_us > p_config->max_run_us) &&
        (!rec_white(p_record, dark)))
    {
        return rec_finish(p_record, S3_BARCODE_REJECTED);
    }

    const unsigned int quiet =
        bc_quiet_update(&p_record->quiet, (uint32_t)step * S3_REC_STEP_US, dark,
                        p_record->paper, rec_white_band(p_record),
                        rec_faint_eligible(p_record));
    p_record->white_us = p_record->quiet.strict_us;

    if (quiet & (unsigned)S3_QUIET_BREAK)
    {
        p_record->b_blocks_retry      = false;
        p_record->b_did_try_early_fit = false;
    }

    if (S3_REC_PROCESSING == (int)p_record->state)
    {
        if (!(quiet & (unsigned)S3_QUIET_LOW))
        {
            p_record->work.b_is_strict_fit_broken = true;
        }

        if (quiet & (unsigned)S3_QUIET_BREAK)
        {
            /* A tentative or early interval may be an internal white gap.
             * Keep those real samples for a later complete interpretation. */
            if ((p_record->work.b_is_provisional) ||
                (p_record->work.b_is_early))
            {
                p_record->work  = (bc_decoder_work_t){0};
                p_record->state = S3_REC_RECORDING;
            }
            else
            {
                return rec_finish(p_record, S3_BARCODE_REJECTED);
            }
        }
    }

    if ((S3_REC_QUIET == (int)p_record->state) &&
        (!(quiet & (unsigned)S3_QUIET_LOW)))
    {
        const bool b_has_ink = (!p_record->b_accepts_faint_quiet) ||
                               ((quiet & (unsigned)S3_QUIET_BREAK) != 0U);

        if (b_has_ink && (!p_record->b_has_early_candidate))
        {
            return rec_finish(p_record, S3_BARCODE_REJECTED);
        }

        if (b_has_ink)
        {
            /* The early fit still lacked its full measured quiet margin.
             * Continuing ink cancels it without discarding this recording. */
            p_record->candidate             = (bc_decoder_stored_result_t){0};
            p_record->work                  = (bc_decoder_work_t){0};
            p_record->b_has_early_candidate = false;
            p_record->required_us           = 0U;
            p_record->quiet_required_us     = 0U;
            p_record->b_accepts_faint_quiet = false;
            p_record->state                 = S3_REC_RECORDING;
        }

        return S3_BARCODE_NONE; /* Wait until the pulse is resolved. */
    }

    if (S3_REC_RECORDING == (int)p_record->state)
    {
        const bool b_is_strict =
            p_record->quiet.strict_us >= p_config->max_run_us;
        const bool b_is_tentative =
            p_record->quiet.tolerant_us >= p_config->max_run_us;
        const uint32_t earliest =
            p_config->quiet_modules * p_config->min_narrow_us;
        const bool b_is_early = (!p_record->b_did_try_early_fit) &&
                                (p_record->quiet.strict_us >= earliest);

        if (!(quiet & (unsigned)S3_QUIET_LOW))
        {
            return S3_BARCODE_NONE;
        }

        if (b_is_strict)
        {
            work_begin(p_record, false, false);
        }
        else if (b_is_tentative && (!p_record->b_blocks_retry))
        {
            work_begin(p_record, true, false);
        }
        else if (b_is_early)
        {
            /* One bounded early attempt per real-ink/quiet interval. Failed
             * partial frames still get the original final max_run_us try. */
            p_record->b_did_try_early_fit = true;
            work_begin(p_record, false, true);
        }
        else
        {
            return S3_BARCODE_NONE;
        }
    }

    if (S3_REC_PROCESSING == (int)p_record->state)
    {
        work_advance(p_record, p_config);

        if ((int)p_record->work.phase != RW_DONE)
        {
            return S3_BARCODE_NONE;
        }

        const bool b_is_usable = (p_record->work.b_has_candidate) &&
                                 ((p_record->work.b_allows_noise) ||
                                  ((!p_record->work.b_is_provisional) &&
                                   (!p_record->work.b_is_strict_fit_broken)));

        const bool b_should_reject =
            (p_record->work.b_is_ambiguous) || (!b_is_usable);

        if (b_should_reject && (((p_record->work.b_is_early) ||
                                 (p_record->work.b_is_provisional)) ||
                                (p_record->work.b_is_strict_fit_broken)))
        {
            p_record->b_blocks_retry = true;
            p_record->state          = S3_REC_RECORDING;
            return S3_BARCODE_NONE;
        }

        if (b_should_reject)
        {
            return rec_finish(p_record, S3_BARCODE_REJECTED);
        }

        p_record->candidate             = p_record->work.candidate;
        p_record->required_us           = p_record->work.required_us;
        p_record->quiet_required_us     = p_record->work.quiet_required_us;
        p_record->b_has_early_candidate = p_record->work.b_is_early;
        p_record->b_accepts_faint_quiet = p_record->work.b_allows_noise;
        p_record->state                 = S3_REC_QUIET;
    }

    const uint32_t qualified = p_record->b_accepts_faint_quiet
                                   ? p_record->quiet.tolerant_us
                                   : p_record->quiet.strict_us;
    /* Require a fresh complete quiet interval as well as corrected edge time.
     * A cancelled job must never borrow white from before later real ink. */
    if (((!(quiet & (unsigned)S3_QUIET_LOW)) ||
         (p_record->frame_us < p_record->required_us)) ||
        (qualified < p_record->quiet_required_us))
    {
        return S3_BARCODE_NONE;
    }

    /* clang-format off */
    p_result->letter        =
        (char)((unsigned int)('A' + (int)p_record->candidate.letter_code) - 1U);
    p_result->b_is_reversed = p_record->candidate.b_is_reversed;
    p_result->timestamp_us  = timestamp_us;
    /* clang-format on */
    return rec_finish(p_record, S3_BARCODE_DECODED);
}

#ifdef BC_DECODER_FIT_TEST
/** @brief Score one fit against two positive reference widths for the host
 * arithmetic fixture. The nine widths remain owned by the caller.
 */
uint32_t bc_decoder_fit_test_score (const uint32_t p_adjusted[9], uint16_t star,
                                    uint32_t narrow, uint32_t wide)
{
    return rec_fit_error(p_adjusted, star, narrow, wide);
}

/** @brief Apply the decoder's configuration validation for the fit fixture.
 */
bool bc_decoder_fit_test_config_valid (const bc_decoder_config_t * p_config)
{
    return config_valid(p_config);
}

/** @brief Classify nine widths using the existing sentinel-fit constraints.
 * The gap and widths are in microseconds. On success, return the pattern
 * and narrow width through the caller's output pointers.
 */
bool bc_decoder_fit_test_character (const uint32_t p_widths[9], uint32_t gap,
                                    const bc_decoder_config_t * p_config,
                                    uint16_t * p_pattern, uint32_t * p_narrow)
{
    return character(p_widths, gap, p_config, p_pattern, p_narrow);
}
#endif

#ifdef BC_DECODER_WORK_TEST
/** @brief Run the synchronous reference against a frozen recording.
 * Write the decoded result and the required duration in microseconds using
 * the reference decoder's output contract.
 */
bool bc_decoder_work_test_candidate (const bc_decoder_record_t * p_record,
                                     const bc_decoder_config_t * p_config,
                                     bool                        b_is_faint,
                                     bc_decoder_result_t *       p_candidate,
                                     uint32_t *                  p_required_us)
{
    return rec_candidate(p_record, p_config, b_is_faint, p_candidate,
                         p_required_us);
}

/** @brief Return whether the frozen recording qualifies for the faint path.
 */
bool bc_decoder_work_test_faint_eligible (const bc_decoder_record_t * p_record)
{
    return rec_faint_eligible(p_record);
}

/** @brief Advance the caller-owned recording within one update's work quota.
 * The fixture prepares the phase and resets the per-update counters first.
 */
void bc_decoder_work_test_advance (bc_decoder_record_t *       p_record,
                                   const bc_decoder_config_t * p_config)
{
    work_advance(p_record, p_config);
}

/** @brief Return whether the fixture's decoder workspace is finished. */
bool bc_decoder_work_test_is_done (const bc_decoder_record_t * p_record)
{
    return RW_DONE == (int)p_record->work.phase;
}
#endif

#ifdef BC_DECODER_BOUNDARY_TEST
/** @brief Apply the decoder's validation to the fixture configuration. */
bool bc_decoder_boundary_test_config_valid (
    const bc_decoder_config_t * p_config)
{
    return config_valid(p_config);
}

/** @brief Initialize the caller-owned workspace for a synthetic recording. */
void bc_decoder_boundary_test_begin (bc_decoder_record_t * p_record,
                                     bool b_is_provisional, bool b_is_early)
{
    work_begin(p_record, b_is_provisional, b_is_early);
}

/** @brief Advance the fixture's recording within the existing work quota. */
void bc_decoder_boundary_test_advance (bc_decoder_record_t *       p_record,
                                       const bc_decoder_config_t * p_config)
{
    work_advance(p_record, p_config);
}
#endif

/** @brief Check the ordering and range of timing and quiet-zone limits before
 * the decoder uses them. A null configuration is invalid.
 */
static bool config_valid (const bc_decoder_config_t * p_config)
{
    /* Form the full run-by-quiet-modules product before checking that it
     * stays below the unsigned clock's half range. */
    return (((((((((p_config != 0) && (p_config->max_gap_us > 0U)) &&
                  (p_config->min_narrow_us > 0U)) &&
                 (p_config->max_run_us >= p_config->min_narrow_us)) &&
                (p_config->max_frame_us > p_config->max_run_us)) &&
               (p_config->max_frame_us < 0x80000000U)) &&
              (p_config->max_gap_us <= p_config->max_run_us)) &&
             (p_config->quiet_modules >= 10U)) &&
            (p_config->quiet_modules <= 50U)) &&
           (((uint64_t)p_config->max_run_us * p_config->quiet_modules) <
            0x80000000ULL);
}

/** @brief Classify a sequence of nine element durations in microseconds. Write
 * the narrow mean and accumulate the pattern in scan order. Reject
 * inconsistent widths, allowing the payload's single extended space
 * only where specified. Outputs can be partial on rejection.
 *
 * Exactly six narrow and three wide elements, normally consistent to 35%.
 * Only the payload may contain one narrow white gap up to 150% of its mean.
 * Width ratio 1.8..3.5 permits rasterisation/jitter around nominal 2..3.
 */
static bool character_widths (const uint32_t * p_runs, uint32_t gap,
                              const bc_decoder_config_t * p_config,
                              uint16_t * p_pattern, uint32_t * p_narrow,
                              bool b_is_payload)
{
    uint32_t     sorted[CODE39_ELEMENTS];
    uint64_t     nsum = 0U;
    uint64_t     wsum = 0U;
    unsigned int element_index;
    unsigned int insert_at;
    unsigned int extended_spaces = 0U;
    *p_pattern                   = 0U;

    for (element_index = 0U; element_index < CODE39_ELEMENTS; element_index++)
    {
        if ((0U == p_runs[element_index]) ||
            (p_runs[element_index] > p_config->max_run_us))
        {
            return false;
        }

        sorted[element_index] = p_runs[element_index];
    }

    for (element_index = 1U; element_index < CODE39_ELEMENTS; element_index++)
    {
        const uint32_t width_us = sorted[element_index];
        insert_at               = element_index;

        while ((insert_at > 0U) && (sorted[insert_at - 1U] > width_us))
        {
            sorted[insert_at] = sorted[insert_at - 1U];
            insert_at--;
        }

        sorted[insert_at] = width_us;
    }

    for (element_index = 0U; element_index < CODE39_NARROW_ELEMENTS;
         element_index++)
    {
        nsum += sorted[element_index];
    }

    for (element_index = CODE39_NARROW_ELEMENTS;
         element_index < CODE39_ELEMENTS; element_index++)
    {
        wsum += sorted[element_index];
    }

    /* Sums can exceed uint32_t, but each mean of uint32_t widths fits it. */
    *p_narrow           = (uint32_t)(nsum / CODE39_NARROW_ELEMENTS);
    const uint32_t wide = (uint32_t)(wsum / CODE39_WIDE_ELEMENTS);

    /* Widen before multiplying by percentages or comparing the gap. */
    if ((((*p_narrow < p_config->min_narrow_us) ||
          ((uint64_t)*p_narrow < (3ULL * gap))) ||
         (((uint64_t)wide * 100U) < ((uint64_t)*p_narrow * 180U))) ||
        (((uint64_t)wide * 100U) > ((uint64_t)*p_narrow * 350U)))
    {
        return false;
    }

    /* Widen for the sum so the midpoint retains its full uint32_t range. */
    const uint32_t split = (uint32_t)(((uint64_t)sorted[5] + sorted[6]) / 2U);

    for (element_index = 0U; element_index < CODE39_ELEMENTS; element_index++)
    {
        const bool     b_is_wide = p_runs[element_index] > split;
        const uint32_t reference = b_is_wide ? wide : *p_narrow;
        const uint64_t scaled    = (uint64_t)p_runs[element_index] * 100U;

        if (scaled < ((uint64_t)reference * S3_BARCODE_WIDTH_MIN_PERCENT))
        {
            return false;
        }

        if (scaled > ((uint64_t)reference * S3_BARCODE_WIDTH_MAX_PERCENT))
        {
            if (((((!b_is_payload) || b_is_wide) || (!(element_index % 2U))) ||
                 (extended_spaces != 0U)) ||
                (scaled >
                 ((uint64_t)reference * S3_BARCODE_PAYLOAD_SPACE_MAX_PERCENT)))
            {
                return false;
            }

            extended_spaces++;
        }
        /* The nine-element pattern fits in 16 bits. Shift an unsigned word. */
        *p_pattern =
            (uint16_t)(((uint32_t)*p_pattern << 1U) | (b_is_wide ? 1U : 0U));
    }

    return true;
}

/** @brief Classify a sentinel's nine widths and return its pattern and narrow
 * mean using the strict width limits.
 *
 * Sentinel fitting always uses the original strict width checks.
 */
static bool character (const uint32_t * p_runs, uint32_t gap,
                       const bc_decoder_config_t * p_config,
                       uint16_t * p_pattern, uint32_t * p_narrow)
{
    return character_widths(p_runs, gap, p_config, p_pattern, p_narrow, false);
}

/** @brief Start a new streaming run at the supplied wrapping microsecond
 * timestamp and input colour.
 */
static void seed_decoder (bc_decoder_t * p_decoder, uint32_t now,
                          bool b_is_black)
{
    bc_decoder_reset(p_decoder);
    p_decoder->b_has_sample = true;
    p_decoder->b_is_black   = b_is_black;
    p_decoder->last_us      = now;
    p_decoder->run_start_us = now;
}

/** @brief Quantize a positive microsecond interval for storage. Write its step
 * count and carry the residual on success. A zero interval or
 * unrepresentable count leaves both outputs untouched.
 *
 * Elapsed time as a 6-bit step, carrying the rounding remainder forward so
 * summed steps track real time. False when the gap does not fit (stall).
 */
static bool rec_step (bc_decoder_record_t * p_record, uint32_t gap_us,
                      uint16_t * p_step)
{
    const uint32_t total = gap_us + p_record->residual_us;
    const uint32_t units = total / S3_REC_STEP_US;

    if ((0U == gap_us) || (units > S3_REC_STEP_MAX))
    {
        return false;
    }

    /* The remainder is below one step and the accepted quotient fits
     * in six bits. Both casts retain the complete values. */
    p_record->residual_us = (uint16_t)(total - (units * S3_REC_STEP_US));
    *p_step               = (uint16_t)units;
    return true;
}

/** @brief Check whether the retained paper level and prominence allow the
 * faint extraction path.
 *
 * Calibration can clamp a quiet paper background near zero while narrow
 * bars still rise only 10..25 units. Keep their small turns and quiet-zone
 * interruptions visible. The normal batch extractor retains its learned
 * prominence; this only enables a bounded second extraction if bars are
 * lost.
 */
static bool rec_faint_eligible (const bc_decoder_record_t * p_record)
{
    return (((p_record != 0) && (p_record->paper <= S3_REC_FAINT_BAND)) &&
            (p_record->prominence != 0U)) &&
           (p_record->prominence <= S3_REC_FAINT_MAX_NOISE);
}

/** @brief Select the faint reversal band for eligible records or use the
 * learned prominence for ordinary monitoring.
 */
static unsigned int rec_monitor_band (const bc_decoder_record_t * p_record)
{
    return rec_faint_eligible(p_record) ? S3_REC_FAINT_BAND
                                        : p_record->prominence;
}

/** @brief Return the default paper band narrowed by a smaller nonzero
 * monitoring band, in normalized darkness units.
 */
static unsigned int rec_white_band (const bc_decoder_record_t * p_record)
{
    unsigned int       band    = S3_REC_WHITE_BAND;
    const unsigned int monitor = rec_monitor_band(p_record);

    if ((monitor != 0U) && (monitor < band))
    {
        band = monitor;
    }

    return band;
}

/** @brief Compare normalized darkness with the stored paper reference plus its
 * current white band.
 */
static bool rec_white (const bc_decoder_record_t * p_record, unsigned int dark)
{
    return dark <= ((unsigned int)p_record->paper + rec_white_band(p_record));
}

/** @brief Add two nonnegative duration components, saturating at 0x7FFFFFFF
 * microseconds. The second component must fit that limit.
 */
static uint32_t sat_add (uint32_t elapsed_us, uint32_t delta_us)
{
    return (elapsed_us > (0x7FFFFFFFU - delta_us)) ? 0x7FFFFFFFU
                                                   : elapsed_us + delta_us;
}

/** @brief Discard the earliest pre-trigger sample and adjust the measured
 * paper interval before the retained history.
 *
 * Keep real, timestamped readings while waiting. The first ring entry has
 * no preceding step: dropping it also drops the next entry's incoming
 * interval.
 */
static void rec_pre_drop (bc_decoder_record_t * p_record)
{
    const bool b_was_white =
        (p_record->pre[p_record->pre_head] & 0x3FFU) <= S3_REC_FAINT_BAND;
    p_record->pre_head = (uint16_t)((p_record->pre_head + 1U) % S3_REC_PRE_MAX);
    p_record->pre_count--;
    const uint32_t elapsed =
        ((uint32_t)p_record->pre[p_record->pre_head] >> 10U) * S3_REC_STEP_US;
    /* While waiting, leading_us measures verified low-band white BEFORE
     * the retained history. A faint mark that falls out of the ring must
     * still break that margin; the stronger recording trigger cannot prove
     * that everything before it was white. The absolute band is conservative
     * for all eligible paper levels (0..5), even if the reference changes. */
    /* clang-format off */
    p_record->leading_us               =
        b_was_white && ((p_record->pre[p_record->pre_head] &
                                            0x3FFU) <= S3_REC_FAINT_BAND)
                               ? sat_add(p_record->leading_us, elapsed)
                               : 0U;
    p_record->pre_us                  -= elapsed;
    p_record->pre[p_record->pre_head] &= 0x3FFU;
    /* clang-format on */
}

/** @brief Append normalized darkness and its quantized step to the pre-trigger
 * ring. Evict old samples to satisfy both its time and capacity
 * bounds.
 */
static void rec_pre_push (bc_decoder_record_t * p_record, unsigned int dark,
                          uint16_t step)
{

    if (S3_REC_PRE_MAX == p_record->pre_count)
    {
        rec_pre_drop(p_record);
    }

    if (0U == p_record->pre_count)
    {
        step = 0U;
    }

    unsigned int value_at =
        ((unsigned int)p_record->pre_head + p_record->pre_count) %
        S3_REC_PRE_MAX;
    /* The caller supplies ten-bit darkness and a six-bit step. */
    p_record->pre[value_at] = (uint16_t)(dark | ((unsigned int)step << 10));
    p_record->pre_count++;
    p_record->pre_us += (uint32_t)step * S3_REC_STEP_US;

    while ((p_record->pre_count > RECORD_MIN_LEAD_SAMPLES) &&
           (p_record->pre_us > S3_REC_PRE_US))
    {
        rec_pre_drop(p_record);
    }
}

/** @brief Estimate the prominence threshold from the retained leading paper
 * interval, falling back to the recorder's noise estimate when the
 * prefix is unsuitable.
 *
 * The waiting EWMA can include faint ink below the stronger recording
 * trigger. If the retained history starts with an actual blank-paper
 * interval, measure its noise alone. Stop at the first ink sample; never
 * use inter-bar valleys to certify a noisy background as quiet. Otherwise
 * retain the EWMA estimate.
 */
static unsigned int rec_paper_prominence (const bc_decoder_record_t * p_record,
                                          const bc_decoder_config_t * p_config)
{
    unsigned int prominence = (p_record->noise_estimate + 3U) / 4U;

    if ((p_record->paper <= S3_REC_FAINT_BAND) && (p_record->pre_count > 1U))
    {
        const unsigned int floor =
            (unsigned int)p_record->paper + S3_REC_FAINT_BAND;
        unsigned int previous    = p_record->pre[p_record->pre_head] & 0x3FFU;
        unsigned int differences = 0U;
        uint32_t     elapsed     = 0U;
        uint32_t     sum         = 0U;

        for (unsigned int history_index = 1U;
             (previous <= floor) && (history_index < p_record->pre_count);
             history_index++)
        {
            const uint16_t sample_word =
                p_record
                    ->pre[((unsigned int)p_record->pre_head + history_index) %
                          S3_REC_PRE_MAX];
            const unsigned int dark = sample_word & 0x3FFU;

            if (dark > floor)
            {
                break;
            }

            elapsed += ((uint32_t)sample_word >> 10U) * S3_REC_STEP_US;
            sum     += (dark > previous) ? dark - previous : previous - dark;
            differences++;
            previous = dark;
        }

        if ((differences != 0U) &&
            (elapsed >= (p_config->quiet_modules * p_config->min_narrow_us)))
        {
            prominence = ((4U * sum) + differences - 1U) / differences;
        }
    }

    if (prominence < S3_REC_MIN_PROMINENCE)
    {
        prominence = S3_REC_MIN_PROMINENCE;
    }

    if (prominence > S3_REC_PROMINENCE)
    {
        prominence = S3_REC_PROMINENCE;
    }

    return prominence;
}

/* Edge time where the readings cross mid between samples i and i+1. */
/** @brief Return a sample's clamped position from 0 through 1024 between the
 * segment levels. The inputs use the 10-bit darkness scale.
 *
 * A result of 1024 means the reading is still at the starting level. Zero
 * means the reading has reached the ending level.
 */
static uint32_t rec_frac (unsigned int darkness, unsigned int value_lo,
                          unsigned int value_hi)
{
    const int32_t num = (int32_t)darkness - (int32_t)value_hi;
    const int32_t den = (int32_t)value_lo - (int32_t)value_hi;
    /* 10-bit samples bound num*1024 to +/-1,047,552. Keep this exact
     * operation 32-bit: RP2040 otherwise calls a slow 64-bit divide twice
     * per sample while processing the completed recording. */
    const int32_t fraction = (0 == den) ? 0 : (num * 1024) / den;
    return (fraction < 0) ? 0U
                          : ((fraction > 1024) ? 1024U : (uint32_t)fraction);
}

/** @brief Find alternating bounds for faint ink regions in caller-provided
 * extrema storage. Return zero when a complete unambiguous region
 * sequence cannot be extracted.
 *
 * Group low-contrast ink regions, retaining one strongest peak per region.
 * A bar must return below the noise band for more than 30 ms before another
 * can start. Internal plateau wiggles therefore cannot replace a faint bar.
 * Require exactly 15 complete regions: never prune this path to a desired
 * count, infer a missing bar, or use an expected letter to choose peaks.
 */
static unsigned int rec_faint_extrema (const bc_decoder_record_t * p_recorder,
                                       uint16_t p_ext[S3_REC_EXT_MAX])
{
    const unsigned int floor =
        (unsigned int)p_recorder->paper + S3_REC_FAINT_BAND;
    unsigned int cursor        = 0U;
    unsigned int extrema_count = 1U;
    p_ext[0]                   = 0U;

    while (cursor < p_recorder->count)
    {
        uint32_t low_us = 0U;

        while ((cursor < p_recorder->count) &&
               (REC_DARK(p_recorder, cursor) <= floor))
        {
            cursor++;
        }

        if (cursor == p_recorder->count)
        {
            break;
        }

        if ((extrema_count + 2U) > (S3_BARCODE_RUNS + 2U))
        {
            return 0U;
        }

        unsigned int peak = cursor++;
        unsigned int dip  = peak;

        while (cursor < p_recorder->count)
        {
            if (REC_DARK(p_recorder, cursor) > floor)
            {
                if (REC_DARK(p_recorder, cursor) > REC_DARK(p_recorder, peak))
                {
                    peak = cursor;
                }

                low_us = 0U;
            }
            else
            {
                if ((0U == low_us) ||
                    (REC_DARK(p_recorder, cursor) < REC_DARK(p_recorder, dip)))
                {
                    dip = cursor;
                }

                low_us += REC_STEP_US(p_recorder, cursor);

                if (low_us > S3_REC_FAINT_DIP_US)
                {
                    break;
                }
            }

            cursor++;
        }

        if (low_us <= S3_REC_FAINT_DIP_US)
        {
            /* Reject the final bar because it never completed. */
            return 0U;
        }
        /* Include the entire valley, including its first low readings. */
        while ((cursor < p_recorder->count) &&
               (REC_DARK(p_recorder, cursor) <= floor))
        {
            if (REC_DARK(p_recorder, cursor) < REC_DARK(p_recorder, dip))
            {
                dip = cursor;
            }

            cursor++;
        }

        p_ext[extrema_count++] = (uint16_t)peak;
        p_ext[extrema_count++] = (uint16_t)dip;
    }

    return (S3_BARCODE_RUNS + 2U) == extrema_count ? extrema_count : 0U;
}

/** @brief Reconstruct run widths and edge times in microseconds from the
 * primary or faint extrema. Return the number of widths recovered.
 * Only use the outputs when the returned count meets the caller's
 * requirement.
 * @details Select extrema using the normal or faint fit. Retain the longest
 * observed sample interval for quiet-zone checks. The normal fit folds
 * shallow or brief pairs to fit the Code 39 shape. Edge interpolation then
 * measures widths across adjacent extrema using the recorded sample clock.
 * A zero result can leave partial scratch outputs and is a failed fit.
 */
static unsigned int rec_extract_runs (const bc_decoder_record_t * p_record,
                                      uint32_t   p_runs[S3_BARCODE_RUNS],
                                      uint32_t * p_first_edge_us,
                                      uint32_t * p_last_edge_us,
                                      uint32_t * p_end_us,
                                      uint32_t * p_max_step_us, bool b_is_faint)
{
    /* Alternating dip, bump, dip, ... indices; 15 bars need 31 extremes. */
    uint16_t     ext[S3_REC_EXT_MAX];
    uint32_t     edge_prev     = 0U;
    uint32_t     elapsed_us    = 0U;
    unsigned int extrema_count = 1U;
    unsigned int best          = 0U;
    unsigned int sample_index;
    unsigned int segment_index = 0U;
    unsigned int edges         = 0U;
    bool         b_is_rising   = true;

    if ((((((((0 == p_record) || (0 == p_runs)) || (0 == p_first_edge_us)) ||
            (0 == p_last_edge_us)) ||
           (0 == p_end_us)) ||
          (0 == p_max_step_us)) ||
         (p_record->count < 3U)) ||
        (p_record->count > S3_REC_MAX))
    {
        return 0U;
    }

    *p_max_step_us = 0U;
    ext[0]         = 0U;

    if (b_is_faint)
    {
        extrema_count = rec_faint_extrema(p_record, ext);

        if (0U == extrema_count)
        {
            return 0U;
        }

        for (sample_index = 1U; sample_index < p_record->count; sample_index++)
        {
            if (REC_STEP_US(p_record, sample_index) > *p_max_step_us)
            {
                *p_max_step_us = REC_STEP_US(p_record, sample_index);
            }
        }
    }
    else
    {
        for (sample_index = 1U; sample_index < p_record->count; sample_index++)
        {
            const unsigned int sample_dark = REC_DARK(p_record, sample_index);
            unsigned int       best_dark   = REC_DARK(p_record, best);

            if (REC_STEP_US(p_record, sample_index) > *p_max_step_us)
            {
                *p_max_step_us = REC_STEP_US(p_record, sample_index);
            }

            const bool b_extends_extremum = b_is_rising
                                                ? (sample_dark > best_dark)
                                                : (sample_dark < best_dark);
            const bool b_has_reversal =
                !b_extends_extremum &&
                ((b_is_rising ? best_dark - sample_dark
                              : sample_dark - best_dark) >=
                 (p_record->prominence ? p_record->prominence
                                       : S3_REC_PROMINENCE));

            if (b_extends_extremum)
            {
                best = sample_index;
            }
            else if (b_has_reversal && (extrema_count >= (S3_REC_EXT_MAX - 1U)))
            {
                /* Reject an extrema sequence that would fill the buffer. */
                return 0U;
            }
            else if (b_has_reversal)
            {
                /* The sample count check keeps best below S3_REC_MAX. */
                ext[extrema_count++] = (uint16_t)best;
                best                 = sample_index;
                b_is_rising          = !b_is_rising;
            }
            else
            {
                /* Keep the extremum until the reversal reaches the
                 * prominence limit. */
            }
        }

        if (!b_is_rising)
        {
            /* Keep the final dip after the last peak. */
            ext[extrema_count++] = (uint16_t)best;
        }

        if (extrema_count < 3U)
        {
            /* The sampled region contains no complete bar. */
            return 0U;
        }
        /* A sag or scratch inside a wide bar (or a smudge inside a wide gap)
         * adds an extra dip/bump pair. While there are more than 31 extremes,
         * drop the neighbouring interior pair that is "smallest" = height times
         * time apart: a wiggle is shallow or brief, a real stripe is deep and
         * about one narrow module long. The strict Code 39 checks still have to
         * pass afterwards; they reduce false accepts but are not a checksum. */
        while (extrema_count > (S3_BARCODE_RUNS + RECORD_EDGE_ENDPOINTS))
        {
            unsigned int cursor;
            unsigned int span_index;
            unsigned int cut   = 1U;
            uint64_t     small = UINT64_MAX;

            for (cursor = 1U; (cursor + RECORD_NOISE_PAIR) < extrema_count;
                 cursor++)
            {
                unsigned int prev_dark = REC_DARK(p_record, ext[cursor]);
                unsigned int next_dark = REC_DARK(p_record, ext[cursor + 1U]);
                uint64_t     span      = 0U;

                for (span_index = (unsigned int)ext[cursor] + 1U;
                     span_index <= (unsigned int)ext[cursor + 1U]; span_index++)
                {
                    span += REC_STEP_US(p_record, span_index);
                }

                const uint64_t size = (uint64_t)((prev_dark > next_dark)
                                                     ? prev_dark - next_dark
                                                     : next_dark - prev_dark) *
                                      span;

                if (size < small)
                {
                    small = size;
                    cut   = cursor;
                }
            }

            for (cursor = cut; (cursor + RECORD_NOISE_PAIR) < extrema_count;
                 cursor++)
            {
                ext[cursor] = ext[cursor + RECORD_NOISE_PAIR];
            }

            extrema_count -= RECORD_NOISE_PAIR;
        }
    }
    /* Edges of the retained bumps: first rising / last falling crossing of
     * the local halfway level. Once an interior dip has been merged into a
     * bar above, its early downward crossing must not end that bar while a
     * later rebound still crosses the same level. Neither extraction path
     * uses the expected letter or marker widths to choose edges.
     * Keep the time-weighted estimate only as a fallback if no crossing is
     * available. All stripe, marker and actual quiet checks still follow. */
    {
        uint32_t t_lo  = 0U;
        uint32_t at_lo = 0U;

        for (sample_index = 1U; sample_index < p_record->count; sample_index++)
        {
            const uint32_t step = REC_STEP_US(p_record, sample_index);
            elapsed_us         += step;

            if ((segment_index + 1U) < extrema_count)
            {
                const unsigned int value_lo =
                    REC_DARK(p_record, ext[segment_index]);
                const unsigned int value_hi =
                    REC_DARK(p_record, ext[segment_index + 1U]);
                /* step<=31500, each fraction<=1024: product<=64,512,000.
                 * Across 2048 readings the elapsed sum fits uint32_t too. */
                at_lo +=
                    (step * (rec_frac(REC_DARK(p_record, sample_index - 1U),
                                      value_lo, value_hi) +
                             rec_frac(REC_DARK(p_record, sample_index),
                                      value_lo, value_hi))) /
                    2048U;

                if (sample_index == (unsigned int)ext[segment_index + 1U])
                {
                    const uint32_t     guess = t_lo + (uint32_t)at_lo;
                    const unsigned int mid   = (value_lo + value_hi) / 2U;
                    const bool         b_does_span_rise = value_hi > value_lo;
                    /* Near the noise floor, measure the half-height core
                     * connected to the maximum. An earlier weak shoulder
                     * must not consume the neighbouring white gap. Strong
                     * bars retain the outer crossings used for wide dips. */
                    const bool b_should_use_core_crossing =
                        b_is_faint &&
                        (((b_does_span_rise ? value_hi : value_lo) -
                          (unsigned int)p_record->paper) <= S3_REC_RISE);
                    uint32_t     edge          = guess;
                    uint32_t     value_tj      = t_lo;
                    bool         b_has_crossed = false;
                    unsigned int cursor;

                    for (cursor = (unsigned int)ext[segment_index];
                         cursor < sample_index; cursor++)
                    {
                        unsigned int prev_dark = REC_DARK(p_record, cursor);
                        unsigned int next_dark =
                            REC_DARK(p_record, cursor + 1U);

                        const uint32_t value_sj =
                            REC_STEP_US(p_record, cursor + 1U);
                        const bool b_is_crossing =
                            b_does_span_rise
                                ? (b_should_use_core_crossing
                                       ? ((prev_dark <= mid) &&
                                          (next_dark > mid))
                                       : ((prev_dark < mid) &&
                                          (next_dark >= mid)))
                                : ((prev_dark > mid) && (next_dark <= mid));

                        if (b_is_crossing)
                        {
                            const uint32_t crossing_us =
                                value_tj +
                                ((value_sj *
                                  (uint32_t)(b_does_span_rise
                                                 ? mid - prev_dark
                                                 : prev_dark - mid)) /
                                 (uint32_t)(b_does_span_rise
                                                ? next_dark - prev_dark
                                                : prev_dark - next_dark));

                            const bool b_keep_crossing =
                                (!b_has_crossed) || (b_should_use_core_crossing
                                                         ? b_does_span_rise
                                                         : !b_does_span_rise);
                            edge = b_keep_crossing ? crossing_us : edge;

                            b_has_crossed = true;
                        }

                        value_tj += value_sj;
                    }

                    if (0U == edges)
                    {
                        *p_first_edge_us = edge;
                    }
                    else
                    {
                        p_runs[edges - 1U] = edge - edge_prev;
                    }

                    edge_prev = edge;
                    edges++;
                    segment_index++;
                    t_lo  = elapsed_us;
                    at_lo = 0U;
                }
            }
        }
    }
    *p_last_edge_us = edge_prev;
    *p_end_us       = elapsed_us;
    return (edges >= 2U) ? edges - 1U : 0U;
}

/** @brief Compute the mean of the six smallest of nine run durations in
 * microseconds.
 *
 * Average of the six smallest of nine runs (as character() computes
 * narrow).
 */
static uint32_t rec_narrow (const uint32_t * p_runs)
{
    uint32_t     sorted[CODE39_ELEMENTS];
    uint64_t     sum = 0U;
    unsigned int element_index;
    unsigned int insert_at;

    for (element_index = 0U; element_index < CODE39_ELEMENTS; element_index++)
    {
        const uint32_t run_us = p_runs[element_index];
        insert_at             = element_index;

        while ((insert_at > 0U) && (sorted[insert_at - 1U] > run_us))
        {
            sorted[insert_at] = sorted[insert_at - 1U];
            insert_at--;
        }

        sorted[insert_at] = run_us;
    }

    for (element_index = 0U; element_index < CODE39_NARROW_ELEMENTS;
         element_index++)
    {
        sum += sorted[element_index];
    }

    return (uint32_t)(sum / CODE39_NARROW_ELEMENTS);
}

/** @brief Score a fitted sentinel using the worst relative deviation within
 * the narrow and wide groups.
 *
 * All narrow elements share one reference, and all wide elements another.
 * Find the greatest deviation in each group before comparing their ratios.
 * max(floor(x)) == floor(max(x)), so this gives exactly the former score
 * and tie behaviour with one 64-bit division per fit instead of nine.
 * RP2040 performs that division in software in the synchronous sampling
 * task.
 */
static uint32_t rec_fit_error (const uint32_t p_adjusted[CODE39_ELEMENTS],
                               uint16_t star, uint32_t narrow, uint32_t wide)
{
    uint32_t     narrow_delta = 0U;
    uint32_t     wide_delta   = 0U;
    unsigned int element_index;

    for (element_index = 0U; element_index < CODE39_ELEMENTS; element_index++)
    {
        const bool     b_is_wide = (star & (1U << (8U - element_index))) != 0U;
        const uint32_t reference = b_is_wide ? wide : narrow;
        const uint32_t delta     = (p_adjusted[element_index] > reference)
                                       ? p_adjusted[element_index] - reference
                                       : reference - p_adjusted[element_index];

        if (b_is_wide)
        {
            if (delta > wide_delta)
            {
                wide_delta = delta;
            }
        }
        else if (delta > narrow_delta)
        {
            narrow_delta = delta;
        }
        else
        {
            /* The current narrow-run deviation stays within the recorded
             * maximum. */
        }
    }
    /* character() has established positive references and deviations no
     * greater than 35%, so the returned score is at most 350000. Cross
     * products fit uint64_t even for the full uint32_t input range. */
    if (((uint64_t)wide_delta * narrow) > ((uint64_t)narrow_delta * wide))
    {
        return (uint32_t)((uint64_t)wide_delta * 1000000U / wide);
    }

    return (uint32_t)((uint64_t)narrow_delta * 1000000U / narrow);
}

/** @brief Try a common bias for both sentinels in either orientation. A
 * successful fit corrects all run widths in place and returns the
 * fitted bias. The optional flag reports whether a sentinel fit was found.
 *
 * Fit only the two known sentinels, never the payload. Both star
 * orientations have three narrow bars and three narrow spaces: their narrow
 * mean is invariant under a shared bar expansion / space contraction.
 * Search a fixed 129-point grid within +/- 3/4 of the smaller narrow mean.
 * Minimise the worst fractional sentinel error, retaining the original
 * 35%/ratio/sample checks. A single bias must explain BOTH stars; ambiguous
 * orientation is refused.
 */
static bool rec_correct_shared_runs (uint32_t p_runs[S3_BARCODE_RUNS],
                                     uint32_t gap,
                                     const bc_decoder_config_t * p_config,
                                     int32_t *                   p_bias_us,
                                     bool * pb_did_match_sentinels)
{
    unsigned int orientation;
    unsigned int found = 0U;
    unsigned int run_index;
    unsigned int marker_index;
    int32_t      chosen = 0;

    if (pb_did_match_sentinels != 0)
    {
        *pb_did_match_sentinels = false;
    }

    if ((((0 == p_runs) || (0 == p_bias_us)) || (!config_valid(p_config))) ||
        (gap > p_config->max_gap_us))
    {
        return false;
    }

    *p_bias_us = 0;

    for (orientation = 0U; orientation < CODE39_ORIENTATIONS; orientation++)
    {
        const uint16_t star       = orientation ? 0x052U : 0x094U;
        uint32_t       minimum    = UINT32_MAX;
        uint32_t       best_score = UINT32_MAX;
        int32_t        best_bias  = 0;
        int            tick;

        for (marker_index = 0U; marker_index < CODE39_SENTINELS; marker_index++)
        {
            uint64_t sum = 0U;

            for (run_index = 0U; run_index < CODE39_ELEMENTS; run_index++)
            {
                if (!(star & (1U << (8U - run_index))))
                {
                    sum += p_runs[(marker_index * 20U) + run_index];
                }
            }

            if ((sum / CODE39_NARROW_ELEMENTS) < minimum)
            {
                minimum = (uint32_t)(sum / CODE39_NARROW_ELEMENTS);
            }
        }

        if ((minimum < p_config->min_narrow_us) ||
            (minimum > p_config->max_run_us))
        {
            continue;
        }

        /* The validated run limit is below 2^31 / quiet_modules, with at
         * least ten modules. Multiplying the mean by three fits uint32_t,
         * and the bound fits int32_t. */
        const int32_t bound = (int32_t)(minimum * 3U / 4U);

        for (tick = -BIAS_GRID_RADIUS; tick <= BIAS_GRID_RADIUS; tick++)
        {
            /* Widen before multiplying by the signed tick. Dividing by
             * the radius brings the result back within +/-bound. */
            int32_t  bias = (int32_t)((int64_t)bound * tick / BIAS_GRID_RADIUS);
            uint32_t score      = 0U;
            bool     b_is_valid = true;

            for (marker_index = 0U;
                 (marker_index < CODE39_SENTINELS) && b_is_valid;
                 marker_index++)
            {
                uint32_t adjusted[CODE39_ELEMENTS];
                uint32_t narrow;
                uint32_t wide = 0U;
                uint16_t pattern;

                for (run_index = 0U; run_index < CODE39_ELEMENTS; run_index++)
                {
                    int64_t adjusted_us =
                        (int64_t)p_runs[(marker_index * 20U) + run_index] +
                        ((run_index % 2U) ? -bias : bias);

                    if ((adjusted_us <= 0) ||
                        (adjusted_us > (int64_t)p_config->max_run_us))
                    {
                        b_is_valid = false;
                        break;
                    }

                    adjusted[run_index] = (uint32_t)adjusted_us;

                    if (star & (1U << (8U - run_index)))
                    {
                        wide += adjusted[run_index];
                    }
                }

                if (((!b_is_valid) || (!character(adjusted, gap, p_config,
                                                  &pattern, &narrow))) ||
                    (pattern != star))
                {
                    b_is_valid = false;
                    break;
                }

                wide /= CODE39_WIDE_ELEMENTS;
                const uint32_t error =
                    rec_fit_error(adjusted, star, narrow, wide);

                if (error > score)
                {
                    score = error;
                }
            }

            if (b_is_valid && ((score < best_score) ||
                               ((score == best_score) &&
                                ((bias < 0 ? -bias : bias) <
                                 (best_bias < 0 ? -best_bias : best_bias)))))
            {
                best_score = score;
                best_bias  = bias;
            }
        }

        if (best_score != UINT32_MAX)
        {
            found++;
            chosen = best_bias;
        }
    }

    if (pb_did_match_sentinels != 0)
    {
        *pb_did_match_sentinels = (found != 0U);
    }

    if (found != 1U)
    {
        return false;
    }
    /* Validate before modifying, including the payload and inter-character
     * spaces; their patterns play no part in choosing the correction. */
    for (run_index = 0U; run_index < S3_BARCODE_RUNS; run_index++)
    {
        int64_t adjusted_us =
            (int64_t)p_runs[run_index] + ((run_index % 2U) ? -chosen : chosen);

        if ((adjusted_us <= 0) || (adjusted_us > (int64_t)p_config->max_run_us))
        {
            return false;
        }
    }

    for (run_index = 0U; run_index < S3_BARCODE_RUNS; run_index++)
    {
        p_runs[run_index] = (uint32_t)((int64_t)p_runs[run_index] +
                                       ((run_index % 2U) ? -chosen : chosen));
    }

    *p_bias_us = chosen;
    return true;
}

/** @brief Fit a signed bias in microseconds to one known sentinel. Reject
 * candidates that fail width, sampling or template checks.
 *
 * Single-star fit, using only known sentinel widths. Even-us candidates
 * make the two outer edge shifts exact; no quiet-zone rounding can add
 * margin.
 */
static bool rec_fit_star (const uint32_t * p_runs, uint16_t star, uint32_t gap,
                          const bc_decoder_config_t * p_config,
                          int32_t * p_bias, uint32_t * p_narrow)
{
    uint64_t     sum        = 0U;
    uint32_t     best_score = UINT32_MAX;
    int          tick;
    unsigned int element_index;
    *p_bias = 0;

    for (element_index = 0U; element_index < CODE39_ELEMENTS; element_index++)
    {
        if (!(star & (1U << (8U - element_index))))
        {
            sum += p_runs[element_index];
        }
    }

    *p_narrow = (uint32_t)(sum / CODE39_NARROW_ELEMENTS);

    if ((*p_narrow < p_config->min_narrow_us) ||
        (*p_narrow > p_config->max_run_us))
    {
        return false;
    }

    /* The checked mean is at most max_run_us, below 2^31 / 10 for the
     * validated configuration. Three times that mean fits uint32_t. */
    const int32_t bound = (int32_t)(*p_narrow * 3U / 4U);

    for (tick = -BIAS_GRID_RADIUS; tick <= BIAS_GRID_RADIUS; tick++)
    {
        /* The widened product is divided before narrowing. The radius
         * bounds the quotient and the final even step to +/-bound. */
        const int32_t trial = (int32_t)((int64_t)bound * tick / 128) * 2;
        uint32_t      adjusted[CODE39_ELEMENTS];
        uint32_t      narrow_us;
        uint32_t      wide  = 0U;
        uint32_t      score = 0U;
        uint16_t      pattern;
        bool          b_is_valid = true;

        for (element_index = 0U; element_index < CODE39_ELEMENTS;
             element_index++)
        {
            /* Widen the run before adding a signed trial. The checked
             * positive result fits the destination because max_run_us
             * is a uint32_t limit. */
            const int64_t adjusted_us = (int64_t)p_runs[element_index] +
                                        ((element_index % 2U) ? -trial : trial);

            if ((adjusted_us <= 0) ||
                (adjusted_us > (int64_t)p_config->max_run_us))
            {
                b_is_valid = false;
                break;
            }

            adjusted[element_index] = (uint32_t)adjusted_us;

            if (star & (1U << (8U - element_index)))
            {
                wide += adjusted[element_index];
            }
        }

        if (((!b_is_valid) ||
             (!character(adjusted, gap, p_config, &pattern, &narrow_us))) ||
            (pattern != star))
        {
            continue;
        }

        wide /= CODE39_WIDE_ELEMENTS;
        score = rec_fit_error(adjusted, star, narrow_us, wide);

        if ((score < best_score) ||
            ((score == best_score) && ((trial < 0 ? -trial : trial) <
                                       (*p_bias < 0 ? -*p_bias : *p_bias))))
        {
            best_score = score;
            *p_bias    = trial;
        }
    }

    return best_score != UINT32_MAX;
}

#ifdef S3_BARCODE_SYNC_REFERENCE
/* Validate each independently measured set of edges by the same rules. A
 * count of 29 alone does not prove that the right 15 bumps were selected.
 * Neither extraction nor sentinel fitting uses an expected payload letter. */

/** @brief Evaluate one extraction path in the synchronous reference build.
 * Derive a candidate and its required quiet interval from measured
 * widths and both sentinels.
 */
static bool rec_candidate (const bc_decoder_record_t * p_record,
                           const bc_decoder_config_t * p_config,
                           bool b_is_faint, bc_decoder_result_t * p_candidate,
                           uint32_t * p_required_us)
{
    uint32_t     runs[S3_BARCODE_RUNS];
    uint32_t     first = 0U;
    uint32_t     last  = 0U;
    uint32_t     end   = 0U;
    uint32_t     gap   = 0U;
    uint32_t     leading;
    int32_t      bias[2] = {0, 0};
    int64_t      half;
    int64_t      need;
    bool         b_did_match_shared_fit = false;
    unsigned int run_count =
        rec_extract_runs(p_record, runs, &first, &last, &end, &gap, b_is_faint);
    if (run_count != S3_BARCODE_RUNS)
    {
        return false;
    }
    leading = sat_add(p_record->leading_us, first);
    /* Trailing white is provisional here; the live reader waits for the
     * real margin below before it releases this candidate. */
    if (!bc_decoder_decode_runs(runs, run_count, leading, 0x7FFFFFFFU, gap,
                                p_config, p_candidate))
    {
        if (rec_correct_shared_runs(runs, gap, p_config, &bias[0],
                                    &b_did_match_shared_fit))
        {
            bias[1] = bias[0];
        }
        else if (b_did_match_shared_fit ||
                 !bc_decoder_record_fix_varying(runs, gap, p_config, bias))
        {
            return false;
        }
        else
        {
            /* The varying-width fit supplied both bias values. */
        }
        /* Within an extraction, a chosen model is final. Never try another
         * correction merely because its payload or leading margin fails. */
        half =
            (bias[0] > 0) ? ((int64_t)bias[0] + 1) / 2 : (int64_t)bias[0] / 2;
        if ((int64_t)leading < half)
        {
            return false;
        }
        leading = (uint32_t)((int64_t)leading - half);
        if (!bc_decoder_decode_runs(runs, run_count, leading, 0x7FFFFFFFU, gap,
                                    p_config, p_candidate))
        {
            return false;
        }
    }
    /* Expanding a bar moves its outer edge into the quiet zone too.
     * Round toward requiring more white, never inventing extra margin. */
    /* clang-format off */
    half           =
        (bias[1] > 0) ? ((int64_t)bias[1] + 1) / 2 : (int64_t)bias[1] / 2;
    need           = (int64_t)last + half +
           (int64_t)p_config->quiet_modules * (int64_t)(rec_narrow(&runs[20]));
    *p_required_us = (need > 0x7FFFFFFF) ? 0x7FFFFFFFU : (uint32_t)need;
    /* clang-format on */
    return true;
}

#endif
/** @brief Advance the frozen-record cursor by one sample and update elapsed
 * time and maximum observed sample interval while inside the frozen
 * prefix.
 */
static void work_next (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_workspace = &p_recorder->work;
    p_workspace->cursor++;

    if (p_workspace->cursor < p_workspace->frozen_count)
    {
        const uint32_t step    = REC_STEP_US(p_recorder, p_workspace->cursor);
        p_workspace->total_us += step;

        if (step > p_workspace->max_step_us)
        {
            p_workspace->max_step_us = step;
        }
    }
}

/** @brief Initialize the primary or faint path for the existing frozen record.
 * Reset search and sentinel-fit state and select the path's first work
 * phase.
 */
static void work_path_begin (bc_decoder_record_t * p_record, bool b_is_faint)
{
    bc_decoder_work_t * p_work = &p_record->work;
    p_work->b_is_faint         = b_is_faint;
    p_work->extrema_count      = 1U;
    p_work->cursor             = 0U;
    p_work->total_us           = 0U;
    p_work->max_step_us        = 0U;
    p_work->best               = 0U;
    p_work->best_us            = 0U;
    p_work->b_is_rising        = true;
    p_work->ext[0]             = 0U;
    p_work->ext_us[0]          = 0U;
    p_work->b_is_strong        = false;
    p_work->b_does_bias_vary   = false;
    p_work->orientation        = 0U;
    p_work->found              = 0U;
    p_work->chosen[0]          = 0;
    p_work->chosen[1]          = 0;
    p_work->phase              = b_is_faint ? RW_FAINT_WHITE : RW_PRIMARY;

    if (!b_is_faint)
    {
        work_next(p_record);
    }
}

/** @brief Freeze the current sample count, record whether the attempt is early
 * or provisional and put the recorder into processing state.
 */
static void work_begin (bc_decoder_record_t * p_record, bool b_is_provisional,
                        bool b_is_early)
{
    p_record->work                  = (bc_decoder_work_t){0};
    p_record->work.frozen_count     = p_record->count;
    p_record->work.b_is_provisional = b_is_provisional;
    p_record->work.b_is_early       = b_is_early;
    work_path_begin(p_record, false);
    p_record->state = S3_REC_PROCESSING;
}

/** @brief End the current extraction path so the dispatcher can select the
 * next path or finish the job.
 */
static void work_bad_path (bc_decoder_record_t * p_record)
{
    p_record->work.phase = RW_PATH_END;
}

/** @brief Retain a successful letter and direction, record disagreement with
 * an earlier path, and keep the required measured quiet margins for
 * final validation.
 */
static void work_keep_candidate (bc_decoder_record_t *       p_record,
                                 const bc_decoder_config_t * p_config,
                                 const bc_decoder_result_t * p_candidate)
{
    bc_decoder_work_t * p_work = &p_record->work;
    const int32_t       bias   = p_work->chosen[1];

    const int64_t half = bias > 0 ? ((int64_t)bias + 1) / 2 : (int64_t)bias / 2;
    const uint32_t quiet =
        p_config->quiet_modules * rec_narrow(&p_work->runs[20]);
    /* Widen both uint32_t times exactly before adding the signed half-width.
     * The upper limit is the clock half range minus one. The unsigned
     * conversion retains the low 32 bits if the sum is below zero. */
    const int64_t  end      = (int64_t)p_work->last_us + half + (int64_t)quiet;
    const uint32_t required = end > 0x7FFFFFFF ? 0x7FFFFFFFU : (uint32_t)end;

    /* The validated A..Z letter maps to a nonzero retained code in 1..26. */
    if ((p_work->b_has_candidate) &&
        ((p_work->candidate.letter_code !=
          (uint8_t)((int)p_candidate->letter - 'A' + 1)) ||
         (p_work->candidate.b_is_reversed != p_candidate->b_is_reversed)))
    {
        p_work->b_is_ambiguous = true;
    }

    if ((!p_work->b_is_faint) && (p_work->b_is_strong))
    {
        p_work->b_allows_noise = true;
    }

    if ((!p_work->b_has_candidate) || (required > p_work->required_us))
    {
        p_work->required_us = required;
    }

    if ((!p_work->b_has_candidate) || (quiet > p_work->quiet_required_us))
    {
        p_work->quiet_required_us = quiet;
    }

    /* clang-format off */
    p_work->candidate       = (bc_decoder_stored_result_t){
        (uint8_t)((int)p_candidate->letter - 'A' + 1),
        p_candidate->b_is_reversed};
    p_work->b_has_candidate = true;
    p_work->phase           = RW_PATH_END;
    /* clang-format on */
}

/** @brief Score one position of the sentinel bias grid. Invalid trials return
 * UINT32_MAX. Update the best score and break ties in favour of the
 * smaller absolute bias.
 *
 * Evaluate one unchanged grid position. The outer grid is resumed next
 * tick; every inner loop here is fixed at at most two nine-element
 * sentinels.
 */
static uint32_t work_fit_trial (bc_decoder_record_t *       p_record,
                                const bc_decoder_config_t * p_config)
{
    bc_decoder_work_t * p_work = &p_record->work;
    const uint16_t      star   = p_work->orientation ? 0x052U : 0x094U;
    /* Widen before multiplying by a signed grid tick. Each quotient
     * and the even varying-bias result stay within the signed bound. */
    const int32_t bias =
        p_work->b_does_bias_vary
            ? (int32_t)((int64_t)p_work->bound * p_work->tick / 128) * 2
            : (int32_t)((int64_t)p_work->bound * p_work->tick /
                        BIAS_GRID_RADIUS);
    uint32_t           score = 0U;
    const unsigned int begin =
        p_work->b_does_bias_vary ? p_work->star_index : 0U;
    const unsigned int end =
        p_work->b_does_bias_vary ? begin + 1U : CODE39_SENTINELS;

    for (unsigned int marker_index = begin; marker_index < end; marker_index++)
    {
        uint32_t adjusted[CODE39_ELEMENTS];
        uint32_t narrow;
        uint32_t wide = 0U;
        uint16_t pattern;

        for (unsigned int element_index = 0U; element_index < CODE39_ELEMENTS;
             element_index++)
        {
            /* A signed trial can move a full-width run in either
             * direction. Narrow only after checking both width limits. */
            const int64_t adjusted_us =
                (int64_t)p_work->runs[(marker_index * 20U) + element_index] +
                ((element_index % 2U) ? -bias : bias);

            if ((adjusted_us <= 0) ||
                (adjusted_us > (int64_t)p_config->max_run_us))
            {
                return UINT32_MAX;
            }

            adjusted[element_index] = (uint32_t)adjusted_us;

            if (star & (1U << (8U - element_index)))
            {
                wide += adjusted[element_index];
            }
        }

        if ((!character(adjusted, p_work->max_step_us, p_config, &pattern,
                        &narrow)) ||
            (pattern != star))
        {
            return UINT32_MAX;
        }

        const uint32_t error =
            rec_fit_error(adjusted, star, narrow, wide / CODE39_WIDE_ELEMENTS);

        if (error > score)
        {
            score = error;
        }
    }

    if ((score < p_work->best_score) ||
        ((score == p_work->best_score) &&
         ((bias < 0 ? -bias : bias) <
          (p_work->best_bias < 0 ? -p_work->best_bias : p_work->best_bias))))
    {
        p_work->best_score = score;
        p_work->best_bias  = bias;
    }

    return score;
}

/** @brief Apply the selected shared or varying bias to the scratch run widths.
 * Return false on an invalid corrected width. Partially updated
 * scratch belongs to the rejected fit.
 */
static bool work_apply (bc_decoder_work_t *         p_work,
                        const bc_decoder_config_t * p_config)
{
    uint64_t total      = 0U;
    uint64_t start      = 0U;
    uint64_t finish     = 0U;
    uint64_t elapsed_us = 0U;
    int32_t  previous   = -p_work->chosen[0] / 2;

    if (p_work->b_does_bias_vary)
    {
        for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS;
             run_index++)
        {
            if ((!p_work->runs[run_index]) ||
                (p_work->runs[run_index] > p_config->max_run_us))
            {
                return false;
            }

            total += p_work->runs[run_index];

            if (8U == run_index)
            {
                start = total;
            }

            if (19U == run_index)
            {
                finish = total;
            }
        }

        if (total > p_config->max_frame_us)
        {
            return false;
        }
    }
    /* Process 29 widths and use at most eleven interpolated edge divisions.
     * A chosen model that fails ends this path before scratch can be reused. */
    for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS; run_index++)
    {
        int64_t width;

        if (!p_work->b_does_bias_vary)
        {
            width = (int64_t)p_work->runs[run_index] +
                    ((run_index % 2U) ? -p_work->chosen[0] : p_work->chosen[0]);
        }
        else
        {
            int32_t bias;
            elapsed_us += p_work->runs[run_index];

            if (elapsed_us <= start)
            {
                bias = p_work->chosen[0];
            }
            else if (elapsed_us >= finish)
            {
                bias = p_work->chosen[1];
            }
            else
            {
                /* Divide the widened weighted sum before narrowing.
                 * The interpolated bias stays between the endpoint fits. */
                bias = (int32_t)((((int64_t)p_work->chosen[0] *
                                   (int64_t)(finish - elapsed_us)) +
                                  ((int64_t)p_work->chosen[1] *
                                   (int64_t)(elapsed_us - start))) /
                                 (int64_t)(finish - start));
            }

            const int32_t shift = (run_index % 2U) ? -bias / 2 : bias / 2;
            width    = (int64_t)p_work->runs[run_index] + shift - previous;
            previous = shift;
        }

        if ((width <= 0) || (width > (int64_t)p_config->max_run_us))
        {
            return false;
        }

        /* The validated width lies in 1..max_run_us, a uint32_t range. */
        p_work->runs[run_index] = (uint32_t)width;
    }

    return true;
}

/** @brief Advance one primary extrema scan step.
 * @param p_recorder Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_primary (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_workspace = &p_recorder->work;

    if (p_workspace->cursor >= p_workspace->frozen_count)
    {
        if (!p_workspace->b_is_rising)
        {
            /* clang-format off */
            p_workspace->ext[p_workspace->extrema_count]      =
                p_workspace->best;
            p_workspace->ext_us[p_workspace->extrema_count++] =
                p_workspace->best_us;
            /* clang-format on */
        }

        p_workspace->phase = RW_PRUNE_BEGIN;
        return;
    }
    else
    {
        const unsigned int sample_dark =
            REC_DARK(p_recorder, p_workspace->cursor);
        const unsigned int best_dark = REC_DARK(p_recorder, p_workspace->best);

        if (p_workspace->b_is_rising ? sample_dark > best_dark
                                     : sample_dark < best_dark)
        {
            p_workspace->best    = p_workspace->cursor;
            p_workspace->best_us = p_workspace->total_us;
        }
        else if ((p_workspace->b_is_rising ? best_dark - sample_dark
                                           : sample_dark - best_dark) >=
                 (p_recorder->prominence ? p_recorder->prominence
                                         : S3_REC_PROMINENCE))
        {
            if (p_workspace->extrema_count >= (S3_REC_EXT_MAX - 1U))
            {
                work_bad_path(p_recorder);
                return;
            }

            /* clang-format off */
            p_workspace->ext[p_workspace->extrema_count]      =
                p_workspace->best;
            p_workspace->ext_us[p_workspace->extrema_count++] =
                p_workspace->best_us;
            p_workspace->best                                 =
                p_workspace->cursor;
            p_workspace->best_us                              =
                p_workspace->total_us;
            p_workspace->b_is_rising                          =
                !p_workspace->b_is_rising;
            /* clang-format on */
        }
        else
        {
            /* Keep tracking the same extremum on the next work unit. */
        }

        work_next(p_recorder);
        return;
    }
}

/** @brief Advance one quiet-background step of faint recovery.
 * @param p_recorder Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_faint_white (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_workspace = &p_recorder->work;

    if (p_workspace->cursor >= p_workspace->frozen_count)
    {
        p_workspace->phase = RW_EDGES_BEGIN;
        return;
    }

    if (REC_DARK(p_recorder, p_workspace->cursor) <=
        (p_recorder->paper + S3_REC_FAINT_BAND))
    {
        work_next(p_recorder);
        return;
    }

    if ((p_workspace->extrema_count + 2U) > (S3_BARCODE_RUNS + 2U))
    {
        work_bad_path(p_recorder);
        return;
    }

    /* clang-format off */
    p_workspace->peak    = p_workspace->dip = p_workspace->cursor;
    p_workspace->peak_us = p_workspace->dip_us = p_workspace->total_us;
    p_workspace->low_us  = 0U;
    /* clang-format on */
    work_next(p_recorder);
    p_workspace->phase = RW_FAINT_INK;
    return;
}

/** @brief Advance one ink-region step of faint recovery.
 * @param p_recorder Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_faint_ink (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_workspace = &p_recorder->work;

    if (p_workspace->cursor >= p_workspace->frozen_count)
    {
        work_bad_path(p_recorder);
        return;
    }

    if (REC_DARK(p_recorder, p_workspace->cursor) >
        (p_recorder->paper + S3_REC_FAINT_BAND))
    {
        if (REC_DARK(p_recorder, p_workspace->cursor) >
            REC_DARK(p_recorder, p_workspace->peak))
        {
            p_workspace->peak    = p_workspace->cursor;
            p_workspace->peak_us = p_workspace->total_us;
        }

        p_workspace->low_us = 0U;
    }
    else
    {
        if ((!p_workspace->low_us) ||
            (REC_DARK(p_recorder, p_workspace->cursor) <
             REC_DARK(p_recorder, p_workspace->dip)))
        {
            p_workspace->dip    = p_workspace->cursor;
            p_workspace->dip_us = p_workspace->total_us;
        }

        p_workspace->low_us += REC_STEP_US(p_recorder, p_workspace->cursor);

        if (p_workspace->low_us > S3_REC_FAINT_DIP_US)
        {
            p_workspace->phase = RW_FAINT_VALLEY;
            return;
        }
    }

    work_next(p_recorder);
    return;
}

/** @brief Advance one valley step of faint recovery.
 * @param p_recorder Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_faint_valley (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_workspace = &p_recorder->work;

    if ((p_workspace->cursor < p_workspace->frozen_count) &&
        (REC_DARK(p_recorder, p_workspace->cursor) <=
         (p_recorder->paper + S3_REC_FAINT_BAND)))
    {
        if (REC_DARK(p_recorder, p_workspace->cursor) <
            REC_DARK(p_recorder, p_workspace->dip))
        {
            p_workspace->dip    = p_workspace->cursor;
            p_workspace->dip_us = p_workspace->total_us;
        }

        work_next(p_recorder);
        return;
    }

    p_workspace->ext[p_workspace->extrema_count]      = p_workspace->peak;
    p_workspace->ext_us[p_workspace->extrema_count++] = p_workspace->peak_us;
    p_workspace->ext[p_workspace->extrema_count]      = p_workspace->dip;
    p_workspace->ext_us[p_workspace->extrema_count++] = p_workspace->dip_us;
    p_workspace->phase                                = RW_FAINT_WHITE;
    return;
}

/** @brief Begin one bounded extrema-pruning iteration.
 * @param p_recorder Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_prune_begin (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_work = &p_recorder->work;

    if (p_work->extrema_count <= (S3_BARCODE_RUNS + 2U))
    {
        p_work->phase = RW_EDGES_BEGIN;
        return;
    }

    p_work->smallest = UINT64_MAX;
    p_work->cut      = 1U;
    p_work->scan     = 1U;
    p_work->phase    = RW_PRUNE_SCAN;
    return;
}

/** @brief Compare one extrema pair while choosing a pruning cut.
 * @param p_recorder Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_prune_scan (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_workspace = &p_recorder->work;

    if ((p_workspace->scan + 2U) >= p_workspace->extrema_count)
    {
        p_workspace->copy  = p_workspace->cut;
        p_workspace->phase = RW_PRUNE_COPY;
        return;
    }

    {
        const unsigned int prev_dark =
            REC_DARK(p_recorder, p_workspace->ext[p_workspace->scan]);
        const unsigned int next_dark =
            REC_DARK(p_recorder, p_workspace->ext[p_workspace->scan + 1U]);
        const uint64_t size =
            (uint64_t)(prev_dark > next_dark ? prev_dark - next_dark
                                             : next_dark - prev_dark) *
            (p_workspace->ext_us[p_workspace->scan + 1U] -
             p_workspace->ext_us[p_workspace->scan]);

        if (size < p_workspace->smallest)
        {
            p_workspace->smallest = size;
            p_workspace->cut      = p_workspace->scan;
        }

        p_workspace->scan++;
        return;
    }
}

/** @brief Copy one retained extrema entry or complete its pruning cut.
 * @param p_recorder Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_prune_copy (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_work = &p_recorder->work;

    if ((p_work->copy + 2U) >= p_work->extrema_count)
    {
        p_work->extrema_count -= 2U;
        p_work->phase          = RW_PRUNE_BEGIN;
        return;
    }

    p_work->ext[p_work->copy]    = p_work->ext[p_work->copy + 2U];
    p_work->ext_us[p_work->copy] = p_work->ext_us[p_work->copy + 2U];
    p_work->copy++;
    return;
}

/** @brief Check extrema count and prominence, then initialize edge traversal.
 * @param p_recorder Nonnull caller-owned record with the current workspace
 * state.
 * @pre The caller owns the record exclusively and has charged one work unit.
 * @post Mutates the same workspace fields as one original phase dispatch;
 * no allocation, locking or complete-frame rescan is introduced.
 */
static void work_edges_begin (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_workspace = &p_recorder->work;

    if (p_workspace->extrema_count != (S3_BARCODE_RUNS + 2U))
    {
        work_bad_path(p_recorder);
        return;
    }

    p_workspace->b_is_strong = !p_workspace->b_is_faint;

    for (unsigned int extrema_index                                        = 1U;
         (extrema_index + 1U) < p_workspace->extrema_count; extrema_index += 2U)
    {
        const unsigned int peak_dark =
            REC_DARK(p_recorder, p_workspace->ext[extrema_index]);
        const unsigned int prev_dark =
            REC_DARK(p_recorder, p_workspace->ext[extrema_index - 1U]);
        const unsigned int next_dark =
            REC_DARK(p_recorder, p_workspace->ext[extrema_index + 1U]);

        if ((peak_dark < (prev_dark + S3_QUIET_STRONG_PROMINENCE)) ||
            (peak_dark < (next_dark + S3_QUIET_STRONG_PROMINENCE)))
        {
            p_workspace->b_is_strong = false;
        }
    }

    p_workspace->segment          = 0U;
    p_workspace->edges            = 0U;
    p_workspace->edge_previous_us = 0U;
    p_workspace->phase            = RW_EDGE_BEGIN;
    return;
}

/** @brief Initialize one edge segment or advance to direct decoding.
 * @param p_recorder Nonnull caller-owned record with the current workspace
 * state.
 * @pre The caller owns the record exclusively and has charged one work unit.
 * @post Mutates the same workspace fields as one original phase dispatch;
 * no allocation, locking or complete-frame rescan is introduced.
 */
static void work_edge_begin (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_work = &p_recorder->work;

    if ((p_work->segment + 1U) >= p_work->extrema_count)
    {
        p_work->last_us = p_work->edge_previous_us;
        p_work->phase   = RW_DIRECT;
        return;
    }

    if (p_work->ext[p_work->segment + 1U] <= p_work->ext[p_work->segment])
    {
        work_bad_path(p_recorder);
        return;
    }

    p_work->edge_cursor   = p_work->ext[p_work->segment] + 1U;
    p_work->segment_us    = p_work->ext_us[p_work->segment];
    p_work->integral_us   = 0U;
    p_work->b_has_crossed = false;
    p_work->phase         = RW_EDGE_SCAN;
    return;
}

/** @brief Accumulate one sample interval and update a completed edge segment.
 * @param p_recorder Nonnull caller-owned record with the current workspace
 * state.
 * @pre The caller owns the record exclusively and has charged one work unit.
 * @post Mutates the same workspace fields as one original phase dispatch;
 * no allocation, locking or complete-frame rescan is introduced.
 */
static void work_edge_scan (bc_decoder_record_t * p_recorder)
{
    bc_decoder_work_t * p_workspace = &p_recorder->work;
    {
        const unsigned int sample_index  = p_workspace->edge_cursor;
        const unsigned int segment_index = p_workspace->segment;
        const unsigned int value_lo =
            REC_DARK(p_recorder, p_workspace->ext[segment_index]);
        const unsigned int value_hi =
            REC_DARK(p_recorder, p_workspace->ext[segment_index + 1U]);
        const unsigned int prev_dark = REC_DARK(p_recorder, sample_index - 1U);
        const unsigned int next_dark = REC_DARK(p_recorder, sample_index);
        const unsigned int mid       = (value_lo + value_hi) / 2U;
        const uint32_t     step      = REC_STEP_US(p_recorder, sample_index);
        const bool         b_is_rising = value_hi > value_lo;
        const bool         b_should_use_core_crossing =
            (p_workspace->b_is_faint) && (((b_is_rising ? value_hi : value_lo) -
                                           p_recorder->paper) <= S3_REC_RISE);
        const bool b_has_crossed_edge =
            b_is_rising ? (b_should_use_core_crossing
                               ? (prev_dark <= mid) && (next_dark > mid)
                               : (prev_dark < mid) && (next_dark >= mid))
                        : (prev_dark > mid) && (next_dark <= mid);
        p_workspace->integral_us +=
            (step * (rec_frac(prev_dark, value_lo, value_hi) +
                     rec_frac(next_dark, value_lo, value_hi))) /
            2048U;

        if (b_has_crossed_edge)
        {
            const uint32_t edge =
                p_workspace->segment_us +
                ((step *
                  (uint32_t)(b_is_rising ? mid - prev_dark : prev_dark - mid)) /
                 (uint32_t)(b_is_rising ? next_dark - prev_dark
                                        : prev_dark - next_dark));

            if ((!p_workspace->b_has_crossed) ||
                (b_should_use_core_crossing ? b_is_rising : !b_is_rising))
            {
                p_workspace->edge_us = edge;
            }

            p_workspace->b_has_crossed = true;
        }

        p_workspace->segment_us += step;
        p_workspace->edge_cursor++;

        if (sample_index == p_workspace->ext[segment_index + 1U])
        {
            if (!p_workspace->b_has_crossed)
            {
                p_workspace->edge_us = p_workspace->ext_us[segment_index] +
                                       p_workspace->integral_us;
            }

            if (!p_workspace->edges)
            {
                p_workspace->first_us = p_workspace->edge_us;
            }
            else
            {
                p_workspace->runs[p_workspace->edges - 1U] =
                    p_workspace->edge_us - p_workspace->edge_previous_us;
            }

            p_workspace->edge_previous_us = p_workspace->edge_us;
            p_workspace->edges++;
            p_workspace->segment++;
            p_workspace->phase = RW_EDGE_BEGIN;
        }

        return;
    }
}

/** @brief Check the adjusted quiet interval and decode the candidate runs.
 * @param p_recorder Recorder and workspace owned by the sampling task.
 * @param p_config Validated timing and width limits for this scan.
 * @post Keep a valid candidate or mark this path unsuccessful.
 */
static void work_validate (bc_decoder_record_t *       p_recorder,
                           const bc_decoder_config_t * p_config)
{
    bc_decoder_work_t * p_work = &p_recorder->work;
    bc_decoder_result_t candidate;
    const int64_t       half = p_work->chosen[0] > 0
                                   ? ((int64_t)p_work->chosen[0] + 1) / 2
                                   : (int64_t)p_work->chosen[0] / 2;
    const int64_t       leading =
        (int64_t)sat_add(p_recorder->leading_us, p_work->first_us) - half;

    /* Reject negative leading time. The unsigned conversion retains the
     * low 32 bits; a negative bias can extend a saturated leading sum. */
    if ((leading >= 0) &&
        (bc_decoder_decode_runs(p_work->runs, S3_BARCODE_RUNS,
                                (uint32_t)leading, 0x7FFFFFFFU,
                                p_work->max_step_us, p_config, &candidate)))
    {
        work_keep_candidate(p_recorder, p_config, &candidate);
    }
    else
    {
        work_bad_path(p_recorder);
    }
}

/** @brief Advance pending decoder work within the current update's budgets.
 * @param p_recorder Recorder and workspace owned by the sampling task.
 * @param p_config Validated scan configuration.
 * @note Charge a work unit before dispatch. If the next step is a fit trial
 * and the fit budget is exhausted, defer it without charging either counter.
 * @details The phases select the extraction path, prune extrema, locate
 * edges, then test the direct and fitted runs. The retained workspace lets
 * the next sample resume at the same phase with fresh work and fit budgets.
 */
static void work_advance (bc_decoder_record_t *       p_recorder,
                          const bc_decoder_config_t * p_config)
{
    bc_decoder_work_t * p_work = &p_recorder->work;

    while ((p_recorder->work_units < S3_REC_WORK_UNITS) &&
           ((int)p_work->phase != RW_DONE))
    {
        if ((RW_FIT_TICK == (int)p_work->phase) &&
            (p_recorder->fit_ticks >= S3_REC_FIT_TICKS))
        {
            break;
        }

        p_recorder->work_units++;

        switch (p_work->phase)
        {
        case RW_PRIMARY:
            work_primary(p_recorder);
        break;
        case RW_FAINT_WHITE:
            work_faint_white(p_recorder);
        break;
        case RW_FAINT_INK:
            work_faint_ink(p_recorder);
        break;
        case RW_FAINT_VALLEY:
            work_faint_valley(p_recorder);
        break;
        case RW_PRUNE_BEGIN:
            work_prune_begin(p_recorder);
        break;
        case RW_PRUNE_SCAN:
            work_prune_scan(p_recorder);
        break;
        case RW_PRUNE_COPY:
            work_prune_copy(p_recorder);
        break;
        case RW_EDGES_BEGIN:
            work_edges_begin(p_recorder);
        break;
        case RW_EDGE_BEGIN:
            work_edge_begin(p_recorder);
        break;
        case RW_EDGE_SCAN:
            work_edge_scan(p_recorder);
        break;
        case RW_DIRECT:
        {
            bc_decoder_result_t candidate;

            const uint32_t leading =
                sat_add(p_recorder->leading_us, p_work->first_us);

            if (bc_decoder_decode_runs(p_work->runs, S3_BARCODE_RUNS, leading,
                                       0x7FFFFFFFU, p_work->max_step_us,
                                       p_config, &candidate))
            {
                work_keep_candidate(p_recorder, p_config, &candidate);
                break;
            }

            p_work->orientation      = 0U;
            p_work->star_index       = 0U;
            p_work->found            = 0U;
            p_work->b_does_bias_vary = false;
            p_work->phase            = RW_FIT_BEGIN;
        }
        break;
        case RW_FIT_BEGIN:
            if (p_work->orientation >= CODE39_ORIENTATIONS)
            {
                p_work->phase = RW_FIT_END;
                break;
            }
            {
                const uint16_t     star = p_work->orientation ? 0x052U : 0x094U;
                const unsigned int begin =
                    p_work->b_does_bias_vary ? p_work->star_index : 0U;
                const unsigned int end =
                    p_work->b_does_bias_vary ? begin + 1U : CODE39_SENTINELS;
                p_work->minimum = UINT32_MAX;

                for (unsigned int marker_index = begin; marker_index < end;
                     marker_index++)
                {
                    uint64_t sum = 0U;

                    for (unsigned int element_index = 0U;
                         element_index < CODE39_ELEMENTS; element_index++)
                    {
                        if (!(star & (1U << (8U - element_index))))
                        {
                            sum += p_work->runs[(marker_index * 20U) +
                                                element_index];
                        }
                    }

                    /* Each mean of uint32_t widths fits uint32_t. */
                    p_work->narrow[marker_index] =
                        (uint32_t)(sum / CODE39_NARROW_ELEMENTS);

                    if (p_work->narrow[marker_index] < p_work->minimum)
                    {
                        p_work->minimum = p_work->narrow[marker_index];
                    }
                }

                if ((p_work->minimum < p_config->min_narrow_us) ||
                    (p_work->minimum > p_config->max_run_us))
                {
                    p_work->orientation++;
                    p_work->star_index = 0U;
                    break;
                }

                /* The configured maximum run is below 2^31 / 10.
                 * Tripling the minimum fits before the signed bound cast. */
                p_work->bound      = (int32_t)(p_work->minimum * 3U / 4U);
                p_work->best_score = UINT32_MAX;
                p_work->best_bias  = 0;
                p_work->tick       = -BIAS_GRID_RADIUS;
                p_work->phase      = RW_FIT_TICK;
            }
        break;
        case RW_FIT_TICK:
            (void)work_fit_trial(p_recorder, p_config);
            p_recorder->fit_ticks++;
            p_work->tick++;

            if (p_work->tick <= BIAS_GRID_RADIUS)
            {
                break;
            }

            if (!p_work->b_does_bias_vary)
            {
                if (p_work->best_score != UINT32_MAX)
                {
                    p_work->found++;
                    p_work->chosen[0] = p_work->chosen[1] = p_work->best_bias;
                }

                p_work->orientation++;
            }
            else if (UINT32_MAX == p_work->best_score)
            {
                p_work->orientation++;
                p_work->star_index = 0U;
            }
            else
            {
                p_work->trial_bias[p_work->star_index] = p_work->best_bias;

                if (!p_work->star_index)
                {
                    p_work->star_index = 1U;
                }
                else
                {
                    const int32_t  first_bias = p_work->trial_bias[0];
                    const int32_t  last_bias  = p_work->trial_bias[1];
                    const int32_t  delta      = last_bias - first_bias;
                    const uint32_t minimum =
                        p_work->narrow[0] < p_work->narrow[1]
                            ? p_work->narrow[0]
                            : p_work->narrow[1];

                    if ((!(((first_bias < 0) && (last_bias > 0)) ||
                           ((first_bias > 0) && (last_bias < 0)))) &&
                        ((uint32_t)(delta < 0 ? -delta : delta) <=
                         (minimum / 2U)))
                    {
                        p_work->chosen[0] = first_bias;
                        p_work->chosen[1] = last_bias;
                        p_work->found++;
                    }

                    p_work->orientation++;
                    p_work->star_index = 0U;
                }
            }

            p_work->phase = RW_FIT_BEGIN;
        break;
        case RW_FIT_END:
            if (1U == p_work->found)
            {
                p_work->phase = RW_APPLY;
                break;
            }

            if ((!p_work->b_does_bias_vary) && (!p_work->found))
            {
                p_work->b_does_bias_vary = true;
                p_work->orientation      = 0U;
                p_work->star_index       = 0U;
                p_work->phase            = RW_FIT_BEGIN;
                break;
            }

            work_bad_path(p_recorder);
        break;
        case RW_APPLY:
            p_work->phase =
                work_apply(p_work, p_config) ? RW_VALIDATE : RW_PATH_END;
        break;
        case RW_VALIDATE:
            work_validate(p_recorder, p_config);
        break;
        case RW_PATH_END:
            if ((!p_work->b_is_faint) && (rec_faint_eligible(p_recorder)))
            {
                work_path_begin(p_recorder, true);
            }
            else
            {
                p_work->phase = RW_DONE;
            }
        break;
        default:
            p_work->phase          = RW_DONE;
            p_work->b_is_ambiguous = true;
        break;
        }
    }
}

/** @brief End a recording with the supplied status. Preserve paper, noise,
 * timing continuity and the last work counters. After a failure,
 * require a fresh paper transition before recording again.
 */
static bc_decoder_status_t rec_finish (bc_decoder_record_t * p_record,
                                       bc_decoder_status_t   status)
{
    const uint16_t paper      = p_record->paper;
    const uint16_t noise      = p_record->noise_estimate;
    const bool     b_was_seen = p_record->b_has_sample;
    const uint32_t last       = p_record->last_us;
    const uint16_t units      = p_record->work_units;
    const uint16_t ticks      = p_record->fit_ticks;
    bc_decoder_record_reset(p_record);
    p_record->b_has_sample   = b_was_seen;
    p_record->paper          = paper;
    p_record->last_us        = last; /* Keep the paper reference. */
    p_record->noise_estimate = noise;
    p_record->prev           = paper;
    p_record->work_units     = units;
    p_record->fit_ticks      = ticks;
    /* Like bc_decoder_update: after a failure, only a new white-to-dark
     * start can begin the next recording (no second report mid-bar). */
    p_record->b_is_blocked =
        (status != (unsigned int)S3_BARCODE_DECODED) ? 1U : 0U;
    return status;
}

/* End of file. */

