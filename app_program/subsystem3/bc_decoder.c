/** @file bc_decoder.c
 * @brief Decode Code 39 letters from bounded streaming and recorded reflectance
 * samples.
 */
/* Code 39 symbol facts cross-checked against ZXing Code39Reader encodings:
 * Repository: https://github.com/zxing/zxing
 * Source: core/src/main/java/com/google/zxing/oned/Code39Reader.java
 * This decoder uses its own bounded duration/sampling validation algorithm. */
#include "bc_decoder.h"
static const uint16_t g_letters[26] = {
    0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U, 0x00DU, 0x10CU, 0x04CU,
    0x01CU, 0x103U, 0x043U, 0x142U, 0x013U, 0x112U, 0x052U, 0x007U, 0x106U,
    0x046U, 0x016U, 0x181U, 0x0C1U, 0x1C0U, 0x091U, 0x190U, 0x0D0U};

s3_barcode_config_t s3_barcode_defaults (void)
{
    /* At 0.04 m/s a 19 mm wide element lasts 475 ms. A 2:1 symbol can
     * require ~11.4 s including trailing quiet; allow bounded extra time. */
    /* At 0.10 m/s the small F's measured narrow means are about 29..35 ms.
     * Permit 25 ms modules, retaining character()'s independent requirement
     * of at least three observed sample intervals per narrow mean. */
    return (s3_barcode_config_t){30000U, 25000U, 1000000U, 15000000U, 10U};
}

static bool config_valid (const s3_barcode_config_t * p_config)
{
    return (p_config != 0) && (p_config->max_gap_us > 0U) &&
           (p_config->min_narrow_us > 0U) &&
           (p_config->max_run_us >= p_config->min_narrow_us) &&
           (p_config->max_frame_us > p_config->max_run_us) &&
           (p_config->max_frame_us < 0x80000000U) &&
           (p_config->max_gap_us <= p_config->max_run_us) &&
           (p_config->quiet_modules >= 10U) &&
           (p_config->quiet_modules <= 50U) &&
           ((uint64_t)p_config->max_run_us * p_config->quiet_modules <
            0x80000000ULL);
}

void s3_barcode_reset (s3_barcode_decoder_t * p_decoder)
{
    if (p_decoder != 0)
    {
        s3_barcode_stored_config_t config = p_decoder->config;
        bool configured = p_decoder->configured;
        *p_decoder = (s3_barcode_decoder_t){0};
        p_decoder->config = config;
        p_decoder->configured = configured;
    }
}

bool s3_barcode_init (s3_barcode_decoder_t * p_decoder,
                      const s3_barcode_config_t * p_config)
{
    if (p_decoder == 0)
    {
        return false;
    }
    *p_decoder = (s3_barcode_decoder_t){0};
    if (!config_valid(p_config))
    {
        return false;
    }
    p_decoder->config = (s3_barcode_stored_config_t){
        p_config->max_gap_us, p_config->min_narrow_us, p_config->max_run_us,
        p_config->max_frame_us, p_config->quiet_modules};
    p_decoder->configured = true;
    return true;
}

/* Exactly six narrow and three wide elements, normally consistent to 35%.
 * Only the payload may contain one narrow white gap up to 150% of its mean.
 * Width ratio 1.8..3.5 permits rasterisation/jitter around nominal 2..3. */
static bool character_widths (const uint32_t * p_runs, uint32_t gap,
                              const s3_barcode_config_t * p_config,
                              uint16_t * p_pattern, uint32_t * p_narrow,
                              bool payload)
{
    uint32_t sorted[9];
    uint64_t nsum = 0U, wsum = 0U;
    uint32_t wide, split;
    unsigned int value_i, value_j, extended_spaces = 0U;
    *p_pattern = 0U;
    for (value_i = 0U; value_i < 9U; value_i++)
    {
        if ((p_runs[value_i] == 0U) || (p_runs[value_i] > p_config->max_run_us))
        {
            return false;
        }
        sorted[value_i] = p_runs[value_i];
    }
    for (value_i = 1U; value_i < 9U; value_i++)
    {
        uint32_t item = sorted[value_i];
        value_j = value_i;
        while ((value_j > 0U) && (sorted[value_j - 1U] > item))
        {
            sorted[value_j] = sorted[value_j - 1U];
            value_j--;
        }
        sorted[value_j] = item;
    }
    for (value_i = 0U; value_i < 6U; value_i++)
    {
        nsum += sorted[value_i];
    }
    for (value_i = 6U; value_i < 9U; value_i++)
    {
        wsum += sorted[value_i];
    }
    *p_narrow = (uint32_t)(nsum / 6U);
    wide = (uint32_t)(wsum / 3U);
    if ((*p_narrow < p_config->min_narrow_us) ||
        ((uint64_t)*p_narrow < 3ULL * gap) ||
        ((uint64_t)wide * 100U < (uint64_t)*p_narrow * 180U) ||
        ((uint64_t)wide * 100U > (uint64_t)*p_narrow * 350U))
    {
        return false;
    }
    split = (uint32_t)(((uint64_t)sorted[5] + sorted[6]) / 2U);
    for (value_i = 0U; value_i < 9U; value_i++)
    {
        bool is_wide = p_runs[value_i] > split;
        uint32_t reference = is_wide ? wide : *p_narrow;
        const uint64_t scaled = (uint64_t)p_runs[value_i] * 100U;
        if (scaled < (uint64_t)reference * S3_BARCODE_WIDTH_MIN_PERCENT)
        {
            return false;
        }
        if (scaled > (uint64_t)reference * S3_BARCODE_WIDTH_MAX_PERCENT)
        {
            if (!payload || is_wide || !(value_i % 2U) ||
                (extended_spaces != 0U) ||
                (scaled >
                 (uint64_t)reference * S3_BARCODE_PAYLOAD_SPACE_MAX_PERCENT))
            {
                return false;
            }
            extended_spaces++;
        }
        *p_pattern = (uint16_t)((*p_pattern << 1) | (is_wide ? 1U : 0U));
    }
    return true;
}

/* Sentinel fitting always uses the original strict width checks. */
static bool character (const uint32_t * p_runs, uint32_t gap,
                       const s3_barcode_config_t * p_config,
                       uint16_t * p_pattern, uint32_t * p_narrow)
{
    return character_widths(p_runs, gap, p_config, p_pattern, p_narrow, false);
}

bool s3_barcode_decode_runs (const uint32_t * p_runs, unsigned int count,
                             uint32_t leading_us, uint32_t trailing_us,
                             uint32_t observed_gap_us,
                             const s3_barcode_config_t * p_config,
                             s3_barcode_result_t * p_result)
{
    unsigned int orientation, value_i, value_k;
    uint64_t total = 0U;
    if (p_result == 0)
    {
        return false;
    }
    *p_result = (s3_barcode_result_t){0};
    if ((p_runs == 0) || (count != S3_BARCODE_RUNS) ||
        !config_valid(p_config) || (observed_gap_us > p_config->max_gap_us))
    {
        return false;
    }
    for (value_i = 0U; value_i < count; value_i++)
    {
        if ((p_runs[value_i] == 0U) || (p_runs[value_i] > p_config->max_run_us))
        {
            return false;
        }
        total += p_runs[value_i];
    }
    if (total > p_config->max_frame_us)
    {
        return false;
    }
    for (orientation = 0U; orientation < 2U; orientation++)
    {
        uint32_t ordered[S3_BARCODE_RUNS], narrow[3];
        uint16_t pattern[3];
        bool good = true;
        for (value_i = 0U; value_i < count; value_i++)
        {
            ordered[value_i] =
                p_runs[orientation ? count - 1U - value_i : value_i];
        }
        for (value_i = 0U; value_i < 3U; value_i++)
        {
            if (!character_widths(&ordered[value_i * 10U], observed_gap_us,
                                  p_config, &pattern[value_i], &narrow[value_i],
                                  value_i == 1U))
            {
                good = false;
                break;
            }
        }
        if (!good || (pattern[0] != 0x094U) || (pattern[2] != 0x094U))
        {
            continue;
        }
        for (value_i = 0U; value_i < 2U; value_i++)
        {
            uint64_t reference =
                ((uint64_t)narrow[value_i] + narrow[value_i + 1U]) / 2U;
            uint64_t space = ordered[9U + value_i * 10U];
            if ((space * 2U < reference) || (space * 2U > reference * 3U))
            {
                good = false;
            }
        }
        if (!good ||
            ((uint64_t)(orientation ? trailing_us : leading_us) <
             (uint64_t)p_config->quiet_modules * narrow[0]) ||
            ((uint64_t)(orientation ? leading_us : trailing_us) <
             (uint64_t)p_config->quiet_modules * narrow[2]))
        {
            continue;
        }
        for (value_k = 0U; value_k < 26U; value_k++)
        {
            if (pattern[1] == g_letters[value_k])
            {
                p_result->letter = (char)('A' + value_k);
                p_result->reversed = orientation != 0U;
                return true;
            }
        }
    }
    return false;
}

static void seed (s3_barcode_decoder_t * p_decoder, uint32_t now, bool black)
{
    s3_barcode_reset(p_decoder);
    p_decoder->seen = true;
    p_decoder->black = black;
    p_decoder->last_us = now;
    p_decoder->run_start_us = now;
}

s3_barcode_status_t s3_barcode_update (s3_barcode_decoder_t * p_decoder,
                                       uint32_t timestamp_us, bool valid,
                                       bool black,
                                       s3_barcode_result_t * p_result)
{
    uint32_t gap, elapsed;
    if (p_result == 0)
    {
        return S3_BARCODE_REJECTED;
    }
    *p_result = (s3_barcode_result_t){0};
    if ((p_decoder == 0) || !p_decoder->configured)
    {
        return S3_BARCODE_REJECTED;
    }
    if (!valid)
    {
        s3_barcode_reset(p_decoder);
        return S3_BARCODE_REJECTED;
    }
    if (!p_decoder->seen)
    {
        seed(p_decoder, timestamp_us, black);
        return S3_BARCODE_NONE;
    }
    gap = timestamp_us - p_decoder->last_us;
    if ((gap == 0U) || (gap > p_decoder->config.max_gap_us))
    {
        seed(p_decoder, timestamp_us, black);
        return S3_BARCODE_TIMING_ERROR;
    }
    p_decoder->last_us = timestamp_us;
    elapsed = timestamp_us - p_decoder->run_start_us;
    if (p_decoder->collecting)
    {
        if (gap > p_decoder->observed_gap_us)
        {
            p_decoder->observed_gap_us = gap;
        }
        if ((uint32_t)(timestamp_us - p_decoder->frame_start_us) >
            p_decoder->config.max_frame_us)
        {
            seed(p_decoder, timestamp_us, black);
            return S3_BARCODE_REJECTED;
        }
    }
    if (p_decoder->pending)
    {
        if (black)
        {
            seed(p_decoder, timestamp_us, black);
            return S3_BARCODE_REJECTED;
        }
        if (elapsed >= p_decoder->trailing_required_us)
        {
            p_result->letter =
                p_decoder->candidate.letter_code == 0U
                    ? 0
                    : (char)('A' + p_decoder->candidate.letter_code - 1U);
            p_result->reversed = p_decoder->candidate.reversed;
            p_result->timestamp_us = timestamp_us;
            p_decoder->collecting = false;
            p_decoder->pending = false;
            p_decoder->count = 0U;
            return S3_BARCODE_DECODED;
        }
        return S3_BARCODE_NONE;
    }
    if (black != p_decoder->black)
    {
        if (p_decoder->collecting)
        {
            if (p_decoder->count >= S3_BARCODE_RUNS)
            {
                seed(p_decoder, timestamp_us, black);
                return S3_BARCODE_OVERFLOW;
            }
            p_decoder->runs[p_decoder->count++] = elapsed;
            if (p_decoder->count == S3_BARCODE_RUNS)
            {
                uint16_t pattern;
                uint32_t narrow;
                s3_barcode_result_t candidate;
                const s3_barcode_config_t config = {
                    p_decoder->config.max_gap_us,
                    p_decoder->config.min_narrow_us,
                    p_decoder->config.max_run_us,
                    p_decoder->config.max_frame_us,
                    p_decoder->config.quiet_modules};
                if (black ||
                    !s3_barcode_decode_runs(p_decoder->runs, p_decoder->count,
                                            p_decoder->leading_us, 0x7FFFFFFFU,
                                            p_decoder->observed_gap_us, &config,
                                            &candidate) ||
                    !character(&p_decoder->runs[20], p_decoder->observed_gap_us,
                               &config, &pattern, &narrow))
                {
                    seed(p_decoder, timestamp_us, black);
                    return S3_BARCODE_REJECTED;
                }
                p_decoder->candidate = (s3_barcode_stored_result_t){
                    (uint8_t)(candidate.letter - 'A' + 1), candidate.reversed};
                p_decoder->trailing_required_us =
                    narrow * p_decoder->config.quiet_modules;
                p_decoder->pending = true;
            }
        }
        else if (black)
        {
            p_decoder->leading_us = elapsed;
            p_decoder->frame_start_us = timestamp_us;
            p_decoder->collecting = true;
            p_decoder->count = 0U;
            p_decoder->observed_gap_us = gap;
        }
        p_decoder->black = black;
        p_decoder->run_start_us = timestamp_us;
    }
    else if (p_decoder->collecting && (elapsed > p_decoder->config.max_run_us))
    {
        seed(p_decoder, timestamp_us, black);
        return S3_BARCODE_REJECTED;
    }
    return S3_BARCODE_NONE;
}

/* ---- Record-then-decide reader ------------------------------------------ */
#define REC_DARK(r, i) ((unsigned int)((r)->sample[i] & 0x3FFU))
#define REC_STEP_US(r, i) ((uint32_t)((r)->sample[i] >> 10) * S3_REC_STEP_US)

void s3_barcode_record_reset (s3_barcode_record_t * p_record)
{
    if (p_record != 0)
    {
        *p_record = (s3_barcode_record_t){0};
    }
}

bool s3_barcode_record_active (const s3_barcode_record_t * p_record)
{
    return (p_record != 0) && (p_record->state != (uint16_t)S3_REC_WAITING);
}

/* Elapsed time as a 6-bit step, carrying the rounding remainder forward so
 * summed steps track real time. False when the gap does not fit (stall). */
static bool rec_step (s3_barcode_record_t * p_r, uint32_t gap_us,
                      uint16_t * p_step)
{
    uint32_t total = gap_us + p_r->residual_us, units = total / S3_REC_STEP_US;
    if ((gap_us == 0U) || (units > S3_REC_STEP_MAX))
    {
        return false;
    }
    p_r->residual_us = (uint16_t)(total - units * S3_REC_STEP_US);
    *p_step = (uint16_t)units;
    return true;
}

/* Calibration can clamp a quiet paper background near zero while narrow
 * bars still rise only 10..25 units. Keep their small turns and quiet-zone
 * interruptions visible. The normal batch extractor retains its learned
 * prominence; this only enables a bounded second extraction if bars are lost.
 */
static bool rec_faint_eligible (const s3_barcode_record_t * p_r)
{
    return p_r != 0 && p_r->paper <= S3_REC_FAINT_BAND &&
           p_r->prominence != 0U && p_r->prominence <= S3_REC_FAINT_MAX_NOISE;
}

static unsigned int rec_monitor_band (const s3_barcode_record_t * p_r)
{
    return rec_faint_eligible(p_r) ? S3_REC_FAINT_BAND : p_r->prominence;
}

static unsigned int rec_white_band (const s3_barcode_record_t * p_r)
{
    unsigned int band = S3_REC_WHITE_BAND;
    const unsigned int monitor = rec_monitor_band(p_r);
    if ((monitor != 0U) && (monitor < band))
    {
        band = monitor;
    }
    return band;
}

static bool rec_white (const s3_barcode_record_t * p_r, unsigned int dark)
{
    return dark <= (unsigned int)p_r->paper + rec_white_band(p_r);
}

static uint32_t sat_add (uint32_t value_a, uint32_t value_b)
{
    return (value_a > 0x7FFFFFFFU - value_b) ? 0x7FFFFFFFU : value_a + value_b;
}

/* Keep real, timestamped readings while waiting. The first ring entry has no
 * preceding step: dropping it also drops the next entry's incoming interval. */
static void rec_pre_drop (s3_barcode_record_t * p_r)
{
    const bool was_white =
        (p_r->pre[p_r->pre_head] & 0x3FFU) <= S3_REC_FAINT_BAND;
    uint32_t elapsed;
    p_r->pre_head = (uint16_t)((p_r->pre_head + 1U) % S3_REC_PRE_MAX);
    p_r->pre_count--;
    elapsed = (uint32_t)(p_r->pre[p_r->pre_head] >> 10) * S3_REC_STEP_US;
    /* While waiting, leading_us measures verified low-band white BEFORE
     * the retained history. A faint mark that falls out of the ring must
     * still break that margin; the stronger recording trigger cannot prove
     * that everything before it was white. The absolute band is conservative
     * for all eligible paper levels (0..5), even if the reference changes. */
    p_r->leading_us =
        was_white && ((p_r->pre[p_r->pre_head] & 0x3FFU) <= S3_REC_FAINT_BAND)
            ? sat_add(p_r->leading_us, elapsed)
            : 0U;
    p_r->pre_us -= elapsed;
    p_r->pre[p_r->pre_head] &= 0x3FFU;
}

static void rec_pre_push (s3_barcode_record_t * p_r, unsigned int dark,
                          uint16_t step)
{
    unsigned int value_at;
    if (p_r->pre_count == S3_REC_PRE_MAX)
    {
        rec_pre_drop(p_r);
    }
    if (p_r->pre_count == 0U)
    {
        step = 0U;
    }
    value_at = ((unsigned int)p_r->pre_head + p_r->pre_count) % S3_REC_PRE_MAX;
    p_r->pre[value_at] = (uint16_t)(dark | ((unsigned int)step << 10));
    p_r->pre_count++;
    p_r->pre_us += (uint32_t)step * S3_REC_STEP_US;
    while ((p_r->pre_count > 1U) && (p_r->pre_us > S3_REC_PRE_US))
    {
        rec_pre_drop(p_r);
    }
}

/* The waiting EWMA can include faint ink below the stronger recording trigger.
 * If the retained history starts with an actual blank-paper interval, measure
 * its noise alone. Stop at the first ink sample; never use inter-bar valleys
 * to certify a noisy background as quiet. Otherwise retain the EWMA estimate.
 */
static unsigned int rec_paper_prominence (const s3_barcode_record_t * p_r,
                                          const s3_barcode_config_t * p_config)
{
    unsigned int prominence = (p_r->noise_q4 + 3U) / 4U;
    if (p_r->paper <= S3_REC_FAINT_BAND && p_r->pre_count > 1U)
    {
        const unsigned int floor = (unsigned int)p_r->paper + S3_REC_FAINT_BAND;
        unsigned int previous = p_r->pre[p_r->pre_head] & 0x3FFU,
                     differences = 0U;
        uint32_t elapsed = 0U, sum = 0U;
        for (unsigned int value_i = 1U;
             previous <= floor && value_i < p_r->pre_count; value_i++)
        {
            const uint16_t entry =
                p_r->pre[((unsigned int)p_r->pre_head + value_i) %
                         S3_REC_PRE_MAX];
            const unsigned int dark = entry & 0x3FFU;
            if (dark > floor)
            {
                break;
            }
            elapsed += (uint32_t)(entry >> 10) * S3_REC_STEP_US;
            sum += (dark > previous) ? dark - previous : previous - dark;
            differences++;
            previous = dark;
        }
        if (differences != 0U &&
            elapsed >= p_config->quiet_modules * p_config->min_narrow_us)
        {
            prominence = (4U * sum + differences - 1U) / differences;
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
/* How far reading d still sits at the start level lo of a segment that ends
 * at level hi: 1024 = still at lo, 0 = already at hi. */
static uint32_t rec_frac (unsigned int value_d, unsigned int value_lo,
                          unsigned int value_hi)
{
    int32_t num = (int32_t)value_d - (int32_t)value_hi,
            den = (int32_t)value_lo - (int32_t)value_hi;
    /* 10-bit samples bound num*1024 to +/-1,047,552. Keep this exact
     * operation 32-bit: RP2040 otherwise calls a slow 64-bit divide twice
     * per sample while processing the completed recording. */
    int32_t value_f = (den == 0) ? 0 : (num * 1024) / den;
    return (value_f < 0) ? 0U : ((value_f > 1024) ? 1024U : (uint32_t)value_f);
}

/* Group low-contrast ink regions, retaining one strongest peak per region.
 * A bar must return below the noise band for more than 30 ms before another
 * can start. Internal plateau wiggles therefore cannot replace a faint bar.
 * Require exactly 15 complete regions: never prune this path to a desired
 * count, infer a missing bar, or use an expected letter to choose peaks. */
static unsigned int rec_faint_extrema (const s3_barcode_record_t * p_r,
                                       uint16_t p_ext[S3_REC_EXT_MAX])
{
    const unsigned int floor = (unsigned int)p_r->paper + S3_REC_FAINT_BAND;
    unsigned int cursor = 0U, value_n = 1U;
    p_ext[0] = 0U;
    while (cursor < p_r->count)
    {
        unsigned int peak, dip;
        uint32_t low_us = 0U;
        while (cursor < p_r->count && REC_DARK(p_r, cursor) <= floor)
        {
            cursor++;
        }
        if (cursor == p_r->count)
        {
            break;
        }
        if (value_n + 2U > S3_BARCODE_RUNS + 2U)
        {
            return 0U;
        }
        peak = cursor++;
        dip = peak;
        while (cursor < p_r->count)
        {
            if (REC_DARK(p_r, cursor) > floor)
            {
                if (REC_DARK(p_r, cursor) > REC_DARK(p_r, peak))
                {
                    peak = cursor;
                }
                low_us = 0U;
            }
            else
            {
                if (low_us == 0U || REC_DARK(p_r, cursor) < REC_DARK(p_r, dip))
                {
                    dip = cursor;
                }
                low_us += REC_STEP_US(p_r, cursor);
                if (low_us > S3_REC_FAINT_DIP_US)
                {
                    break;
                }
            }
            cursor++;
        }
        if (low_us <= S3_REC_FAINT_DIP_US)
        {
            return 0U;
        } /* incomplete last bar */
        /* Include the entire valley, including its first low readings. */
        while (cursor < p_r->count && REC_DARK(p_r, cursor) <= floor)
        {
            if (REC_DARK(p_r, cursor) < REC_DARK(p_r, dip))
            {
                dip = cursor;
            }
            cursor++;
        }
        p_ext[value_n++] = (uint16_t)peak;
        p_ext[value_n++] = (uint16_t)dip;
    }
    return value_n == S3_BARCODE_RUNS + 2U ? value_n : 0U;
}

static unsigned int rec_extract_runs (const s3_barcode_record_t * p_record,
                                      uint32_t p_runs[S3_BARCODE_RUNS],
                                      uint32_t * p_first_edge_us,
                                      uint32_t * p_last_edge_us,
                                      uint32_t * p_end_us,
                                      uint32_t * p_max_step_us, bool faint)
{
    /* Alternating dip, bump, dip, ... indices; 15 bars need 31 extremes. */
    uint16_t ext[S3_REC_EXT_MAX];
    uint32_t edge_prev = 0U, value_t = 0U;
    unsigned int value_n = 1U, best = 0U, value_i, value_k = 0U, edges = 0U;
    bool rising = true;
    if ((p_record == 0) || (p_runs == 0) || (p_first_edge_us == 0) ||
        (p_last_edge_us == 0) || (p_end_us == 0) || (p_max_step_us == 0) ||
        (p_record->count < 3U) || (p_record->count > S3_REC_MAX))
    {
        return 0U;
    }
    *p_max_step_us = 0U;
    ext[0] = 0U;
    if (faint)
    {
        value_n = rec_faint_extrema(p_record, ext);
        if (value_n == 0U)
        {
            return 0U;
        }
        for (value_i = 1U; value_i < p_record->count; value_i++)
        {
            if (REC_STEP_US(p_record, value_i) > *p_max_step_us)
            {
                *p_max_step_us = REC_STEP_US(p_record, value_i);
            }
        }
    }
    else
    {
        for (value_i = 1U; value_i < p_record->count; value_i++)
        {
            unsigned int value_d = REC_DARK(p_record, value_i),
                         value_b = REC_DARK(p_record, best);
            if (REC_STEP_US(p_record, value_i) > *p_max_step_us)
            {
                *p_max_step_us = REC_STEP_US(p_record, value_i);
            }
            if (rising ? (value_d > value_b) : (value_d < value_b))
            {
                best = value_i;
            }
            else if ((rising ? value_b - value_d : value_d - value_b) >=
                     (p_record->prominence ? p_record->prominence
                                           : S3_REC_PROMINENCE))
            {
                if (value_n >= S3_REC_EXT_MAX - 1U)
                {
                    return 0U;
                } /* far too many stripes */
                ext[value_n++] = (uint16_t)best;
                best = value_i;
                rising = !rising;
            }
        }
        if (!rising)
        {
            ext[value_n++] = (uint16_t)best;
        } /* dip after the last bump */
        if (value_n < 3U)
        {
            return 0U;
        } /* no complete bar */
        /* A sag or scratch inside a wide bar (or a smudge inside a wide gap)
         * adds an extra dip/bump pair. While there are more than 31 extremes,
         * drop the neighbouring interior pair that is "smallest" = height times
         * time apart: a wiggle is shallow or brief, a real stripe is deep and
         * about one narrow module long. The strict Code 39 checks still have to
         * pass afterwards; they reduce false accepts but are not a checksum. */
        while (value_n > S3_BARCODE_RUNS + 2U)
        {
            unsigned int value_j, value_m, cut = 1U;
            uint64_t small = UINT64_MAX;
            for (value_j = 1U; value_j + 2U < value_n; value_j++)
            {
                unsigned int value_a = REC_DARK(p_record, ext[value_j]),
                             value_b = REC_DARK(p_record, ext[value_j + 1U]);
                uint64_t span = 0U, size;
                for (value_m = (unsigned int)ext[value_j] + 1U;
                     value_m <= (unsigned int)ext[value_j + 1U]; value_m++)
                {
                    span += REC_STEP_US(p_record, value_m);
                }
                size = (uint64_t)((value_a > value_b) ? value_a - value_b
                                                      : value_b - value_a) *
                       span;
                if (size < small)
                {
                    small = size;
                    cut = value_j;
                }
            }
            for (value_j = cut; value_j + 2U < value_n; value_j++)
            {
                ext[value_j] = ext[value_j + 2U];
            }
            value_n -= 2U;
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
        uint32_t t_lo = 0U, at_lo = 0U;
        for (value_i = 1U; value_i < p_record->count; value_i++)
        {
            uint32_t step = REC_STEP_US(p_record, value_i);
            value_t += step;
            if (value_k + 1U < value_n)
            {
                unsigned int value_lo = REC_DARK(p_record, ext[value_k]),
                             value_hi = REC_DARK(p_record, ext[value_k + 1U]);
                /* step<=31500, each fraction<=1024: product<=64,512,000.
                 * Across 2048 readings the elapsed sum fits uint32_t too. */
                at_lo += (step * (rec_frac(REC_DARK(p_record, value_i - 1U),
                                           value_lo, value_hi) +
                                  rec_frac(REC_DARK(p_record, value_i),
                                           value_lo, value_hi))) /
                         2048U;
                if (value_i == (unsigned int)ext[value_k + 1U])
                {
                    const uint32_t guess = t_lo + (uint32_t)at_lo;
                    const unsigned int mid = (value_lo + value_hi) / 2U;
                    const bool value_up = value_hi > value_lo;
                    /* Near the noise floor, measure the half-height core
                     * connected to the maximum. An earlier weak shoulder
                     * must not consume the neighbouring white gap. Strong
                     * bars retain the outer crossings used for wide dips. */
                    const bool core =
                        faint && ((value_up ? value_hi : value_lo) -
                                      (unsigned int)p_record->paper <=
                                  S3_REC_RISE);
                    uint32_t edge = guess, value_tj = t_lo;
                    bool crossed = false;
                    unsigned int value_j;
                    for (value_j = (unsigned int)ext[value_k];
                         value_j < value_i; value_j++)
                    {
                        unsigned int value_a = REC_DARK(p_record, value_j),
                                     value_b = REC_DARK(p_record, value_j + 1U);
                        uint32_t value_sj = REC_STEP_US(p_record, value_j + 1U);
                        const bool crossing =
                            value_up
                                ? (core ? (value_a <= mid && value_b > mid)
                                        : (value_a < mid && value_b >= mid))
                                : (value_a > mid && value_b <= mid);
                        if (crossing)
                        {
                            uint32_t value_x =
                                value_tj +
                                (value_sj * (uint32_t)(value_up
                                                           ? mid - value_a
                                                           : value_a - mid)) /
                                    (uint32_t)(value_up ? value_b - value_a
                                                        : value_a - value_b);
                            if (!crossed || (core ? value_up : !value_up))
                            {
                                edge = value_x;
                            }
                            crossed = true;
                        }
                        value_tj += value_sj;
                    }
                    if (edges == 0U)
                    {
                        *p_first_edge_us = edge;
                    }
                    else
                    {
                        p_runs[edges - 1U] = edge - edge_prev;
                    }
                    edge_prev = edge;
                    edges++;
                    value_k++;
                    t_lo = value_t;
                    at_lo = 0U;
                }
            }
        }
    }
    *p_last_edge_us = edge_prev;
    *p_end_us = value_t;
    return (edges >= 2U) ? edges - 1U : 0U;
}

unsigned int s3_barcode_record_runs (const s3_barcode_record_t * p_record,
                                     uint32_t p_runs[S3_BARCODE_RUNS],
                                     uint32_t * p_first_edge_us,
                                     uint32_t * p_last_edge_us,
                                     uint32_t * p_end_us,
                                     uint32_t * p_max_step_us)
{
    unsigned int value_n =
        rec_extract_runs(p_record, p_runs, p_first_edge_us, p_last_edge_us,
                         p_end_us, p_max_step_us, false);
    if (value_n != S3_BARCODE_RUNS && rec_faint_eligible(p_record))
    {
        value_n =
            rec_extract_runs(p_record, p_runs, p_first_edge_us, p_last_edge_us,
                             p_end_us, p_max_step_us, true);
    }
    return value_n;
}

/* Average of the six smallest of nine runs (as character() computes narrow). */
static uint32_t rec_narrow (const uint32_t * p_runs)
{
    uint32_t value_s[9];
    uint64_t sum = 0U;
    unsigned int value_i, value_j;
    for (value_i = 0U; value_i < 9U; value_i++)
    {
        uint32_t value_v = p_runs[value_i];
        value_j = value_i;
        while ((value_j > 0U) && (value_s[value_j - 1U] > value_v))
        {
            value_s[value_j] = value_s[value_j - 1U];
            value_j--;
        }
        value_s[value_j] = value_v;
    }
    for (value_i = 0U; value_i < 6U; value_i++)
    {
        sum += value_s[value_i];
    }
    return (uint32_t)(sum / 6U);
}

/* All narrow elements share one reference, and all wide elements another.
 * Find the greatest deviation in each group before comparing their ratios.
 * max(floor(x)) == floor(max(x)), so this gives exactly the former score and
 * tie behaviour with one 64-bit division per fit instead of nine. RP2040
 * performs that division in software in the synchronous sampling task. */
static uint32_t rec_fit_error (const uint32_t p_adjusted[9], uint16_t star,
                               uint32_t narrow, uint32_t wide)
{
    uint32_t narrow_delta = 0U, wide_delta = 0U;
    unsigned int value_i;
    for (value_i = 0U; value_i < 9U; value_i++)
    {
        const bool is_wide = (star & (1U << (8U - value_i))) != 0U;
        const uint32_t reference = is_wide ? wide : narrow;
        const uint32_t delta = (p_adjusted[value_i] > reference)
                                   ? p_adjusted[value_i] - reference
                                   : reference - p_adjusted[value_i];
        if (is_wide)
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
    }
    /* character() has established positive references and deviations no
     * greater than 35%, so the returned score is at most 350000. Cross
     * products fit uint64_t even for the full uint32_t input range. */
    if ((uint64_t)wide_delta * narrow > (uint64_t)narrow_delta * wide)
    {
        return (uint32_t)((uint64_t)wide_delta * 1000000U / wide);
    }
    return (uint32_t)((uint64_t)narrow_delta * 1000000U / narrow);
}

/* Fit only the two known sentinels, never the payload. Both star orientations
 * have three narrow bars and three narrow spaces: their narrow mean is
 * invariant under a shared bar expansion / space contraction. Search a fixed
 * 129-point grid within +/- 3/4 of the smaller narrow mean. Minimise the worst
 * fractional sentinel error, retaining the original 35%/ratio/sample checks.
 * A single bias must explain BOTH stars; ambiguous orientation is refused. */
static bool rec_correct_shared_runs (uint32_t p_runs[S3_BARCODE_RUNS],
                                     uint32_t gap,
                                     const s3_barcode_config_t * p_config,
                                     int32_t * p_bias_us,
                                     bool * p_sentinels_matched)
{
    unsigned int orientation, found = 0U, value_i, value_c;
    int32_t chosen = 0;
    if (p_sentinels_matched != 0)
    {
        *p_sentinels_matched = false;
    }
    if ((p_runs == 0) || (p_bias_us == 0) || !config_valid(p_config) ||
        (gap > p_config->max_gap_us))
    {
        return false;
    }
    *p_bias_us = 0;
    for (orientation = 0U; orientation < 2U; orientation++)
    {
        uint16_t star = orientation ? 0x052U : 0x094U;
        uint32_t minimum = UINT32_MAX, best_score = UINT32_MAX;
        int32_t bound, best_bias = 0;
        int tick;
        for (value_c = 0U; value_c < 2U; value_c++)
        {
            uint64_t sum = 0U;
            for (value_i = 0U; value_i < 9U; value_i++)
            {
                if (!(star & (1U << (8U - value_i))))
                {
                    sum += p_runs[value_c * 20U + value_i];
                }
            }
            if (sum / 6U < minimum)
            {
                minimum = (uint32_t)(sum / 6U);
            }
        }
        if ((minimum < p_config->min_narrow_us) ||
            (minimum > p_config->max_run_us))
        {
            continue;
        }
        bound = (int32_t)(minimum * 3U / 4U);
        for (tick = -64; tick <= 64; tick++)
        {
            int32_t bias = (int32_t)((int64_t)bound * tick / 64);
            uint32_t score = 0U;
            bool good = true;
            for (value_c = 0U; (value_c < 2U) && good; value_c++)
            {
                uint32_t adjusted[9], narrow, wide = 0U;
                uint16_t pattern;
                for (value_i = 0U; value_i < 9U; value_i++)
                {
                    int64_t value_v = (int64_t)p_runs[value_c * 20U + value_i] +
                                      ((value_i % 2U) ? -bias : bias);
                    if ((value_v <= 0) || (value_v > p_config->max_run_us))
                    {
                        good = false;
                        break;
                    }
                    adjusted[value_i] = (uint32_t)value_v;
                    if (star & (1U << (8U - value_i)))
                    {
                        wide += adjusted[value_i];
                    }
                }
                if (!good ||
                    !character(adjusted, gap, p_config, &pattern, &narrow) ||
                    (pattern != star))
                {
                    good = false;
                    break;
                }
                wide /= 3U;
                const uint32_t error =
                    rec_fit_error(adjusted, star, narrow, wide);
                if (error > score)
                {
                    score = error;
                }
            }
            if (good && ((score < best_score) ||
                         ((score == best_score) &&
                          ((bias < 0 ? -bias : bias) <
                           (best_bias < 0 ? -best_bias : best_bias)))))
            {
                best_score = score;
                best_bias = bias;
            }
        }
        if (best_score != UINT32_MAX)
        {
            found++;
            chosen = best_bias;
        }
    }
    if (p_sentinels_matched != 0)
    {
        *p_sentinels_matched = (found != 0U);
    }
    if (found != 1U)
    {
        return false;
    }
    /* Validate before modifying, including the payload and inter-character
     * spaces; their patterns play no part in choosing the correction. */
    for (value_i = 0U; value_i < S3_BARCODE_RUNS; value_i++)
    {
        int64_t value_v =
            (int64_t)p_runs[value_i] + ((value_i % 2U) ? -chosen : chosen);
        if ((value_v <= 0) || (value_v > p_config->max_run_us))
        {
            return false;
        }
    }
    for (value_i = 0U; value_i < S3_BARCODE_RUNS; value_i++)
    {
        p_runs[value_i] = (uint32_t)((int64_t)p_runs[value_i] +
                                     ((value_i % 2U) ? -chosen : chosen));
    }
    *p_bias_us = chosen;
    return true;
}

bool s3_barcode_record_correct_runs (uint32_t p_runs[S3_BARCODE_RUNS],
                                     uint32_t gap,
                                     const s3_barcode_config_t * p_config,
                                     int32_t * p_bias_us)
{
    return rec_correct_shared_runs(p_runs, gap, p_config, p_bias_us, 0);
}

/* Single-star fit, using only known sentinel widths. Even-us candidates make
 * the two outer edge shifts exact; no quiet-zone rounding can add margin. */
static bool rec_fit_star (const uint32_t * p_runs, uint16_t star, uint32_t gap,
                          const s3_barcode_config_t * p_config,
                          int32_t * p_bias, uint32_t * p_narrow)
{
    uint64_t sum = 0U;
    uint32_t best_score = UINT32_MAX;
    int32_t bound;
    int tick;
    unsigned int value_i;
    *p_bias = 0;
    for (value_i = 0U; value_i < 9U; value_i++)
    {
        if (!(star & (1U << (8U - value_i))))
        {
            sum += p_runs[value_i];
        }
    }
    *p_narrow = (uint32_t)(sum / 6U);
    if ((*p_narrow < p_config->min_narrow_us) ||
        (*p_narrow > p_config->max_run_us))
    {
        return false;
    }
    bound = (int32_t)(*p_narrow * 3U / 4U);
    for (tick = -64; tick <= 64; tick++)
    {
        int32_t trial = (int32_t)((int64_t)bound * tick / 128) * 2;
        uint32_t adjusted[9], value_n, wide = 0U, score = 0U;
        uint16_t pattern;
        bool good = true;
        for (value_i = 0U; value_i < 9U; value_i++)
        {
            int64_t value_v =
                (int64_t)p_runs[value_i] + ((value_i % 2U) ? -trial : trial);
            if ((value_v <= 0) || (value_v > p_config->max_run_us))
            {
                good = false;
                break;
            }
            adjusted[value_i] = (uint32_t)value_v;
            if (star & (1U << (8U - value_i)))
            {
                wide += adjusted[value_i];
            }
        }
        if (!good || !character(adjusted, gap, p_config, &pattern, &value_n) ||
            (pattern != star))
        {
            continue;
        }
        wide /= 3U;
        score = rec_fit_error(adjusted, star, value_n, wide);
        if ((score < best_score) ||
            ((score == best_score) && ((trial < 0 ? -trial : trial) <
                                       (*p_bias < 0 ? -*p_bias : *p_bias))))
        {
            best_score = score;
            *p_bias = trial;
        }
    }
    return best_score != UINT32_MAX;
}

bool s3_barcode_record_fix_varying (uint32_t p_runs[S3_BARCODE_RUNS],
                                    uint32_t gap,
                                    const s3_barcode_config_t * p_config,
                                    int32_t p_bias_us[2])
{
    uint32_t adjusted[S3_BARCODE_RUNS];
    uint64_t total = 0U, start = 0U, finish = 0U, value_t = 0U;
    int32_t chosen[2] = {0, 0}, previous_shift;
    unsigned int value_i, orientation, found = 0U;
    if ((p_runs == 0) || (p_bias_us == 0) || !config_valid(p_config) ||
        (gap > p_config->max_gap_us))
    {
        return false;
    }
    p_bias_us[0] = 0;
    p_bias_us[1] = 0;
    for (value_i = 0U; value_i < S3_BARCODE_RUNS; value_i++)
    {
        if ((p_runs[value_i] == 0U) || (p_runs[value_i] > p_config->max_run_us))
        {
            return false;
        }
        total += p_runs[value_i];
        if (value_i == 8U)
        {
            start = total;
        } /* edge 9: end of first star */
        if (value_i == 19U)
        {
            finish = total;
        } /* edge 20: start of last star */
    }
    if (total > p_config->max_frame_us)
    {
        return false;
    }
    for (orientation = 0U; orientation < 2U; orientation++)
    {
        uint16_t star = orientation ? 0x052U : 0x094U;
        uint32_t narrow[2], minimum;
        int32_t bias[2], delta;
        if (!rec_fit_star(p_runs, star, gap, p_config, &bias[0], &narrow[0]) ||
            !rec_fit_star(&p_runs[20], star, gap, p_config, &bias[1],
                          &narrow[1]))
        {
            continue;
        }
        /* Two endpoints do not justify arbitrary distortion between them. */
        if (((bias[0] < 0) && (bias[1] > 0)) ||
            ((bias[0] > 0) && (bias[1] < 0)))
        {
            continue;
        }
        minimum = (narrow[0] < narrow[1]) ? narrow[0] : narrow[1];
        delta = bias[1] - bias[0];
        if ((uint32_t)(delta < 0 ? -delta : delta) > minimum / 2U)
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
    previous_shift = -chosen[0] / 2;
    for (value_i = 0U; value_i < S3_BARCODE_RUNS; value_i++)
    {
        int32_t bias, shift;
        int64_t width;
        value_t += p_runs[value_i];
        if (value_t <= start)
        {
            bias = chosen[0];
        }
        else if (value_t >= finish)
        {
            bias = chosen[1];
        }
        else
        {
            /* Weighted interpolation is symmetric on reversing the scan.
             * Use one shift per edge so neighbouring widths stay adjacent. */
            bias = (int32_t)(((int64_t)chosen[0] * (int64_t)(finish - value_t) +
                              (int64_t)chosen[1] * (int64_t)(value_t - start)) /
                             (int64_t)(finish - start));
        }
        shift = (value_i % 2U) ? -bias / 2 : bias / 2;
        width = (int64_t)p_runs[value_i] + shift - previous_shift;
        if ((width <= 0) || (width > p_config->max_run_us))
        {
            return false;
        }
        adjusted[value_i] = (uint32_t)width;
        previous_shift = shift;
    }
    for (value_i = 0U; value_i < S3_BARCODE_RUNS; value_i++)
    {
        p_runs[value_i] = adjusted[value_i];
    }
    p_bias_us[0] = chosen[0];
    p_bias_us[1] = chosen[1];
    return true;
}

/* Validate each independently measured set of edges by the same rules. A
 * count of 29 alone does not prove that the right 15 bumps were selected.
 * Neither extraction nor sentinel fitting uses an expected payload letter. */
#ifdef S3_BARCODE_SYNC_REFERENCE
static bool rec_candidate (const s3_barcode_record_t * r,
                           const s3_barcode_config_t * config, bool faint,
                           s3_barcode_result_t * candidate,
                           uint32_t * required_us)
{
    uint32_t runs[S3_BARCODE_RUNS], first = 0U, last = 0U, end = 0U, gap = 0U;
    uint32_t leading;
    int32_t bias[2] = {0, 0};
    int64_t half, need;
    bool shared_matched = false;
    unsigned int n =
        rec_extract_runs(r, runs, &first, &last, &end, &gap, faint);
    if (n != S3_BARCODE_RUNS)
    {
        return false;
    }
    leading = sat_add(r->leading_us, first);
    /* Trailing white is provisional here; the live reader waits for the
     * real margin below before it releases this candidate. */
    if (!s3_barcode_decode_runs(runs, n, leading, 0x7FFFFFFFU, gap, config,
                                candidate))
    {
        if (rec_correct_shared_runs(runs, gap, config, &bias[0],
                                    &shared_matched))
        {
            bias[1] = bias[0];
        }
        else if (shared_matched ||
                 !s3_barcode_record_fix_varying(runs, gap, config, bias))
        {
            return false;
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
        if (!s3_barcode_decode_runs(runs, n, leading, 0x7FFFFFFFU, gap, config,
                                    candidate))
        {
            return false;
        }
    }
    /* Expanding a bar moves its outer edge into the quiet zone too.
     * Round toward requiring more white, never inventing extra margin. */
    half = (bias[1] > 0) ? ((int64_t)bias[1] + 1) / 2 : (int64_t)bias[1] / 2;
    need = (int64_t)last + half +
           (int64_t)config->quiet_modules * rec_narrow(&runs[20]);
    *required_us = (need > 0x7FFFFFFF) ? 0x7FFFFFFFU : (uint32_t)need;
    return true;
}
#endif
enum
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
};

static void work_next (s3_barcode_record_t * p_r)
{
    s3_barcode_work_t * p_w = &p_r->work;
    p_w->cursor++;
    if (p_w->cursor < p_w->frozen_count)
    {
        const uint32_t step = REC_STEP_US(p_r, p_w->cursor);
        p_w->total_us += step;
        if (step > p_w->max_step_us)
        {
            p_w->max_step_us = step;
        }
    }
}

static void work_path_begin (s3_barcode_record_t * p_r, bool faint)
{
    s3_barcode_work_t * p_w = &p_r->work;
    p_w->faint = faint;
    p_w->n = 1U;
    p_w->cursor = 0U;
    p_w->total_us = 0U;
    p_w->max_step_us = 0U;
    p_w->best = 0U;
    p_w->best_us = 0U;
    p_w->rising = true;
    p_w->ext[0] = 0U;
    p_w->ext_us[0] = 0U;
    p_w->strong = false;
    p_w->varying = false;
    p_w->orientation = 0U;
    p_w->found = 0U;
    p_w->chosen[0] = 0;
    p_w->chosen[1] = 0;
    p_w->phase = faint ? RW_FAINT_WHITE : RW_PRIMARY;
    if (!faint)
    {
        work_next(p_r);
    }
}

static void work_begin (s3_barcode_record_t * p_r, bool provisional, bool early)
{
    p_r->work = (s3_barcode_work_t){0};
    p_r->work.frozen_count = p_r->count;
    p_r->work.provisional = provisional;
    p_r->work.early = early;
    work_path_begin(p_r, false);
    p_r->state = S3_REC_PROCESSING;
}

static void work_bad_path (s3_barcode_record_t * p_r)
{
    p_r->work.phase = RW_PATH_END;
}

static void work_keep_candidate (s3_barcode_record_t * p_r,
                                 const s3_barcode_config_t * p_config,
                                 const s3_barcode_result_t * p_candidate)
{
    s3_barcode_work_t * p_w = &p_r->work;
    const int32_t bias = p_w->chosen[1];
    const int64_t half = bias > 0 ? ((int64_t)bias + 1) / 2 : (int64_t)bias / 2;
    const uint32_t quiet = p_config->quiet_modules * rec_narrow(&p_w->runs[20]);
    const int64_t end = (int64_t)p_w->last_us + half + quiet;
    const uint32_t required = end > 0x7FFFFFFF ? 0x7FFFFFFFU : (uint32_t)end;
    if (p_w->good && ((p_w->candidate.letter_code !=
                       (uint8_t)(p_candidate->letter - 'A' + 1)) ||
                      (p_w->candidate.reversed != p_candidate->reversed)))
    {
        p_w->ambiguous = true;
    }
    if (!p_w->faint && p_w->strong)
    {
        p_w->allow_noise = true;
    }
    if (!p_w->good || required > p_w->required_us)
    {
        p_w->required_us = required;
    }
    if (!p_w->good || quiet > p_w->quiet_required_us)
    {
        p_w->quiet_required_us = quiet;
    }
    p_w->candidate = (s3_barcode_stored_result_t){
        (uint8_t)(p_candidate->letter - 'A' + 1), p_candidate->reversed};
    p_w->good = true;
    p_w->phase = RW_PATH_END;
}

/* Evaluate one unchanged grid position. The outer grid is resumed next tick;
 * every inner loop here is fixed at at most two nine-element sentinels. */
static uint32_t work_fit_trial (s3_barcode_record_t * p_r,
                                const s3_barcode_config_t * p_config)
{
    s3_barcode_work_t * p_w = &p_r->work;
    const uint16_t star = p_w->orientation ? 0x052U : 0x094U;
    const int32_t bias =
        p_w->varying ? (int32_t)((int64_t)p_w->bound * p_w->tick / 128) * 2
                     : (int32_t)((int64_t)p_w->bound * p_w->tick / 64);
    uint32_t score = 0U;
    const unsigned int begin = p_w->varying ? p_w->star_index : 0U;
    const unsigned int end = p_w->varying ? begin + 1U : 2U;
    for (unsigned int value_c = begin; value_c < end; value_c++)
    {
        uint32_t adjusted[9], narrow, wide = 0U;
        uint16_t pattern;
        for (unsigned int value_i = 0U; value_i < 9U; value_i++)
        {
            const int64_t value_v =
                (int64_t)p_w->runs[value_c * 20U + value_i] +
                ((value_i % 2U) ? -bias : bias);
            if (value_v <= 0 || value_v > p_config->max_run_us)
            {
                return UINT32_MAX;
            }
            adjusted[value_i] = (uint32_t)value_v;
            if (star & (1U << (8U - value_i)))
            {
                wide += adjusted[value_i];
            }
        }
        if (!character(adjusted, p_w->max_step_us, p_config, &pattern,
                       &narrow) ||
            pattern != star)
        {
            return UINT32_MAX;
        }
        const uint32_t error = rec_fit_error(adjusted, star, narrow, wide / 3U);
        if (error > score)
        {
            score = error;
        }
    }
    if (score < p_w->best_score ||
        (score == p_w->best_score &&
         (bias < 0 ? -bias : bias) <
             (p_w->best_bias < 0 ? -p_w->best_bias : p_w->best_bias)))
    {
        p_w->best_score = score;
        p_w->best_bias = bias;
    }
    return score;
}

static bool work_apply (s3_barcode_work_t * p_w,
                        const s3_barcode_config_t * p_config)
{
    uint64_t total = 0U, start = 0U, finish = 0U, value_t = 0U;
    int32_t previous = -p_w->chosen[0] / 2;
    if (p_w->varying)
    {
        for (unsigned int value_i = 0U; value_i < S3_BARCODE_RUNS; value_i++)
        {
            if (!p_w->runs[value_i] ||
                p_w->runs[value_i] > p_config->max_run_us)
            {
                return false;
            }
            total += p_w->runs[value_i];
            if (value_i == 8U)
            {
                start = total;
            }
            if (value_i == 19U)
            {
                finish = total;
            }
        }
        if (total > p_config->max_frame_us)
        {
            return false;
        }
    }
    /* Fixed 29 widths; at most eleven interpolated edge divisions. A failed
     * chosen model is final, so partially written scratch is never retried. */
    for (unsigned int value_i = 0U; value_i < S3_BARCODE_RUNS; value_i++)
    {
        int64_t width;
        if (!p_w->varying)
        {
            width = (int64_t)p_w->runs[value_i] +
                    ((value_i % 2U) ? -p_w->chosen[0] : p_w->chosen[0]);
        }
        else
        {
            int32_t bias, shift;
            value_t += p_w->runs[value_i];
            if (value_t <= start)
            {
                bias = p_w->chosen[0];
            }
            else if (value_t >= finish)
            {
                bias = p_w->chosen[1];
            }
            else
            {
                bias = (int32_t)(((int64_t)p_w->chosen[0] *
                                      (int64_t)(finish - value_t) +
                                  (int64_t)p_w->chosen[1] *
                                      (int64_t)(value_t - start)) /
                                 (int64_t)(finish - start));
            }
            shift = (value_i % 2U) ? -bias / 2 : bias / 2;
            width = (int64_t)p_w->runs[value_i] + shift - previous;
            previous = shift;
        }
        if (width <= 0 || width > p_config->max_run_us)
        {
            return false;
        }
        p_w->runs[value_i] = (uint32_t)width;
    }
    return true;
}

/** @brief Advance one primary extrema scan step.
 * @param p_r Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_primary (s3_barcode_record_t * p_r)
{
    s3_barcode_work_t * p_w = &p_r->work;
    if (p_w->cursor >= p_w->frozen_count)
    {
        if (!p_w->rising)
        {
            p_w->ext[p_w->n] = p_w->best;
            p_w->ext_us[p_w->n++] = p_w->best_us;
        }
        p_w->phase = RW_PRUNE_BEGIN;
        return;
    }
    else
    {
        const unsigned int value_d = REC_DARK(p_r, p_w->cursor),
                           value_b = REC_DARK(p_r, p_w->best);
        if (p_w->rising ? value_d > value_b : value_d < value_b)
        {
            p_w->best = p_w->cursor;
            p_w->best_us = p_w->total_us;
        }
        else if ((p_w->rising ? value_b - value_d : value_d - value_b) >=
                 (p_r->prominence ? p_r->prominence : S3_REC_PROMINENCE))
        {
            if (p_w->n >= S3_REC_EXT_MAX - 1U)
            {
                work_bad_path(p_r);
                return;
            }
            p_w->ext[p_w->n] = p_w->best;
            p_w->ext_us[p_w->n++] = p_w->best_us;
            p_w->best = p_w->cursor;
            p_w->best_us = p_w->total_us;
            p_w->rising = !p_w->rising;
        }
        work_next(p_r);
        return;
    }
}

/** @brief Advance one quiet-background step of faint recovery.
 * @param p_r Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_faint_white (s3_barcode_record_t * p_r)
{
    s3_barcode_work_t * p_w = &p_r->work;
    if (p_w->cursor >= p_w->frozen_count)
    {
        p_w->phase = RW_EDGES_BEGIN;
        return;
    }
    if (REC_DARK(p_r, p_w->cursor) <= p_r->paper + S3_REC_FAINT_BAND)
    {
        work_next(p_r);
        return;
    }
    if (p_w->n + 2U > S3_BARCODE_RUNS + 2U)
    {
        work_bad_path(p_r);
        return;
    }
    p_w->peak = p_w->dip = p_w->cursor;
    p_w->peak_us = p_w->dip_us = p_w->total_us;
    p_w->low_us = 0U;
    work_next(p_r);
    p_w->phase = RW_FAINT_INK;
    return;
}

/** @brief Advance one ink-region step of faint recovery.
 * @param p_r Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_faint_ink (s3_barcode_record_t * p_r)
{
    s3_barcode_work_t * p_w = &p_r->work;
    if (p_w->cursor >= p_w->frozen_count)
    {
        work_bad_path(p_r);
        return;
    }
    if (REC_DARK(p_r, p_w->cursor) > p_r->paper + S3_REC_FAINT_BAND)
    {
        if (REC_DARK(p_r, p_w->cursor) > REC_DARK(p_r, p_w->peak))
        {
            p_w->peak = p_w->cursor;
            p_w->peak_us = p_w->total_us;
        }
        p_w->low_us = 0U;
    }
    else
    {
        if (!p_w->low_us ||
            REC_DARK(p_r, p_w->cursor) < REC_DARK(p_r, p_w->dip))
        {
            p_w->dip = p_w->cursor;
            p_w->dip_us = p_w->total_us;
        }
        p_w->low_us += REC_STEP_US(p_r, p_w->cursor);
        if (p_w->low_us > S3_REC_FAINT_DIP_US)
        {
            p_w->phase = RW_FAINT_VALLEY;
            return;
        }
    }
    work_next(p_r);
    return;
}

/** @brief Advance one valley step of faint recovery.
 * @param p_r Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_faint_valley (s3_barcode_record_t * p_r)
{
    s3_barcode_work_t * p_w = &p_r->work;
    if (p_w->cursor < p_w->frozen_count &&
        REC_DARK(p_r, p_w->cursor) <= p_r->paper + S3_REC_FAINT_BAND)
    {
        if (REC_DARK(p_r, p_w->cursor) < REC_DARK(p_r, p_w->dip))
        {
            p_w->dip = p_w->cursor;
            p_w->dip_us = p_w->total_us;
        }
        work_next(p_r);
        return;
    }
    p_w->ext[p_w->n] = p_w->peak;
    p_w->ext_us[p_w->n++] = p_w->peak_us;
    p_w->ext[p_w->n] = p_w->dip;
    p_w->ext_us[p_w->n++] = p_w->dip_us;
    p_w->phase = RW_FAINT_WHITE;
    return;
}

/** @brief Begin one bounded extrema-pruning iteration.
 * @param p_r Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_prune_begin (s3_barcode_record_t * p_r)
{
    s3_barcode_work_t * p_w = &p_r->work;
    if (p_w->n <= S3_BARCODE_RUNS + 2U)
    {
        p_w->phase = RW_EDGES_BEGIN;
        return;
    }
    p_w->smallest = UINT64_MAX;
    p_w->cut = 1U;
    p_w->scan = 1U;
    p_w->phase = RW_PRUNE_SCAN;
    return;
}

/** @brief Compare one extrema pair while choosing a pruning cut.
 * @param p_r Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_prune_scan (s3_barcode_record_t * p_r)
{
    s3_barcode_work_t * p_w = &p_r->work;
    if (p_w->scan + 2U >= p_w->n)
    {
        p_w->copy = p_w->cut;
        p_w->phase = RW_PRUNE_COPY;
        return;
    }
    {
        const unsigned int value_a = REC_DARK(p_r, p_w->ext[p_w->scan]),
                           value_b = REC_DARK(p_r, p_w->ext[p_w->scan + 1U]);
        const uint64_t size =
            (uint64_t)(value_a > value_b ? value_a - value_b
                                         : value_b - value_a) *
            (p_w->ext_us[p_w->scan + 1U] - p_w->ext_us[p_w->scan]);
        if (size < p_w->smallest)
        {
            p_w->smallest = size;
            p_w->cut = p_w->scan;
        }
        p_w->scan++;
        return;
    }
}

/** @brief Copy one retained extrema entry or complete its pruning cut.
 * @param p_r Recorder owning the retained samples and workspace.
 * @return Nothing. The retained workspace records the next phase.
 * @note The dispatcher has already consumed this step's work unit.
 *       This helper neither loops over phases nor owns sample storage.
 */
static void work_prune_copy (s3_barcode_record_t * p_r)
{
    s3_barcode_work_t * p_w = &p_r->work;
    if (p_w->copy + 2U >= p_w->n)
    {
        p_w->n -= 2U;
        p_w->phase = RW_PRUNE_BEGIN;
        return;
    }
    p_w->ext[p_w->copy] = p_w->ext[p_w->copy + 2U];
    p_w->ext_us[p_w->copy] = p_w->ext_us[p_w->copy + 2U];
    p_w->copy++;
    return;
}

/* Each call performs bounded sample/pruning work and at most 32 fit ticks.
 * No complete-frame extraction or grid search runs synchronously here. */
static void work_advance (s3_barcode_record_t * p_r,
                          const s3_barcode_config_t * p_config)
{
    s3_barcode_work_t * p_w = &p_r->work;
    while (p_r->work_units < S3_REC_WORK_UNITS && p_w->phase != RW_DONE)
    {
        if (p_w->phase == RW_FIT_TICK && p_r->fit_ticks >= S3_REC_FIT_TICKS)
        {
            break;
        }
        p_r->work_units++;
        switch (p_w->phase)
        {
        case RW_PRIMARY:
            work_primary(p_r);
            break;
        case RW_FAINT_WHITE:
            work_faint_white(p_r);
            break;
        case RW_FAINT_INK:
            work_faint_ink(p_r);
            break;
        case RW_FAINT_VALLEY:
            work_faint_valley(p_r);
            break;
        case RW_PRUNE_BEGIN:
            work_prune_begin(p_r);
            break;
        case RW_PRUNE_SCAN:
            work_prune_scan(p_r);
            break;
        case RW_PRUNE_COPY:
            work_prune_copy(p_r);
            break;
        case RW_EDGES_BEGIN:
            if (p_w->n != S3_BARCODE_RUNS + 2U)
            {
                work_bad_path(p_r);
                break;
            }
            p_w->strong = !p_w->faint;
            for (unsigned int value_i = 1U; value_i + 1U < p_w->n;
                 value_i += 2U)
            {
                const unsigned int value_p = REC_DARK(p_r, p_w->ext[value_i]);
                const unsigned int value_a =
                                       REC_DARK(p_r, p_w->ext[value_i - 1U]),
                                   value_b =
                                       REC_DARK(p_r, p_w->ext[value_i + 1U]);
                if (value_p < value_a + S3_QUIET_STRONG_PROMINENCE ||
                    value_p < value_b + S3_QUIET_STRONG_PROMINENCE)
                {
                    p_w->strong = false;
                }
            }
            p_w->segment = 0U;
            p_w->edges = 0U;
            p_w->edge_previous_us = 0U;
            p_w->phase = RW_EDGE_BEGIN;
            break;
        case RW_EDGE_BEGIN:
            if (p_w->segment + 1U >= p_w->n)
            {
                p_w->last_us = p_w->edge_previous_us;
                p_w->phase = RW_DIRECT;
                break;
            }
            if (p_w->ext[p_w->segment + 1U] <= p_w->ext[p_w->segment])
            {
                work_bad_path(p_r);
                break;
            }
            p_w->edge_cursor = p_w->ext[p_w->segment] + 1U;
            p_w->segment_us = p_w->ext_us[p_w->segment];
            p_w->integral_us = 0U;
            p_w->crossed = false;
            p_w->phase = RW_EDGE_SCAN;
            break;
        case RW_EDGE_SCAN:
        {
            const unsigned int value_i = p_w->edge_cursor,
                               value_k = p_w->segment;
            const unsigned int value_lo = REC_DARK(p_r, p_w->ext[value_k]),
                               value_hi = REC_DARK(p_r, p_w->ext[value_k + 1U]);
            const unsigned int value_a = REC_DARK(p_r, value_i - 1U),
                               value_b = REC_DARK(p_r, value_i),
                               mid = (value_lo + value_hi) / 2U;
            const uint32_t step = REC_STEP_US(p_r, value_i);
            const bool value_up = value_hi > value_lo,
                       core = p_w->faint &&
                              (value_up ? value_hi : value_lo) - p_r->paper <=
                                  S3_REC_RISE;
            const bool crossed = value_up
                                     ? (core ? value_a <= mid && value_b > mid
                                             : value_a < mid && value_b >= mid)
                                     : value_a > mid && value_b <= mid;
            p_w->integral_us +=
                (step * (rec_frac(value_a, value_lo, value_hi) +
                         rec_frac(value_b, value_lo, value_hi))) /
                2048U;
            if (crossed)
            {
                const uint32_t edge =
                    p_w->segment_us +
                    step *
                        (uint32_t)(value_up ? mid - value_a : value_a - mid) /
                        (uint32_t)(value_up ? value_b - value_a
                                            : value_a - value_b);
                if (!p_w->crossed || (core ? value_up : !value_up))
                {
                    p_w->edge_us = edge;
                }
                p_w->crossed = true;
            }
            p_w->segment_us += step;
            p_w->edge_cursor++;
            if (value_i == p_w->ext[value_k + 1U])
            {
                if (!p_w->crossed)
                {
                    p_w->edge_us = p_w->ext_us[value_k] + p_w->integral_us;
                }
                if (!p_w->edges)
                {
                    p_w->first_us = p_w->edge_us;
                }
                else
                {
                    p_w->runs[p_w->edges - 1U] =
                        p_w->edge_us - p_w->edge_previous_us;
                }
                p_w->edge_previous_us = p_w->edge_us;
                p_w->edges++;
                p_w->segment++;
                p_w->phase = RW_EDGE_BEGIN;
            }
            break;
        }
        case RW_DIRECT:
        {
            s3_barcode_result_t candidate;
            const uint32_t leading = sat_add(p_r->leading_us, p_w->first_us);
            if (s3_barcode_decode_runs(p_w->runs, S3_BARCODE_RUNS, leading,
                                       0x7FFFFFFFU, p_w->max_step_us, p_config,
                                       &candidate))
            {
                work_keep_candidate(p_r, p_config, &candidate);
                break;
            }
            p_w->orientation = 0U;
            p_w->star_index = 0U;
            p_w->found = 0U;
            p_w->varying = false;
            p_w->phase = RW_FIT_BEGIN;
            break;
        }
        case RW_FIT_BEGIN:
            if (p_w->orientation >= 2U)
            {
                p_w->phase = RW_FIT_END;
                break;
            }
            {
                const uint16_t star = p_w->orientation ? 0x052U : 0x094U;
                const unsigned int begin = p_w->varying ? p_w->star_index : 0U;
                const unsigned int end = p_w->varying ? begin + 1U : 2U;
                p_w->minimum = UINT32_MAX;
                for (unsigned int value_c = begin; value_c < end; value_c++)
                {
                    uint64_t sum = 0U;
                    for (unsigned int value_i = 0U; value_i < 9U; value_i++)
                    {
                        if (!(star & (1U << (8U - value_i))))
                        {
                            sum += p_w->runs[value_c * 20U + value_i];
                        }
                    }
                    p_w->narrow[value_c] = (uint32_t)(sum / 6U);
                    if (p_w->narrow[value_c] < p_w->minimum)
                    {
                        p_w->minimum = p_w->narrow[value_c];
                    }
                }
                if (p_w->minimum < p_config->min_narrow_us ||
                    p_w->minimum > p_config->max_run_us)
                {
                    p_w->orientation++;
                    p_w->star_index = 0U;
                    break;
                }
                p_w->bound = (int32_t)(p_w->minimum * 3U / 4U);
                p_w->best_score = UINT32_MAX;
                p_w->best_bias = 0;
                p_w->tick = -64;
                p_w->phase = RW_FIT_TICK;
                break;
            }
        case RW_FIT_TICK:
            (void)work_fit_trial(p_r, p_config);
            p_r->fit_ticks++;
            p_w->tick++;
            if (p_w->tick <= 64)
            {
                break;
            }
            if (!p_w->varying)
            {
                if (p_w->best_score != UINT32_MAX)
                {
                    p_w->found++;
                    p_w->chosen[0] = p_w->chosen[1] = p_w->best_bias;
                }
                p_w->orientation++;
            }
            else if (p_w->best_score == UINT32_MAX)
            {
                p_w->orientation++;
                p_w->star_index = 0U;
            }
            else
            {
                p_w->trial_bias[p_w->star_index] = p_w->best_bias;
                if (!p_w->star_index)
                {
                    p_w->star_index = 1U;
                }
                else
                {
                    const int32_t value_a = p_w->trial_bias[0],
                                  value_b = p_w->trial_bias[1],
                                  delta = value_b - value_a;
                    const uint32_t minimum = p_w->narrow[0] < p_w->narrow[1]
                                                 ? p_w->narrow[0]
                                                 : p_w->narrow[1];
                    if (!((value_a < 0 && value_b > 0) ||
                          (value_a > 0 && value_b < 0)) &&
                        (uint32_t)(delta < 0 ? -delta : delta) <= minimum / 2U)
                    {
                        p_w->chosen[0] = value_a;
                        p_w->chosen[1] = value_b;
                        p_w->found++;
                    }
                    p_w->orientation++;
                    p_w->star_index = 0U;
                }
            }
            p_w->phase = RW_FIT_BEGIN;
            break;
        case RW_FIT_END:
            if (p_w->found == 1U)
            {
                p_w->phase = RW_APPLY;
                break;
            }
            if (!p_w->varying && !p_w->found)
            {
                p_w->varying = true;
                p_w->orientation = 0U;
                p_w->star_index = 0U;
                p_w->phase = RW_FIT_BEGIN;
                break;
            }
            work_bad_path(p_r);
            break;
        case RW_APPLY:
            p_w->phase = work_apply(p_w, p_config) ? RW_VALIDATE : RW_PATH_END;
            break;
        case RW_VALIDATE:
        {
            s3_barcode_result_t candidate;
            const int64_t half = p_w->chosen[0] > 0
                                     ? ((int64_t)p_w->chosen[0] + 1) / 2
                                     : (int64_t)p_w->chosen[0] / 2;
            const int64_t leading =
                (int64_t)sat_add(p_r->leading_us, p_w->first_us) - half;
            if (leading >= 0 &&
                s3_barcode_decode_runs(p_w->runs, S3_BARCODE_RUNS,
                                       (uint32_t)leading, 0x7FFFFFFFU,
                                       p_w->max_step_us, p_config, &candidate))
            {
                work_keep_candidate(p_r, p_config, &candidate);
            }
            else
            {
                work_bad_path(p_r);
            }
            break;
        }
        case RW_PATH_END:
            if (!p_w->faint && rec_faint_eligible(p_r))
            {
                work_path_begin(p_r, true);
            }
            else
            {
                p_w->phase = RW_DONE;
            }
            break;
        default:
            p_w->phase = RW_DONE;
            p_w->ambiguous = true;
            break;
        }
    }
}

static s3_barcode_status_t rec_finish (s3_barcode_record_t * p_r,
                                       s3_barcode_status_t status)
{
    uint16_t paper = p_r->paper, noise = p_r->noise_q4;
    bool seen = p_r->seen;
    uint32_t last = p_r->last_us;
    const uint16_t units = p_r->work_units, ticks = p_r->fit_ticks;
    s3_barcode_record_reset(p_r);
    p_r->seen = seen;
    p_r->paper = paper;
    p_r->last_us = last; /* keep the white reference */
    p_r->noise_q4 = noise;
    p_r->prev = paper;
    p_r->work_units = units;
    p_r->fit_ticks = ticks;
    /* Like s3_barcode_update: after a failure, only a new white-to-dark
     * start can begin the next recording (no second report mid-bar). */
    p_r->blocked = (status != S3_BARCODE_DECODED) ? 1U : 0U;
    return status;
}

s3_barcode_status_t
s3_barcode_record_update (s3_barcode_record_t * p_r,
                          const s3_barcode_config_t * p_config,
                          uint32_t timestamp_us, bool valid, uint16_t darkness,
                          s3_barcode_result_t * p_result)
{
    unsigned int dark = (darkness > 1000U) ? 1000U : darkness;
    uint16_t step = 0U;
    if ((p_result == 0) || (p_r == 0) || !config_valid(p_config))
    {
        return S3_BARCODE_NONE;
    }
    p_r->work_units = 0U;
    p_r->fit_ticks = 0U;
    *p_result = (s3_barcode_result_t){0};
    if (!valid)
    {
        s3_barcode_record_reset(p_r);
        return S3_BARCODE_NONE;
    }
    if (!p_r->seen)
    {
        p_r->seen = 1U;
        p_r->paper = (uint16_t)dark;
        p_r->last_us = timestamp_us;
        p_r->prev = (uint16_t)dark;
        rec_pre_push(p_r, dark, 0U);
        return S3_BARCODE_NONE;
    }
    if (!rec_step(p_r, timestamp_us - p_r->last_us, &step) ||
        ((uint32_t)step * S3_REC_STEP_US >
         p_config->max_gap_us + S3_REC_STEP_US))
    {
        bool was = s3_barcode_record_active(p_r);
        s3_barcode_record_reset(p_r);
        p_r->seen = 1U;
        p_r->paper = (uint16_t)dark;
        p_r->last_us = timestamp_us;
        p_r->prev = (uint16_t)dark;
        rec_pre_push(p_r, dark, 0U);
        return was ? S3_BARCODE_TIMING_ERROR : S3_BARCODE_NONE;
    }
    p_r->last_us = timestamp_us;
    if (p_r->state == (uint16_t)S3_REC_WAITING)
    {
        /* Learn the paper: follow lighter readings at once, darker ones slowly.
         */
        if (dark < p_r->paper)
        {
            p_r->paper = (uint16_t)dark;
        }
        else if (dark < (unsigned int)p_r->paper + S3_REC_RISE)
        {
            p_r->paper = (uint16_t)(p_r->paper + ((dark - p_r->paper) >> 5));
        } /* A starting bar must not raise the paper reference. */
        if (p_r->blocked)
        {
            if (!rec_white(p_r, dark) ||
                (dark >= (unsigned int)p_r->paper + S3_REC_RISE))
            {
                p_r->white_us = 0U;
                return S3_BARCODE_NONE;
            }
            p_r->blocked = 0U;
        }
        if (dark < (unsigned int)p_r->paper + S3_REC_RISE)
        {
            /* Learn noise only near paper, not from the rising bar. Freeze
             * it once recording starts so bars cannot raise their own gate. */
            if ((dark <= (unsigned int)p_r->paper + S3_REC_WHITE_BAND) &&
                (p_r->prev <= (unsigned int)p_r->paper + S3_REC_WHITE_BAND))
            {
                unsigned int delta =
                    (dark > p_r->prev) ? dark - p_r->prev : p_r->prev - dark;
                p_r->noise_q4 =
                    (uint16_t)(p_r->noise_q4 - p_r->noise_q4 / 16U + delta);
            }
            /* Margin before a barcode: everything short of a bar counts, as
             * in s3_barcode_update (the rising slope is never a bar). */
            p_r->white_us =
                sat_add(p_r->white_us, (uint32_t)step * S3_REC_STEP_US);
            p_r->prev = (uint16_t)dark;
            rec_pre_push(p_r, dark, step);
            return S3_BARCODE_NONE;
        }
        /* Trigger on a stronger bar, retaining real earlier readings so a
         * faint opening star/slow edge is not discarded. The history is part
         * of the already-measured leading interval, never extra quiet time. */
        p_r->state = (uint16_t)S3_REC_RECORDING;
        p_r->prominence = (uint16_t)rec_paper_prominence(p_r, p_config);
        p_r->prefix_us = p_r->pre_us;
        if (!rec_faint_eligible(p_r))
        {
            p_r->leading_us = (p_r->white_us >= p_r->prefix_us)
                                  ? p_r->white_us - p_r->prefix_us
                                  : 0U;
        }
        p_r->white_us = 0U;
        p_r->frame_us = p_r->prefix_us;
        p_r->required_us = 0U;
        s3_barcode_quiet_reset(&p_r->quiet);
        p_r->retry_blocked = false;
        p_r->quiet_tolerant = false;
        p_r->early_tried = false;
        p_r->early_candidate = false;
        p_r->count = p_r->pre_count;
        for (unsigned int value_i = 0U; value_i < p_r->pre_count; value_i++)
        {
            p_r->sample[value_i] =
                p_r->pre[((unsigned int)p_r->pre_head + value_i) %
                         S3_REC_PRE_MAX];
        }
        if (p_r->count == 0U)
        {
            p_r->sample[0] = p_r->prev;
            p_r->count = 1U;
        }
        p_r->best = (uint16_t)p_r->prev;
        p_r->falling = 0U;
        p_r->turn_us = 0U;
    }
    if (p_r->count >= S3_REC_MAX)
    {
        return rec_finish(p_r, S3_BARCODE_OVERFLOW);
    }
    p_r->sample[p_r->count++] = (uint16_t)(dark | ((unsigned int)step << 10));
    p_r->frame_us = sat_add(p_r->frame_us, (uint32_t)step * S3_REC_STEP_US);
    /* The batch validator still limits the actual 29 widths to max_frame_us.
     * Leading history must not shorten the existing post-trigger timeout. */
    if (p_r->frame_us - p_r->prefix_us > p_config->max_frame_us)
    {
        return rec_finish(p_r, S3_BARCODE_REJECTED);
    }
    /* Follow bumps and dips as they come: no stripe is longer than
     * max_run_us, so this long without a turn while dark is a line or a
     * patch, not a barcode (s3_barcode_update rejects the same case). */
    p_r->turn_us = sat_add(p_r->turn_us, (uint32_t)step * S3_REC_STEP_US);
    if (p_r->falling ? (dark < p_r->best) : (dark > p_r->best))
    {
        p_r->best = (uint16_t)dark;
    }
    else if ((p_r->falling ? dark - p_r->best : p_r->best - dark) >=
             rec_monitor_band(p_r))
    {
        p_r->best = (uint16_t)dark;
        p_r->falling = p_r->falling ? 0U : 1U;
        p_r->turn_us = 0U;
    }
    if ((p_r->turn_us > p_config->max_run_us) && !rec_white(p_r, dark))
    {
        return rec_finish(p_r, S3_BARCODE_REJECTED);
    }
    const unsigned int quiet = s3_barcode_quiet_update(
        &p_r->quiet, (uint32_t)step * S3_REC_STEP_US, dark, p_r->paper,
        rec_white_band(p_r), rec_faint_eligible(p_r));
    p_r->white_us = p_r->quiet.strict_us;
    if (quiet & S3_QUIET_BREAK)
    {
        p_r->retry_blocked = false;
        p_r->early_tried = false;
    }
    if (p_r->state == S3_REC_PROCESSING)
    {
        if (!(quiet & S3_QUIET_LOW))
        {
            p_r->work.strict_broken = true;
        }
        if (quiet & S3_QUIET_BREAK)
        {
            /* A tentative or early interval may be an internal white gap.
             * Keep those real samples for a later complete interpretation. */
            if (p_r->work.provisional || p_r->work.early)
            {
                p_r->work = (s3_barcode_work_t){0};
                p_r->state = S3_REC_RECORDING;
            }
            else
            {
                return rec_finish(p_r, S3_BARCODE_REJECTED);
            }
        }
    }
    if (p_r->state == S3_REC_QUIET && !(quiet & S3_QUIET_LOW))
    {
        if (!p_r->quiet_tolerant || (quiet & S3_QUIET_BREAK))
        {
            if (!p_r->early_candidate)
            {
                return rec_finish(p_r, S3_BARCODE_REJECTED);
            }
            /* The early fit still lacked its full measured quiet margin.
             * Continuing ink cancels it without discarding this recording. */
            p_r->candidate = (s3_barcode_stored_result_t){0};
            p_r->work = (s3_barcode_work_t){0};
            p_r->early_candidate = false;
            p_r->required_us = 0U;
            p_r->quiet_required_us = 0U;
            p_r->quiet_tolerant = false;
            p_r->state = S3_REC_RECORDING;
        }
        return S3_BARCODE_NONE; /* never release during an unresolved pulse */
    }
    if (p_r->state == S3_REC_RECORDING)
    {
        const bool strict = p_r->quiet.strict_us >= p_config->max_run_us;
        const bool tentative = p_r->quiet.tolerant_us >= p_config->max_run_us;
        const uint32_t earliest =
            p_config->quiet_modules * p_config->min_narrow_us;
        const bool early =
            !p_r->early_tried && p_r->quiet.strict_us >= earliest;
        if (!(quiet & S3_QUIET_LOW))
        {
            return S3_BARCODE_NONE;
        }
        if (strict)
        {
            work_begin(p_r, false, false);
        }
        else if (tentative && !p_r->retry_blocked)
        {
            work_begin(p_r, true, false);
        }
        else if (early)
        {
            /* One bounded early attempt per real-ink/quiet interval. Failed
             * partial frames still get the original final max_run_us try. */
            p_r->early_tried = true;
            work_begin(p_r, false, true);
        }
        else
        {
            return S3_BARCODE_NONE;
        }
    }
    if (p_r->state == S3_REC_PROCESSING)
    {
        work_advance(p_r, p_config);
        if (p_r->work.phase != RW_DONE)
        {
            return S3_BARCODE_NONE;
        }
        const bool usable =
            p_r->work.good &&
            (p_r->work.allow_noise ||
             (!p_r->work.provisional && !p_r->work.strict_broken));
        if (p_r->work.ambiguous || !usable)
        {
            if (p_r->work.early || p_r->work.provisional ||
                p_r->work.strict_broken)
            {
                p_r->retry_blocked = true;
                p_r->state = S3_REC_RECORDING;
                return S3_BARCODE_NONE;
            }
            return rec_finish(p_r, S3_BARCODE_REJECTED);
        }
        p_r->candidate = p_r->work.candidate;
        p_r->required_us = p_r->work.required_us;
        p_r->quiet_required_us = p_r->work.quiet_required_us;
        p_r->early_candidate = p_r->work.early;
        p_r->quiet_tolerant = p_r->work.allow_noise;
        p_r->state = S3_REC_QUIET;
    }
    const uint32_t qualified =
        p_r->quiet_tolerant ? p_r->quiet.tolerant_us : p_r->quiet.strict_us;
    /* Require a fresh complete quiet interval as well as corrected edge time.
     * A cancelled job must never borrow white from before later real ink. */
    if (!(quiet & S3_QUIET_LOW) || p_r->frame_us < p_r->required_us ||
        qualified < p_r->quiet_required_us)
    {
        return S3_BARCODE_NONE;
    }
    p_result->letter = (char)('A' + p_r->candidate.letter_code - 1U);
    p_result->reversed = p_r->candidate.reversed;
    p_result->timestamp_us = timestamp_us;
    return rec_finish(p_r, S3_BARCODE_DECODED);
}

/* End of file. */
