/** @file barcode_work_test.c
 * @brief Characterize private barcode phase boundaries before decomposition.
 *
 * Host-only exception: include the subject implementation to seed private
 * phase boundaries without adding production entry points. Compile this same
 * candidate fixture with -I SUBJECT for both revisions. Synthetic scratch
 * states are bounded, but are not all claimed reachable through the public
 * recorder. Their outcomes are observations, not new decoder requirements.
 */
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "app_program/subsystem3/bc_decoder.c"

static s3_barcode_record_t g_seed;
static s3_barcode_record_t g_record;
static s3_barcode_record_t g_before;
static unsigned int g_calls;
static unsigned int g_boundaries;

static void fill_runs (s3_barcode_work_t * p_work, bool reversed)
{
    const uint16_t patterns[3] = {0x094U, 0x109U, 0x094U};
    unsigned int at = 0U;
    for (unsigned int symbol = 0U; symbol < 3U; symbol++)
    {
        for (unsigned int bit = 0U; bit < 9U; bit++)
        {
            p_work->runs[at++] =
                (patterns[symbol] & (1U << (8U - bit))) ? 60000U : 30000U;
        }
        if (symbol != 2U)
        {
            p_work->runs[at++] = 30000U;
        }
    }
    assert(at == S3_BARCODE_RUNS);
    if (reversed)
    {
        for (unsigned int i = 0U; i < S3_BARCODE_RUNS / 2U; i++)
        {
            const uint32_t saved = p_work->runs[i];
            p_work->runs[i] = p_work->runs[S3_BARCODE_RUNS - 1U - i];
            p_work->runs[S3_BARCODE_RUNS - 1U - i] = saved;
        }
    }
}

static void seed_boundary (uint8_t phase)
{
    memset(&g_seed, 0, sizeof(g_seed));
    g_seed.count = 160U;
    g_seed.state = S3_REC_PROCESSING;
    g_seed.leading_us = 300000U;
    g_seed.prominence = S3_REC_PROMINENCE;
    for (unsigned int i = 0U; i < S3_REC_MAX; i++)
    {
        g_seed.sample[i] = (uint16_t)((10U << 10) | ((i % 2U) ? 1000U : 0U));
    }
    s3_barcode_work_t * p_work = &g_seed.work;
    for (unsigned int i = 0U; i < S3_REC_EXT_MAX; i++)
    {
        p_work->ext[i] = (uint16_t)i;
        p_work->ext_us[i] = i * 5000U;
    }
    fill_runs(p_work, false);
    p_work->phase = phase;
    p_work->frozen_count = g_seed.count;
    p_work->cursor = 2U;
    p_work->n = S3_BARCODE_RUNS + 2U;
    p_work->best = 1U;
    p_work->peak = 1U;
    p_work->dip = 2U;
    p_work->scan = 1U;
    p_work->cut = 1U;
    p_work->copy = 1U;
    p_work->edge_cursor = 1U;
    p_work->total_us = 10000U;
    p_work->best_us = 5000U;
    p_work->peak_us = 5000U;
    p_work->dip_us = 10000U;
    p_work->first_us = 300000U;
    p_work->last_us = 1440000U;
    p_work->max_step_us = 5000U;
    p_work->minimum = 30000U;
    p_work->bound = 22500;
    p_work->best_score = UINT32_MAX;
    p_work->narrow[0] = 30000U;
    p_work->narrow[1] = 30000U;
    p_work->smallest = UINT64_MAX;
    p_work->rising = true;
    p_work->strong = true;
}

/* Named fields exclude unspecified struct padding. All scratch slots, even
 * currently unused slots, are emitted so partial writes remain observable. */
static void trace (const char * p_name, unsigned int variant,
                   const char * p_stage, const s3_barcode_config_t * p_config)
{
    const s3_barcode_work_t * p_work = &g_record.work;
    printf("BWORK %u %s %u %s cfg=%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
           ",%u record=%u,%u,%u,%u,%" PRIu32 ",%u,%u",
           g_calls, p_name, variant, p_stage, p_config->max_gap_us,
           p_config->min_narrow_us, p_config->max_run_us,
           p_config->max_frame_us, (unsigned int)p_config->quiet_modules,
           (unsigned int)g_record.count, (unsigned int)g_record.state,
           (unsigned int)g_record.paper, (unsigned int)g_record.prominence,
           g_record.leading_us, (unsigned int)g_record.work_units,
           (unsigned int)g_record.fit_ticks);
#define TRACE_U(member)                                                        \
    printf(" " #member "=%" PRIu32, (uint32_t)p_work->member)
#define TRACE_S(member) printf(" " #member "=%" PRId32, (int32_t)p_work->member)
    TRACE_U(total_us);
    TRACE_U(best_us);
    TRACE_U(peak_us);
    TRACE_U(dip_us);
    TRACE_U(low_us);
    TRACE_U(first_us);
    TRACE_U(last_us);
    TRACE_U(max_step_us);
    TRACE_U(edge_us);
    TRACE_U(integral_us);
    TRACE_U(segment_us);
    TRACE_U(edge_previous_us);
    TRACE_U(minimum);
    TRACE_U(best_score);
    TRACE_U(required_us);
    TRACE_U(quiet_required_us);
    printf(" smallest=%" PRIu64, p_work->smallest);
    TRACE_S(bound);
    TRACE_S(best_bias);
    TRACE_U(frozen_count);
    TRACE_U(cursor);
    TRACE_U(n);
    TRACE_U(best);
    TRACE_U(peak);
    TRACE_U(dip);
    TRACE_U(scan);
    TRACE_U(cut);
    TRACE_U(copy);
    TRACE_U(segment);
    TRACE_U(edge_cursor);
    TRACE_U(edges);
    TRACE_S(tick);
    TRACE_U(phase);
    TRACE_U(orientation);
    TRACE_U(star_index);
    TRACE_U(found);
    TRACE_U(faint);
    TRACE_U(rising);
    TRACE_U(crossed);
    TRACE_U(strong);
    TRACE_U(varying);
    TRACE_U(good);
    TRACE_U(allow_noise);
    TRACE_U(ambiguous);
    TRACE_U(provisional);
    TRACE_U(strict_broken);
    TRACE_U(early);
    TRACE_U(candidate.letter_code);
    TRACE_U(candidate.reversed);
#undef TRACE_S
#undef TRACE_U
    printf(" ext=");
    for (unsigned int i = 0U; i < S3_REC_EXT_MAX; i++)
    {
        printf("%s%u", i ? "," : "", (unsigned int)p_work->ext[i]);
    }
    printf(" ext_us=");
    for (unsigned int i = 0U; i < S3_REC_EXT_MAX; i++)
    {
        printf("%s%" PRIu32, i ? "," : "", p_work->ext_us[i]);
    }
    printf(" runs=");
    for (unsigned int i = 0U; i < S3_BARCODE_RUNS; i++)
    {
        printf("%s%" PRIu32, i ? "," : "", p_work->runs[i]);
    }
    printf(" narrow=%" PRIu32 ",%" PRIu32 " chosen=%" PRId32 ",%" PRId32
           " trial_bias=%" PRId32 ",%" PRId32 "\n",
           p_work->narrow[0], p_work->narrow[1], p_work->chosen[0],
           p_work->chosen[1], p_work->trial_bias[0], p_work->trial_bias[1]);
}

static void advance (const char * p_name, unsigned int variant,
                     const s3_barcode_config_t * p_config)
{
    memcpy(&g_before, &g_record, sizeof(g_before));
    g_calls++;
    trace(p_name, variant, "before", p_config);
    work_advance(&g_record, p_config);
    trace(p_name, variant, "after", p_config);
    assert(g_record.work_units >= g_before.work_units);
    assert(g_record.work_units <= S3_REC_WORK_UNITS);
    assert(g_record.fit_ticks >= g_before.fit_ticks);
    assert(g_record.fit_ticks <= S3_REC_FIT_TICKS);
    assert((unsigned int)(g_record.fit_ticks - g_before.fit_ticks) <=
           (unsigned int)(g_record.work_units - g_before.work_units));
    if (g_before.work_units == S3_REC_WORK_UNITS ||
        g_before.work.phase == RW_DONE ||
        (g_before.work.phase == RW_FIT_TICK &&
         g_before.fit_ticks == S3_REC_FIT_TICKS))
    {
        assert(memcmp(&g_before, &g_record, sizeof(g_record)) == 0);
    }
    else if (g_before.work_units == S3_REC_WORK_UNITS - 1U)
    {
        assert(g_record.work_units == S3_REC_WORK_UNITS);
    }
    /* work_advance owns only the workspace and these two budget counters. */
    memcpy(&g_before.work, &g_record.work, sizeof(g_before.work));
    g_before.work_units = g_record.work_units;
    g_before.fit_ticks = g_record.fit_ticks;
    assert(memcmp(&g_before, &g_record, sizeof(g_record)) == 0);
}

static void boundary (const char * p_name, const s3_barcode_config_t * p_config)
{
    static const uint16_t work_budgets[4] = {127U, 128U, 127U, 127U};
    static const uint16_t fit_budgets[4] = {0U, 0U, 31U, 32U};
    assert(config_valid(p_config));
    g_boundaries++;
    for (unsigned int i = 0U; i < 4U; i++)
    {
        memcpy(&g_record, &g_seed, sizeof(g_record));
        g_record.work_units = work_budgets[i];
        g_record.fit_ticks = fit_budgets[i];
        advance(p_name, i, p_config);
    }
}

/* These labels describe input states, not asserted output interpretations. */
static void phase_boundaries (void)
{
    s3_barcode_config_t config = s3_barcode_defaults();
#define CASE(name, phase_value, setup)                                         \
    do                                                                         \
    {                                                                          \
        seed_boundary(phase_value);                                            \
        setup;                                                                 \
        boundary(name, &config);                                               \
    } while (0)
    CASE("primary-rise", RW_PRIMARY, g_seed.work.cursor = 3U);
    CASE("primary-turn", RW_PRIMARY, (void)0);
    CASE("primary-falling-low", RW_PRIMARY, g_seed.work.rising = false);
    CASE("primary-falling-turn", RW_PRIMARY, g_seed.work.rising = false;
         g_seed.work.best = 0U; g_seed.work.cursor = 1U);
    CASE("primary-ext-capacity", RW_PRIMARY,
         g_seed.work.n = S3_REC_EXT_MAX - 1U);
    CASE("primary-eof-rising", RW_PRIMARY,
         g_seed.work.cursor = g_seed.work.frozen_count);
    CASE("primary-eof-falling", RW_PRIMARY,
         g_seed.work.cursor = g_seed.work.frozen_count;
         g_seed.work.rising = false);
    CASE("primary-default-prominence", RW_PRIMARY, g_seed.prominence = 0U);
    CASE("white-eof", RW_FAINT_WHITE,
         g_seed.work.cursor = g_seed.work.frozen_count);
    CASE("white-quiet", RW_FAINT_WHITE, (void)0);
    CASE("white-ink", RW_FAINT_WHITE, g_seed.work.cursor = 3U);
    CASE("white-ext-capacity", RW_FAINT_WHITE, g_seed.work.cursor = 3U;
         g_seed.work.n = 30U);
    CASE("ink-eof", RW_FAINT_INK,
         g_seed.work.cursor = g_seed.work.frozen_count);
    CASE("ink-new-peak", RW_FAINT_INK, g_seed.work.cursor = 3U;
         g_seed.work.peak = 2U; g_seed.work.low_us = 10000U);
    CASE("ink-equal-peak", RW_FAINT_INK, g_seed.work.cursor = 3U;
         g_seed.work.low_us = 10000U);
    CASE("ink-first-low", RW_FAINT_INK, (void)0);
    CASE("ink-dip-exact-limit", RW_FAINT_INK, g_seed.work.low_us = 25000U);
    CASE("ink-dip-over-limit", RW_FAINT_INK, g_seed.work.low_us = 30000U);
    CASE("valley-quiet", RW_FAINT_VALLEY, (void)0);
    CASE("valley-lower-dip", RW_FAINT_VALLEY, g_seed.work.dip = 1U);
    CASE("valley-next-ink", RW_FAINT_VALLEY, g_seed.work.cursor = 3U;
         g_seed.work.n = 1U);
    CASE("valley-eof", RW_FAINT_VALLEY,
         g_seed.work.cursor = g_seed.work.frozen_count;
         g_seed.work.n = 1U);
    CASE("prune-exact-count", RW_PRUNE_BEGIN, (void)0);
    CASE("prune-excess", RW_PRUNE_BEGIN, g_seed.work.n = 33U);
    CASE("prune-first-minimum", RW_PRUNE_SCAN, (void)0);
    CASE("prune-tied-minimum", RW_PRUNE_SCAN, g_seed.work.smallest = 5000000U);
    CASE("prune-larger-area", RW_PRUNE_SCAN, g_seed.work.smallest = 1U);
    CASE("prune-scan-end", RW_PRUNE_SCAN, g_seed.work.scan = 29U);
    CASE("prune-copy-entry", RW_PRUNE_COPY, (void)0);
    CASE("prune-copy-end", RW_PRUNE_COPY, g_seed.work.copy = 29U);
    CASE("edges-strong", RW_EDGES_BEGIN, (void)0);
    CASE("edges-weak", RW_EDGES_BEGIN, g_seed.sample[1] = (10U << 10) | 5U);
    CASE("edges-faint", RW_EDGES_BEGIN, g_seed.work.faint = true);
    CASE("edges-wrong-count", RW_EDGES_BEGIN, g_seed.work.n = 30U);
    CASE("edge-start", RW_EDGE_BEGIN, (void)0);
    CASE("edge-equal-extrema", RW_EDGE_BEGIN, g_seed.work.ext[1] = 0U);
    CASE("edge-end", RW_EDGE_BEGIN, g_seed.work.segment = 30U;
         g_seed.work.edge_previous_us = 12345U);
    CASE("edge-rising-cross", RW_EDGE_SCAN, (void)0);
    CASE("edge-falling-cross", RW_EDGE_SCAN, g_seed.work.segment = 1U;
         g_seed.work.edge_cursor = 2U; g_seed.work.edges = 1U;
         g_seed.work.edge_previous_us = 2500U; g_seed.work.segment_us = 5000U);
    CASE("edge-core-midpoint", RW_EDGE_SCAN, g_seed.work.faint = true;
         g_seed.sample[0] = (10U << 10) | 5U;
         g_seed.sample[1] = (10U << 10) | 10U);
    CASE("edge-no-crossing", RW_EDGE_SCAN, g_seed.sample[1] = g_seed.sample[0]);
    CASE("edge-intermediate", RW_EDGE_SCAN, g_seed.work.ext[1] = 3U;
         g_seed.work.ext_us[1] = 15000U);
    CASE("edge-repeat-outer", RW_EDGE_SCAN, g_seed.work.crossed = true;
         g_seed.work.edge_us = 77U);
    CASE("edge-repeat-core", RW_EDGE_SCAN, g_seed.work.faint = true;
         g_seed.work.crossed = true; g_seed.work.edge_us = 77U;
         g_seed.sample[1] = (10U << 10) | 10U);
    CASE("direct-valid", RW_DIRECT, (void)0);
    CASE("direct-reversed", RW_DIRECT, fill_runs(&g_seed.work, true));
    CASE("direct-zero-width", RW_DIRECT, g_seed.work.runs[0] = 0U);
    CASE("direct-existing-same", RW_DIRECT, g_seed.work.good = true;
         g_seed.work.candidate.letter_code = 1U);
    CASE("direct-conflict", RW_DIRECT, g_seed.work.good = true;
         g_seed.work.candidate.letter_code = 2U);
    CASE("fit-start-shared", RW_FIT_BEGIN, (void)0);
    CASE("fit-start-varying-second", RW_FIT_BEGIN, g_seed.work.varying = true;
         g_seed.work.star_index = 1U);
    CASE("fit-orientations-end", RW_FIT_BEGIN, g_seed.work.orientation = 2U);
    CASE("fit-small-minimum", RW_FIT_BEGIN,
         memset(g_seed.work.runs, 0, sizeof(g_seed.work.runs)));
    CASE(
        "fit-large-minimum", RW_FIT_BEGIN,
        for (unsigned int i = 0U; i < S3_BARCODE_RUNS;
             i++) { g_seed.work.runs[i] = config.max_run_us + 1U; });
    CASE("fit-tick-negative", RW_FIT_TICK, g_seed.work.tick = -64);
    CASE("fit-tick-zero", RW_FIT_TICK, (void)0);
    CASE("fit-tick-63", RW_FIT_TICK, g_seed.work.tick = 63);
    CASE("fit-tick-64-miss", RW_FIT_TICK, g_seed.work.tick = 64);
    CASE("fit-tick-64-hit", RW_FIT_TICK, g_seed.work.tick = 64;
         g_seed.work.best_score = 0U; g_seed.work.best_bias = 2000);
    CASE("fit-varying-first", RW_FIT_TICK, g_seed.work.tick = 64;
         g_seed.work.varying = true; g_seed.work.best_score = 0U;
         g_seed.work.best_bias = 2000);
    CASE("fit-varying-second", RW_FIT_TICK, g_seed.work.tick = 64;
         g_seed.work.varying = true; g_seed.work.star_index = 1U;
         g_seed.work.best_score = 0U; g_seed.work.best_bias = 2000;
         g_seed.work.trial_bias[0] = 4000);
    CASE("fit-varying-sign-conflict", RW_FIT_TICK, g_seed.work.tick = 64;
         g_seed.work.varying = true; g_seed.work.star_index = 1U;
         g_seed.work.best_score = 0U; g_seed.work.best_bias = 2000;
         g_seed.work.trial_bias[0] = -4000);
    CASE("fit-varying-delta-limit", RW_FIT_TICK, g_seed.work.tick = 64;
         g_seed.work.varying = true; g_seed.work.star_index = 1U;
         g_seed.work.best_score = 0U; g_seed.work.best_bias = 15000);
    CASE("fit-varying-delta-over", RW_FIT_TICK, g_seed.work.tick = 64;
         g_seed.work.varying = true; g_seed.work.star_index = 1U;
         g_seed.work.best_score = 0U; g_seed.work.best_bias = 15001);
    CASE("fit-varying-miss", RW_FIT_TICK, g_seed.work.tick = 64;
         g_seed.work.varying = true);
    CASE("fit-end-one", RW_FIT_END, g_seed.work.found = 1U);
    CASE("fit-end-none", RW_FIT_END, (void)0);
    CASE("fit-end-ambiguous", RW_FIT_END, g_seed.work.found = 2U);
    CASE("fit-end-varying-none", RW_FIT_END, g_seed.work.varying = true);
    CASE("apply-unbiased", RW_APPLY, (void)0);
    CASE("apply-positive", RW_APPLY, g_seed.work.chosen[0] = 1000);
    CASE("apply-negative", RW_APPLY, g_seed.work.chosen[0] = -1000);
    CASE("apply-first-width-zero", RW_APPLY, g_seed.work.chosen[0] = -30000);
    CASE("apply-partial-before-zero", RW_APPLY, g_seed.work.chosen[0] = 60000);
    CASE("apply-partial-before-maximum", RW_APPLY, g_seed.work.chosen[0] = 1000;
         g_seed.work.runs[2] = config.max_run_us);
    CASE("apply-varying", RW_APPLY, g_seed.work.varying = true;
         g_seed.work.chosen[0] = 2000; g_seed.work.chosen[1] = 4000);
    CASE("apply-varying-negative-odd", RW_APPLY, g_seed.work.varying = true;
         g_seed.work.chosen[0] = -2001; g_seed.work.chosen[1] = -4001);
    CASE("apply-varying-zero-input", RW_APPLY, g_seed.work.varying = true;
         g_seed.work.runs[20] = 0U);
    CASE("apply-varying-large-input", RW_APPLY, g_seed.work.varying = true;
         g_seed.work.runs[20] = config.max_run_us + 1U);
    CASE(
        "apply-varying-large-frame", RW_APPLY, g_seed.work.varying = true;
        for (unsigned int i = 0U; i < S3_BARCODE_RUNS;
             i++) { g_seed.work.runs[i] = config.max_run_us; });
    CASE("apply-varying-partial", RW_APPLY, g_seed.work.varying = true;
         g_seed.work.chosen[0] = 60000; g_seed.work.chosen[1] = 60000);
    CASE("validate-valid", RW_VALIDATE, (void)0);
    CASE("validate-leading-negative", RW_VALIDATE, g_seed.leading_us = 0U;
         g_seed.work.first_us = 0U; g_seed.work.chosen[0] = 1);
    CASE("validate-odd-negative-bias", RW_VALIDATE, g_seed.work.chosen[0] = -3);
    CASE("validate-invalid-width", RW_VALIDATE, g_seed.work.runs[28] = 0U);
    CASE("path-primary-ineligible", RW_PATH_END, (void)0);
    CASE("path-faint-eligible", RW_PATH_END,
         g_seed.prominence = S3_REC_MIN_PROMINENCE);
    CASE("path-already-faint", RW_PATH_END,
         g_seed.prominence = S3_REC_MIN_PROMINENCE;
         g_seed.work.faint = true);
    CASE("done", RW_DONE, (void)0);
    CASE("unknown-zero", 0U, (void)0);
    CASE("unknown-255", UINT8_MAX, (void)0);
    CASE("primary-time-wrap", RW_PRIMARY,
         g_seed.work.total_us = UINT32_MAX - 1000U);
    CASE("prune-time-wrap", RW_PRUNE_SCAN,
         g_seed.work.ext_us[1] = UINT32_MAX - 1000U;
         g_seed.work.ext_us[2] = 1000U);
    CASE("edge-time-wrap", RW_EDGE_SCAN,
         g_seed.work.segment_us = UINT32_MAX - 1000U);
    CASE("direct-leading-saturation", RW_DIRECT, g_seed.leading_us = UINT32_MAX;
         g_seed.work.first_us = UINT32_MAX);
    CASE("candidate-required-saturation", RW_DIRECT,
         g_seed.work.last_us = UINT32_MAX);
    /* Valid configuration boundary; wide arithmetic remains signed-safe. */
    config.max_run_us = 214748364U;
    config.max_frame_us = 2147483647U;
    CASE(
        "fit-large-valid-config", RW_FIT_BEGIN,
        for (unsigned int i = 0U; i < S3_BARCODE_RUNS;
             i++) { g_seed.work.runs[i] = 200000000U; });
    CASE(
        "apply-large-valid-widths", RW_APPLY, g_seed.work.chosen[0] = 100000000;
        for (unsigned int i = 0U; i < S3_BARCODE_RUNS;
             i++) { g_seed.work.runs[i] = 100000001U; });
#undef CASE
}

static void append_samples (uint16_t darkness, unsigned int count)
{
    for (unsigned int i = 0U; i < count; i++)
    {
        assert(g_record.count < S3_REC_MAX);
        g_record.sample[g_record.count++] = (uint16_t)((10U << 10) | darkness);
    }
}

static void complete_path (unsigned int variant)
{
    const s3_barcode_config_t config = s3_barcode_defaults();
    memset(&g_record, 0, sizeof(g_record));
    fill_runs(&g_record.work, variant == 2U);
    /* Save the input widths before work_begin clears its owned workspace. */
    uint32_t widths[S3_BARCODE_RUNS];
    memcpy(widths, g_record.work.runs, sizeof(widths));
    append_samples(0U, 60U);
    for (unsigned int i = 0U; i < S3_BARCODE_RUNS; i++)
    {
        const bool black = (i % 2U) == 0U;
        const uint16_t darkness =
            !black || variant == 4U ? 0U : (variant == 1U ? 10U : 1000U);
        int32_t width = (int32_t)widths[i];
        if (variant == 3U)
        {
            width += black ? 15000 : -15000;
        }
        append_samples(darkness, (unsigned int)width / 5000U);
    }
    append_samples(0U, 60U);
    g_record.leading_us = 300000U;
    g_record.prominence =
        variant == 1U ? S3_REC_MIN_PROMINENCE : S3_REC_PROMINENCE;
    work_begin(&g_record, variant == 1U, variant == 3U);
    unsigned int calls = 0U;
    while (g_record.work.phase != RW_DONE)
    {
        /* A guard detects nontermination; it does not dictate decoded output.
         */
        assert(calls++ < 4096U);
        g_record.work_units = 0U;
        g_record.fit_ticks = 0U;
        advance("complete-sampled-path", variant, &config);
    }
}

int main (void)
{
    phase_boundaries();
    for (unsigned int variant = 0U; variant < 5U; variant++)
    {
        complete_path(variant);
    }
    fprintf(
        stderr,
        "barcode work: %u boundary scenarios, %u calls, %u full scratch rows\n",
        g_boundaries, g_calls, g_calls * 2U);
    return 0;
}

/* End of file. */

