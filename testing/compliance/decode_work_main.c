/** @file decode_work_main.c
 * @brief Characterize private barcode phase boundaries before decomposition.
 *
 * Use host-only probes for the subject's private phases. The runner derives
 * phase constants from the implementation under test. The synthetic states
 * are bounded and can include states outside the public recorder's paths.
 */
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "decode_work_main.h"
#include "decode_work_phases.h"

#define FIXTURE_CODE39_ELEMENTS (9U)
#define COMPLETE_PATH_VARIANTS (5U)

/* Select the former field only when the shared fixture builds the baseline. */
#ifndef TEST_EXTREMA_COUNT_MEMBER
#define TEST_EXTREMA_COUNT_MEMBER extrema_count
#endif

static bc_decoder_record_t g_seed;
static bc_decoder_record_t g_record;
static bc_decoder_record_t g_before;
static unsigned int        g_calls;
static unsigned int        g_boundaries;

/** @brief Fill the decoder work area with a synthetic A frame, reversing the
 * order of its widths when requested.
 */
static void fill_runs (bc_decoder_work_t * p_work, bool b_is_reversed)
{
    const uint16_t patterns[3] = {0x094U, 0x109U, 0x094U};
    unsigned int   run_index   = 0U;
    for (unsigned int symbol = 0U;
         symbol < (sizeof patterns / sizeof patterns[0]); symbol++)
    {
        for (unsigned int bit = 0U; bit < FIXTURE_CODE39_ELEMENTS; bit++)
        {
            p_work->runs[run_index++] =
                (patterns[symbol] & (1U << (8U - bit))) ? 60000U : 30000U;
        }
        if (symbol != 2U)
        {
            p_work->runs[run_index++] = 30000U;
        }
    }
    assert(S3_BARCODE_RUNS == run_index);
    if (b_is_reversed)
    {
        for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS / 2U;
             run_index++)
        {
            const uint32_t saved = p_work->runs[run_index];
            /* clang-format off */
            p_work->runs[run_index]                        =
                p_work->runs[S3_BARCODE_RUNS - 1U - run_index];
            p_work->runs[S3_BARCODE_RUNS - 1U - run_index] = saved;
            /* clang-format on */
        }
    }
}

/** @brief Prepare a decoder work snapshot at the requested processing phase
 * with known durations, thresholds and fit state.
 */
static void seed_boundary (uint8_t phase)
{
    memset(&g_seed, 0, sizeof(g_seed));
    g_seed.count      = 160U;
    g_seed.state      = S3_REC_PROCESSING;
    g_seed.leading_us = 300000U;
    g_seed.prominence = S3_REC_PROMINENCE;
    for (unsigned int sample_index = 0U; sample_index < S3_REC_MAX;
         sample_index++)
    {
        g_seed.sample[sample_index] =
            (uint16_t)((10U << 10) | ((sample_index % 2U) ? 1000U : 0U));
    }
    bc_decoder_work_t * p_work = &g_seed.work;
    for (unsigned int extremum_index = 0U; extremum_index < S3_REC_EXT_MAX;
         extremum_index++)
    {
        p_work->ext[extremum_index]    = (uint16_t)extremum_index;
        p_work->ext_us[extremum_index] = extremum_index * 5000U;
    }
    fill_runs(p_work, false);
    p_work->phase                     = phase;
    p_work->frozen_count              = g_seed.count;
    p_work->cursor                    = 2U;
    p_work->TEST_EXTREMA_COUNT_MEMBER = S3_BARCODE_RUNS + 2U;
    p_work->best                      = 1U;
    p_work->peak                      = 1U;
    p_work->dip                       = 2U;
    p_work->scan                      = 1U;
    p_work->cut                       = 1U;
    p_work->copy                      = 1U;
    p_work->edge_cursor               = 1U;
    p_work->total_us                  = 10000U;
    p_work->best_us                   = 5000U;
    p_work->peak_us                   = 5000U;
    p_work->dip_us                    = 10000U;
    p_work->first_us                  = 300000U;
    p_work->last_us                   = 1440000U;
    p_work->max_step_us               = 5000U;
    p_work->minimum                   = 30000U;
    p_work->bound                     = 22500;
    p_work->best_score                = UINT32_MAX;
    p_work->narrow[0]                 = 30000U;
    p_work->narrow[1]                 = 30000U;
    p_work->smallest                  = UINT64_MAX;
    p_work->b_is_rising               = true;
    p_work->b_is_strong               = true;
}

/** @brief Print a named unsigned field with the explicit trace width. */
static void trace_unsigned (const char * p_label, uint32_t value)
{
    printf(" %s=%" PRIu32, p_label, value);
}

/** @brief Print a named signed field with the explicit trace width. */
static void trace_signed (const char * p_label, int32_t value)
{
    printf(" %s=%" PRId32, p_label, value);
}

/** @brief Emit every named field and scratch slot so that partial writes
 * remain observable. Native struct padding is excluded.
 */
static void trace (const char * p_name, unsigned int variant,
                   const char * p_stage, const bc_decoder_config_t * p_config)
{
    const bc_decoder_work_t * p_work = &g_record.work;
    printf("BWORK %u %s %u %s cfg=%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
           ",%u record=%u,%u,%u,%u,%" PRIu32 ",%u,%u",
           g_calls, p_name, variant, p_stage, p_config->max_gap_us,
           p_config->min_narrow_us, p_config->max_run_us,
           p_config->max_frame_us, (unsigned int)p_config->quiet_modules,
           (unsigned int)g_record.count, (unsigned int)g_record.state,
           (unsigned int)g_record.paper, (unsigned int)g_record.prominence,
           g_record.leading_us, (unsigned int)g_record.work_units,
           (unsigned int)g_record.fit_ticks);
    trace_unsigned("total_us", (uint32_t)p_work->total_us);
    trace_unsigned("best_us", (uint32_t)p_work->best_us);
    trace_unsigned("peak_us", (uint32_t)p_work->peak_us);
    trace_unsigned("dip_us", (uint32_t)p_work->dip_us);
    trace_unsigned("low_us", (uint32_t)p_work->low_us);
    trace_unsigned("first_us", (uint32_t)p_work->first_us);
    trace_unsigned("last_us", (uint32_t)p_work->last_us);
    trace_unsigned("max_step_us", (uint32_t)p_work->max_step_us);
    trace_unsigned("edge_us", (uint32_t)p_work->edge_us);
    trace_unsigned("integral_us", (uint32_t)p_work->integral_us);
    trace_unsigned("segment_us", (uint32_t)p_work->segment_us);
    trace_unsigned("edge_previous_us", (uint32_t)p_work->edge_previous_us);
    trace_unsigned("minimum", (uint32_t)p_work->minimum);
    trace_unsigned("best_score", (uint32_t)p_work->best_score);
    trace_unsigned("required_us", (uint32_t)p_work->required_us);
    trace_unsigned("quiet_required_us", (uint32_t)p_work->quiet_required_us);
    printf(" smallest=%" PRIu64, p_work->smallest);
    trace_signed("bound", (int32_t)p_work->bound);
    trace_signed("best_bias", (int32_t)p_work->best_bias);
    trace_unsigned("frozen_count", (uint32_t)p_work->frozen_count);
    trace_unsigned("cursor", (uint32_t)p_work->cursor);
    trace_unsigned("n", (uint32_t)p_work->TEST_EXTREMA_COUNT_MEMBER);
    trace_unsigned("best", (uint32_t)p_work->best);
    trace_unsigned("peak", (uint32_t)p_work->peak);
    trace_unsigned("dip", (uint32_t)p_work->dip);
    trace_unsigned("scan", (uint32_t)p_work->scan);
    trace_unsigned("cut", (uint32_t)p_work->cut);
    trace_unsigned("copy", (uint32_t)p_work->copy);
    trace_unsigned("segment", (uint32_t)p_work->segment);
    trace_unsigned("edge_cursor", (uint32_t)p_work->edge_cursor);
    trace_unsigned("edges", (uint32_t)p_work->edges);
    trace_signed("tick", (int32_t)p_work->tick);
    trace_unsigned("phase", (uint32_t)p_work->phase);
    trace_unsigned("orientation", (uint32_t)p_work->orientation);
    trace_unsigned("star_index", (uint32_t)p_work->star_index);
    trace_unsigned("found", (uint32_t)p_work->found);
    trace_unsigned("faint", (uint32_t)p_work->b_is_faint);
    trace_unsigned("rising", (uint32_t)p_work->b_is_rising);
    trace_unsigned("crossed", (uint32_t)p_work->b_has_crossed);
    trace_unsigned("strong", (uint32_t)p_work->b_is_strong);
    trace_unsigned("varying", (uint32_t)p_work->b_does_bias_vary);
    trace_unsigned("good", (uint32_t)p_work->b_has_candidate);
    trace_unsigned("allow_noise", (uint32_t)p_work->b_allows_noise);
    trace_unsigned("ambiguous", (uint32_t)p_work->b_is_ambiguous);
    trace_unsigned("provisional", (uint32_t)p_work->b_is_provisional);
    trace_unsigned("strict_broken", (uint32_t)p_work->b_is_strict_fit_broken);
    trace_unsigned("early", (uint32_t)p_work->b_is_early);
    trace_unsigned("candidate.letter_code",
                   (uint32_t)p_work->candidate.letter_code);
    trace_unsigned("candidate.reversed",
                   (uint32_t)p_work->candidate.b_is_reversed);
    printf(" ext=");
    for (unsigned int extremum_index = 0U; extremum_index < S3_REC_EXT_MAX;
         extremum_index++)
    {
        printf("%s%u", extremum_index ? "," : "",
               (unsigned int)p_work->ext[extremum_index]);
    }
    printf(" ext_us=");
    for (unsigned int extremum_index = 0U; extremum_index < S3_REC_EXT_MAX;
         extremum_index++)
    {
        printf("%s%" PRIu32, extremum_index ? "," : "",
               p_work->ext_us[extremum_index]);
    }
    printf(" runs=");
    for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS; run_index++)
    {
        printf("%s%" PRIu32, run_index ? "," : "", p_work->runs[run_index]);
    }
    printf(" narrow=%" PRIu32 ",%" PRIu32 " chosen=%" PRId32 ",%" PRId32
           " trial_bias=%" PRId32 ",%" PRId32 "\n",
           p_work->narrow[0], p_work->narrow[1], p_work->chosen[0],
           p_work->chosen[1], p_work->trial_bias[0], p_work->trial_bias[1]);
}

/** @brief Trace one decoder work update before and after execution and check
 * the fixture's accounting of retained state.
 */
static void advance (const char * p_name, unsigned int variant,
                     const bc_decoder_config_t * p_config)
{
    memcpy(&g_before, &g_record, sizeof(g_before));
    g_calls++;
    trace(p_name, variant, "before", p_config);
    bc_decoder_boundary_test_advance(&g_record, p_config);
    trace(p_name, variant, "after", p_config);
    assert(g_record.work_units >= g_before.work_units);
    assert(g_record.work_units <= S3_REC_WORK_UNITS);
    assert(g_record.fit_ticks >= g_before.fit_ticks);
    assert(g_record.fit_ticks <= S3_REC_FIT_TICKS);
    assert((unsigned int)(g_record.fit_ticks - g_before.fit_ticks) <=
           (unsigned int)(g_record.work_units - g_before.work_units));
    if (S3_REC_WORK_UNITS == g_before.work_units ||
        RW_DONE == (int)g_before.work.phase ||
        (RW_FIT_TICK == (int)g_before.work.phase &&
         S3_REC_FIT_TICKS == g_before.fit_ticks))
    {
        assert(memcmp(&g_before, &g_record, sizeof(g_record)) == 0);
    }
    else if ((S3_REC_WORK_UNITS - 1U) == g_before.work_units)
    {
        assert(S3_REC_WORK_UNITS == g_record.work_units);
    }
    else
    {
        /* No budget-boundary assertion applies on this update. */
    }
    /* work_advance owns only the workspace and these two budget counters. */
    memcpy(&g_before.work, &g_record.work, sizeof(g_before.work));
    g_before.work_units = g_record.work_units;
    g_before.fit_ticks  = g_record.fit_ticks;
    assert(memcmp(&g_before, &g_record, sizeof(g_record)) == 0);
}

/** @brief Run the seeded phase at four work-unit and fit-tick budget
 * boundaries and emit the resulting transitions.
 */
static void boundary (const char * p_name, const bc_decoder_config_t * p_config)
{
    static const uint16_t work_budgets[4] = {127U, 128U, 127U, 127U};
    static const uint16_t fit_budgets[4]  = {0U, 0U, 31U, 32U};
    assert(bc_decoder_boundary_test_config_valid(p_config));
    g_boundaries++;
    for (unsigned int budget_index = 0U;
         budget_index < (sizeof work_budgets / sizeof work_budgets[0]);
         budget_index++)
    {
        memcpy(&g_record, &g_seed, sizeof(g_record));
        g_record.work_units = work_budgets[budget_index];
        g_record.fit_ticks  = fit_budgets[budget_index];
        advance(p_name, budget_index, p_config);
    }
}

/** @brief Exercise decoder inputs at phase boundaries. Each label describes
 * the input supplied to that update.
 * @details Seed the private recorder state so each observation hits a
 * specific phase. Emit both recorder and workspace fields before and after
 * the call. Include terminal indexes, budget exhaustion, exact threshold
 * equality and rejected or incomplete paths. The output is compared with
 * the corresponding fixed-baseline trace.
 */
static void phase_boundaries (void)
{
    bc_decoder_config_t config = bc_decoder_defaults();
#define CASE(name, phase_value, setup)                                         \
    do                                                                         \
    {                                                                          \
        seed_boundary((phase_value));                                          \
        setup;                                                                 \
        boundary((name), &config);                                             \
    } while (0)
    CASE("primary-rise", RW_PRIMARY, g_seed.work.cursor = 3U);
    CASE("primary-turn", RW_PRIMARY, (void)0);
    CASE("primary-falling-low", RW_PRIMARY, g_seed.work.b_is_rising = false);
    CASE("primary-falling-turn", RW_PRIMARY, {
        g_seed.work.b_is_rising = false;
        g_seed.work.best        = 0U;
        g_seed.work.cursor      = 1U;
    });
    CASE("primary-ext-capacity", RW_PRIMARY,
         g_seed.work.TEST_EXTREMA_COUNT_MEMBER = S3_REC_EXT_MAX - 1U);
    CASE("primary-eof-rising", RW_PRIMARY,
         g_seed.work.cursor = g_seed.work.frozen_count);
    CASE("primary-eof-falling", RW_PRIMARY, {
        g_seed.work.cursor      = g_seed.work.frozen_count;
        g_seed.work.b_is_rising = false;
    });
    CASE("primary-default-prominence", RW_PRIMARY, g_seed.prominence = 0U);
    CASE("white-eof", RW_FAINT_WHITE,
         g_seed.work.cursor = g_seed.work.frozen_count);
    CASE("white-quiet", RW_FAINT_WHITE, (void)0);
    CASE("white-ink", RW_FAINT_WHITE, g_seed.work.cursor = 3U);
    CASE("white-ext-capacity", RW_FAINT_WHITE, {
        g_seed.work.cursor                    = 3U;
        g_seed.work.TEST_EXTREMA_COUNT_MEMBER = 30U;
    });
    CASE("ink-eof", RW_FAINT_INK,
         g_seed.work.cursor = g_seed.work.frozen_count);
    CASE("ink-new-peak", RW_FAINT_INK, {
        g_seed.work.cursor = 3U;
        g_seed.work.peak   = 2U;
        g_seed.work.low_us = 10000U;
    });
    CASE("ink-equal-peak", RW_FAINT_INK, {
        g_seed.work.cursor = 3U;
        g_seed.work.low_us = 10000U;
    });
    CASE("ink-first-low", RW_FAINT_INK, (void)0);
    CASE("ink-dip-exact-limit", RW_FAINT_INK, g_seed.work.low_us = 25000U);
    CASE("ink-dip-over-limit", RW_FAINT_INK, g_seed.work.low_us = 30000U);
    CASE("valley-quiet", RW_FAINT_VALLEY, (void)0);
    CASE("valley-lower-dip", RW_FAINT_VALLEY, g_seed.work.dip = 1U);
    CASE("valley-next-ink", RW_FAINT_VALLEY, {
        g_seed.work.cursor                    = 3U;
        g_seed.work.TEST_EXTREMA_COUNT_MEMBER = 1U;
    });
    CASE("valley-eof", RW_FAINT_VALLEY, {
        g_seed.work.cursor                    = g_seed.work.frozen_count;
        g_seed.work.TEST_EXTREMA_COUNT_MEMBER = 1U;
    });
    CASE("prune-exact-count", RW_PRUNE_BEGIN, (void)0);
    CASE("prune-excess", RW_PRUNE_BEGIN,
         g_seed.work.TEST_EXTREMA_COUNT_MEMBER = 33U);
    CASE("prune-first-minimum", RW_PRUNE_SCAN, (void)0);
    CASE("prune-tied-minimum", RW_PRUNE_SCAN, g_seed.work.smallest = 5000000U);
    CASE("prune-larger-area", RW_PRUNE_SCAN, g_seed.work.smallest = 1U);
    CASE("prune-scan-end", RW_PRUNE_SCAN, g_seed.work.scan = 29U);
    CASE("prune-copy-entry", RW_PRUNE_COPY, (void)0);
    CASE("prune-copy-end", RW_PRUNE_COPY, g_seed.work.copy = 29U);
    CASE("edges-strong", RW_EDGES_BEGIN, (void)0);
    CASE("edges-weak", RW_EDGES_BEGIN, g_seed.sample[1] = (10U << 10) | 5U);
    CASE("edges-faint", RW_EDGES_BEGIN, g_seed.work.b_is_faint = true);
    CASE("edges-wrong-count", RW_EDGES_BEGIN,
         g_seed.work.TEST_EXTREMA_COUNT_MEMBER = 30U);
    CASE("edge-start", RW_EDGE_BEGIN, (void)0);
    CASE("edge-equal-extrema", RW_EDGE_BEGIN, g_seed.work.ext[1] = 0U);
    CASE("edge-end", RW_EDGE_BEGIN, {
        g_seed.work.segment          = 30U;
        g_seed.work.edge_previous_us = 12345U;
    });
    CASE("edge-rising-cross", RW_EDGE_SCAN, (void)0);
    CASE("edge-falling-cross", RW_EDGE_SCAN, {
        g_seed.work.segment          = 1U;
        g_seed.work.edge_cursor      = 2U;
        g_seed.work.edges            = 1U;
        g_seed.work.edge_previous_us = 2500U;
        g_seed.work.segment_us       = 5000U;
    });
    CASE("edge-core-midpoint", RW_EDGE_SCAN, {
        g_seed.work.b_is_faint = true;
        g_seed.sample[0]       = (10U << 10) | 5U;
        g_seed.sample[1]       = (10U << 10) | 10U;
    });
    CASE("edge-no-crossing", RW_EDGE_SCAN, g_seed.sample[1] = g_seed.sample[0]);
    CASE("edge-intermediate", RW_EDGE_SCAN, {
        g_seed.work.ext[1]    = 3U;
        g_seed.work.ext_us[1] = 15000U;
    });
    CASE("edge-repeat-outer", RW_EDGE_SCAN, {
        g_seed.work.b_has_crossed = true;
        g_seed.work.edge_us       = 77U;
    });
    CASE("edge-repeat-core", RW_EDGE_SCAN, {
        g_seed.work.b_is_faint    = true;
        g_seed.work.b_has_crossed = true;
        g_seed.work.edge_us       = 77U;
        g_seed.sample[1]          = (10U << 10) | 10U;
    });
    CASE("direct-valid", RW_DIRECT, (void)0);
    CASE("direct-reversed", RW_DIRECT, fill_runs(&g_seed.work, true));
    CASE("direct-zero-width", RW_DIRECT, g_seed.work.runs[0] = 0U);
    CASE("direct-existing-same", RW_DIRECT, {
        g_seed.work.b_has_candidate       = true;
        g_seed.work.candidate.letter_code = 1U;
    });
    CASE("direct-conflict", RW_DIRECT, {
        g_seed.work.b_has_candidate       = true;
        g_seed.work.candidate.letter_code = 2U;
    });
    CASE("fit-start-shared", RW_FIT_BEGIN, (void)0);
    CASE("fit-start-varying-second", RW_FIT_BEGIN, {
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.star_index       = 1U;
    });
    CASE("fit-orientations-end", RW_FIT_BEGIN, g_seed.work.orientation = 2U);
    CASE("fit-small-minimum", RW_FIT_BEGIN,
         memset(g_seed.work.runs, 0, sizeof(g_seed.work.runs)));
    CASE("fit-large-minimum", RW_FIT_BEGIN, {
        for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS;
             run_index++)
        {
            g_seed.work.runs[run_index] = config.max_run_us + 1U;
        }
    });
    CASE("fit-tick-negative", RW_FIT_TICK, g_seed.work.tick = -64);
    CASE("fit-tick-zero", RW_FIT_TICK, (void)0);
    CASE("fit-tick-63", RW_FIT_TICK, g_seed.work.tick = 63);
    CASE("fit-tick-64-miss", RW_FIT_TICK, g_seed.work.tick = 64);
    CASE("fit-tick-64-hit", RW_FIT_TICK, {
        g_seed.work.tick       = 64;
        g_seed.work.best_score = 0U;
        g_seed.work.best_bias  = 2000;
    });
    CASE("fit-varying-first", RW_FIT_TICK, {
        g_seed.work.tick             = 64;
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.best_score       = 0U;
        g_seed.work.best_bias        = 2000;
    });
    CASE("fit-varying-second", RW_FIT_TICK, {
        g_seed.work.tick             = 64;
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.star_index       = 1U;
        g_seed.work.best_score       = 0U;
        g_seed.work.best_bias        = 2000;
        g_seed.work.trial_bias[0]    = 4000;
    });
    CASE("fit-varying-sign-conflict", RW_FIT_TICK, {
        g_seed.work.tick             = 64;
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.star_index       = 1U;
        g_seed.work.best_score       = 0U;
        g_seed.work.best_bias        = 2000;
        g_seed.work.trial_bias[0]    = -4000;
    });
    CASE("fit-varying-delta-limit", RW_FIT_TICK, {
        g_seed.work.tick             = 64;
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.star_index       = 1U;
        g_seed.work.best_score       = 0U;
        g_seed.work.best_bias        = 15000;
    });
    CASE("fit-varying-delta-over", RW_FIT_TICK, {
        g_seed.work.tick             = 64;
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.star_index       = 1U;
        g_seed.work.best_score       = 0U;
        g_seed.work.best_bias        = 15001;
    });
    CASE("fit-varying-miss", RW_FIT_TICK, {
        g_seed.work.tick             = 64;
        g_seed.work.b_does_bias_vary = true;
    });
    CASE("fit-end-one", RW_FIT_END, g_seed.work.found = 1U);
    CASE("fit-end-none", RW_FIT_END, (void)0);
    CASE("fit-end-ambiguous", RW_FIT_END, g_seed.work.found = 2U);
    CASE("fit-end-varying-none", RW_FIT_END,
         g_seed.work.b_does_bias_vary = true);
    CASE("apply-unbiased", RW_APPLY, (void)0);
    CASE("apply-positive", RW_APPLY, g_seed.work.chosen[0] = 1000);
    CASE("apply-negative", RW_APPLY, g_seed.work.chosen[0] = -1000);
    CASE("apply-first-width-zero", RW_APPLY, g_seed.work.chosen[0] = -30000);
    CASE("apply-partial-before-zero", RW_APPLY, g_seed.work.chosen[0] = 60000);
    CASE("apply-partial-before-maximum", RW_APPLY, {
        g_seed.work.chosen[0] = 1000;
        g_seed.work.runs[2]   = config.max_run_us;
    });
    CASE("apply-varying", RW_APPLY, {
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.chosen[0]        = 2000;
        g_seed.work.chosen[1]        = 4000;
    });
    CASE("apply-varying-negative-odd", RW_APPLY, {
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.chosen[0]        = -2001;
        g_seed.work.chosen[1]        = -4001;
    });
    CASE("apply-varying-zero-input", RW_APPLY, {
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.runs[20]         = 0U;
    });
    CASE("apply-varying-large-input", RW_APPLY, {
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.runs[20]         = config.max_run_us + 1U;
    });
    CASE("apply-varying-large-frame", RW_APPLY, {
        g_seed.work.b_does_bias_vary = true;
        for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS;
             run_index++)
        {
            g_seed.work.runs[run_index] = config.max_run_us;
        }
    });
    CASE("apply-varying-partial", RW_APPLY, {
        g_seed.work.b_does_bias_vary = true;
        g_seed.work.chosen[0]        = 60000;
        g_seed.work.chosen[1]        = 60000;
    });
    CASE("validate-valid", RW_VALIDATE, (void)0);
    CASE("validate-leading-negative", RW_VALIDATE, {
        g_seed.leading_us     = 0U;
        g_seed.work.first_us  = 0U;
        g_seed.work.chosen[0] = 1;
    });
    CASE("validate-odd-negative-bias", RW_VALIDATE, g_seed.work.chosen[0] = -3);
    CASE("validate-invalid-width", RW_VALIDATE, g_seed.work.runs[28] = 0U);
    CASE("path-primary-ineligible", RW_PATH_END, (void)0);
    CASE("path-faint-eligible", RW_PATH_END,
         g_seed.prominence = S3_REC_MIN_PROMINENCE);
    CASE("path-already-faint", RW_PATH_END, {
        g_seed.prominence      = S3_REC_MIN_PROMINENCE;
        g_seed.work.b_is_faint = true;
    });
    CASE("done", RW_DONE, (void)0);
    CASE("unknown-zero", 0U, (void)0);
    CASE("unknown-255", UINT8_MAX, (void)0);
    CASE("primary-time-wrap", RW_PRIMARY,
         g_seed.work.total_us = UINT32_MAX - 1000U);
    CASE("prune-time-wrap", RW_PRUNE_SCAN, {
        g_seed.work.ext_us[1] = UINT32_MAX - 1000U;
        g_seed.work.ext_us[2] = 1000U;
    });
    CASE("edge-time-wrap", RW_EDGE_SCAN,
         g_seed.work.segment_us = UINT32_MAX - 1000U);
    CASE("direct-leading-saturation", RW_DIRECT, {
        g_seed.leading_us    = UINT32_MAX;
        g_seed.work.first_us = UINT32_MAX;
    });
    CASE("candidate-required-saturation", RW_DIRECT,
         g_seed.work.last_us = UINT32_MAX);
    /* Valid configuration boundary; wide arithmetic remains signed-safe. */
    config.max_run_us   = 214748364U;
    config.max_frame_us = 2147483647U;
    CASE("fit-large-valid-config", RW_FIT_BEGIN, {
        for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS;
             run_index++)
        {
            g_seed.work.runs[run_index] = 200000000U;
        }
    });
    CASE("apply-large-valid-widths", RW_APPLY, {
        g_seed.work.chosen[0] = 100000000;
        for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS;
             run_index++)
        {
            g_seed.work.runs[run_index] = 100000001U;
        }
    });
#undef CASE
}

/** @brief Append compact samples at the supplied darkness, checking the
 * record capacity before every write.
 */
static void append_samples (uint16_t darkness, unsigned int count)
{
    for (unsigned int sample_index = 0U; sample_index < count; sample_index++)
    {
        assert(g_record.count < S3_REC_MAX);
        g_record.sample[g_record.count++] = (uint16_t)((10U << 10) | darkness);
    }
}

/** @brief Drive a complete synthetic frame through bounded decoding updates
 * and trace the processing phases.
 */
static void complete_path (unsigned int variant)
{
    const bc_decoder_config_t config = bc_decoder_defaults();
    memset(&g_record, 0, sizeof(g_record));
    fill_runs(&g_record.work, 2U == variant);
    /* Save the input widths before work_begin clears its owned workspace. */
    uint32_t widths[S3_BARCODE_RUNS];
    memcpy(widths, g_record.work.runs, sizeof(widths));
    append_samples(0U, 60U);
    for (unsigned int run_index = 0U; run_index < S3_BARCODE_RUNS; run_index++)
    {
        const bool     b_is_black = (run_index % 2U) == 0U;
        const uint16_t darkness =
            !b_is_black || 4U == variant ? 0U : (1U == variant ? 10U : 1000U);
        int32_t width = (int32_t)widths[run_index];
        if (3U == variant)
        {
            width += b_is_black ? 15000 : -15000;
        }
        append_samples(darkness, (unsigned int)width / 5000U);
    }
    append_samples(0U, 60U);
    g_record.leading_us = 300000U;
    g_record.prominence =
        1U == variant ? S3_REC_MIN_PROMINENCE : S3_REC_PROMINENCE;
    bc_decoder_boundary_test_begin(&g_record, 1U == variant, 3U == variant);
    unsigned int calls = 0U;
    while ((int)g_record.work.phase != RW_DONE)
    {
        /* A guard detects nontermination; it does not dictate decoded output.
         */
        assert(calls++ < 4096U);
        g_record.work_units = 0U;
        g_record.fit_ticks  = 0U;
        advance("complete-sampled-path", variant, &config);
    }
}

/** @brief Inject barcode work at phase boundaries and emit the complete
 * scratch state before and after updates for differential
 * comparison.
 */
int main (void)
{
    phase_boundaries();
    for (unsigned int variant = 0U; variant < COMPLETE_PATH_VARIANTS; variant++)
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

