/** @file bcap_main.c
 * @brief Barcode capture: episode detection, pre-trigger samples, quiet end,
 * pool limits, decoder counters and the USB dump format.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 */
#include "bcap_main.h"
#include "../../app_program/app_types.h"
#include "bc_capture.h"
#include "run_timing.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define EXTRA_PRETRIGGER_SAMPLES (30U)
#define SYNTHETIC_BAR_COUNT (6)
#define QUIET_ALREADY_SAMPLED (2U)
#define EXCESS_EPISODES (5U)
#define SAMPLES_PER_LONG_EPISODE (300)
#define COLLECTING_QUIET_WINDOWS (3U)
#define LETTER_CAPACITY_CASES (20)

static bc_capture_t g_cap;
static uint32_t     g_sample_time_us;
static float64_t    g_pos;

static void     feed (uint16_t dark, uint8_t status, char letter);
static unsigned dump (char * p_all, size_t size);
static void     check_positions (const float64_t * p_positions,
                                 const uint16_t * p_expected, unsigned count);
static void     fixed_point_odometry_tests (void);

/** @brief Feed samples and odometry into capture storage and check trigger
 * history, episode boundaries, retained data and text export.
 */
int main (void)
{
    static char text[200000];
    unsigned    index;
    g_sample_time_us = 1000000U;
    g_pos            = 1.0;
    assert(BCAP_PRE * S3_SAMPLE_PERIOD_US == S3_REC_PRE_US);
    assert(BCAP_END_QUIET * S3_SAMPLE_PERIOD_MS == 400U);
    bc_capture_begin_run(&g_cap, g_sample_time_us);

    for (index = 0; index < (BCAP_PRE + EXTRA_PRETRIGGER_SAMPLES); index++)
    {
        feed(20, 0, 0); /* Wrap the pre-trigger history with real samples. */
    }

    assert(0 == (int)g_cap.episodes && !g_cap.b_is_active);

    for (index = 0; index < (unsigned)SYNTHETIC_BAR_COUNT; index++)
    {
        /* bars */
        feed(900, 0, 0);
        feed(40, 0, 0);
    }

    const uint32_t last_black_us =
        g_sample_time_us - (2U * S3_SAMPLE_PERIOD_US);
    assert(g_cap.b_is_active);

    for (index = 0; index < (BCAP_END_QUIET - QUIET_ALREADY_SAMPLED); index++)
    {
        feed(30, 0, 0); /* The last gap contributed one sample. */
    }

    assert(g_cap.b_is_active); /* Quiet has not completed. */
    feed(30, 1, 'Z');          /* Decode at the end. */
    assert(!g_cap.b_is_active && 1 == (int)g_cap.episodes);
    assert(g_sample_time_us - S3_SAMPLE_PERIOD_US - last_black_us == 400000U);
    assert(1 == (int)g_cap.decoded && strcmp(g_cap.letters, "Z") == 0);
    /* Pre-trigger samples kept; distance counts from the first stored one. */
    assert((BCAP_PRE + 12U + BCAP_END_QUIET - 1U) == g_cap.episode[0].count);
    assert((30U * S3_SAMPLE_PERIOD_MS) == g_cap.episode[0].start_ms);
    assert(S3_SAMPLE_PERIOD_MS ==
           g_cap.pool[g_cap.episode[0].first + BCAP_PRE].dt_ms);
    assert(0U == g_cap.pool[g_cap.episode[0].first].distance_mm);
    assert(BCAP_PRE ==
           g_cap.pool[g_cap.episode[0].first + BCAP_PRE].distance_mm);

    for (index = 0U; index <= BCAP_PRE; ++index)
    {
        assert(g_cap.pool[g_cap.episode[0].first + index].distance_mm ==
               index); /* Keep the wrapped history in order. */
    }

    assert((g_cap.pool[g_cap.episode[0].first + BCAP_PRE].flags & 3U) == 2U);
    /* Status counters outside episodes too. */
    feed(10, 2, 0);
    feed(10, 3, 0);
    feed(10, 4, 0);
    assert(1 == (int)g_cap.rejected && 1 == (int)g_cap.timing &&
           1 == (int)g_cap.overflow);
    /* A run that ends mid-episode closes it. */
    feed(800, 0, 0);
    assert(g_cap.b_is_active);
    bc_capture_end_run(&g_cap);
    assert(!g_cap.b_is_active && 2 == (int)g_cap.episodes);
    {
        const unsigned lines = dump(text, sizeof text);
        assert(strstr(text,
                      "[BCAP] run summary: episodes=2 not_stored=0 decoded=1 "
                      "letters=Z rejected=1 timing_error=1 overflow=1"));
        assert(strstr(text, "[BCAP] episode 1 t="));
        assert(strstr(text, "BC1 0: "));
        assert(strstr(text, "[BCAP] end\n"));
        assert(lines == 2U + 2U +
                            (g_cap.episode[0].count + BCAP_PER_LINE - 1U) /
                                BCAP_PER_LINE +
                            (g_cap.episode[1].count + BCAP_PER_LINE - 1U) /
                                BCAP_PER_LINE);
    }
    /* Pool and episode limits: later episodes are counted, never overflow. */
    bc_capture_begin_run(&g_cap, g_sample_time_us);

    for (unsigned episode_index = 0;
         episode_index < (BCAP_MAX_EPISODES + EXCESS_EPISODES); episode_index++)
    {
        for (index = 0; index < (unsigned)SAMPLES_PER_LONG_EPISODE; index++)
        {
            feed(index % 2U ? 900 : 50, 0, 0);
        }

        for (index = 0; index < BCAP_END_QUIET; index++)
        {
            feed(10, 0, 0);
        }
    }

    assert(g_cap.used <= BCAP_POOL && g_cap.episodes <= BCAP_MAX_EPISODES);
    assert((int)g_cap.dropped > 0);
    (void)dump(text, sizeof text);
    /* A faint barcode (never BLACK) is still captured while a reader is
     * collecting, and stays one episode through its white gaps. */
    bc_capture_begin_run(&g_cap, g_sample_time_us);

    for (index = 0; index < (BCAP_PRE + EXTRA_PRETRIGGER_SAMPLES); index++)
    {
        feed(20, 0, 0);
    }

    {
        bc_capture_input_t sample_input = {
            g_sample_time_us, 650U, 500U, 0U, 0U, 0, true, false, g_pos};

        for (index = 0; index < (COLLECTING_QUIET_WINDOWS * BCAP_END_QUIET);
             index++)
        {
            /* Follow grey bars with white during processing. */
            sample_input.t_us     = g_sample_time_us;
            sample_input.darkness = (index < 10U) ? 500U : 20U;
            sample_input.surface  = (index < 10U) ? 0U : 1U;
            bc_capture_record(&g_cap, &sample_input);
            g_sample_time_us += S3_SAMPLE_PERIOD_US;
            assert(g_cap.b_is_active && 0 == (int)g_cap.episodes);
        }

        sample_input.b_is_collecting = false;
        sample_input.t_us            = g_sample_time_us;
        sample_input.status          = 1U;
        sample_input.letter          = 'Y';
        bc_capture_record(&g_cap, &sample_input);
        assert(!g_cap.b_is_active && 1 == (int)g_cap.episodes &&
               strcmp(g_cap.letters, "Y") == 0);
        assert((BCAP_PRE + COLLECTING_QUIET_WINDOWS * BCAP_END_QUIET + 1U) ==
               g_cap.episode[0].count);
    }
    /* Letters buffer never overflows. */
    bc_capture_begin_run(&g_cap, g_sample_time_us);

    for (index = 0; index < (unsigned)LETTER_CAPACITY_CASES; index++)
    {
        feed(10, 1, (char)((unsigned)('A') + index));
    }

    assert(20 == (int)g_cap.decoded &&
           strlen(g_cap.letters) == sizeof g_cap.letters - 1U);
    fixed_point_odometry_tests();
    puts("PASS: barcode capture episodes, pre-trigger, quiet end, run end, "
         "counters, bounded pool/letters, fixed-point odometry, faint "
         "reader-collecting capture and USB dump format");
    return 0;
}

/** @brief Append a barcode-capture sample with the supplied darkness and
 * decoder status, then advance time and travel by one fixture step.
 */
static void feed (uint16_t dark, uint8_t status, char letter)
{
    const bc_capture_input_t input = {g_sample_time_us,
                                      (uint16_t)(150U + dark),
                                      dark,
                                      dark >= 600U ? 2U : 1U,
                                      status,
                                      letter,
                                      false,
                                      false,
                                      g_pos};
    bc_capture_record(&g_cap, &input);
    g_sample_time_us += S3_SAMPLE_PERIOD_US;
    g_pos            += 0.001;
}

/** @brief Append each formatted capture line and newline to the supplied
 * output text and return the number of emitted lines.
 */
static unsigned dump (char * p_all, size_t size)
{
    bc_capture_cursor_t cur = {0};
    char                line[200];
    unsigned            lines = 0U;
    p_all[0]                  = '\0';

    while (bc_capture_format_next(&g_cap, &cur, line, sizeof line))
    {
        assert(strlen(line) <= 160U);
        strncat(p_all, line, size - strlen(p_all) - 2U);
        strcat(p_all, "\n");
        lines++;
    }

    return lines;
}

/** @brief Feed the supplied positions into a fresh capture and compare every
 * retained pre-trigger distance with its expected millimetre value.
 */
static void check_positions (const float64_t * p_positions,
                             const uint16_t * p_expected, unsigned count)
{
    assert(count > 0U && count <= BCAP_PRE);
    bc_capture_begin_run(&g_cap, g_sample_time_us);

    for (unsigned sample_index = 0U; sample_index < count; ++sample_index)
    {
        const bc_capture_input_t input = {
            .t_us     = g_sample_time_us,
            .raw      = 200,
            .surface  = (sample_index + 1U) == count ? 2U : 1U,
            .centre_m = p_positions[sample_index]};
        bc_capture_record(&g_cap, &input);
        g_sample_time_us += S3_SAMPLE_PERIOD_US;
    }

    assert(g_cap.b_is_active && g_cap.episode[0].count == count);

    for (unsigned sample_index = 0U; sample_index < count; ++sample_index)
    {
        assert(g_cap.pool[g_cap.episode[0].first + sample_index].distance_mm ==
               p_expected[sample_index]);
    }

    bc_capture_end_run(&g_cap);
}

/** @brief Check conversion of negative, large and extreme finite odometry
 * readings into bounded capture distances.
 */
static void fixed_point_odometry_tests (void)
{
    /* Preserve unsigned forward distance through negative positions, reverse
     * motion and crossing zero. Rounding remains to the nearest wire mm. */
    const float64_t negative[]    = {-10.0, -9.999, -9.990, -9.980};
    const uint16_t  negative_mm[] = {0, 1, 10, 20};
    check_positions(negative, negative_mm, 4);
    const float64_t reverse[] = {10.0, 9.99, 9.98, 9.97};
    const uint16_t  zero_mm[] = {0, 0, 0, 0};
    check_positions(reverse, zero_mm, 4);
    const float64_t rounded[]    = {10.0, 9.99, 10.00049, 10.00051, 10.010};
    const uint16_t  rounded_mm[] = {0, 0, 0, 1, 10};
    check_positions(rounded, rounded_mm, 5);
    const float64_t crossing[]    = {-0.00051, -0.00002, 0.0, 0.00049, 0.001};
    const uint16_t  crossing_mm[] = {0, 0, 1, 1, 2};
    check_positions(crossing, crossing_mm, 5);
    /* Absolute odometry can outlive many runs; it must not wrap at 32-bit
     * millimetre or micrometre limits or lose small increments in that range.
     */
    const float64_t long_run[]    = {10000000.0, 10000000.001, 10000000.100,
                                     10000001.234};
    const uint16_t  long_run_mm[] = {0, 1, 100, 1234};
    check_positions(long_run, long_run_mm, 4);
    const float64_t saturated[]    = {0.0, 65.5344, 65.5346, 65.535, 70.0};
    const uint16_t  saturated_mm[] = {0, 65534, 65535, 65535, 65535};
    check_positions(saturated, saturated_mm, 5);
    /* A missing origin makes the episode's distances unavailable. Missing
     * individual readings must not fabricate a forward jump from NaN/Inf. */
    const float64_t invalid_origin[] = {NAN, 1.0, 2.0, 3.0};
    check_positions(invalid_origin, zero_mm, 4);
    const float64_t invalid_sample[] = {1.0, NAN, INFINITY, -INFINITY, 1.001};
    const uint16_t  invalid_sample_mm[] = {0, 0, 0, 0, 1};
    check_positions(invalid_sample, invalid_sample_mm, 5);
    const float64_t finite_extremes[] = {-DBL_MAX, -DBL_MAX, DBL_MAX, DBL_MAX};
    const uint16_t  finite_extremes_mm[] = {0, 0, 65535, 65535};
    check_positions(finite_extremes, finite_extremes_mm, 4);
}

/* End of file. */

