/** @file bcap_test.c
 * @brief Barcode capture: episode detection, pre-trigger samples, quiet end,
 * pool limits, decoder counters and the USB dump format.
 */
/* Barcode capture: episode detection, pre-trigger samples, quiet end, pool
 * limits, decoder counters and the USB dump format. */
#include "bc_capture.h"
#include "run_timing.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static bcap_t cap;
static uint32_t t;
static double pos;

static void feed (uint16_t dark, uint8_t status, char letter)
{
    const bcap_input_t in = {t,      (uint16_t)(150U + dark),
                             dark,   dark >= 600U ? 2U : 1U,
                             status, letter,
                             false,  false,
                             pos};
    bcap_record(&cap, &in);
    t += S3_SAMPLE_PERIOD_US;
    pos += 0.001;
}

static unsigned dump (char * all, size_t size)
{
    bcap_cursor_t cur = {0};
    char line[200];
    unsigned lines = 0U;
    all[0] = '\0';
    while (bcap_format_next(&cap, &cur, line, sizeof line))
    {
        assert(strlen(line) <= 160U);
        strncat(all, line, size - strlen(all) - 2U);
        strcat(all, "\n");
        lines++;
    }
    return lines;
}

static void check_positions (const double * positions,
                             const uint16_t * expected, unsigned count)
{
    assert(count > 0U && count <= BCAP_PRE);
    bcap_begin_run(&cap, t);
    for (unsigned i = 0U; i < count; ++i)
    {
        const bcap_input_t in = {.t_us = t,
                                 .raw = 200,
                                 .surface = i + 1U == count ? 2U : 1U,
                                 .centre_m = positions[i]};
        bcap_record(&cap, &in);
        t += S3_SAMPLE_PERIOD_US;
    }
    assert(cap.active && cap.episode[0].count == count);
    for (unsigned i = 0U; i < count; ++i)
        assert(cap.pool[cap.episode[0].first + i].mm == expected[i]);
    bcap_end_run(&cap);
}

static void fixed_point_odometry_tests (void)
{
    /* Preserve unsigned forward distance through negative positions, reverse
     * motion and crossing zero. Rounding remains to the nearest wire mm. */
    const double negative[] = {-10.0, -9.999, -9.990, -9.980};
    const uint16_t negative_mm[] = {0, 1, 10, 20};
    check_positions(negative, negative_mm, 4);
    const double reverse[] = {10.0, 9.99, 9.98, 9.97};
    const uint16_t zero_mm[] = {0, 0, 0, 0};
    check_positions(reverse, zero_mm, 4);
    const double rounded[] = {10.0, 9.99, 10.00049, 10.00051, 10.010};
    const uint16_t rounded_mm[] = {0, 0, 0, 1, 10};
    check_positions(rounded, rounded_mm, 5);
    const double crossing[] = {-0.00051, -0.00002, 0.0, 0.00049, 0.001};
    const uint16_t crossing_mm[] = {0, 0, 1, 1, 2};
    check_positions(crossing, crossing_mm, 5);
    /* Absolute odometry can outlive many runs; it must not wrap at 32-bit
     * millimetre or micrometre limits or lose small increments in that range. */
    const double long_run[] = {10000000.0, 10000000.001, 10000000.100,
                               10000001.234};
    const uint16_t long_run_mm[] = {0, 1, 100, 1234};
    check_positions(long_run, long_run_mm, 4);
    const double saturated[] = {0.0, 65.5344, 65.5346, 65.535, 70.0};
    const uint16_t saturated_mm[] = {0, 65534, 65535, 65535, 65535};
    check_positions(saturated, saturated_mm, 5);
    /* A missing origin makes the episode's distances unavailable. Missing
     * individual readings must not fabricate a forward jump from NaN/Inf. */
    const double invalid_origin[] = {NAN, 1.0, 2.0, 3.0};
    check_positions(invalid_origin, zero_mm, 4);
    const double invalid_sample[] = {1.0, NAN, INFINITY, -INFINITY, 1.001};
    const uint16_t invalid_sample_mm[] = {0, 0, 0, 0, 1};
    check_positions(invalid_sample, invalid_sample_mm, 5);
    const double finite_extremes[] = {-DBL_MAX, -DBL_MAX, DBL_MAX, DBL_MAX};
    const uint16_t finite_extremes_mm[] = {0, 0, 65535, 65535};
    check_positions(finite_extremes, finite_extremes_mm, 4);
}

int main (void)
{
    static char text[200000];
    unsigned i;
    t = 1000000U;
    pos = 1.0;
    assert(BCAP_PRE * S3_SAMPLE_PERIOD_US == S3_REC_PRE_US);
    assert(BCAP_END_QUIET * S3_SAMPLE_PERIOD_MS == 400U);
    bcap_begin_run(&cap, t);
    for (i = 0; i < BCAP_PRE + 30U; i++)
        feed(20, 0, 0); /* wrap real pre-trigger history */
    assert(cap.episodes == 0 && !cap.active);
    for (i = 0; i < 6; i++)
    {
        feed(900, 0, 0);
        feed(40, 0, 0);
    } /* bars */
    const uint32_t last_black_us = t - 2U * S3_SAMPLE_PERIOD_US;
    assert(cap.active);
    for (i = 0; i < BCAP_END_QUIET - 2U; i++)
        feed(30, 0, 0); /* the last gap counted 1 */
    assert(cap.active); /* not yet quiet */
    feed(30, 1, 'Z');   /* decoded at end */
    assert(!cap.active && cap.episodes == 1);
    assert(t - S3_SAMPLE_PERIOD_US - last_black_us == 400000U);
    assert(cap.decoded == 1 && strcmp(cap.letters, "Z") == 0);
    /* Pre-trigger samples kept; distance counts from the first stored one. */
    assert(cap.episode[0].count == BCAP_PRE + 12U + BCAP_END_QUIET - 1U);
    assert(cap.episode[0].start_ms == 30U * S3_SAMPLE_PERIOD_MS);
    assert(cap.pool[cap.episode[0].first + BCAP_PRE].dt_ms ==
           S3_SAMPLE_PERIOD_MS);
    assert(cap.pool[cap.episode[0].first].mm == 0U);
    assert(cap.pool[cap.episode[0].first + BCAP_PRE].mm == BCAP_PRE);
    for (i = 0U; i <= BCAP_PRE; ++i)
        assert(cap.pool[cap.episode[0].first + i].mm ==
               i); /* wrapped history in order */
    assert((cap.pool[cap.episode[0].first + BCAP_PRE].flags & 3U) == 2U);
    /* Status counters outside episodes too. */
    feed(10, 2, 0);
    feed(10, 3, 0);
    feed(10, 4, 0);
    assert(cap.rejected == 1 && cap.timing == 1 && cap.overflow == 1);
    /* A run that ends mid-episode closes it. */
    feed(800, 0, 0);
    assert(cap.active);
    bcap_end_run(&cap);
    assert(!cap.active && cap.episodes == 2);
    {
        const unsigned lines = dump(text, sizeof text);
        assert(strstr(
            text,
            "[BCAP] run summary: episodes=2 not_stored=0 decoded=1 letters=Z rejected=1 timing_error=1 overflow=1"));
        assert(strstr(text, "[BCAP] episode 1 t="));
        assert(strstr(text, "BC1 0: "));
        assert(strstr(text, "[BCAP] end\n"));
        assert(lines ==
               2U + 2U +
                   (cap.episode[0].count + BCAP_PER_LINE - 1U) / BCAP_PER_LINE +
                   (cap.episode[1].count + BCAP_PER_LINE - 1U) / BCAP_PER_LINE);
    }
    /* Pool and episode limits: later episodes are counted, never overflow. */
    bcap_begin_run(&cap, t);
    for (unsigned e = 0; e < BCAP_MAX_EPISODES + 5U; e++)
    {
        for (i = 0; i < 300; i++)
            feed(i % 2 ? 900 : 50, 0, 0);
        for (i = 0; i < BCAP_END_QUIET; i++)
            feed(10, 0, 0);
    }
    assert(cap.used <= BCAP_POOL && cap.episodes <= BCAP_MAX_EPISODES);
    assert(cap.dropped > 0);
    (void)dump(text, sizeof text);
    /* A faint barcode (never BLACK) is still captured while a reader is
     * collecting, and stays one episode through its white gaps. */
    bcap_begin_run(&cap, t);
    for (i = 0; i < BCAP_PRE + 30U; i++)
        feed(20, 0, 0);
    {
        bcap_input_t in = {t, 650U, 500U, 0U, 0U, 0, true, false, pos};
        for (i = 0; i < 3U * BCAP_END_QUIET; i++)
        { /* grey bars, then white while the reader still decides */
            in.t_us = t;
            in.darkness = (i < 10U) ? 500U : 20U;
            in.surface = (i < 10U) ? 0U : 1U;
            bcap_record(&cap, &in);
            t += S3_SAMPLE_PERIOD_US;
            assert(cap.active && cap.episodes == 0);
        }
        in.collecting = false;
        in.t_us = t;
        in.status = 1U;
        in.letter = 'Y';
        bcap_record(&cap, &in);
        assert(!cap.active && cap.episodes == 1 &&
               strcmp(cap.letters, "Y") == 0);
        assert(cap.episode[0].count == BCAP_PRE + 3U * BCAP_END_QUIET + 1U);
    }
    /* Letters buffer never overflows. */
    bcap_begin_run(&cap, t);
    for (i = 0; i < 20; i++)
        feed(10, 1, (char)('A' + i));
    assert(cap.decoded == 20 && strlen(cap.letters) == sizeof cap.letters - 1U);
    fixed_point_odometry_tests();
    puts(
        "PASS: barcode capture episodes, pre-trigger, quiet end, run end, counters, bounded pool/letters, fixed-point odometry, faint reader-collecting capture and USB dump format");
    return 0;
}

/* End of file. */

