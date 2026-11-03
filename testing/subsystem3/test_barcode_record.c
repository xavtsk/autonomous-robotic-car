/* Host tests for the record-then-decide barcode reader (s3_barcode_record_*).
 * Synthetic darkness traces only; real-car evidence is in the BCAP traces.
 * Each trace: paper, a Code 39 "*X*" frame (bars/gaps at chosen darkness
 * levels, optional blur = moving average over an aperture, optional noise),
 * then paper again. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "barcode_decoder.h"

static const uint16_t letters[26] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
    0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
    0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0};

typedef struct {
    uint32_t narrow_us, wide_us;  /* module widths                        */
    uint32_t period_us;           /* sample spacing                       */
    uint32_t blur_us;             /* sensor aperture (0 = sharp)          */
    uint16_t paper, bar, gap;     /* darkness levels                      */
    uint16_t noise;               /* +/- noise amplitude                  */
    uint32_t lead_us, trail_us;   /* paper before / after the frame       */
} shape_t;

static uint32_t rng = 12345U;
static uint32_t next_rand(void) { rng = rng * 1103515245U + 12345U; return (rng >> 8) & 0xFFFFU; }

static void make_runs(char letter, const shape_t *s, uint32_t runs[29])
{
    uint16_t sym[3] = {0x094U, letters[(unsigned int)(letter - 'A')], 0x094U};
    unsigned int c, i;
    for (c = 0U; c < 3U; c++) {
        for (i = 0U; i < 9U; i++) {
            runs[c * 10U + i] = (sym[c] & (1U << (8U - i))) ? s->wide_us : s->narrow_us;
        }
        if (c < 2U) { runs[c * 10U + 9U] = s->narrow_us; }
    }
}

/* Ideal level at time t (us from trace start). Odd stripes are gaps. */
static uint16_t ideal(const shape_t *s, const uint32_t *runs, unsigned int n, uint32_t t)
{
    uint32_t edge = s->lead_us;
    unsigned int i;
    if (t < edge) { return s->paper; }
    for (i = 0U; i < n; i++) {
        edge += runs[i];
        if (t < edge) { return (i % 2U) == 0U ? s->bar : s->gap; }
    }
    return s->paper;
}

/* Feed a trace; returns the number of DECODED results (last one in *out). */
typedef struct { unsigned int decoded, rejected, timing, overflow; s3_barcode_result_t last; } tally_t;

static tally_t feed(s3_barcode_record_t *r, uint32_t *clock, const shape_t *s,
                    const uint32_t *runs, unsigned int n)
{
    const s3_barcode_config_t cfg = s3_barcode_defaults();
    tally_t k = {0};
    uint32_t total = s->lead_us + s->trail_us, t;
    unsigned int i;
    for (i = 0U; i < n; i++) { total += runs[i]; }
    for (t = 0U; t < total; t += s->period_us) {
        uint32_t sum = 0U, cnt = 0U, u;
        int32_t v;
        s3_barcode_result_t res;
        s3_barcode_status_t st;
        if (s->blur_us == 0U) { sum = ideal(s, runs, n, t); cnt = 1U; }
        else {
            for (u = (t > s->blur_us / 2U) ? t - s->blur_us / 2U : 0U; u <= t + s->blur_us / 2U; u += 1000U) {
                sum += ideal(s, runs, n, u); cnt++;
            }
        }
        v = (int32_t)(sum / cnt);
        if (s->noise != 0U) { v += (int32_t)(next_rand() % (2U * s->noise + 1U)) - (int32_t)s->noise; }
        if (v < 0) { v = 0; }
        if (v > 1000) { v = 1000; }
        *clock += s->period_us;
        st = s3_barcode_record_update(r, &cfg, *clock, true, (uint16_t)v, &res);
        if (st == S3_BARCODE_DECODED) { k.decoded++; k.last = res; }
        else if (st == S3_BARCODE_REJECTED) { k.rejected++; }
        else if (st == S3_BARCODE_TIMING_ERROR) { k.timing++; }
        else if (st == S3_BARCODE_OVERFLOW) { k.overflow++; }
    }
    return k;
}

static void reverse_runs(uint32_t runs[29])
{
    unsigned int i;
    for (i = 0U; i < 14U; i++) { uint32_t x = runs[i]; runs[i] = runs[28U - i]; runs[28U - i] = x; }
}

static const shape_t sharp = {40000U, 120000U, 10000U, 0U, 10U, 1000U, 10U, 0U, 1500000U, 1500000U};

static void test_all_letters(void)
{
    /* Sharp, blurred (thin bars only reach ~half), and faint grey (Y-like:
     * bars 550, gaps 300 - never black/white for fixed 400/600 limits). */
    shape_t shapes[3];
    unsigned int s, c, dir;
    shapes[0] = sharp;
    shapes[1] = sharp; shapes[1].narrow_us = 60000U; shapes[1].wide_us = 150000U;
    shapes[1].blur_us = 60000U; shapes[1].noise = 15U;
    shapes[2] = sharp; shapes[2].narrow_us = 50000U; shapes[2].wide_us = 130000U;
    shapes[2].bar = 550U; shapes[2].gap = 300U; shapes[2].blur_us = 20000U; shapes[2].noise = 10U;
    for (s = 0U; s < 3U; s++) {
        for (c = 0U; c < 26U; c++) {
            for (dir = 0U; dir < 2U; dir++) {
                static s3_barcode_record_t r;
                uint32_t runs[29], clock = 0xFFFF0000U; /* also crosses the 32-bit wrap */
                tally_t k;
                make_runs((char)('A' + c), &shapes[s], runs);
                if (dir) { reverse_runs(runs); }
                s3_barcode_record_reset(&r);
                k = feed(&r, &clock, &shapes[s], runs, 29U);
                if (!(k.decoded == 1U && k.last.letter == (char)('A' + c) &&
                      k.last.reversed == (dir != 0U) && k.rejected == 0U)) {
                    printf("shape %u letter %c dir %u: decoded %u (%c) rejected %u\n",
                           s, 'A' + c, dir, k.decoded, k.last.letter ? k.last.letter : '-', k.rejected);
                    assert(0);
                }
                assert(!s3_barcode_record_active(&r));
            }
        }
    }
}

static void test_faint_y_beats_fixed_limits(void)
{
    /* The fixed 400/600 reader sees no black at all in the faint trace. */
    shape_t s = sharp;
    s3_barcode_decoder_t live;
    s3_barcode_config_t cfg = s3_barcode_defaults();
    s3_barcode_result_t res;
    uint32_t runs[29], t = 0U, clock = 0U, total = 0U;
    unsigned int i, live_decoded = 0U;
    static s3_barcode_record_t r;
    s.bar = 550U; s.gap = 300U;
    make_runs('Y', &s, runs);
    assert(s3_barcode_init(&live, &cfg));
    for (i = 0U; i < 29U; i++) { total += runs[i]; }
    total += s.lead_us + s.trail_us;
    for (t = 0U; t < total; t += s.period_us) {
        uint16_t v = ideal(&s, runs, 29U, t);
        static bool black = false;
        if (v >= 600U) { black = true; } else if (v <= 400U) { black = false; }
        if (s3_barcode_update(&live, 1000U + t, true, black, &res) == S3_BARCODE_DECODED) { live_decoded++; }
    }
    assert(live_decoded == 0U);
    s3_barcode_record_reset(&r);
    {
        tally_t k = feed(&r, &clock, &s, runs, 29U);
        assert(k.decoded == 1U && k.last.letter == 'Y');
    }
}

static void test_scratch_inside_wide_bar(void)
{
    /* A 20 ms scratch to darkness 450 in the middle of a wide bar adds a
     * false dip; the reader trims it and still reads the letter. */
    static s3_barcode_record_t r;
    const s3_barcode_config_t cfg = s3_barcode_defaults();
    uint32_t runs[29], clock = 0U, t, total = 0U, edge;
    unsigned int i, decoded = 0U;
    char got = 0;
    shape_t s = sharp;
    s.narrow_us = 140000U; s.wide_us = 420000U;   /* big barcode, slow pass */
    make_runs('Z', &s, runs);
    for (i = 0U; i < 29U; i++) { total += runs[i]; }
    total += s.lead_us + s.trail_us;
    edge = s.lead_us;
    for (i = 0U; i < 14U; i++) { edge += runs[i]; }
    edge += runs[14] / 2U;   /* centre of stripe 14 (a wide bar in Z) */
    assert(runs[14] == s.wide_us);
    s3_barcode_record_reset(&r);
    for (t = 0U; t < total; t += s.period_us) {
        s3_barcode_result_t res;
        uint16_t v = ideal(&s, runs, 29U, t);
        if ((t >= edge) && (t < edge + 20000U)) { v = 450U; }
        clock += s.period_us;
        if (s3_barcode_record_update(&r, &cfg, clock, true, v, &res) == S3_BARCODE_DECODED) {
            decoded++; got = res.letter;
        }
    }
    assert(decoded == 1U && got == 'Z');
}

static void test_grey_shelf_in_wide_bar(void)
{
    /* Seen on the car (3 Nov, big Z): a wide bar whose first half reads grey
     * (~550) and second half black. Its edge is where it leaves the paper,
     * not the middle of the grey shelf. */
    static s3_barcode_record_t r;
    const s3_barcode_config_t cfg = s3_barcode_defaults();
    uint32_t runs[29], clock = 0U, t, total = 0U, start;
    unsigned int i, decoded = 0U, bar;
    char got = 0;
    shape_t s = sharp;
    s.narrow_us = 140000U; s.wide_us = 420000U;
    make_runs('Z', &s, runs);
    for (i = 0U; i < 29U; i++) { total += runs[i]; }
    total += s.lead_us + s.trail_us;
    for (bar = 0U; bar < 29U; bar += 2U) {
        if (runs[bar] != s.wide_us) { continue; }
        start = s.lead_us;
        for (i = 0U; i < bar; i++) { start += runs[i]; }
        s3_barcode_record_reset(&r); clock = 0U; decoded = 0U;
        for (t = 0U; t < total; t += s.period_us) {
            s3_barcode_result_t res;
            uint16_t v = ideal(&s, runs, 29U, t);
            if ((t >= start) && (t < start + runs[bar] / 2U)) { v = 550U; }
            clock += s.period_us;
            if (s3_barcode_record_update(&r, &cfg, clock, true, v, &res) == S3_BARCODE_DECODED) {
                decoded++; got = res.letter;
            }
        }
        assert(decoded == 1U && got == 'Z');
    }
}

static void test_never_a_wrong_letter(void)
{
    /* Every stripe width jittered by up to +/-25 %, random levels, blur and
     * noise: the reader may refuse, but must not report a letter that was
     * not there. (Off-line: 30000 such frames gave 0 wrong letters at
     * +/-25 %; at +/-35 % it was 3 wrong in 30000 - widths that far off can
     * really spell another letter, since Code 39 has no check digit.) */
    unsigned int trial, wrong = 0U, right = 0U;
    for (trial = 0U; trial < 3000U; trial++) {
        static s3_barcode_record_t r;
        shape_t s = sharp;
        uint32_t runs[29], clock = 0U;
        unsigned int i;
        char letter = (char)('A' + next_rand() % 26U);
        tally_t k;
        s.narrow_us = 40000U + (next_rand() % 40U) * 1000U;
        s.wide_us = s.narrow_us * (20U + next_rand() % 15U) / 10U;
        s.bar = (uint16_t)(450U + next_rand() % 550U);
        s.gap = (uint16_t)(next_rand() % 350U);
        s.blur_us = (next_rand() % 50U) * 1000U;
        s.noise = (uint16_t)(next_rand() % 40U);
        make_runs(letter, &s, runs);
        for (i = 0U; i < 29U; i++) { runs[i] = runs[i] * (75U + next_rand() % 51U) / 100U; }
        s3_barcode_record_reset(&r);
        k = feed(&r, &clock, &s, runs, 29U);
        if (k.decoded) { if (k.last.letter == letter) { right++; } else { wrong++; } }
    }
    assert(wrong == 0U);
    assert(right > 0U);
}

static void test_cut_pass_is_not_read(void)
{
    /* The first two stripes were never seen: 27 stripes cannot decode. */
    static s3_barcode_record_t r;
    uint32_t runs[29], clock = 0U;
    tally_t k;
    make_runs('Y', &sharp, runs);
    s3_barcode_record_reset(&r);
    k = feed(&r, &clock, &sharp, &runs[2], 27U);
    assert(k.decoded == 0U && k.rejected == 1U);
}

static void test_short_quiet_zones(void)
{
    static s3_barcode_record_t r;
    uint32_t runs[29], clock = 0U;
    tally_t k;
    shape_t s = sharp;
    make_runs('A', &s, runs);
    s.lead_us = 200000U;   /* < 10 narrow modules (400 ms) before the frame */
    s3_barcode_record_reset(&r);
    k = feed(&r, &clock, &s, runs, 29U);
    assert(k.decoded == 0U);
    /* Trailing quiet broken by a dark patch 200 ms after the frame. */
    {
        uint32_t ext[31];
        unsigned int i;
        for (i = 0U; i < 29U; i++) { ext[i] = runs[i]; }
        ext[29] = 200000U; ext[30] = 80000U;   /* gap, then another bar */
        s = sharp;
        s3_barcode_record_reset(&r);
        k = feed(&r, &clock, &s, ext, 31U);
        assert(k.decoded == 0U);
    }
}

static void test_line_crossing(void)
{
    /* 1.2 s of black (a track line): rejected once, like the fixed reader,
     * and not reported again while it is still black or afterwards. */
    static s3_barcode_record_t r;
    const s3_barcode_config_t cfg = s3_barcode_defaults();
    s3_barcode_result_t res;
    uint32_t t = 0U;
    unsigned int i, rejected = 0U, other = 0U;
    s3_barcode_record_reset(&r);
    for (i = 0U; i < 360U; i++) {
        uint16_t v = (i < 100U || i >= 220U) ? 10U : 1000U;
        s3_barcode_status_t st;
        t += 10000U;
        st = s3_barcode_record_update(&r, &cfg, t, true, v, &res);
        if (st == S3_BARCODE_REJECTED) { rejected++; assert(i < 220U); }
        else if (st != S3_BARCODE_NONE) { other++; }
    }
    assert(rejected == 1U && other == 0U && !s3_barcode_record_active(&r));
}

static void test_limits_and_faults(void)
{
    static s3_barcode_record_t r;
    const s3_barcode_config_t cfg = s3_barcode_defaults();
    s3_barcode_config_t bad = cfg;
    s3_barcode_result_t res;
    uint32_t t = 0U;
    unsigned int i, overflow = 0U;
    /* Recording memory full: 30 ms stripes sampled every 5 ms. */
    s3_barcode_record_reset(&r);
    for (i = 0U; i < 200U; i++) { t += 5000U; (void)s3_barcode_record_update(&r, &cfg, t, true, 10U, &res); }
    for (i = 0U; i < 1500U; i++) {
        t += 5000U;
        if (s3_barcode_record_update(&r, &cfg, t, true, ((i / 6U) % 2U) ? 50U : 900U, &res) ==
            S3_BARCODE_OVERFLOW) { overflow++; }
    }
    assert(overflow >= 1U);
    /* A missing reading (gap > max_gap_us) while recording. */
    s3_barcode_record_reset(&r); t = 0U;
    for (i = 0U; i < 100U; i++) { t += 10000U; assert(s3_barcode_record_update(&r, &cfg, t, true, 10U, &res) == S3_BARCODE_NONE); }
    t += 10000U; (void)s3_barcode_record_update(&r, &cfg, t, true, 900U, &res);
    assert(s3_barcode_record_active(&r));
    t += 40000U;
    assert(s3_barcode_record_update(&r, &cfg, t, true, 900U, &res) == S3_BARCODE_TIMING_ERROR);
    assert(!s3_barcode_record_active(&r));
    /* ... but only reported while a barcode was being read. */
    t += 40000U;
    assert(s3_barcode_record_update(&r, &cfg, t, true, 10U, &res) == S3_BARCODE_NONE);
    /* Invalid reading: start over silently. */
    t += 10000U; (void)s3_barcode_record_update(&r, &cfg, t, true, 900U, &res);
    assert(s3_barcode_record_active(&r));
    t += 10000U;
    assert(s3_barcode_record_update(&r, &cfg, t, false, 900U, &res) == S3_BARCODE_NONE);
    assert(!s3_barcode_record_active(&r) && !r.seen);
    /* Bad arguments. */
    bad.quiet_modules = 0U;
    assert(s3_barcode_record_update(&r, &bad, t, true, 10U, &res) == S3_BARCODE_NONE);
    assert(s3_barcode_record_update(&r, &cfg, t, true, 10U, 0) == S3_BARCODE_NONE);
    assert(s3_barcode_record_update(0, &cfg, t, true, 10U, &res) == S3_BARCODE_NONE);
    assert(!s3_barcode_record_active(0));
    s3_barcode_record_reset(0);
    /* Darkness above 1000 is clamped, never stored as a wrapped value. */
    s3_barcode_record_reset(&r); t = 0U;
    for (i = 0U; i < 50U; i++) { t += 10000U; (void)s3_barcode_record_update(&r, &cfg, t, true, 10U, &res); }
    t += 10000U; (void)s3_barcode_record_update(&r, &cfg, t, true, 0xFFFFU, &res);
    assert(r.count == 2U && (r.sample[1] & 0x3FFU) == 1000U);
}

int main(void)
{
    assert(sizeof(((s3_barcode_record_t *)0)->sample[0]) == 2U);
    test_all_letters();
    test_faint_y_beats_fixed_limits();
    test_scratch_inside_wide_bar();
    test_grey_shelf_in_wide_bar();
    test_never_a_wrong_letter();
    test_cut_pass_is_not_read();
    test_short_quiet_zones();
    test_line_crossing();
    test_limits_and_faults();
    puts("PASS: record-then-decide barcode reader (26 letters x 2 directions x sharp/blurred/faint, "
         "scratch trim, grey shelf, 3000 frames with +/-25 % width jitter and no wrong letter, cut pass, quiet zones, line "
         "crossing, overflow, timing, invalid input)");
    return 0;
}
