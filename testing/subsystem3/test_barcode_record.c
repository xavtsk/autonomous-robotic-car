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
typedef struct {
    unsigned int decoded, rejected, timing, overflow, max_recorded;
    s3_barcode_result_t last;
} tally_t;

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
        if (r->count > k.max_recorded) { k.max_recorded = r->count; }
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
    shape_t shapes[6];
    unsigned int s, c, dir;
    shapes[0] = sharp;
    shapes[1] = sharp; shapes[1].narrow_us = 60000U; shapes[1].wide_us = 150000U;
    shapes[1].blur_us = 60000U; shapes[1].noise = 15U;
    shapes[2] = sharp; shapes[2].narrow_us = 50000U; shapes[2].wide_us = 130000U;
    shapes[2].bar = 550U; shapes[2].gap = 300U; shapes[2].blur_us = 20000U; shapes[2].noise = 10U;
    for (s = 0U; s < 3U; s++) { shapes[s + 3U] = shapes[s]; shapes[s + 3U].period_us = 5000U; }
    for (s = 0U; s < 6U; s++) {
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

static void test_200hz_long_frame_and_rejections(void)
{
    /* A 5.95-second frame plus its real quiet zone exceeds the old 1024
     * readings at 200 Hz. It must survive without wrapping the count or
     * decoding early, in either travel direction. */
    static s3_barcode_record_t r;
    shape_t s = sharp;
    const char payloads[] = {'A', 'Z'};
    unsigned int c, dir, i;
    s.period_us = 5000U; s.narrow_us = 140000U; s.wide_us = 350000U;
    s.trail_us = 1800000U;
    for (c = 0U; c < sizeof payloads; c++) {
        for (dir = 0U; dir < 2U; dir++) {
            uint32_t runs[29], clock = 0U, frame_us = 0U;
            tally_t k;
            make_runs(payloads[c], &s, runs);
            if (dir) { reverse_runs(runs); }
            for (i = 0U; i < 29U; i++) { frame_us += runs[i]; }
            s3_barcode_record_reset(&r);
            k = feed(&r, &clock, &s, runs, 29U);
            assert(k.max_recorded > 1024U);
            assert(k.decoded == 1U && k.last.letter == payloads[c]);
            assert(k.last.reversed == (dir != 0U));
            assert(k.rejected == 0U && k.timing == 0U && k.overflow == 0U);
            assert(k.last.timestamp_us >= s.lead_us + frame_us + 10U * s.narrow_us);
            assert(!s3_barcode_record_active(&r));
        }
    }
    /* More readings must not relax the existing physical-width rules or
     * invent the opening sentinel in a late-started capture. */
    {
        uint32_t runs[29], clock = 0U;
        tally_t k;
        s = sharp; s.period_us = 5000U;
        s.narrow_us = 20000U; s.wide_us = 60000U;
        make_runs('A', &s, runs);
        s3_barcode_record_reset(&r);
        k = feed(&r, &clock, &s, runs, 29U);
        assert(k.decoded == 0U && k.rejected == 1U && k.overflow == 0U);
        s = sharp; s.period_us = 5000U;
        make_runs('A', &s, runs);
        s3_barcode_record_reset(&r); clock = 0U;
        k = feed(&r, &clock, &s, &runs[10], 19U);
        assert(k.decoded == 0U && k.rejected == 1U && k.overflow == 0U);
        /* Faint bars below the former 150-unit start gate now start a full
         * recording; the same physical trace works at either sample rate. */
        s.bar = s.paper + 100U;
        s3_barcode_record_reset(&r); clock = 0U;
        k = feed(&r, &clock, &s, runs, 29U);
        assert(k.decoded == 1U && k.last.letter == 'A' && k.rejected == 0U);
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

static void test_faint_recording_start(void)
{
    /* The complete frame is visible but every bar is below the old start
     * threshold. Long real paper exercises history wrap before the frame. */
    static s3_barcode_record_t r;
    const char payloads[] = {'A', 'Z'};
    unsigned int rate, c, dir;
    for (rate = 0U; rate < 2U; rate++) {
        for (c = 0U; c < sizeof payloads; c++) {
            for (dir = 0U; dir < 2U; dir++) {
                shape_t s = sharp;
                uint32_t runs[29], clock = 0xFFFF0000U;
                tally_t k;
                s.period_us = rate ? 5000U : 10000U;
                s.narrow_us = 80000U; s.wide_us = 200000U;
                s.bar = 110U; s.blur_us = 40000U; s.lead_us = 4000000U;
                make_runs(payloads[c], &s, runs);
                if (dir) { reverse_runs(runs); }
                s3_barcode_record_reset(&r);
                k = feed(&r, &clock, &s, runs, 29U);
                assert(k.decoded == 1U && k.last.letter == payloads[c]);
                assert(k.last.reversed == (dir != 0U));
                assert(k.rejected == 0U && k.timing == 0U && k.overflow == 0U);
            }
        }
    }
}

/* The complete opening star is visible at only 40 units above paper, below
 * the new start gate. Recording begins on the payload's first stronger bar.
 * fault: 0 normal, 1 invalid sample, 2 missing sample immediately beforehand. */
static tally_t feed_weak_opening(s3_barcode_record_t *r, uint32_t *clock,
                               const shape_t *s, const uint32_t runs[29],
                               unsigned int fault)
{
    const s3_barcode_config_t cfg = s3_barcode_defaults();
    tally_t k = {0};
    uint32_t opening_end = s->lead_us, payload_start, total, t;
    unsigned int i;
    for (i = 0U; i < 9U; i++) { opening_end += runs[i]; }
    payload_start = opening_end + runs[9];
    total = s->lead_us + s->trail_us;
    for (i = 0U; i < 29U; i++) { total += runs[i]; }
    for (t = 0U; t < total; t += s->period_us) {
        uint16_t dark = ideal(s, runs, 29U, t);
        const bool at_fault = t + s->period_us == payload_start;
        s3_barcode_result_t result;
        s3_barcode_status_t status;
        if (t < opening_end && dark == s->bar) { dark = s->paper + 40U; }
        *clock += (fault == 2U && at_fault) ? 40000U : s->period_us;
        status = s3_barcode_record_update(r, &cfg, *clock,
            !(fault == 1U && at_fault), dark, &result);
        if (r->count > k.max_recorded) { k.max_recorded = r->count; }
        if (status == S3_BARCODE_DECODED) { k.decoded++; k.last = result; }
        else if (status == S3_BARCODE_REJECTED) { k.rejected++; }
        else if (status == S3_BARCODE_TIMING_ERROR) { k.timing++; }
        else if (status == S3_BARCODE_OVERFLOW) { k.overflow++; }
    }
    return k;
}

static void test_real_pretrigger_history(void)
{
    static s3_barcode_record_t r;
    const char payloads[] = {'A', 'Z'};
    unsigned int rate, c, dir, fault;
    for (rate = 0U; rate < 2U; rate++) {
        for (c = 0U; c < sizeof payloads; c++) {
            for (dir = 0U; dir < 2U; dir++) {
                shape_t s = sharp;
                uint32_t runs[29], clock = 0xFFFF0000U;
                tally_t k;
                s.period_us = rate ? 5000U : 10000U;
                s.narrow_us = 80000U; s.wide_us = 200000U;
                s.lead_us = 4000000U; /* multiple ring wraps, real quiet */
                make_runs(payloads[c], &s, runs);
                if (dir) { reverse_runs(runs); }
                s3_barcode_record_reset(&r);
                k = feed_weak_opening(&r, &clock, &s, runs, 0U);
                if (!(k.decoded == 1U && k.last.letter == payloads[c])) {
                    printf("weak opening rate=%u letter=%c dir=%u: decoded=%u got=%c rejected=%u\n",
                           rate, payloads[c], dir, k.decoded,
                           k.last.letter ? k.last.letter : '-', k.rejected);
                    assert(0);
                }
                assert(k.last.reversed == (dir != 0U));
                assert(k.rejected == 0U && k.timing == 0U && k.overflow == 0U);
                /* The opening star is not extra leading white. Only 600 ms
                 * of real paper precedes it: below ten 80 ms modules. */
                s.lead_us = 600000U; clock = 0U;
                s3_barcode_record_reset(&r);
                k = feed_weak_opening(&r, &clock, &s, runs, 0U);
                assert(k.decoded == 0U && k.rejected == 1U);
            }
        }
        /* Neither invalid data nor a sampling gap may join old buffered
         * opening bars onto a later payload to invent a complete frame. */
        for (fault = 1U; fault <= 2U; fault++) {
            shape_t s = sharp;
            uint32_t runs[29], clock = 0U;
            tally_t k;
            s.period_us = rate ? 5000U : 10000U;
            s.narrow_us = 80000U; s.wide_us = 200000U;
            s.lead_us = 2000000U;
            make_runs('A', &s, runs);
            s3_barcode_record_reset(&r);
            k = feed_weak_opening(&r, &clock, &s, runs, fault);
            assert(k.decoded == 0U && k.rejected == 1U && k.overflow == 0U);
        }
    }
    /* A delayed trigger on the top of a blurred, faint first bar must still
     * retain both sides of its edge. Check measured widths before decoding
     * so sentinel correction cannot conceal a clipped opening edge. */
    {
        shape_t s = sharp;
        uint32_t runs[29], measured[29], first, last, end, gap, clock = 0U;
        tally_t k;
        s.narrow_us = 120000U; s.wide_us = 300000U;
        s.bar = 70U; s.blur_us = 120000U; s.trail_us = 200000U;
        make_runs('A', &s, runs);
        s3_barcode_record_reset(&r);
        k = feed(&r, &clock, &s, runs, 29U);
        assert(k.decoded == 0U && k.rejected == 0U && s3_barcode_record_active(&r));
        assert(s3_barcode_record_runs(&r, measured, &first, &last, &end, &gap) == 29U);
        assert(measured[0] >= 110000U && measured[0] <= 130000U);
        assert(first > 0U && last < end);
    }
    /* Saved paper before the trigger must not consume the existing frame
     * deadline. This 8.1 s barcode needs another 1.8 s of actual white;
     * adding the buffered paper would incorrectly push it over 10 s. */
    {
        shape_t s = sharp;
        uint32_t runs[29], clock = 0U;
        tally_t k;
        s.narrow_us = 180000U; s.wide_us = 500000U;
        s.lead_us = 2500000U; s.trail_us = 2100000U;
        make_runs('A', &s, runs);
        s3_barcode_record_reset(&r);
        k = feed(&r, &clock, &s, runs, 29U);
        assert(k.decoded == 1U && k.last.letter == 'A');
        assert(k.rejected == 0U && k.timing == 0U && k.overflow == 0U);
    }
}

static void test_faint_patch_rearms_only_on_paper(void)
{
    static s3_barcode_record_t r;
    const s3_barcode_config_t cfg = s3_barcode_defaults();
    uint32_t clock = 0U;
    unsigned int i, rejected = 0U;
    s3_barcode_record_reset(&r);
    for (i = 0U; i < 700U; i++) {
        const uint16_t dark = i < 200U || i >= 600U ? 10U : 70U;
        s3_barcode_result_t result;
        s3_barcode_status_t status;
        clock += 10000U;
        status = s3_barcode_record_update(&r, &cfg, clock, true, dark, &result);
        assert(status != S3_BARCODE_DECODED && status != S3_BARCODE_OVERFLOW);
        if (status == S3_BARCODE_REJECTED) { rejected++; }
    }
    assert(rejected == 1U && !s3_barcode_record_active(&r));
    /* A real subsequent frame is still allowed after actual white paper. */
    {
        uint32_t runs[29];
        tally_t k;
        make_runs('A', &sharp, runs);
        k = feed(&r, &clock, &sharp, runs, 29U);
        assert(k.decoded == 1U && k.last.letter == 'A' && k.rejected == 0U);
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

/* One rectangular reflectance defect in an otherwise ideal printed frame.
 * Reversing the full sampled waveform mirrors the defect as well as the
 * barcode; the same piece of paper is scanned in either direction. */
static tally_t feed_patched_frame(const shape_t *s, const uint32_t runs[29],
                                 uint32_t patch_start, uint32_t patch_us,
                                 uint16_t patch_dark, bool reversed)
{
    static s3_barcode_record_t r;
    const s3_barcode_config_t cfg = s3_barcode_defaults();
    tally_t k = {0};
    uint32_t total = s->lead_us + s->trail_us, t, clock = 0U;
    unsigned int i;
    for (i = 0U; i < 29U; i++) { total += runs[i]; }
    assert(total % s->period_us == 0U);
    s3_barcode_record_reset(&r);
    for (t = 0U; t < total; t += s->period_us) {
        const uint32_t physical_t = reversed ? total - s->period_us - t : t;
        uint16_t dark = ideal(s, runs, 29U, physical_t);
        s3_barcode_result_t result;
        s3_barcode_status_t status;
        if (physical_t >= patch_start && physical_t < patch_start + patch_us) {
            dark = patch_dark;
        }
        clock += s->period_us;
        status = s3_barcode_record_update(&r, &cfg, clock, true, dark, &result);
        if (r.count > k.max_recorded) { k.max_recorded = r.count; }
        if (status == S3_BARCODE_DECODED) { k.decoded++; k.last = result; }
        else if (status == S3_BARCODE_REJECTED) { k.rejected++; }
        else if (status == S3_BARCODE_TIMING_ERROR) { k.timing++; }
        else if (status == S3_BARCODE_OVERFLOW) { k.overflow++; }
    }
    return k;
}

static void test_merged_wide_bar_dip(void)
{
    /* A broad, pale patch inside one wide black bar falls below its local
     * midpoint, then rebounds before the actual white gap. Pruning merges
     * this extra dip/bump pair; edge selection must retain the whole bar.
     * The old crossing nearest the weighted estimate selected the internal
     * rebound and clipped the bar. This shape and expectation do not depend
     * on the payload letter. Test each wide payload bar independently. */
    shape_t s = sharp;
    unsigned int c, bar, dir, cases = 0U;
    s.narrow_us = 100000U; s.wide_us = 300000U;
    for (c = 0U; c < 26U; c++) {
        uint32_t runs[29], start = s.lead_us;
        make_runs((char)('A' + c), &s, runs);
        for (bar = 0U; bar < 29U; bar++) {
            if (bar >= 10U && bar <= 18U && bar % 2U == 0U && runs[bar] == s.wide_us) {
                for (dir = 0U; dir < 2U; dir++) {
                    const tally_t k = feed_patched_frame(&s, runs, start + 60000U,
                                                         180000U, 100U, dir != 0U);
                    if (!(k.decoded == 1U && k.last.letter == (char)('A' + c) &&
                          k.last.reversed == (dir != 0U) && k.rejected == 0U &&
                          k.timing == 0U && k.overflow == 0U)) {
                        printf("merged wide-bar dip: letter=%c bar=%u dir=%u decoded=%u got=%c rejected=%u\n",
                               'A' + c, bar, dir, k.decoded,
                               k.last.letter ? k.last.letter : '-', k.rejected);
                        fflush(stdout);
                        assert(0);
                    }
                    cases++;
                }
            }
            start += runs[bar];
        }
    }
    printf("PASS: merged wide-bar midpoint dip and rebound, %u A-Z/both-direction scans\n", cases);
}

static void test_white_gap_spikes_never_wrong_letter(void)
{
    /* The outer-crossing rule must not turn a brief dark smudge in white
     * paper into a different letter. A 20 ms spike is above the local
     * midpoint, near either side of every narrow payload bar. Refusal is
     * allowed: Code 39 has no checksum and the pristine frame was altered. */
    shape_t s = sharp;
    unsigned int c, bar, dir, side, offset, cases = 0U, decoded = 0U;
    s.narrow_us = 100000U; s.wide_us = 300000U;
    for (c = 0U; c < 26U; c++) {
        uint32_t runs[29], start = s.lead_us;
        make_runs((char)('A' + c), &s, runs);
        for (bar = 0U; bar < 29U; bar++) {
            if (bar >= 10U && bar <= 18U && bar % 2U == 0U && runs[bar] == s.narrow_us) {
                for (side = 0U; side < 2U; side++) {
                    for (offset = 10000U; offset <= 50000U; offset += 40000U) {
                        const uint32_t patch_start = side ? start + runs[bar] + offset :
                                                           start - offset - 20000U;
                        for (dir = 0U; dir < 2U; dir++) {
                            const tally_t k = feed_patched_frame(&s, runs, patch_start,
                                                                 20000U, 700U, dir != 0U);
                            assert(k.decoded <= 1U && k.timing == 0U && k.overflow == 0U);
                            assert(k.decoded + k.rejected == 1U);
                            if (k.decoded != 0U) {
                                assert(k.last.letter == (char)('A' + c));
                                assert(k.last.reversed == (dir != 0U));
                                decoded++;
                            }
                            cases++;
                        }
                    }
                }
            }
            start += runs[bar];
        }
    }
    printf("PASS: isolated 20 ms white-gap spikes, %u scans (%u correct, remainder rejected; no wrong letter)\n",
           cases, decoded);
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
    /* Recording memory full before the frame-duration timeout: 30 ms stripes
     * sampled every 2.5 ms, including after the 200 Hz capacity increase. */
    s3_barcode_record_reset(&r);
    for (i = 0U; i < 400U; i++) { t += 2500U; (void)s3_barcode_record_update(&r, &cfg, t, true, 10U, &res); }
    for (i = 0U; i < S3_REC_MAX + 100U; i++) {
        t += 2500U;
        if (s3_barcode_record_update(&r, &cfg, t, true, ((i / 12U) % 2U) ? 50U : 900U, &res) ==
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
    assert(r.count >= 2U && (r.sample[r.count - 1U] & 0x3FFU) == 1000U);
}

static void test_shared_width_bias(void)
{
    unsigned int c, dir, sign, i;
    const s3_barcode_config_t cfg=s3_barcode_defaults();
    shape_t s=sharp;
    s.narrow_us=80000U; s.wide_us=200000U;
    for (c=0U; c<26U; c++) {
        for (dir=0U; dir<2U; dir++) {
            for (sign=0U; sign<2U; sign++) {
                uint32_t runs[29], corrected[29], clock=0U;
                int32_t bias=sign ? -40000 : 40000, fitted;
                static s3_barcode_record_t r;
                s3_barcode_result_t result;
                tally_t k;
                make_runs((char)('A'+c),&s,runs);
                if (dir) { reverse_runs(runs); }
                for (i=0U; i<29U; i++) {
                    runs[i]=(uint32_t)((int32_t)runs[i]+((i%2U) ? bias : -bias));
                }
                assert(!s3_barcode_decode_runs(runs,29U,s.lead_us,s.trail_us,
                    s.period_us,&cfg,&result));
                memcpy(corrected,runs,sizeof(runs));
                assert(s3_barcode_record_correct_runs(corrected,s.period_us,&cfg,&fitted));
                assert(fitted>=bias-1000 && fitted<=bias+1000);
                s3_barcode_record_reset(&r);
                k=feed(&r,&clock,&s,runs,29U);
                assert(k.decoded==1U && k.last.letter==(char)('A'+c));
                assert(k.last.reversed==(dir!=0U) && k.rejected==0U);
            }
        }
    }
    /* Identical sentinels give an identical fit for every payload, including
     * an invalid one. A fit alone must never authorise a letter. */
    {
        uint32_t runs[29], corrected[29], clock=0U;
        int32_t fitted;
        static s3_barcode_record_t r;
        tally_t k;
        make_runs('A',&s,runs);
        for (i=10U; i<19U; i++) { runs[i]=s.narrow_us; }
        for (i=0U; i<29U; i++) { runs[i]=(i%2U) ? runs[i]+40000U : runs[i]-40000U; }
        memcpy(corrected,runs,sizeof(runs));
        assert(s3_barcode_record_correct_runs(corrected,s.period_us,&cfg,&fitted));
        assert(fitted>=39000 && fitted<=41000);
        s3_barcode_record_reset(&r);
        k=feed(&r,&clock,&s,runs,29U);
        assert(k.decoded==0U && k.rejected==1U);
        /* Opposite bias at the end cannot be explained by one shared fit. */
        make_runs('A',&s,runs);
        for (i=0U; i<29U; i++) {
            int32_t bias=(i<20U) ? 40000 : -40000;
            runs[i]=(uint32_t)((int32_t)runs[i]+((i%2U) ? bias : -bias));
        }
        memcpy(corrected,runs,sizeof(runs));
        assert(!s3_barcode_record_correct_runs(corrected,s.period_us,&cfg,&fitted));
        assert(memcmp(corrected,runs,sizeof(runs))==0);
        /* Bar expansion consumes the leading margin as well. */
        make_runs('A',&s,runs);
        for (i=0U; i<29U; i++) { runs[i]=(i%2U) ? runs[i]+40000U : runs[i]-40000U; }
        s.lead_us=810000U; /* appears >= 10 modules until the edge is corrected */
        s3_barcode_record_reset(&r);
        k=feed(&r,&clock,&s,runs,29U);
        assert(k.decoded==0U && k.rejected==1U);
    }
}

static void test_adaptive_peaks_and_quiet(void)
{
    const s3_barcode_config_t cfg=s3_barcode_defaults();
    unsigned int mode;
    for (mode=0U; mode<6U; mode++) {
        static s3_barcode_record_t r;
        shape_t s=sharp;
        const unsigned int scenario=mode%3U;
        uint32_t runs[29], t, total, finish, clock=0U;
        unsigned int i, decoded=0U;
        bool pending=false;
        if (mode>=3U) { s.period_us=5000U; }
        s.narrow_us=140000U; s.wide_us=350000U; s.trail_us=1800000U;
        make_runs('Y',&s,runs);
        finish=s.lead_us;
        for (i=0U; i<29U; i++) { finish+=runs[i]; }
        total=finish+s.trail_us;
        s3_barcode_record_reset(&r);
        for (t=0U; t<total; t+=s.period_us) {
            uint16_t dark=ideal(&s,runs,29U,t);
            uint32_t edge=s.lead_us;
            s3_barcode_result_t result;
            s3_barcode_status_t status;
            /* Two genuinely faint bars, including the final bar: the fixed
             * 100-unit prominence loses them, the quiet-paper gate sees them. */
            for (i=0U; i<29U; i++) {
                if ((t>=edge) && (t<edge+runs[i]) && ((i==10U)||(i==28U))) { dark=70U; }
                edge+=runs[i];
            }
            if ((scenario==1U) && (t>=finish+1100000U) && (t<finish+1180000U)) { dark=70U; }
            /* Same physical noise waveform at both sample rates. */
            if ((scenario==2U) && (t<s.lead_us)) { dark=(t/10000U)%2U ? 65U : 10U; }
            clock+=s.period_us;
            status=s3_barcode_record_update(&r,&cfg,clock,true,dark,&result);
            if (r.state==S3_REC_QUIET) { pending=true; }
            if (status==S3_BARCODE_DECODED) {
                decoded++; assert(result.letter=='Y');
                assert(t>=finish+cfg.quiet_modules*s.narrow_us-s.period_us);
            }
        }
        if (scenario==0U) { assert(decoded==1U && pending); }
        else { assert(decoded==0U); } /* broken quiet / peaks below learned noise */
        if (scenario==1U) { assert(pending); }
    }
}

int main(void)
{
    assert(sizeof(((s3_barcode_record_t *)0)->sample[0]) == 2U);
    test_all_letters();
    test_200hz_long_frame_and_rejections();
    test_faint_y_beats_fixed_limits();
    test_faint_recording_start();
    test_real_pretrigger_history();
    test_faint_patch_rearms_only_on_paper();
    test_scratch_inside_wide_bar();
    test_grey_shelf_in_wide_bar();
    test_merged_wide_bar_dip();
    test_white_gap_spikes_never_wrong_letter();
    test_never_a_wrong_letter();
    test_cut_pass_is_not_read();
    test_short_quiet_zones();
    test_line_crossing();
    test_limits_and_faults();
    test_shared_width_bias();
    test_adaptive_peaks_and_quiet();
    puts("PASS: record-then-decide barcode reader (26 letters x 2 directions x sharp/blurred/faint at 100/200 Hz, "
         "200 Hz long frames beyond 1024 samples, unchanged minimum widths and complete-frame requirement, "
         "scratch trim, grey shelf, 3000 frames with +/-25 % width jitter and no wrong letter, cut pass, quiet zones, line "
         "crossing, overflow, timing, invalid input, sentinel-only bias both signs/all letters/directions, "
         "invalid payload, inconsistent stars, corrected quiet, adaptive faint peaks/noise, faint starts, real pre-trigger history, "
         "weak opening stars, history wrap/reset, no invented quiet and faint-patch rearming)");
    return 0;
}
