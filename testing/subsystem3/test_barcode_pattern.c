/* Pure duration-pattern tests, independent of ADC/edge extraction.
 *
 * A successful frame result here means exactly '* A-Z *' in one orientation.
 * It does NOT establish a physical barcode: leading/trailing quiet, the
 * intercharacter gap's size, sampling freshness, speed and contrast belong to
 * the future frontend/frame validator. Recorded durations below deliberately
 * include hardware failures; classifying their extracted pattern is not a
 * claim that those captures decoded on the robot or had valid quiet zones.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "barcode_pattern.h"

/* Independent expected Code 39 table (MSB is the first element).
 * Standard alphabet order, including the four space-heavy special symbols.
 * This table is test data, never imported from the production implementation.
 */
static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";
static const uint16_t encodings[] = {
    0x034, 0x121, 0x061, 0x160, 0x031, 0x130, 0x070, 0x025, 0x124, 0x064,
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00d, 0x10c, 0x04c, 0x01c,
    0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106, 0x046, 0x016,
    0x181, 0x0c1, 0x1c0, 0x091, 0x190, 0x0d0, 0x085, 0x184, 0x0c4,
    0x0a8, 0x0a2, 0x08a, 0x02a, 0x094
};
static unsigned int checks;
static uint32_t rng = UINT32_C(0x43503926);

static unsigned int symbol_index(char c)
{
    const char *p = strchr(alphabet, c);
    assert(p != NULL);
    return (unsigned int)(p - alphabet);
}

static void symbol(uint32_t widths[9], char c, uint32_t narrow,
                   uint32_t wide, int32_t bias)
{
    uint16_t pattern = encodings[symbol_index(c)];
    unsigned int i;
    for (i = 0; i < 9U; ++i) {
        int64_t v = (pattern & (1U << (8U - i))) ? wide : narrow;
        /* A physical synthetic distortion, not a classifier input:
         * narrower measured black bars, wider measured white spaces.
         */
        v += (i & 1U) ? bias : -(int64_t)bias;
        assert(v > 0 && v <= UINT32_MAX);
        widths[i] = (uint32_t)v;
    }
}

static void frame(uint32_t widths[29], char c, uint32_t narrow,
                  uint32_t wide, int32_t bias)
{
    symbol(widths, '*', narrow, wide, bias);
    symbol(widths + 10, c, narrow, wide, bias);
    symbol(widths + 20, '*', narrow, wide, bias);
    widths[9] = widths[19] = narrow;
}

static void reverse(uint32_t *widths, unsigned int count)
{
    unsigned int i;
    for (i = 0; i < count / 2U; ++i) {
        uint32_t temp = widths[i];
        widths[i] = widths[count - 1U - i];
        widths[count - 1U - i] = temp;
    }
}

static void jitter(uint32_t *widths, unsigned int count, unsigned int permille)
{
    unsigned int i;
    for (i = 0; i < count; ++i) {
        int32_t offset;
        rng = rng * UINT32_C(1664525) + UINT32_C(1013904223);
        offset = (int32_t)((rng >> 8) % (2U * permille + 1U)) -
                 (int32_t)permille;
        widths[i] = (uint32_t)((uint64_t)widths[i] *
                              (uint32_t)(1000 + offset) / 1000U);
        assert(widths[i] != 0U);
    }
}

static void expect_frame(const uint32_t widths[29], uint32_t gap,
                         char letter, bool reversed)
{
    s3_code39_frame_t out;
    s3_code39_status_t status = s3_code39_classify_frame(widths, 29U, gap, &out);
    if (status != S3_CODE39_OK || out.letter != letter || out.reversed != reversed) {
        fprintf(stderr, "frame check %u: expected %c/%u, got status=%u %c/%u\n",
                checks, letter, (unsigned)reversed, (unsigned)status,
                out.letter ? out.letter : '-', (unsigned)out.reversed);
        assert(false);
    }
    assert((uint64_t)out.min_separation_us > 2ULL * gap);
    ++checks;
}

static void expect_refusal(const uint32_t *widths, unsigned int count,
                           uint32_t gap)
{
    s3_code39_frame_t out = {.letter = 'X', .reversed = true,
                             .min_separation_us = UINT32_MAX};
    assert(s3_code39_classify_frame(widths, count, gap, &out) != S3_CODE39_OK);
    assert(out.letter == 0 && !out.reversed && out.min_separation_us == 0U);
    ++checks;
}

static void test_all_symbols(void)
{
    unsigned int i, ratio;
    assert(sizeof(encodings) / sizeof(encodings[0]) == sizeof(alphabet) - 1U);
    for (i = 0; i < sizeof(alphabet) - 1U; ++i) {
        for (ratio = 2; ratio <= 3U; ++ratio) {
            uint32_t widths[9];
            s3_code39_symbol_t out;
            symbol(widths, alphabet[i], 100000U, ratio * 100000U, 0);
            assert(s3_code39_classify9(widths, 10000U, &out) == S3_CODE39_OK);
            assert(out.character == alphabet[i] && out.pattern == encodings[i]);
            assert(out.min_separation_us == (ratio - 1U) * 100000U);
            ++checks;
        }
    }
    /* '$', '/', '+', '%' have only narrow black bars. They still classify
     * from the three wide spaces; an absent black wide class is not an error.
     */
    for (i = symbol_index('$'); i <= symbol_index('%'); ++i) {
        uint32_t widths[9];
        s3_code39_symbol_t out;
        symbol(widths, alphabet[i], 100000U, 250000U, 0);
        assert(s3_code39_classify9(widths, 10000U, &out) == S3_CODE39_OK);
        assert(out.bar_threshold_us == 150000U);
        assert(out.space_threshold_us == 175000U);
        assert(out.min_separation_us == 150000U);
        ++checks;
    }
}

static void test_clean_frames(void)
{
    const uint32_t scales[] = {1000U, 10000U, 80000U, 500000U};
    const unsigned int ratios[] = {200U, 250U, 300U};
    unsigned int letter, dir, scale, ratio, b;
    for (letter = 0; letter < 26U; ++letter) {
        for (dir = 0; dir < 2U; ++dir) {
            for (scale = 0; scale < sizeof(scales) / sizeof(scales[0]); ++scale) {
                uint32_t narrow = scales[scale];
                for (ratio = 0; ratio < 3U; ++ratio) {
                    for (b = 0; b < 3U; ++b) {
                        uint32_t widths[29];
                        int32_t bias = ((int32_t)b - 1) * (int32_t)(narrow / 4U);
                        frame(widths, (char)('A' + letter), narrow,
                              narrow * ratios[ratio] / 100U, bias);
                        jitter(widths, 29U, 30U);
                        if (dir) { reverse(widths, 29U); }
                        expect_frame(widths, narrow / 20U,
                                     (char)('A' + letter), dir != 0U);
                    }
                }
            }
        }
    }
}

static void test_distorted_frames_no_wrong_letter(void)
{
    const int32_t biases[] = {-30000, -15000, 0, 30000, 55000};
    const uint32_t wides[] = {160000U, 200000U, 240000U};
    unsigned int letter, dir, ratio, b, repeat, accepted = 0U, rejected = 0U;
    for (letter = 0; letter < 26U; ++letter) {
        for (dir = 0; dir < 2U; ++dir) {
            for (ratio = 0; ratio < 3U; ++ratio) {
                for (b = 0; b < sizeof(biases) / sizeof(biases[0]); ++b) {
                    for (repeat = 0; repeat < 20U; ++repeat) {
                        uint32_t widths[29];
                        s3_code39_frame_t out;
                        s3_code39_status_t status;
                        frame(widths, (char)('A' + letter), 80000U,
                              wides[ratio], biases[b]);
                        jitter(widths, 29U, 80U);
                        if (dir) { reverse(widths, 29U); }
                        status = s3_code39_classify_frame(widths, 29U, 10000U, &out);
                        if (status == S3_CODE39_OK) {
                            assert(out.letter == (char)('A' + letter));
                            assert(out.reversed == (dir != 0U));
                            assert(out.min_separation_us > 20000U);
                            ++accepted;
                        } else {
                            assert(out.letter == 0);
                            ++rejected;
                        }
                        ++checks;
                    }
                }
            }
        }
    }
    assert(accepted > 0U && accepted + rejected == 15600U);
    printf("Pattern distorted frames: %u accepted, %u safely rejected, zero wrong letters\n",
           accepted, rejected);
}

static void test_separation_boundary(void)
{
    uint32_t widths[9];
    s3_code39_symbol_t out;
    symbol(widths, 'A', 100U, 200U, 0);
    assert(s3_code39_classify9(widths, 49U, &out) == S3_CODE39_OK);
    assert(out.min_separation_us == 100U);
    assert(s3_code39_classify9(widths, 50U, &out) == S3_CODE39_INSUFFICIENT_SEPARATION);
    assert(out.pattern == encodings[symbol_index('A')]);
    assert(out.min_separation_us == 100U);
    symbol(widths, 'A', 100U, 199U, 0);
    assert(s3_code39_classify9(widths, 50U, &out) == S3_CODE39_INSUFFICIENT_SEPARATION);
    symbol(widths, 'A', 100U, 201U, 0);
    assert(s3_code39_classify9(widths, 50U, &out) == S3_CODE39_OK);
    symbol(widths, 'A', 100U, 150U, 0);
    assert(s3_code39_classify9(widths, 1U, &out) != S3_CODE39_OK);
    checks += 5U;
}

static void test_one_colour_or_symbol_at_boundary(void)
{
    unsigned int colour, group, dir;
    for (colour = 0; colour < 2U; ++colour) {
        for (group = 0; group < 3U; ++group) {
            for (dir = 0; dir < 2U; ++dir) {
                uint32_t widths[29], ordered[29];
                uint32_t *target = widths + group * 10U;
                unsigned int i, first_wide = 9U;
                char expected = group == 1U ? 'A' : '*';
                s3_code39_symbol_t out;
                s3_code39_frame_t decoded;
                frame(widths, 'A', 100U, 300U, 0);
                /* Only one colour in one symbol approaches the margin.
                 * The opposite colour and other two symbols stay clean.
                 */
                for (i = colour; i < 9U; i += 2U) {
                    if (target[i] == 300U) {
                        target[i] = 201U;
                        if (first_wide == 9U) { first_wide = i; }
                    }
                }
                assert(first_wide < 9U);
                assert(s3_code39_classify9(target, 50U, &out) == S3_CODE39_OK);
                assert(out.character == expected && out.min_separation_us == 101U);
                ++checks;
                memcpy(ordered, widths, sizeof(ordered));
                if (dir) { reverse(ordered, 29U); }
                expect_frame(ordered, 50U, 'A', dir != 0U);
                assert(s3_code39_classify_frame(ordered, 29U, 50U, &decoded) == S3_CODE39_OK);
                assert(decoded.min_separation_us == 101U);

                /* One measured wide element now has exactly 2*gap of
                 * separation. Its bits still form the expected symbol, so
                 * refusal must come from sampling separation, not lookup.
                 */
                target[first_wide] = 200U;
                assert(s3_code39_classify9(target, 50U, &out) ==
                       S3_CODE39_INSUFFICIENT_SEPARATION);
                assert(out.pattern == encodings[symbol_index(expected)]);
                assert(out.min_separation_us == 100U);
                ++checks;
                memcpy(ordered, widths, sizeof(ordered));
                if (dir) { reverse(ordered, 29U); }
                expect_refusal(ordered, 29U, 50U);
            }
        }
    }
}

static void test_invalid_inputs_and_overflow(void)
{
    uint32_t widths[29];
    s3_code39_symbol_t out = {.pattern = 0xffffU, .character = 'X',
        .bar_threshold_us = UINT32_MAX, .space_threshold_us = UINT32_MAX,
        .min_separation_us = UINT32_MAX};
    s3_code39_frame_t decoded;
    unsigned int i;
    frame(widths, 'A', 100U, 300U, 0);
    assert(s3_code39_classify9(NULL, 1U, &out) == S3_CODE39_INVALID_INPUT);
    assert(out.pattern == 0U && out.character == 0 && out.bar_threshold_us == 0U &&
           out.space_threshold_us == 0U && out.min_separation_us == 0U);
    assert(s3_code39_classify9(widths, 1U, NULL) == S3_CODE39_INVALID_INPUT);
    assert(s3_code39_classify9(widths, 0U, &out) == S3_CODE39_INVALID_INPUT);
    assert(s3_code39_classify_frame(widths, 29U, 1U, NULL) == S3_CODE39_INVALID_INPUT);
    expect_refusal(NULL, 29U, 1U);
    expect_refusal(widths, 0U, 1U);
    expect_refusal(widths, 28U, 1U);
    expect_refusal(widths, 30U, 1U); /* Count checked before dereference. */
    expect_refusal(widths, 29U, 0U);
    for (i = 0; i < 29U; ++i) {
        uint32_t saved = widths[i];
        widths[i] = 0U;
        expect_refusal(widths, 29U, 1U);
        if (i < 9U) {
            assert(s3_code39_classify9(widths, 1U, &out) == S3_CODE39_INVALID_INPUT);
            ++checks;
        }
        widths[i] = saved;
    }
    symbol(widths, 'A', UINT32_MAX / 15U, 3U * (UINT32_MAX / 15U), 0);
    assert(s3_code39_classify9(widths, 1U, &out) == S3_CODE39_OK);
    assert(out.character == 'A'); /* Nine-element sum is exactly UINT32_MAX. */
    ++widths[0];
    assert(s3_code39_classify9(widths, 1U, &out) == S3_CODE39_INVALID_INPUT);
    for (i = 0; i < 29U; ++i) { widths[i] = UINT32_MAX; }
    assert(s3_code39_classify9(widths, 1U, &out) == S3_CODE39_INVALID_INPUT);
    expect_refusal(widths, 29U, 1U);
    frame(widths, 'A', UINT32_MAX / 47U, 3U * (UINT32_MAX / 47U), 0);
    widths[9] += UINT32_MAX % 47U;
    assert(s3_code39_classify_frame(widths, 29U, 1U, &decoded) == S3_CODE39_OK);
    assert(decoded.letter == 'A' && !decoded.reversed);
    ++widths[9];
    assert(s3_code39_classify_frame(widths, 29U, 1U, &decoded) == S3_CODE39_INVALID_INPUT);
    symbol(widths, 'A', 100U, 300U, 0);
    assert(s3_code39_classify9(widths, UINT32_MAX, &out) == S3_CODE39_INSUFFICIENT_SEPARATION);
    checks += 10U;
}

static void test_invalid_patterns_and_contract(void)
{
    uint32_t widths[29];
    s3_code39_symbol_t out;
    unsigned int i, dir;
    for (i = 0; i < 9U; ++i) { widths[i] = 100000U; }
    assert(s3_code39_classify9(widths, 10000U, &out) == S3_CODE39_UNKNOWN_PATTERN);
    assert(out.pattern == 0U && out.character == 0);
    ++checks;
    for (i = 0; i < sizeof(alphabet) - 1U; ++i) {
        char c = alphabet[i];
        if (c >= 'A' && c <= 'Z') { continue; }
        frame(widths, c, 100000U, 250000U, 0);
        expect_refusal(widths, 29U, 10000U);
        reverse(widths, 29U);
        expect_refusal(widths, 29U, 10000U);
    }
    for (dir = 0; dir < 2U; ++dir) {
        frame(widths, 'A', 100000U, 250000U, 0);
        symbol(widths, 'B', 100000U, 250000U, 0);
        if (dir) { reverse(widths, 29U); }
        expect_refusal(widths, 29U, 10000U);
        frame(widths, 'A', 100000U, 250000U, 0);
        symbol(widths + 20, 'C', 100000U, 250000U, 0);
        if (dir) { reverse(widths, 29U); }
        expect_refusal(widths, 29U, 10000U);
    }
    /* One star facing each way cannot constitute two plausible orientations.
     * A true two-orientation ambiguity is unreachable with the asymmetric
     * star and reversal-invariant thresholds; do not claim to test that arm.
     */
    frame(widths, 'A', 100000U, 250000U, 0);
    reverse(widths + 20, 9U);
    expect_refusal(widths, 29U, 10000U);
    reverse(widths, 29U);
    expect_refusal(widths, 29U, 10000U);

    /* Deliberately document the pure pattern boundary: nonzero gap durations
     * are ignored by this API, even when physically implausible. A future
     * caller MUST perform gap and quiet-zone validation before acting.
     */
    frame(widths, 'A', 100000U, 250000U, 0);
    widths[9] = 1U;
    widths[19] = 10000000U;
    expect_frame(widths, 10000U, 'A', false);
}

static void test_measured_width_patterns(void)
{
    /* Fixed measured widths from separate-width-research/recorded-results.json.
     * Raw BCAP input, calibration215/3550, midpoint reconstructed sample times.
     * These are uncorrected extraction results. The diagnostic extraction used
     * explicit synthetic quiet padding, NOT evidence of measured quiet.
     */
    static const struct {
        const char *source;
        char letter;
        bool reversed;
        uint32_t gap;
        uint32_t widths[29];
    } measured[] = {
        {"20261105-182223-small-a-rejected: faint", 'A', false, 10500U,
         {22625,257900,35100,136956,120649,172201,118144,135112,28256,153539,
          110570,139173,24763,112387,29416,250949,28585,133688,124191,132643,
          30050,249290,25938,146300,116974,152059,119982,133685,20187}},
        {"20261105-185445-small-a-width-rejected: faint", 'A', false, 12500U,
         {24482,255698,32125,127549,123218,169535,125737,140077,26532,138930,
          124320,140125,26180,107945,27700,260300,29071,136381,132743,124555,
          23150,264037,23370,140970,120870,158452,116654,132837,20955}},
        {"20261105-190459-small-a-width-rejected: faint", 'A', false, 12500U,
         {30696,260443,24125,128477,123648,166638,133034,134058,18870,138753,
          136759,132688,31575,109048,26377,260776,33274,136563,133755,126032,
          26425,252056,33461,147809,112499,146334,122216,135250,33150}},
        {"20261103-152854-successful-z: primary", 'Z', true, 21500U,
         {139298,161095,448070,156897,456819,156524,149142,462675,139210,155399,
          143123,152530,136529,153416,437795,166535,447770,466061,145382,159691,
          137345,165699,441801,152312,444672,161210,134712,470971,143006}},
        {"20261103-212737-wide-f-shifted-rejected: primary", 'F', true, 11500U,
         {151118,166339,434873,185763,379451,248465,86011,501133,111996,174988,
          128249,172705,129397,465884,438416,149347,468441,174706,145512,150462,
          137187,150406,382426,202864,392730,178671,112389,514985,139727}},
        {"20261105-183024-wide-z-decoded: primary", 'Z', false, 19500U,
         {87705,474014,101486,197539,351883,273971,324867,202396,86434,189538,
          96079,487610,361407,239018,334774,241324,66068,221502,65685,222077,
          94022,485085,99720,201158,369894,243334,349179,222248,88400}}
    };
    unsigned int i;
    for (i = 0; i < sizeof(measured) / sizeof(measured[0]); ++i) {
        uint32_t reversed[29];
        (void)measured[i].source;
        expect_frame(measured[i].widths, measured[i].gap,
                     measured[i].letter, measured[i].reversed);
        memcpy(reversed, measured[i].widths, sizeof(reversed));
        reverse(reversed, 29U);
        expect_frame(reversed, measured[i].gap,
                     measured[i].letter, !measured[i].reversed);
    }
}

int main(void)
{
    test_all_symbols();
    test_clean_frames();
    test_distorted_frames_no_wrong_letter();
    test_separation_boundary();
    test_one_colour_or_symbol_at_boundary();
    test_invalid_inputs_and_overflow();
    test_invalid_patterns_and_contract();
    test_measured_width_patterns();
    printf("Barcode pattern tests: %u checks passed (pattern-only, no physical-frame claim)\n",
           checks);
    return 0;
}
