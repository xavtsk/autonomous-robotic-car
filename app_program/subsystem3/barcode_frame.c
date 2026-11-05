#include "barcode_frame.h"
#include "barcode_pattern.h"

/* Class means are kept as small rational numbers until comparisons finish.
 * With fixed nine-element symbols and max_run bounded by config validation,
 * every product below fits uint64_t. */
typedef struct { uint64_t sum; unsigned int count; } class_mean_t;
typedef struct {
    class_mean_t narrow[2], wide[2];
    uint64_t narrow_num, wide_num;
    unsigned int narrow_den, wide_den;
    uint32_t module;
    uint32_t quiet;
} symbol_geometry_t;

bool s3_barcode_frame_config_valid(const s3_barcode_config_t *config)
{
    return config!=0 && config->max_gap_us>0U && config->min_narrow_us>0U &&
        config->max_run_us>=config->min_narrow_us &&
        config->max_gap_us<=config->max_run_us &&
        config->max_frame_us>config->max_run_us &&
        config->max_frame_us<0x80000000U &&
        config->quiet_modules>=10U && config->quiet_modules<=50U &&
        (uint64_t)config->max_run_us*config->quiet_modules<0x80000000ULL;
}

static uint32_t ceil_div(uint64_t numerator, unsigned int denominator)
{
    return (uint32_t)((numerator+denominator-1U)/denominator);
}

static bool geometry(const uint32_t *widths, uint32_t gap,
    const s3_barcode_config_t *config, symbol_geometry_t *out)
{
    s3_code39_symbol_t symbol;
    symbol_geometry_t result={0};
    if (s3_code39_classify9(widths,gap,&symbol)!=S3_CODE39_OK) { return false; }
    for (unsigned int i=0U; i<9U; i++) {
        class_mean_t *mean=(symbol.pattern & (1U<<(8U-i))) ?
            &result.wide[i%2U] : &result.narrow[i%2U];
        mean->sum+=widths[i];
        mean->count++;
    }
    /* Frame payloads are A-Z, and '*' plus all A-Z symbols have both classes
     * of both colours. A full-alphabet pattern-only success alone is not enough. */
    if (result.narrow[0].count==0U || result.narrow[1].count==0U ||
        result.wide[0].count==0U || result.wide[1].count==0U) { return false; }
    result.narrow_num=result.narrow[0].sum*result.narrow[1].count+
        result.narrow[1].sum*result.narrow[0].count;
    result.narrow_den=2U*result.narrow[0].count*result.narrow[1].count;
    result.wide_num=result.wide[0].sum*result.wide[1].count+
        result.wide[1].sum*result.wide[0].count;
    result.wide_den=2U*result.wide[0].count*result.wide[1].count;
    if (result.narrow_num<(uint64_t)config->min_narrow_us*result.narrow_den ||
        result.narrow_num<3ULL*gap*result.narrow_den) { return false; }
    {
        /* These colour-balanced class means are signal estimates, not known
         * printed dimensions. Give each mean one observed sample interval
         * of uncertainty: the measured ratio interval must overlap 1.8..3.5.
         * Keep the rational endpoints exact instead of rounding a ratio.
         * This sampling allowance does not cover arbitrary optical/noise
         * distortion, and does not weaken exact patterns or class separation.
         * The narrow >= 3*gap check above makes its lower endpoint positive. */
        const uint64_t narrow_error=(uint64_t)gap*result.narrow_den;
        const uint64_t wide_error=(uint64_t)gap*result.wide_den;
        if (result.wide_num<=wide_error ||
            10ULL*(result.wide_num+wide_error)*result.narrow_den<
                18ULL*(result.narrow_num-narrow_error)*result.wide_den ||
            10ULL*(result.wide_num-wide_error)*result.narrow_den>
                35ULL*(result.narrow_num+narrow_error)*result.wide_den) { return false; }
    }
    result.module=ceil_div(result.narrow_num,result.narrow_den);
    {
        const uint64_t black=result.narrow[0].sum*result.narrow[1].count;
        const uint64_t white=result.narrow[1].sum*result.narrow[0].count;
        const unsigned int denominator=4U*result.narrow[0].count*result.narrow[1].count;
        const uint32_t extension=white>black ? ceil_div(white-black,denominator) : 0U;
        const uint64_t quiet=(uint64_t)config->quiet_modules*result.module+extension+gap;
        if (quiet>UINT32_MAX) { return false; }
        result.quiet=(uint32_t)quiet;
    }
    *out=result;
    return true;
}

s3_barcode_frame_status_t s3_barcode_frame_validate(
    const uint32_t *widths_us, unsigned int count,
    uint32_t leading_us, uint32_t trailing_us, uint32_t observed_gap_us,
    const s3_barcode_config_t *config, s3_barcode_frame_t *out)
{
    s3_code39_frame_t pattern;
    symbol_geometry_t symbols[3];
    s3_barcode_frame_t result={0};
    uint64_t total=0U;
    if (out==0) { return S3_FRAME_INVALID_INPUT; }
    *out=(s3_barcode_frame_t){0};
    if (widths_us==0 || count!=S3_CODE39_FRAME_WIDTHS || observed_gap_us==0U ||
        !s3_barcode_frame_config_valid(config)) { return S3_FRAME_INVALID_INPUT; }
    if (observed_gap_us>config->max_gap_us) { return S3_FRAME_TIMING; }
    for (unsigned int i=0U; i<count; i++) {
        if (widths_us[i]==0U) { return S3_FRAME_INVALID_INPUT; }
        if (widths_us[i]>config->max_run_us) { return S3_FRAME_GEOMETRY; }
        total+=widths_us[i];
    }
    if (total>config->max_frame_us) { return S3_FRAME_GEOMETRY; }
    if (s3_code39_classify_frame(widths_us,count,observed_gap_us,&pattern)!=S3_CODE39_OK) {
        return S3_FRAME_PATTERN;
    }
    for (unsigned int i=0U; i<3U; i++) {
        if (!geometry(widths_us+i*10U,observed_gap_us,config,&symbols[i])) {
            return S3_FRAME_GEOMETRY;
        }
        result.module_us[i]=symbols[i].module;
    }
    for (unsigned int i=0U; i<2U; i++) {
        const class_mean_t a=symbols[i].narrow[1], b=symbols[i+1U].narrow[1];
        const uint64_t reference=a.sum*b.count+b.sum*a.count;
        const unsigned int denominator=2U*a.count*b.count;
        const uint64_t space=(uint64_t)widths_us[9U+i*10U]*denominator;
        if (2U*space<reference || space>2U*reference) { return S3_FRAME_GEOMETRY; }
    }
    result.required_leading_us=symbols[0].quiet;
    result.required_trailing_us=symbols[2].quiet;
    if (total+result.required_trailing_us>config->max_frame_us) {
        return S3_FRAME_GEOMETRY;
    }
    if (leading_us<result.required_leading_us) { return S3_FRAME_LEADING_QUIET; }
    result.letter=pattern.letter;
    result.reversed=pattern.reversed;
    result.min_separation_us=pattern.min_separation_us;
    *out=result;
    return trailing_us<result.required_trailing_us ? S3_FRAME_WAIT_QUIET : S3_FRAME_OK;
}
