#include "barcode_pattern.h"

/* Independently implemented separate-colour width classification, informed by
 * ZXing-C++ RowReader::NarrowWideThreshold and the Code 39 discussion in
 * US5352878A (Figure 14):
 * https://github.com/zxing-cpp/zxing-cpp/blob/master/core/src/oned/ODRowReader.h
 * https://patents.google.com/patent/US5352878A/en
 *
 * Our sensor can shorten black widths while lengthening white widths. This
 * module therefore does not import ZXing-C++'s image-specific spread guards
 * or require a common narrow mean across colours. Its explicit separation
 * guard is experimental; this is not a standards-conformance verifier. */
static const char code39_characters[] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";
static const uint16_t code39_patterns[] = {
    0x034U,0x121U,0x061U,0x160U,0x031U,0x130U,0x070U,0x025U,0x124U,0x064U,
    0x109U,0x049U,0x148U,0x019U,0x118U,0x058U,0x00DU,0x10CU,0x04CU,0x01CU,
    0x103U,0x043U,0x142U,0x013U,0x112U,0x052U,0x007U,0x106U,0x046U,0x016U,
    0x181U,0x0C1U,0x1C0U,0x091U,0x190U,0x0D0U,0x085U,0x184U,0x0C4U,
    0x0A8U,0x0A2U,0x08AU,0x02AU,0x094U
};
_Static_assert(sizeof code39_characters-1U ==
    sizeof code39_patterns/sizeof code39_patterns[0], "Code 39 table size");

static bool widths_valid(const uint32_t *widths, unsigned int count)
{
    uint64_t total=0U;
    if (widths==0) { return false; }
    for (unsigned int i=0U; i<count; i++) {
        if (widths[i]==0U) { return false; }
        total+=widths[i];
    }
    return total<=UINT32_MAX;
}

s3_code39_status_t s3_code39_classify9(
    const uint32_t widths_us[S3_CODE39_SYMBOL_WIDTHS],
    uint32_t max_sample_gap_us, s3_code39_symbol_t *out)
{
    uint32_t smallest[2], largest[2], cutoff[2];
    uint32_t closest=UINT32_MAX;
    bool split_colour=false;
    if (out==0) { return S3_CODE39_INVALID_INPUT; }
    *out=(s3_code39_symbol_t){0};
    if (max_sample_gap_us==0U || !widths_valid(widths_us,S3_CODE39_SYMBOL_WIDTHS)) {
        return S3_CODE39_INVALID_INPUT;
    }
    smallest[0]=largest[0]=widths_us[0];
    smallest[1]=largest[1]=widths_us[1];
    for (unsigned int i=2U; i<S3_CODE39_SYMBOL_WIDTHS; i++) {
        const unsigned int colour=i%2U;
        if (widths_us[i]<smallest[colour]) { smallest[colour]=widths_us[i]; }
        if (widths_us[i]>largest[colour]) { largest[colour]=widths_us[i]; }
    }
    for (unsigned int colour=0U; colour<2U; colour++) {
        const uint64_t midpoint=((uint64_t)smallest[colour]+largest[colour])/2U;
        const uint64_t minimum=(uint64_t)smallest[colour]*3U/2U;
        const uint64_t chosen=midpoint>minimum ? midpoint : minimum;
        /* With four or five positive widths of each colour and a bounded
         * total, chosen fits uint32_t. Keep the check explicit nonetheless. */
        if (chosen>UINT32_MAX) { return S3_CODE39_INVALID_INPUT; }
        cutoff[colour]=(uint32_t)chosen;
    }
    out->bar_threshold_us=cutoff[0];
    out->space_threshold_us=cutoff[1];
    for (unsigned int i=0U; i<S3_CODE39_SYMBOL_WIDTHS; i++) {
        if (widths_us[i]>cutoff[i%2U]) {
            out->pattern=(uint16_t)(out->pattern | (1U<<(8U-i)));
        }
    }
    for (unsigned int colour=0U; colour<2U; colour++) {
        uint32_t widest_narrow=0U, narrowest_wide=UINT32_MAX;
        for (unsigned int i=colour; i<S3_CODE39_SYMBOL_WIDTHS; i+=2U) {
            if (widths_us[i]>cutoff[colour]) {
                if (widths_us[i]<narrowest_wide) { narrowest_wide=widths_us[i]; }
            } else if (widths_us[i]>widest_narrow) { widest_narrow=widths_us[i]; }
        }
        /* Some Code 39 symbols contain five narrow bars and three wide
         * spaces. Their unsplit bar class supplies no separation measure. */
        if (widest_narrow!=0U && narrowest_wide!=UINT32_MAX) {
            const uint32_t separation=narrowest_wide-widest_narrow;
            split_colour=true;
            if (separation<closest) { closest=separation; }
        }
    }
    out->min_separation_us=split_colour ? closest : 0U;
    for (unsigned int i=0U; i<sizeof code39_patterns/sizeof code39_patterns[0]; i++) {
        if (out->pattern==code39_patterns[i]) {
            out->character=code39_characters[i];
            break;
        }
    }
    if (out->character==0) { return S3_CODE39_UNKNOWN_PATTERN; }
    if (!split_colour || (uint64_t)closest<=2ULL*max_sample_gap_us) {
        return S3_CODE39_INSUFFICIENT_SEPARATION;
    }
    return S3_CODE39_OK;
}

s3_code39_status_t s3_code39_classify_frame(
    const uint32_t *widths_us, unsigned int count,
    uint32_t max_sample_gap_us, s3_code39_frame_t *out)
{
    s3_code39_frame_t selected={0};
    bool found=false;
    if (out==0) { return S3_CODE39_INVALID_INPUT; }
    *out=(s3_code39_frame_t){0};
    if (count!=S3_CODE39_FRAME_WIDTHS || max_sample_gap_us==0U ||
        !widths_valid(widths_us,count)) { return S3_CODE39_INVALID_INPUT; }
    for (unsigned int reverse=0U; reverse<2U; reverse++) {
        s3_code39_symbol_t symbols[3];
        uint32_t separation=UINT32_MAX;
        bool valid=true;
        for (unsigned int c=0U; c<3U; c++) {
            uint32_t symbol[S3_CODE39_SYMBOL_WIDTHS];
            for (unsigned int i=0U; i<S3_CODE39_SYMBOL_WIDTHS; i++) {
                const unsigned int at=c*10U+i;
                symbol[i]=widths_us[reverse ? S3_CODE39_FRAME_WIDTHS-1U-at : at];
            }
            if (s3_code39_classify9(symbol,max_sample_gap_us,&symbols[c])!=S3_CODE39_OK) {
                valid=false; break;
            }
            if (symbols[c].min_separation_us<separation) {
                separation=symbols[c].min_separation_us;
            }
        }
        if (!valid || symbols[0].character!='*' || symbols[2].character!='*' ||
            symbols[1].character<'A' || symbols[1].character>'Z') { continue; }
        if (found && (selected.letter!=symbols[1].character ||
            selected.reversed!=(reverse!=0U))) { return S3_CODE39_AMBIGUOUS; }
        selected=(s3_code39_frame_t){separation,symbols[1].character,reverse!=0U};
        found=true;
    }
    if (!found) { return S3_CODE39_INVALID_FRAME; }
    *out=selected;
    return S3_CODE39_OK;
}
