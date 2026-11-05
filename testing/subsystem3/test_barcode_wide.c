/* Synthetic physical-size coverage at 40 mm/s. These tests check the decoder's
 * permitted geometry/timing, not the dimensions or success of an unmeasured
 * printed barcode. One measured Z payload below has synthetic markers/quiet. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "barcode_decoder.h"

static const uint16_t letters[26] = {
    0x109,0x049,0x148,0x019,0x118,0x058,0x00D,0x10C,0x04C,0x01C,
    0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,
    0x181,0x0C1,0x1C0,0x091,0x190,0x0D0};

static s3_barcode_record_t record;
static uint32_t clock_us;
typedef struct {
    unsigned int decoded, rejected, timing, overflow;
    s3_barcode_result_t result;
} tally_t;

static void make_runs(unsigned int letter, uint32_t narrow, uint32_t wide,
                      uint32_t runs[29])
{
    const uint16_t symbols[3] = {0x094U, letters[letter], 0x094U};
    for (unsigned int c=0U; c<3U; c++) {
        for (unsigned int i=0U; i<9U; i++) {
            runs[c*10U+i] = (symbols[c] & (1U<<(8U-i))) ? wide : narrow;
        }
        if (c<2U) { runs[c*10U+9U] = narrow; }
    }
}

static void reverse_runs(uint32_t runs[29])
{
    for (unsigned int i=0U; i<14U; i++) {
        uint32_t value=runs[i]; runs[i]=runs[28U-i]; runs[28U-i]=value;
    }
}

static void feed(const s3_barcode_config_t *cfg, uint16_t darkness, tally_t *out)
{
    s3_barcode_result_t result;
    clock_us += 10000U;
    switch (s3_barcode_record_update(&record,cfg,clock_us,true,darkness,&result)) {
    case S3_BARCODE_DECODED: out->decoded++; out->result=result; break;
    case S3_BARCODE_REJECTED: out->rejected++; break;
    case S3_BARCODE_TIMING_ERROR: out->timing++; break;
    case S3_BARCODE_OVERFLOW: out->overflow++; break;
    default: break;
    }
}

static tally_t trace(const s3_barcode_config_t *cfg, const uint32_t runs[29],
                      unsigned int count, uint32_t leading, uint32_t trailing)
{
    tally_t out={0};
    uint32_t end=leading+trailing;
    for (unsigned int i=0U; i<count; i++) { end+=runs[i]; }
    s3_barcode_record_reset(&record);
    clock_us=UINT32_MAX-1000000U;
    for (uint32_t t=0U; t<end; t+=10000U) {
        uint32_t edge=leading;
        uint16_t dark=10U;
        if (t>=leading) {
            for (unsigned int i=0U; i<count; i++) {
                edge+=runs[i];
                if (t<edge) { dark=(i%2U) ? 10U : 900U; break; }
            }
        }
        feed(cfg,dark,&out);
    }
    return out;
}

static void test_physical_wide_gaps(void)
{
    const s3_barcode_config_t cfg=s3_barcode_defaults();
    const unsigned int wide_mm[]={8U,14U,19U};
    const unsigned int ratio_tenths[]={20U,25U,30U};
    unsigned int cases=0U;
    for (unsigned int size=0U; size<3U; size++) {
        /* time = distance / speed, at the configured normal 40 mm/s. */
        const uint32_t wide=wide_mm[size]*1000000U/40U;
        for (unsigned int ratio=0U; ratio<3U; ratio++) {
            const uint32_t narrow=wide*10U/ratio_tenths[ratio];
            /* Reader waits at least its one-second end-of-barcode window. */
            const uint32_t margin=12U*narrow+50000U;
            const uint32_t quiet=margin>1200000U ? margin : 1200000U;
            for (unsigned int letter=0U; letter<26U; letter++) {
                for (unsigned int direction=0U; direction<2U; direction++) {
                    uint32_t runs[29];
                    make_runs(letter,narrow,wide,runs);
                    if (direction) { reverse_runs(runs); }
                    tally_t out=trace(&cfg,runs,29U,quiet,quiet);
                    if (out.decoded!=1U || out.result.letter!=(char)('A'+letter) ||
                        out.result.reversed!=(direction!=0U) || out.rejected ||
                        out.timing || out.overflow) {
                        fprintf(stderr,"wide=%umm ratio=%u/10 letter=%c direction=%u "
                            "decoded=%u (%c) rejected=%u timing=%u overflow=%u\n",
                            wide_mm[size],ratio_tenths[ratio],(char)('A'+letter),
                            direction,out.decoded,out.result.letter?out.result.letter:'-',
                            out.rejected,out.timing,out.overflow);
                        assert(0);
                    }
                    cases++;
                }
            }
        }
    }
    assert(cases==468U);
}

static void test_long_frame_needs_real_quiet(void)
{
    s3_barcode_config_t cfg=s3_barcode_defaults();
    const uint32_t narrow=237500U, wide=475000U; /* 19 mm wide, 2:1 ratio */
    uint32_t runs[29];
    s3_barcode_result_t result;
    make_runs(25U,narrow,wide,runs);

    /* Geometry fits the old ten-second frame limit, but the actual complete
     * recording also needs 2.375 seconds of trailing quiet. The old wall
     * timeout was too short even though its batch width check passed. */
    s3_barcode_config_t old=cfg;
    old.max_frame_us=10000000U;
    assert(s3_barcode_decode_runs(runs,29U,3000000U,3000000U,10000U,&old,&result));
    tally_t out=trace(&old,runs,29U,3000000U,3000000U);
    assert(out.decoded==0U && out.rejected!=0U && !out.timing && !out.overflow);

    out=trace(&cfg,runs,29U,3000000U,8U*narrow);
    assert(out.decoded==0U && !out.rejected && !out.timing && !out.overflow);
    for (unsigned int i=0U; i<100U; i++) { feed(&cfg,10U,&out); }
    assert(out.decoded==1U && out.result.letter=='Z' && !out.rejected);

    out=trace(&cfg,runs,29U,8U*narrow,3000000U);
    assert(out.decoded==0U); /* no fabricated leading quiet */
    out=trace(&cfg,runs,27U,3000000U,3000000U);
    assert(out.decoded==0U); /* incomplete closing marker */
    runs[13]=1100000U;
    out=trace(&cfg,runs,29U,3000000U,3000000U);
    assert(out.decoded==0U && out.rejected!=0U); /* absolute single-gap limit */
}

static bool batch(const uint32_t runs[29], s3_barcode_result_t *result)
{
    const s3_barcode_config_t cfg=s3_barcode_defaults();
    return s3_barcode_decode_runs(runs,29U,3000000U,3000000U,10000U,&cfg,result);
}

static void test_one_extended_narrow_white_gap(void)
{
    uint32_t runs[29];
    s3_barcode_result_t result;
    for (unsigned int direction=0U; direction<2U; direction++) {
        make_runs(25U,100000U,250000U,runs);
        runs[13]=166000U; /* 149.55% of the resulting narrow mean (111000). */
        if (direction) { reverse_runs(runs); }
        assert(batch(runs,&result) && result.letter=='Z' &&
               result.reversed==(direction!=0U));
    }

    make_runs(25U,100000U,250000U,runs);
    runs[13]=168000U; /* 150.90% of the resulting narrow mean: too wide. */
    assert(!batch(runs,&result));

    make_runs(25U,100000U,250000U,runs);
    runs[13]=runs[15]=180000U; /* each 142.11%, but two outliers are refused. */
    assert(!batch(runs,&result));

    make_runs(25U,100000U,250000U,runs);
    runs[10]=166000U; /* a narrow black bar keeps the strict 135% limit. */
    assert(!batch(runs,&result));
    make_runs(25U,100000U,250000U,runs);
    runs[3]=166000U; /* opening star remains strict. */
    assert(!batch(runs,&result));
    make_runs(25U,100000U,250000U,runs);
    runs[23]=166000U; /* closing star remains strict. */
    assert(!batch(runs,&result));
    make_runs(25U,100000U,250000U,runs);
    runs[11]=500000U; /* wide white gaps keep the strict 135% limit. */
    assert(!batch(runs,&result));
}

static void test_measured_z_payload(void)
{
    /* From the 3 Nov 20:38:55 waveform after sentinel correction. The markers
     * and margins here are synthetic: this isolates payload validation and
     * does not claim that the faulted hardware run decoded Z. */
    const uint32_t payload[9]={109370U,322217U,282249U,185373U,300880U,
                              121629U,106343U,105846U,115817U};
    uint32_t runs[29];
    s3_barcode_result_t result;
    make_runs(25U,124000U,302000U,runs);
    memcpy(runs+10,payload,sizeof(payload));
    assert(batch(runs,&result) && result.letter=='Z');
    reverse_runs(runs);
    assert(batch(runs,&result) && result.letter=='Z' && result.reversed);
}

int main(void)
{
    test_physical_wide_gaps();
    test_long_frame_needs_real_quiet();
    test_one_extended_narrow_white_gap();
    test_measured_z_payload();
    puts("PASS: 468 synthetic A-Z scans at 8/14/19 mm wide gaps and 40 mm/s, real large-frame quiet, "
         "truncation/absolute bounds, one extended narrow payload white gap, strict markers/bars and measured Z payload");
    return 0;
}
