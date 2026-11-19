/* Synthetic fast-scan regressions, not claims about physical test success.
 * F and P share their wide-bar centres: the narrow bar between them must
 * remain observed and distinguish the letters in either travel direction. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "bc_decoder.h"

enum { STEP_US=5000U, PAPER=100U, INK=900U, MARGIN_US=1500000U };
enum { INTACT, SHORT_LEAD, SHORT_TRAIL, MISSING_NARROW, WEAK_FRAME };
typedef struct {
    unsigned int decoded, rejected, timing, overflow;
    uint32_t decode_delay_us, rejection_delay_us;
    s3_barcode_result_t last;
} tally_t;

static void make_runs(char letter,uint32_t narrow,unsigned int ratio,
    bool reverse,uint32_t runs[S3_BARCODE_RUNS])
{
    const uint16_t payload=letter=='F' ? 0x058U : 0x052U;
    const uint16_t symbols[3]={0x094U,payload,0x094U};
    assert(letter=='F' || letter=='P');
    for (unsigned int c=0U;c<3U;c++) {
        for (unsigned int i=0U;i<9U;i++) {
            runs[c*10U+i]=(symbols[c]&(1U<<(8U-i))) ? ratio*narrow : narrow;
        }
        if (c<2U) { runs[c*10U+9U]=narrow; }
    }
    if (reverse) {
        for (unsigned int i=0U;i<S3_BARCODE_RUNS/2U;i++) {
            const uint32_t saved=runs[i];
            runs[i]=runs[S3_BARCODE_RUNS-1U-i];
            runs[S3_BARCODE_RUNS-1U-i]=saved;
        }
    }
}

static tally_t scan(char letter,bool reverse,uint32_t narrow,unsigned int ratio,
    uint32_t erosion,uint32_t phase,unsigned int damage)
{
    static s3_barcode_record_t record;
    const s3_barcode_config_t config=s3_barcode_defaults();
    const uint32_t margin=12U*narrow>MARGIN_US ? 12U*narrow : MARGIN_US;
    const uint32_t lead=(damage==SHORT_LEAD ? 8U*narrow : margin)+phase;
    const uint32_t trail=damage==SHORT_TRAIL ? 8U*narrow : margin;
    uint32_t runs[S3_BARCODE_RUNS],begin[S3_BARCODE_RUNS],end[S3_BARCODE_RUNS];
    uint32_t total=lead;
    tally_t tally={0};
    /* Canonical stripe 16 is the narrow black bar separating F from P.
     * Delete it geometrically, without substituting a different payload. */
    const unsigned int missing=reverse ? S3_BARCODE_RUNS-1U-16U : 16U;
    make_runs(letter,narrow,ratio,reverse,runs);
    assert(erosion<narrow && phase<STEP_US);
    for (unsigned int i=0U;i<S3_BARCODE_RUNS;i++) {
        /* Fixed printed widths first; the optics shrink every black bar by
         * the same duration and enlarge each internal white space equally.
         * Both outer margins gain half the erosion as well. */
        begin[i]=total+((i%2U) ? 0U : erosion/2U);
        end[i]=total+runs[i]-((i%2U) ? 0U : erosion/2U);
        total+=runs[i];
    }
    total+=trail;
    s3_barcode_record_reset(&record);
    for (uint32_t t=0U;t<total;t+=STEP_US) {
        uint16_t darkness=PAPER;
        s3_barcode_result_t result;
        for (unsigned int i=0U;i<S3_BARCODE_RUNS;i+=2U) {
            if (damage==MISSING_NARROW && i==missing) { continue; }
            if (t>=begin[i] && t<end[i]) {
                darkness=damage==WEAK_FRAME ? PAPER+S3_QUIET_STRONG_PROMINENCE-5U : INK;
                break;
            }
        }
        const s3_barcode_status_t status=s3_barcode_record_update(&record,
            &config,0xFFFF0000U+t,true,darkness,&result);
        assert(record.work_units<=S3_REC_WORK_UNITS);
        assert(record.fit_ticks<=S3_REC_FIT_TICKS);
        if (status==S3_BARCODE_DECODED) {
            assert(t>=end[S3_BARCODE_RUNS-1U]);
            tally.decoded++;tally.last=result;
            tally.decode_delay_us=t-end[S3_BARCODE_RUNS-1U];
        } else if (status==S3_BARCODE_REJECTED) {
            if (!tally.rejected) {
                tally.rejection_delay_us=t>=end[S3_BARCODE_RUNS-1U] ?
                    t-end[S3_BARCODE_RUNS-1U] : UINT32_MAX;
            }
            tally.rejected++;
        }
        else if (status==S3_BARCODE_TIMING_ERROR) { tally.timing++; }
        else if (status==S3_BARCODE_OVERFLOW) { tally.overflow++; }
    }
    return tally;
}

static void test_fast_frames(void)
{
    const char letters[]={'F','P'};
    /* Half-period module widths also vary the individual sample counts;
     * shifting only period-aligned widths would preserve every measured run. */
    const uint32_t narrow[]={25000U,27500U,30000U,32500U,35000U};
    unsigned int checks=0U;
    for (unsigned int c=0U;c<sizeof letters;c++) {
        for (unsigned int reverse=0U;reverse<2U;reverse++) {
            for (unsigned int n=0U;n<sizeof narrow/sizeof narrow[0];n++) {
                for (unsigned int ratio=2U;ratio<=3U;ratio++) {
                    for (unsigned int optical=0U;optical<2U;optical++) {
                        /* At 25 ms,20 ms would exceed the existing 75% bias
                         * bound.15 ms remains within that unchanged model.
                         * At 30 ms,20 ms erosion leaves a 10 ms optical core. */
                        const uint32_t erosion=optical ?
                            (narrow[n]<30000U ? narrow[n]-10000U : 20000U) : 0U;
                        for (uint32_t phase=0U;phase<STEP_US;phase+=1000U) {
                            const tally_t got=scan(letters[c],reverse!=0U,narrow[n],
                                ratio,erosion,phase,INTACT);
                            if (got.decoded!=1U || got.last.letter!=letters[c] ||
                                got.last.reversed!=(reverse!=0U) || got.rejected) {
                                printf("fast %c reverse=%u narrow=%u ratio=%u erosion=%u phase=%u: decoded=%u letter=%c rejected=%u\n",
                                    letters[c],reverse,narrow[n],ratio,erosion,phase,
                                    got.decoded,got.last.letter ? got.last.letter : '-',got.rejected);
                            }
                            assert(got.decoded==1U && got.last.letter==letters[c]);
                            assert(got.last.reversed==(reverse!=0U) && got.rejected==0U);
                            assert(got.timing==0U && got.overflow==0U);
                            assert(got.decode_delay_us<1000000U);
                            checks++;
                        }
                    }
                }
            }
        }
    }
    printf("PASS: %u synthetic fast F/P scans, both directions, 5 sample phases, sharp/eroded 25-35 ms modules\n",checks);
}

static void test_fast_refusals(void)
{
    const char letters[]={'F','P'};
    for (unsigned int c=0U;c<sizeof letters;c++) {
        for (unsigned int reverse=0U;reverse<2U;reverse++) {
            for (unsigned int damage=SHORT_LEAD;damage<=MISSING_NARROW;damage++) {
                for (uint32_t phase=0U;phase<STEP_US;phase+=1000U) {
                    const tally_t got=scan(letters[c],reverse!=0U,30000U,3U,
                        20000U,phase,damage);
                    assert(got.decoded==0U && got.timing==0U && got.overflow==0U);
                    if (damage==MISSING_NARROW) {
                        /* The 250 ms attempt cannot end an incomplete frame.
                         * Its unchanged final 1 s attempt still rejects it. */
                        assert(got.rejected==1U);
                        assert(got.rejection_delay_us>=1000000U &&
                               got.rejection_delay_us<1500000U);
                    }
                }
            }
            /* More frequent readings do not make sub-minimum modules valid. */
            const tally_t small=scan(letters[c],reverse!=0U,20000U,3U,0U,0U,INTACT);
            assert(small.decoded==0U && small.rejected!=0U);
            assert(small.timing==0U && small.overflow==0U);
        }
    }
}

static void test_early_analysis_preserves_quiet_and_large_frames(void)
{
    const char letters[]={'F','P'};
    for (unsigned int c=0U;c<sizeof letters;c++) {
        for (unsigned int reverse=0U;reverse<2U;reverse++) {
            const tally_t small=scan(letters[c],reverse!=0U,30000U,3U,
                20000U,0U,INTACT);
            assert(small.decoded==1U && small.rejected==0U);
            /* Do not confuse beginning computation at 250 ms with permission
             * to stop: this frame still needs its full 300 ms quiet margin. */
            assert(small.decode_delay_us>=300000U && small.decode_delay_us<1000000U);
            const tally_t weak=scan(letters[c],reverse!=0U,30000U,3U,
                20000U,0U,WEAK_FRAME);
            /* Below the strong-frame threshold: only actual strict quiet,
             * not the tolerant noise clock, can authorize this early fit. */
            assert(weak.decoded==1U && weak.last.letter==letters[c] && weak.rejected==0U);
            assert(weak.last.reversed==(reverse!=0U));
            assert(weak.decode_delay_us>=300000U && weak.decode_delay_us<1000000U);
            /* 450 ms internal white elements exceed the early-work trigger.
             * Neither a failed partial fit nor new ink may discard the frame. */
            const tally_t large=scan(letters[c],reverse!=0U,150000U,3U,
                60000U,0U,INTACT);
            assert(large.decoded==1U && large.last.letter==letters[c]);
            assert(large.last.reversed==(reverse!=0U));
            assert(large.rejected==0U && large.timing==0U && large.overflow==0U);
            assert(large.decode_delay_us>=1500000U);
        }
    }
    /* A short incomplete fragment followed by 500 ms white must remain open.
     * Its early attempt may fail, but cannot publish a letter or rejection. */
    static s3_barcode_record_t record;
    const s3_barcode_config_t config=s3_barcode_defaults();
    s3_barcode_result_t result;
    s3_barcode_record_reset(&record);
    for (uint32_t t=0U;t<1500000U;t+=STEP_US) {
        const uint16_t dark=t>=900000U && t<1000000U ? INK : PAPER;
        assert(s3_barcode_record_update(&record,&config,t,true,dark,&result)==S3_BARCODE_NONE);
    }
    assert(s3_barcode_record_active(&record));
    assert(record.early_tried);
}

static void test_exact_width_and_sampling_boundaries(void)
{
    const s3_barcode_config_t config=s3_barcode_defaults();
    const char letters[]={'F','P'};
    assert(config.min_narrow_us==25000U);
    for (unsigned int c=0U;c<sizeof letters;c++) {
        for (unsigned int reverse=0U;reverse<2U;reverse++) {
            uint32_t runs[S3_BARCODE_RUNS];
            s3_barcode_result_t result;
            make_runs(letters[c],24999U,3U,reverse!=0U,runs);
            assert(!s3_barcode_decode_runs(runs,S3_BARCODE_RUNS,MARGIN_US,
                MARGIN_US,STEP_US,&config,&result));
            make_runs(letters[c],25000U,3U,reverse!=0U,runs);
            assert(s3_barcode_decode_runs(runs,S3_BARCODE_RUNS,MARGIN_US,
                MARGIN_US,STEP_US,&config,&result));
            assert(result.letter==letters[c] && result.reversed==(reverse!=0U));
            make_runs(letters[c],30000U,3U,reverse!=0U,runs);
            assert(s3_barcode_decode_runs(runs,S3_BARCODE_RUNS,MARGIN_US,
                MARGIN_US,10000U,&config,&result));
            assert(!s3_barcode_decode_runs(runs,S3_BARCODE_RUNS,MARGIN_US,
                MARGIN_US,10001U,&config,&result));
            /* A physically too-short payload white gap remains invalid.
             * Use exact durations here: 5 ms sample rounding could turn a
             * 16.2 ms gap into 20 ms and obscure the intended 65% boundary. */
            const unsigned int gap=reverse ? S3_BARCODE_RUNS-1U-11U : 11U;
            runs[gap]=30000U*54U/100U;
            assert(!s3_barcode_decode_runs(runs,S3_BARCODE_RUNS,MARGIN_US,
                MARGIN_US,STEP_US,&config,&result));
        }
    }
}

int main(void)
{
    test_fast_frames();
    test_fast_refusals();
    test_exact_width_and_sampling_boundaries();
    test_early_analysis_preserves_quiet_and_large_frames();
    puts("PASS: fast F/P quiet/missing-bar refusals, 25 ms floor, 3-times-sample-gap guard and malformed payload gap");
    return 0;
}
