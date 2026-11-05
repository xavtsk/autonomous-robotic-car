/* Synthetic low-contrast printed-barcode regressions. These are generated
 * sensor waveforms, not additional observed hardware successes. Physical
 * Code 39 widths are fixed first; a common optical bias shortens every black
 * bar and lengthens every white space. Narrow bars have the weak contrast
 * seen in the November captures, independently of their payload letter. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "barcode_decoder.h"

static const uint16_t letters[26] = {
    0x109,0x049,0x148,0x019,0x118,0x058,0x00D,0x10C,0x04C,0x01C,
    0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,
    0x181,0x0C1,0x1C0,0x091,0x190,0x0D0
};
enum { STEP=10000, NARROW=100000, WIDE=250000, BIAS=60000,
       MARGIN=1600000, MAX_SAMPLES=1200 };
enum { WIGGLE=1, SHOULDER=2, SHORT_LEAD=4, BROKEN_TRAIL=8,
       CUT_START=16, EXTRA_PULSE=32, PAPER_NOISE=64,
       MARK_BEFORE_HISTORY=128, EARLY_MARK=256,
       PRETRIGGER_RAMP=512, WRONG_PRIMARY=1024 };
typedef struct {
    unsigned int decoded, rejected, timing, overflow;
    char letter;
    bool reversed;
} tally_t;

static unsigned int waveform(char letter, unsigned int flags, unsigned int weak,
                             uint16_t values[MAX_SAMPLES])
{
    const uint16_t symbols[3]={0x094U,letters[(unsigned int)(letter-'A')],0x094U};
    uint32_t physical[29], width[29], start[29], total;
    const uint32_t lead=(flags&SHORT_LEAD) ? 200000U : MARGIN;
    unsigned int i,c,t,shoulder=29U,wide_bar=29U,weak_bar=29U;
    for(c=0U;c<3U;c++) {
        for(i=0U;i<9U;i++) {
            physical[c*10U+i]=(symbols[c]&(1U<<(8U-i))) ? WIDE : NARROW;
        }
        if(c<2U) { physical[c*10U+9U]=NARROW; }
    }
    /* A legitimately slightly short payload space leaves room to expose an
     * early edge caused by a leading shoulder. Choose by stripe geometry,
     * not by a payload bit pattern or expected decoded letter. */
    for(i=12U;i<19U;i+=2U) {
        if(physical[i]==NARROW && physical[i-1U]==NARROW) {
            shoulder=i;
            if(flags&SHOULDER) { physical[i-1U]=80000U; }
            break;
        }
    }
    {
        const s3_barcode_config_t cfg=s3_barcode_defaults();
        s3_barcode_result_t check;
        assert(s3_barcode_decode_runs(physical,29U,MARGIN,MARGIN,STEP,&cfg,&check));
        assert(check.letter==letter && !check.reversed);
    }
    total=lead;
    for(i=0U;i<29U;i++) {
        width[i]=(i%2U) ? physical[i]+BIAS : physical[i]-BIAS;
        start[i]=total; total+=width[i];
        if(i%2U==0U && physical[i]==WIDE && wide_bar==29U) { wide_bar=i; }
        if(i>=10U && i<=18U && i%2U==0U && physical[i]==NARROW && weak_bar==29U) { weak_bar=i; }
    }
    assert(total+MARGIN<MAX_SAMPLES*STEP);
    for(t=0U;t<total+MARGIN;t+=STEP) {
        uint16_t dark=0U;
        for(i=0U;i<29U;i+=2U) {
            if(t>=start[i] && t<start[i]+width[i]) {
                dark=(uint16_t)(physical[i]==WIDE ? 220U+((i*17U)%180U) : weak);
                if((flags&WRONG_PRIMARY) && i==weak_bar) { dark=12U; }
                if((flags&(WIGGLE|WRONG_PRIMARY)) && i==wide_bar &&
                   t>=start[i]+80000U && t<start[i]+120000U) {
                    dark=(uint16_t)(dark-40U);
                }
            }
        }
        /* An 8 -> 6 -> 12 shoulder immediately before a weak core. */
        if((flags&SHOULDER) && shoulder<29U) {
            if(t==start[shoulder]-20000U) { dark=8U; }
            if(t==start[shoulder]-10000U) { dark=6U; }
        }
        if((flags&CUT_START) && t<start[2U]) { dark=0U; }
        if((flags&EXTRA_PULSE) && t>=start[1U]+80000U && t<start[1U]+120000U) { dark=12U; }
        if((flags&BROKEN_TRAIL) && t>=total+500000U && t<total+550000U) { dark=12U; }
        if((flags&PAPER_NOISE) && dark==0U) { dark=(t/STEP)%2U ? 3U : 0U; }
        /* This 100 ms faint mark leaves only 960 ms of observed white before
         * the first bar. By the first strong bar's trigger it is outside the
         * retained 1.5 s history; it must still invalidate the leading quiet
         * credit. Reversing the same waveform tests a trailing interruption.
         * The earlier control mark leaves enough real white on either side. */
        if((flags&MARK_BEFORE_HISTORY) && t>=540000U && t<640000U) { dark=12U; }
        if((flags&EARLY_MARK) && t>=200000U && t<300000U) { dark=12U; }
        /* A genuine rising bar shoulder below the recording-start rise must
         * not be learned as quiet-paper noise and disable weak-bar recovery. */
        if((flags&PRETRIGGER_RAMP) && t==start[wide_bar]-STEP) { dark=45U; }
        values[t/STEP]=dark;
    }
    return (total+MARGIN)/STEP;
}

static tally_t scan(char letter, bool reverse, unsigned int flags, unsigned int weak)
{
    static uint16_t values[MAX_SAMPLES];
    static s3_barcode_record_t record;
    const s3_barcode_config_t cfg=s3_barcode_defaults();
    const unsigned int count=waveform(letter,flags,weak,values);
    tally_t out={0};
    uint32_t clock=0xFFFF0000U;
    unsigned int i;
    s3_barcode_record_reset(&record);
    for(i=0U;i<count;i++) {
        s3_barcode_result_t result;
        s3_barcode_status_t status;
        clock+=STEP;
        status=s3_barcode_record_update(&record,&cfg,clock,true,
                                        values[reverse ? count-1U-i : i],&result);
        if(status==S3_BARCODE_DECODED) {
            out.decoded++; out.letter=result.letter; out.reversed=result.reversed;
        }
        if(status==S3_BARCODE_REJECTED) { out.rejected++; }
        if(status==S3_BARCODE_TIMING_ERROR) { out.timing++; }
        if(status==S3_BARCODE_OVERFLOW) { out.overflow++; }
    }
    return out;
}

static void test_noisy_paper_is_not_a_clean_prefix(void)
{
    static s3_barcode_record_t record;
    const s3_barcode_config_t cfg=s3_barcode_defaults();
    s3_barcode_result_t result;
    uint32_t clock=0U;
    unsigned int i;
    s3_barcode_record_reset(&record);
    /* Repeated 12-unit paper excursions provide no stable low-band prefix.
     * A later strong bar must not cause that measured noise to be replaced
     * by a clean-paper estimate merely because some individual readings are
     * zero. Do not require a particular estimator formula, just preserve a
     * noise gate above the low-noise recovery eligibility limit. */
    for(i=0U;i<160U;i++) {
        clock+=STEP;
        assert(s3_barcode_record_update(&record,&cfg,clock,true,
                                       (i%2U) ? 12U : 0U,&result)==S3_BARCODE_NONE);
    }
    clock+=STEP;
    assert(s3_barcode_record_update(&record,&cfg,clock,true,500U,&result)==S3_BARCODE_NONE);
    assert(s3_barcode_record_active(&record));
    assert(record.prominence>S3_REC_FAINT_MAX_NOISE);
}

int main(void)
{
    unsigned int c,dir,mode,checks=0U;
    const struct { unsigned int flags,weak; } positive[]={
        {0U,10U},{0U,12U},{0U,25U},{WIGGLE,12U},{SHOULDER,12U},
        {WIGGLE|SHOULDER,12U},{WIGGLE|SHOULDER|PAPER_NOISE,12U},{EARLY_MARK,12U},
        {PRETRIGGER_RAMP,12U},{WRONG_PRIMARY,25U}
    };
    const unsigned int negative[]={SHORT_LEAD,BROKEN_TRAIL,CUT_START,EXTRA_PULSE,MARK_BEFORE_HISTORY};
    for(c=0U;c<26U;c++) {
        for(dir=0U;dir<2U;dir++) {
            for(mode=0U;mode<sizeof positive/sizeof positive[0];mode++) {
                const tally_t got=scan((char)('A'+c),dir!=0U,positive[mode].flags,positive[mode].weak);
                if(got.decoded!=1U || got.letter!=(char)('A'+c) || got.reversed!=(dir!=0U)) {
                    printf("faint recovery letter=%c direction=%u mode=%u: decoded=%u letter=%c rejected=%u\n",
                           'A'+c,dir,positive[mode].flags,got.decoded,got.letter ? got.letter : '-',got.rejected);
                    fflush(stdout);
                    assert(0);
                }
                assert(got.timing==0U && got.overflow==0U);
                checks++;
            }
            for(mode=0U;mode<sizeof negative/sizeof negative[0];mode++) {
                const tally_t got=scan((char)('A'+c),dir!=0U,negative[mode],12U);
                if(got.decoded!=0U) {
                    printf("faint refusal letter=%c direction=%u mode=%u: decoded=%u letter=%c\n",
                           'A'+c,dir,negative[mode],got.decoded,got.letter ? got.letter : '-');
                    fflush(stdout);
                }
                assert(got.decoded==0U && got.timing==0U && got.overflow==0U);
                checks++;
            }
        }
    }
    test_noisy_paper_is_not_a_clean_prefix();
    checks++;
    printf("PASS: %u faint-bar A-Z/both-direction recovery and real-quiet/truncation/extra-pulse checks\n",checks);
    return 0;
}
