/* Public-API regressions for noisy trailing paper in the recording reader.
 * Synthetic Code 39 waveforms, not observed hardware successes. A valid
 * strong frame may bridge bounded paper excursions; faint bars and genuine
 * quiet-zone interruptions must remain visible. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "barcode_decoder.h"

static const uint16_t letters[26]={
    0x109,0x049,0x148,0x019,0x118,0x058,0x00D,0x10C,0x04C,0x01C,
    0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,
    0x181,0x0C1,0x1C0,0x091,0x190,0x0D0};
enum { STEP=10000U,NARROW=150000U,WIDE=450000U,LEAD=2000000U };
static s3_barcode_record_t record;
static uint32_t clock_us,elapsed_us;
typedef struct {
    unsigned int decoded,rejected,timing,overflow;
    char letter;
    bool reversed;
    uint32_t first_decode_us;
    uint16_t decode_dark;
} tally_t;
static tally_t tally;
static unsigned int case_boundary;

static void reset(void)
{
    s3_barcode_record_reset(&record);
    clock_us=0xFFFF0000U; elapsed_us=0U;
    tally=(tally_t){0};
}
static void sample(uint16_t darkness)
{
    const s3_barcode_config_t config=s3_barcode_defaults();
    s3_barcode_result_t result;
    clock_us+=STEP;elapsed_us+=STEP;
    const s3_barcode_status_t status=s3_barcode_record_update(
        &record,&config,clock_us,true,darkness,&result);
    if(status==S3_BARCODE_DECODED) {
        if(tally.decoded==0U)tally.first_decode_us=elapsed_us;
        tally.decoded++;tally.letter=result.letter;tally.reversed=result.reversed;
        tally.decode_dark=darkness;
    }
    if(status==S3_BARCODE_REJECTED)tally.rejected++;
    if(status==S3_BARCODE_TIMING_ERROR)tally.timing++;
    if(status==S3_BARCODE_OVERFLOW)tally.overflow++;
}
static void level(uint32_t duration,uint16_t darkness)
{
    assert(duration%STEP==0U);
    for(uint32_t t=0U;t<duration;t+=STEP)sample(darkness);
}
static void make_widths(char letter,bool reverse,uint32_t widths[29])
{
    const uint16_t symbols[3]={0x094U,letters[(unsigned int)(letter-'A')],0x094U};
    for(unsigned int c=0U;c<3U;c++) {
        for(unsigned int i=0U;i<9U;i++)
            widths[c*10U+i]=(symbols[c]&(1U<<(8U-i)))?WIDE:NARROW;
        if(c<2U)widths[c*10U+9U]=NARROW;
    }
    if(reverse)for(unsigned int i=0U;i<14U;i++) {
        uint32_t swap=widths[i];widths[i]=widths[28U-i];widths[28U-i]=swap;
    }
}
/* weak==0: uniform strong bars. Otherwise narrow bars use weak contrast and
 * wide bars remain strong. Every frame has the same 60 ms optical width bias
 * so these exercise sentinel fitting as well as deferred quiet validation. */
static uint32_t frame(char letter,bool reverse,uint16_t strong,uint16_t weak,
                      int missing_bar,unsigned int first_run)
{
    uint32_t widths[29];make_widths(letter,reverse,widths);
    level(LEAD,0U);
    for(unsigned int i=first_run;i<29U;i++) {
        const bool black=i%2U==0U;
        uint32_t duration=black?widths[i]-60000U:widths[i]+60000U;
        uint16_t dark=black?((weak!=0U&&widths[i]==NARROW)?weak:strong):0U;
        if((int)i==missing_bar)dark=0U;
        level(duration,dark);
    }
    return elapsed_us;
}
static void noisy_tail(uint32_t duration,uint16_t amplitude,unsigned int high_samples)
{
    /* Every 100 ms contains a bounded excursion and a return to low paper. */
    for(unsigned int i=0U;i<duration/STEP;i++)
        sample(i%10U>=10U-high_samples?amplitude:0U);
}
static void check_no_transport_failure(void)
{
    assert(tally.timing==0U&&tally.overflow==0U);
}
static void check_success(char letter,bool reverse,uint32_t end)
{
    if(tally.decoded!=1U||tally.letter!=letter||tally.reversed!=reverse) {
        printf("quiet record %c reverse=%u boundary=%u decoded=%u letter=%c rejected=%u state=%u\n",
            letter,reverse,case_boundary,tally.decoded,tally.letter?tally.letter:'-',tally.rejected,record.state);
        printf("paper=%u prominence=%u phase=%u good=%u allow_noise=%u provisional=%u "
               "strong=%u frozen=%u leading=%u quiet=%u/%u retry=%u\n",
            record.paper,record.prominence,record.work.phase,record.work.good,
            record.work.allow_noise,record.work.provisional,record.work.strong,
            record.work.frozen_count,record.leading_us,record.quiet.strict_us,
            record.quiet.tolerant_us,record.retry_blocked);
        fflush(stdout);assert(0);
    }
    assert(tally.first_decode_us>=end+10U*NARROW-STEP);
    assert(tally.decode_dark<=5U); /* Never release during a bridged excursion. */
    check_no_transport_failure();
}

static unsigned int all_letters_both_directions(void)
{
    unsigned int checks=0U;
    for(unsigned int letter=0U;letter<26U;letter++)for(unsigned int reverse=0U;reverse<2U;reverse++) {
        for(unsigned int boundary=0U;boundary<2U;boundary++) {
            case_boundary=boundary;
            reset();
            const uint32_t end=frame((char)('A'+letter),reverse!=0U,
                boundary?60U:400U,0U,-1,0U);
            /* Tolerated paper must stay strictly below the primary peak
             * prominence (15), otherwise it creates real extractor peaks. */
            noisy_tail(3000000U,boundary?14U:6U,boundary?4U:1U);
            check_success((char)('A'+letter),reverse!=0U,end);
            assert(tally.rejected==0U);
            checks++;
        }
    }
    return checks;
}

static unsigned int weak_frames_keep_strict_paper(void)
{
    unsigned int checks=0U;
    for(unsigned int reverse=0U;reverse<2U;reverse++) {
        for(unsigned int faint=0U;faint<2U;faint++) {
            reset();
            const uint32_t end=frame('Z',reverse!=0U,faint?400U:55U,
                faint?12U:0U,-1,0U);
            noisy_tail(2000000U,12U,2U);
            /* A tentative strong-frame check must neither accept nor discard
             * a faint scan; it still needs a genuinely clean trailing margin. */
            assert(tally.decoded==0U);
            assert(s3_barcode_record_active(&record));
            level(2500000U,0U);
            if(!faint)check_success('Z',reverse!=0U,end);
            else {
                /* Twelve-unit marks are as dark as the faint barcode's
                 * narrow bars. They are genuine ambiguous extra ink, not
                 * something a strong-frame noise allowance may erase. */
                assert(tally.decoded==0U);check_no_transport_failure();
            }
            checks++;
        }
    }
    return checks;
}

static unsigned int invalid_shapes_still_refused(void)
{
    unsigned int checks=0U;
    for(unsigned int reverse=0U;reverse<2U;reverse++)for(unsigned int cut=0U;cut<2U;cut++) {
        reset();
        (void)frame('Z',reverse!=0U,400U,0U,cut?-1:12,cut?10U:0U);
        noisy_tail(2000000U,12U,2U);level(2500000U,0U);
        assert(tally.decoded==0U);check_no_transport_failure();checks++;
    }
    return checks;
}

static unsigned int genuine_interruptions_are_not_bridged(void)
{
    unsigned int checks=0U;
    for(unsigned int reverse=0U;reverse<2U;reverse++)for(unsigned int kind=0U;kind<2U;kind++) {
        reset();
        (void)frame('Z',reverse!=0U,400U,0U,-1,0U);
        level(1400000U,0U);
        /* Above the amplitude bound, or longer than the duration bound.
         * There is insufficient fresh quiet before this test's endpoint. */
        level(kind?50000U:40000U,kind?12U:15U);
        level(400000U,0U);
        assert(tally.decoded==0U);check_no_transport_failure();checks++;
    }
    return checks;
}

int main(void)
{
    unsigned int checks=all_letters_both_directions();
    checks+=weak_frames_keep_strict_paper();
    checks+=invalid_shapes_still_refused();
    checks+=genuine_interruptions_are_not_bridged();
    printf("PASS: %u public recording-reader noisy-paper, weak-bar and invalid-frame checks\n",checks);
    return 0;
}
