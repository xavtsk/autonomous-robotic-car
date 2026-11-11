/* Bounded-work and cancellation tests. The synchronous reference is used only
 * here to compare candidate semantics; firmware never builds that function. */
#define S3_BARCODE_SYNC_REFERENCE
#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem3/barcode_decoder.c"

static s3_barcode_record_t record;
static uint32_t clock_us;
static unsigned int max_units,max_ticks,max_extrema,processing_updates;
static const uint16_t codes[26]={
    0x109U,0x049U,0x148U,0x019U,0x118U,0x058U,0x00DU,0x10CU,0x04CU,0x01CU,
    0x103U,0x043U,0x142U,0x013U,0x112U,0x052U,0x007U,0x106U,0x046U,0x016U,
    0x181U,0x0C1U,0x1C0U,0x091U,0x190U,0x0D0U};
static uint16_t samples[2000];

static s3_barcode_status_t feed(uint16_t dark,bool valid,uint32_t step,
    s3_barcode_result_t *result)
{
    const s3_barcode_config_t config=s3_barcode_defaults();
    clock_us+=step;
    const s3_barcode_status_t status=s3_barcode_record_update(
        &record,&config,clock_us,valid,dark,result);
    assert(record.work_units<=S3_REC_WORK_UNITS);
    assert(record.fit_ticks<=S3_REC_FIT_TICKS);
    if (record.work_units>max_units) { max_units=record.work_units; }
    if (record.fit_ticks>max_ticks) { max_ticks=record.fit_ticks; }
    if (record.work.n>max_extrema) { max_extrema=record.work.n; }
    if (record.state==S3_REC_PROCESSING) { processing_updates++; }
    return status;
}

static unsigned int wave(unsigned int letter,unsigned int mode)
{
    const uint16_t patterns[3]={0x094U,codes[letter],0x094U};
    const uint32_t narrow=mode==2U ? 200000U : 100000U;
    const uint32_t wide=mode==2U ? 600000U : 250000U;
    uint32_t widths[29];
    unsigned int count=0U;
    for (unsigned int c=0U;c<3U;c++) {
        for (unsigned int i=0U;i<9U;i++) { widths[c*10U+i]=(patterns[c]&(1U<<(8U-i))) ? wide : narrow; }
        if (c<2U) { widths[c*10U+9U]=narrow; }
    }
    for (unsigned int i=0U;i<240U;i++) { samples[count++]=0U; }
    for (unsigned int i=0U;i<29U;i++) {
        const uint32_t observed=(i%2U) ? widths[i]+60000U : widths[i]-60000U;
        for (uint32_t t=0U;t<observed;t+=10000U) {
            uint16_t dark=(i%2U) ? 0U : (mode==1U && widths[i]==narrow ? 12U : 400U);
            if (mode==2U && !(i%2U) && widths[i]==wide && t>=50000U && t<400000U && (t%50000U)<20000U) { dark=360U; }
            assert(count<sizeof(samples)/sizeof(samples[0]));
            samples[count++]=dark;
        }
    }
    for (unsigned int i=0U;i<280U;i++) { samples[count++]=0U; }
    return count;
}

static void compare_frozen_candidate(void)
{
    s3_barcode_record_t snapshot=record;
    const s3_barcode_config_t config=s3_barcode_defaults();
    s3_barcode_result_t expected={0},other={0};
    uint32_t required=0U,other_required=0U;
    snapshot.count=snapshot.work.frozen_count;
    bool good=rec_candidate(&snapshot,&config,false,&expected,&required);
    bool ambiguous=false;
    if (rec_faint_eligible(&snapshot) && rec_candidate(&snapshot,&config,true,&other,&other_required)) {
        ambiguous=good && (expected.letter!=other.letter || expected.reversed!=other.reversed);
        expected=other;good=true;
        if (other_required>required) { required=other_required; }
    }
    /* Advance only a diagnostic copy, with its frozen physical observations.
     * This comparison is not a live decode or a substitute for fresh samples. */
    while (snapshot.work.phase!=RW_DONE) {
        snapshot.work_units=0U;snapshot.fit_ticks=0U;
        work_advance(&snapshot,&config);
        assert(snapshot.work_units<=S3_REC_WORK_UNITS);
        assert(snapshot.fit_ticks<=S3_REC_FIT_TICKS);
    }
    assert(snapshot.work.good==good);
    assert(snapshot.work.ambiguous==ambiguous);
    if (good && !ambiguous) {
        assert(snapshot.work.candidate.letter_code==(unsigned int)(expected.letter-'A'+1));
        assert(snapshot.work.candidate.reversed==expected.reversed);
        assert(snapshot.work.required_us==required);
    }
}

static void all_letters(void)
{
    unsigned int checks=0U;
    for (unsigned int letter=0U;letter<26U;letter++) {
        for (unsigned int reverse=0U;reverse<2U;reverse++) {
            for (unsigned int mode=0U;mode<3U;mode++) {
                const unsigned int count=wave(letter,mode);
                unsigned int decoded=0U;
                bool compared=false;
                s3_barcode_record_reset(&record);clock_us=0xFFFF0000U;
                for (unsigned int i=0U;i<count;i++) {
                    s3_barcode_result_t result;
                    const s3_barcode_status_t status=feed(samples[reverse ? count-1U-i : i],true,10000U,&result);
                    if (!compared && record.state==S3_REC_PROCESSING) { compare_frozen_candidate();compared=true; }
                    if (status==S3_BARCODE_DECODED) {
                        decoded++;assert(result.letter==(char)('A'+letter));assert(result.reversed==(reverse!=0U));
                    }
                    assert(status!=S3_BARCODE_TIMING_ERROR && status!=S3_BARCODE_OVERFLOW);
                }
                assert(compared && decoded==1U);checks++;
            }
        }
    }
    assert(checks==156U);
}

static void cancel_pending_jobs(void)
{
    for (unsigned int mode=0U;mode<4U;mode++) {
        const unsigned int count=wave(25U,0U);
        s3_barcode_result_t result;
        bool pending=false;
        s3_barcode_record_reset(&record);clock_us=0U;
        for (unsigned int i=0U;i<count;i++) {
            assert(feed(samples[i],true,10000U,&result)!=S3_BARCODE_DECODED);
            if (record.state==S3_REC_PROCESSING) { pending=true;break; }
        }
        assert(pending);
        if (mode==0U) { assert(feed(0U,false,10000U,&result)==S3_BARCODE_NONE); }
        if (mode==1U) { assert(feed(0U,true,40000U,&result)==S3_BARCODE_TIMING_ERROR); }
        if (mode==2U) { assert(feed(400U,true,10000U,&result)==S3_BARCODE_REJECTED); }
        if (mode==3U) { s3_barcode_record_reset(&record); }
        assert(record.state==S3_REC_WAITING && record.work.phase==0U);
        for (unsigned int i=0U;i<300U;i++) { assert(feed(0U,true,10000U,&result)!=S3_BARCODE_DECODED); }
    }
}

int main(void)
{
    _Static_assert(sizeof(s3_barcode_work_t)<=1200U,"Incremental barcode workspace exceeded its RAM budget");
    all_letters();cancel_pending_jobs();
    assert(max_units==S3_REC_WORK_UNITS && max_ticks==S3_REC_FIT_TICKS);
    assert(max_extrema>=100U && processing_updates>100U);
    printf("PASS: 156 incremental A-Z/direction/sharp-faint-ripple equivalence scans; invalid/timing/new-ink/reset cancellation; maximum work=%u ticks=%u extrema=%u workspace=%u record=%u bytes\n",
        max_units,max_ticks,max_extrema,(unsigned int)sizeof(s3_barcode_work_t),(unsigned int)sizeof(s3_barcode_record_t));
    return 0;
}
