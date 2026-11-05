/* Host evidence for a width bias that changes between the two known stars.
 * Synthetic edges and one measured-width fixture; not hardware proof. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "barcode_decoder.h"

static const uint16_t letters[26]={
    0x109,0x049,0x148,0x019,0x118,0x058,0x00D,0x10C,0x04C,0x01C,
    0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,
    0x181,0x0C1,0x1C0,0x091,0x190,0x0D0};

static void make_runs(unsigned int letter,uint32_t runs[29])
{
    uint16_t patterns[3]={0x094U,letters[letter],0x094U};
    unsigned int c,i;
    for(c=0U;c<3U;c++) {
        for(i=0U;i<9U;i++) { runs[c*10U+i]=(patterns[c]&(1U<<(8U-i)))?250000U:100000U; }
        if(c<2U) { runs[c*10U+9U]=100000U; }
    }
}

static void reverse(uint32_t runs[29])
{
    unsigned int i;
    for(i=0U;i<14U;i++) { uint32_t v=runs[i];runs[i]=runs[28U-i];runs[28U-i]=v; }
}

/* Move physical boundaries to simulate bar erosion/dilation. Bias varies in
 * true elapsed time; the reader must infer it from measured elapsed time. */
static void distort(uint32_t runs[29],int32_t first,int32_t last)
{
    uint32_t edges[30]={0U};
    int64_t shifted[30];
    unsigned int i;
    for(i=0U;i<29U;i++) { edges[i+1U]=edges[i]+runs[i]; }
    for(i=0U;i<30U;i++) {
        int32_t bias;
        if(i<=9U) { bias=first; }
        else if(i>=20U) { bias=last; }
        else { bias=first+(int32_t)((int64_t)(last-first)*(edges[i]-edges[9])/(edges[20]-edges[9])); }
        shifted[i]=(int64_t)edges[i]+((i%2U)?-(int64_t)bias/2:(int64_t)bias/2);
    }
    for(i=0U;i<29U;i++) {
        assert(shifted[i+1U]>shifted[i]);
        runs[i]=(uint32_t)(shifted[i+1U]-shifted[i]);
    }
}

static void test_letters_and_payload_independence(void)
{
    const s3_barcode_config_t cfg=s3_barcode_defaults();
    unsigned int c,sign,dir,slope,i;
    for(sign=0U;sign<2U;sign++) {
        for(dir=0U;dir<2U;dir++) {
            for(slope=0U;slope<2U;slope++) {
                int32_t reference[2]={0,0};
                for(c=0U;c<27U;c++) {
                    uint32_t runs[29];
                    int32_t first=slope?20000:60000,last=slope?60000:20000,bias[2];
                    s3_barcode_result_t result;
                    make_runs(c%26U,runs);
                    if(c==26U) { for(i=10U;i<19U;i++) { runs[i]=100000U; } }
                    if(sign) { first=-first;last=-last; }
                    distort(runs,first,last);
                    if(dir) { int32_t b=first;reverse(runs);first=last;last=b; }
                    assert(s3_barcode_record_correct_varying_runs(runs,10000U,&cfg,bias));
                    assert(bias[0]>first-1500 && bias[0]<first+1500);
                    assert(bias[1]>last-1500 && bias[1]<last+1500);
                    if(c==0U) { reference[0]=bias[0];reference[1]=bias[1]; }
                    else { assert(bias[0]==reference[0] && bias[1]==reference[1]); }
                    if(c<26U) {
                        assert(s3_barcode_decode_runs(runs,29U,2000000U,2000000U,10000U,&cfg,&result));
                        assert(result.letter==(char)('A'+c) && result.reversed==(dir!=0U));
                    } else {
                        assert(!s3_barcode_decode_runs(runs,29U,2000000U,2000000U,10000U,&cfg,&result));
                    }
                }
            }
        }
    }
}

static void refuses_unchanged(const uint32_t original[29],uint32_t gap,
                              const s3_barcode_config_t *cfg)
{
    uint32_t runs[29];int32_t bias[2]={123,456};
    memcpy(runs,original,sizeof(runs));
    assert(!s3_barcode_record_correct_varying_runs(runs,gap,cfg,bias));
    assert(memcmp(runs,original,sizeof(runs))==0);
}

static void test_bad_fits(void)
{
    s3_barcode_config_t cfg=s3_barcode_defaults(),bad=cfg;
    uint32_t runs[29];unsigned int i;
    make_runs(0U,runs);distort(runs,20000,-20000);
    refuses_unchanged(runs,10000U,&cfg); /* change of bias sign */
    make_runs(0U,runs);distort(runs,70000,5000);
    refuses_unchanged(runs,10000U,&cfg); /* change exceeds half a narrow module */
    make_runs(0U,runs);
    for(i=0U;i<4U;i++) { uint32_t v=runs[20U+i];runs[20U+i]=runs[28U-i];runs[28U-i]=v; }
    refuses_unchanged(runs,10000U,&cfg); /* opposite sentinel orientations */
    make_runs(0U,runs);runs[20]=0U;
    refuses_unchanged(runs,10000U,&cfg);
    make_runs(0U,runs);distort(runs,60000,20000);
    refuses_unchanged(runs,40000U,&cfg); /* sampling gap */
    bad.quiet_modules=0U;refuses_unchanged(runs,10000U,&bad);
}

/* Extracted from the 18:26:59 BCAP trace at dt_ms*1000+500 us. Only the
 * widths are measured here; this test explicitly synthesises sharp levels
 * and white margins to exercise the streaming candidate/quiet state machine. */
static const uint32_t measured_a[29]={
    90168,532275,39187,286517,303129,353667,284937,300000,43563,270000,
    323734,304891,45700,240328,55231,565541,70736,241099,372331,211134,
    104314,481392,133752,189043,410586,202229,360209,268500,54666};

static unsigned int feed_measured(const uint32_t runs[29],uint32_t lead,uint32_t trail,
                                 bool interrupt,uint32_t *decoded_after)
{
    static s3_barcode_record_t record;
    const s3_barcode_config_t cfg=s3_barcode_defaults();
    uint32_t finish=lead,t;unsigned int i,decoded=0U;
    for(i=0U;i<29U;i++) { finish+=runs[i]; }
    s3_barcode_record_reset(&record);*decoded_after=0U;
    for(t=0U;t<finish+trail;t+=10000U) {
        uint32_t edge=lead;uint16_t dark=10U;
        s3_barcode_result_t result;s3_barcode_status_t status;
        for(i=0U;i<29U;i++) {
            if(t>=edge && t<edge+runs[i]) { dark=(i%2U)?10U:900U;break; }
            edge+=runs[i];
        }
        if(interrupt && t>=finish+1500000U && t<finish+1570000U) { dark=70U; }
        status=s3_barcode_record_update(&record,&cfg,t+10000U,true,dark,&result);
        assert(status!=S3_BARCODE_TIMING_ERROR && status!=S3_BARCODE_OVERFLOW);
        if(status==S3_BARCODE_DECODED) {
            decoded++;assert(result.letter=='A');*decoded_after=t-finish;
        }
    }
    return decoded;
}

static void test_measured_stars_and_quiet(void)
{
    const s3_barcode_config_t cfg=s3_barcode_defaults();
    uint32_t runs[29],elapsed;int32_t shared,bias[2];s3_barcode_result_t result;
    unsigned int i;
    memcpy(runs,measured_a,sizeof(runs));
    assert(!s3_barcode_record_correct_runs(runs,12500U,&cfg,&shared));
    assert(s3_barcode_record_correct_varying_runs(runs,12500U,&cfg,bias));
    assert(bias[0]>120000 && bias[0]<140000 && bias[1]>60000 && bias[1]<75000);
    assert(s3_barcode_decode_runs(runs,29U,2200000U,2000000U,12500U,&cfg,&result));
    assert(result.letter=='A');
    assert(feed_measured(measured_a,2200000U,2000000U,false,&elapsed)==1U);
    /* End star's narrow mean is ~159 ms. Its expansion consumes additional
     * trailing white; a candidate must not emit as soon as one second passes. */
    assert(elapsed>=1600000U);
    /* These margins exceed ten observed narrow modules, but not ten modules
     * plus the outward movement of the corresponding outer edge. */
    assert(feed_measured(measured_a,1890000U,2000000U,false,&elapsed)==0U);
    assert(feed_measured(measured_a,2200000U,1600000U,false,&elapsed)==0U);
    assert(feed_measured(measured_a,2200000U,2000000U,true,&elapsed)==0U);
    memcpy(runs,measured_a,sizeof(runs));
    for(i=10U;i<19U;i++) { runs[i]=170000U; }
    assert(feed_measured(runs,2200000U,2000000U,false,&elapsed)==0U);
}

int main(void)
{
    test_letters_and_payload_independence();
    test_bad_fits();
    test_measured_stars_and_quiet();
    puts("PASS: varying sentinel correction, all letters/signs/directions/slopes, payload independence, "
         "invalid payload, inconsistent stars, bounded drift, unchanged failure inputs, measured A and actual quiet waiting");
    return 0;
}
