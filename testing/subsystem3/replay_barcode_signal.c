/* stdin: elapsed_us raw_adc valid(0/1). Optional SYNTHETIC lead/trail ms,
 * paper ADC and polarity. Never substitutes normalized dashboard previews. */
#include <stdio.h>
#include <stdlib.h>
#include "barcode_signal.h"
static s3_barcode_signal_t signal;
static uint32_t now;
static unsigned int decoded, rejected, timing, overflow;
static bool high=true;
static void feed(unsigned int dt, unsigned int raw, bool valid)
{
    const s3_barcode_config_t config=s3_barcode_defaults();
    s3_barcode_result_t result;
    now+=dt;
    s3_barcode_status_t status=s3_barcode_signal_update(&signal,&config,now,valid,
        (uint16_t)raw,high,&result);
    if (status==S3_BARCODE_DECODED) {
        decoded++;
        printf("letter=%c reversed=%u timestamp_us=%u\n",result.letter,result.reversed,now);
    } else if (status==S3_BARCODE_REJECTED) { rejected++; }
    else if (status==S3_BARCODE_TIMING_ERROR) { timing++; }
    else if (status==S3_BARCODE_OVERFLOW) { overflow++; }
    if (status!=S3_BARCODE_NONE) {
        printf("status=%u frame_status=%u\n",(unsigned)status,signal.frame_status);
    }
}
int main(int argc, char **argv)
{
    unsigned int dt, raw, valid;
    unsigned long lead=argc>1 ? strtoul(argv[1],0,10) : 0U;
    unsigned long trail=argc>2 ? strtoul(argv[2],0,10) : 0U;
    unsigned long paper=argc>3 ? strtoul(argv[3],0,10) : 170U;
    high=argc<=4 || strtoul(argv[4],0,10)!=0U;
    if (lead>10000U || trail>10000U || paper>4095U) { return 2; }
    printf("synthetic_leading_ms=%lu synthetic_trailing_ms=%lu\n",lead,trail);
    for (unsigned int i=0U; i<lead/10U; i++) { feed(10000U,(unsigned int)paper,true); }
    while (scanf("%u %u %u",&dt,&raw,&valid)==3) {
        if (dt>1000000U || raw>65535U || valid>1U) { return 2; }
        feed(dt,raw,valid!=0U);
    }
    for (unsigned int i=0U; i<trail/10U; i++) { feed(10000U,(unsigned int)paper,true); }
    printf("decoded=%u rejected=%u timing=%u overflow=%u active=%u count=%u frame_status=%u\n",
        decoded,rejected,timing,overflow,signal.recording,signal.count,signal.frame_status);
    return 0;
}
