/* Diagnostic stdin: elapsed_us darkness, one sample per line. No padding by
 * default. Optional arguments add SYNTHETIC leading/trailing white milliseconds
 * to investigate clipped captures, never to claim an observed successful run. */
#include <stdio.h>
#include <stdlib.h>
#include "barcode_decoder.h"

static s3_barcode_record_t record;
static uint32_t now;
static unsigned int decoded, rejected, timing, overflow;

static void feed(unsigned int dt, unsigned int darkness)
{
    const s3_barcode_config_t config=s3_barcode_defaults();
    s3_barcode_result_t result;
    s3_barcode_status_t status;
    now+=dt;
    status=s3_barcode_record_update(&record,&config,now,true,(uint16_t)darkness,&result);
    if (status==S3_BARCODE_DECODED) {
        decoded++;
        printf("letter=%c reversed=%u timestamp_us=%u\n",result.letter,result.reversed,now);
    } else if (status==S3_BARCODE_REJECTED) { rejected++; }
    else if (status==S3_BARCODE_TIMING_ERROR) { timing++; }
    else if (status==S3_BARCODE_OVERFLOW) { overflow++; }
}

int main(int argc, char **argv)
{
    unsigned int dt, dark, i;
    unsigned long lead=argc>1 ? strtoul(argv[1],0,10) : 0U;
    unsigned long trail=argc>2 ? strtoul(argv[2],0,10) : 0U;
    if (lead>10000U || trail>10000U) { return 2; }
    printf("synthetic_leading_ms=%lu synthetic_trailing_ms=%lu\n",lead,trail);
    for (i=0U; i<lead/10U; i++) { feed(10000U,0U); }
    while (scanf("%u %u",&dt,&dark)==2) {
        if (dark>1000U || dt==0U || dt>1000000U) { return 2; }
        feed(dt,dark);
    }
    for (i=0U; i<trail/10U; i++) { feed(10000U,0U); }
    printf("decoded=%u rejected=%u timing=%u overflow=%u state=%u count=%u\n",
        decoded,rejected,timing,overflow,record.state,record.count);
    return 0;
}
