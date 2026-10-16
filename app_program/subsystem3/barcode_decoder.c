/* Code 39 symbol facts cross-checked against ZXing Code39Reader encodings:
 * https://github.com/zxing/zxing/blob/master/core/src/main/java/com/google/zxing/oned/Code39Reader.java
 * This decoder uses its own bounded duration/sampling validation algorithm. */
#include "barcode_decoder.h"
static const uint16_t g_letters[26] = {
    0x109U,0x049U,0x148U,0x019U,0x118U,0x058U,0x00DU,0x10CU,0x04CU,0x01CU,
    0x103U,0x043U,0x142U,0x013U,0x112U,0x052U,0x007U,0x106U,0x046U,0x016U,
    0x181U,0x0C1U,0x1C0U,0x091U,0x190U,0x0D0U
};
s3_barcode_config_t s3_barcode_defaults(void)
{
    return (s3_barcode_config_t){30000U,30000U,1000000U,10000000U,10U};
}
static bool config_valid(const s3_barcode_config_t *config)
{
    return (config != 0) && (config->max_gap_us > 0U) &&
        (config->min_narrow_us > 0U) && (config->max_run_us >= config->min_narrow_us) &&
        (config->max_frame_us > config->max_run_us) && (config->max_frame_us < 0x80000000U) &&
        (config->max_gap_us <= config->max_run_us) &&
        (config->quiet_modules >= 10U) && (config->quiet_modules <= 50U) &&
        ((uint64_t)config->max_run_us*config->quiet_modules < 0x80000000ULL);
}
void s3_barcode_reset(s3_barcode_decoder_t *decoder)
{
    if (decoder != 0) {
        s3_barcode_config_t config=decoder->config;
        bool configured=decoder->configured;
        *decoder=(s3_barcode_decoder_t){0};
        decoder->config=config;
        decoder->configured=configured;
    }
}
bool s3_barcode_init(s3_barcode_decoder_t *decoder, const s3_barcode_config_t *config)
{
    if (decoder == 0) { return false; }
    *decoder=(s3_barcode_decoder_t){0};
    if (!config_valid(config)) { return false; }
    decoder->config=*config;
    decoder->configured=true;
    return true;
}
/* Exactly six narrow and three wide elements, internally consistent to 35%.
 * Width ratio 1.8..3.5 permits rasterisation/jitter around nominal 2..3. */
static bool character(const uint32_t *runs, uint32_t gap,
    const s3_barcode_config_t *config, uint16_t *pattern, uint32_t *narrow)
{
    uint32_t sorted[9];
    uint64_t nsum=0U,wsum=0U;
    uint32_t wide,split;
    unsigned int i,j;
    *pattern=0U;
    for(i=0U;i<9U;i++) {
        if ((runs[i]==0U) || (runs[i]>config->max_run_us)) { return false; }
        sorted[i]=runs[i];
    }
    for(i=1U;i<9U;i++) {
        uint32_t item=sorted[i];j=i;
        while((j>0U) && (sorted[j-1U]>item)) { sorted[j]=sorted[j-1U];j--; }
        sorted[j]=item;
    }
    for(i=0U;i<6U;i++) { nsum+=sorted[i]; }
    for(i=6U;i<9U;i++) { wsum+=sorted[i]; }
    *narrow=(uint32_t)(nsum/6U);wide=(uint32_t)(wsum/3U);
    if ((*narrow<config->min_narrow_us) || ((uint64_t)*narrow<3ULL*gap) ||
        ((uint64_t)wide*100U<(uint64_t)*narrow*180U) ||
        ((uint64_t)wide*100U>(uint64_t)*narrow*350U)) { return false; }
    split=(uint32_t)(((uint64_t)sorted[5]+sorted[6])/2U);
    for(i=0U;i<9U;i++) {
        bool is_wide=runs[i]>split;
        uint32_t reference=is_wide?wide:*narrow;
        if (((uint64_t)runs[i]*100U<(uint64_t)reference*65U) ||
            ((uint64_t)runs[i]*100U>(uint64_t)reference*135U)) { return false; }
        *pattern=(uint16_t)((*pattern<<1)|(is_wide?1U:0U));
    }
    return true;
}
bool s3_barcode_decode_runs(const uint32_t *runs, unsigned int count,
    uint32_t leading_us, uint32_t trailing_us, uint32_t observed_gap_us,
    const s3_barcode_config_t *config, s3_barcode_result_t *result)
{
    unsigned int orientation,i,k;
    uint64_t total=0U;
    if (result == 0) { return false; }
    *result=(s3_barcode_result_t){0};
    if ((runs==0) || (count!=S3_BARCODE_RUNS) || !config_valid(config) ||
        (observed_gap_us>config->max_gap_us)) { return false; }
    for(i=0U;i<count;i++) {
        if ((runs[i]==0U) || (runs[i]>config->max_run_us)) { return false; }
        total+=runs[i];
    }
    if (total>config->max_frame_us) { return false; }
    for(orientation=0U;orientation<2U;orientation++) {
        uint32_t ordered[S3_BARCODE_RUNS],narrow[3];
        uint16_t pattern[3];
        bool good=true;
        for(i=0U;i<count;i++) { ordered[i]=runs[orientation?count-1U-i:i]; }
        for(i=0U;i<3U;i++) {
            if(!character(&ordered[i*10U],observed_gap_us,config,&pattern[i],&narrow[i])) { good=false;break; }
        }
        if(!good || (pattern[0]!=0x094U) || (pattern[2]!=0x094U)) { continue; }
        for(i=0U;i<2U;i++) {
            uint64_t reference=((uint64_t)narrow[i]+narrow[i+1U])/2U;
            uint64_t space=ordered[9U+i*10U];
            if((space*2U<reference)||(space*2U>reference*3U)) { good=false; }
        }
        if(!good || ((uint64_t)(orientation?trailing_us:leading_us)<(uint64_t)config->quiet_modules*narrow[0]) ||
            ((uint64_t)(orientation?leading_us:trailing_us)<(uint64_t)config->quiet_modules*narrow[2])) { continue; }
        for(k=0U;k<26U;k++) {
            if(pattern[1]==g_letters[k]) {
                result->letter=(char)('A'+k);result->reversed=orientation!=0U;return true;
            }
        }
    }
    return false;
}
static void seed(s3_barcode_decoder_t *decoder,uint32_t now,bool black)
{
    s3_barcode_reset(decoder);
    decoder->seen=true;decoder->black=black;
    decoder->last_us=now;decoder->run_start_us=now;
}
s3_barcode_status_t s3_barcode_update(s3_barcode_decoder_t *decoder,
    uint32_t timestamp_us, bool valid, bool black, s3_barcode_result_t *result)
{
    uint32_t gap,elapsed;
    if(result==0) { return S3_BARCODE_REJECTED; }
    *result=(s3_barcode_result_t){0};
    if((decoder==0)||!decoder->configured) { return S3_BARCODE_REJECTED; }
    if(!valid) { s3_barcode_reset(decoder);return S3_BARCODE_REJECTED; }
    if(!decoder->seen) { seed(decoder,timestamp_us,black);return S3_BARCODE_NONE; }
    gap=timestamp_us-decoder->last_us;
    if((gap==0U)||(gap>decoder->config.max_gap_us)) {
        seed(decoder,timestamp_us,black);return S3_BARCODE_TIMING_ERROR;
    }
    decoder->last_us=timestamp_us;
    elapsed=timestamp_us-decoder->run_start_us;
    if(decoder->collecting) {
        if(gap>decoder->observed_gap_us) { decoder->observed_gap_us=gap; }
        if((uint32_t)(timestamp_us-decoder->frame_start_us)>decoder->config.max_frame_us) {
            seed(decoder,timestamp_us,black);return S3_BARCODE_REJECTED;
        }
    }
    if(decoder->pending) {
        if(black) { seed(decoder,timestamp_us,black);return S3_BARCODE_REJECTED; }
        if(elapsed>=decoder->trailing_required_us) {
            *result=decoder->candidate;result->timestamp_us=timestamp_us;
            decoder->collecting=false;decoder->pending=false;decoder->count=0U;
            return S3_BARCODE_DECODED;
        }
        return S3_BARCODE_NONE;
    }
    if(black!=decoder->black) {
        if(decoder->collecting) {
            if(decoder->count>=S3_BARCODE_RUNS) {
                seed(decoder,timestamp_us,black);return S3_BARCODE_OVERFLOW;
            }
            decoder->runs[decoder->count++]=elapsed;
            if(decoder->count==S3_BARCODE_RUNS) {
                uint16_t pattern;
                uint32_t narrow;
                if(black || !s3_barcode_decode_runs(decoder->runs,decoder->count,
                    decoder->leading_us,0x7FFFFFFFU,decoder->observed_gap_us,
                    &decoder->config,&decoder->candidate) ||
                    !character(&decoder->runs[20],decoder->observed_gap_us,
                        &decoder->config,&pattern,&narrow)) {
                    seed(decoder,timestamp_us,black);return S3_BARCODE_REJECTED;
                }
                decoder->trailing_required_us=narrow*decoder->config.quiet_modules;
                decoder->pending=true;
            }
        } else if(black) {
            decoder->leading_us=elapsed;decoder->frame_start_us=timestamp_us;
            decoder->collecting=true;decoder->count=0U;decoder->observed_gap_us=gap;
        }
        decoder->black=black;decoder->run_start_us=timestamp_us;
    } else if(decoder->collecting && (elapsed>decoder->config.max_run_us)) {
        seed(decoder,timestamp_us,black);return S3_BARCODE_REJECTED;
    }
    return S3_BARCODE_NONE;
}
