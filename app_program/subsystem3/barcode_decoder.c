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
    /* At 0.04 m/s a 19 mm wide element lasts 475 ms. A 2:1 symbol can
     * require ~11.4 s including trailing quiet; allow bounded extra time. */
    return (s3_barcode_config_t){30000U,30000U,1000000U,15000000U,10U};
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
        s3_barcode_stored_config_t config=decoder->config;
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
    decoder->config=(s3_barcode_stored_config_t){config->max_gap_us,
        config->min_narrow_us,config->max_run_us,config->max_frame_us,config->quiet_modules};
    decoder->configured=true;
    return true;
}
/* Exactly six narrow and three wide elements, normally consistent to 35%.
 * Only the payload may contain one narrow white gap up to 150% of its mean.
 * Width ratio 1.8..3.5 permits rasterisation/jitter around nominal 2..3. */
static bool character_widths(const uint32_t *runs, uint32_t gap,
    const s3_barcode_config_t *config, uint16_t *pattern, uint32_t *narrow,
    bool payload)
{
    uint32_t sorted[9];
    uint64_t nsum=0U,wsum=0U;
    uint32_t wide,split;
    unsigned int i,j,extended_spaces=0U;
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
        const uint64_t scaled=(uint64_t)runs[i]*100U;
        if (scaled<(uint64_t)reference*S3_BARCODE_WIDTH_MIN_PERCENT) { return false; }
        if (scaled>(uint64_t)reference*S3_BARCODE_WIDTH_MAX_PERCENT) {
            if (!payload || is_wide || !(i%2U) || (extended_spaces!=0U) ||
                (scaled>(uint64_t)reference*S3_BARCODE_PAYLOAD_SPACE_MAX_PERCENT)) {
                return false;
            }
            extended_spaces++;
        }
        *pattern=(uint16_t)((*pattern<<1)|(is_wide?1U:0U));
    }
    return true;
}
/* Sentinel fitting always uses the original strict width checks. */
static bool character(const uint32_t *runs, uint32_t gap,
    const s3_barcode_config_t *config, uint16_t *pattern, uint32_t *narrow)
{
    return character_widths(runs,gap,config,pattern,narrow,false);
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
            if(!character_widths(&ordered[i*10U],observed_gap_us,config,
                &pattern[i],&narrow[i],i==1U)) { good=false;break; }
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
            result->letter=decoder->candidate.letter_code == 0U ? 0 :
                (char)('A'+decoder->candidate.letter_code-1U);
            result->reversed=decoder->candidate.reversed;
            result->timestamp_us=timestamp_us;
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
                s3_barcode_result_t candidate;
                const s3_barcode_config_t config={decoder->config.max_gap_us,
                    decoder->config.min_narrow_us,decoder->config.max_run_us,
                    decoder->config.max_frame_us,decoder->config.quiet_modules};
                if(black || !s3_barcode_decode_runs(decoder->runs,decoder->count,
                    decoder->leading_us,0x7FFFFFFFU,decoder->observed_gap_us,
                    &config,&candidate) ||
                    !character(&decoder->runs[20],decoder->observed_gap_us,
                        &config,&pattern,&narrow)) {
                    seed(decoder,timestamp_us,black);return S3_BARCODE_REJECTED;
                }
                decoder->candidate=(s3_barcode_stored_result_t){
                    (uint8_t)(candidate.letter-'A'+1),candidate.reversed};
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

/* ---- Record-then-decide reader ------------------------------------------ */
#define REC_DARK(r,i) ((unsigned int)((r)->sample[i] & 0x3FFU))
#define REC_STEP_US(r,i) ((uint32_t)((r)->sample[i] >> 10) * S3_REC_STEP_US)
void s3_barcode_record_reset(s3_barcode_record_t *record)
{
    if (record != 0) { *record=(s3_barcode_record_t){0}; }
}
bool s3_barcode_record_active(const s3_barcode_record_t *record)
{
    return (record != 0) && (record->state != (uint16_t)S3_REC_WAITING);
}
/* Elapsed time as a 6-bit step, carrying the rounding remainder forward so
 * summed steps track real time. False when the gap does not fit (stall). */
static bool rec_step(s3_barcode_record_t *r, uint32_t gap_us, uint16_t *step)
{
    uint32_t total=gap_us+r->residual_us, units=total/S3_REC_STEP_US;
    if ((gap_us == 0U) || (units > S3_REC_STEP_MAX)) { return false; }
    r->residual_us=(uint16_t)(total-units*S3_REC_STEP_US);
    *step=(uint16_t)units;
    return true;
}
/* Calibration can clamp a quiet paper background near zero while narrow
 * bars still rise only 10..25 units. Keep their small turns and quiet-zone
 * interruptions visible. The normal batch extractor retains its learned
 * prominence; this only enables a bounded second extraction if bars are lost. */
static bool rec_faint_eligible(const s3_barcode_record_t *r)
{
    return r != 0 && r->paper <= S3_REC_FAINT_BAND &&
        r->prominence != 0U && r->prominence <= S3_REC_FAINT_MAX_NOISE;
}
static unsigned int rec_monitor_band(const s3_barcode_record_t *r)
{
    return rec_faint_eligible(r) ? S3_REC_FAINT_BAND : r->prominence;
}
static bool rec_white(const s3_barcode_record_t *r, unsigned int dark)
{
    unsigned int band=S3_REC_WHITE_BAND;
    const unsigned int monitor=rec_monitor_band(r);
    if ((monitor != 0U) && (monitor < band)) { band=monitor; }
    return dark <= (unsigned int)r->paper + band;
}
static uint32_t sat_add(uint32_t a, uint32_t b)
{
    return (a > 0x7FFFFFFFU - b) ? 0x7FFFFFFFU : a + b;
}
/* Keep real, timestamped readings while waiting. The first ring entry has no
 * preceding step: dropping it also drops the next entry's incoming interval. */
static void rec_pre_drop(s3_barcode_record_t *r)
{
    const bool was_white=(r->pre[r->pre_head]&0x3FFU)<=S3_REC_FAINT_BAND;
    uint32_t elapsed;
    r->pre_head=(uint16_t)((r->pre_head+1U)%S3_REC_PRE_MAX);
    r->pre_count--;
    elapsed=(uint32_t)(r->pre[r->pre_head]>>10)*S3_REC_STEP_US;
    /* While waiting, leading_us measures verified low-band white BEFORE
     * the retained history. A faint mark that falls out of the ring must
     * still break that margin; the stronger recording trigger cannot prove
     * that everything before it was white. The absolute band is conservative
     * for all eligible paper levels (0..5), even if the reference changes. */
    r->leading_us=was_white && ((r->pre[r->pre_head]&0x3FFU)<=S3_REC_FAINT_BAND) ?
        sat_add(r->leading_us,elapsed) : 0U;
    r->pre_us-=elapsed;
    r->pre[r->pre_head]&=0x3FFU;
}
static void rec_pre_push(s3_barcode_record_t *r, unsigned int dark, uint16_t step)
{
    unsigned int at;
    if (r->pre_count == S3_REC_PRE_MAX) { rec_pre_drop(r); }
    if (r->pre_count == 0U) { step=0U; }
    at=((unsigned int)r->pre_head+r->pre_count)%S3_REC_PRE_MAX;
    r->pre[at]=(uint16_t)(dark|((unsigned int)step<<10));
    r->pre_count++;
    r->pre_us+=(uint32_t)step*S3_REC_STEP_US;
    while ((r->pre_count>1U) && (r->pre_us>S3_REC_PRE_US)) { rec_pre_drop(r); }
}
/* The waiting EWMA can include faint ink below the stronger recording trigger.
 * If the retained history starts with an actual blank-paper interval, measure
 * its noise alone. Stop at the first ink sample; never use inter-bar valleys
 * to certify a noisy background as quiet. Otherwise retain the EWMA estimate. */
static unsigned int rec_paper_prominence(const s3_barcode_record_t *r,
    const s3_barcode_config_t *config)
{
    unsigned int prominence=(r->noise_q4+3U)/4U;
    if (r->paper<=S3_REC_FAINT_BAND && r->pre_count>1U) {
        const unsigned int floor=(unsigned int)r->paper+S3_REC_FAINT_BAND;
        unsigned int previous=r->pre[r->pre_head]&0x3FFU, differences=0U;
        uint32_t elapsed=0U, sum=0U;
        for (unsigned int i=1U; previous<=floor && i<r->pre_count; i++) {
            const uint16_t entry=r->pre[((unsigned int)r->pre_head+i)%S3_REC_PRE_MAX];
            const unsigned int dark=entry&0x3FFU;
            if (dark>floor) { break; }
            elapsed+=(uint32_t)(entry>>10)*S3_REC_STEP_US;
            sum+=(dark>previous) ? dark-previous : previous-dark;
            differences++;
            previous=dark;
        }
        if (differences!=0U && elapsed>=config->quiet_modules*config->min_narrow_us) {
            prominence=(4U*sum+differences-1U)/differences;
        }
    }
    if (prominence<S3_REC_MIN_PROMINENCE) { prominence=S3_REC_MIN_PROMINENCE; }
    if (prominence>S3_REC_PROMINENCE) { prominence=S3_REC_PROMINENCE; }
    return prominence;
}
/* Edge time where the readings cross mid between samples i and i+1. */
/* How far reading d still sits at the start level lo of a segment that ends
 * at level hi: 1024 = still at lo, 0 = already at hi. */
static uint32_t rec_frac(unsigned int d, unsigned int lo, unsigned int hi)
{
    int32_t num=(int32_t)d-(int32_t)hi, den=(int32_t)lo-(int32_t)hi;
    /* 10-bit samples bound num*1024 to +/-1,047,552. Keep this exact
     * operation 32-bit: RP2040 otherwise calls a slow 64-bit divide twice
     * per sample while processing the completed recording. */
    int32_t f=(den == 0) ? 0 : (num*1024)/den;
    return (f < 0) ? 0U : ((f > 1024) ? 1024U : (uint32_t)f);
}
/* Group low-contrast ink regions, retaining one strongest peak per region.
 * A bar must return below the noise band for more than 30 ms before another
 * can start. Internal plateau wiggles therefore cannot replace a faint bar.
 * Require exactly 15 complete regions: never prune this path to a desired
 * count, infer a missing bar, or use an expected letter to choose peaks. */
static unsigned int rec_faint_extrema(const s3_barcode_record_t *r,
    uint16_t ext[S3_REC_EXT_MAX])
{
    const unsigned int floor=(unsigned int)r->paper+S3_REC_FAINT_BAND;
    unsigned int cursor=0U, n=1U;
    ext[0]=0U;
    while (cursor<r->count) {
        unsigned int peak, dip;
        uint32_t low_us=0U;
        while (cursor<r->count && REC_DARK(r,cursor)<=floor) { cursor++; }
        if (cursor==r->count) { break; }
        if (n+2U>S3_BARCODE_RUNS+2U) { return 0U; }
        peak=cursor++; dip=peak;
        while (cursor<r->count) {
            if (REC_DARK(r,cursor)>floor) {
                if (REC_DARK(r,cursor)>REC_DARK(r,peak)) { peak=cursor; }
                low_us=0U;
            } else {
                if (low_us==0U || REC_DARK(r,cursor)<REC_DARK(r,dip)) { dip=cursor; }
                low_us+=REC_STEP_US(r,cursor);
                if (low_us>S3_REC_FAINT_DIP_US) { break; }
            }
            cursor++;
        }
        if (low_us<=S3_REC_FAINT_DIP_US) { return 0U; } /* incomplete last bar */
        /* Include the entire valley, including its first low readings. */
        while (cursor<r->count && REC_DARK(r,cursor)<=floor) {
            if (REC_DARK(r,cursor)<REC_DARK(r,dip)) { dip=cursor; }
            cursor++;
        }
        ext[n++]=(uint16_t)peak;
        ext[n++]=(uint16_t)dip;
    }
    return n==S3_BARCODE_RUNS+2U ? n : 0U;
}
static unsigned int rec_extract_runs(const s3_barcode_record_t *record,
    uint32_t runs[S3_BARCODE_RUNS], uint32_t *first_edge_us,
    uint32_t *last_edge_us, uint32_t *end_us, uint32_t *max_step_us, bool faint)
{
    /* Alternating dip, bump, dip, ... indices; 15 bars need 31 extremes. */
    uint16_t ext[S3_REC_EXT_MAX];
    uint32_t edge_prev=0U, t=0U;
    unsigned int n=1U, best=0U, i, k=0U, edges=0U;
    bool rising=true;
    if ((record == 0) || (runs == 0) || (first_edge_us == 0) || (last_edge_us == 0) ||
        (end_us == 0) || (max_step_us == 0) || (record->count < 3U) ||
        (record->count > S3_REC_MAX)) { return 0U; }
    *max_step_us=0U;
    ext[0]=0U;
    if (faint) {
        n=rec_faint_extrema(record,ext);
        if (n==0U) { return 0U; }
        for (i=1U; i<record->count; i++) {
            if (REC_STEP_US(record,i)>*max_step_us) { *max_step_us=REC_STEP_US(record,i); }
        }
    } else {
        for (i=1U; i<record->count; i++) {
            unsigned int d=REC_DARK(record,i), b=REC_DARK(record,best);
            if (REC_STEP_US(record,i) > *max_step_us) { *max_step_us=REC_STEP_US(record,i); }
            if (rising ? (d > b) : (d < b)) { best=i; }
            else if ((rising ? b-d : d-b) >=
                     (record->prominence ? record->prominence : S3_REC_PROMINENCE)) {
                if (n >= S3_REC_EXT_MAX-1U) { return 0U; }   /* far too many stripes */
                ext[n++]=(uint16_t)best; best=i; rising=!rising;
            }
        }
        if (!rising) { ext[n++]=(uint16_t)best; }  /* dip after the last bump */
        if (n < 3U) { return 0U; }                 /* no complete bar */
        /* A sag or scratch inside a wide bar (or a smudge inside a wide gap)
         * adds an extra dip/bump pair. While there are more than 31 extremes,
         * drop the neighbouring interior pair that is "smallest" = height times
         * time apart: a wiggle is shallow or brief, a real stripe is deep and
         * about one narrow module long. The strict Code 39 checks still have to
         * pass afterwards; they reduce false accepts but are not a checksum. */
        while (n > S3_BARCODE_RUNS+2U) {
            unsigned int j, m, cut=1U;
            uint64_t small=UINT64_MAX;
            for (j=1U; j+2U < n; j++) {
                unsigned int a=REC_DARK(record,ext[j]), b=REC_DARK(record,ext[j+1U]);
                uint64_t span=0U, size;
                for (m=(unsigned int)ext[j]+1U; m<=(unsigned int)ext[j+1U]; m++) { span+=REC_STEP_US(record,m); }
                size=(uint64_t)((a > b) ? a-b : b-a)*span;
                if (size < small) { small=size; cut=j; }
            }
            for (j=cut; j+2U < n; j++) { ext[j]=ext[j+2U]; }
            n-=2U;
        }
    }
    /* Edges of the retained bumps: first rising / last falling crossing of
     * the local halfway level. Once an interior dip has been merged into a
     * bar above, its early downward crossing must not end that bar while a
     * later rebound still crosses the same level. Neither extraction path
     * uses the expected letter or marker widths to choose edges.
     * Keep the time-weighted estimate only as a fallback if no crossing is
     * available. All stripe, marker and actual quiet checks still follow. */
    {
        uint32_t t_lo=0U, at_lo=0U;
        for (i=1U; i<record->count; i++) {
            uint32_t step=REC_STEP_US(record,i);
            t+=step;
            if (k+1U < n) {
                unsigned int lo=REC_DARK(record,ext[k]), hi=REC_DARK(record,ext[k+1U]);
                /* step<=31500, each fraction<=1024: product<=64,512,000.
                 * Across 2048 readings the elapsed sum fits uint32_t too. */
                at_lo+=(step*(rec_frac(REC_DARK(record,i-1U),lo,hi)+
                              rec_frac(REC_DARK(record,i),lo,hi)))/2048U;
                if (i == (unsigned int)ext[k+1U]) {
                    const uint32_t guess=t_lo+(uint32_t)at_lo;
                    const unsigned int mid=(lo+hi)/2U;
                    const bool up=hi > lo;
                    /* Near the noise floor, measure the half-height core
                     * connected to the maximum. An earlier weak shoulder
                     * must not consume the neighbouring white gap. Strong
                     * bars retain the outer crossings used for wide dips. */
                    const bool core=faint &&
                        ((up ? hi : lo)-(unsigned int)record->paper<=S3_REC_RISE);
                    uint32_t edge=guess, tj=t_lo;
                    bool crossed=false;
                    unsigned int j;
                    for (j=(unsigned int)ext[k]; j<i; j++) {
                        unsigned int a=REC_DARK(record,j), b=REC_DARK(record,j+1U);
                        uint32_t sj=REC_STEP_US(record,j+1U);
                        const bool crossing=up ?
                            (core ? (a<=mid && b>mid) : (a<mid && b>=mid)) :
                            (a>mid && b<=mid);
                        if (crossing) {
                            uint32_t x=tj+(sj*(uint32_t)(up ? mid-a : a-mid))/
                                            (uint32_t)(up ? b-a : a-b);
                            if (!crossed || (core ? up : !up)) { edge=x; }
                            crossed=true;
                        }
                        tj+=sj;
                    }
                    if (edges == 0U) { *first_edge_us=edge; }
                    else { runs[edges-1U]=edge-edge_prev; }
                    edge_prev=edge; edges++; k++;
                    t_lo=t; at_lo=0U;
                }
            }
        }
    }
    *last_edge_us=edge_prev;
    *end_us=t;
    return (edges >= 2U) ? edges-1U : 0U;
}
unsigned int s3_barcode_record_runs(const s3_barcode_record_t *record,
    uint32_t runs[S3_BARCODE_RUNS], uint32_t *first_edge_us,
    uint32_t *last_edge_us, uint32_t *end_us, uint32_t *max_step_us)
{
    unsigned int n=rec_extract_runs(record,runs,first_edge_us,last_edge_us,
                                   end_us,max_step_us,false);
    if (n!=S3_BARCODE_RUNS && rec_faint_eligible(record)) {
        n=rec_extract_runs(record,runs,first_edge_us,last_edge_us,end_us,max_step_us,true);
    }
    return n;
}
/* Average of the six smallest of nine runs (as character() computes narrow). */
static uint32_t rec_narrow(const uint32_t *runs)
{
    uint32_t s[9]; uint64_t sum=0U; unsigned int i,j;
    for (i=0U; i<9U; i++) {
        uint32_t v=runs[i]; j=i;
        while ((j > 0U) && (s[j-1U] > v)) { s[j]=s[j-1U]; j--; }
        s[j]=v;
    }
    for (i=0U; i<6U; i++) { sum+=s[i]; }
    return (uint32_t)(sum/6U);
}
/* Fit only the two known sentinels, never the payload. Both star orientations
 * have three narrow bars and three narrow spaces: their narrow mean is
 * invariant under a shared bar expansion / space contraction. Search a fixed
 * 129-point grid within +/- 3/4 of the smaller narrow mean. Minimise the worst
 * fractional sentinel error, retaining the original 35%/ratio/sample checks.
 * A single bias must explain BOTH stars; ambiguous orientation is refused. */
static bool rec_correct_shared_runs(uint32_t runs[S3_BARCODE_RUNS],
    uint32_t gap, const s3_barcode_config_t *config, int32_t *bias_us,
    bool *sentinels_matched)
{
    unsigned int orientation, found=0U, i, c;
    int32_t chosen=0;
    if (sentinels_matched!=0) { *sentinels_matched=false; }
    if ((runs == 0) || (bias_us == 0) || !config_valid(config) ||
        (gap > config->max_gap_us)) { return false; }
    *bias_us=0;
    for (orientation=0U; orientation<2U; orientation++) {
        uint16_t star=orientation ? 0x052U : 0x094U;
        uint32_t minimum=UINT32_MAX, best_score=UINT32_MAX;
        int32_t bound, best_bias=0;
        int tick;
        for (c=0U; c<2U; c++) {
            uint64_t sum=0U;
            for (i=0U; i<9U; i++) {
                if (!(star & (1U<<(8U-i)))) { sum+=runs[c*20U+i]; }
            }
            if (sum/6U < minimum) { minimum=(uint32_t)(sum/6U); }
        }
        if ((minimum < config->min_narrow_us) || (minimum > config->max_run_us)) { continue; }
        bound=(int32_t)(minimum*3U/4U);
        for (tick=-64; tick<=64; tick++) {
            int32_t bias=(int32_t)((int64_t)bound*tick/64);
            uint32_t score=0U;
            bool good=true;
            for (c=0U; (c<2U) && good; c++) {
                uint32_t adjusted[9], narrow, wide=0U;
                uint16_t pattern;
                for (i=0U; i<9U; i++) {
                    int64_t v=(int64_t)runs[c*20U+i]+((i%2U) ? -bias : bias);
                    if ((v<=0) || (v>config->max_run_us)) { good=false; break; }
                    adjusted[i]=(uint32_t)v;
                    if (star & (1U<<(8U-i))) { wide+=adjusted[i]; }
                }
                if (!good || !character(adjusted,gap,config,&pattern,&narrow) ||
                    (pattern != star)) { good=false; break; }
                wide/=3U;
                for (i=0U; i<9U; i++) {
                    uint32_t reference=(star & (1U<<(8U-i))) ? wide : narrow;
                    uint32_t delta=(adjusted[i]>reference) ? adjusted[i]-reference : reference-adjusted[i];
                    uint32_t error=(uint32_t)((uint64_t)delta*1000000U/reference);
                    if (error>score) { score=error; }
                }
            }
            if (good && ((score<best_score) || ((score==best_score) &&
                ((bias<0 ? -bias : bias)<(best_bias<0 ? -best_bias : best_bias))))) {
                best_score=score; best_bias=bias;
            }
        }
        if (best_score != UINT32_MAX) { found++; chosen=best_bias; }
    }
    if (sentinels_matched!=0) { *sentinels_matched=(found!=0U); }
    if (found != 1U) { return false; }
    /* Validate before modifying, including the payload and inter-character
     * spaces; their patterns play no part in choosing the correction. */
    for (i=0U; i<S3_BARCODE_RUNS; i++) {
        int64_t v=(int64_t)runs[i]+((i%2U) ? -chosen : chosen);
        if ((v<=0) || (v>config->max_run_us)) { return false; }
    }
    for (i=0U; i<S3_BARCODE_RUNS; i++) {
        runs[i]=(uint32_t)((int64_t)runs[i]+((i%2U) ? -chosen : chosen));
    }
    *bias_us=chosen;
    return true;
}
bool s3_barcode_record_correct_runs(uint32_t runs[S3_BARCODE_RUNS],
    uint32_t gap, const s3_barcode_config_t *config, int32_t *bias_us)
{
    return rec_correct_shared_runs(runs,gap,config,bias_us,0);
}
/* Single-star fit, using only known sentinel widths. Even-us candidates make
 * the two outer edge shifts exact; no quiet-zone rounding can add margin. */
static bool rec_fit_star(const uint32_t *runs, uint16_t star, uint32_t gap,
    const s3_barcode_config_t *config, int32_t *bias, uint32_t *narrow)
{
    uint64_t sum=0U;
    uint32_t best_score=UINT32_MAX;
    int32_t bound;
    int tick;
    unsigned int i;
    *bias=0;
    for (i=0U;i<9U;i++) { if (!(star & (1U<<(8U-i)))) { sum+=runs[i]; } }
    *narrow=(uint32_t)(sum/6U);
    if ((*narrow<config->min_narrow_us) || (*narrow>config->max_run_us)) { return false; }
    bound=(int32_t)(*narrow*3U/4U);
    for (tick=-64;tick<=64;tick++) {
        int32_t trial=(int32_t)((int64_t)bound*tick/128)*2;
        uint32_t adjusted[9], n, wide=0U, score=0U;
        uint16_t pattern;
        bool good=true;
        for (i=0U;i<9U;i++) {
            int64_t v=(int64_t)runs[i]+((i%2U) ? -trial : trial);
            if ((v<=0) || (v>config->max_run_us)) { good=false;break; }
            adjusted[i]=(uint32_t)v;
            if (star & (1U<<(8U-i))) { wide+=adjusted[i]; }
        }
        if (!good || !character(adjusted,gap,config,&pattern,&n) || (pattern!=star)) { continue; }
        wide/=3U;
        for (i=0U;i<9U;i++) {
            uint32_t reference=(star & (1U<<(8U-i))) ? wide : n;
            uint32_t delta=(adjusted[i]>reference) ? adjusted[i]-reference : reference-adjusted[i];
            uint32_t error=(uint32_t)((uint64_t)delta*1000000U/reference);
            if (error>score) { score=error; }
        }
        if ((score<best_score) || ((score==best_score) &&
            ((trial<0 ? -trial : trial)<(*bias<0 ? -*bias : *bias)))) {
            best_score=score;*bias=trial;
        }
    }
    return best_score!=UINT32_MAX;
}
bool s3_barcode_record_correct_varying_runs(uint32_t runs[S3_BARCODE_RUNS],
    uint32_t gap, const s3_barcode_config_t *config, int32_t bias_us[2])
{
    uint32_t adjusted[S3_BARCODE_RUNS];
    uint64_t total=0U, start=0U, finish=0U, t=0U;
    int32_t chosen[2]={0,0}, previous_shift;
    unsigned int i, orientation, found=0U;
    if ((runs==0) || (bias_us==0) || !config_valid(config) || (gap>config->max_gap_us)) { return false; }
    bias_us[0]=0;bias_us[1]=0;
    for (i=0U;i<S3_BARCODE_RUNS;i++) {
        if ((runs[i]==0U) || (runs[i]>config->max_run_us)) { return false; }
        total+=runs[i];
        if (i==8U) { start=total; }   /* edge 9: end of first star */
        if (i==19U) { finish=total; } /* edge 20: start of last star */
    }
    if (total>config->max_frame_us) { return false; }
    for (orientation=0U;orientation<2U;orientation++) {
        uint16_t star=orientation ? 0x052U : 0x094U;
        uint32_t narrow[2], minimum;
        int32_t bias[2], delta;
        if (!rec_fit_star(runs,star,gap,config,&bias[0],&narrow[0]) ||
            !rec_fit_star(&runs[20],star,gap,config,&bias[1],&narrow[1])) { continue; }
        /* Two endpoints do not justify arbitrary distortion between them. */
        if (((bias[0]<0) && (bias[1]>0)) || ((bias[0]>0) && (bias[1]<0))) { continue; }
        minimum=(narrow[0]<narrow[1]) ? narrow[0] : narrow[1];
        delta=bias[1]-bias[0];
        if ((uint32_t)(delta<0 ? -delta : delta)>minimum/2U) { continue; }
        chosen[0]=bias[0];chosen[1]=bias[1];found++;
    }
    if (found!=1U) { return false; }
    previous_shift=-chosen[0]/2;
    for (i=0U;i<S3_BARCODE_RUNS;i++) {
        int32_t bias, shift;
        int64_t width;
        t+=runs[i];
        if (t<=start) { bias=chosen[0]; }
        else if (t>=finish) { bias=chosen[1]; }
        else {
            /* Weighted interpolation is symmetric on reversing the scan.
             * Use one shift per edge so neighbouring widths stay adjacent. */
            bias=(int32_t)(((int64_t)chosen[0]*(int64_t)(finish-t)+
                           (int64_t)chosen[1]*(int64_t)(t-start))/(int64_t)(finish-start));
        }
        shift=(i%2U) ? -bias/2 : bias/2;
        width=(int64_t)runs[i]+shift-previous_shift;
        if ((width<=0) || (width>config->max_run_us)) { return false; }
        adjusted[i]=(uint32_t)width;
        previous_shift=shift;
    }
    for (i=0U;i<S3_BARCODE_RUNS;i++) { runs[i]=adjusted[i]; }
    bias_us[0]=chosen[0];bias_us[1]=chosen[1];
    return true;
}
/* Validate each independently measured set of edges by the same rules. A
 * count of 29 alone does not prove that the right 15 bumps were selected.
 * Neither extraction nor sentinel fitting uses an expected payload letter. */
static bool rec_candidate(const s3_barcode_record_t *r,
    const s3_barcode_config_t *config, bool faint,
    s3_barcode_result_t *candidate, uint32_t *required_us)
{
    uint32_t runs[S3_BARCODE_RUNS], first=0U, last=0U, end=0U, gap=0U;
    uint32_t leading;
    int32_t bias[2]={0,0};
    int64_t half, need;
    bool shared_matched=false;
    unsigned int n=rec_extract_runs(r,runs,&first,&last,&end,&gap,faint);
    if (n != S3_BARCODE_RUNS) { return false; }
    leading=sat_add(r->leading_us,first);
    /* Trailing white is provisional here; the live reader waits for the
     * real margin below before it releases this candidate. */
    if (!s3_barcode_decode_runs(runs,n,leading,0x7FFFFFFFU,gap,config,candidate)) {
        if (rec_correct_shared_runs(runs,gap,config,&bias[0],&shared_matched)) {
            bias[1]=bias[0];
        } else if (shared_matched ||
            !s3_barcode_record_correct_varying_runs(runs,gap,config,bias)) {
            return false;
        }
        /* Within an extraction, a chosen model is final. Never try another
         * correction merely because its payload or leading margin fails. */
        half=(bias[0]>0) ? ((int64_t)bias[0]+1)/2 : (int64_t)bias[0]/2;
        if ((int64_t)leading<half) { return false; }
        leading=(uint32_t)((int64_t)leading-half);
        if (!s3_barcode_decode_runs(runs,n,leading,0x7FFFFFFFU,gap,config,candidate)) {
            return false;
        }
    }
    /* Expanding a bar moves its outer edge into the quiet zone too.
     * Round toward requiring more white, never inventing extra margin. */
    half=(bias[1]>0) ? ((int64_t)bias[1]+1)/2 : (int64_t)bias[1]/2;
    need=(int64_t)last+half+(int64_t)config->quiet_modules*rec_narrow(&runs[20]);
    *required_us=(need>0x7FFFFFFF) ? 0x7FFFFFFFU : (uint32_t)need;
    return true;
}
static s3_barcode_status_t rec_finish(s3_barcode_record_t *r, s3_barcode_status_t status)
{
    uint16_t paper=r->paper, noise=r->noise_q4; bool seen=r->seen; uint32_t last=r->last_us;
    s3_barcode_record_reset(r);
    r->seen=seen; r->paper=paper; r->last_us=last;   /* keep the white reference */
    r->noise_q4=noise; r->prev=paper;
    /* Like s3_barcode_update: after a failure, only a new white-to-dark
     * start can begin the next recording (no second report mid-bar). */
    r->blocked=(status != S3_BARCODE_DECODED) ? 1U : 0U;
    return status;
}
s3_barcode_status_t s3_barcode_record_update(s3_barcode_record_t *r,
    const s3_barcode_config_t *config, uint32_t timestamp_us, bool valid,
    uint16_t darkness, s3_barcode_result_t *result)
{
    unsigned int dark=(darkness > 1000U) ? 1000U : darkness;
    uint16_t step=0U;
    if ((result == 0) || (r == 0) || !config_valid(config)) { return S3_BARCODE_NONE; }
    *result=(s3_barcode_result_t){0};
    if (!valid) { s3_barcode_record_reset(r); return S3_BARCODE_NONE; }
    if (!r->seen) {
        r->seen=1U; r->paper=(uint16_t)dark; r->last_us=timestamp_us;
        r->prev=(uint16_t)dark; rec_pre_push(r,dark,0U); return S3_BARCODE_NONE;
    }
    if (!rec_step(r,timestamp_us-r->last_us,&step) ||
        ((uint32_t)step*S3_REC_STEP_US > config->max_gap_us+S3_REC_STEP_US)) {
        bool was=s3_barcode_record_active(r);
        s3_barcode_record_reset(r);
        r->seen=1U; r->paper=(uint16_t)dark; r->last_us=timestamp_us; r->prev=(uint16_t)dark;
        rec_pre_push(r,dark,0U);
        return was ? S3_BARCODE_TIMING_ERROR : S3_BARCODE_NONE;
    }
    r->last_us=timestamp_us;
    if (r->state == (uint16_t)S3_REC_WAITING) {
        /* Learn the paper: follow lighter readings at once, darker ones slowly. */
        if (dark < r->paper) { r->paper=(uint16_t)dark; }
        else if (dark < (unsigned int)r->paper+S3_REC_RISE) {
            r->paper=(uint16_t)(r->paper+((dark-r->paper)>>5));
        } /* A starting bar must not raise the paper reference. */
        if (r->blocked) {
            if (!rec_white(r,dark) || (dark >= (unsigned int)r->paper+S3_REC_RISE)) {
                r->white_us=0U; return S3_BARCODE_NONE;
            }
            r->blocked=0U;
        }
        if (dark < (unsigned int)r->paper + S3_REC_RISE) {
            /* Learn noise only near paper, not from the rising bar. Freeze
             * it once recording starts so bars cannot raise their own gate. */
            if ((dark <= (unsigned int)r->paper+S3_REC_WHITE_BAND) &&
                (r->prev <= (unsigned int)r->paper+S3_REC_WHITE_BAND)) {
                unsigned int delta=(dark>r->prev) ? dark-r->prev : r->prev-dark;
                r->noise_q4=(uint16_t)(r->noise_q4-r->noise_q4/16U+delta);
            }
            /* Margin before a barcode: everything short of a bar counts, as
             * in s3_barcode_update (the rising slope is never a bar). */
            r->white_us=sat_add(r->white_us,(uint32_t)step*S3_REC_STEP_US);
            r->prev=(uint16_t)dark;
            rec_pre_push(r,dark,step);
            return S3_BARCODE_NONE;
        }
        /* Trigger on a stronger bar, retaining real earlier readings so a
         * faint opening star/slow edge is not discarded. The history is part
         * of the already-measured leading interval, never extra quiet time. */
        r->state=(uint16_t)S3_REC_RECORDING;
        r->prominence=(uint16_t)rec_paper_prominence(r,config);
        r->prefix_us=r->pre_us;
        if (!rec_faint_eligible(r)) {
            r->leading_us=(r->white_us>=r->prefix_us) ? r->white_us-r->prefix_us : 0U;
        }
        r->white_us=0U; r->frame_us=r->prefix_us; r->required_us=0U;
        r->count=r->pre_count;
        for (unsigned int i=0U;i<r->pre_count;i++) {
            r->sample[i]=r->pre[((unsigned int)r->pre_head+i)%S3_REC_PRE_MAX];
        }
        if (r->count == 0U) { r->sample[0]=r->prev; r->count=1U; }
        r->best=(uint16_t)r->prev; r->falling=0U; r->turn_us=0U;
    }
    if (r->count >= S3_REC_MAX) { return rec_finish(r,S3_BARCODE_OVERFLOW); }
    r->sample[r->count++]=(uint16_t)(dark | ((unsigned int)step << 10));
    r->frame_us=sat_add(r->frame_us,(uint32_t)step*S3_REC_STEP_US);
    /* The batch validator still limits the actual 29 widths to max_frame_us.
     * Leading history must not shorten the existing post-trigger timeout. */
    if (r->frame_us-r->prefix_us > config->max_frame_us) { return rec_finish(r,S3_BARCODE_REJECTED); }
    /* Follow bumps and dips as they come: no stripe is longer than
     * max_run_us, so this long without a turn while dark is a line or a
     * patch, not a barcode (s3_barcode_update rejects the same case). */
    r->turn_us=sat_add(r->turn_us,(uint32_t)step*S3_REC_STEP_US);
    if (r->falling ? (dark < r->best) : (dark > r->best)) { r->best=(uint16_t)dark; }
    else if ((r->falling ? dark-r->best : r->best-dark) >= rec_monitor_band(r)) {
        r->best=(uint16_t)dark; r->falling=r->falling ? 0U : 1U; r->turn_us=0U;
    }
    if ((r->turn_us > config->max_run_us) && !rec_white(r,dark)) {
        return rec_finish(r,S3_BARCODE_REJECTED);
    }
    if (!rec_white(r,dark)) {
        /* Dark after the barcode was judged complete: quiet zone broken. */
        if (r->state == (uint16_t)S3_REC_QUIET) { return rec_finish(r,S3_BARCODE_REJECTED); }
        r->white_us=0U;
        return S3_BARCODE_NONE;
    }
    r->white_us=sat_add(r->white_us,(uint32_t)step*S3_REC_STEP_US);
    /* No stripe is longer than max_run_us, so this much white ends the code. */
    if (r->white_us < config->max_run_us) { return S3_BARCODE_NONE; }
    if (r->state == (uint16_t)S3_REC_RECORDING) {
        s3_barcode_result_t candidate;
        uint32_t required=0U;
        bool good=rec_candidate(r,config,false,&candidate,&required);
        if (rec_faint_eligible(r)) {
            s3_barcode_result_t recovered;
            uint32_t recovery_required=0U;
            if (rec_candidate(r,config,true,&recovered,&recovery_required)) {
                /* Two valid but disagreeing interpretations are ambiguous.
                 * When they agree, require the more conservative margin. */
                if (good && ((candidate.letter!=recovered.letter) ||
                    (candidate.reversed!=recovered.reversed))) {
                    return rec_finish(r,S3_BARCODE_REJECTED);
                }
                candidate=recovered;
                if (!good || recovery_required>required) { required=recovery_required; }
                good=true;
            }
        }
        if (!good) { return rec_finish(r,S3_BARCODE_REJECTED); }
        r->required_us=required;
        r->candidate=(s3_barcode_stored_result_t){(uint8_t)(candidate.letter-'A'+1),candidate.reversed};
        r->state=(uint16_t)S3_REC_QUIET;
    }
    if (r->frame_us<r->required_us) { return S3_BARCODE_NONE; }
    result->letter=(char)('A'+r->candidate.letter_code-1U);
    result->reversed=r->candidate.reversed;
    result->timestamp_us=timestamp_us;
    return rec_finish(r,S3_BARCODE_DECODED);
}
