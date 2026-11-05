#include "barcode_signal.h"

/* Values are polarity-oriented raw ADC, never calibrated/clipped darkness. */
static uint32_t entry(const s3_barcode_signal_t *r, unsigned int logical)
{
    unsigned int bit=((r->head+logical)%S3_SIGNAL_CAPACITY)*18U;
    unsigned int byte=bit/8U, shift=bit%8U;
    uint32_t word=(uint32_t)r->samples[byte] |
        (uint32_t)r->samples[byte+1U]<<8 | (uint32_t)r->samples[byte+2U]<<16;
    return (word>>shift)&0x3FFFFU;
}
static unsigned int value(const s3_barcode_signal_t *r, unsigned int i)
{ return entry(r,i)&0xFFFU; }
static uint32_t step(const s3_barcode_signal_t *r, unsigned int i)
{ return i==0U ? 0U : (entry(r,i)>>12)*500U; }
static void append(s3_barcode_signal_t *r, unsigned int raw, unsigned int units)
{
    unsigned int bit=((r->head+r->count)%S3_SIGNAL_CAPACITY)*18U;
    unsigned int byte=bit/8U, shift=bit%8U;
    uint32_t word=(uint32_t)r->samples[byte] |
        (uint32_t)r->samples[byte+1U]<<8 | (uint32_t)r->samples[byte+2U]<<16;
    word=(word&~(0x3FFFFUL<<shift)) | ((raw|(units<<12))<<shift);
    r->samples[byte]=(uint8_t)word;
    r->samples[byte+1U]=(uint8_t)(word>>8);
    r->samples[byte+2U]=(uint8_t)(word>>16);
    r->count++;
}
void s3_barcode_signal_reset(s3_barcode_signal_t *r)
{ if (r!=0) { *r=(s3_barcode_signal_t){0}; } }
bool s3_barcode_signal_active(const s3_barcode_signal_t *r)
{ return r!=0 && r->recording; }
static unsigned int median3(unsigned int a, unsigned int b, unsigned int c)
{
    if (a>b) { unsigned int t=a; a=b; b=t; }
    if (b>c) { b=c; }
    return a>b ? a : b;
}
static unsigned int band(const s3_barcode_signal_t *r)
{
    unsigned int n=(r->noise_q4+3U)/4U;
    return n<20U ? 20U : (n>100U ? 100U : n);
}
static s3_barcode_status_t finish(s3_barcode_signal_t *r, s3_barcode_status_t status)
{
    /* Preserve the learned paper/polarity and input filter. A black patch
     * cannot become the new paper merely because a capture timed out. */
    r->recording=0U; r->blocked=1U; r->head=0U; r->count=0U;
    r->elapsed_us=0U; r->white_us=0U; r->turn_us=0U;
    r->next_analysis_us=0U; r->observed_gap_us=0U;
    return status;
}

/* Local prominence finds alternating valleys/peaks. No stripe-count pruning
 * or expected-letter fit is allowed. Small preliminary marks remain present
 * and constrain quiet even when a later start marker is found. */
static unsigned int extrema(const s3_barcode_signal_t *r, uint16_t *indices)
{
    unsigned int n=1U, best=0U;
    bool rising=true;
    indices[0]=0U;
    for (unsigned int i=1U; i<r->count; i++) {
        unsigned int a=value(r,i), b=value(r,best);
        if (rising ? a>b : a<b) { best=i; }
        else if ((rising ? b-a : a-b)>=r->prominence) {
            if (n>=S3_SIGNAL_EXTREMA-1U) { return 0U; }
            indices[n++]=(uint16_t)best; best=i; rising=!rising;
        }
    }
    if (!rising) { indices[n++]=(uint16_t)best; }
    /* A shallow dip within an ink plateau is not a white gap. Merge only
     * when it remains above two thirds of the smaller adjacent peak's rise
     * above the flanking valleys. This local contrast rule is independent
     * of the number of bars and of every Code 39 pattern. */
    for (unsigned int j=1U; j+3U<n;) {
        unsigned int a=value(r,indices[j]), b=value(r,indices[j+2U]);
        unsigned int left=value(r,indices[j-1U]), right=value(r,indices[j+3U]);
        unsigned int base=left<right ? left : right, peak=a<b ? a : b;
        unsigned int dip=value(r,indices[j+1U]);
        if (dip>base && peak>base && 3U*(dip-base)>2U*(peak-base)) {
            if (b>a) { indices[j]=indices[j+2U]; }
            for (unsigned int k=j+1U; k+2U<n; k++) { indices[k]=indices[k+2U]; }
            n-=2U;
            if (j>1U) { j-=2U; }
        } else { j+=2U; }
    }
    return n;
}
static s3_barcode_status_t analyse(s3_barcode_signal_t *r,
    const s3_barcode_config_t *config, s3_barcode_result_t *out)
{
    uint16_t ext[S3_SIGNAL_EXTREMA];
    uint32_t edges[S3_SIGNAL_EXTREMA], at=0U;
    unsigned int n=extrema(r,ext), k=0U, edge_count=0U;
    bool crossed=false, found=false;
    s3_barcode_frame_t selected={0};
    r->frame_status=S3_FRAME_PATTERN;
    if (n==0U) { return S3_BARCODE_OVERFLOW; }
    if (n<3U) { return S3_BARCODE_NONE; }
    /* First rising / last falling crossing of each local halfway level.
     * All time arithmetic is bounded by capacity*31.5ms (<81 seconds). */
    for (unsigned int i=1U; i<r->count; i++) {
        uint32_t dt=step(r,i);
        if (k+1U<n) {
            unsigned int lo=value(r,ext[k]), hi=value(r,ext[k+1U]);
            unsigned int mid=(lo+hi)/2U, a=value(r,i-1U), b=value(r,i);
            bool up=hi>lo;
            bool core=(up ? hi-lo : lo-hi)<=4U*r->prominence;
            if (up ? (a<=mid && b>mid) : (a>mid && b<=mid)) {
                uint32_t edge=at+dt*(up ? mid-a : a-mid)/(up ? b-a : a-b);
                if (!crossed || (core ? up : !up)) { edges[edge_count]=edge; }
                crossed=true;
            }
            if (i==ext[k+1U]) {
                if (!crossed) { return S3_BARCODE_NONE; }
                edge_count++; k++; crossed=false;
            }
        }
        at+=dt;
    }
    /* Thirty edges identify fifteen consecutive bars. Search every black
     * start, retaining all neighbouring ink when measuring actual quiet. */
    for (unsigned int start=0U; start+30U<=edge_count; start+=2U) {
        uint32_t widths[29];
        s3_barcode_frame_t candidate;
        uint32_t leading=edges[start]-(start ? edges[start-1U] : 0U);
        uint32_t trailing=(start+30U<edge_count ? edges[start+30U] : at)-edges[start+29U];
        for (unsigned int i=0U; i<29U; i++) { widths[i]=edges[start+i+1U]-edges[start+i]; }
        s3_barcode_frame_status_t status=s3_barcode_frame_validate(widths,29U,
            leading,trailing,r->observed_gap_us,config,&candidate);
        if (status==S3_FRAME_OK || status==S3_FRAME_WAIT_QUIET) {
            r->frame_status=(uint16_t)status;
            if (status==S3_FRAME_OK) {
                /* More than one complete frame in one capture is ambiguous;
                 * do not silently choose a payload or the earliest match. */
                if (found) { r->frame_status=S3_FRAME_PATTERN; return S3_BARCODE_REJECTED; }
                selected=candidate; found=true;
            }
        } else if (!found && r->frame_status!=S3_FRAME_WAIT_QUIET && status>r->frame_status) {
            r->frame_status=(uint16_t)status;
        }
    }
    if (found) {
        r->frame_status=S3_FRAME_OK;
        out->letter=selected.letter; out->reversed=selected.reversed;
        return S3_BARCODE_DECODED;
    }
    return S3_BARCODE_NONE;
}

static s3_barcode_status_t filtered(s3_barcode_signal_t *r,
    const s3_barcode_config_t *config, uint32_t timestamp_us,
    unsigned int raw, s3_barcode_result_t *out)
{
    uint32_t dt=r->count || r->recording ? timestamp_us-r->sample_us : 0U;
    uint32_t total=dt+r->residual_us, units=total/500U;
    unsigned int paper=r->paper_q4/16U, noise_band=r->recording ? r->prominence : band(r);
    r->sample_us=timestamp_us; r->residual_us=(uint16_t)(total%500U);
    if (units>63U) { return finish(r,S3_BARCODE_TIMING_ERROR); }
    if (!r->recording) {
        if (r->blocked) {
            if (raw>paper+noise_band) { return S3_BARCODE_NONE; }
            r->blocked=0U;
        }
        if (r->count>=S3_SIGNAL_HISTORY) {
            r->head=(uint16_t)((r->head+1U)%S3_SIGNAL_CAPACITY); r->count--;
        }
        if (raw<=paper+noise_band) {
            r->paper_q4=(uint16_t)((int32_t)r->paper_q4+(int32_t)raw-(int32_t)paper);
            /* Old approach history measured against a different paper level
             * cannot define this frame's first edge or quiet margin. Drop it
             * conservatively and require fresh observed white; never flatten
             * a drift or manufacture the lost time. */
            if (r->count!=0U) {
                unsigned int oldest=value(r,0U), updated=r->paper_q4/16U;
                unsigned int drift=oldest>updated ? oldest-updated : updated-oldest;
                if (drift>noise_band) { r->head=0U; r->count=0U; }
            }
            append(r,raw,units); r->previous_filtered=(uint16_t)raw;
            return S3_BARCODE_NONE;
        }
        r->recording=1U; r->prominence=(uint16_t)noise_band;
        r->frame_status=S3_FRAME_PATTERN; r->elapsed_us=0U;
        r->next_analysis_us=0U; r->white_us=0U; r->turn_us=0U;
        r->best=r->previous_filtered; r->falling=0U;
    }
    if (r->count>=S3_SIGNAL_CAPACITY) { return finish(r,S3_BARCODE_OVERFLOW); }
    append(r,raw,units); r->elapsed_us+=dt; r->turn_us+=dt;
    if (r->elapsed_us>config->max_frame_us) { return finish(r,S3_BARCODE_REJECTED); }
    if (r->falling ? raw<r->best : raw>r->best) { r->best=(uint16_t)raw; }
    else if ((r->falling ? raw-r->best : r->best-raw)>=r->prominence) {
        r->best=(uint16_t)raw; r->falling=!r->falling; r->turn_us=0U;
    }
    if (raw>paper+noise_band) {
        r->white_us=0U;
        if (r->turn_us>config->max_run_us) { return finish(r,S3_BARCODE_REJECTED); }
        return S3_BARCODE_NONE;
    }
    r->white_us+=dt;
    if (r->white_us>=config->quiet_modules*config->min_narrow_us &&
        (r->elapsed_us>=r->next_analysis_us || r->frame_status==S3_FRAME_WAIT_QUIET)) {
        s3_barcode_status_t status=analyse(r,config,out);
        r->next_analysis_us=r->elapsed_us+50000U;
        if (status!=S3_BARCODE_NONE) { return finish(r,status); }
    }
    /* A permitted white stripe cannot exceed max_run. A complete candidate
     * may legitimately require a longer final quiet margin; keep waiting. */
    if (r->white_us>config->max_run_us && r->frame_status!=S3_FRAME_WAIT_QUIET) {
        return finish(r,S3_BARCODE_REJECTED);
    }
    return S3_BARCODE_NONE;
}

s3_barcode_status_t s3_barcode_signal_update(s3_barcode_signal_t *r,
    const s3_barcode_config_t *config, uint32_t timestamp_us, bool valid,
    uint16_t raw, bool black_high, s3_barcode_result_t *out)
{
    s3_barcode_status_t status;
    if (out!=0) { *out=(s3_barcode_result_t){0}; }
    if (r==0 || out==0 || !s3_barcode_frame_config_valid(config)) { return S3_BARCODE_NONE; }
    if (!valid || raw>4095U) {
        bool active=s3_barcode_signal_active(r);
        s3_barcode_signal_reset(r);
        return active ? S3_BARCODE_REJECTED : S3_BARCODE_NONE;
    }
    unsigned int oriented=black_high ? raw : 4095U-raw;
    if (r->seen && (timestamp_us-r->last_us==0U ||
        timestamp_us-r->last_us>config->max_gap_us || r->polarity!=(unsigned int)black_high)) {
        bool active=s3_barcode_signal_active(r);
        s3_barcode_signal_reset(r);
        return active ? S3_BARCODE_TIMING_ERROR : S3_BARCODE_NONE;
    }
    if (!r->seen) {
        r->seen=1U; r->polarity=black_high; r->last_us=timestamp_us;
        r->previous_us=timestamp_us; r->previous_raw=(uint16_t)oriented;
        r->older_raw=(uint16_t)oriented; r->paper_q4=(uint16_t)(oriented*16U);
        /* Begin conservatively until actual white samples establish noise;
         * retained history still contains faint marks preceding the trigger. */
        r->noise_q4=400U;
        r->previous_filtered=(uint16_t)oriented;
        return S3_BARCODE_NONE;
    }
    uint32_t gap=timestamp_us-r->last_us;
    if (gap>r->observed_gap_us) { r->observed_gap_us=gap; }
    r->last_us=timestamp_us;
    /* Estimate electrical/paper noise before median filtering; otherwise the
     * filter would make its own detection gate unjustifiably permissive. */
    if (!r->recording && !r->blocked && oriented<=r->paper_q4/16U+band(r) &&
        r->previous_raw<=r->paper_q4/16U+band(r)) {
        unsigned int delta=oriented>r->previous_raw ? oriented-r->previous_raw : r->previous_raw-oriented;
        r->noise_q4=(uint16_t)(r->noise_q4-r->noise_q4/16U+delta);
    }
    if (r->seen<2U) { r->seen=2U; }
    status=filtered(r,config,r->previous_us,median3(r->older_raw,r->previous_raw,oriented),out);
    r->older_raw=r->previous_raw; r->previous_raw=(uint16_t)oriented; r->previous_us=timestamp_us;
    if (status==S3_BARCODE_DECODED) { out->timestamp_us=timestamp_us; }
    return status;
}
