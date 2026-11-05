/* Raw streaming tests. Labels come only from the independent stimulus table;
 * no expected letter is passed to the reader. Quiet in these generated signals
 * is synthetic test evidence, never a claim about physical scanner performance. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "barcode_signal.h"

#define WAVE_MAX 4096U
static unsigned int checks;
static const uint16_t encodings[26]={
    0x109U,0x049U,0x148U,0x019U,0x118U,0x058U,0x00dU,0x10cU,0x04cU,0x01cU,
    0x103U,0x043U,0x142U,0x013U,0x112U,0x052U,0x007U,0x106U,0x046U,0x016U,
    0x181U,0x0c1U,0x1c0U,0x091U,0x190U,0x0d0U
};
typedef struct {
    s3_barcode_signal_t signal;
    s3_barcode_config_t config;
    uint32_t now, step_us, decoded_at;
    unsigned int decoded, rejected, timing, overflow;
    bool high;
    s3_barcode_result_t result;
} harness_t;

static void init(harness_t *h, bool high, uint32_t start)
{
    *h=(harness_t){0};
    h->config=s3_barcode_defaults();
    h->step_us=10000U;
    h->now=start;
    h->high=high;
}

static s3_barcode_status_t input(harness_t *h, uint16_t raw, bool valid,
    bool high, uint32_t timestamp)
{
    s3_barcode_result_t result={.letter='Q',.reversed=true,.timestamp_us=1U};
    s3_barcode_status_t status=s3_barcode_signal_update(&h->signal,&h->config,
        timestamp,valid,raw,high,&result);
    if (status==S3_BARCODE_DECODED) {
        h->decoded++;
        h->result=result;
        h->decoded_at=timestamp;
        assert(result.timestamp_us==timestamp);
        assert(result.letter>='A' && result.letter<='Z');
    } else {
        assert(result.letter==0 && !result.reversed && result.timestamp_us==0U);
        if (status==S3_BARCODE_REJECTED) { h->rejected++; }
        else if (status==S3_BARCODE_TIMING_ERROR) { h->timing++; }
        else if (status==S3_BARCODE_OVERFLOW) { h->overflow++; }
        else { assert(status==S3_BARCODE_NONE); }
    }
    return status;
}

static s3_barcode_status_t tick(harness_t *h, uint16_t oriented)
{
    assert(oriented<=4095U);
    h->now+=h->step_us;
    return input(h,h->high ? oriented : (uint16_t)(4095U-oriented),true,h->high,h->now);
}

static void level(harness_t *h, uint16_t oriented, uint32_t duration)
{
    assert(duration%h->step_us==0U);
    for (uint32_t t=0U; t<duration; t+=h->step_us) { (void)tick(h,oriented); }
}

static unsigned int append(uint16_t *wave, unsigned int at, uint16_t value,
    unsigned int samples)
{
    assert(at+samples<=WAVE_MAX);
    for (unsigned int i=0U; i<samples; i++) { wave[at+i]=value; }
    return at+samples;
}

static void blur(uint16_t *wave, unsigned int count, uint16_t paper)
{
    uint16_t tmp[WAVE_MAX];
    static const unsigned int weights[7]={1U,2U,3U,4U,3U,2U,1U};
    memcpy(tmp,wave,count*sizeof wave[0]);
    for (unsigned int i=0U; i<count; i++) {
        uint32_t sum=0U;
        for (unsigned int k=0U; k<7U; k++) {
            int index=(int)i+(int)k-3;
            uint16_t value=index<0 || (unsigned int)index>=count ? paper : tmp[index];
            sum+=(uint32_t)value*weights[k];
        }
        wave[i]=(uint16_t)((sum+8U)/16U);
    }
}

/* 80 ms narrow, 240 ms wide. Profile 0 is sharp, 1 is optically smoothed,
 * 2 additionally attenuates narrow bars to only 35 ADC above local paper.
 * For baseline150/170 those narrow peaks are below the old white=215 clamp. */
static unsigned int waveform(uint16_t *wave, char letter, bool reversed,
    uint16_t paper, unsigned int profile, unsigned int lead, unsigned int tail)
{
    unsigned int at=0U;
    at=append(wave,at,paper,lead);
    for (unsigned int c=0U; c<3U; c++) {
        const uint16_t pattern=c==1U ? encodings[(unsigned int)(letter-'A')] : 0x094U;
        for (unsigned int i=0U; i<9U; i++) {
            bool wide=(pattern & (1U<<(8U-i)))!=0U;
            uint16_t value=paper;
            if (i%2U==0U) { value=(uint16_t)(paper+(profile==2U && !wide ? 35U : 800U)); }
            at=append(wave,at,value,wide ? 24U : 8U);
        }
        if (c!=2U) { at=append(wave,at,paper,8U); }
    }
    at=append(wave,at,paper,tail);
    if (reversed) {
        /* Reverse only the barcode; approach and exit test margins stay put. */
        unsigned int span=at-lead-tail;
        for (unsigned int i=0U; i<span/2U; i++) {
            uint16_t value=wave[lead+i];
            wave[lead+i]=wave[lead+span-1U-i];
            wave[lead+span-1U-i]=value;
        }
    }
    if (profile!=0U) {
        blur(wave,at,paper);
    }
    return at;
}

static void play(harness_t *h, const uint16_t *wave, unsigned int count)
{
    for (unsigned int i=0U; i<count; i++) { (void)tick(h,wave[i]); }
}

static void expect_letter(const harness_t *h, char letter, bool reversed,
    const char *label)
{
    if (h->decoded!=1U || h->result.letter!=letter || h->result.reversed!=reversed ||
        h->timing!=0U || h->overflow!=0U) {
        fprintf(stderr,"%s: expected %c/%u, got decoded=%u %c/%u rejected=%u timing=%u overflow=%u state=%u count=%u frame=%u paper=%u prominence=%u\n",
            label,letter,(unsigned int)reversed,h->decoded,h->result.letter ? h->result.letter : '-',
            (unsigned int)h->result.reversed,h->rejected,h->timing,h->overflow,
            (unsigned int)h->signal.recording,(unsigned int)h->signal.count,
            (unsigned int)h->signal.frame_status,(unsigned int)(h->signal.paper_q4/16U),
            (unsigned int)h->signal.prominence);
        assert(false);
    }
    checks++;
}

static void test_alphabet_raw_profiles(void)
{
    const uint16_t papers[]={150U,170U,500U};
    for (unsigned int c=0U; c<26U; c++) {
        for (unsigned int direction=0U; direction<2U; direction++) {
            for (unsigned int polarity=0U; polarity<2U; polarity++) {
                for (unsigned int paper=0U; paper<3U; paper++) {
                    for (unsigned int profile=0U; profile<3U; profile++) {
                        uint16_t wave[WAVE_MAX];
                        harness_t h;
                        unsigned int n=waveform(wave,(char)('A'+c),direction!=0U,
                            papers[paper],profile,200U,200U);
                        init(&h,polarity!=0U,0U);
                        play(&h,wave,n);
                        expect_letter(&h,(char)('A'+c),direction!=0U,"raw alphabet/profile");
                        assert(h.rejected==0U);
                        /* The actual waveform ends at lead+376 samples. A
                         * complete result must await about ten real modules. */
                        assert(h.decoded_at>=(200U+376U+79U)*10000U);
                    }
                }
            }
        }
    }
}

static void test_quiet_and_noise_marks(void)
{
    uint16_t wave[WAVE_MAX];
    harness_t h;
    unsigned int n;
    for (unsigned int direction=0U; direction<2U; direction++) {
        for (unsigned int leading=0U; leading<=20U; leading+=20U) {
            n=waveform(wave,'A',direction!=0U,170U,0U,leading,200U);
            init(&h,true,0U); play(&h,wave,n);
            assert(h.decoded==0U);
            checks++;
        }
        n=waveform(wave,'A',direction!=0U,170U,0U,200U,20U);
        init(&h,true,0U); play(&h,wave,n);
        assert(h.decoded==0U && s3_barcode_signal_active(&h.signal));
        /* A new real mark interrupts the insufficient exit white. */
        level(&h,970U,40000U); level(&h,170U,2000000U);
        assert(h.decoded==0U);
        checks++;

        n=waveform(wave,'A',direction!=0U,170U,0U,200U,200U);
        /* A single isolated sample is removed by the documented median3
         * filter. It must not trigger a count-based waveform recovery. */
        wave[170]=2000U;
        init(&h,true,0U); play(&h,wave,n);
        expect_letter(&h,'A',direction!=0U,"isolated filtered spike");
        /* Two samples form a retained mark. Only300ms of white follows it;
         * the earlier two seconds cannot be credited across this real ink. */
        wave[171]=2000U;
        init(&h,true,0U); play(&h,wave,n);
        assert(h.decoded==0U);
        checks++;
    }
}

static void test_extra_and_incomplete_bars(void)
{
    uint16_t wave[WAVE_MAX];
    harness_t h;
    unsigned int n;
    for (unsigned int direction=0U; direction<2U; direction++) {
        n=waveform(wave,'A',direction!=0U,150U,0U,200U,200U);
        /* First marker is15 narrow units=120 samples; its inter-character
         * white is the next8. A retained extra bar breaks the 29-width frame. */
        wave[322]=950U; wave[323]=950U; wave[324]=950U;
        init(&h,true,0U); play(&h,wave,n);
        assert(h.decoded==0U);
        checks++;
        n=waveform(wave,'A',direction!=0U,150U,0U,200U,200U);
        /* Truncate half the closing marker and replace it with white. */
        for (unsigned int i=520U; i<n; i++) { wave[i]=150U; }
        init(&h,true,0U); play(&h,wave,n);
        assert(h.decoded==0U);
        checks++;
    }
}

static void start_active(harness_t *h)
{
    level(h,170U,1200000U);
    level(h,970U,80000U);
    assert(s3_barcode_signal_active(&h->signal));
}

static void test_invalid_samples_and_timing(void)
{
    harness_t h;
    s3_barcode_result_t out={.letter='Q'};
    s3_barcode_config_t cfg=s3_barcode_defaults();
    assert(s3_barcode_signal_update(NULL,&cfg,0U,true,170U,true,&out)==S3_BARCODE_NONE);
    assert(out.letter==0);
    s3_barcode_signal_reset(NULL);
    assert(!s3_barcode_signal_active(NULL));
    for (unsigned int failure=0U; failure<5U; failure++) {
        init(&h,true,0U); start_active(&h);
        if (failure==0U) {
            h.now+=10000U;
            assert(input(&h,4096U,true,true,h.now)==S3_BARCODE_REJECTED);
        } else if (failure==1U) {
            h.now+=10000U;
            assert(input(&h,170U,false,true,h.now)==S3_BARCODE_REJECTED);
        } else if (failure==2U) {
            assert(input(&h,170U,true,true,h.now)==S3_BARCODE_TIMING_ERROR);
        } else if (failure==3U) {
            h.now+=30001U;
            assert(input(&h,170U,true,true,h.now)==S3_BARCODE_TIMING_ERROR);
        } else {
            h.now+=10000U;
            assert(input(&h,170U,true,false,h.now)==S3_BARCODE_TIMING_ERROR);
        }
        assert(!s3_barcode_signal_active(&h.signal) && h.signal.count==0U && h.signal.seen==0U);
        assert(h.decoded==0U);
        checks++;
    }
    /* Losing evidence while a syntactically valid candidate waits for its
     * exit margin must also cancel that candidate, not merely skip a sample. */
    for (unsigned int failure=0U; failure<3U; failure++) {
        uint16_t wave[WAVE_MAX];
        unsigned int n=waveform(wave,'A',false,170U,0U,200U,50U);
        init(&h,true,0U); play(&h,wave,n);
        assert(h.decoded==0U && s3_barcode_signal_active(&h.signal));
        assert(h.signal.frame_status==S3_FRAME_WAIT_QUIET);
        h.now+=failure==2U ? 30001U : 10000U;
        assert(input(&h,failure==1U ? 4096U : 170U,failure!=0U,true,h.now)==
            (failure==2U ? S3_BARCODE_TIMING_ERROR : S3_BARCODE_REJECTED));
        level(&h,170U,2000000U);
        assert(h.decoded==0U && !s3_barcode_signal_active(&h.signal));
        checks++;
    }
    /* Ordinary unsigned timestamp wrap is valid and must not look like a
     * missing/negative sample interval. */
    {
        uint16_t wave[WAVE_MAX];
        unsigned int n=waveform(wave,'Z',true,170U,2U,200U,200U);
        init(&h,false,UINT32_MAX-3000000U);
        play(&h,wave,n);
        expect_letter(&h,'Z',true,"timestamp wrap");
    }
    checks+=2U;
}

static void test_reset_rearm_and_solid_line(void)
{
    uint16_t wave[WAVE_MAX];
    harness_t h;
    unsigned int n=waveform(wave,'A',false,170U,2U,200U,200U);
    init(&h,true,0U);
    for (unsigned int repetition=0U; repetition<3U; repetition++) {
        play(&h,wave,n);
        assert(h.decoded==repetition+1U && h.result.letter=='A');
        assert(h.timing==0U && h.overflow==0U && h.rejected==0U);
        checks++;
    }
    s3_barcode_signal_reset(&h.signal);
    assert(h.signal.seen==0U && h.signal.count==0U && !s3_barcode_signal_active(&h.signal));
    h.decoded=0U;
    play(&h,wave,n);
    expect_letter(&h,'A',false,"explicit reset");

    init(&h,true,0U);
    level(&h,170U,1200000U);
    level(&h,970U,2200000U);
    assert(h.decoded==0U && h.rejected==1U && h.signal.blocked);
    assert(h.signal.paper_q4/16U==170U);
    play(&h,wave,n);
    expect_letter(&h,'A',false,"solid line rearm");
}

static void test_adaptive_paper_and_history(void)
{
    uint16_t wave[WAVE_MAX];
    harness_t h;
    unsigned int n;
    /* More than one packed-buffer revolution of quiet approach exercises
     * head wrapping without fabricating lead credit from discarded samples. */
    init(&h,true,0U);
    level(&h,170U,30000000U);
    assert(h.signal.count==S3_SIGNAL_HISTORY);
    n=waveform(wave,'A',false,170U,2U,200U,200U);
    play(&h,wave,n);
    expect_letter(&h,'A',false,"history ring wrap");

    init(&h,true,0U);
    level(&h,150U,2000000U);
    for (unsigned int raw=151U; raw<=500U; raw++) { (void)tick(&h,(uint16_t)raw); }
    level(&h,500U,2000000U);
    assert(!s3_barcode_signal_active(&h.signal) && h.decoded==0U);
    assert(h.signal.paper_q4/16U>=499U && h.signal.paper_q4/16U<=500U);
    /* Old raw levels may be discarded as ambient paper changes. Only the
     * bounded fresh history, not a full ring, is required here. */
    assert(h.signal.count>0U && h.signal.count<=S3_SIGNAL_HISTORY);
    n=waveform(wave,'F',true,500U,2U,200U,200U);
    play(&h,wave,n);
    expect_letter(&h,'F',true,"adaptive raw paper");
}

static void test_adc_endpoints_and_fractional_timing(void)
{
    uint16_t wave[WAVE_MAX];
    harness_t h;
    for (unsigned int polarity=0U; polarity<2U; polarity++) {
        unsigned int n=waveform(wave,'A',false,150U,0U,200U,200U);
        for (unsigned int i=0U; i<n; i++) { wave[i]=wave[i]==150U ? 0U : 4095U; }
        init(&h,polarity!=0U,0U);
        play(&h,wave,n);
        expect_letter(&h,'A',false,"ADC endpoints");
        for (unsigned int direction=0U; direction<2U; direction++) {
            for (char letter='A'; letter<='Z'; letter=(char)(letter+25)) {
                n=waveform(wave,letter,direction!=0U,170U,2U,200U,200U);
                init(&h,polarity!=0U,0U);
                for (unsigned int i=0U; i<n; i++) {
                    h.step_us=i%2U==0U ? 7501U : 12499U;
                    (void)tick(&h,wave[i]);
                }
                expect_letter(&h,letter,direction!=0U,"fractional sample timing");
            }
        }
    }
}

static void test_buffer_bound(void)
{
    harness_t h;
    init(&h,true,0U);
    h.step_us=500U;
    h.config.max_run_us=2000000U;
    level(&h,170U,1000000U);
    assert(h.signal.count==S3_SIGNAL_HISTORY);
    level(&h,970U,1500000U);
    assert(h.overflow==1U && h.decoded==0U && h.timing==0U && h.rejected==0U);
    assert(h.signal.count==0U && !s3_barcode_signal_active(&h.signal) && h.signal.blocked);
    checks++;
}

static uint32_t random_state=UINT32_C(0x5349474e);
static unsigned int random_below(unsigned int bound)
{
    random_state=random_state*UINT32_C(1664525)+UINT32_C(1013904223);
    return (random_state>>8)%bound;
}

static void test_jitter_noise_and_internal_dips(void)
{
    unsigned int accepted=0U, refused=0U;
    for (unsigned int c=0U; c<26U; c++) {
        for (unsigned int direction=0U; direction<2U; direction++) {
            for (unsigned int repeat=0U; repeat<30U; repeat++) {
                uint16_t wave[WAVE_MAX];
                harness_t h;
                uint16_t paper=(uint16_t)(150U+random_below(351U));
                unsigned int at=append(wave,0U,paper,200U);
                for (unsigned int group=0U; group<3U; group++) {
                    uint16_t pattern=group==1U ? encodings[c] : 0x094U;
                    for (unsigned int i=0U; i<9U; i++) {
                        bool wide=(pattern & (1U<<(8U-i)))!=0U;
                        unsigned int samples=wide ? 22U+random_below(5U) : 7U+random_below(3U);
                        unsigned int amplitude=i%2U!=0U ? 0U :
                            (wide ? 650U+random_below(301U) : 35U+random_below(46U));
                        unsigned int start=at;
                        at=append(wave,at,(uint16_t)(paper+amplitude),samples);
                        if (wide && i%2U==0U && repeat%3U!=0U) {
                            /* Local dips bracket the production2/3 merge
                             * boundary:70% is shallow,60% remains a real
                             * subregion. Both must avoid an incorrect letter. */
                            unsigned int percent=repeat%3U==1U ? 70U : 60U;
                            for (unsigned int d=0U; d<4U; d++) {
                                wave[start+samples/2U+d]=(uint16_t)(paper+amplitude*percent/100U);
                            }
                        }
                    }
                    if (group!=2U) { at=append(wave,at,paper,7U+random_below(3U)); }
                }
                at=append(wave,at,paper,200U);
                if (direction!=0U) {
                    for (unsigned int i=0U; i<(at-400U)/2U; i++) {
                        uint16_t temp=wave[200U+i];
                        wave[200U+i]=wave[at-201U-i];
                        wave[at-201U-i]=temp;
                    }
                }
                blur(wave,at,paper);
                for (unsigned int i=0U; i<at; i++) {
                    wave[i]=(uint16_t)((int)wave[i]+(int)random_below(9U)-4);
                }
                init(&h,repeat%2U!=0U,0U);
                for (unsigned int i=0U; i<at; i++) {
                    h.step_us=i%2U==0U ? 9501U : 10499U;
                    (void)tick(&h,wave[i]);
                }
                if (h.decoded!=0U) {
                    if (h.decoded!=1U || h.result.letter!=(char)('A'+c) ||
                        h.result.reversed!=(direction!=0U)) {
                        fprintf(stderr,"STRESS wrong letter: source=%c direction=%u repeat=%u decoded=%u result=%c/%u paper=%u\n",
                            (char)('A'+c),direction,repeat,h.decoded,h.result.letter,
                            (unsigned int)h.result.reversed,(unsigned int)paper);
                        assert(false);
                    }
                    accepted++;
                } else { refused++; }
                assert(h.timing==0U && h.overflow==0U);
                checks++;
            }
        }
    }
    assert(accepted>780U && accepted+refused==1560U);
    printf("Raw stress: %u correct, %u safely refused, zero wrong letters\n",accepted,refused);
}

int main(void)
{
    test_alphabet_raw_profiles();
    test_quiet_and_noise_marks();
    test_extra_and_incomplete_bars();
    test_invalid_samples_and_timing();
    test_reset_rearm_and_solid_line();
    test_buffer_bound();
    test_adc_endpoints_and_fractional_timing();
    test_adaptive_paper_and_history();
    test_jitter_noise_and_internal_dips();
    printf("Barcode raw signal tests: %u scenarios passed\n",checks);
    return 0;
}
