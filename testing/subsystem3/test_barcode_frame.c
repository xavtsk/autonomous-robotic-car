/* Independent geometry/quiet tests for an already extracted duration frame.
 * Synthetic white supplied here is test input, not observed hardware proof. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "barcode_frame.h"

static unsigned int checks;
static const uint16_t letters[26]={
    0x109U,0x049U,0x148U,0x019U,0x118U,0x058U,0x00dU,0x10cU,0x04cU,0x01cU,
    0x103U,0x043U,0x142U,0x013U,0x112U,0x052U,0x007U,0x106U,0x046U,0x016U,
    0x181U,0x0c1U,0x1c0U,0x091U,0x190U,0x0d0U
};

static s3_barcode_config_t config(void)
{
    return (s3_barcode_config_t){30000U,30000U,1000000U,15000000U,10U};
}

static void symbol(uint32_t *widths, uint16_t pattern, uint32_t narrow,
    uint32_t wide, int32_t bias)
{
    for (unsigned int i=0U; i<9U; i++) {
        int64_t width=(pattern & (1U<<(8U-i))) ? wide : narrow;
        width+=(i%2U)!=0U ? bias : -(int64_t)bias;
        assert(width>0 && width<=UINT32_MAX);
        widths[i]=(uint32_t)width;
    }
}

static void make_frame(uint32_t *widths, char letter, uint32_t narrow,
    uint32_t wide, int32_t bias)
{
    symbol(widths,0x094U,narrow,wide,bias);
    symbol(widths+10U,letters[(unsigned int)(letter-'A')],narrow,wide,bias);
    symbol(widths+20U,0x094U,narrow,wide,bias);
    widths[9]=widths[19]=(uint32_t)((int64_t)narrow+bias);
}

static void reverse(uint32_t *widths)
{
    for (unsigned int i=0U; i<14U; i++) {
        const uint32_t value=widths[i];
        widths[i]=widths[28U-i];
        widths[28U-i]=value;
    }
}

static s3_barcode_frame_status_t validate(const uint32_t *widths,
    uint32_t lead, uint32_t tail, uint32_t gap, s3_barcode_frame_t *out)
{
    const s3_barcode_config_t cfg=config();
    return s3_barcode_frame_validate(widths,29U,lead,tail,gap,&cfg,out);
}

static void expect_refusal(const uint32_t *widths, unsigned int count,
    uint32_t lead, uint32_t tail, uint32_t gap, const s3_barcode_config_t *cfg,
    s3_barcode_frame_status_t expected)
{
    s3_barcode_frame_t out={.letter='Q',.reversed=true,.module_us={1U,2U,3U},
        .required_leading_us=1U,.required_trailing_us=2U,.min_separation_us=3U};
    assert(s3_barcode_frame_validate(widths,count,lead,tail,gap,cfg,&out)==expected);
    assert(out.letter==0 && !out.reversed && out.module_us[0]==0U &&
        out.module_us[1]==0U && out.module_us[2]==0U &&
        out.required_leading_us==0U && out.required_trailing_us==0U &&
        out.min_separation_us==0U);
    checks++;
}

static void test_all_letters(void)
{
    const uint32_t scales[]={30000U,80000U,240000U};
    const unsigned int ratios[]={200U,250U,300U};
    const int32_t biases[]={-25,0,25,65};
    for (unsigned int l=0U; l<26U; l++) {
        for (unsigned int s=0U; s<3U; s++) {
            for (unsigned int r=0U; r<3U; r++) {
                for (unsigned int b=0U; b<4U; b++) {
                    for (unsigned int dir=0U; dir<2U; dir++) {
                        uint32_t widths[29];
                        s3_barcode_frame_t out;
                        const uint32_t narrow=scales[s];
                        const int32_t bias=(int32_t)narrow*biases[b]/100;
                        make_frame(widths,(char)('A'+l),narrow,
                            narrow*ratios[r]/100U,bias);
                        if (dir!=0U) { reverse(widths); }
                        assert(validate(widths,10000000U,10000000U,10000U,&out)==S3_FRAME_OK);
                        assert(out.letter==(char)('A'+l) && out.reversed==(dir!=0U));
                        assert(out.module_us[0]==narrow && out.module_us[1]==narrow &&
                            out.module_us[2]==narrow);
                        assert(out.required_leading_us==narrow*10U+10000U+
                            (bias>0 ? ((uint32_t)bias+1U)/2U : 0U));
                        assert(out.required_trailing_us==out.required_leading_us);
                        checks++;
                    }
                }
            }
        }
    }
}

static void test_quiet_and_acquisition_order(void)
{
    uint32_t widths[29];
    s3_barcode_frame_t out;
    s3_barcode_config_t cfg=config();
    make_frame(widths,'A',80000U,200000U,0);
    symbol(widths+20U,0x094U,100000U,250000U,40000);
    /* The gap between 80 ms and 140 ms white narrow classes is 110 ms. */
    widths[19]=110000U;
    for (unsigned int dir=0U; dir<2U; dir++) {
        const uint32_t lead=dir!=0U ? 1030000U : 810000U;
        const uint32_t tail=dir!=0U ? 810000U : 1030000U;
        assert(validate(widths,lead,tail,10000U,&out)==S3_FRAME_OK);
        assert(out.reversed==(dir!=0U));
        assert(out.required_leading_us==lead && out.required_trailing_us==tail);
        assert(out.module_us[0]==(dir!=0U ? 100000U : 80000U));
        assert(out.module_us[2]==(dir!=0U ? 80000U : 100000U));
        assert(validate(widths,lead,tail-1U,10000U,&out)==S3_FRAME_WAIT_QUIET);
        assert(out.letter=='A' && out.required_trailing_us==tail);
        assert(validate(widths,lead,0U,10000U,&out)==S3_FRAME_WAIT_QUIET);
        expect_refusal(widths,29U,lead-1U,tail,10000U,&cfg,S3_FRAME_LEADING_QUIET);
        checks+=3U;
        reverse(widths);
    }
    /* A caller cannot present omitted approach white as zero and obtain a
     * candidate that merely waits for trailing white. */
    expect_refusal(widths,29U,0U,UINT32_MAX,10000U,&cfg,S3_FRAME_LEADING_QUIET);
}

static void test_gap_boundaries(void)
{
    uint32_t widths[29];
    s3_barcode_frame_t out;
    s3_barcode_config_t cfg=config();
    for (unsigned int gap=9U; gap<=19U; gap+=10U) {
        for (unsigned int dir=0U; dir<2U; dir++) {
            const uint32_t values[]={69999U,70000U,280000U,280001U};
            for (unsigned int i=0U; i<4U; i++) {
                /* White narrow = 140 ms even though black narrow = 20 ms. */
                make_frame(widths,'A',80000U,240000U,60000);
                widths[gap]=values[i];
                if (dir!=0U) { reverse(widths); }
                if (i==0U || i==3U) {
                    expect_refusal(widths,29U,2000000U,2000000U,10000U,&cfg,S3_FRAME_GEOMETRY);
                } else {
                    assert(validate(widths,2000000U,2000000U,10000U,&out)==S3_FRAME_OK);
                    checks++;
                }
            }
        }
    }
}

static void test_geometry_boundaries(void)
{
    uint32_t widths[29];
    s3_barcode_frame_t out;
    s3_barcode_config_t cfg=config();
    /* With narrow=100 ms and gap=10 ms, interval overlap permits wide
     * endpoints 1.8*(100-10)-10=152 ms and 3.5*(100+10)+10=395 ms.
     * One microsecond outside each endpoint must still refuse. */
    const uint32_t wide[]={151999U,152000U,395000U,395001U};
    for (unsigned int i=0U; i<4U; i++) {
        make_frame(widths,'A',100000U,wide[i],0);
        if (i==0U || i==3U) {
            expect_refusal(widths,29U,2000000U,2000000U,10000U,&cfg,S3_FRAME_GEOMETRY);
        } else {
            assert(validate(widths,2000000U,2000000U,10000U,&out)==S3_FRAME_OK);
            checks++;
        }
    }
    /* A smaller observed interval gives a correspondingly smaller allowance;
     * the boundary must not become a fixed relaxed wide/narrow ratio. */
    {
        const uint32_t fine[]={179997U,179998U,350004U,350005U};
        for (unsigned int i=0U; i<4U; i++) {
            make_frame(widths,'A',100000U,fine[i],0);
            if (i==0U || i==3U) {
                expect_refusal(widths,29U,2000000U,2000000U,1U,&cfg,S3_FRAME_GEOMETRY);
            } else {
                assert(validate(widths,2000000U,2000000U,1U,&out)==S3_FRAME_OK);
                checks++;
            }
        }
    }
    make_frame(widths,'A',29999U,89997U,0);
    expect_refusal(widths,29U,1000000U,1000000U,9999U,&cfg,S3_FRAME_GEOMETRY);
    make_frame(widths,'A',30000U,90000U,0);
    assert(validate(widths,1000000U,1000000U,10000U,&out)==S3_FRAME_OK);
    expect_refusal(widths,29U,1000000U,1000000U,10001U,&cfg,S3_FRAME_GEOMETRY);
    make_frame(widths,'A',100000U,300000U,0);
    widths[10]=1000001U;
    expect_refusal(widths,29U,2000000U,2000000U,10000U,&cfg,S3_FRAME_GEOMETRY);
    make_frame(widths,'A',100000U,300000U,0);
    /* Total span 4.7 s, trailing quiet requirement 1.01 s. */
    cfg.max_frame_us=5710000U;
    assert(s3_barcode_frame_validate(widths,29U,1010000U,1010000U,10000U,&cfg,&out)==S3_FRAME_OK);
    cfg.max_frame_us--;
    expect_refusal(widths,29U,1010000U,1010000U,10000U,&cfg,S3_FRAME_GEOMETRY);
    checks+=2U;
}

static void test_invalid_input(void)
{
    uint32_t widths[29];
    s3_barcode_frame_t out;
    s3_barcode_config_t cfg=config(), broken;
    make_frame(widths,'A',100000U,300000U,0);
    expect_refusal(NULL,29U,2000000U,2000000U,10000U,&cfg,S3_FRAME_INVALID_INPUT);
    expect_refusal(widths,28U,2000000U,2000000U,10000U,&cfg,S3_FRAME_INVALID_INPUT);
    expect_refusal(widths,30U,2000000U,2000000U,10000U,&cfg,S3_FRAME_INVALID_INPUT);
    expect_refusal(widths,29U,2000000U,2000000U,0U,&cfg,S3_FRAME_INVALID_INPUT);
    expect_refusal(widths,29U,2000000U,2000000U,30001U,&cfg,S3_FRAME_TIMING);
    expect_refusal(widths,29U,2000000U,2000000U,10000U,NULL,S3_FRAME_INVALID_INPUT);
    assert(s3_barcode_frame_validate(widths,29U,2000000U,2000000U,10000U,&cfg,NULL)==S3_FRAME_INVALID_INPUT);
    assert(!s3_barcode_frame_config_valid(NULL));
    broken=cfg; broken.quiet_modules=9U;
    expect_refusal(widths,29U,2000000U,2000000U,10000U,&broken,S3_FRAME_INVALID_INPUT);
    broken=cfg; broken.quiet_modules=51U;
    assert(!s3_barcode_frame_config_valid(&broken));
    broken=cfg; broken.max_frame_us=UINT32_MAX;
    assert(!s3_barcode_frame_config_valid(&broken));
    broken=cfg; broken.max_run_us=UINT32_MAX;
    assert(!s3_barcode_frame_config_valid(&broken));
    broken=cfg; broken.max_gap_us=0U;
    assert(!s3_barcode_frame_config_valid(&broken));
    for (unsigned int i=0U; i<29U; i++) {
        uint32_t saved=widths[i];
        widths[i]=0U;
        expect_refusal(widths,29U,2000000U,2000000U,10000U,&cfg,S3_FRAME_INVALID_INPUT);
        widths[i]=saved;
    }
    symbol(widths,letters[1],100000U,300000U,0);
    expect_refusal(widths,29U,2000000U,2000000U,10000U,&cfg,S3_FRAME_PATTERN);
    for (unsigned int i=0U; i<29U; i++) { widths[i]=UINT32_MAX; }
    expect_refusal(widths,29U,UINT32_MAX,UINT32_MAX,10000U,&cfg,S3_FRAME_GEOMETRY);
    /* Valid large configuration must not wrap geometry arithmetic. */
    broken=(s3_barcode_config_t){30000U,30000U,40000000U,2000000000U,50U};
    assert(s3_barcode_frame_config_valid(&broken));
    make_frame(widths,'Z',10000000U,30000000U,6000000);
    assert(s3_barcode_frame_validate(widths,29U,UINT32_MAX,UINT32_MAX,30000U,&broken,&out)==S3_FRAME_OK);
    assert(out.required_leading_us==503030000U);
    checks+=8U;
}

static void test_measured_patterns(void)
{
    /* Same uncorrected vectors/provenance as test_barcode_pattern.c. The two
     * seconds of quiet supplied below are synthetic, deliberately explicit.
     * They do not establish real quiet in the original A/Z/F recordings. */
    static const struct { char letter; bool reversed; uint32_t gap, widths[29]; } cases[]={
        {'A',false,10500U,{22625,257900,35100,136956,120649,172201,118144,135112,28256,153539,
            110570,139173,24763,112387,29416,250949,28585,133688,124191,132643,
            30050,249290,25938,146300,116974,152059,119982,133685,20187}},
        {'A',false,12500U,{24482,255698,32125,127549,123218,169535,125737,140077,26532,138930,
            124320,140125,26180,107945,27700,260300,29071,136381,132743,124555,
            23150,264037,23370,140970,120870,158452,116654,132837,20955}},
        {'A',false,12500U,{30696,260443,24125,128477,123648,166638,133034,134058,18870,138753,
            136759,132688,31575,109048,26377,260776,33274,136563,133755,126032,
            26425,252056,33461,147809,112499,146334,122216,135250,33150}},
        {'Z',true,21500U,{139298,161095,448070,156897,456819,156524,149142,462675,139210,155399,
            143123,152530,136529,153416,437795,166535,447770,466061,145382,159691,
            137345,165699,441801,152312,444672,161210,134712,470971,143006}},
        {'F',true,11500U,{151118,166339,434873,185763,379451,248465,86011,501133,111996,174988,
            128249,172705,129397,465884,438416,149347,468441,174706,145512,150462,
            137187,150406,382426,202864,392730,178671,112389,514985,139727}},
        {'Z',false,19500U,{87705,474014,101486,197539,351883,273971,324867,202396,86434,189538,
            96079,487610,361407,239018,334774,241324,66068,221502,65685,222077,
            94022,485085,99720,201158,369894,243334,349179,222248,88400}},
        /* Raw median3/local-extrema signal measurements, not the legacy
         * calibrated extraction above. Midpoint dt reconstruction; observed
         * gap12.5ms. Closing-star mean ratios1.7623/1.7875 overlap1.8 after
         * sampling uncertainty. Supplied quiet remains SYNTHETIC here.
         * 20261105-185701-small-a-decoded.txt, frozen prominence77ADC. */
        {'A',false,12500U,{37718,241390,44245,123088,125799,155228,136089,121316,36771,134626,
            127655,117219,46285,91174,53154,225769,54910,122358,132846,121629,
            50134,223098,58309,128147,114922,158130,121055,135539,50389}},
        /* 20261105-190459-small-a-width-rejected.txt, prominence63ADC. */
        {'A',false,12500U,{49827,239440,40431,121760,125861,164085,135404,115499,49456,124486,
            139128,120940,47091,96509,45522,242865,51883,122683,136341,111331,
            63464,223590,47494,136238,116057,144577,124020,119706,65229}}
    };
    const s3_barcode_config_t cfg=config();
    for (unsigned int i=0U; i<sizeof cases/sizeof cases[0]; i++) {
        uint32_t widths[29];
        s3_barcode_frame_t out;
        memcpy(widths,cases[i].widths,sizeof widths);
        for (unsigned int dir=0U; dir<2U; dir++) {
            assert(validate(widths,2000000U,2000000U,cases[i].gap,&out)==S3_FRAME_OK);
            assert(out.letter==cases[i].letter && out.reversed==(cases[i].reversed!=(dir!=0U)));
            assert(validate(widths,2000000U,0U,cases[i].gap,&out)==S3_FRAME_WAIT_QUIET);
            expect_refusal(widths,29U,0U,2000000U,cases[i].gap,&cfg,S3_FRAME_LEADING_QUIET);
            checks+=2U;
            reverse(widths);
        }
    }
}

int main(void)
{
    test_all_letters();
    test_quiet_and_acquisition_order();
    test_gap_boundaries();
    test_geometry_boundaries();
    test_invalid_input();
    test_measured_patterns();
    printf("Barcode frame validation: %u checks passed (measured-vector quiet is synthetic)\n",checks);
    return 0;
}
