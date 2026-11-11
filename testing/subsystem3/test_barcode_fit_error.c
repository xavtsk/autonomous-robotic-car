/* Exact-equivalence regression for the RP2040 fit-score optimization.
 * Include the implementation to exercise its private arithmetic helper;
 * compile this file alone, without a second barcode_decoder.c object. */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem3/barcode_decoder.c"

static uint32_t random_state=0x8421937U;
static uint32_t random_word(void)
{
    random_state=random_state*1664525U+1013904223U;
    return random_state;
}

/* The previous production calculation, retained as an independent oracle. */
static uint32_t original_error(const uint32_t adjusted[9],uint16_t star,
    uint32_t narrow,uint32_t wide)
{
    uint32_t score=0U;
    for (unsigned int i=0U;i<9U;i++) {
        const uint32_t reference=(star & (1U<<(8U-i))) ? wide : narrow;
        const uint32_t delta=(adjusted[i]>reference) ?
            adjusted[i]-reference : reference-adjusted[i];
        const uint32_t error=(uint32_t)((uint64_t)delta*1000000U/reference);
        if (error>score) { score=error; }
    }
    return score;
}

static void check(const uint32_t adjusted[9],uint16_t star,
    uint32_t narrow,uint32_t wide)
{
    assert(narrow!=0U && wide!=0U);
    assert(rec_fit_error(adjusted,star,narrow,wide)==
           original_error(adjusted,star,narrow,wide));
    assert(rec_fit_error(adjusted,star,narrow,wide)<=350000U);
}

static void exact_boundaries(void)
{
    /* Includes minimum operational widths, non-divisible references,
     * and large widths whose 35% deviations still fit the legal max_run bound. */
    const uint32_t narrow[]={30000U,30001U,100003U,50000000U};
    unsigned int checks=0U;
    for (unsigned int reverse=0U;reverse<2U;reverse++) {
        const uint16_t star=reverse ? 0x052U : 0x094U;
        for (unsigned int n=0U;n<sizeof(narrow)/sizeof(narrow[0]);n++) {
            const uint32_t wide=narrow[n]*3U;
            for (unsigned int winner=0U;winner<9U;winner++) {
                for (unsigned int side=0U;side<2U;side++) {
                    uint32_t adjusted[9];
                    for (unsigned int i=0U;i<9U;i++) {
                        adjusted[i]=(star & (1U<<(8U-i))) ? wide : narrow[n];
                    }
                    /* Exact zero, tiny deviations, and the strict 35%
                     * endpoint cover ties and integer floor boundaries. */
                    check(adjusted,star,narrow[n],wide);checks++;
                    if (side) { adjusted[winner]++; }
                    else { adjusted[winner]--; }
                    check(adjusted,star,narrow[n],wide);checks++;
                    const uint32_t reference=(star & (1U<<(8U-winner))) ? wide : narrow[n];
                    const uint32_t delta=(uint32_t)((uint64_t)reference*35U/100U);
                    adjusted[winner]=side ? reference+delta : reference-delta;
                    check(adjusted,star,narrow[n],wide);checks++;
                }
            }
        }
    }
    assert(checks==432U);
}

static unsigned int validated_random_fits(void)
{
    unsigned int checked=0U;
    for (unsigned int domain=0U;domain<2U;domain++) {
        s3_barcode_config_t config=s3_barcode_defaults();
        if (domain) {
            config.max_run_us=0x7FFFFFFFU/config.quiet_modules;
            config.max_frame_us=0x7FFFFFFFU;
        }
        assert(config_valid(&config));
        for (unsigned int trial=0U;trial<100000U;trial++) {
            const uint16_t star=(trial&1U) ? 0x052U : 0x094U;
            const uint32_t base=config.min_narrow_us+
                random_word()%(config.max_run_us/5U-config.min_narrow_us+1U);
            const uint32_t broad=(uint32_t)((uint64_t)base*(200U+random_word()%151U)/100U);
            uint32_t adjusted[9],narrow=0U,wide=0U;
            uint16_t pattern=0U;
            for (unsigned int i=0U;i<9U;i++) {
                const uint32_t reference=(star & (1U<<(8U-i))) ? broad : base;
                adjusted[i]=(uint32_t)((uint64_t)reference*(65U+random_word()%71U)/100U);
                if (star & (1U<<(8U-i))) { wide+=adjusted[i]; }
            }
            if (!character(adjusted,10000U,&config,&pattern,&narrow) || pattern!=star) { continue; }
            wide/=3U;
            check(adjusted,star,narrow,wide);
            checked++;
        }
    }
    assert(checked>50000U);
    return checked;
}

int main(void)
{
    exact_boundaries();
    const unsigned int random_checks=validated_random_fits();
    printf("PASS: exact fit-score equivalence, 432 boundary cases and %u validated random fits across default/maximal duration limits\n",random_checks);
    return 0;
}
