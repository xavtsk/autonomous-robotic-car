#include <assert.h>
#include <stdio.h>
#include "barcode_quiet.h"

int main(void)
{
    s3_barcode_quiet_t q={0};
    for (unsigned int period=5000U;period<=10000U;period+=5000U) {
        for (unsigned int duration=period;duration<=40000U;duration+=period) {
            s3_barcode_quiet_reset(&q);
            assert(s3_barcode_quiet_update(&q,200000U,0U,0U,5U,true)==S3_QUIET_LOW);
            for (unsigned int elapsed=0U;elapsed<duration;elapsed+=period) {
                assert(s3_barcode_quiet_update(&q,period,14U,0U,5U,true)==S3_QUIET_PENDING);
                assert(q.strict_us==0U && q.tolerant_us==200000U);
            }
            assert(s3_barcode_quiet_update(&q,period,5U,0U,5U,true)==S3_QUIET_LOW);
            assert(q.strict_us==period && q.tolerant_us==200000U+duration+period);
            assert(q.pulse_us==0U && !q.pulse_bad);
        }
    }
    /* A long or stronger excursion loses all earlier quiet. Later real paper
     * starts a new interval, rather than retroactively restoring that margin. */
    for (unsigned int strong=0U;strong<2U;strong++) {
        s3_barcode_quiet_reset(&q);
        (void)s3_barcode_quiet_update(&q,1000000U,0U,0U,5U,true);
        const unsigned int flags=s3_barcode_quiet_update(&q,strong?10000U:40500U,
            strong?15U:14U,0U,5U,true);
        assert(flags==(S3_QUIET_PENDING|S3_QUIET_BREAK));
        assert(q.strict_us==0U && q.tolerant_us==0U);
        (void)s3_barcode_quiet_update(&q,10000U,0U,0U,5U,true);
        assert(q.strict_us==10000U && q.tolerant_us==10000U);
    }
    /* Without permission there is no relaxed interpretation at all. */
    s3_barcode_quiet_reset(&q);
    (void)s3_barcode_quiet_update(&q,100000U,100U,100U,5U,false);
    assert(s3_barcode_quiet_update(&q,10000U,106U,100U,5U,false)==
           (S3_QUIET_PENDING|S3_QUIET_BREAK));
    assert(q.strict_us==0U && q.tolerant_us==0U);
    /* Wider existing bands retain their own strict semantics. */
    s3_barcode_quiet_reset(&q);
    (void)s3_barcode_quiet_update(&q,100000U,75U,0U,75U,true);
    assert(q.strict_us==100000U && q.tolerant_us==100000U);
    assert(s3_barcode_quiet_update(&q,10000U,76U,0U,75U,true)&S3_QUIET_BREAK);
    /* No leading interval is manufactured from a pulse before any paper. */
    s3_barcode_quiet_reset(&q);
    (void)s3_barcode_quiet_update(&q,10000U,10U,0U,5U,true);
    (void)s3_barcode_quiet_update(&q,10000U,0U,0U,5U,true);
    assert(q.tolerant_us==10000U);
    q.strict_us=q.tolerant_us=UINT32_MAX-5000U;
    (void)s3_barcode_quiet_update(&q,10000U,0U,0U,5U,true);
    assert(q.strict_us==UINT32_MAX && q.tolerant_us==UINT32_MAX);
    puts("PASS: strict/tolerant quiet clocks, pulse boundaries, cancellation and saturation");
    return 0;
}
