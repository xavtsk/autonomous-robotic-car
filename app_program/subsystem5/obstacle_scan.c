#include "obstacle_scan.h"
void s5_scan_begin(s5_scan_t *s,uint32_t now)
{
    *s=(s5_scan_t){0}; s->angle=30U; s->nearest=90U;
    s->nearest_mm=S5_RANGE_MAX_MM; s->started_us=now;
}
void s5_scan_accept(s5_scan_t *s,uint32_t now,bool valid,unsigned mm)
{
    if(!s || s->done) { return; }
    if((uint32_t)(now-s->started_us)>=S5_SCAN_TIMEOUT_US) {
        s->failed=s->done=true; return;
    }
    if(!valid || mm<S5_RANGE_MIN_MM || mm>S5_RANGE_MAX_MM) {
        if(++s->attempts<3U) { return; }
        s->failed=s->done=true; return;
    }
    s->attempts=0;
    if(s->count>=S5_SAMPLES_MAX ||
       !s5_sample_pack(s->angle,mm,true,&s->sample[s->count])) {
        s->failed=s->done=true; return;
    }
    s->count++;
    if(!s->fine && mm<s->nearest_mm) { s->nearest_mm=(uint16_t)mm; s->nearest=s->angle; }
    if(!s->fine) {
        if(s->angle<150U) { s->angle+=30U; }
        else {
            s->fine=true; s->angle=s->nearest>45U?s->nearest-15U:30U;
            s->fine_end=s->nearest<135U?s->nearest+15U:150U;
        }
    } else if(s->angle<s->fine_end) { s->angle+=5U; }
    else { s->done=true; }
}
