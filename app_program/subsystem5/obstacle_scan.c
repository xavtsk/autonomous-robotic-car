#include "obstacle_scan.h"
_Static_assert(S5_SAMPLES_MAX>=17U,"Two-sided edge refinement needs 17 samples");
/* Seven coarse samples, then refine both occupied-to-far boundary intervals.
 * At most ten new interior angles are needed; no duplicated coarse samples. */
static const uint8_t coarse_angles[]={30U,45U,60U,90U,120U,135U,150U};
void s5_scan_begin_profile(s5_scan_t *s,uint32_t now,const s5_config_t *config)
{
    *s=(s5_scan_t){0}; s->angle=30U;
    s->config=config?*config:s5_defaults();s->started_us=now;
}
void s5_scan_begin(s5_scan_t *s,uint32_t now)
{ s5_scan_begin_profile(s,now,0); }
static bool boundaries(s5_scan_t *s)
{
    unsigned mm[7],a; bool valid;
    for(unsigned i=0;i<7U;i++) {
        if(!s5_sample_unpack(&s->sample[i],&a,&mm[i],&valid) || !valid) { return false; }
    }
    if(s5_profile_front_class(90U,mm[3],mm[3],&s->config)!=S5_FRONT_NEAR) { return false; }
    unsigned first=3U,last=3U;
    while(first && s5_profile_front_class(coarse_angles[first-1U],mm[first-1U],mm[3],&s->config)==S5_FRONT_NEAR) { first--; }
    while(last+1U<7U && s5_profile_front_class(coarse_angles[last+1U],mm[last+1U],mm[3],&s->config)==S5_FRONT_NEAR) { last++; }
    if(first==0U || last==6U ||
       s5_profile_front_class(coarse_angles[first-1U],mm[first-1U],mm[3],&s->config)!=S5_FRONT_FAR ||
       s5_profile_front_class(coarse_angles[last+1U],mm[last+1U],mm[3],&s->config)!=S5_FRONT_FAR) { return false; }
    for(a=coarse_angles[first-1U]+5U;a<coarse_angles[first];a+=5U) { s->fine_angle[s->fine_count++]=(uint8_t)a; }
    for(a=coarse_angles[last]+5U;a<coarse_angles[last+1U];a+=5U) { s->fine_angle[s->fine_count++]=(uint8_t)a; }
    s->fine=1U;s->fine_index=0U;
    if(!s->fine_count) { s->done=true; } else { s->angle=s->fine_angle[0]; }
    return true;
}
bool s5_peek_angle_valid(unsigned angle)
{ return angle%5U==0U && ((angle>=30U && angle<=60U) || (angle>=120U && angle<=150U)); }
bool s5_scan_begin_at(s5_scan_t *s,uint32_t now,unsigned angle)
{
    if(!s || !s5_peek_angle_valid(angle)) { return false; }
    s5_scan_begin(s,now); s->angle=(uint8_t)angle; s->single=1U; return true;
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
    s->readings[s->reading_count++]=(uint16_t)mm;
    if(s->reading_count<3U) { return; }
    /* Independent per-angle median rejects one short/long return without
     * widening the profile or turning a missing echo into free space. */
    unsigned a=s->readings[0],b=s->readings[1],c=s->readings[2],swap;
    if(a>b) { swap=a;a=b;b=swap; }
    if(b>c) { swap=b;b=c;c=swap; }
    if(a>b) { b=a; }
    mm=b;s->reading_count=0U;
    if(s->count>=S5_SAMPLES_MAX ||
       !s5_sample_pack(s->angle,mm,true,&s->sample[s->count])) {
        s->failed=s->done=true; return;
    }
    s->count++;
    if(s->single) { s->done=true; return; }
    if(!s->fine) {
        if(++s->index<sizeof(coarse_angles)/sizeof(coarse_angles[0])) { s->angle=coarse_angles[s->index]; }
        else if(!boundaries(s)) { s->failed=s->done=true; }
    } else if(++s->fine_index<s->fine_count) { s->angle=s->fine_angle[s->fine_index]; }
    else { s->done=true; }
}
