#include "obstacle_profile.h"
/* sin/cos of angles -75..75 in 5-degree steps, scaled by 1000.
 * The extra endpoints cover uncertainty around the 30/150-degree scan limits.
 * x positive LEFT, y forward, servo 90 straight, 150 left. */
static const int16_t sx[31]={-966,-940,-906,-866,-819,-766,-707,-643,-574,-500,-423,-342,-259,-174,-87,0,87,174,259,342,423,500,574,643,707,766,819,866,906,940,966};
static const int16_t cy[31]={259,342,423,500,574,643,707,766,819,866,906,940,966,985,996,1000,996,985,966,940,906,866,819,766,707,643,574,500,423,342,259};
static int floor_milli(int n)
{ return n>=0?n/1000:-((-n+999)/1000); }
static int ceil_milli(int n)
{ return n>=0?(n+999)/1000:-((-n)/1000); }
bool s5_sample_pack(unsigned angle,unsigned mm,bool valid,s5_sample_t *out)
{
    uint32_t word;
    if(!out || angle<30U || angle>150U || (angle-30U)%5U ||
       (valid && (mm<S5_RANGE_MIN_MM || mm>S5_RANGE_MAX_MM))) { return false; }
    word=(valid?mm:0U)|(((angle-30U)/5U)<<12)|((uint32_t)valid<<17);
    for(unsigned i=0;i<3;i++) { out->bytes[i]=(uint8_t)(word>>(8U*i)); }
    return true;
}
bool s5_sample_unpack(const s5_sample_t *s,unsigned *angle,unsigned *mm,bool *valid)
{
    uint32_t w;
    if(!s || !angle || !mm || !valid) { return false; }
    w=(uint32_t)s->bytes[0]|((uint32_t)s->bytes[1]<<8)|((uint32_t)s->bytes[2]<<16);
    if((w>>18) || ((w>>12)&31U)>24U) { return false; }
    *angle=30U+5U*((w>>12)&31U); *mm=w&4095U; *valid=((w>>17)&1U)!=0;
    return *valid?(*mm>=S5_RANGE_MIN_MM && *mm<=S5_RANGE_MAX_MM):*mm==0U;
}
s5_front_class_t s5_profile_front_class(unsigned angle,unsigned mm,unsigned centre,
                                        const s5_config_t *cfg)
{
    if(!cfg || angle<30U || angle>150U || angle%5U ||
       mm<S5_RANGE_MIN_MM || mm>S5_RANGE_MAX_MM ||
       centre<S5_RANGE_MIN_MM || centre>=cfg->profile_mm) { return S5_FRONT_BLOCKED; }
    unsigned index=(angle-15U)/5U,low=index-3U,high=index+3U;
    unsigned near_cos=cy[low]<cy[high]?(unsigned)cy[low]:(unsigned)cy[high];
    unsigned far_cos=(low<=15U && high>=15U)?1000U:
                     (cy[low]>cy[high]?(unsigned)cy[low]:(unsigned)cy[high]);
    unsigned centre_low=centre*966U/1000U;
    unsigned slab_low=centre_low>cfg->margin_mm?centre_low-cfg->margin_mm:0U;
    unsigned slab_high=centre+cfg->margin_mm;
    unsigned near_y=mm*near_cos/1000U;
    unsigned far_y=(mm*far_cos+999U)/1000U;
    if(near_y>slab_high+1U) { return S5_FRONT_FAR; }
    if(mm<cfg->profile_mm && far_y+1U>=slab_low) { return S5_FRONT_NEAR; }
    return S5_FRONT_BLOCKED;
}
bool s5_profile_build(const s5_sample_t *samples,unsigned count,
                      const s5_config_t *cfg,s5_profile_t *out)
{
    s5_profile_t p={0}; bool left=false,right=false;
    uint16_t ranges[25]={0}; unsigned order[25],used=0U,centre_at=0U;
    if(!out) { return false; } *out=p;
    if(!samples || !cfg || count<5U || count>S5_SAMPLES_MAX) { return false; }
    for(unsigned i=0;i<count;i++) {
        unsigned angle,mm; bool valid;
        if(!s5_sample_unpack(&samples[i],&angle,&mm,&valid) || !valid) { return false; }
        unsigned index=(angle-30U)/5U;
        if(!ranges[index] || mm<ranges[index]) { ranges[index]=(uint16_t)mm; }
    }
    if(!ranges[12] || ranges[12]>=cfg->profile_mm) { return false; }
    for(unsigned i=0;i<25U;i++) {
        if(!ranges[i]) { continue; }
        if(i==12U) { centre_at=used; }
        order[used++]=i;
    }
    unsigned first=centre_at,last=centre_at;
    while(first && s5_profile_front_class(30U+5U*order[first-1U],ranges[order[first-1U]],ranges[12],cfg)==S5_FRONT_NEAR) { first--; }
    while(last+1U<used && s5_profile_front_class(30U+5U*order[last+1U],ranges[order[last+1U]],ranges[12],cfg)==S5_FRONT_NEAR) { last++; }
    /* Both edges need an actual farther return beyond the connected near
     * component. Reaching a scan limit or a nearer foreign object is unknown. */
    if(first==0U || last+1U==used ||
       s5_profile_front_class(30U+5U*order[first-1U],ranges[order[first-1U]],ranges[12],cfg)!=S5_FRONT_FAR ||
       s5_profile_front_class(30U+5U*order[last+1U],ranges[order[last+1U]],ranges[12],cfg)!=S5_FRONT_FAR) { return false; }
    p.closest_mm=p.left_clear_mm=p.right_clear_mm=S5_RANGE_MAX_MM;
    p.front_mm=S5_RANGE_MAX_MM;
    p.left_edge_mm=-4000; p.right_edge_mm=4000;
    for(unsigned i=0;i<used;i++) {
        unsigned angle=30U+5U*order[i],mm=ranges[order[i]],index=(angle-15U)/5U;
        bool component=i>=first && i<=last;
        /* Side clearance only from >=45 deg off-centre: at 60/120 the ~30 deg
         * beam still hits a close obstacle and made both sides look blocked. */
        if(angle>=135U) { left=true; if(mm<p.left_clear_mm) { p.left_clear_mm=(uint16_t)mm; } }
        if(angle<=45U) { right=true; if(mm<p.right_clear_mm) { p.right_clear_mm=(uint16_t)mm; } }
        /* A disconnected/distant return does not enlarge this obstacle, but
         * it remains a possible blocker on the corresponding route. */
        if(!component && angle>90U && mm<p.left_clear_mm) { p.left_clear_mm=(uint16_t)mm; }
        if(!component && angle<90U && mm<p.right_clear_mm) { p.right_clear_mm=(uint16_t)mm; }
        if(component) {
            /* Working beam envelope: +/-15 degrees, not a calibrated sensor
             * accuracy guarantee. Keep both bearing extremes; shifting a
             * return toward centre can underestimate the obstacle edge.
             * Outward 1 mm padding covers the integer trig table rounding. */
            unsigned low=index-3U,high=index+3U;
            unsigned near_cos=cy[low]<cy[high]?(unsigned)cy[low]:(unsigned)cy[high];
            unsigned far_cos=(low<=15U && high>=15U)?1000U:
                             (cy[low]>cy[high]?(unsigned)cy[low]:(unsigned)cy[high]);
            int x_low=floor_milli((int)mm*sx[low])-1;
            int x_high=ceil_milli((int)mm*sx[high])+1;
            unsigned y_low=mm*near_cos/1000U;
            unsigned y_high=(mm*far_cos+999U)/1000U;
            if(y_low) { y_low--; }
            if(y_high<mm) { y_high++; }
            if(mm<p.closest_mm) { p.closest_mm=(uint16_t)mm; }
            if(x_high>p.left_edge_mm) { p.left_edge_mm=(int16_t)x_high; }
            if(x_low<p.right_edge_mm) { p.right_edge_mm=(int16_t)x_low; }
            if(y_high>p.forward_mm) { p.forward_mm=(uint16_t)y_high; }
            if(y_low<p.front_mm) { p.front_mm=(uint16_t)y_low; }
        }
    }
    if(!left || !right) { return false; }
    /* These bound observed returns only; no promise that acoustically
     * invisible surfaces or an unseen rear face are detected. */
    unsigned width=(unsigned)(p.left_edge_mm-p.right_edge_mm);
    if(width>4095U) { return false; }
    p.width_mm=(uint16_t)width;
    p.valid=true; *out=p; return true;
}
bool s5_peek_merge(s5_profile_t *p,const s5_sample_t *s)
{
    unsigned angle,mm; bool valid;
    if(!p || !s || !s5_sample_unpack(s,&angle,&mm,&valid) || !valid ||
       (angle>60U && angle<120U)) { return false; }
    if(!p->valid) {
        *p=(s5_profile_t){0}; p->closest_mm=(uint16_t)mm;
        p->left_clear_mm=p->right_clear_mm=S5_RANGE_MAX_MM; p->valid=true;
    }
    if(angle>=120U) { p->left_clear_mm=(uint16_t)mm; } else { p->right_clear_mm=(uint16_t)mm; }
    return true;
}
