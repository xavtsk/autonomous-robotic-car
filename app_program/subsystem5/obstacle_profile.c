#include "obstacle_profile.h"
/* sin/cos of angles -60..60 in 5-degree steps, scaled by 1000.
 * x positive LEFT, y forward, servo 90 straight, 150 left. */
static const int16_t sx[25]={-866,-819,-766,-707,-643,-574,-500,-423,-342,-259,-174,-87,0,87,174,259,342,423,500,574,643,707,766,819,866};
static const int16_t cy[25]={500,574,643,707,766,819,866,906,940,966,985,996,1000,996,985,966,940,906,866,819,766,707,643,574,500};
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
bool s5_profile_build(const s5_sample_t *samples,unsigned count,
                      const s5_config_t *cfg,s5_profile_t *out)
{
    s5_profile_t p={0}; bool found=false,left=false,right=false,centre=false;
    if(!out) { return false; } *out=p;
    if(!samples || !cfg || count<5U || count>S5_SAMPLES_MAX) { return false; }
    p.closest_mm=p.left_clear_mm=p.right_clear_mm=S5_RANGE_MAX_MM;
    p.left_edge_mm=-4000; p.right_edge_mm=4000;
    for(unsigned i=0;i<count;i++) {
        unsigned angle,mm,index; bool valid;
        if(!s5_sample_unpack(&samples[i],&angle,&mm,&valid) || !valid) { return false; }
        index=(angle-30U)/5U;
        if(angle>=120U) { left=true; if(mm<p.left_clear_mm) { p.left_clear_mm=(uint16_t)mm; } }
        if(angle<=60U) { right=true; if(mm<p.right_clear_mm) { p.right_clear_mm=(uint16_t)mm; } }
        if(angle==90U) { centre=true; }
        if(mm<cfg->profile_mm) {
            int x=(int)mm*sx[index]/1000, y=(int)mm*cy[index]/1000;
            found=true;
            if(mm<p.closest_mm) { p.closest_mm=(uint16_t)mm; }
            if(x>p.left_edge_mm) { p.left_edge_mm=(int16_t)x; }
            if(x<p.right_edge_mm) { p.right_edge_mm=(int16_t)x; }
            if(y>p.forward_mm) { p.forward_mm=(uint16_t)y; }
        }
    }
    if(!found || !left || !right || !centre) { return false; }
    /* A point return is not an obstacle edge. Inflate by half the largest
     * coarse angular gap (15 deg) plus nominal half beam (7.5 deg):
     * sin(22.5 deg) ~= .383. This is a conservative application envelope,
     * not a promise that acoustically invisible surfaces are detectable. */
    int uncertainty=(int)((cfg->profile_mm*383U+999U)/1000U);
    p.left_edge_mm=(int16_t)(p.left_edge_mm+uncertainty);
    p.right_edge_mm=(int16_t)(p.right_edge_mm-uncertainty);
    p.width_mm=(uint16_t)(p.left_edge_mm-p.right_edge_mm);
    p.valid=true; *out=p; return true;
}
