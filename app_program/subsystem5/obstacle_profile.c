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
    s5_profile_t p={0}; bool found=false,left=false,right=false,centre=false; unsigned far=0;
    if(!out) { return false; } *out=p;
    if(!samples || !cfg || count<5U || count>S5_SAMPLES_MAX) { return false; }
    p.closest_mm=p.left_clear_mm=p.right_clear_mm=S5_RANGE_MAX_MM;
    p.left_edge_mm=-4000; p.right_edge_mm=4000;
    for(unsigned i=0;i<count;i++) {
        unsigned angle,mm,index; bool valid;
        if(!s5_sample_unpack(&samples[i],&angle,&mm,&valid) || !valid) { return false; }
        index=(angle-30U)/5U;
        /* Side clearance only from >=45 deg off-centre: at 60/120 the ~30 deg
         * beam still hits a close obstacle and made both sides look blocked. */
        if(angle>=135U) { left=true; if(mm<p.left_clear_mm) { p.left_clear_mm=(uint16_t)mm; } }
        if(angle<=45U) { right=true; if(mm<p.right_clear_mm) { p.right_clear_mm=(uint16_t)mm; } }
        if(angle==90U) { centre=true; }
        /* Obstacle shape only from 45..135 deg; 30/150 are side-clearance
         * readings (track surroundings there made the obstacle look 391 mm wide). */
        if(mm<cfg->profile_mm && angle>=45U && angle<=135U) {
            /* Beam-width correction: the HC-SR04 reports the nearest return
             * anywhere in its ~30 deg cone, so a return at servo angle t may
             * come from up to 15 deg (3 table steps) nearer the centre. */
            unsigned c=index>12U?(index-3U>12U?index-3U:12U):(index+3U<12U?index+3U:12U);
            int x=(int)mm*sx[c]/1000, y=(int)mm*cy[c]/1000;
            found=true; if(mm>far) { far=mm; }
            if(mm<p.closest_mm) { p.closest_mm=(uint16_t)mm; }
            if(x>p.left_edge_mm) { p.left_edge_mm=(int16_t)x; }
            if(x<p.right_edge_mm) { p.right_edge_mm=(int16_t)x; }
            if(y>p.forward_mm) { p.forward_mm=(uint16_t)y; }
        }
    }
    if(!found || !left || !right || !centre) { return false; }
    /* A point return is not an obstacle edge. With the beam already corrected
     * above, inflate by half the largest coarse angular gap (15 deg):
     * sin(15 deg) ~= .259, at the FARTHEST obstacle return actually seen.
     * Not a promise that acoustically invisible surfaces are detectable. */
    int uncertainty=(int)((far*259U+999U)/1000U);
    p.left_edge_mm=(int16_t)(p.left_edge_mm+uncertainty);
    p.right_edge_mm=(int16_t)(p.right_edge_mm-uncertainty);
    p.width_mm=(uint16_t)(p.left_edge_mm-p.right_edge_mm);
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
