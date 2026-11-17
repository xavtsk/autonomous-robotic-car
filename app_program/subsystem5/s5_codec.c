#include "s5_codec.h"
static void put(uint8_t *p,unsigned *at,uint32_t value,unsigned bits)
{
    for(unsigned i=0;i<bits;i++,(*at)++) { p[*at/8U]|=(uint8_t)(((value>>i)&1U)<<(*at%8U)); }
}
static uint32_t get(const uint8_t *p,unsigned *at,unsigned bits)
{
    uint32_t v=0; for(unsigned i=0;i<bits;i++,(*at)++) { v|=((uint32_t)((p[*at/8U]>>(*at%8U))&1U))<<i; } return v;
}
bool s5_observation_pack(const s5_observation_t *v,uint8_t out[S5_PACKET_BYTES])
{
    unsigned at=0; uint8_t p[S5_PACKET_BYTES]={0};
    if(!v || !out || v->range_mm>4000U || (v->range_valid && v->range_mm<30U) ||
       (!v->range_valid && v->range_mm) || (v->no_echo && v->range_valid)) { return false; }
    const s5_profile_t *q=&v->profile;
    if(q->closest_mm>4000U || q->width_mm>4095U || q->left_clear_mm>4000U ||
       q->right_clear_mm>4000U || q->forward_mm>4000U || q->left_edge_mm < -4000 ||
       q->left_edge_mm>4000 || q->right_edge_mm < -4000 || q->right_edge_mm>4000 ||
       (q->valid && (!v->scan_done || q->closest_mm<30U || q->left_edge_mm<q->right_edge_mm))) { return false; }
    put(p,&at,v->range_us,32); put(p,&at,v->scan_us,32); put(p,&at,v->scan_id,16);
    put(p,&at,v->range_mm,12); put(p,&at,v->range_valid,1); put(p,&at,v->no_echo,1); put(p,&at,v->scan_done,1);
    put(p,&at,q->valid,1); put(p,&at,q->closest_mm,12); put(p,&at,q->width_mm,12);
    put(p,&at,q->left_clear_mm,12); put(p,&at,q->right_clear_mm,12); put(p,&at,q->forward_mm,12);
    put(p,&at,(uint32_t)(q->left_edge_mm+4000),13); put(p,&at,(uint32_t)(q->right_edge_mm+4000),13);
    for(unsigned i=0;i<S5_PACKET_BYTES;i++) { out[i]=p[i]; } return true;
}
bool s5_observation_unpack(const uint8_t in[S5_PACKET_BYTES],s5_observation_t *v)
{
    unsigned at=0; s5_observation_t t={0}; uint8_t check[S5_PACKET_BYTES];
    if(!in || !v) { return false; }
    t.range_us=get(in,&at,32);t.scan_us=get(in,&at,32);t.scan_id=(uint16_t)get(in,&at,16);
    t.range_mm=(uint16_t)get(in,&at,12);t.range_valid=get(in,&at,1)!=0;t.no_echo=get(in,&at,1)!=0;t.scan_done=get(in,&at,1)!=0;
    t.profile.valid=get(in,&at,1)!=0;t.profile.closest_mm=(uint16_t)get(in,&at,12);t.profile.width_mm=(uint16_t)get(in,&at,12);
    t.profile.left_clear_mm=(uint16_t)get(in,&at,12);t.profile.right_clear_mm=(uint16_t)get(in,&at,12);t.profile.forward_mm=(uint16_t)get(in,&at,12);
    t.profile.left_edge_mm=(int16_t)((int)get(in,&at,13)-4000);t.profile.right_edge_mm=(int16_t)((int)get(in,&at,13)-4000);
    if(!s5_observation_pack(&t,check)) { return false; }
    for(unsigned i=0;i<S5_PACKET_BYTES;i++) { if(check[i]!=in[i]) { return false; } }
    *v=t;return true;
}
