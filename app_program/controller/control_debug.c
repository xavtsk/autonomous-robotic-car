#include "control_debug.h"
#include <string.h>
static void put(uint8_t *p, unsigned *bit, unsigned width, uint32_t value)
{
    for (unsigned i=0; i<width; ++i,++*bit)
        p[1U+*bit/8U] |= (uint8_t)(((value>>i)&1U)<<(*bit%8U));
}
static uint32_t get(const uint8_t *p, unsigned *bit, unsigned width)
{
    uint32_t v=0;
    for (unsigned i=0; i<width; ++i,++*bit)
        v |= (uint32_t)((p[1U+*bit/8U]>>(*bit%8U))&1U)<<i;
    return v;
}
bool control_debug_pack(const control_debug_sample_t *s, control_debug_packet_t *p)
{
    unsigned bit=0;
    if (!s || !p || s->motion_state>5 || s->motion_fault>5 || s->drive_state>6 ||
        s->reason>9 || s->pending>4 || s->barcode_status>4 || s->cal_sensor>2 ||
        s->line_error < -100 || s->line_error > 100 ||
        s->duty_left < -100 || s->duty_left > 100 ||
        s->duty_right < -100 || s->duty_right > 100 ||
        s->left_white_limit>1000 || s->left_black_limit>1000 ||
        s->right_white_limit>1000 || s->right_black_limit>1000 ||
        s->barcode_white_limit>1000 || s->barcode_black_limit>1000 ||
        (s->motion_valid && !s->motion_available)) return false;
#define CD_U(name,width) if ((uint64_t)s->name >= (UINT64_C(1)<<width)) return false;
#define CD_S(name,width) if ((int64_t)s->name < -(INT64_C(1)<<(width-1)) || (int64_t)s->name >= (INT64_C(1)<<(width-1))) return false;
    CONTROL_DEBUG_FIELDS(CD_U,CD_S)
#undef CD_U
#undef CD_S
    memset(p,0,sizeof *p); p->bytes[0]=1;
#define CD_U(name,width) put(p->bytes,&bit,width,(uint32_t)s->name);
    CONTROL_DEBUG_FIELDS(CD_U,CD_U)
#undef CD_U
    return true;
}
bool control_debug_unpack(const control_debug_packet_t *p, control_debug_sample_t *s)
{
    unsigned bit=0;
    control_debug_packet_t canonical;
    if (!p || !s || p->bytes[0]!=1) return false;
#define CD_U(name,width) s->name=get(p->bytes,&bit,width);
#define CD_S(name,width) { uint32_t v=get(p->bytes,&bit,width); s->name=(int32_t)(v & ((UINT32_C(1)<<(width-1))-1U)) - (int32_t)(v & (UINT32_C(1)<<(width-1))); }
    CONTROL_DEBUG_FIELDS(CD_U,CD_S)
#undef CD_U
#undef CD_S
    return control_debug_pack(s,&canonical) && memcmp(p,&canonical,sizeof *p)==0;
}
