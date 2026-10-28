#include "ir_debug.h"
#include <string.h>

static void put(uint8_t *bytes, unsigned offset, unsigned width, uint32_t value)
{
    for (unsigned i = 0U; i < width; ++i) {
        if ((value >> i) & 1U) bytes[1U + (offset + i) / 8U] |= (uint8_t)(1U << ((offset + i) % 8U));
    }
}
static uint32_t get(const uint8_t *bytes, unsigned offset, unsigned width)
{
    uint32_t value = 0U;
    for (unsigned i = 0U; i < width; ++i) {
        value |= (uint32_t)((bytes[1U + (offset + i) / 8U] >> ((offset + i) % 8U)) & 1U) << i;
    }
    return value;
}
bool ir_debug_pack(const ir_debug_sample_t *s, ir_debug_packet_t *p)
{
    if (!s || !p || s->raw_valid > 7U || s->normalised_valid > 7U ||
        (s->normalised_valid & (uint8_t)~s->raw_valid)) return false;
    for (unsigned i = 0U; i < 3U; ++i) {
        if (((s->raw_valid >> i) & 1U) && s->raw[i] > 4095U) return false;
        if (((s->normalised_valid >> i) & 1U) && (s->darkness[i] > 1000U || s->surface[i] > 2U)) return false;
    }
    memset(p, 0, sizeof *p);
    p->bytes[0] = IR_DEBUG_VERSION;
    put(p->bytes, 0U, 32U, s->timestamp_us);
    for (unsigned i = 0U; i < 3U; ++i) {
        if ((s->raw_valid >> i) & 1U) put(p->bytes, 32U + 12U * i, 12U, s->raw[i]);
        if ((s->normalised_valid >> i) & 1U) {
            put(p->bytes, 68U + 10U * i, 10U, s->darkness[i]);
            put(p->bytes, 104U + 2U * i, 2U, s->surface[i]);
        }
    }
    put(p->bytes, 98U, 3U, s->raw_valid);
    put(p->bytes, 101U, 3U, s->normalised_valid);
    put(p->bytes, 110U, 8U, s->sequence);
    return true;
}
bool ir_debug_unpack(const ir_debug_packet_t *p, ir_debug_sample_t *s)
{
    ir_debug_packet_t canonical;
    if (!p || !s || p->bytes[0] != IR_DEBUG_VERSION || (p->bytes[15] & 0xc0U)) return false;
    memset(s, 0, sizeof *s);
    s->timestamp_us = get(p->bytes, 0U, 32U);
    s->raw_valid = (uint8_t)get(p->bytes, 98U, 3U);
    s->normalised_valid = (uint8_t)get(p->bytes, 101U, 3U);
    s->sequence = (uint8_t)get(p->bytes, 110U, 8U);
    for (unsigned i = 0U; i < 3U; ++i) {
        s->raw[i] = (uint16_t)get(p->bytes, 32U + 12U * i, 12U);
        s->darkness[i] = (uint16_t)get(p->bytes, 68U + 10U * i, 10U);
        s->surface[i] = (uint8_t)get(p->bytes, 104U + 2U * i, 2U);
    }
    return ir_debug_pack(s, &canonical) && memcmp(p, &canonical, sizeof *p) == 0;
}
bool ir_debug_due(uint32_t now, uint32_t *last, bool *seen)
{
    if (!last || !seen || (*seen && (uint32_t)(now - *last) < IR_DEBUG_PERIOD_US)) return false;
    *seen = true;
    *last = now;
    return true;
}
