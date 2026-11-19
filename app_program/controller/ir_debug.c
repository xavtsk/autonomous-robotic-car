#include "ir_debug.h"
#include <string.h>

static void put (uint8_t * p_bytes, unsigned offset, unsigned width,
                 uint32_t value)
{
    for (unsigned value_i = 0U; value_i < width; ++value_i)
    {
        if ((value >> value_i) & 1U)
        {
            p_bytes[1U + (offset + value_i) / 8U] |=
                (uint8_t)(1U << ((offset + value_i) % 8U));
        }
    }
}

static uint32_t get (const uint8_t * p_bytes, unsigned offset, unsigned width)
{
    uint32_t value = 0U;
    for (unsigned value_i = 0U; value_i < width; ++value_i)
    {
        value |= (uint32_t)((p_bytes[1U + (offset + value_i) / 8U] >>
                             ((offset + value_i) % 8U)) &
                            1U)
                 << value_i;
    }
    return value;
}

bool ir_debug_pack (const ir_debug_sample_t * p_s, ir_debug_packet_t * p_p)
{
    if (!p_s || !p_p || p_s->raw_valid > 7U || p_s->normalised_valid > 7U ||
        (p_s->normalised_valid & (uint8_t)~p_s->raw_valid))
    {
        return false;
    }
    for (unsigned value_i = 0U; value_i < 3U; ++value_i)
    {
        if (((p_s->raw_valid >> value_i) & 1U) && p_s->raw[value_i] > 4095U)
        {
            return false;
        }
        if (((p_s->normalised_valid >> value_i) & 1U) &&
            (p_s->darkness[value_i] > 1000U || p_s->surface[value_i] > 2U))
        {
            return false;
        }
    }
    memset(p_p, 0, sizeof *p_p);
    p_p->bytes[0] = IR_DEBUG_VERSION;
    put(p_p->bytes, 0U, 32U, p_s->timestamp_us);
    for (unsigned value_i = 0U; value_i < 3U; ++value_i)
    {
        if ((p_s->raw_valid >> value_i) & 1U)
        {
            put(p_p->bytes, 32U + 12U * value_i, 12U, p_s->raw[value_i]);
        }
        if ((p_s->normalised_valid >> value_i) & 1U)
        {
            put(p_p->bytes, 68U + 10U * value_i, 10U, p_s->darkness[value_i]);
            put(p_p->bytes, 104U + 2U * value_i, 2U, p_s->surface[value_i]);
        }
    }
    put(p_p->bytes, 98U, 3U, p_s->raw_valid);
    put(p_p->bytes, 101U, 3U, p_s->normalised_valid);
    put(p_p->bytes, 110U, 8U, p_s->sequence);
    return true;
}

bool ir_debug_unpack (const ir_debug_packet_t * p_p, ir_debug_sample_t * p_s)
{
    ir_debug_packet_t canonical;
    if (!p_p || !p_s || p_p->bytes[0] != IR_DEBUG_VERSION ||
        (p_p->bytes[15] & 0xc0U))
    {
        return false;
    }
    memset(p_s, 0, sizeof *p_s);
    p_s->timestamp_us = get(p_p->bytes, 0U, 32U);
    p_s->raw_valid = (uint8_t)get(p_p->bytes, 98U, 3U);
    p_s->normalised_valid = (uint8_t)get(p_p->bytes, 101U, 3U);
    p_s->sequence = (uint8_t)get(p_p->bytes, 110U, 8U);
    for (unsigned value_i = 0U; value_i < 3U; ++value_i)
    {
        p_s->raw[value_i] = (uint16_t)get(p_p->bytes, 32U + 12U * value_i, 12U);
        p_s->darkness[value_i] =
            (uint16_t)get(p_p->bytes, 68U + 10U * value_i, 10U);
        p_s->surface[value_i] =
            (uint8_t)get(p_p->bytes, 104U + 2U * value_i, 2U);
    }
    return ir_debug_pack(p_s, &canonical) &&
           memcmp(p_p, &canonical, sizeof *p_p) == 0;
}

bool ir_debug_due (uint32_t now, uint32_t * p_last, bool * p_seen)
{
    if (!p_last || !p_seen ||
        (*p_seen && (uint32_t)(now - *p_last) < IR_DEBUG_PERIOD_US))
    {
        return false;
    }
    *p_seen = true;
    *p_last = now;
    return true;
}
