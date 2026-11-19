#include "s5_codec.h"

static void put (uint8_t * p_p, unsigned * p_at, uint32_t value, unsigned bits)
{
    for (unsigned value_i = 0; value_i < bits; value_i++, (*p_at)++)
    {
        p_p[*p_at / 8U] |= (uint8_t)(((value >> value_i) & 1U) << (*p_at % 8U));
    }
}

static uint32_t get (const uint8_t * p_p, unsigned * p_at, unsigned bits)
{
    uint32_t value_v = 0;
    for (unsigned value_i = 0; value_i < bits; value_i++, (*p_at)++)
    {
        value_v |= ((uint32_t)((p_p[*p_at / 8U] >> (*p_at % 8U)) & 1U))
                   << value_i;
    }
    return value_v;
}

bool s5_observation_pack (const s5_observation_t * p_v,
                          uint8_t p_out[S5_PACKET_BYTES])
{
    unsigned value_at = 0;
    uint8_t value_p[S5_PACKET_BYTES] = {0};
    if (!p_v || !p_out || p_v->range_mm > 4000U ||
        (p_v->range_valid && p_v->range_mm < 30U) ||
        (!p_v->range_valid && p_v->range_mm) ||
        (p_v->no_echo && p_v->range_valid))
    {
        return false;
    }
    const s5_profile_t * p_q = &p_v->profile;
    if (p_q->closest_mm > 4000U || p_q->width_mm > 4095U ||
        p_q->left_clear_mm > 4000U || p_q->right_clear_mm > 4000U ||
        p_q->forward_mm > 4000U || p_q->left_edge_mm < -4000 ||
        p_q->left_edge_mm > 4000 || p_q->right_edge_mm < -4000 ||
        p_q->right_edge_mm > 4000 ||
        (p_q->valid && (!p_v->scan_done || p_q->closest_mm < 30U ||
                        p_q->left_edge_mm < p_q->right_edge_mm)))
    {
        return false;
    }
    put(value_p, &value_at, p_v->range_us, 32);
    put(value_p, &value_at, p_v->scan_us, 32);
    put(value_p, &value_at, p_v->scan_id, 16);
    put(value_p, &value_at, p_v->range_mm, 12);
    put(value_p, &value_at, p_v->range_valid, 1);
    put(value_p, &value_at, p_v->no_echo, 1);
    put(value_p, &value_at, p_v->scan_done, 1);
    put(value_p, &value_at, p_q->valid, 1);
    put(value_p, &value_at, p_q->closest_mm, 12);
    put(value_p, &value_at, p_q->width_mm, 12);
    put(value_p, &value_at, p_q->left_clear_mm, 12);
    put(value_p, &value_at, p_q->right_clear_mm, 12);
    put(value_p, &value_at, p_q->forward_mm, 12);
    put(value_p, &value_at, (uint32_t)(p_q->left_edge_mm + 4000), 13);
    put(value_p, &value_at, (uint32_t)(p_q->right_edge_mm + 4000), 13);
    for (unsigned value_i = 0; value_i < S5_PACKET_BYTES; value_i++)
    {
        p_out[value_i] = value_p[value_i];
    }
    return true;
}

bool s5_observation_unpack (const uint8_t p_in[S5_PACKET_BYTES],
                            s5_observation_t * p_v)
{
    unsigned value_at = 0;
    s5_observation_t value_t = {0};
    uint8_t check[S5_PACKET_BYTES];
    if (!p_in || !p_v)
    {
        return false;
    }
    value_t.range_us = get(p_in, &value_at, 32);
    value_t.scan_us = get(p_in, &value_at, 32);
    value_t.scan_id = (uint16_t)get(p_in, &value_at, 16);
    value_t.range_mm = (uint16_t)get(p_in, &value_at, 12);
    value_t.range_valid = get(p_in, &value_at, 1) != 0;
    value_t.no_echo = get(p_in, &value_at, 1) != 0;
    value_t.scan_done = get(p_in, &value_at, 1) != 0;
    value_t.profile.valid = get(p_in, &value_at, 1) != 0;
    value_t.profile.closest_mm = (uint16_t)get(p_in, &value_at, 12);
    value_t.profile.width_mm = (uint16_t)get(p_in, &value_at, 12);
    value_t.profile.left_clear_mm = (uint16_t)get(p_in, &value_at, 12);
    value_t.profile.right_clear_mm = (uint16_t)get(p_in, &value_at, 12);
    value_t.profile.forward_mm = (uint16_t)get(p_in, &value_at, 12);
    value_t.profile.left_edge_mm =
        (int16_t)((int)get(p_in, &value_at, 13) - 4000);
    value_t.profile.right_edge_mm =
        (int16_t)((int)get(p_in, &value_at, 13) - 4000);
    if (!s5_observation_pack(&value_t, check))
    {
        return false;
    }
    for (unsigned value_i = 0; value_i < S5_PACKET_BYTES; value_i++)
    {
        if (check[value_i] != p_in[value_i])
        {
            return false;
        }
    }
    *p_v = value_t;
    return true;
}
