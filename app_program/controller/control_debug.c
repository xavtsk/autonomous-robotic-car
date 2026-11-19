#include "control_debug.h"
#include <string.h>

static void put (uint8_t * p_p, unsigned * p_bit, unsigned width,
                 uint32_t value)
{
    for (unsigned value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        p_p[1U + *p_bit / 8U] |=
            (uint8_t)(((value >> value_i) & 1U) << (*p_bit % 8U));
    }
}

static uint32_t get (const uint8_t * p_p, unsigned * p_bit, unsigned width)
{
    uint32_t value_v = 0;
    for (unsigned value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        value_v |= (uint32_t)((p_p[1U + *p_bit / 8U] >> (*p_bit % 8U)) & 1U)
                   << value_i;
    }
    return value_v;
}

static bool unsigned_fits (uint32_t value, unsigned width)
{
    /* The 64-bit limit can represent 2^32 for every schema width. */
    return (uint64_t)value < (UINT64_C(1) << width);
}

static bool signed_fits (int32_t value, unsigned width)
{
    const int64_t limit = INT64_C(1) << (width - 1U);
    return ((int64_t)value >= -limit) && ((int64_t)value < limit);
}

bool control_debug_pack (const control_debug_sample_t * p_sample,
                         control_debug_packet_t * p_packet)
{
    unsigned bit = 0;
    bool fields_valid = true;
    if (!p_sample || !p_packet || p_sample->motion_state > 5 ||
        p_sample->motion_fault > 5 || p_sample->drive_state > 6 ||
        p_sample->reason > 10 || p_sample->pending > 4 ||
        p_sample->barcode_status > 4 || p_sample->cal_sensor > 2 ||
        p_sample->line_error < -100 || p_sample->line_error > 100 ||
        p_sample->duty_left < -100 || p_sample->duty_left > 100 ||
        p_sample->duty_right < -100 || p_sample->duty_right > 100 ||
        p_sample->left_white_limit > 1000 ||
        p_sample->left_black_limit > 1000 ||
        p_sample->right_white_limit > 1000 ||
        p_sample->right_black_limit > 1000 ||
        p_sample->barcode_white_limit > 1000 ||
        p_sample->barcode_black_limit > 1000 ||
        (p_sample->motion_valid && !p_sample->motion_available))
    {
        return false;
    }
#define CD_U(name, width)                                                      \
    (fields_valid = fields_valid && unsigned_fits(p_sample->name, (width)));
#define CD_S(name, width)                                                      \
    (fields_valid = fields_valid && signed_fits(p_sample->name, (width)));
    CONTROL_DEBUG_FIELDS(CD_U, CD_S)
#undef CD_U
#undef CD_S
    if (!fields_valid)
    {
        return false;
    }
    memset(p_packet, 0, sizeof *p_packet);
    p_packet->bytes[0] = 1;
#define CD_U(name, width)                                                      \
    put(p_packet->bytes, &bit, width, (uint32_t)p_sample->name);
    CONTROL_DEBUG_FIELDS(CD_U, CD_U)
#undef CD_U
    return true;
}

bool control_debug_unpack (const control_debug_packet_t * p_packet,
                           control_debug_sample_t * p_sample)
{
    unsigned bit = 0;
    control_debug_packet_t canonical;
    if (!p_packet || !p_sample || p_packet->bytes[0] != 1)
    {
        return false;
    }
#define CD_U(name, width) p_sample->name = get(p_packet->bytes, &bit, width);
#define CD_S(name, width)                                                      \
    {                                                                          \
        uint32_t value_bits = get(p_packet->bytes, &bit, width);               \
        p_sample->name =                                                       \
            (int32_t)(value_bits & ((UINT32_C(1) << (width - 1)) - 1U)) -      \
            (int32_t)(value_bits & (UINT32_C(1) << (width - 1)));              \
    }
    CONTROL_DEBUG_FIELDS(CD_U, CD_S)
#undef CD_U
#undef CD_S
    return control_debug_pack(p_sample, &canonical) &&
           memcmp(p_packet, &canonical, sizeof *p_packet) == 0;
}
