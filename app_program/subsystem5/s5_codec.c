/** @file s5_codec.c
 * @brief Encode and decode copied obstacle-observation packets.
 * Byte and flag masks are widened unsigned before bitwise operations.
 */
#include "s5_codec.h"

static void     write_bits (uint8_t * p_packet, unsigned * p_at, uint32_t value,
                            unsigned bits);
static uint32_t read_bits (const uint8_t * p_packet, unsigned * p_at,
                           unsigned bits);

/** @brief Check range, flags and scan geometry, then copy the canonical
 * encoded bytes to the output. Invalid observations leave the output
 * untouched.
 */
bool s5_codec_observation_pack (const s5_codec_observation_t * p_observation,
                                uint8_t p_out[S5_PACKET_BYTES])
{
    unsigned bit_position             = 0;
    uint8_t  encoded[S5_PACKET_BYTES] = {0};

    if ((((((0 == p_observation) || (!p_out)) ||
           (p_observation->range_mm > 4000U)) ||
          ((p_observation->b_is_range_valid) &&
           (p_observation->range_mm < 30U))) ||
         ((!p_observation->b_is_range_valid) && (p_observation->range_mm))) ||
        ((p_observation->b_has_no_echo) && (p_observation->b_is_range_valid)))
    {
        return false;
    }

    const profile_obstacle_t * p_profile = &p_observation->profile;

    if ((((((((((p_profile->closest_mm > 4000U) ||
                (p_profile->width_mm > 4095U)) ||
               (p_profile->left_clear_mm > 4000U)) ||
              (p_profile->right_clear_mm > 4000U)) ||
             (p_profile->forward_mm > 4000U)) ||
            (p_profile->left_edge_mm < -4000)) ||
           (p_profile->left_edge_mm > 4000)) ||
          (p_profile->right_edge_mm < -4000)) ||
         (p_profile->right_edge_mm > 4000)) ||
        ((p_profile->b_is_valid) &&
         (((!p_observation->b_is_scan_done) || (p_profile->closest_mm < 30U)) ||
          (p_profile->left_edge_mm < p_profile->right_edge_mm))))
    {
        return false;
    }

    const unsigned face_width_max_mm = p_profile->face_width_max_mm;
    const unsigned face_width_min_mm = p_profile->face_width_min_mm;

    if (((face_width_min_mm > face_width_max_mm) ||
         (face_width_max_mm > 4095U)) ||
        ((!p_profile->b_is_valid) && (face_width_max_mm != 0U)))
    {
        return false;
    }

    write_bits(encoded, &bit_position, p_observation->range_us, 32);
    write_bits(encoded, &bit_position, p_observation->scan_us, 32);
    write_bits(encoded, &bit_position, p_observation->scan_id, 16);
    write_bits(encoded, &bit_position, p_observation->range_mm, 12);
    write_bits(encoded, &bit_position, p_observation->b_is_range_valid, 1);
    write_bits(encoded, &bit_position, p_observation->b_has_no_echo, 1);
    write_bits(encoded, &bit_position, p_observation->b_is_scan_done, 1);
    write_bits(encoded, &bit_position, p_profile->b_is_valid, 1);
    write_bits(encoded, &bit_position, p_profile->closest_mm, 12);
    write_bits(encoded, &bit_position, p_profile->width_mm, 12);
    write_bits(encoded, &bit_position, p_profile->left_clear_mm, 12);
    write_bits(encoded, &bit_position, p_profile->right_clear_mm, 12);
    write_bits(encoded, &bit_position, p_profile->forward_mm, 12);
    /* Validation bounds each signed edge to ±4000 mm. The bias makes it
     * 0..8000 before conversion and packing into thirteen bits. */
    write_bits(encoded, &bit_position,
               (uint32_t)(p_profile->left_edge_mm + 4000), 13);
    write_bits(encoded, &bit_position,
               (uint32_t)(p_profile->right_edge_mm + 4000), 13);
    write_bits(encoded, &bit_position, p_profile->face_width_min_mm, 12);
    write_bits(encoded, &bit_position, p_profile->face_width_max_mm, 12);

    for (unsigned byte_index = 0; byte_index < S5_PACKET_BYTES; byte_index++)
    {
        p_out[byte_index] = encoded[byte_index];
    }

    return true;
}

/** @brief Decode into a temporary observation and verify its canonical bytes
 * by re-encoding. Write the caller's observation only after every
 * check passes.
 */
bool s5_codec_observation_unpack (const uint8_t p_in[S5_PACKET_BYTES],
                                  s5_codec_observation_t * p_observation)
{
    unsigned               bit_position = 0;
    s5_codec_observation_t decoded      = {0};
    uint8_t                check[S5_PACKET_BYTES];

    if ((!p_in) || (0 == p_observation))
    {
        return false;
    }

    decoded.range_us = read_bits(p_in, &bit_position, 32);
    decoded.scan_us  = read_bits(p_in, &bit_position, 32);
    /* These reads produce at most sixteen bits. The twelve-bit values
     * retain even invalid raw readings for the canonical re-encode check. */
    /* clang-format off */
    decoded.scan_id                =
        (uint16_t)read_bits(p_in, &bit_position, 16);
    decoded.range_mm               =
        (uint16_t)read_bits(p_in, &bit_position, 12);
    decoded.b_is_range_valid       = read_bits(p_in, &bit_position, 1) != 0U;
    decoded.b_has_no_echo          = read_bits(p_in, &bit_position, 1) != 0U;
    decoded.b_is_scan_done         = read_bits(p_in, &bit_position, 1) != 0U;
    decoded.profile.b_is_valid     = read_bits(p_in, &bit_position, 1) != 0U;
    decoded.profile.closest_mm     =
        (uint16_t)read_bits(p_in, &bit_position, 12);
    decoded.profile.width_mm       =
        (uint16_t)read_bits(p_in, &bit_position, 12);
    decoded.profile.left_clear_mm  =
        (uint16_t)read_bits(p_in, &bit_position, 12);
    decoded.profile.right_clear_mm =
        (uint16_t)read_bits(p_in, &bit_position, 12);
    decoded.profile.forward_mm     =
        (uint16_t)read_bits(p_in, &bit_position, 12);
    /* clang-format on */
    /* Raw thirteen-bit values are 0..8191. Unbias in int, so the complete
     * malformed or valid range of -4000..4191 remains representable. */
    /* clang-format off */
    decoded.profile.left_edge_mm  =
        (int16_t)((int)read_bits(p_in, &bit_position, 13) - 4000);
    decoded.profile.right_edge_mm =
        (int16_t)((int)read_bits(p_in, &bit_position, 13) - 4000);
    /* Each raw 12-bit width fits in uint16_t before pair validation. */
    decoded.profile.face_width_min_mm =
        (uint16_t)read_bits(p_in, &bit_position, 12);
    decoded.profile.face_width_max_mm =
        (uint16_t)read_bits(p_in, &bit_position, 12);
    /* clang-format on */

    if (!s5_codec_observation_pack(&decoded, check))
    {
        return false;
    }

    for (unsigned byte_index = 0; byte_index < S5_PACKET_BYTES; byte_index++)
    {
        if (check[byte_index] != p_in[byte_index])
        {
            return false;
        }
    }

    *p_observation = decoded;
    return true;
}

/** @brief Append an unsigned field of the given bit width to a zeroed packet,
 * advancing the caller's bit cursor.
 */
static void write_bits (uint8_t * p_packet, unsigned * p_at, uint32_t value,
                        unsigned bits)
{
    unsigned bit_offset = 0;

    while (bit_offset < bits)
    {
        /* The selected bit is zero or one before shifting within a byte. */
        p_packet[*p_at / 8U] =
            (uint8_t)((unsigned)p_packet[*p_at / 8U] |
                      (((value >> bit_offset) & 1U) << (*p_at % 8U)));
        bit_offset++;
        (*p_at)++;
    }
}

/** @brief Read an unsigned field at the current bit cursor and advance the
 * cursor by its width.
 */
static uint32_t read_bits (const uint8_t * p_packet, unsigned * p_at,
                           unsigned bits)
{
    uint32_t field      = 0;
    unsigned bit_offset = 0;

    while (bit_offset < bits)
    {
        /* Widen the selected zero or one before shifting into the field. */
        field |= ((uint32_t)((p_packet[*p_at / 8U] >> (*p_at % 8U)) & 1U))
                 << bit_offset;
        bit_offset++;
        (*p_at)++;
    }

    return field;
}

/* End of file. */

