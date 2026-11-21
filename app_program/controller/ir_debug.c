/** @file ir_debug.c
 * @brief Optional observation-only IR preview.
 * Byte and flag masks are widened unsigned before bitwise operations.
 */
#include "ir_debug.h"
#include <string.h>

static void     write_bits (uint8_t * p_bytes, unsigned offset, unsigned width,
                            uint32_t value);
static uint32_t read_bits (const uint8_t * p_bytes, unsigned offset,
                           unsigned width);

/** @brief Validate raw and normalized validity masks and their enabled
 * channels before writing the packet. Return false on invalid input.
 * Absent channels encode as zero.
 */
bool ir_debug_pack (const ir_debug_sample_t * p_sample,
                    ir_debug_packet_t *       p_packet)
{
    /* Narrow the complement to the eight validity bits stored in the mask. */
    if (((((0 == p_sample) || (0 == p_packet)) || (p_sample->raw_valid > 7U)) ||
         (p_sample->normalised_valid > 7U)) ||
        (p_sample->normalised_valid & (uint8_t)~p_sample->raw_valid))
    {
        return false;
    }

    for (unsigned channel_index = 0U; channel_index < IR_DEBUG_CHANNELS;
         ++channel_index)
    {
        if (((p_sample->raw_valid >> channel_index) & 1U) &&
            (p_sample->raw[channel_index] > 4095U))
        {
            return false;
        }

        if (((p_sample->normalised_valid >> channel_index) & 1U) &&
            ((p_sample->darkness[channel_index] > 1000U) ||
             (p_sample->surface[channel_index] > 2U)))
        {
            return false;
        }
    }

    memset(p_packet, 0, sizeof *p_packet);
    p_packet->bytes[0] = IR_DEBUG_VERSION;
    write_bits(p_packet->bytes, 0U, 32U, p_sample->timestamp_us);

    for (unsigned channel_index = 0U; channel_index < IR_DEBUG_CHANNELS;
         ++channel_index)
    {
        if ((p_sample->raw_valid >> channel_index) & 1U)
        {
            write_bits(p_packet->bytes, 32U + (12U * channel_index), 12U,
                       p_sample->raw[channel_index]);
        }

        if ((p_sample->normalised_valid >> channel_index) & 1U)
        {
            write_bits(p_packet->bytes, 68U + (10U * channel_index), 10U,
                       p_sample->darkness[channel_index]);
            write_bits(p_packet->bytes, 104U + (2U * channel_index), 2U,
                       p_sample->surface[channel_index]);
        }
    }

    write_bits(p_packet->bytes, 98U, 3U, p_sample->raw_valid);
    write_bits(p_packet->bytes, 101U, 3U, p_sample->normalised_valid);
    write_bits(p_packet->bytes, 110U, 8U, p_sample->sequence);
    return true;
}

/** @brief Check version and reserved bits, populate caller storage, then
 * re-encode to verify canonical bytes. A late validation failure can
 * leave the decoded fields in caller storage.
 */
bool ir_debug_unpack (const ir_debug_packet_t * p_packet,
                      ir_debug_sample_t *       p_sample)
{

    if ((((0 == p_packet) || (0 == p_sample)) ||
         (p_packet->bytes[0] != IR_DEBUG_VERSION)) ||
        (p_packet->bytes[15] & 0xc0U))
    {
        return false;
    }

    memset(p_sample, 0, sizeof *p_sample);
    p_sample->timestamp_us = read_bits(p_packet->bytes, 0U, 32U);
    /* The wire widths cap the two masks at seven and sequence at 255. */
    p_sample->raw_valid        = (uint8_t)read_bits(p_packet->bytes, 98U, 3U);
    p_sample->normalised_valid = (uint8_t)read_bits(p_packet->bytes, 101U, 3U);
    p_sample->sequence         = (uint8_t)read_bits(p_packet->bytes, 110U, 8U);

    for (unsigned channel_index = 0U; channel_index < IR_DEBUG_CHANNELS;
         ++channel_index)
    {
        /* The 12-, 10- and 2-bit fields fit their destination types. */
        /* clang-format off */
        p_sample->raw[channel_index]      =
            (uint16_t)read_bits(p_packet->bytes,
                                32U + (12U * channel_index), 12U);
        p_sample->darkness[channel_index] =
            (uint16_t)read_bits(p_packet->bytes,
                                68U + (10U * channel_index), 10U);
        p_sample->surface[channel_index]  =
            (uint8_t)read_bits(p_packet->bytes,
                                104U + (2U * channel_index), 2U);
        /* clang-format on */
    }

    ir_debug_packet_t canonical;
    return (ir_debug_pack(p_sample, &canonical)) &&
           (memcmp(p_packet, &canonical, sizeof *p_packet) == 0);
}

/** @brief Accept the first observation and subsequent ones separated by at
 * least IR_DEBUG_PERIOD_US on a wrapping microsecond clock. On
 * acceptance update the seen flag and timestamp.
 */
bool ir_debug_due (uint32_t now, uint32_t * p_last, bool * pb_has_sampled)
{
    /* Elapsed microseconds wrap modulo 2^32 at the hardware clock rollover. */
    if (((0 == p_last) || (0 == pb_has_sampled)) ||
        ((*pb_has_sampled) && ((now - *p_last) < IR_DEBUG_PERIOD_US)))
    {
        return false;
    }

    *pb_has_sampled = true;
    *p_last         = now;
    return true;
}

/** @brief Set a field's one bits in a zeroed packet, least significant bit
 * first. Offset and width refer to bits after the version byte.
 */
static void write_bits (uint8_t * p_bytes, unsigned offset, unsigned width,
                        uint32_t value)
{
    for (unsigned bit_index = 0U; bit_index < width; ++bit_index)
    {
        if ((value >> bit_index) & 1U)
        {
            /* A shift of zero through seven gives one bit within a byte. */
            p_bytes[1U + ((offset + bit_index) / 8U)] =
                (uint8_t)((unsigned)p_bytes[1U + ((offset + bit_index) / 8U)] |
                          (1U << ((offset + bit_index) % 8U)));
        }
    }
}

/** @brief Read an unsigned field of at most 32 bits from the packet payload,
 * using a bit offset after the version byte.
 */
static uint32_t read_bits (const uint8_t * p_bytes, unsigned offset,
                           unsigned width)
{
    uint32_t value = 0U;

    for (unsigned bit_index = 0U; bit_index < width; ++bit_index)
    {
        /* Widen the extracted zero or one before shifting it into the result.
         */
        value |= (uint32_t)((p_bytes[1U + ((offset + bit_index) / 8U)] >>
                             ((offset + bit_index) % 8U)) &
                            1U)
                 << bit_index;
    }

    return value;
}

/* End of file. */

