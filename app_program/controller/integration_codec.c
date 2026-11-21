/** @file integration_codec.c
 * @brief Pack and unpack copied controller command and status data.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 * Byte and flag masks are widened unsigned before bitwise operations.
 */
#include "integration_codec.h"
#include "../build_assert.h"
#include "../app_types.h"
#include <float.h>
/* Compile-time guarantee: exact odometry and calibration representations. */
BUILD_ASSERT(sizeof(float64_t) == 8U && DBL_MANT_DIG == 53 &&
                 sizeof(float32_t) == 4U,
             integration_codec_c_check_a_t);
/* Compile-time guarantee: copied runtime storage. */
BUILD_ASSERT(sizeof(integration_codec_status_packet_t) == 146U &&
                 sizeof(integration_codec_control_packet_t) == 11U,
             integration_codec_c_check_b_t);

static void     write_bits (uint8_t * p_bytes, unsigned * p_bit, uint32_t value,
                            unsigned width);
static uint32_t read_bits (const uint8_t * p_bytes, unsigned * p_bit,
                           unsigned width);
static void put_double (uint8_t * p_bytes, unsigned * p_bit, float64_t value);
static float64_t get_double (const uint8_t * p_bytes, unsigned * p_bit);
static void put_float (uint8_t * p_bytes, unsigned * p_bit, float32_t value);
static float32_t get_float (const uint8_t * p_bytes, unsigned * p_bit);
static bool      summary_valid (const ir_calibration_summary_t * p_summary);
static void      write_summary (uint8_t * p_bytes, unsigned * p_bit,
                                const ir_calibration_summary_t * p_summary);
static void      read_summary (const uint8_t * p_bytes, unsigned * p_bit,
                               ir_calibration_summary_t * p_summary);

/** @brief Publish a canonical control packet only after validation succeeds.
 * @param p_value Request source described in the header contract.
 * @param p_out Packet destination, unchanged on rejection.
 * @return Whether the entire packet was encoded and assigned.
 * @see integration_codec.h for field applicability and null-pointer behavior.
 */
bool integration_codec_pack_cmd (const s3_integration_request_t *     p_value,
                                 integration_codec_control_packet_t * p_out)
{
    integration_codec_control_packet_t packet = {{0}};
    unsigned                           bit    = 0;

    if (((0 == p_value) || (0 == p_out)) ||
        ((unsigned)p_value->kind > (unsigned int)S3_CONTROL_RESET))
    {
        return false;
    }

    const bool b_needs_sensor =
        (((unsigned int)S3_CONTROL_WHITE == p_value->kind) ||
         ((unsigned int)S3_CONTROL_BLACK == p_value->kind)) ||
        ((unsigned int)S3_CONTROL_APPLY_CALIBRATION == p_value->kind);

    if (b_needs_sensor &&
        ((unsigned)p_value->sensor >= (unsigned int)S3_SENSOR_COUNT))
    {
        return false;
    }

    write_bits(packet.bytes, &bit, p_value->kind, 3);
    write_bits(packet.bytes, &bit, b_needs_sensor ? p_value->sensor : 0U, 2);

    for (unsigned letter_index = 0; letter_index < S3_BARCODE_LETTERS;
         letter_index++)
    {
        const unsigned action =
            (unsigned int)S3_CONTROL_MAPPING == p_value->kind
                ? (unsigned)p_value->map[letter_index]
                : 0U;

        if (action >= (unsigned int)S3_NAV_ACTION_COUNT)
        {
            return false;
        }

        write_bits(packet.bytes, &bit, action, 3);
    }

    *p_out = packet;
    return true;
}

/** @brief Decode a request and reject any noncanonical byte representation.
 * @param p_packet Packet source; follows p_packet in the header contract.
 * @param p_out Request destination, unchanged on rejection.
 * @return Whether the decoded request was assigned after a repack comparison.
 * @see integration_codec.h for reserved fields and null-pointer behavior.
 */
bool integration_codec_unpack_cmd (
    const integration_codec_control_packet_t * p_packet,
    s3_integration_request_t *                 p_out)
{
    s3_integration_request_t decoded = {0};
    unsigned                 bit     = 0;

    if (((0 == p_packet) || (0 == p_out)) || (p_packet->bytes[10] & 0xf8U))
    {
        return false;
    }

    /* clang-format off */
    decoded.kind   = (s3_integration_kind_t)read_bits(p_packet->bytes, &bit, 3);
    decoded.sensor =
        (ir_acquisition_sensor_id_t)read_bits(p_packet->bytes, &bit, 2);
    /* clang-format on */

    for (unsigned letter_index = 0; letter_index < S3_BARCODE_LETTERS;
         letter_index++)
    {
        decoded.map[letter_index] =
            (line_navigation_action_t)read_bits(p_packet->bytes, &bit, 3);
    }

    integration_codec_control_packet_t canonical;
    if (!integration_codec_pack_cmd(&decoded, &canonical))
    {
        return false;
    }

    for (unsigned byte_index = 0; byte_index < sizeof(*p_packet); byte_index++)
    {
        if (p_packet->bytes[byte_index] != canonical.bytes[byte_index])
        {
            return false;
        }
    }

    *p_out = decoded;
    return true;
}

/** @brief Encode and publish a complete copied runtime status packet.
 * @param p_value Stable status source; follows p_value in the header contract.
 * @param p_out Packet destination, unchanged on rejection.
 * @return Whether all range/navigation checks passed and output was assigned.
 * @see integration_codec.h for exact representation and validation limits.
 */
bool integration_codec_pack_status (const s3_integration_status_t *     p_value,
                                    integration_codec_status_packet_t * p_out)
{
    integration_codec_status_packet_t packet = {0};
    unsigned                          bit    = 0;

    if ((((((((((0 == p_value) || (0 == p_out)) ||
               ((unsigned)p_value->state > (unsigned int)S3_DRIVE_FAULT)) ||
              ((unsigned)p_value->reason >
               (unsigned int)S3_DRIVE_TERRAIN_FAULT)) ||
             ((unsigned)p_value->motion.state > (unsigned int)MOTION_FAULT)) ||
            ((unsigned)p_value->motion.fault >
             (unsigned int)MOTION_FAULT_TIMEOUT)) ||
           ((unsigned)p_value->calibration_sensor >=
            (unsigned int)S3_SENSOR_COUNT)) ||
          (!summary_valid(&p_value->white))) ||
         (!summary_valid(&p_value->black))) ||
        (!navigation_codec_pack(&p_value->navigation, &packet.navigation)))
    {
        return false;
    }

    put_double(packet.feedback, &bit, p_value->motion.centre_m);
    write_bits(packet.feedback, &bit, p_value->motion.timestamp_us, 32);
    write_bits(packet.feedback, &bit, p_value->motion.epoch, 32);
    write_bits(packet.feedback, &bit, p_value->motion.state, 3);
    write_bits(packet.feedback, &bit, p_value->motion.fault, 3);
    write_bits(packet.feedback, &bit, p_value->motion.b_is_feedback_valid, 1);
    write_bits(packet.feedback, &bit, p_value->motion.b_is_braked, 1);
    write_bits(packet.feedback, &bit, p_value->motion.command_id, 8);
    write_bits(packet.feedback, &bit, p_value->motion.report_loss_count, 32);
    bit = 0;
    write_bits(packet.metadata, &bit, p_value->state, 3);
    write_bits(packet.metadata, &bit, p_value->reason, 4);
    write_bits(packet.metadata, &bit, p_value->samples, 32);
    write_bits(packet.metadata, &bit, p_value->max_gap_us, 32);
    write_bits(packet.metadata, &bit, p_value->above_target, 32);
    write_bits(packet.metadata, &bit, p_value->command_sequence, 32);
    write_bits(packet.metadata, &bit, p_value->b_did_command_succeed, 1);
    write_bits(packet.metadata, &bit, p_value->b_is_collecting, 1);
    write_bits(packet.metadata, &bit, p_value->calibration_sensor, 2);
    write_summary(packet.metadata, &bit, &p_value->white);
    write_summary(packet.metadata, &bit, &p_value->black);
    /* Preserve the full signed ADC status as a modulo-2^32 wire word. */
    write_bits(packet.metadata, &bit, (uint32_t)p_value->adc_status, 32);
    write_bits(packet.metadata, &bit, p_value->run_timing.started_us, 32);
    write_bits(packet.metadata, &bit, p_value->run_timing.samples, 32);
    write_bits(packet.metadata, &bit, p_value->run_timing.max_gap_us, 32);
    write_bits(packet.metadata, &bit, p_value->run_timing.above_target, 32);
    write_bits(packet.metadata, &bit, p_value->run_timing.above_safety, 32);
    write_bits(packet.metadata, &bit, p_value->run_timing.last_safety_us, 32);
    *p_out = packet;
    return true;
}

/** @brief Validate packet structure and publish a decoded status copy.
 * @param p_packet Packet source; follows p_packet in the header contract.
 * @param p_out Status destination, unchanged on rejection.
 * @return Whether all structural checks passed and output was assigned.
 * @see integration_codec.h for enum, reserved-bit and floating-value behavior.
 */
bool integration_codec_unpack_status (
    const integration_codec_status_packet_t * p_packet,
    s3_integration_status_t *                 p_out)
{
    s3_integration_status_t decoded = {0};
    unsigned                bit     = 0;

    if ((((0 == p_packet) || (0 == p_out)) ||
         (p_packet->metadata[63] & 0xe0U)) ||
        (!navigation_codec_unpack(&p_packet->navigation, &decoded.navigation)))
    {
        return false;
    }

    decoded.motion.centre_m     = get_double(p_packet->feedback, &bit);
    decoded.motion.timestamp_us = read_bits(p_packet->feedback, &bit, 32);
    decoded.motion.epoch        = read_bits(p_packet->feedback, &bit, 32);

    /* clang-format off */
    decoded.motion.state               =
        (motion_state_t)read_bits(p_packet->feedback, &bit, 3);
    decoded.motion.fault               =
        (motion_fault_t)read_bits(p_packet->feedback, &bit, 3);
    decoded.motion.b_is_feedback_valid =
        read_bits(p_packet->feedback, &bit, 1) != 0U;
    decoded.motion.b_is_braked         =
        read_bits(p_packet->feedback, &bit, 1) != 0U;
    /* clang-format on */

    /* The eight-bit wire ID is in 0..255, including the zero sentinel. */
    /* clang-format off */
    decoded.motion.command_id        =
        (uint8_t)read_bits(p_packet->feedback, &bit, 8);
    decoded.motion.report_loss_count = read_bits(p_packet->feedback, &bit, 32);
    /* clang-format on */

    if ((decoded.motion.state > (unsigned int)MOTION_FAULT) ||
        (decoded.motion.fault > (unsigned int)MOTION_FAULT_TIMEOUT))
    {
        return false;
    }

    bit            = 0;
    decoded.state  = (s3_drive_state_t)read_bits(p_packet->metadata, &bit, 3);
    decoded.reason = (s3_drive_reason_t)read_bits(p_packet->metadata, &bit, 4);

    if ((decoded.state > (unsigned int)S3_DRIVE_FAULT) ||
        (decoded.reason > (unsigned int)S3_DRIVE_TERRAIN_FAULT))
    {
        return false;
    }

    /* clang-format off */
    decoded.samples               = read_bits(p_packet->metadata, &bit, 32);
    decoded.max_gap_us            = read_bits(p_packet->metadata, &bit, 32);
    decoded.above_target          = read_bits(p_packet->metadata, &bit, 32);
    decoded.command_sequence      = read_bits(p_packet->metadata, &bit, 32);
    decoded.b_did_command_succeed =
        read_bits(p_packet->metadata, &bit, 1) != 0U;
    decoded.b_is_collecting       =
        read_bits(p_packet->metadata, &bit, 1) != 0U;
    decoded.calibration_sensor    =
        (ir_acquisition_sensor_id_t)read_bits(p_packet->metadata, &bit, 2);
    /* clang-format on */

    if (decoded.calibration_sensor >= (unsigned int)S3_SENSOR_COUNT)
    {
        return false;
    }

    read_summary(p_packet->metadata, &bit, &decoded.white);
    read_summary(p_packet->metadata, &bit, &decoded.black);
    const uint32_t error = read_bits(p_packet->metadata, &bit, 32);

    /* Compare in the unsigned wire domain. Both branches produce
     * an int32_t value from a representable magnitude. */
    decoded.adc_status = error <= (uint32_t)INT32_MAX
                             ? (int32_t)error
                             : -1 - (int32_t)(UINT32_MAX - error);

    decoded.run_timing.started_us     = read_bits(p_packet->metadata, &bit, 32);
    decoded.run_timing.samples        = read_bits(p_packet->metadata, &bit, 32);
    decoded.run_timing.max_gap_us     = read_bits(p_packet->metadata, &bit, 32);
    decoded.run_timing.above_target   = read_bits(p_packet->metadata, &bit, 32);
    decoded.run_timing.above_safety   = read_bits(p_packet->metadata, &bit, 32);
    decoded.run_timing.last_safety_us = read_bits(p_packet->metadata, &bit, 32);
    *p_out                            = decoded;
    return true;
}

/** @brief Append the low width bits of value, least-significant bit first.
 * @param p_bytes Writable packet containing zeroed destination bits.
 * @param p_bit Nonnull bit cursor, advanced by width.
 * @param value Unsigned value; bits above width are ignored.
 * @param width Number of bits, at most 32.
 * @pre The packet has space for the full range. Only one bits are written;
 * zero bits remain unchanged. The caller provides ownership and bounds.
 */
static void write_bits (uint8_t * p_bytes, unsigned * p_bit, uint32_t value,
                        unsigned width)
{
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        if (value & ((uint32_t)1U << bit_offset))
        {
            /* The modulo constrains the mask to one bit in a byte. */
            p_bytes[*p_bit / 8U] = (uint8_t)((unsigned)p_bytes[*p_bit / 8U] |
                                             (1U << (*p_bit % 8U)));
        }

        bit_offset++;
        (*p_bit)++;
    }
}

/** @brief Read width packet bits, least-significant bit first.
 * @param p_bytes Readable packet containing the entire requested range.
 * @param p_bit Nonnull bit cursor, advanced by width.
 * @param width Number of bits, at most 32.
 * @return The extracted unsigned value with higher bits zero.
 * @pre Pointers and packet bounds are valid; this helper performs no checks.
 */
static uint32_t read_bits (const uint8_t * p_bytes, unsigned * p_bit,
                           unsigned width)
{
    uint32_t field      = 0;
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        if (p_bytes[*p_bit / 8U] & (1U << (*p_bit % 8U)))
        {
            field |= (uint32_t)1U << bit_offset;
        }

        bit_offset++;
        (*p_bit)++;
    }

    return field;
}

/** @brief Append the existing binary64 representation in low-word-first order.
 * @param p_bytes Writable packet with 64 zeroed bits available at the cursor.
 * @param p_bit Nonnull bit cursor, advanced by 64.
 * @param value Value whose representation is copied without validation.
 * @pre The caller provides valid pointers, bounds and exclusive output access.
 */
static void put_double (uint8_t * p_bytes, unsigned * p_bit, float64_t value)
{
    app_types_binary64_view_t copy;

    copy.value = value;
    /* The first conversion keeps the low 32 bits of the representation;
     * shifting gives the next conversion exactly the high 32 bits. */
    write_bits(p_bytes, p_bit, (uint32_t)copy.bits, 32);
    write_bits(p_bytes, p_bit, (uint32_t)(copy.bits >> 32), 32);
}

/** @brief Reconstruct binary64 from two low-word-first packet words.
 * @param p_bytes Readable packet with 64 bits available at the cursor.
 * @param p_bit Nonnull bit cursor, advanced by 64.
 * @return The stored floating representation, including nonfinite values.
 * @pre The caller provides valid pointers and packet bounds.
 */
static float64_t get_double (const uint8_t * p_bytes, unsigned * p_bit)
{

    const uint32_t            value_lo = read_bits(p_bytes, p_bit, 32);
    const uint32_t            value_hi = read_bits(p_bytes, p_bit, 32);
    app_types_binary64_view_t copy;
    copy.bits = (uint64_t)value_lo | ((uint64_t)value_hi << 32);
    return copy.value;
}

/** @brief Append a binary32 representation without numeric conversion.
 * @param p_bytes Writable packet with 32 zeroed bits available at the cursor.
 * @param p_bit Nonnull bit cursor, advanced by 32.
 * @param value Value whose representation is copied without validation.
 * @pre The caller provides valid pointers, bounds and exclusive output access.
 */
static void put_float (uint8_t * p_bytes, unsigned * p_bit, float32_t value)
{
    app_types_binary32_view_t copy;

    copy.value = value;
    write_bits(p_bytes, p_bit, copy.bits, 32);
}

/** @brief Reconstruct binary32 from the next 32 packet bits.
 * @param p_bytes Readable packet with 32 bits available at the cursor.
 * @param p_bit Nonnull bit cursor, advanced by 32.
 * @return The stored floating representation, including nonfinite values.
 * @pre The caller provides valid pointers and packet bounds.
 */
static float32_t get_float (const uint8_t * p_bytes, unsigned * p_bit)
{
    app_types_binary32_view_t copy;

    copy.bits = read_bits(p_bytes, p_bit, 32);
    return copy.value;
}

/** @brief Check that a calibration summary's integer fields fit the packet.
 * @param p_summary Nonnull summary to read.
 * @return True for count at most 65535 and both extrema at most 4095.
 * @note This check covers field widths only. The extrema order,
 * availability and average are outside its scope.
 */
static bool summary_valid (const ir_calibration_summary_t * p_summary)
{
    return ((p_summary->count <= 65535U) && (p_summary->minimum <= 4095U)) &&
           (p_summary->maximum <= 4095U);
}

/** @brief Append a calibration summary's 73-bit stored representation.
 * @param p_bytes Writable packet with 73 zeroed bits available at the cursor.
 * @param p_bit Nonnull bit cursor, advanced by 73.
 * @param p_summary Nonnull summary already accepted by summary_valid.
 * @pre The caller owns the output and has checked packet bounds.
 */
static void write_summary (uint8_t * p_bytes, unsigned * p_bit,
                           const ir_calibration_summary_t * p_summary)
{
    write_bits(p_bytes, p_bit, p_summary->count, 16);
    write_bits(p_bytes, p_bit, p_summary->minimum, 12);
    write_bits(p_bytes, p_bit, p_summary->maximum, 12);
    put_float(p_bytes, p_bit, p_summary->average);
    write_bits(p_bytes, p_bit, p_summary->b_is_available, 1);
}

/** @brief Extract a calibration summary from 73 stored bits.
 * @param p_bytes Readable packet with 73 bits available at the cursor.
 * @param p_bit Nonnull bit cursor, advanced by 73.
 * @param p_summary Nonnull destination; all five summary fields are
 * overwritten.
 * @pre The caller owns the destination and has checked packet bounds.
 */
static void read_summary (const uint8_t * p_bytes, unsigned * p_bit,
                          ir_calibration_summary_t * p_summary)
{
    /* Each twelve-bit endpoint fits in its uint16_t destination. */
    p_summary->count          = read_bits(p_bytes, p_bit, 16);
    p_summary->minimum        = (uint16_t)read_bits(p_bytes, p_bit, 12);
    p_summary->maximum        = (uint16_t)read_bits(p_bytes, p_bit, 12);
    p_summary->average        = get_float(p_bytes, p_bit);
    p_summary->b_is_available = read_bits(p_bytes, p_bit, 1) != 0U;
}

/* End of file. */

