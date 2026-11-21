/** @file navigation_codec.c
 * @brief Pack and unpack navigation, barcode and raw-frame snapshots.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 * Byte and half-word values fit in int, and uint32_t fits in int64_t.
 * The casts make the existing arithmetic promotions explicit.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 * Byte and flag masks are widened unsigned before bitwise operations.
 */
#include "navigation_codec.h"
#include "../build_assert.h"
#include "../app_types.h"
#include <float.h>

/* Compile-time guarantee: shared steering requires the existing binary32 float
 * format. */
BUILD_ASSERT(sizeof(float32_t) == 4U && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
             navigation_codec_c_check_a_t);
/* Compile-time guarantee: navigation packet layout. */
BUILD_ASSERT(sizeof(navigation_codec_packet_t) == 60U,
             navigation_codec_c_check_b_t);
/* Compile-time guarantee: odometry binary64. */
BUILD_ASSERT(sizeof(float64_t) == 8U && DBL_MANT_DIG == 53,
             navigation_codec_c_check_c_t);
/* Compile-time guarantee: barcode packet layout. */
BUILD_ASSERT(sizeof(navigation_codec_barcode_t) == 5U,
             navigation_codec_c_check_d_t);
/* Compile-time guarantee: raw-frame packet layout changed. */
BUILD_ASSERT(sizeof(navigation_codec_raw_frame_t) == 34U &&
                 S3_SENSOR_COUNT == 3,
             navigation_codec_c_check_e_t);
/* Compile-time guarantee: event/sensor mask widths changed. */
BUILD_ASSERT(S3_EVENT_SENSOR_FAULT == 16 && S3_ALL_SENSORS == 7,
             navigation_codec_c_check_f_t);

static void     write_bits (uint8_t * p_bytes, unsigned * p_bit, uint32_t value,
                            unsigned width);
static uint32_t read_bits (const uint8_t * p_bytes, unsigned * p_bit,
                           unsigned width);
static uint32_t float_bits (float32_t value);
static float32_t bits_float (uint32_t value);
static bool      letter_valid (char letter);
static void write_double (uint8_t * p_bytes, unsigned * p_bit, float64_t value);
static float64_t read_double (const uint8_t * p_bytes, unsigned * p_bit);
static void      write_barcode (uint8_t * p_bytes, unsigned * p_bit,
                                const bc_decoder_result_t * p_value);
static bool      read_barcode (const uint8_t * p_bytes, unsigned * p_bit,
                               bc_decoder_result_t * p_value);

/** @brief Build a validated barcode packet locally before publishing it.
 * @see navigation_codec.h for inputs, bit layout and unchanged-on-failure
 * output.
 */
bool navigation_codec_barcode_pack (const bc_decoder_result_t *  p_value,
                                    navigation_codec_barcode_t * p_out)
{
    navigation_codec_barcode_t copy = {{0}};
    unsigned                   bit  = 0;

    if (((0 == p_value) || (0 == p_out)) || (!letter_valid(p_value->letter)))
    {
        return false;
    }

    write_barcode(copy.bytes, &bit, p_value);
    *p_out = copy;
    return true;
}

/** @brief Validate barcode padding and letter code before assigning output.
 * @see navigation_codec.h for units, ownership and failure behavior.
 */
bool navigation_codec_barcode_unpack (
    const navigation_codec_barcode_t * p_packet, bc_decoder_result_t * p_out)
{
    bc_decoder_result_t value = {0};
    unsigned            bit   = 0;

    if ((((0 == p_packet) || (0 == p_out)) ||
         ((p_packet->bytes[4] & 0xc0U) != 0U)) ||
        (!read_barcode(p_packet->bytes, &bit, &value)))
    {
        return false;
    }

    *p_out = value;
    return true;
}

/** @brief Validate all raw readings before publishing a complete frame packet.
 * @see navigation_codec.h for raw/error preservation and output ownership.
 */
bool navigation_codec_raw_pack (const ir_acquisition_frame_t * p_value,
                                navigation_codec_raw_frame_t * p_out)
{
    navigation_codec_raw_frame_t copy = {{0}};
    unsigned                     bit  = 0;

    if ((0 == p_value) || (0 == p_out))
    {
        return false;
    }

    for (unsigned sensor_index = 0;
         sensor_index < (unsigned int)S3_SENSOR_COUNT; ++sensor_index)
    {
        if (p_value->sensor[sensor_index].raw > 4095U)
        {
            return false;
        }
    }

    write_bits(copy.bytes, &bit, p_value->timestamp_us, 32);

    for (unsigned sensor_index = 0;
         sensor_index < (unsigned int)S3_SENSOR_COUNT; ++sensor_index)
    {
        const ir_acquisition_sample_t * p_sample =
            &p_value->sensor[sensor_index];
        write_bits(copy.bytes, &bit, p_sample->timestamp_us, 32);
        write_bits(copy.bytes, &bit, p_sample->raw, 12);
        write_bits(copy.bytes, &bit, (uint32_t)p_sample->error, 32);
        write_bits(copy.bytes, &bit, p_sample->b_is_enabled, 1);
        write_bits(copy.bytes, &bit, p_sample->b_is_valid, 1);
    }

    *p_out = copy;
    return true;
}

/** @brief Reconstruct all sample fields after validating frame padding.
 * @see navigation_codec.h for accepted error/timestamp values and
 * unchanged-on-failure output.
 */
bool navigation_codec_raw_unpack (const navigation_codec_raw_frame_t * p_packet,
                                  ir_acquisition_frame_t *             p_out)
{
    ir_acquisition_frame_t value = {0};
    unsigned               bit   = 0;

    if (((0 == p_packet) || (0 == p_out)) ||
        ((p_packet->bytes[33] & 0xfcU) != 0U))
    {
        return false;
    }

    value.timestamp_us = read_bits(p_packet->bytes, &bit, 32);

    for (unsigned sensor_index = 0;
         sensor_index < (unsigned int)S3_SENSOR_COUNT; ++sensor_index)
    {
        ir_acquisition_sample_t * p_sample = &value.sensor[sensor_index];
        p_sample->timestamp_us = read_bits(p_packet->bytes, &bit, 32);
        p_sample->raw          = (uint16_t)read_bits(p_packet->bytes, &bit, 12);
        const uint32_t error   = read_bits(p_packet->bytes, &bit, 32);
        p_sample->error        = error <= (unsigned int)INT32_MAX
                                     ? (int32_t)error
                                     : -1 - (int32_t)(UINT32_MAX - error);
        p_sample->b_is_enabled = read_bits(p_packet->bytes, &bit, 1) != 0U;
        p_sample->b_is_valid   = read_bits(p_packet->bytes, &bit, 1) != 0U;
    }

    *p_out = value;
    return true;
}

/** @brief Validate bounded navigation fields and commit one complete packet.
 * @param[in] p_value Borrowed coherent navigation output.
 * @param[out] p_out Writable packet. Validation failure leaves it unchanged.
 * @details Validate event masks, state codes, the barcode letter and every
 * sensor's darkness and surface before encoding. Write the timestamp, sensor,
 * line, barcode, steering and capture fields in their fixed order. Copy
 * float and double representations without quantization. A zero-initialized
 * local packet supplies the reserved bits and is published only on success.
 * @return Whether validation and packing succeeded.
 * @see navigation_codec.h for field domains, float precision and ownership.
 */
bool navigation_codec_pack (const line_navigation_output_t * p_value,
                            navigation_codec_packet_t *      p_out)
{
    navigation_codec_packet_t copy = {{0}};
    unsigned                  bit  = 0;

    if (((((((((((((0 == p_value) || (0 == p_out)) ||
                  ((p_value->events & ~31U) != 0U)) ||
                 (p_value->fault_mask > 7U)) ||
                ((unsigned)p_value->barcode_status >
                 (unsigned int)S3_BARCODE_OVERFLOW)) ||
               ((unsigned)p_value->action >=
                (unsigned int)S3_NAV_ACTION_COUNT)) ||
              (p_value->line.error < -100)) ||
             (p_value->line.error > 100)) ||
            (!letter_valid(p_value->barcode.letter))) ||
           ((unsigned)p_value->lifecycle >
            (unsigned int)S3_RUN_FAULT_LATCHED)) ||
          (p_value->readiness_blockers > 31U)) ||
         (p_value->latched_faults > 15U)) ||
        (p_value->accepted_calibration > 7U))
    {
        return false;
    }

    for (unsigned sensor_index = 0;
         sensor_index < (unsigned int)S3_SENSOR_COUNT; ++sensor_index)
    {
        if ((p_value->sensor[sensor_index].darkness > 1000U) ||
            ((unsigned)p_value->sensor[sensor_index].surface >
             (unsigned int)IR_SURFACE_BLACK))
        {
            return false;
        }
    }

    write_bits(copy.bytes, &bit, p_value->timestamp_us, 32);
    write_bits(copy.bytes, &bit, p_value->sample_gap_us, 32);
    write_bits(copy.bytes, &bit, p_value->events, 5);
    write_bits(copy.bytes, &bit, p_value->fault_mask, 3);

    for (unsigned sensor_index = 0;
         sensor_index < (unsigned int)S3_SENSOR_COUNT; ++sensor_index)
    {
        write_bits(copy.bytes, &bit, p_value->sensor[sensor_index].b_is_valid,
                   1);
        write_bits(copy.bytes, &bit, p_value->sensor[sensor_index].darkness,
                   10);
        write_bits(copy.bytes, &bit, p_value->sensor[sensor_index].surface, 2);
    }

    write_bits(copy.bytes, &bit, p_value->line.b_is_valid, 1);
    write_bits(copy.bytes, &bit, p_value->line.b_is_line_present, 1);
    write_bits(copy.bytes, &bit, p_value->line.b_has_junction_candidate, 1);
    /* The validated error is in -100..100. Bias it to 0..200 before
     * conversion to unsigned for the eight-bit field. */
    write_bits(copy.bytes, &bit, (unsigned)(p_value->line.error + 100), 8);
    write_bits(copy.bytes, &bit, p_value->line.b_has_lost_event, 1);
    write_bits(copy.bytes, &bit, p_value->line.b_has_reacquired_event, 1);
    write_bits(copy.bytes, &bit, p_value->line.b_has_junction_event, 1);
    write_bits(copy.bytes, &bit, p_value->steering.b_is_valid, 1);
    write_bits(copy.bytes, &bit, float_bits(p_value->steering.left_mps), 32);
    write_bits(copy.bytes, &bit, float_bits(p_value->steering.right_mps), 32);
    write_bits(copy.bytes, &bit, p_value->barcode_status, 3);
    write_barcode(copy.bytes, &bit, &p_value->barcode);
    write_bits(copy.bytes, &bit, p_value->action, 3);
    write_bits(copy.bytes, &bit, p_value->steering_preview.b_is_valid, 1);
    write_bits(copy.bytes, &bit, float_bits(p_value->steering_preview.left_mps),
               32);
    write_bits(copy.bytes, &bit,
               float_bits(p_value->steering_preview.right_mps), 32);
    write_bits(copy.bytes, &bit, p_value->lifecycle, 2);
    write_bits(copy.bytes, &bit, p_value->readiness_blockers, 5);
    write_bits(copy.bytes, &bit, p_value->latched_faults, 4);
    write_bits(copy.bytes, &bit, p_value->accepted_calibration, 3);
    write_bits(copy.bytes, &bit, p_value->b_is_stop_requested, 1);
    write_bits(copy.bytes, &bit, p_value->b_has_odometry, 1);
    write_bits(copy.bytes, &bit, p_value->b_is_rearming, 1);
    write_bits(copy.bytes, &bit, p_value->b_is_diagnostic, 1);
    write_double(copy.bytes, &bit, p_value->capture_distance.start_m);
    write_double(copy.bytes, &bit, p_value->capture_distance.end_m);
    write_bits(copy.bytes, &bit, p_value->capture_distance.epoch, 32);
    write_bits(copy.bytes, &bit, p_value->capture_distance.b_is_start_valid, 1);
    write_bits(copy.bytes, &bit, p_value->capture_distance.b_is_end_valid, 1);
    *p_out = copy;
    return true;
}

/** @brief Validate navigation codes into a local result before publishing it.
 * @details Read the sensor fields first, then line state, steering, barcode
 * fields and capture metadata in wire order. The reserved bit and bounded
 * fields are checked before the final copy. Ten-bit darkness can represent
 * values beyond 1000, so check its full raw range after the uint16_t cast.
 * Line error decodes from 0..200 to -100..100 inside int16_t.
 * @see navigation_codec.h for padding, output mutation and diagnostic values.
 */
bool navigation_codec_unpack (const navigation_codec_packet_t * p_packet,
                              line_navigation_output_t *        p_out)
{
    line_navigation_output_t value = {0};
    unsigned                 bit   = 0;

    if (((0 == p_packet) || (0 == p_out)) ||
        ((p_packet->bytes[59] & 0x80U) != 0U))
    {
        return false;
    }

    value.timestamp_us  = read_bits(p_packet->bytes, &bit, 32);
    value.sample_gap_us = read_bits(p_packet->bytes, &bit, 32);
    value.events        = read_bits(p_packet->bytes, &bit, 5);
    value.fault_mask    = (uint8_t)read_bits(p_packet->bytes, &bit, 3);

    for (unsigned sensor_index = 0;
         sensor_index < (unsigned int)S3_SENSOR_COUNT; ++sensor_index)
    {
        /* clang-format off */
        value.sensor[sensor_index].b_is_valid =
            read_bits(p_packet->bytes, &bit, 1) != 0U;
        value.sensor[sensor_index].darkness   =
            (uint16_t)read_bits(p_packet->bytes, &bit, 10);
        value.sensor[sensor_index].surface    =
            (ir_reflectance_surface_t)read_bits(p_packet->bytes, &bit, 2);
        /* clang-format on */

        if ((value.sensor[sensor_index].darkness > 1000U) ||
            (value.sensor[sensor_index].surface >
             (unsigned int)IR_SURFACE_BLACK))
        {
            return false;
        }
    }

    /* clang-format off */
    value.line.b_is_valid               =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.line.b_is_line_present        =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.line.b_has_junction_candidate =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    /* clang-format on */
    const unsigned error = read_bits(p_packet->bytes, &bit, 8);

    if (error > 200U)
    {
        return false;
    }

    /* clang-format off */
    value.line.error                  = (int16_t)((int)error - 100);
    value.line.b_has_lost_event       =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.line.b_has_reacquired_event =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.line.b_has_junction_event   =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.steering.b_is_valid         =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.steering.left_mps           =
        bits_float(read_bits(p_packet->bytes, &bit, 32));
    value.steering.right_mps          =
        bits_float(read_bits(p_packet->bytes, &bit, 32));
    /* clang-format on */

    value.barcode_status =
        (bc_decoder_status_t)read_bits(p_packet->bytes, &bit, 3);

    if ((value.barcode_status > (unsigned int)S3_BARCODE_OVERFLOW) ||
        (!read_barcode(p_packet->bytes, &bit, &value.barcode)))
    {
        return false;
    }

    value.action =
        (line_navigation_action_t)read_bits(p_packet->bytes, &bit, 3);

    if (value.action >= (unsigned int)S3_NAV_ACTION_COUNT)
    {
        return false;
    }

    /* clang-format off */
    value.steering_preview.b_is_valid =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.steering_preview.left_mps   =
        bits_float(read_bits(p_packet->bytes, &bit, 32));
    value.steering_preview.right_mps  =
        bits_float(read_bits(p_packet->bytes, &bit, 32));
    value.lifecycle                   =
        (line_navigation_run_state_t)read_bits(p_packet->bytes, &bit, 2);
    /* clang-format on */

    if (value.lifecycle > (unsigned int)S3_RUN_FAULT_LATCHED)
    {
        return false;
    }

    /* clang-format off */
    value.readiness_blockers                =
        read_bits(p_packet->bytes, &bit, 5);
    value.latched_faults                    =
        read_bits(p_packet->bytes, &bit, 4);
    value.accepted_calibration              =
        (uint8_t)read_bits(p_packet->bytes, &bit, 3);
    value.b_is_stop_requested               =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.b_has_odometry                    =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.b_is_rearming                     =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.b_is_diagnostic                   =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.capture_distance.start_m          =
        read_double(p_packet->bytes, &bit);
    value.capture_distance.end_m            =
        read_double(p_packet->bytes, &bit);
    value.capture_distance.epoch            =
        read_bits(p_packet->bytes, &bit, 32);
    value.capture_distance.b_is_start_valid =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    value.capture_distance.b_is_end_valid   =
        read_bits(p_packet->bytes, &bit, 1) != 0U;
    *p_out                                  = value;
    /* clang-format on */
    return true;
}

/** @brief OR one low-bit-first field into a zero-initialized packet region.
 * @param[in,out] p_bytes Writable storage covering the complete field.
 * @param[in,out] p_bit Non-null cursor; advanced by width bits.
 * @param[in] value Bits to write; only its lowest width bits are examined.
 * @param[in] width Field width, 0..32 bits.
 * @pre Pointers and capacity are valid and destination field bits are zero.
 * @post Set value bits are ORed into the packet; no bits are cleared.
 */
static void write_bits (uint8_t * p_bytes, unsigned * p_bit, uint32_t value,
                        unsigned width)
{
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        if ((value & ((uint32_t)1U << bit_offset)) != 0U)
        {
            p_bytes[*p_bit / 8U] = (uint8_t)((unsigned)p_bytes[*p_bit / 8U] |
                                             (1U << (*p_bit % 8U)));
        }

        ++bit_offset;
        ++*p_bit;
    }
}

/** @brief Read one unsigned field from the packet's low-bit-first layout.
 * @param[in] p_bytes Stable readable storage covering the complete field.
 * @param[in,out] p_bit Non-null cursor; advanced by width bits.
 * @param[in] width Field width, 0..32 bits; zero reads no bytes.
 * @return The field value, or zero for width zero.
 * @pre Pointers and capacity are valid; no bounds or null checks are made.
 */
static uint32_t read_bits (const uint8_t * p_bytes, unsigned * p_bit,
                           unsigned width)
{
    uint32_t value      = 0;
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        if ((p_bytes[*p_bit / 8U] & (1U << (*p_bit % 8U))) != 0U)
        {
            value |= (uint32_t)1U << bit_offset;
        }

        ++bit_offset;
        ++*p_bit;
    }

    return value;
}

/** @brief Obtain the binary32 representation used by steering packets.
 * @param[in] value Steering value in m/s, including diagnostic nonfinite data.
 * @return The union-reinterpreted 32 bits, without numerical conversion.
 * @details No arithmetic, rounding, finite-value check or state mutation
 * occurs.
 */
static uint32_t float_bits (float32_t value)
{
    app_types_binary32_view_t copy;

    copy.value = value;
    return copy.bits;
}

/** @brief Reconstruct a binary32 steering value from its packet bits.
 * @param[in] value Unmodified 32-bit field read from the packet.
 * @return The union-reinterpreted float, including signed zero or nonfinite
 * representations; no scaling or validity check is performed.
 */
static float32_t bits_float (uint32_t value)
{
    app_types_binary32_view_t copy;

    copy.bits = value;
    return copy.value;
}

/** @brief Test the supported barcode alphabet and absent-letter sentinel.
 * @param[in] letter Decoded character to inspect.
 * @return True for NUL or A..Z; false for every other character.
 */
static bool letter_valid (char letter)
{
    return (0U == letter) || (((int)letter >= 'A') && ((int)letter <= 'Z'));
}

/** @brief Append a binary64 distance as its low word then high word.
 * @param[in,out] p_bytes Writable packet with 64 zero destination bits.
 * @param[in,out] p_bit Non-null cursor; advanced by 64 bits.
 * @param[in] value Distance in metres; nonfinite values are not rejected.
 * @pre Storage covers the field. No null, capacity or numeric check is made.
 * @post The representation is copied without scaling or rounding.
 */
static void write_double (uint8_t * p_bytes, unsigned * p_bit, float64_t value)
{
    app_types_binary64_view_t copy;

    copy.value = value;
    write_bits(p_bytes, p_bit, (uint32_t)copy.bits, 32);
    write_bits(p_bytes, p_bit, (uint32_t)(copy.bits >> 32), 32);
}

/** @brief Reconstruct a binary64 distance from two packet words.
 * @param[in] p_bytes Stable readable packet containing the next 64 bits.
 * @param[in,out] p_bit Non-null cursor; advanced by 64 bits.
 * @return The represented distance in metres, without finite-value checking.
 * @pre Pointers and capacity are valid; low-word-first storage is assumed.
 */
static float64_t read_double (const uint8_t * p_bytes, unsigned * p_bit)
{

    const uint32_t            low  = read_bits(p_bytes, p_bit, 32);
    const uint32_t            high = read_bits(p_bytes, p_bit, 32);
    app_types_binary64_view_t copy;
    copy.bits = (uint64_t)low | ((uint64_t)high << 32);
    return copy.value;
}

/** @brief Append a previously validated barcode, direction and timestamp.
 * @param[in,out] p_bytes Writable packet with 38 zero destination bits.
 * @param[in,out] p_bit Non-null cursor; advanced by 38 bits.
 * @param[in] p_value Stable result with letter NUL or A..Z.
 * @pre Pointers, capacity and letter validity were checked by the caller.
 * @post Letter code, reversed bit and microsecond timestamp are appended.
 */
static void write_barcode (uint8_t * p_bytes, unsigned * p_bit,
                           const bc_decoder_result_t * p_value)
{
    write_bits(p_bytes, p_bit,
               0U == p_value->letter
                   ? 0U
                   : (unsigned)((int)p_value->letter - 'A') + 1U,
               5);
    write_bits(p_bytes, p_bit, p_value->b_is_reversed, 1);
    write_bits(p_bytes, p_bit, p_value->timestamp_us, 32);
}

/** @brief Read a barcode field while rejecting unused letter codes.
 * @param[in] p_bytes Stable packet containing at least the next 38 bits.
 * @param[in,out] p_bit Non-null cursor; advances five bits on an invalid code
 * and 38 bits on success.
 * @param[out] p_value Writable result; unchanged when the code is invalid.
 * @return False for letter codes 27..31; true after decoding codes 0..26,
 * the direction bit and unchanged microsecond timestamp.
 * @pre Pointers and capacity are valid. Padding is checked by the caller.
 */
static bool read_barcode (const uint8_t * p_bytes, unsigned * p_bit,
                          bc_decoder_result_t * p_value)
{
    const unsigned code = read_bits(p_bytes, p_bit, 5);

    if (code > 26U)
    {
        return false;
    }

    /* clang-format off */
    p_value->letter        =
        0U == code ? (char)0 : (char)((unsigned int)'A' + code - 1U);
    p_value->b_is_reversed = read_bits(p_bytes, p_bit, 1) != 0U;
    p_value->timestamp_us  = read_bits(p_bytes, p_bit, 32);
    /* clang-format on */
    return true;
}

/* End of file. */

