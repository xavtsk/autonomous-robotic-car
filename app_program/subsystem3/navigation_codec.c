/** @file navigation_codec.c
 * @brief Pack and unpack navigation, barcode and raw-frame snapshots.
 */
#include "navigation_codec.h"
#include "../build_assert.h"
#include "../app_types.h"
#include <float.h>

/* Compile-time guarantee: shared steering requires the existing binary32 float
 * format. */
BUILD_ASSERT(sizeof(float32_t) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
             navigation_codec_c_check_a_t);
/* Compile-time guarantee: navigation packet layout. */
BUILD_ASSERT(sizeof(s3_navigation_packet_t) == 60,
             navigation_codec_c_check_b_t);
/* Compile-time guarantee: odometry binary64. */
BUILD_ASSERT(sizeof(float64_t) == 8 && DBL_MANT_DIG == 53,
             navigation_codec_c_check_c_t);
/* Compile-time guarantee: barcode packet layout. */
BUILD_ASSERT(sizeof(s3_barcode_packet_t) == 5, navigation_codec_c_check_d_t);
/* Compile-time guarantee: raw-frame packet layout changed. */
BUILD_ASSERT(sizeof(s3_raw_frame_packet_t) == 34 && S3_SENSOR_COUNT == 3,
             navigation_codec_c_check_e_t);
/* Compile-time guarantee: event/sensor mask widths changed. */
BUILD_ASSERT(S3_EVENT_SENSOR_FAULT == 16 && S3_ALL_SENSORS == 7,
             navigation_codec_c_check_f_t);

/** @brief OR one low-bit-first field into a zero-initialized packet region.
 * @param[in,out] p_bytes Writable storage covering the complete field.
 * @param[in,out] p_bit Non-null cursor; advanced by width bits.
 * @param[in] value Bits to write; only its lowest width bits are examined.
 * @param[in] width Field width, 0..32 bits.
 * @pre Pointers and capacity are valid and destination field bits are zero.
 * @post Set value bits are ORed into the packet; no bits are cleared.
 */
static void put (uint8_t * p_bytes, unsigned * p_bit, uint32_t value,
                 unsigned width)
{
    for (unsigned value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        if ((value & ((uint32_t)1U << value_i)) != 0U)
        {
            p_bytes[*p_bit / 8U] |= (uint8_t)(1U << (*p_bit % 8U));
        }
    }
}

/** @brief Read one unsigned field from the packet's low-bit-first layout.
 * @param[in] p_bytes Stable readable storage covering the complete field.
 * @param[in,out] p_bit Non-null cursor; advanced by width bits.
 * @param[in] width Field width, 0..32 bits; zero reads no bytes.
 * @return The field value, or zero for width zero.
 * @pre Pointers and capacity are valid; no bounds or null checks are made.
 */
static uint32_t get (const uint8_t * p_bytes, unsigned * p_bit, unsigned width)
{
    uint32_t value = 0;
    for (unsigned value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        if ((p_bytes[*p_bit / 8U] & (1U << (*p_bit % 8U))) != 0U)
        {
            value |= (uint32_t)1U << value_i;
        }
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
    union
    {
        float32_t f;
        uint32_t bits;
    } copy;

    copy.f = value;
    return copy.bits;
}

/** @brief Reconstruct a binary32 steering value from its packet bits.
 * @param[in] value Unmodified 32-bit field read from the packet.
 * @return The union-reinterpreted float, including signed zero or nonfinite
 * representations; no scaling or validity check is performed.
 */
static float32_t bits_float (uint32_t value)
{
    union
    {
        float32_t f;
        uint32_t bits;
    } copy;

    copy.bits = value;
    return copy.f;
}

/** @brief Test the supported barcode alphabet and absent-letter sentinel.
 * @param[in] value_c Decoded character to inspect.
 * @return True for NUL or A..Z; false for every other character.
 */
static bool letter_valid (char value_c)
{
    return value_c == 0 || (value_c >= 'A' && value_c <= 'Z');
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
    union
    {
        float64_t value;
        uint64_t bits;
    } copy;

    copy.value = value;
    put(p_bytes, p_bit, (uint32_t)copy.bits, 32);
    put(p_bytes, p_bit, (uint32_t)(copy.bits >> 32), 32);
}

/** @brief Reconstruct a binary64 distance from two packet words.
 * @param[in] p_bytes Stable readable packet containing the next 64 bits.
 * @param[in,out] p_bit Non-null cursor; advanced by 64 bits.
 * @return The represented distance in metres, without finite-value checking.
 * @pre Pointers and capacity are valid; low-word-first storage is assumed.
 */
static float64_t read_double (const uint8_t * p_bytes, unsigned * p_bit)
{
    union
    {
        float64_t value;
        uint64_t bits;
    } copy;

    uint32_t low = get(p_bytes, p_bit, 32), high = get(p_bytes, p_bit, 32);
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
                           const s3_barcode_result_t * p_value)
{
    put(p_bytes, p_bit,
        p_value->letter == 0 ? 0U : (unsigned)(p_value->letter - 'A') + 1U, 5);
    put(p_bytes, p_bit, p_value->reversed, 1);
    put(p_bytes, p_bit, p_value->timestamp_us, 32);
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
                          s3_barcode_result_t * p_value)
{
    unsigned code = get(p_bytes, p_bit, 5);
    if (code > 26U)
    {
        return false;
    }
    p_value->letter = code == 0U ? 0 : (char)('A' + code - 1U);
    p_value->reversed = get(p_bytes, p_bit, 1) != 0U;
    p_value->timestamp_us = get(p_bytes, p_bit, 32);
    return true;
}

/** @brief Build a validated barcode packet locally before publishing it.
 * @see navigation_codec.h for inputs, bit layout and unchanged-on-failure
 * output.
 */
bool s3_barcode_pack (const s3_barcode_result_t * p_value,
                      s3_barcode_packet_t * p_out)
{
    s3_barcode_packet_t copy = {{0}};
    unsigned bit = 0;
    if (!p_value || !p_out || !letter_valid(p_value->letter))
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
bool s3_barcode_unpack (const s3_barcode_packet_t * p_packet,
                        s3_barcode_result_t * p_out)
{
    s3_barcode_result_t value = {0};
    unsigned bit = 0;
    if (!p_packet || !p_out || (p_packet->bytes[4] & 0xc0U) != 0U ||
        !read_barcode(p_packet->bytes, &bit, &value))
    {
        return false;
    }
    *p_out = value;
    return true;
}

/** @brief Validate all raw readings before publishing a complete frame packet.
 * @see navigation_codec.h for raw/error preservation and output ownership.
 */
bool s3_raw_frame_pack (const s3_sensor_frame_t * p_value,
                        s3_raw_frame_packet_t * p_out)
{
    s3_raw_frame_packet_t copy = {{0}};
    unsigned bit = 0;
    if (!p_value || !p_out)
    {
        return false;
    }
    for (unsigned value_i = 0; value_i < S3_SENSOR_COUNT; ++value_i)
    {
        if (p_value->sensor[value_i].raw > 4095U)
        {
            return false;
        }
    }
    put(copy.bytes, &bit, p_value->timestamp_us, 32);
    for (unsigned value_i = 0; value_i < S3_SENSOR_COUNT; ++value_i)
    {
        const s3_raw_sample_t * p_sample = &p_value->sensor[value_i];
        put(copy.bytes, &bit, p_sample->timestamp_us, 32);
        put(copy.bytes, &bit, p_sample->raw, 12);
        put(copy.bytes, &bit, (uint32_t)p_sample->error, 32);
        put(copy.bytes, &bit, p_sample->enabled, 1);
        put(copy.bytes, &bit, p_sample->valid, 1);
    }
    *p_out = copy;
    return true;
}

/** @brief Reconstruct all sample fields after validating frame padding.
 * @see navigation_codec.h for accepted error/timestamp values and
 * unchanged-on-failure output.
 */
bool s3_raw_frame_unpack (const s3_raw_frame_packet_t * p_packet,
                          s3_sensor_frame_t * p_out)
{
    s3_sensor_frame_t value = {0};
    unsigned bit = 0;
    if (!p_packet || !p_out || (p_packet->bytes[33] & 0xfcU) != 0U)
    {
        return false;
    }
    value.timestamp_us = get(p_packet->bytes, &bit, 32);
    for (unsigned value_i = 0; value_i < S3_SENSOR_COUNT; ++value_i)
    {
        s3_raw_sample_t * p_sample = &value.sensor[value_i];
        uint32_t error;
        p_sample->timestamp_us = get(p_packet->bytes, &bit, 32);
        p_sample->raw = (uint16_t)get(p_packet->bytes, &bit, 12);
        error = get(p_packet->bytes, &bit, 32);
        p_sample->error = error <= INT32_MAX
                              ? (int32_t)error
                              : -1 - (int32_t)(UINT32_MAX - error);
        p_sample->enabled = get(p_packet->bytes, &bit, 1) != 0U;
        p_sample->valid = get(p_packet->bytes, &bit, 1) != 0U;
    }
    *p_out = value;
    return true;
}

/** @brief Validate bounded navigation fields and commit one complete packet.
 * @see navigation_codec.h for field domains, float precision and ownership.
 */
bool s3_navigation_pack (const s3_navigation_output_t * p_value,
                         s3_navigation_packet_t * p_out)
{
    s3_navigation_packet_t copy = {{0}};
    unsigned bit = 0;
    if (!p_value || !p_out || (p_value->events & ~31U) != 0U ||
        p_value->fault_mask > 7U ||
        (unsigned)p_value->barcode_status > S3_BARCODE_OVERFLOW ||
        (unsigned)p_value->action >= S3_NAV_ACTION_COUNT ||
        p_value->line.error < -100 || p_value->line.error > 100 ||
        !letter_valid(p_value->barcode.letter) ||
        (unsigned)p_value->lifecycle > S3_RUN_FAULT_LATCHED ||
        p_value->readiness_blockers > 31U || p_value->latched_faults > 15U ||
        p_value->accepted_calibration > 7U)
    {
        return false;
    }
    for (unsigned value_i = 0; value_i < S3_SENSOR_COUNT; ++value_i)
    {
        if (p_value->sensor[value_i].darkness > 1000U ||
            (unsigned)p_value->sensor[value_i].surface > IR_SURFACE_BLACK)
        {
            return false;
        }
    }
    put(copy.bytes, &bit, p_value->timestamp_us, 32);
    put(copy.bytes, &bit, p_value->sample_gap_us, 32);
    put(copy.bytes, &bit, p_value->events, 5);
    put(copy.bytes, &bit, p_value->fault_mask, 3);
    for (unsigned value_i = 0; value_i < S3_SENSOR_COUNT; ++value_i)
    {
        put(copy.bytes, &bit, p_value->sensor[value_i].valid, 1);
        put(copy.bytes, &bit, p_value->sensor[value_i].darkness, 10);
        put(copy.bytes, &bit, p_value->sensor[value_i].surface, 2);
    }
    put(copy.bytes, &bit, p_value->line.valid, 1);
    put(copy.bytes, &bit, p_value->line.present, 1);
    put(copy.bytes, &bit, p_value->line.junction_candidate, 1);
    put(copy.bytes, &bit, (unsigned)(p_value->line.error + 100), 8);
    put(copy.bytes, &bit, p_value->line.lost_event, 1);
    put(copy.bytes, &bit, p_value->line.reacquired_event, 1);
    put(copy.bytes, &bit, p_value->line.junction_event, 1);
    put(copy.bytes, &bit, p_value->steering.valid, 1);
    put(copy.bytes, &bit, float_bits(p_value->steering.left_mps), 32);
    put(copy.bytes, &bit, float_bits(p_value->steering.right_mps), 32);
    put(copy.bytes, &bit, p_value->barcode_status, 3);
    write_barcode(copy.bytes, &bit, &p_value->barcode);
    put(copy.bytes, &bit, p_value->action, 3);
    put(copy.bytes, &bit, p_value->steering_preview.valid, 1);
    put(copy.bytes, &bit, float_bits(p_value->steering_preview.left_mps), 32);
    put(copy.bytes, &bit, float_bits(p_value->steering_preview.right_mps), 32);
    put(copy.bytes, &bit, p_value->lifecycle, 2);
    put(copy.bytes, &bit, p_value->readiness_blockers, 5);
    put(copy.bytes, &bit, p_value->latched_faults, 4);
    put(copy.bytes, &bit, p_value->accepted_calibration, 3);
    put(copy.bytes, &bit, p_value->stop_requested, 1);
    put(copy.bytes, &bit, p_value->odometry_available, 1);
    put(copy.bytes, &bit, p_value->rearming, 1);
    put(copy.bytes, &bit, p_value->diagnostic, 1);
    write_double(copy.bytes, &bit, p_value->capture_distance.start_m);
    write_double(copy.bytes, &bit, p_value->capture_distance.end_m);
    put(copy.bytes, &bit, p_value->capture_distance.epoch, 32);
    put(copy.bytes, &bit, p_value->capture_distance.start_valid, 1);
    put(copy.bytes, &bit, p_value->capture_distance.end_valid, 1);
    *p_out = copy;
    return true;
}

/** @brief Validate navigation codes into a local result before publishing it.
 * @see navigation_codec.h for padding, output mutation and diagnostic values.
 */
bool s3_navigation_unpack (const s3_navigation_packet_t * p_packet,
                           s3_navigation_output_t * p_out)
{
    s3_navigation_output_t value = {0};
    unsigned bit = 0, error;
    if (!p_packet || !p_out || (p_packet->bytes[59] & 0x80U) != 0U)
    {
        return false;
    }
    value.timestamp_us = get(p_packet->bytes, &bit, 32);
    value.sample_gap_us = get(p_packet->bytes, &bit, 32);
    value.events = get(p_packet->bytes, &bit, 5);
    value.fault_mask = (uint8_t)get(p_packet->bytes, &bit, 3);
    for (unsigned value_i = 0; value_i < S3_SENSOR_COUNT; ++value_i)
    {
        value.sensor[value_i].valid = get(p_packet->bytes, &bit, 1) != 0U;
        value.sensor[value_i].darkness =
            (uint16_t)get(p_packet->bytes, &bit, 10);
        value.sensor[value_i].surface =
            (ir_surface_t)get(p_packet->bytes, &bit, 2);
        if (value.sensor[value_i].darkness > 1000U ||
            value.sensor[value_i].surface > IR_SURFACE_BLACK)
        {
            return false;
        }
    }
    value.line.valid = get(p_packet->bytes, &bit, 1) != 0U;
    value.line.present = get(p_packet->bytes, &bit, 1) != 0U;
    value.line.junction_candidate = get(p_packet->bytes, &bit, 1) != 0U;
    error = get(p_packet->bytes, &bit, 8);
    if (error > 200U)
    {
        return false;
    }
    value.line.error = (int16_t)((int)error - 100);
    value.line.lost_event = get(p_packet->bytes, &bit, 1) != 0U;
    value.line.reacquired_event = get(p_packet->bytes, &bit, 1) != 0U;
    value.line.junction_event = get(p_packet->bytes, &bit, 1) != 0U;
    value.steering.valid = get(p_packet->bytes, &bit, 1) != 0U;
    value.steering.left_mps = bits_float(get(p_packet->bytes, &bit, 32));
    value.steering.right_mps = bits_float(get(p_packet->bytes, &bit, 32));
    value.barcode_status = (s3_barcode_status_t)get(p_packet->bytes, &bit, 3);
    if (value.barcode_status > S3_BARCODE_OVERFLOW ||
        !read_barcode(p_packet->bytes, &bit, &value.barcode))
    {
        return false;
    }
    value.action = (s3_nav_action_t)get(p_packet->bytes, &bit, 3);
    if (value.action >= S3_NAV_ACTION_COUNT)
    {
        return false;
    }
    value.steering_preview.valid = get(p_packet->bytes, &bit, 1) != 0U;
    value.steering_preview.left_mps =
        bits_float(get(p_packet->bytes, &bit, 32));
    value.steering_preview.right_mps =
        bits_float(get(p_packet->bytes, &bit, 32));
    value.lifecycle = (s3_run_state_t)get(p_packet->bytes, &bit, 2);
    if (value.lifecycle > S3_RUN_FAULT_LATCHED)
    {
        return false;
    }
    value.readiness_blockers = get(p_packet->bytes, &bit, 5);
    value.latched_faults = get(p_packet->bytes, &bit, 4);
    value.accepted_calibration = (uint8_t)get(p_packet->bytes, &bit, 3);
    value.stop_requested = get(p_packet->bytes, &bit, 1) != 0U;
    value.odometry_available = get(p_packet->bytes, &bit, 1) != 0U;
    value.rearming = get(p_packet->bytes, &bit, 1) != 0U;
    value.diagnostic = get(p_packet->bytes, &bit, 1) != 0U;
    value.capture_distance.start_m = read_double(p_packet->bytes, &bit);
    value.capture_distance.end_m = read_double(p_packet->bytes, &bit);
    value.capture_distance.epoch = get(p_packet->bytes, &bit, 32);
    value.capture_distance.start_valid = get(p_packet->bytes, &bit, 1) != 0U;
    value.capture_distance.end_valid = get(p_packet->bytes, &bit, 1) != 0U;
    *p_out = value;
    return true;
}

/* End of file. */

