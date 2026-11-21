/** @file test_codec_main.c
 * @brief Independent semantic round-trip / malformed-packet checks.
 *
 * No hardware.
 * Unsigned casts spell out the original modulo conversion.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "test_codec_main.h"
#include "../../app_program/app_types.h"
#include "navigation_codec.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define FIRST_INVALID_ADC_CODE 4096U
#define MAX_LINE_ERROR_PERCENT 100
#define MAX_NORMALISED_SURFACE 1000U
#define BITS_PER_PACKET_BYTE 8U

#define ADC_CODE_MAX 4095U
#define FIRST_NAV_PADDING_BIT 266U
#define NAV_STORAGE_BIT_COUNT 272U
#define EVENT_MASK_COMBINATIONS 32
#define FAULT_MASK_COMBINATIONS 8
#define FIXTURE_SENSOR_COUNT 3
#define LETTER_WIRE_CODE_MAX 26
#define SCAN_DIRECTION_COUNT 2
#define PACKET_RANDOM_TRIAL_COUNT 20000U

static uint32_t  bits (float32_t value);
static float32_t number (uint32_t value);
static void      same_barcode (const bc_decoder_result_t * p_result,
                               const bc_decoder_result_t * p_second);
static void      same (const line_navigation_output_t * p_first,
                       const line_navigation_output_t * p_second);
static void      field (uint8_t * p_bytes, unsigned start, unsigned width,
                        uint32_t value);
static void      same_raw (const ir_acquisition_frame_t * p_frame,
                           const ir_acquisition_frame_t * p_second);
static void      raw_frames (void);
static unsigned  combinations (void);
static void      barcodes_and_floats (void);
static void      malformed (void);
static void      known_layout_and_atomic_failure (void);
static void      hostile_bytes (void);
static void      hostile_raw_bytes (void);

/** @brief Exercise accepted and malformed navigation and barcode packets.
 * Check decoded fields, exact float bits and output sizes.
 */
int main (void)
{
    assert(sizeof(navigation_codec_packet_t) == 60U &&
           sizeof(navigation_codec_barcode_t) == 5U);
    assert(sizeof(navigation_codec_raw_frame_t) == 34U);
    unsigned checked = combinations();
    barcodes_and_floats();
    malformed();
    known_layout_and_atomic_failure();
    hostile_bytes();
    raw_frames();
    hostile_raw_bytes();
    printf("PASS: %u error/event/fault combinations, all darkness values, "
           "A-Z/none/direction/actions, exact float bits and malformed packet "
           "rejection\n",
           checked);
    return 0;
}

/** @brief Copy a binary32 representation into an unsigned word for an exact
 * comparison.
 */
static uint32_t bits (float32_t value)
{
    uint32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}

/** @brief Construct a binary32 value from the supplied bit pattern without a
 * numeric conversion.
 */
static float32_t number (uint32_t value)
{
    float32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}

/** @brief Assert equality of decoded letter, orientation and microsecond
 * timestamp.
 */
static void same_barcode (const bc_decoder_result_t * p_result,
                          const bc_decoder_result_t * p_second)
{
    assert(p_result->letter == p_second->letter &&
           p_result->b_is_reversed == p_second->b_is_reversed);
    assert(p_result->timestamp_us == p_second->timestamp_us);
}

/** @brief Assert field agreement between two navigation outputs, including
 * steering, event flags and decoded barcode.
 */
static void same (const line_navigation_output_t * p_first,
                  const line_navigation_output_t * p_second)
{
    assert(p_first->timestamp_us == p_second->timestamp_us &&
           p_first->sample_gap_us == p_second->sample_gap_us);
    assert(p_first->events == p_second->events &&
           p_first->fault_mask == p_second->fault_mask);

    for (unsigned sensor_index = 0; sensor_index < (unsigned)S3_SENSOR_COUNT;
         ++sensor_index)
    {
        assert(p_first->sensor[sensor_index].b_is_valid ==
               p_second->sensor[sensor_index].b_is_valid);
        assert(p_first->sensor[sensor_index].darkness ==
               p_second->sensor[sensor_index].darkness);
        assert(p_first->sensor[sensor_index].surface ==
               p_second->sensor[sensor_index].surface);
    }

    assert(p_first->line.b_is_valid == p_second->line.b_is_valid &&
           p_first->line.b_is_line_present == p_second->line.b_is_line_present);
    assert(p_first->line.b_has_junction_candidate ==
           p_second->line.b_has_junction_candidate);
    assert(p_first->line.error == p_second->line.error);
    assert(p_first->line.b_has_lost_event == p_second->line.b_has_lost_event);
    assert(p_first->line.b_has_reacquired_event ==
           p_second->line.b_has_reacquired_event);
    assert(p_first->line.b_has_junction_event ==
           p_second->line.b_has_junction_event);
    assert(p_first->steering.b_is_valid == p_second->steering.b_is_valid);
    assert(bits(p_first->steering.left_mps) ==
           bits(p_second->steering.left_mps));
    assert(bits(p_first->steering.right_mps) ==
           bits(p_second->steering.right_mps));
    assert(p_first->barcode_status == p_second->barcode_status &&
           p_first->action == p_second->action);
    same_barcode(&p_first->barcode, &p_second->barcode);
}

/** @brief Set an LSB-first field independently to inject a bit pattern that
 * the encoder would reject.
 */
static void field (uint8_t * p_bytes, unsigned start, unsigned width,
                   uint32_t value)
{
    for (unsigned bit_offset = 0; bit_offset < width; ++bit_offset)
    {
        unsigned bit = start + bit_offset;
        p_bytes[bit / 8U] =
            (uint8_t)((p_bytes[bit / 8U] & ~(1U << (bit % 8U))) |
                      (((value >> bit_offset) & 1U) << (bit % 8U)));
    }
}

/** @brief Assert equality of the frame timestamp and every sensor's
 * timestamp, raw reading, error, enable and validity flags.
 */
static void same_raw (const ir_acquisition_frame_t * p_frame,
                      const ir_acquisition_frame_t * p_second)
{
    assert(p_frame->timestamp_us == p_second->timestamp_us);

    for (unsigned sensor_index = 0; sensor_index < (unsigned)S3_SENSOR_COUNT;
         ++sensor_index)
    {
        assert(p_frame->sensor[sensor_index].timestamp_us ==
               p_second->sensor[sensor_index].timestamp_us);
        assert(p_frame->sensor[sensor_index].raw ==
               p_second->sensor[sensor_index].raw);
        assert(p_frame->sensor[sensor_index].error ==
               p_second->sensor[sensor_index].error);
        assert(p_frame->sensor[sensor_index].b_is_enabled ==
               p_second->sensor[sensor_index].b_is_enabled);
        assert(p_frame->sensor[sensor_index].b_is_valid ==
               p_second->sensor[sensor_index].b_is_valid);
    }
}

/** @brief Verify the 266-bit raw-frame layout, round trips and preservation
 * of the output on invalid input.
 */
static void raw_frames (void)
{
    ir_acquisition_frame_t       input = {0};
    ir_acquisition_frame_t       output;
    navigation_codec_raw_frame_t packet;
    navigation_codec_raw_frame_t saved;
    navigation_codec_raw_frame_t expected = {{0}};

    const int32_t errors[] = {0, -1, INT32_MIN, INT32_MAX, -1234567, 1234567};
    input.timestamp_us     = 0xfedcba98U;
    /* Every ADC value remains exact, including disabled/invalid diagnostic
     * samples. Flags are information, not permission to discard fields. */
    for (unsigned raw = 0; raw <= ADC_CODE_MAX; ++raw)
    {
        for (unsigned sensor_index = 0;
             sensor_index < (unsigned)S3_SENSOR_COUNT; ++sensor_index)
        {
            /* clang-format off */
            input.sensor[sensor_index].raw          =
                (uint16_t)((raw + (sensor_index * 997U)) & 4095U);
            input.sensor[sensor_index].timestamp_us =
                raw ^ (UINT32_MAX - sensor_index);
            input.sensor[sensor_index].error        =
                errors[(raw + sensor_index) % 6U];
            input.sensor[sensor_index].b_is_enabled =
                ((raw >> sensor_index) & 1U) != 0U;
            input.sensor[sensor_index].b_is_valid   =
                ((raw >> (sensor_index + 3U)) & 1U) != 0U;
            /* clang-format on */
        }

        assert(navigation_codec_raw_pack(&input, &packet));
        assert((packet.bytes[33] & 0xfcU) == 0U);
        assert(navigation_codec_raw_unpack(&packet, &output));
        same_raw(&input, &output);
    }
    /* Set contrasting values across byte boundaries and construct the wire
     * layout independently of production get/put helpers. */
    input.sensor[0] =
        (ir_acquisition_sample_t){0x01020304U, 4095, INT32_MIN, true, false};
    input.sensor[1] =
        (ir_acquisition_sample_t){UINT32_MAX, 0, INT32_MAX, false, true};
    input.sensor[2] = (ir_acquisition_sample_t){0, 0xabc, -1234567, true, true};
    field(expected.bytes, 0, 32, input.timestamp_us);

    for (unsigned sensor_index = 0; sensor_index < (unsigned)S3_SENSOR_COUNT;
         ++sensor_index)
    {
        unsigned start = 32U + (sensor_index * 78U);
        field(expected.bytes, start, 32,
              input.sensor[sensor_index].timestamp_us);
        field(expected.bytes, start + 32U, 12, input.sensor[sensor_index].raw);
        field(expected.bytes, start + 44U, 32,
              (uint32_t)input.sensor[sensor_index].error);
        field(expected.bytes, start + 76U, 1,
              input.sensor[sensor_index].b_is_enabled);
        field(expected.bytes, start + 77U, 1,
              input.sensor[sensor_index].b_is_valid);
    }

    assert(navigation_codec_raw_pack(&input, &packet));
    assert(memcmp(packet.bytes, expected.bytes, sizeof(packet.bytes)) == 0);
    assert(navigation_codec_raw_unpack(&expected, &output));
    same_raw(&input, &output);
    saved = packet;

    for (unsigned sensor_index = 0; sensor_index < (unsigned)S3_SENSOR_COUNT;
         ++sensor_index)
    {
        uint16_t previous = input.sensor[sensor_index].raw;

        for (unsigned invalid = FIRST_INVALID_ADC_CODE;
             invalid <= (unsigned)UINT16_MAX; ++invalid)
        {
            input.sensor[sensor_index].raw = (uint16_t)invalid;
            assert(!navigation_codec_raw_pack(&input, &packet));
            assert(memcmp(packet.bytes, saved.bytes, sizeof(packet.bytes)) ==
                   0);
        }

        input.sensor[sensor_index].raw = previous;
    }

    for (unsigned bit = FIRST_NAV_PADDING_BIT; bit < NAV_STORAGE_BIT_COUNT;
         ++bit)
    {
        packet = saved;
        field(packet.bytes, bit, 1, 1);
        assert(!navigation_codec_raw_unpack(&packet, &output));
        same_raw(&input, &output);
    }

    assert(!navigation_codec_raw_pack(NULL, &packet));
    assert(!navigation_codec_raw_pack(&input, NULL));
    assert(!navigation_codec_raw_unpack(NULL, &output));
    assert(!navigation_codec_raw_unpack(&packet, NULL));
    puts("PASS: raw-frame 12-bit boundaries, all sensor flags, full signed "
         "errors, independent 266-bit layout and failure non-mutation");
}

/** @brief Pack and unpack the tested sensor and navigation flag
 * combinations, verify field equality and return their count.
 */
static unsigned combinations (void)
{
    line_navigation_output_t  input = {0};
    line_navigation_output_t  output;
    navigation_codec_packet_t packet;
    unsigned                  checked = 0;
    /* clang-format off */
    input.timestamp_us       = 0xfffffff1U;
    input.sample_gap_us      = 0xfedcba98U;
    input.barcode            = (bc_decoder_result_t){'Z', true, 0x87654321U};
    input.action             = S3_NAV_UTURN;
    input.steering.left_mps  = number(0xbd123456U);
    input.steering.right_mps = number(0x3e123456U);
    /* clang-format on */

    for (int error = -MAX_LINE_ERROR_PERCENT; error <= MAX_LINE_ERROR_PERCENT;
         ++error)
    {
        input.line.error = (int16_t)error;

        for (unsigned events = 0; events < (unsigned)EVENT_MASK_COMBINATIONS;
             ++events)
        {
            input.events = events;

            for (unsigned faults = 0;
                 faults < (unsigned)FAULT_MASK_COMBINATIONS; ++faults)
            {
                /* clang-format off */
                input.fault_mask                    = (uint8_t)faults;
                input.line.b_is_valid               = (events & 1U) != 0U;
                input.line.b_is_line_present        = (events & 2U) != 0U;
                input.line.b_has_junction_candidate = (events & 4U) != 0U;
                input.line.b_has_lost_event         = (events & 8U) != 0U;
                input.line.b_has_reacquired_event   = (events & 16U) != 0U;
                input.line.b_has_junction_event     = (faults & 1U) != 0U;
                input.steering.b_is_valid           = (faults & 2U) != 0U;
                input.barcode_status                =
                    (bc_decoder_status_t)(events % 5U);
                /* clang-format on */

                for (unsigned sensor_index = 0;
                     sensor_index < (unsigned)FIXTURE_SENSOR_COUNT;
                     ++sensor_index)
                {
                    /* clang-format off */
                    input.sensor[sensor_index].b_is_valid =
                        (faults & (1U << sensor_index)) != 0U;
                    input.sensor[sensor_index].surface    =
                        (ir_reflectance_surface_t)((events + sensor_index) %
                                                   3U);
                    input.sensor[sensor_index].darkness   =
                        (uint16_t)(((events * 31U) + (faults * 17U) +
                                    (sensor_index * 101U)) %
                                   1001U);
                    /* clang-format on */
                }

                assert(navigation_codec_pack(&input, &packet));
                assert(navigation_codec_unpack(&packet, &output));
                same(&input, &output);
                ++checked;
                assert((packet.bytes[59] & 0x80U) == 0U);
            }
        }
    }

    for (unsigned value = 0; value <= MAX_NORMALISED_SURFACE; ++value)
    {
        input.sensor[0].darkness = (uint16_t)value;
        input.sensor[1].darkness = (uint16_t)(1000U - value);
        assert(navigation_codec_pack(&input, &packet));
        assert(navigation_codec_unpack(&packet, &output));
        same(&input, &output);
    }

    return checked;
}

/** @brief Exercise barcode and floating-point fields across packet encode
 * and decode operations.
 */
static void barcodes_and_floats (void)
{
    line_navigation_output_t   input = {0};
    line_navigation_output_t   output;
    navigation_codec_packet_t  packet;
    navigation_codec_barcode_t barcode_packet;
    bc_decoder_result_t        decoded;
    const uint32_t float_values[] = {0U,          0x80000000U, 0x3dcccccdU,
                                     0xbdcccccdU, 0x00000001U, 0x7f7fffffU,
                                     0x7f800000U, 0x7fc01234U};

    for (unsigned letter = 0; letter <= (unsigned)LETTER_WIRE_CODE_MAX;
         ++letter)
    {
        for (unsigned reversed = 0; reversed < (unsigned)SCAN_DIRECTION_COUNT;
             ++reversed)
        {
            /* clang-format off */
            input.barcode.letter        =
                letter ? (char)((unsigned)('A') + letter - 1U) : 0;
            input.barcode.b_is_reversed = reversed != 0U;
            input.barcode.timestamp_us  = reversed ? UINT32_MAX : 0U;
            /* clang-format on */
            assert(
                navigation_codec_barcode_pack(&input.barcode, &barcode_packet));
            assert((barcode_packet.bytes[4] & 0xc0U) == 0U);
            assert(navigation_codec_barcode_unpack(&barcode_packet, &decoded));
            same_barcode(&input.barcode, &decoded);

            for (unsigned action = 0; action < (unsigned)S3_NAV_ACTION_COUNT;
                 ++action)
            {
                input.action = (line_navigation_action_t)action;
                assert(navigation_codec_pack(&input, &packet));
                assert(navigation_codec_unpack(&packet, &output));
                same(&input, &output);
            }
        }
    }

    for (unsigned case_index = 0;
         case_index < (sizeof(float_values) / sizeof(float_values[0]));
         ++case_index)
    {
        input.steering.b_is_valid =
            false; /* Keep inactive opaque values exact. */
        input.steering.left_mps  = number(float_values[case_index]);
        input.steering.right_mps = number(float_values[7U - case_index]);
        assert(navigation_codec_pack(&input, &packet));
        assert(navigation_codec_unpack(&packet, &output));
        same(&input, &output);
    }
}

/** @brief Inject malformed navigation and barcode data and check rejection
 * by the appropriate codec.
 */
static void malformed (void)
{
    line_navigation_output_t   input = {0};
    line_navigation_output_t   out;
    navigation_codec_packet_t  good;
    navigation_codec_packet_t  packet;
    navigation_codec_barcode_t barcode_packet;
    bc_decoder_result_t        barcode = {0};
    bc_decoder_result_t        barcode_out;
    assert(!navigation_codec_pack(NULL, &packet));
    assert(!navigation_codec_pack(&input, NULL));
    assert(!navigation_codec_unpack(NULL, &out));
    assert(!navigation_codec_unpack(&packet, NULL));
    assert(!navigation_codec_barcode_pack(NULL, &barcode_packet));
    assert(!navigation_codec_barcode_pack(&barcode, NULL));
    assert(!navigation_codec_barcode_unpack(NULL, &barcode_out));
    assert(!navigation_codec_barcode_unpack(&barcode_packet, NULL));
    assert(navigation_codec_pack(&input, &good));

    for (unsigned bit = S3_NAVIGATION_BITS;
         bit < (S3_NAVIGATION_BYTES * BITS_PER_PACKET_BYTE); ++bit)
    {
        packet = good;
        field(packet.bytes, bit, 1, 1);
        assert(!navigation_codec_unpack(&packet, &out));
    }

    packet = good;
    field(packet.bytes, 73, 10, 1001);
    assert(!navigation_codec_unpack(&packet, &out));
    packet = good;
    field(packet.bytes, 83, 2, 3);
    assert(!navigation_codec_unpack(&packet, &out));
    packet = good;
    field(packet.bytes, 114, 8, 201);
    assert(!navigation_codec_unpack(&packet, &out));
    packet = good;
    field(packet.bytes, 190, 3, 5);
    assert(!navigation_codec_unpack(&packet, &out));
    packet = good;
    field(packet.bytes, 193, 5, 27);
    assert(!navigation_codec_unpack(&packet, &out));
    packet = good;
    field(packet.bytes, 231, 3, S3_NAV_ACTION_COUNT);
    assert(!navigation_codec_unpack(&packet, &out));
    assert(navigation_codec_barcode_pack(&barcode, &barcode_packet));
    barcode_packet.bytes[4] |= 0x80U;
    assert(!navigation_codec_barcode_unpack(&barcode_packet, &barcode_out));
    assert(navigation_codec_barcode_pack(&barcode, &barcode_packet));
    field(barcode_packet.bytes, 0, 5, 27);
    assert(!navigation_codec_barcode_unpack(&barcode_packet, &barcode_out));
    input.events = 32;
    assert(!navigation_codec_pack(&input, &packet));
    input.events     = 0;
    input.fault_mask = 8;
    assert(!navigation_codec_pack(&input, &packet));
    input.fault_mask         = 0;
    input.sensor[0].darkness = 1001;
    assert(!navigation_codec_pack(&input, &packet));
    input.sensor[0].darkness = 0;
    input.sensor[0].surface  = (ir_reflectance_surface_t)3;
    assert(!navigation_codec_pack(&input, &packet));
    input.sensor[0].surface = IR_SURFACE_UNKNOWN;
    input.line.error        = -101;
    assert(!navigation_codec_pack(&input, &packet));
    input.line.error = 101;
    assert(!navigation_codec_pack(&input, &packet));
    input.line.error     = 0;
    input.barcode_status = (bc_decoder_status_t)5;
    assert(!navigation_codec_pack(&input, &packet));
    input.barcode_status = S3_BARCODE_NONE;
    input.action         = S3_NAV_ACTION_COUNT;
    assert(!navigation_codec_pack(&input, &packet));
    input.action         = S3_NAV_NONE;
    input.barcode.letter = '[';
    assert(!navigation_codec_pack(&input, &packet));
    barcode.letter = '@';
    assert(!navigation_codec_barcode_pack(&barcode, &barcode_packet));
}

/** @brief Check expected packet bytes and that a rejected encode leaves the
 * prior packet intact.
 */
static void known_layout_and_atomic_failure (void)
{
    line_navigation_output_t  value = {0};
    line_navigation_output_t  decoded;
    navigation_codec_packet_t actual;
    navigation_codec_packet_t expected = {{0}};
    navigation_codec_packet_t unchanged;
    /* clang-format off */
    value.timestamp_us                  = 0x01020304U;
    value.sample_gap_us                 = 0x89abcdefU;
    value.events                        = 21;
    value.fault_mask                    = 5;
    value.sensor[0]                     =
        (line_navigation_sensor_t){true, 1000, IR_SURFACE_BLACK};
    value.sensor[2]                     =
        (line_navigation_sensor_t){true, 517, IR_SURFACE_WHITE};
    value.line.b_is_valid               = true;
    value.line.b_has_junction_candidate = true;
    value.line.error                    = -37;
    value.line.b_has_lost_event         = true;
    value.line.b_has_junction_event     = true;
    value.steering.b_is_valid           = true;
    value.steering.left_mps             = number(0x3dcccccdU);
    value.steering.right_mps            = number(0x80000000U);
    value.barcode_status                = S3_BARCODE_OVERFLOW;
    value.barcode                       =
        (bc_decoder_result_t){'Z', true, 0xfedcba98U};
    value.action                        = S3_NAV_UTURN;
    /* clang-format on */
    field(expected.bytes, 0, 32, 0x01020304U);
    field(expected.bytes, 32, 32, 0x89abcdefU);
    field(expected.bytes, 64, 5, 21);
    field(expected.bytes, 69, 3, 5);
    field(expected.bytes, 72, 1, 1);
    field(expected.bytes, 73, 10, 1000);
    field(expected.bytes, 83, 2, 2);
    field(expected.bytes, 98, 1, 1);
    field(expected.bytes, 99, 10, 517);
    field(expected.bytes, 109, 2, 1);
    field(expected.bytes, 111, 1, 1);
    field(expected.bytes, 113, 1, 1);
    field(expected.bytes, 114, 8, 63);
    field(expected.bytes, 122, 1, 1);
    field(expected.bytes, 124, 1, 1);
    field(expected.bytes, 125, 1, 1);
    field(expected.bytes, 126, 32, 0x3dcccccdU);
    field(expected.bytes, 158, 32, 0x80000000U);
    field(expected.bytes, 190, 3, 4);
    field(expected.bytes, 193, 5, 26);
    field(expected.bytes, 198, 1, 1);
    field(expected.bytes, 199, 32, 0xfedcba98U);
    field(expected.bytes, 231, 3, 4);
    assert(navigation_codec_pack(&value, &actual));
    assert(memcmp(actual.bytes, expected.bytes, sizeof(actual.bytes)) == 0);
    assert(navigation_codec_unpack(&actual, &decoded));
    same(&value, &decoded);
    unchanged    = actual;
    value.events = 32;
    assert(!navigation_codec_pack(&value, &actual));
    assert(memcmp(actual.bytes, unchanged.bytes, sizeof(actual.bytes)) == 0);
    value.events     = 21;
    actual.bytes[59] = (uint8_t)((unsigned)actual.bytes[59] | 0x80U);
    assert(!navigation_codec_unpack(&actual, &decoded));
    same(&value, &decoded);
    navigation_codec_barcode_t barcode_packet;
    navigation_codec_barcode_t previous;
    assert(navigation_codec_barcode_pack(&value.barcode, &barcode_packet));
    previous             = barcode_packet;
    value.barcode.letter = '!';
    assert(!navigation_codec_barcode_pack(&value.barcode, &barcode_packet));
    assert(memcmp(barcode_packet.bytes, previous.bytes,
                  sizeof(barcode_packet.bytes)) == 0);
}

/** @brief Feed 20,000 deterministic candidate navigation packets and require
 * every accepted packet to re-encode identically.
 */
static void hostile_bytes (void)
{
    uint32_t random   = 0x7a31c29dU;
    unsigned accepted = 0;

    for (unsigned iteration = 0; iteration < PACKET_RANDOM_TRIAL_COUNT;
         ++iteration)
    {
        navigation_codec_packet_t input;
        navigation_codec_packet_t encoded;
        line_navigation_output_t  decoded;

        for (unsigned byte_index = 0; byte_index < sizeof(input.bytes);
             ++byte_index)
        {
            random                 ^= random << 13;
            random                 ^= random >> 17;
            random                 ^= random << 5;
            input.bytes[byte_index] = (uint8_t)random;
        }
        /* Exercise both proper and improper padding rather than almost always
         * rejecting at the first reserved-bit check. */
        if (iteration & 1U)
        {
            input.bytes[59] &= 0x7fU;
        }

        if (navigation_codec_unpack(&input, &decoded))
        {
            assert(navigation_codec_pack(&decoded, &encoded));
            assert(memcmp(input.bytes, encoded.bytes, sizeof(input.bytes)) ==
                   0);
            ++accepted;
        }
    }

    assert(accepted > 0U);
    printf("PASS: 20000 hostile packets checked; %u valid packets re-encode "
           "identically\n",
           accepted);
}

/** @brief Feed 20,000 deterministic raw frames and require every accepted
 * frame to re-encode identically.
 */
static void hostile_raw_bytes (void)
{
    uint32_t random   = 0xc0decafeU;
    unsigned accepted = 0;

    for (unsigned iteration = 0; iteration < PACKET_RANDOM_TRIAL_COUNT;
         ++iteration)
    {
        navigation_codec_raw_frame_t input;
        navigation_codec_raw_frame_t encoded;
        ir_acquisition_frame_t       decoded;

        for (unsigned byte_index = 0; byte_index < sizeof(input.bytes);
             ++byte_index)
        {
            random                 ^= random << 13;
            random                 ^= random >> 17;
            random                 ^= random << 5;
            input.bytes[byte_index] = (uint8_t)random;
        }

        if (iteration & 1U)
        {
            input.bytes[33] &= 3U;
        }

        if (navigation_codec_raw_unpack(&input, &decoded))
        {
            assert(navigation_codec_raw_pack(&decoded, &encoded));
            assert(memcmp(input.bytes, encoded.bytes, sizeof(input.bytes)) ==
                   0);
            ++accepted;
        }
    }

    assert(accepted >= 10000U);
    printf("PASS: 20000 hostile raw frames checked; %u valid frames re-encode "
           "identically\n",
           accepted);
}

/* End of file. */

