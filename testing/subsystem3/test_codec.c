/* Independent semantic round-trip / malformed-packet checks. No hardware. */
#include "navigation_codec.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t bits(float value)
{ uint32_t result; memcpy(&result, &value, sizeof(result)); return result; }
static float number(uint32_t value)
{ float result; memcpy(&result, &value, sizeof(result)); return result; }

static void same_barcode(const s3_barcode_result_t *a, const s3_barcode_result_t *b)
{
    assert(a->letter == b->letter && a->reversed == b->reversed);
    assert(a->timestamp_us == b->timestamp_us);
}
static void same(const s3_navigation_output_t *a, const s3_navigation_output_t *b)
{
    assert(a->timestamp_us == b->timestamp_us && a->sample_gap_us == b->sample_gap_us);
    assert(a->events == b->events && a->fault_mask == b->fault_mask);
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i) {
        assert(a->sensor[i].valid == b->sensor[i].valid);
        assert(a->sensor[i].darkness == b->sensor[i].darkness);
        assert(a->sensor[i].surface == b->sensor[i].surface);
    }
    assert(a->line.valid == b->line.valid && a->line.present == b->line.present);
    assert(a->line.junction_candidate == b->line.junction_candidate);
    assert(a->line.error == b->line.error);
    assert(a->line.lost_event == b->line.lost_event);
    assert(a->line.reacquired_event == b->line.reacquired_event);
    assert(a->line.junction_event == b->line.junction_event);
    assert(a->steering.valid == b->steering.valid);
    assert(bits(a->steering.left_mps) == bits(b->steering.left_mps));
    assert(bits(a->steering.right_mps) == bits(b->steering.right_mps));
    assert(a->barcode_status == b->barcode_status && a->action == b->action);
    same_barcode(&a->barcode, &b->barcode);
}

/* Independently set an LSB-first field to inject values the encoder rejects. */
static void field(uint8_t *bytes, unsigned start, unsigned width, uint32_t value)
{
    for (unsigned i = 0; i < width; ++i) {
        unsigned bit = start + i;
        bytes[bit / 8] = (uint8_t)((bytes[bit / 8] & ~(1U << (bit % 8))) |
                                  (((value >> i) & 1U) << (bit % 8)));
    }
}

static void same_raw(const s3_sensor_frame_t *a, const s3_sensor_frame_t *b)
{
    assert(a->timestamp_us == b->timestamp_us);
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i) {
        assert(a->sensor[i].timestamp_us == b->sensor[i].timestamp_us);
        assert(a->sensor[i].raw == b->sensor[i].raw);
        assert(a->sensor[i].error == b->sensor[i].error);
        assert(a->sensor[i].enabled == b->sensor[i].enabled);
        assert(a->sensor[i].valid == b->sensor[i].valid);
    }
}

static void raw_frames(void)
{
    s3_sensor_frame_t input = {0}, output;
    s3_raw_frame_packet_t packet, saved, expected = {{0}};
    const int32_t errors[] = {0, -1, INT32_MIN, INT32_MAX, -1234567, 1234567};
    input.timestamp_us = 0xfedcba98U;
    /* Every ADC value remains exact, including disabled/invalid diagnostic
     * samples. Flags are information, not permission to discard fields. */
    for (unsigned raw = 0; raw <= 4095U; ++raw) {
        for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i) {
            input.sensor[i].raw = (uint16_t)((raw + i * 997U) & 4095U);
            input.sensor[i].timestamp_us = raw ^ (UINT32_MAX - i);
            input.sensor[i].error = errors[(raw + i) % 6U];
            input.sensor[i].enabled = ((raw >> i) & 1U) != 0U;
            input.sensor[i].valid = ((raw >> (i + 3U)) & 1U) != 0U;
        }
        assert(s3_raw_frame_pack(&input, &packet));
        assert((packet.bytes[33] & 0xfcU) == 0U);
        assert(s3_raw_frame_unpack(&packet, &output)); same_raw(&input, &output);
    }
    /* Set contrasting values across byte boundaries and construct the wire
     * layout independently of production get/put helpers. */
    input.sensor[0] = (s3_raw_sample_t){0x01020304U, 4095, INT32_MIN, true, false};
    input.sensor[1] = (s3_raw_sample_t){UINT32_MAX, 0, INT32_MAX, false, true};
    input.sensor[2] = (s3_raw_sample_t){0, 0xabc, -1234567, true, true};
    field(expected.bytes, 0, 32, input.timestamp_us);
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i) {
        unsigned start = 32U + i * 78U;
        field(expected.bytes, start, 32, input.sensor[i].timestamp_us);
        field(expected.bytes, start + 32U, 12, input.sensor[i].raw);
        field(expected.bytes, start + 44U, 32, (uint32_t)input.sensor[i].error);
        field(expected.bytes, start + 76U, 1, input.sensor[i].enabled);
        field(expected.bytes, start + 77U, 1, input.sensor[i].valid);
    }
    assert(s3_raw_frame_pack(&input, &packet));
    assert(memcmp(packet.bytes, expected.bytes, sizeof(packet.bytes)) == 0);
    assert(s3_raw_frame_unpack(&expected, &output)); same_raw(&input, &output);
    saved = packet;
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i) {
        uint16_t previous = input.sensor[i].raw;
        for (unsigned invalid = 4096U; invalid <= UINT16_MAX; ++invalid) {
            input.sensor[i].raw = (uint16_t)invalid;
            assert(!s3_raw_frame_pack(&input, &packet));
            assert(memcmp(packet.bytes, saved.bytes, sizeof(packet.bytes)) == 0);
        }
        input.sensor[i].raw = previous;
    }
    for (unsigned bit = 266U; bit < 272U; ++bit) {
        packet = saved; field(packet.bytes, bit, 1, 1);
        assert(!s3_raw_frame_unpack(&packet, &output)); same_raw(&input, &output);
    }
    assert(!s3_raw_frame_pack(NULL, &packet));
    assert(!s3_raw_frame_pack(&input, NULL));
    assert(!s3_raw_frame_unpack(NULL, &output));
    assert(!s3_raw_frame_unpack(&packet, NULL));
    puts("PASS: raw-frame 12-bit boundaries, all sensor flags, full signed errors, independent 266-bit layout and failure non-mutation");
}

static unsigned combinations(void)
{
    s3_navigation_output_t input = {0}, output;
    s3_navigation_packet_t packet;
    unsigned checked = 0;
    input.timestamp_us = 0xfffffff1U;
    input.sample_gap_us = 0xfedcba98U;
    input.barcode = (s3_barcode_result_t){'Z', true, 0x87654321U};
    input.action = S3_NAV_UTURN;
    input.steering.left_mps = number(0xbd123456U);
    input.steering.right_mps = number(0x3e123456U);
    for (int error = -100; error <= 100; ++error) {
        input.line.error = (int16_t)error;
        for (unsigned events = 0; events < 32; ++events) {
            input.events = events;
            for (unsigned faults = 0; faults < 8; ++faults) {
                input.fault_mask = (uint8_t)faults;
                input.line.valid = (events & 1U) != 0;
                input.line.present = (events & 2U) != 0;
                input.line.junction_candidate = (events & 4U) != 0;
                input.line.lost_event = (events & 8U) != 0;
                input.line.reacquired_event = (events & 16U) != 0;
                input.line.junction_event = (faults & 1U) != 0;
                input.steering.valid = (faults & 2U) != 0;
                input.barcode_status = (s3_barcode_status_t)(events % 5U);
                for (unsigned i = 0; i < 3; ++i) {
                    input.sensor[i].valid = (faults & (1U << i)) != 0;
                    input.sensor[i].surface = (ir_surface_t)((events + i) % 3U);
                    input.sensor[i].darkness = (uint16_t)((events * 31U + faults * 17U + i * 101U) % 1001U);
                }
                assert(s3_navigation_pack(&input, &packet));
                assert(s3_navigation_unpack(&packet, &output));
                same(&input, &output); ++checked;
                assert((packet.bytes[59] & 0x80U) == 0U);
            }
        }
    }
    for (unsigned value = 0; value <= 1000; ++value) {
        input.sensor[0].darkness = (uint16_t)value;
        input.sensor[1].darkness = (uint16_t)(1000U - value);
        assert(s3_navigation_pack(&input, &packet));
        assert(s3_navigation_unpack(&packet, &output)); same(&input, &output);
    }
    return checked;
}

static void barcodes_and_floats(void)
{
    s3_navigation_output_t input = {0}, output;
    s3_navigation_packet_t packet;
    s3_barcode_packet_t barcode_packet;
    s3_barcode_result_t decoded;
    const uint32_t float_values[] = {0U, 0x80000000U, 0x3dcccccdU,
        0xbdcccccdU, 0x00000001U, 0x7f7fffffU, 0x7f800000U, 0x7fc01234U};
    for (unsigned letter = 0; letter <= 26; ++letter) {
        for (unsigned reversed = 0; reversed < 2; ++reversed) {
            input.barcode.letter = letter ? (char)('A' + letter - 1U) : 0;
            input.barcode.reversed = reversed != 0;
            input.barcode.timestamp_us = reversed ? UINT32_MAX : 0U;
            assert(s3_barcode_pack(&input.barcode, &barcode_packet));
            assert((barcode_packet.bytes[4] & 0xc0U) == 0U);
            assert(s3_barcode_unpack(&barcode_packet, &decoded));
            same_barcode(&input.barcode, &decoded);
            for (unsigned action = 0; action < S3_NAV_ACTION_COUNT; ++action) {
                input.action = (s3_nav_action_t)action;
                assert(s3_navigation_pack(&input, &packet));
                assert(s3_navigation_unpack(&packet, &output)); same(&input, &output);
            }
        }
    }
    for (unsigned i = 0; i < sizeof(float_values) / sizeof(float_values[0]); ++i) {
        input.steering.valid = false; /* opaque inactive values remain exact */
        input.steering.left_mps = number(float_values[i]);
        input.steering.right_mps = number(float_values[7U - i]);
        assert(s3_navigation_pack(&input, &packet));
        assert(s3_navigation_unpack(&packet, &output)); same(&input, &output);
    }
}

static void malformed(void)
{
    s3_navigation_output_t input = {0}, out;
    s3_navigation_packet_t good, packet;
    s3_barcode_packet_t barcode_packet;
    s3_barcode_result_t barcode = {0}, barcode_out;
    assert(!s3_navigation_pack(NULL, &packet));
    assert(!s3_navigation_pack(&input, NULL));
    assert(!s3_navigation_unpack(NULL, &out));
    assert(!s3_navigation_unpack(&packet, NULL));
    assert(!s3_barcode_pack(NULL, &barcode_packet));
    assert(!s3_barcode_pack(&barcode, NULL));
    assert(!s3_barcode_unpack(NULL, &barcode_out));
    assert(!s3_barcode_unpack(&barcode_packet, NULL));
    assert(s3_navigation_pack(&input, &good));
    for (unsigned bit = S3_NAVIGATION_BITS; bit < S3_NAVIGATION_BYTES*8U; ++bit) {
        packet = good; field(packet.bytes, bit, 1, 1);
        assert(!s3_navigation_unpack(&packet, &out));
    }
    packet = good; field(packet.bytes, 73, 10, 1001);
    assert(!s3_navigation_unpack(&packet, &out));
    packet = good; field(packet.bytes, 83, 2, 3);
    assert(!s3_navigation_unpack(&packet, &out));
    packet = good; field(packet.bytes, 114, 8, 201);
    assert(!s3_navigation_unpack(&packet, &out));
    packet = good; field(packet.bytes, 190, 3, 5);
    assert(!s3_navigation_unpack(&packet, &out));
    packet = good; field(packet.bytes, 193, 5, 27);
    assert(!s3_navigation_unpack(&packet, &out));
    packet = good; field(packet.bytes, 231, 3, S3_NAV_ACTION_COUNT);
    assert(!s3_navigation_unpack(&packet, &out));
    assert(s3_barcode_pack(&barcode, &barcode_packet));
    barcode_packet.bytes[4] |= 0x80U;
    assert(!s3_barcode_unpack(&barcode_packet, &barcode_out));
    assert(s3_barcode_pack(&barcode, &barcode_packet));
    field(barcode_packet.bytes, 0, 5, 27);
    assert(!s3_barcode_unpack(&barcode_packet, &barcode_out));
    input.events = 32; assert(!s3_navigation_pack(&input, &packet)); input.events = 0;
    input.fault_mask = 8; assert(!s3_navigation_pack(&input, &packet)); input.fault_mask = 0;
    input.sensor[0].darkness = 1001; assert(!s3_navigation_pack(&input, &packet)); input.sensor[0].darkness = 0;
    input.sensor[0].surface = (ir_surface_t)3; assert(!s3_navigation_pack(&input, &packet)); input.sensor[0].surface = IR_SURFACE_UNKNOWN;
    input.line.error = -101; assert(!s3_navigation_pack(&input, &packet));
    input.line.error = 101; assert(!s3_navigation_pack(&input, &packet)); input.line.error = 0;
    input.barcode_status = (s3_barcode_status_t)5; assert(!s3_navigation_pack(&input, &packet)); input.barcode_status = S3_BARCODE_NONE;
    input.action = S3_NAV_ACTION_COUNT; assert(!s3_navigation_pack(&input, &packet)); input.action = S3_NAV_NONE;
    input.barcode.letter = '['; assert(!s3_navigation_pack(&input, &packet));
    barcode.letter = '@'; assert(!s3_barcode_pack(&barcode, &barcode_packet));
}

static void known_layout_and_atomic_failure(void)
{
    s3_navigation_output_t value = {0}, decoded;
    s3_navigation_packet_t actual, expected = {{0}}, unchanged;
    value.timestamp_us = 0x01020304U; value.sample_gap_us = 0x89abcdefU;
    value.events = 21; value.fault_mask = 5;
    value.sensor[0] = (s3_processed_sensor_t){true,1000,IR_SURFACE_BLACK};
    value.sensor[2] = (s3_processed_sensor_t){true,517,IR_SURFACE_WHITE};
    value.line.valid = true; value.line.junction_candidate = true;
    value.line.error = -37; value.line.lost_event = true; value.line.junction_event = true;
    value.steering.valid = true;
    value.steering.left_mps = number(0x3dcccccdU); value.steering.right_mps = number(0x80000000U);
    value.barcode_status = S3_BARCODE_OVERFLOW;
    value.barcode = (s3_barcode_result_t){'Z',true,0xfedcba98U}; value.action = S3_NAV_UTURN;
    field(expected.bytes,0,32,0x01020304U); field(expected.bytes,32,32,0x89abcdefU);
    field(expected.bytes,64,5,21); field(expected.bytes,69,3,5);
    field(expected.bytes,72,1,1); field(expected.bytes,73,10,1000); field(expected.bytes,83,2,2);
    field(expected.bytes,98,1,1); field(expected.bytes,99,10,517); field(expected.bytes,109,2,1);
    field(expected.bytes,111,1,1); field(expected.bytes,113,1,1); field(expected.bytes,114,8,63);
    field(expected.bytes,122,1,1); field(expected.bytes,124,1,1); field(expected.bytes,125,1,1);
    field(expected.bytes,126,32,0x3dcccccdU); field(expected.bytes,158,32,0x80000000U);
    field(expected.bytes,190,3,4); field(expected.bytes,193,5,26); field(expected.bytes,198,1,1);
    field(expected.bytes,199,32,0xfedcba98U); field(expected.bytes,231,3,4);
    assert(s3_navigation_pack(&value, &actual));
    assert(memcmp(actual.bytes, expected.bytes, sizeof(actual.bytes)) == 0);
    assert(s3_navigation_unpack(&actual, &decoded)); same(&value, &decoded);
    unchanged = actual;
    value.events = 32;
    assert(!s3_navigation_pack(&value, &actual));
    assert(memcmp(actual.bytes, unchanged.bytes, sizeof(actual.bytes)) == 0);
    value.events = 21;
    actual.bytes[59] |= 0x80;
    assert(!s3_navigation_unpack(&actual, &decoded)); same(&value, &decoded);
    s3_barcode_packet_t barcode_packet, previous;
    assert(s3_barcode_pack(&value.barcode, &barcode_packet)); previous = barcode_packet;
    value.barcode.letter = '!'; assert(!s3_barcode_pack(&value.barcode, &barcode_packet));
    assert(memcmp(barcode_packet.bytes, previous.bytes, sizeof(barcode_packet.bytes)) == 0);
}

static void hostile_bytes(void)
{
    uint32_t random = 0x7a31c29dU;
    unsigned accepted = 0;
    for (unsigned iteration = 0; iteration < 20000U; ++iteration) {
        s3_navigation_packet_t input, encoded;
        s3_navigation_output_t decoded;
        for (unsigned i = 0; i < sizeof(input.bytes); ++i) {
            random ^= random << 13; random ^= random >> 17; random ^= random << 5;
            input.bytes[i] = (uint8_t)random;
        }
        /* Exercise both proper and improper padding rather than almost always
         * rejecting at the first reserved-bit check. */
        if (iteration & 1U) input.bytes[59] &= 0x7fU;
        if (s3_navigation_unpack(&input, &decoded)) {
            assert(s3_navigation_pack(&decoded, &encoded));
            assert(memcmp(input.bytes, encoded.bytes, sizeof(input.bytes)) == 0);
            ++accepted;
        }
    }
    assert(accepted > 0U);
    printf("PASS: 20000 hostile packets checked; %u valid packets re-encode identically\n", accepted);
}

static void hostile_raw_bytes(void)
{
    uint32_t random = 0xc0decafeU;
    unsigned accepted = 0;
    for (unsigned iteration = 0; iteration < 20000U; ++iteration) {
        s3_raw_frame_packet_t input, encoded;
        s3_sensor_frame_t decoded;
        for (unsigned i = 0; i < sizeof(input.bytes); ++i) {
            random ^= random << 13; random ^= random >> 17; random ^= random << 5;
            input.bytes[i] = (uint8_t)random;
        }
        if (iteration & 1U) input.bytes[33] &= 3U;
        if (s3_raw_frame_unpack(&input, &decoded)) {
            assert(s3_raw_frame_pack(&decoded, &encoded));
            assert(memcmp(input.bytes, encoded.bytes, sizeof(input.bytes)) == 0);
            ++accepted;
        }
    }
    assert(accepted >= 10000U);
    printf("PASS: 20000 hostile raw frames checked; %u valid frames re-encode identically\n", accepted);
}

int main(void)
{
    assert(sizeof(s3_navigation_packet_t) == 60U && sizeof(s3_barcode_packet_t) == 5U);
    assert(sizeof(s3_raw_frame_packet_t) == 34U);
    unsigned checked = combinations();
    barcodes_and_floats(); malformed(); known_layout_and_atomic_failure(); hostile_bytes();
    raw_frames(); hostile_raw_bytes();
    printf("PASS: %u error/event/fault combinations, all darkness values, A-Z/none/direction/actions, exact float bits and malformed packet rejection\n", checked);
    return 0;
}
