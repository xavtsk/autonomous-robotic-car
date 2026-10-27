#include "navigation_codec.h"
#include <float.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
               "shared steering requires the existing binary32 float format");
_Static_assert(sizeof(s3_navigation_packet_t) == 60, "navigation packet layout");
_Static_assert(sizeof(double) == 8 && DBL_MANT_DIG == 53, "odometry binary64");
_Static_assert(sizeof(s3_barcode_packet_t) == 5, "barcode packet layout");
_Static_assert(sizeof(s3_raw_frame_packet_t) == 34 && S3_SENSOR_COUNT == 3,
               "raw-frame packet layout changed");
_Static_assert(S3_EVENT_SENSOR_FAULT == 16 && S3_ALL_SENSORS == 7,
               "event/sensor mask widths changed");

static void put(uint8_t *bytes, unsigned *bit, uint32_t value, unsigned width)
{
    for (unsigned i = 0; i < width; ++i, ++*bit)
        if ((value & ((uint32_t)1U << i)) != 0U)
            bytes[*bit / 8U] |= (uint8_t)(1U << (*bit % 8U));
}
static uint32_t get(const uint8_t *bytes, unsigned *bit, unsigned width)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < width; ++i, ++*bit)
        if ((bytes[*bit / 8U] & (1U << (*bit % 8U))) != 0U)
            value |= (uint32_t)1U << i;
    return value;
}
static uint32_t float_bits(float value)
{
    union { float f; uint32_t bits; } copy;
    copy.f = value;
    return copy.bits;
}
static float bits_float(uint32_t value)
{
    union { float f; uint32_t bits; } copy;
    copy.bits = value;
    return copy.f;
}
static bool letter_valid(char c)
{ return c == 0 || (c >= 'A' && c <= 'Z'); }
static void write_double(uint8_t *bytes, unsigned *bit, double value)
{
    union { double value; uint64_t bits; } copy;
    copy.value=value;
    put(bytes,bit,(uint32_t)copy.bits,32);
    put(bytes,bit,(uint32_t)(copy.bits>>32),32);
}
static double read_double(const uint8_t *bytes, unsigned *bit)
{
    union { double value; uint64_t bits; } copy;
    uint32_t low=get(bytes,bit,32), high=get(bytes,bit,32);
    copy.bits=(uint64_t)low|((uint64_t)high<<32);
    return copy.value;
}
static void write_barcode(uint8_t *bytes, unsigned *bit, const s3_barcode_result_t *value)
{
    put(bytes, bit, value->letter == 0 ? 0U : (unsigned)(value->letter - 'A') + 1U, 5);
    put(bytes, bit, value->reversed, 1);
    put(bytes, bit, value->timestamp_us, 32);
}
static bool read_barcode(const uint8_t *bytes, unsigned *bit, s3_barcode_result_t *value)
{
    unsigned code = get(bytes, bit, 5);
    if (code > 26U) return false;
    value->letter = code == 0U ? 0 : (char)('A' + code - 1U);
    value->reversed = get(bytes, bit, 1) != 0U;
    value->timestamp_us = get(bytes, bit, 32);
    return true;
}
bool s3_barcode_pack(const s3_barcode_result_t *value, s3_barcode_packet_t *out)
{
    s3_barcode_packet_t copy = {{0}};
    unsigned bit = 0;
    if (!value || !out || !letter_valid(value->letter)) return false;
    write_barcode(copy.bytes, &bit, value);
    *out = copy;
    return true;
}
bool s3_barcode_unpack(const s3_barcode_packet_t *packet, s3_barcode_result_t *out)
{
    s3_barcode_result_t value = {0};
    unsigned bit = 0;
    if (!packet || !out || (packet->bytes[4] & 0xc0U) != 0U ||
        !read_barcode(packet->bytes, &bit, &value)) return false;
    *out = value;
    return true;
}
bool s3_raw_frame_pack(const s3_sensor_frame_t *value, s3_raw_frame_packet_t *out)
{
    s3_raw_frame_packet_t copy = {{0}};
    unsigned bit = 0;
    if (!value || !out) return false;
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i)
        if (value->sensor[i].raw > 4095U) return false;
    put(copy.bytes, &bit, value->timestamp_us, 32);
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i) {
        const s3_raw_sample_t *sample = &value->sensor[i];
        put(copy.bytes, &bit, sample->timestamp_us, 32);
        put(copy.bytes, &bit, sample->raw, 12);
        put(copy.bytes, &bit, (uint32_t)sample->error, 32);
        put(copy.bytes, &bit, sample->enabled, 1);
        put(copy.bytes, &bit, sample->valid, 1);
    }
    *out = copy;
    return true;
}
bool s3_raw_frame_unpack(const s3_raw_frame_packet_t *packet, s3_sensor_frame_t *out)
{
    s3_sensor_frame_t value = {0};
    unsigned bit = 0;
    if (!packet || !out || (packet->bytes[33] & 0xfcU) != 0U) return false;
    value.timestamp_us = get(packet->bytes, &bit, 32);
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i) {
        s3_raw_sample_t *sample = &value.sensor[i];
        uint32_t error;
        sample->timestamp_us = get(packet->bytes, &bit, 32);
        sample->raw = (uint16_t)get(packet->bytes, &bit, 12);
        error = get(packet->bytes, &bit, 32);
        sample->error = error <= INT32_MAX ? (int32_t)error
                     : -1 - (int32_t)(UINT32_MAX - error);
        sample->enabled = get(packet->bytes, &bit, 1) != 0U;
        sample->valid = get(packet->bytes, &bit, 1) != 0U;
    }
    *out = value;
    return true;
}
bool s3_navigation_pack(const s3_navigation_output_t *value, s3_navigation_packet_t *out)
{
    s3_navigation_packet_t copy = {{0}};
    unsigned bit = 0;
    if (!value || !out || (value->events & ~31U) != 0U || value->fault_mask > 7U ||
        (unsigned)value->barcode_status > S3_BARCODE_OVERFLOW ||
        (unsigned)value->action >= S3_NAV_ACTION_COUNT ||
        value->line.error < -100 || value->line.error > 100 ||
        !letter_valid(value->barcode.letter) ||
        (unsigned)value->lifecycle > S3_RUN_FAULT_LATCHED ||
        value->readiness_blockers > 31U || value->latched_faults > 15U ||
        value->accepted_calibration > 7U) return false;
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i)
        if (value->sensor[i].darkness > 1000U ||
            (unsigned)value->sensor[i].surface > IR_SURFACE_BLACK) return false;
    put(copy.bytes, &bit, value->timestamp_us, 32);
    put(copy.bytes, &bit, value->sample_gap_us, 32);
    put(copy.bytes, &bit, value->events, 5);
    put(copy.bytes, &bit, value->fault_mask, 3);
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i) {
        put(copy.bytes, &bit, value->sensor[i].valid, 1);
        put(copy.bytes, &bit, value->sensor[i].darkness, 10);
        put(copy.bytes, &bit, value->sensor[i].surface, 2);
    }
    put(copy.bytes, &bit, value->line.valid, 1);
    put(copy.bytes, &bit, value->line.present, 1);
    put(copy.bytes, &bit, value->line.junction_candidate, 1);
    put(copy.bytes, &bit, (unsigned)(value->line.error + 100), 8);
    put(copy.bytes, &bit, value->line.lost_event, 1);
    put(copy.bytes, &bit, value->line.reacquired_event, 1);
    put(copy.bytes, &bit, value->line.junction_event, 1);
    put(copy.bytes, &bit, value->steering.valid, 1);
    put(copy.bytes, &bit, float_bits(value->steering.left_mps), 32);
    put(copy.bytes, &bit, float_bits(value->steering.right_mps), 32);
    put(copy.bytes, &bit, value->barcode_status, 3);
    write_barcode(copy.bytes, &bit, &value->barcode);
    put(copy.bytes, &bit, value->action, 3);
    put(copy.bytes,&bit,value->steering_preview.valid,1);
    put(copy.bytes,&bit,float_bits(value->steering_preview.left_mps),32);
    put(copy.bytes,&bit,float_bits(value->steering_preview.right_mps),32);
    put(copy.bytes,&bit,value->lifecycle,2);
    put(copy.bytes,&bit,value->readiness_blockers,5);
    put(copy.bytes,&bit,value->latched_faults,4);
    put(copy.bytes,&bit,value->accepted_calibration,3);
    put(copy.bytes,&bit,value->stop_requested,1);
    put(copy.bytes,&bit,value->odometry_available,1);
    put(copy.bytes,&bit,value->rearming,1);
    put(copy.bytes,&bit,value->diagnostic,1);
    write_double(copy.bytes,&bit,value->capture_distance.start_m);
    write_double(copy.bytes,&bit,value->capture_distance.end_m);
    put(copy.bytes,&bit,value->capture_distance.epoch,32);
    put(copy.bytes,&bit,value->capture_distance.start_valid,1);
    put(copy.bytes,&bit,value->capture_distance.end_valid,1);
    *out = copy;
    return true;
}
bool s3_navigation_unpack(const s3_navigation_packet_t *packet, s3_navigation_output_t *out)
{
    s3_navigation_output_t value = {0};
    unsigned bit = 0, error;
    if (!packet || !out || (packet->bytes[59] & 0x80U) != 0U) return false;
    value.timestamp_us = get(packet->bytes, &bit, 32);
    value.sample_gap_us = get(packet->bytes, &bit, 32);
    value.events = get(packet->bytes, &bit, 5);
    value.fault_mask = (uint8_t)get(packet->bytes, &bit, 3);
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i) {
        value.sensor[i].valid = get(packet->bytes, &bit, 1) != 0U;
        value.sensor[i].darkness = (uint16_t)get(packet->bytes, &bit, 10);
        value.sensor[i].surface = (ir_surface_t)get(packet->bytes, &bit, 2);
        if (value.sensor[i].darkness > 1000U || value.sensor[i].surface > IR_SURFACE_BLACK)
            return false;
    }
    value.line.valid = get(packet->bytes, &bit, 1) != 0U;
    value.line.present = get(packet->bytes, &bit, 1) != 0U;
    value.line.junction_candidate = get(packet->bytes, &bit, 1) != 0U;
    error = get(packet->bytes, &bit, 8);
    if (error > 200U) return false;
    value.line.error = (int16_t)((int)error - 100);
    value.line.lost_event = get(packet->bytes, &bit, 1) != 0U;
    value.line.reacquired_event = get(packet->bytes, &bit, 1) != 0U;
    value.line.junction_event = get(packet->bytes, &bit, 1) != 0U;
    value.steering.valid = get(packet->bytes, &bit, 1) != 0U;
    value.steering.left_mps = bits_float(get(packet->bytes, &bit, 32));
    value.steering.right_mps = bits_float(get(packet->bytes, &bit, 32));
    value.barcode_status = (s3_barcode_status_t)get(packet->bytes, &bit, 3);
    if (value.barcode_status > S3_BARCODE_OVERFLOW ||
        !read_barcode(packet->bytes, &bit, &value.barcode)) return false;
    value.action = (s3_nav_action_t)get(packet->bytes, &bit, 3);
    if (value.action >= S3_NAV_ACTION_COUNT) return false;
    value.steering_preview.valid=get(packet->bytes,&bit,1)!=0U;
    value.steering_preview.left_mps=bits_float(get(packet->bytes,&bit,32));
    value.steering_preview.right_mps=bits_float(get(packet->bytes,&bit,32));
    value.lifecycle=(s3_run_state_t)get(packet->bytes,&bit,2);
    if (value.lifecycle>S3_RUN_FAULT_LATCHED) return false;
    value.readiness_blockers=get(packet->bytes,&bit,5);
    value.latched_faults=get(packet->bytes,&bit,4);
    value.accepted_calibration=(uint8_t)get(packet->bytes,&bit,3);
    value.stop_requested=get(packet->bytes,&bit,1)!=0U;
    value.odometry_available=get(packet->bytes,&bit,1)!=0U;
    value.rearming=get(packet->bytes,&bit,1)!=0U;
    value.diagnostic=get(packet->bytes,&bit,1)!=0U;
    value.capture_distance.start_m=read_double(packet->bytes,&bit);
    value.capture_distance.end_m=read_double(packet->bytes,&bit);
    value.capture_distance.epoch=get(packet->bytes,&bit,32);
    value.capture_distance.start_valid=get(packet->bytes,&bit,1)!=0U;
    value.capture_distance.end_valid=get(packet->bytes,&bit,1)!=0U;
    *out = value;
    return true;
}
