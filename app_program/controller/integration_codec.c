#include "integration_codec.h"
#include <float.h>
_Static_assert(sizeof(double) == 8 && DBL_MANT_DIG == 53 && sizeof(float) == 4,
               "exact odometry and calibration representations");
_Static_assert(sizeof(s3_status_packet_t) == 146 &&
                   sizeof(s3_control_packet_t) == 11,
               "copied runtime storage");

static void put (uint8_t * p_p, unsigned * p_bit, uint32_t value,
                 unsigned width)
{
    for (unsigned value_i = 0; value_i < width; value_i++, (*p_bit)++)
    {
        if (value & ((uint32_t)1U << value_i))
        {
            p_p[*p_bit / 8] |= (uint8_t)(1U << (*p_bit % 8));
        }
    }
}

static uint32_t get (const uint8_t * p_p, unsigned * p_bit, unsigned width)
{
    uint32_t value_v = 0;
    for (unsigned value_i = 0; value_i < width; value_i++, (*p_bit)++)
    {
        if (p_p[*p_bit / 8] & (1U << (*p_bit % 8)))
        {
            value_v |= (uint32_t)1U << value_i;
        }
    }
    return value_v;
}

static void put_double (uint8_t * p_p, unsigned * p_bit, double value)
{
    union
    {
        double f;
        uint64_t u;
    } value_v;

    value_v.f = value;
    put(p_p, p_bit, (uint32_t)value_v.u, 32);
    put(p_p, p_bit, (uint32_t)(value_v.u >> 32), 32);
}

static double get_double (const uint8_t * p_p, unsigned * p_bit)
{
    union
    {
        double f;
        uint64_t u;
    } value_v;

    uint32_t value_lo = get(p_p, p_bit, 32), value_hi = get(p_p, p_bit, 32);
    value_v.u = (uint64_t)value_lo | ((uint64_t)value_hi << 32);
    return value_v.f;
}

static void put_float (uint8_t * p_p, unsigned * p_bit, float value)
{
    union
    {
        float f;
        uint32_t u;
    } value_v;

    value_v.f = value;
    put(p_p, p_bit, value_v.u, 32);
}

static float get_float (const uint8_t * p_p, unsigned * p_bit)
{
    union
    {
        float f;
        uint32_t u;
    } value_v;

    value_v.u = get(p_p, p_bit, 32);
    return value_v.f;
}

bool s3_control_pack (const s3_control_request_t * p_v,
                      s3_control_packet_t * p_out)
{
    s3_control_packet_t value_p = {{0}};
    unsigned bit = 0;
    bool sensor;
    if (!p_v || !p_out || (unsigned)p_v->kind > S3_CONTROL_RESET)
    {
        return false;
    }
    sensor = p_v->kind == S3_CONTROL_WHITE || p_v->kind == S3_CONTROL_BLACK ||
             p_v->kind == S3_CONTROL_APPLY_CALIBRATION;
    if (sensor && (unsigned)p_v->sensor >= S3_SENSOR_COUNT)
    {
        return false;
    }
    put(value_p.bytes, &bit, p_v->kind, 3);
    put(value_p.bytes, &bit, sensor ? p_v->sensor : 0U, 2);
    for (unsigned value_i = 0; value_i < 26; value_i++)
    {
        unsigned action =
            p_v->kind == S3_CONTROL_MAPPING ? (unsigned)p_v->map[value_i] : 0U;
        if (action >= S3_NAV_ACTION_COUNT)
        {
            return false;
        }
        put(value_p.bytes, &bit, action, 3);
    }
    *p_out = value_p;
    return true;
}

bool s3_control_unpack (const s3_control_packet_t * p_p,
                        s3_control_request_t * p_out)
{
    s3_control_request_t value_v = {0};
    s3_control_packet_t canonical;
    unsigned bit = 0;
    if (!p_p || !p_out || (p_p->bytes[10] & 0xf8U))
    {
        return false;
    }
    value_v.kind = (s3_control_kind_t)get(p_p->bytes, &bit, 3);
    value_v.sensor = (s3_sensor_id_t)get(p_p->bytes, &bit, 2);
    for (unsigned value_i = 0; value_i < 26; value_i++)
    {
        value_v.map[value_i] = (s3_nav_action_t)get(p_p->bytes, &bit, 3);
    }
    if (!s3_control_pack(&value_v, &canonical))
    {
        return false;
    }
    for (unsigned value_i = 0; value_i < sizeof(*p_p); value_i++)
    {
        if (p_p->bytes[value_i] != canonical.bytes[value_i])
        {
            return false;
        }
    }
    *p_out = value_v;
    return true;
}

static bool summary_valid (const s3_calibration_summary_t * p_v)
{
    return p_v->count <= 65535U && p_v->minimum <= 4095U &&
           p_v->maximum <= 4095U;
}

static void put_summary (uint8_t * p_p, unsigned * p_bit,
                         const s3_calibration_summary_t * p_v)
{
    put(p_p, p_bit, p_v->count, 16);
    put(p_p, p_bit, p_v->minimum, 12);
    put(p_p, p_bit, p_v->maximum, 12);
    put_float(p_p, p_bit, p_v->average);
    put(p_p, p_bit, p_v->available, 1);
}

static void get_summary (const uint8_t * p_p, unsigned * p_bit,
                         s3_calibration_summary_t * p_v)
{
    p_v->count = get(p_p, p_bit, 16);
    p_v->minimum = (uint16_t)get(p_p, p_bit, 12);
    p_v->maximum = (uint16_t)get(p_p, p_bit, 12);
    p_v->average = get_float(p_p, p_bit);
    p_v->available = get(p_p, p_bit, 1) != 0;
}

bool s3_status_pack (const s3_integration_status_t * p_v,
                     s3_status_packet_t * p_out)
{
    s3_status_packet_t value_p = {0};
    unsigned bit = 0;
    if (!p_v || !p_out || (unsigned)p_v->state > S3_DRIVE_FAULT ||
        (unsigned)p_v->reason > S3_DRIVE_TERRAIN_FAULT ||
        (unsigned)p_v->motion.state > MOTION_FAULT ||
        (unsigned)p_v->motion.fault > MOTION_FAULT_TIMEOUT ||
        (unsigned)p_v->calibration_sensor >= S3_SENSOR_COUNT ||
        !summary_valid(&p_v->white) || !summary_valid(&p_v->black) ||
        !s3_navigation_pack(&p_v->navigation, &value_p.navigation))
    {
        return false;
    }
    put_double(value_p.feedback, &bit, p_v->motion.centre_m);
    put(value_p.feedback, &bit, p_v->motion.timestamp_us, 32);
    put(value_p.feedback, &bit, p_v->motion.epoch, 32);
    put(value_p.feedback, &bit, p_v->motion.state, 3);
    put(value_p.feedback, &bit, p_v->motion.fault, 3);
    put(value_p.feedback, &bit, p_v->motion.valid, 1);
    put(value_p.feedback, &bit, p_v->motion.braked, 1);
    put(value_p.feedback, &bit, p_v->motion.command_id, 8);
    put(value_p.feedback, &bit, p_v->motion.report_loss_count, 32);
    bit = 0;
    put(value_p.metadata, &bit, p_v->state, 3);
    put(value_p.metadata, &bit, p_v->reason, 4);
    put(value_p.metadata, &bit, p_v->samples, 32);
    put(value_p.metadata, &bit, p_v->max_gap_us, 32);
    put(value_p.metadata, &bit, p_v->above_target, 32);
    put(value_p.metadata, &bit, p_v->command_sequence, 32);
    put(value_p.metadata, &bit, p_v->command_ok, 1);
    put(value_p.metadata, &bit, p_v->collecting, 1);
    put(value_p.metadata, &bit, p_v->calibration_sensor, 2);
    put_summary(value_p.metadata, &bit, &p_v->white);
    put_summary(value_p.metadata, &bit, &p_v->black);
    put(value_p.metadata, &bit, (uint32_t)p_v->adc_status, 32);
    put(value_p.metadata, &bit, p_v->run_timing.started_us, 32);
    put(value_p.metadata, &bit, p_v->run_timing.samples, 32);
    put(value_p.metadata, &bit, p_v->run_timing.max_gap_us, 32);
    put(value_p.metadata, &bit, p_v->run_timing.above_target, 32);
    put(value_p.metadata, &bit, p_v->run_timing.above_safety, 32);
    put(value_p.metadata, &bit, p_v->run_timing.last_safety_us, 32);
    *p_out = value_p;
    return true;
}

bool s3_status_unpack (const s3_status_packet_t * p_p,
                       s3_integration_status_t * p_out)
{
    s3_integration_status_t value_v = {0};
    unsigned bit = 0;
    uint32_t error;
    if (!p_p || !p_out || (p_p->metadata[63] & 0xe0U) ||
        !s3_navigation_unpack(&p_p->navigation, &value_v.navigation))
    {
        return false;
    }
    value_v.motion.centre_m = get_double(p_p->feedback, &bit);
    value_v.motion.timestamp_us = get(p_p->feedback, &bit, 32);
    value_v.motion.epoch = get(p_p->feedback, &bit, 32);
    value_v.motion.state = (motion_state_t)get(p_p->feedback, &bit, 3);
    value_v.motion.fault = (motion_fault_t)get(p_p->feedback, &bit, 3);
    value_v.motion.valid = get(p_p->feedback, &bit, 1) != 0;
    value_v.motion.braked = get(p_p->feedback, &bit, 1) != 0;
    value_v.motion.command_id = (uint8_t)get(p_p->feedback, &bit, 8);
    value_v.motion.report_loss_count = get(p_p->feedback, &bit, 32);
    if (value_v.motion.state > MOTION_FAULT ||
        value_v.motion.fault > MOTION_FAULT_TIMEOUT)
    {
        return false;
    }
    bit = 0;
    value_v.state = (s3_drive_state_t)get(p_p->metadata, &bit, 3);
    value_v.reason = (s3_drive_reason_t)get(p_p->metadata, &bit, 4);
    if (value_v.state > S3_DRIVE_FAULT ||
        value_v.reason > S3_DRIVE_TERRAIN_FAULT)
    {
        return false;
    }
    value_v.samples = get(p_p->metadata, &bit, 32);
    value_v.max_gap_us = get(p_p->metadata, &bit, 32);
    value_v.above_target = get(p_p->metadata, &bit, 32);
    value_v.command_sequence = get(p_p->metadata, &bit, 32);
    value_v.command_ok = get(p_p->metadata, &bit, 1) != 0;
    value_v.collecting = get(p_p->metadata, &bit, 1) != 0;
    value_v.calibration_sensor = (s3_sensor_id_t)get(p_p->metadata, &bit, 2);
    if (value_v.calibration_sensor >= S3_SENSOR_COUNT)
    {
        return false;
    }
    get_summary(p_p->metadata, &bit, &value_v.white);
    get_summary(p_p->metadata, &bit, &value_v.black);
    error = get(p_p->metadata, &bit, 32);
    value_v.adc_status = error <= INT32_MAX
                             ? (int32_t)error
                             : -1 - (int32_t)(UINT32_MAX - error);
    value_v.run_timing.started_us = get(p_p->metadata, &bit, 32);
    value_v.run_timing.samples = get(p_p->metadata, &bit, 32);
    value_v.run_timing.max_gap_us = get(p_p->metadata, &bit, 32);
    value_v.run_timing.above_target = get(p_p->metadata, &bit, 32);
    value_v.run_timing.above_safety = get(p_p->metadata, &bit, 32);
    value_v.run_timing.last_safety_us = get(p_p->metadata, &bit, 32);
    *p_out = value_v;
    return true;
}
