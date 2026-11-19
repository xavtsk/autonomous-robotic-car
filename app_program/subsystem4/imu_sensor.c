/** @file imu_sensor.c
 * @brief Configure and read the LSM303DLHC accelerometer and magnetometer
 * through the bus interface.
 */
/* LSM303DLHC, ST DocID018771 Rev 2. 7-bit I2C addresses. */
#include "imu_sensor.h"
#include <math.h>

static int32_t signed16 (uint8_t value_hi, uint8_t value_lo)
{
    uint32_t value = ((uint32_t)value_hi << 8) | value_lo;
    return (value & 0x8000U) ? (int32_t)value - 65536 : (int32_t)value;
}

bool imu_sample_is_valid (const imu_sample_t * p_s)
{
    unsigned value_i;
    if (!p_s || !p_s->valid)
    {
        return false;
    }
    for (value_i = 0; value_i < 3; ++value_i)
    {
        if (!isfinite(p_s->accel_g[value_i]))
        {
            return false;
        }
        if (p_s->mag_valid && !isfinite(p_s->mag_gauss[value_i]))
        {
            return false;
        }
    }
    return true;
}

bool imu_sensor_init (const imu_bus_t * p_b)
{
    uint8_t value_id[3], value;
    if (!p_b || !p_b->read || !p_b->write)
    {
        return false;
    }
    /* Magnetometer identity H43. Accel reserved WHO_AM_I is not relied upon. */
    if (!p_b->read(p_b->context, 0x1e, 0x0a, value_id, 3) ||
        value_id[0] != 0x48 || value_id[1] != 0x34 || value_id[2] != 0x33)
    {
        return false;
    }
    /* 50 Hz acceleration, BDU, high resolution, +/-4g (2 mg/LSB).
     * Magnetometer 30 Hz, +/-1.3 gauss, continuous conversion. */
    if (!p_b->write(p_b->context, 0x19, 0x20, 0x47) ||
        !p_b->write(p_b->context, 0x19, 0x21, 0x00) ||
        !p_b->write(p_b->context, 0x19, 0x23, 0x98) ||
        !p_b->write(p_b->context, 0x1e, 0x00, 0x14) ||
        !p_b->write(p_b->context, 0x1e, 0x01, 0x20) ||
        !p_b->write(p_b->context, 0x1e, 0x02, 0x00))
    {
        return false;
    }
    return p_b->read(p_b->context, 0x19, 0x23, &value, 1) && value == 0x98;
}

bool imu_sensor_read (const imu_bus_t * p_b, uint32_t now, imu_sample_t * p_out)
{
    uint8_t raw[6], status;
    int32_t value_x, value_y, value_z;
    if (!p_out)
    {
        return false;
    }
    *p_out = (imu_sample_t){0};
    p_out->timestamp_us = now;
    if (!p_b || !p_b->read ||
        !p_b->read(p_b->context, 0x19, 0x27, &status, 1) || !(status & 8))
    {
        return false;
    }
    p_out->overrun = (status & 0x80) != 0;
    if (!p_b->read(p_b->context, 0x19, 0xa8, raw, 6))
    {
        return false;
    }
    value_x = signed16(raw[1], raw[0]) / 16;
    value_y = signed16(raw[3], raw[2]) / 16;
    value_z = signed16(raw[5], raw[4]) / 16;
    /* Reject rail saturation, preserving negative samples. */
    if (value_x <= -2048 || value_x >= 2047 || value_y <= -2048 ||
        value_y >= 2047 || value_z <= -2048 || value_z >= 2047)
    {
        return false;
    }
    p_out->accel_g[0] = -value_y * 0.002f;
    p_out->accel_g[1] = value_x * 0.002f;
    p_out->accel_g[2] = value_z * 0.002f;
    p_out->valid = true;
    if (p_b->read(p_b->context, 0x1e, 0x09, &status, 1) && (status & 3) &&
        p_b->read(p_b->context, 0x1e, 0x03, raw, 6))
    {
        /* Mag byte order is big endian X,Z,Y; a full burst releases LOCK. */
        value_x = signed16(raw[0], raw[1]);
        value_z = signed16(raw[2], raw[3]);
        value_y = signed16(raw[4], raw[5]);
        if (value_x >= -2048 && value_x <= 2047 && value_y >= -2048 &&
            value_y <= 2047 && value_z >= -2048 && value_z <= 2047)
        {
            p_out->mag_gauss[0] = -value_y / 1100.0f;
            p_out->mag_gauss[1] = value_x / 1100.0f;
            p_out->mag_gauss[2] = value_z / 980.0f;
            p_out->mag_valid = true;
        }
    }
    return true;
}

/* End of file. */

