/** @file imu_sensor.c
 * @brief Configure and read the LSM303DLHC accelerometer and magnetometer
 * through the bus interface.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 * Byte and half-word values fit in int, and uint32_t fits in int64_t.
 * The casts make the existing arithmetic promotions explicit.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 */
/* STMicroelectronics LSM303DLHC datasheet, DocID018771 Rev 2.
 * This driver uses the datasheet register map and 7-bit I2C addresses. */
#include "imu_sensor.h"
#include <math.h>

#define IMU_ACCEL_DATA_READY (0x08U)
#define IMU_ACCEL_OVERRUN (0x80U)
#define IMU_MAG_DATA_OR_LOCK (0x03U)

static int32_t signed16 (uint8_t value_hi, uint8_t value_lo);

/**
 * @brief Validate enabled fields without checking sample age or physical range.
 * @param[in] p_sample Borrowed sample; NULL returns false.
 * @return The result specified by imu_sensor_sample_is_valid() in imu_sensor.h.
 */
bool imu_sensor_sample_is_valid (const imu_sensor_sample_t * p_sample)
{

    if ((0 == p_sample) || (!p_sample->b_is_valid))
    {
        return false;
    }

    unsigned axis_index;
    for (axis_index = 0; axis_index < (unsigned int)IMU_AXIS_COUNT;
         ++axis_index)
    {
        if (!isfinite(p_sample->accel_g[axis_index]))
        {
            return false;
        }

        if ((p_sample->b_is_mag_valid) &&
            (!isfinite(p_sample->mag_gauss[axis_index])))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief Apply the bus/device configuration contract in imu_sensor.h.
 * @param[in] p_bus Borrowed bus; the caller serializes its synchronous
 * callbacks.
 * @return True on final matching readback; false retains any earlier writes.
 */
bool imu_sensor_init (const imu_sensor_bus_t * p_bus)
{
    uint8_t device_id[3];

    if (((0 == p_bus) || (0 == p_bus->p_read)) || (0 == p_bus->p_write))
    {
        return false;
    }
    /* Magnetometer identity H43. Accel reserved WHO_AM_I is not relied upon. */
    if ((((!p_bus->p_read(p_bus->p_context, 0x1e, 0x0a, device_id, 3)) ||
          ((int)device_id[0] != 0x48)) ||
         ((int)device_id[1] != 0x34)) ||
        ((int)device_id[2] != 0x33))
    {
        return false;
    }
    /* 50 Hz acceleration, BDU, high resolution, +/-4g (2 mg/LSB).
     * Magnetometer 30 Hz, +/-1.3 gauss, continuous conversion. */
    if ((((((!p_bus->p_write(p_bus->p_context, 0x19, 0x20, 0x47)) ||
            (!p_bus->p_write(p_bus->p_context, 0x19, 0x21, 0x00))) ||
           (!p_bus->p_write(p_bus->p_context, 0x19, 0x23, 0x98))) ||
          (!p_bus->p_write(p_bus->p_context, 0x1e, 0x00, 0x14))) ||
         (!p_bus->p_write(p_bus->p_context, 0x1e, 0x01, 0x20))) ||
        (!p_bus->p_write(p_bus->p_context, 0x1e, 0x02, 0x00)))
    {
        return false;
    }

    uint8_t accel_config;
    return (p_bus->p_read(p_bus->p_context, 0x19, 0x23, &accel_config, 1)) &&
           (0x98 == (int)accel_config);
}

/**
 * @brief Acquire one sample with the partial-output contract in imu_sensor.h.
 * @param[in] p_bus Borrowed bus; missing read callback returns false.
 * @param now_us Caller timestamp in microseconds, including wraparound values.
 * @param[out] p_out Caller-owned sample; NULL returns false without bus access.
 * @return True for accepted acceleration, independently of magnetic validity.
 */
bool imu_sensor_read (const imu_sensor_bus_t * p_bus, uint32_t now_us,
                      imu_sensor_sample_t * p_out)
{
    uint8_t raw[6];
    uint8_t status;

    if (0 == p_out)
    {
        return false;
    }

    *p_out              = (imu_sensor_sample_t){0};
    p_out->timestamp_us = now_us;
    /* Widen the 0..255 register value before masking its status bits. */
    if ((((0 == p_bus) || (0 == p_bus->p_read)) ||
         (!p_bus->p_read(p_bus->p_context, 0x19, 0x27, &status, 1))) ||
        (0U == ((uint32_t)status & IMU_ACCEL_DATA_READY)))
    {
        return false;
    }
    /* Widen the same 0..255 register value for the overrun mask. */
    p_out->b_has_overrun = (((uint32_t)status & IMU_ACCEL_OVERRUN) != 0U);

    if (!p_bus->p_read(p_bus->p_context, 0x19, 0xa8, raw, 6))
    {
        return false;
    }

    int32_t raw_x = signed16(raw[1], raw[0]) / 16;
    int32_t raw_y = signed16(raw[3], raw[2]) / 16;
    int32_t raw_z = signed16(raw[5], raw[4]) / 16;
    /* Reject rail saturation, preserving negative samples. */
    if ((((((raw_x <= -2048) || (raw_x >= 2047)) || (raw_y <= -2048)) ||
          (raw_y >= 2047)) ||
         (raw_z <= -2048)) ||
        (raw_z >= 2047))
    {
        return false;
    }

    p_out->accel_g[0] = -raw_y * 0.002f;
    p_out->accel_g[1] = raw_x * 0.002f;
    p_out->accel_g[2] = raw_z * 0.002f;
    p_out->b_is_valid = true;

    /* Widen the successfully read 0..255 magnetic status before masking. */
    if (((p_bus->p_read(p_bus->p_context, 0x1e, 0x09, &status, 1)) &&
         (((uint32_t)status & IMU_MAG_DATA_OR_LOCK) != 0U)) &&
        (p_bus->p_read(p_bus->p_context, 0x1e, 0x03, raw, 6)))
    {
        /* Mag byte order is big endian X,Z,Y; a full burst releases LOCK. */
        raw_x = signed16(raw[0], raw[1]);
        raw_z = signed16(raw[2], raw[3]);
        raw_y = signed16(raw[4], raw[5]);

        if ((((((raw_x >= -2048) && (raw_x <= 2047)) && (raw_y >= -2048)) &&
              (raw_y <= 2047)) &&
             (raw_z >= -2048)) &&
            (raw_z <= 2047))
        {
            p_out->mag_gauss[0]   = -raw_y / 1100.0f;
            p_out->mag_gauss[1]   = raw_x / 1100.0f;
            p_out->mag_gauss[2]   = raw_z / 980.0f;
            p_out->b_is_mag_valid = true;
        }
    }

    return true;
}

/**
 * @brief Decode two bytes into a sign-extended 16-bit value in int32_t.
 * @param value_hi Most significant byte, 0..255.
 * @param value_lo Least significant byte, 0..255.
 * @return Value in -32768..32767; neither input nor external state is changed.
 */
static int32_t signed16 (uint8_t value_hi, uint8_t value_lo)
{
    /* Widen the 0..255 high byte before shifting its bits into positions
     * 8..15. The uint32_t register value is then 0..65535. */
    const uint32_t value = ((uint32_t)value_hi << 8) | value_lo;

    /* Every register value fits in int32_t. Subtract 65536 in the signed
     * domain when bit 15 is set, yielding -32768..-1 for that half. */
    return (value & 0x8000U) ? (int32_t)value - 65536 : (int32_t)value;
}

/* End of file. */

