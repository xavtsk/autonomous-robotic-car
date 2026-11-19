/** @file imu_sensor.h
 * @brief Declare the IMU bus boundary and vehicle-axis acceleration and
 * magnetic samples.
 */
#ifndef BUDDY4_IMU_SENSOR_H
#define BUDDY4_IMU_SENSOR_H
#include "../app_types.h"
#include <stdbool.h>
#include <stdint.h>
#define BUDDY4_I2C_UNIT 1
#define BUDDY4_SDA_PIN 2
#define BUDDY4_SCL_PIN 3

/* Native sensor axes are rotated to vehicle X forward, Y left, Z up:
 * vehicle = {-sensor Y, sensor X, sensor Z}. No gyro is present. */
typedef struct
{
    float32_t accel_g[3], mag_gauss[3];
    uint32_t timestamp_us;
    bool valid, mag_valid, overrun;
} imu_sample_t;

typedef bool (*imu_read_fn)(void *, uint8_t, uint8_t, uint8_t *, unsigned);
typedef bool (*imu_write_fn)(void *, uint8_t, uint8_t, uint8_t);

typedef struct
{
    void * context;
    imu_read_fn read;
    imu_write_fn write;
} imu_bus_t;

/**
 * @brief Check the sample validity flag and finite enabled measurements.
 * @param[in] p_sample Borrowed sample, or NULL to return false.
 * @return True for a valid sample with finite acceleration on all three axes
 * and, when mag_valid is set, finite magnetic readings on all three axes.
 * @note Does not check age, physical range or overrun. Reads no bus and changes
 * no state; the caller must prevent concurrent writes to the sample.
 */
bool imu_sample_is_valid (const imu_sample_t * p_sample);
/**
 * @brief Configure the sensor using the existing ordered bus transactions.
 * @param[in] p_bus Borrowed synchronous bus callbacks/context; NULL or a
 * missing read/write callback returns false without a bus transaction.
 * @return True after identity, configuration writes and final readback succeed.
 * @note Failure stops at the first rejected transaction or identity/readback
 * mismatch. Earlier device writes are not rolled back. The caller serializes
 * bus/device access and keeps callbacks/context valid throughout the call.
 */
bool imu_sensor_init (const imu_bus_t * p_bus);
/* Returns true for a NEW acceleration sample. mag_valid only for NEW mag data.
 */
/**
 * @brief Read acceleration and opportunistically read magnetic data.
 * @param[in] p_bus Borrowed synchronous read callback/context; may be NULL.
 * @param now_us Caller timestamp in microseconds, copied without an age check.
 * @param[out] p_out Writable sample, or NULL to fail without accessing the bus.
 * @return True when a new, nonsaturated acceleration sample was read. Missing
 * or rejected magnetic data leaves mag_valid false without failing the read.
 * @note A non-NULL output is zeroed and timestamped before bus validation.
 * Failure may also leave the overrun flag set. Acceleration uses g, magnetic
 * readings use gauss, and axes are vehicle X forward, Y left, Z up. Calls do
 * not retain pointers; the caller serializes bus and output access.
 */
bool imu_sensor_read (const imu_bus_t * p_bus, uint32_t now_us,
                      imu_sample_t * p_out);
#endif

/* End of file. */

