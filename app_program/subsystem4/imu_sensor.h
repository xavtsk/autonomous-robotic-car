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
#define IMU_AXIS_COUNT (3)

/* Samples use vehicle X forward, Y left and Z up. The device adapter maps
 * the native axes to {-sensor Y, sensor X, sensor Z}. The IMU has no gyro. */
typedef struct
{
    float32_t accel_g[3];
    float32_t mag_gauss[3];
    uint32_t  timestamp_us;
    bool      b_is_valid;
    bool      b_is_mag_valid;
    bool      b_has_overrun;
} imu_sensor_sample_t;

/** @brief Read count bytes from an I2C device register into caller storage.
 * Return true when the bus adapter completes the requested transfer.
 */
typedef bool ( * imu_sensor_read_fn_t)(void * p_context, uint8_t address,
                                     uint8_t reg, uint8_t * p_data,
                                     unsigned count);
/** @brief Write one byte to an I2C device register using the supplied context.
 * Return true when the bus adapter completes the write.
 */
typedef bool ( * imu_sensor_write_fn_t)(void * p_context, uint8_t address,
                                      uint8_t reg, uint8_t value);

typedef struct
{
    void *                p_context;
    imu_sensor_read_fn_t  p_read;
    imu_sensor_write_fn_t p_write;
} imu_sensor_bus_t;

/**
 * @brief Check the sample validity flag and finite enabled measurements.
 * @param[in] p_sample Borrowed sample, or NULL to return false.
 * @return True for a valid sample with finite acceleration on all three axes
 * and, when mag_valid is set, finite magnetic readings on all three axes.
 * @note The caller checks age, physical range and overrun, and prevents
 * concurrent sample writes. The validity check has no side effects.
 */
bool imu_sensor_sample_is_valid (const imu_sensor_sample_t * p_sample);
/**
 * @brief Configure the sensor using the existing ordered bus transactions.
 * @param[in] p_bus Borrowed synchronous bus callbacks/context; NULL or a
 * missing read/write callback returns false without a bus transaction.
 * @return True after identity, configuration writes and final readback succeed.
 * @note Failure stops at the first rejected transaction or identity/readback
 * mismatch. Earlier device writes are not rolled back. The caller serializes
 * bus/device access and keeps callbacks/context valid throughout the call.
 */
bool imu_sensor_init (const imu_sensor_bus_t * p_bus);
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
bool imu_sensor_read (const imu_sensor_bus_t * p_bus, uint32_t now_us,
                      imu_sensor_sample_t * p_out);
#endif

/* End of file. */

