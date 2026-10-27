#ifndef BUDDY4_IMU_SENSOR_H
#define BUDDY4_IMU_SENSOR_H
#include <stdbool.h>
#include <stdint.h>
#define BUDDY4_I2C_UNIT 1
#define BUDDY4_SDA_PIN 2
#define BUDDY4_SCL_PIN 3
/* Native sensor axes are rotated to vehicle X forward, Y left, Z up:
 * vehicle = {-sensor Y, sensor X, sensor Z}. No gyro is present. */
typedef struct {
    float accel_g[3], mag_gauss[3];
    uint32_t timestamp_us;
    bool valid, mag_valid, overrun;
} imu_sample_t;
typedef bool (*imu_read_fn)(void *, uint8_t, uint8_t, uint8_t *, unsigned);
typedef bool (*imu_write_fn)(void *, uint8_t, uint8_t, uint8_t);
typedef struct { void *context; imu_read_fn read; imu_write_fn write; } imu_bus_t;
bool imu_sample_is_valid(const imu_sample_t *sample);
bool imu_sensor_init(const imu_bus_t *bus);
/* Returns true for a NEW acceleration sample. mag_valid only for NEW mag data. */
bool imu_sensor_read(const imu_bus_t *bus, uint32_t now_us, imu_sample_t *out);
#endif
