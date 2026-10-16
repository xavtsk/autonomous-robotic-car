/* Buddy 3: portable acquisition boundary; one peripheral owner. */
#ifndef BUDDY3_IR_ACQUISITION_H
#define BUDDY3_IR_ACQUISITION_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { S3_SENSOR_BARCODE = 0, S3_SENSOR_LEFT, S3_SENSOR_RIGHT,
               S3_SENSOR_COUNT } s3_sensor_id_t;
#define S3_SENSOR_BIT(id) (1U << (id))
#define S3_ALL_SENSORS (7U)
typedef struct {
    uint8_t channel[S3_SENSOR_COUNT]; /* ADC0/1/2 = GP26/27/28 only. */
    uint8_t enabled_mask;
} s3_acquisition_config_t;
typedef struct {
    uint32_t timestamp_us;
    uint16_t raw;
    int32_t error;
    bool enabled;
    bool valid;
} s3_raw_sample_t;
typedef struct {
    uint32_t timestamp_us; /* End of scan; channels are sequential. */
    s3_raw_sample_t sensor[S3_SENSOR_COUNT];
} s3_sensor_frame_t;
typedef int32_t (*s3_adc_read_fn)(void *context, uint8_t channel, uint16_t *raw);
typedef uint32_t (*s3_clock_fn)(void *context);
bool s3_acquisition_config_valid(const s3_acquisition_config_t *config);
bool s3_acquisition_capture(const s3_acquisition_config_t *config,
                            s3_adc_read_fn read_adc, s3_clock_fn clock_us,
                            void *context, s3_sensor_frame_t *frame);
#endif
