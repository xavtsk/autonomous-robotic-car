#include "ir_acquisition.h"
bool s3_acquisition_config_valid(const s3_acquisition_config_t *config)
{
    uint8_t seen = 0U;
    unsigned int i;
    if ((config == 0) || (config->enabled_mask == 0U) ||
        ((config->enabled_mask & ~S3_ALL_SENSORS) != 0U)) { return false; }
    for (i = 0U; i < S3_SENSOR_COUNT; i++) {
        if (config->channel[i] > 2U) { return false; }
        if ((config->enabled_mask & S3_SENSOR_BIT(i)) != 0U) {
            uint8_t bit = (uint8_t)(1U << config->channel[i]);
            if ((seen & bit) != 0U) { return false; }
            seen |= bit;
        }
    }
    return true;
}
bool s3_acquisition_capture(const s3_acquisition_config_t *config,
                            s3_adc_read_fn read_adc, s3_clock_fn clock_us,
                            void *context, s3_sensor_frame_t *frame)
{
    unsigned int i;
    if (frame == 0) { return false; }
    *frame = (s3_sensor_frame_t){0};
    if (!s3_acquisition_config_valid(config) || (read_adc == 0) || (clock_us == 0)) {
        return false;
    }
    for (i = 0U; i < S3_SENSOR_COUNT; i++) {
        s3_raw_sample_t *sample = &frame->sensor[i];
        sample->enabled = (config->enabled_mask & S3_SENSOR_BIT(i)) != 0U;
        if (sample->enabled) {
            sample->error = read_adc(context, config->channel[i], &sample->raw);
            sample->timestamp_us = clock_us(context);
            sample->valid = (sample->error == 0) && (sample->raw <= 4095U);
            if (!sample->valid) {
                if (sample->error == 0) { sample->error = -1; }
                sample->raw = 0U;
            }
        }
    }
    frame->timestamp_us = clock_us(context);
    return true; /* Individual failures are represented in the frame. */
}
