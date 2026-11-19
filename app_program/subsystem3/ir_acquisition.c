/** @file ir_acquisition.c
 * @brief Buddy 3: portable acquisition boundary; one peripheral owner.
 */
#include "ir_acquisition.h"

bool s3_acquisition_config_valid (const s3_acquisition_config_t * p_config)
{
    uint8_t seen = 0U;
    unsigned int value_i;
    if ((p_config == 0) || (p_config->enabled_mask == 0U) ||
        ((p_config->enabled_mask & ~S3_ALL_SENSORS) != 0U))
    {
        return false;
    }
    for (value_i = 0U; value_i < S3_SENSOR_COUNT; value_i++)
    {
        if (p_config->channel[value_i] > 2U)
        {
            return false;
        }
        if ((p_config->enabled_mask & S3_SENSOR_BIT(value_i)) != 0U)
        {
            uint8_t bit = (uint8_t)(1U << p_config->channel[value_i]);
            if ((seen & bit) != 0U)
            {
                return false;
            }
            seen |= bit;
        }
    }
    return true;
}

bool s3_acquisition_capture (const s3_acquisition_config_t * p_config,
                             s3_adc_read_fn p_read_adc, s3_clock_fn p_clock_us,
                             void * p_context, s3_sensor_frame_t * p_frame)
{
    unsigned int value_i;
    if (p_frame == 0)
    {
        return false;
    }
    *p_frame = (s3_sensor_frame_t){0};
    if (!s3_acquisition_config_valid(p_config) || (p_read_adc == 0) ||
        (p_clock_us == 0))
    {
        return false;
    }
    for (value_i = 0U; value_i < S3_SENSOR_COUNT; value_i++)
    {
        s3_raw_sample_t * p_sample = &p_frame->sensor[value_i];
        p_sample->enabled =
            (p_config->enabled_mask & S3_SENSOR_BIT(value_i)) != 0U;
        if (p_sample->enabled)
        {
            p_sample->error = p_read_adc(p_context, p_config->channel[value_i],
                                         &p_sample->raw);
            p_sample->timestamp_us = p_clock_us(p_context);
            p_sample->valid =
                (p_sample->error == 0) && (p_sample->raw <= 4095U);
            if (!p_sample->valid)
            {
                if (p_sample->error == 0)
                {
                    p_sample->error = -1;
                }
                p_sample->raw = 0U;
            }
        }
    }
    p_frame->timestamp_us = p_clock_us(p_context);
    return true; /* Individual failures are represented in the frame. */
}

/* End of file. */

