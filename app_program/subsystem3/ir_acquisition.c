/** @file ir_acquisition.c
 * @brief Buddy 3: portable acquisition boundary; one peripheral owner.
 */
#include "ir_acquisition.h"

/** @brief Validate every channel and the enabled-channel assignment.
 * @param p_config Readable configuration, or null for rejection. All three
 * channel values must be 0..2, even when disabled; enabled channels must be
 * distinct and enabled_mask must be a nonempty subset of S3_ALL_SENSORS.
 * @return True exactly for that domain; no configuration fields are changed.
 * @note Pure check. The acquisition owner supplies stable input storage.
 */
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

/** @brief Read enabled ADC channels sequentially, preserving per-read errors.
 * @param p_config Configuration accepted by s3_acquisition_config_valid.
 * @param p_read_adc Callback returning zero on success; writes a raw ADC count.
 * @param p_clock_us Callback returning the current uint32 microsecond clock.
 * @param p_context Opaque caller-owned context passed unchanged to callbacks.
 * @param p_frame Writable result, or null for immediate rejection.
 * @return False for invalid arguments/configuration. A nonnull result is
 * zeroed before that validation. True means the scan was performed, even
 * when every individual read failed; inspect each sample's valid/error fields.
 * @note Each enabled read is followed by its own clock call, including errors;
 * a final clock call timestamps the frame. Raw values above 4095 become zero
 * with error -1 if the callback reported success. Other read errors are kept.
 * Disabled samples remain zero. The caller owns the peripheral and serializes
 * callbacks; this function provides no locks, retries or retained pointers.
 */
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

