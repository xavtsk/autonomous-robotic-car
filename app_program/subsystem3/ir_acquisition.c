/** @file ir_acquisition.c
 * @brief Buddy 3: portable acquisition boundary; one peripheral owner.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 * Byte and flag masks are widened unsigned before bitwise operations.
 */
#include "ir_acquisition.h"

static inline void clear_invalid_sample (ir_acquisition_sample_t * p_sample);

/** @brief Validate every channel and the enabled-channel assignment.
 * @param p_config Readable configuration, or null for rejection. All three
 * channel values must be 0..2, even when disabled; enabled channels must be
 * distinct and enabled_mask must be a nonempty subset of S3_ALL_SENSORS.
 * @return True exactly for that domain; no configuration fields are changed.
 * @note Pure check. The acquisition owner supplies stable input storage.
 */
bool ir_acquisition_config_valid (const ir_acquisition_config_t * p_config)
{
    uint8_t seen_channel_mask = 0U;

    if (((0 == p_config) || (0U == p_config->enabled_mask)) ||
        ((p_config->enabled_mask & ~S3_ALL_SENSORS) != 0U))
    {
        return false;
    }

    unsigned int sensor_index;
    for (sensor_index = 0U; sensor_index < (unsigned int)S3_SENSOR_COUNT;
         sensor_index++)
    {
        if (p_config->channel[sensor_index] > 2U)
        {
            return false;
        }

        if ((p_config->enabled_mask & S3_SENSOR_BIT(sensor_index)) != 0U)
        {
            /* The channel check bounds the mask to 1, 2, or 4. */
            const uint8_t bit =
                (uint8_t)(1U << p_config->channel[sensor_index]);

            if (((unsigned)seen_channel_mask & (unsigned)bit) != 0U)
            {
                return false;
            }

            /* Retain the union of channel bits zero through two.
             * The combined mask is at most seven before narrowing. */
            seen_channel_mask =
                (uint8_t)((unsigned)seen_channel_mask | (unsigned)bit);
        }
    }

    return true;
}

/** @brief Read enabled ADC channels sequentially, preserving per-read errors.
 * @param p_config Configuration accepted by ir_acquisition_config_valid.
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
bool ir_acquisition_capture (const ir_acquisition_config_t * p_config,
                             ir_acquisition_read_fn_t        p_read_adc,
                             ir_acquisition_clock_fn_t       p_clock_us,
                             void * p_context, ir_acquisition_frame_t * p_frame)
{

    if (0 == p_frame)
    {
        return false;
    }

    *p_frame = (ir_acquisition_frame_t){0};

    if (((!ir_acquisition_config_valid(p_config)) || (0 == p_read_adc)) ||
        (0 == p_clock_us))
    {
        return false;
    }

    unsigned int sensor_index;
    for (sensor_index = 0U; sensor_index < (unsigned int)S3_SENSOR_COUNT;
         sensor_index++)
    {
        ir_acquisition_sample_t * p_sample = &p_frame->sensor[sensor_index];
        p_sample->b_is_enabled =
            (p_config->enabled_mask & S3_SENSOR_BIT(sensor_index)) != 0U;

        if (p_sample->b_is_enabled)
        {
            /* clang-format off */
            p_sample->error        = p_read_adc(
                p_context, p_config->channel[sensor_index], &p_sample->raw);
            p_sample->timestamp_us = p_clock_us(p_context);
            p_sample->b_is_valid   =
                (0 == p_sample->error) && (p_sample->raw <= 4095U);
            /* clang-format on */

            if (!p_sample->b_is_valid)
            {
                clear_invalid_sample(p_sample);
            }
        }
    }

    p_frame->timestamp_us = p_clock_us(p_context);
    return true; /* Individual failures are represented in the frame. */
}

/** @brief Store the invalid sample fallback. Keep the ADC error if present. */
static inline void clear_invalid_sample (ir_acquisition_sample_t * p_sample)
{
    if (0 == p_sample->error)
    {
        p_sample->error = -1;
    }

    p_sample->raw = 0U;
}

/* End of file. */

