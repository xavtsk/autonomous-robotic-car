/** @file ir_acquisition.h
 * @brief Buddy 3: portable acquisition boundary; one peripheral owner.
 */
#ifndef BUDDY3_IR_ACQUISITION_H
#define BUDDY3_IR_ACQUISITION_H
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    S3_SENSOR_BARCODE = 0,
    S3_SENSOR_LEFT,
    S3_SENSOR_RIGHT,
    S3_SENSOR_COUNT
} ir_acquisition_sensor_id_t;

#define S3_SENSOR_BIT(id) (1U << (id))
#define S3_ALL_SENSORS (7U)
/* The 27 October wiring check mapped front-left to Grove 6 GP27/ADC1,
 * front-right to Grove 5 GP26/ADC0, and barcode to Grove 7 GP28/ADC2.
 * The controller, bench and operator prompts share this mapping. See
 * app_program/subsystem3/INTEGRATION.md for the wiring record and the
 * Robo Pico Board Datasheet.pdf for connector and jumper assignments. */
#define S3_ADC_CHANNEL_LEFT (1U) /* GP27 */
/* The Robo Pico VBAT jumper setting affects use of GP28 as an input. */
#define S3_ADC_CHANNEL_BARCODE (2U)
#define S3_ADC_CHANNEL_RIGHT (0U) /* GP26 */
#define S3_GPIO_FOR_CHANNEL(channel) (26U + (unsigned)(channel))

typedef struct
{
    uint8_t channel[S3_SENSOR_COUNT]; /* ADC0/1/2 = GP26/27/28 only. */
    uint8_t enabled_mask;
} ir_acquisition_config_t;

/* Retained only after validating the wide configuration above. */
typedef struct __attribute__((packed))
{
    uint16_t barcode_channel : 2;
    uint16_t left_channel : 2;
    uint16_t right_channel : 2;
    uint16_t enabled_mask : 3;
} ir_acquisition_stored_t;

/** @brief Expand the retained channel bits into a wide validation structure.
 * @param p_stored Nonnull readable retained configuration owned by the caller.
 * @return A value copy of channels and enabled mask, without validation.
 * @note The copy preserves the original storage and performs no I/O.
 * Previously accepted storage yields the original configuration. Arbitrary
 * stored bits can represent an invalid configuration.
 */
static inline ir_acquisition_config_t
ir_acquisition_expand (const ir_acquisition_stored_t * p_stored)
{
    const ir_acquisition_config_t value = {{p_stored->barcode_channel,
                                            p_stored->left_channel,
                                            p_stored->right_channel},
                                           p_stored->enabled_mask};
    return value;
}

typedef struct
{
    uint32_t timestamp_us;
    uint16_t raw;
    int32_t  error;
    bool     b_is_enabled;
    bool     b_is_valid;
} ir_acquisition_sample_t;

typedef struct
{
    uint32_t timestamp_us; /* End of scan; channels are sequential. */
    ir_acquisition_sample_t sensor[S3_SENSOR_COUNT];
} ir_acquisition_frame_t;

typedef int32_t ( * ir_acquisition_read_fn_t)(void * p_context, uint8_t channel,
                                            uint16_t * p_raw);
typedef uint32_t ( * ir_acquisition_clock_fn_t)(void * p_context);
/** @brief Validate every channel and the enabled-channel assignment.
 * @param p_config Readable configuration, or null for rejection. All three
 * channel values must be 0..2, even when disabled; enabled channels must be
 * distinct and enabled_mask must be a nonempty subset of S3_ALL_SENSORS.
 * @return True exactly for that domain; no configuration fields are changed.
 * @note Pure check. The acquisition owner supplies stable input storage.
 */
bool ir_acquisition_config_valid (const ir_acquisition_config_t * p_config);
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
                             void *                          p_context,
                             ir_acquisition_frame_t *        p_frame);
#endif

/* End of file. */

