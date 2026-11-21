/** @file ir_sensor.c
 * @brief Buddy 3: only this adapter touches the kernel ADC and physical pads.
 * The hardware masks fit in unsigned. Convert them before combining
 * register or packet bits.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 */
#include "ir_sensor.h"
#include <tk/tkernel.h>
#include <sys/sysdef.h>
#include "../monotonic_clock.h"
static ID                      gh_adc;
static ir_acquisition_stored_t g_config;

static int32_t  read_channel (void * p_context, uint8_t channel,
                              uint16_t * p_raw);
static uint32_t clock_us (void * p_context);

/** @brief Validate the ADC mapping, open the exclusive read device, configure
 * enabled analog GPIOs and retain the mapping. Invalid configuration
 * returns E_PAR and a repeated open returns E_OBJ.
 */
int32_t ir_sensor_init (const ir_acquisition_config_t * p_config)
{

    if (!ir_acquisition_config_valid(p_config))
    {
        return E_PAR;
    }

    if (gh_adc > 0)
    {
        return E_OBJ;
    }

    monotonic_clock_start();
    ID h_adc = tk_opn_dev((const UB *)"adca", TD_READ | TD_EXCL);

    if (h_adc < E_OK)
    {
        return (int32_t)h_adc;
    }

    unsigned int sensor_index;
    for (sensor_index = 0U; sensor_index < (unsigned int)S3_SENSOR_COUNT;
         sensor_index++)
    {
        if ((p_config->enabled_mask & S3_SENSOR_BIT(sensor_index)) != 0U)
        {
            const uint32_t pin = 26U + p_config->channel[sensor_index];
            out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
            clr_w(GPIO(pin),
                  (unsigned)GPIO_IE | (unsigned)GPIO_PUE | (unsigned)GPIO_PDE);
        }
    }

    /* clang-format off */
    g_config =
        (ir_acquisition_stored_t){p_config->channel[0], p_config->channel[1],
                                  p_config->channel[2], p_config->enabled_mask};
    gh_adc   = h_adc;
    /* clang-format on */
    return E_OK;
}

/** @brief Clear a non-null output before checking the ADC handle. Return E_OBJ
 * when closed. Otherwise capture enabled channels and return the
 * acquisition result as E_OK or E_PAR.
 */
int32_t ir_sensor_read_frame (ir_acquisition_frame_t * p_frame)
{
    if (0 == p_frame)
    {
        return E_PAR;
    }

    *p_frame = (ir_acquisition_frame_t){0};

    if (gh_adc <= 0)
    {
        return E_OBJ;
    }

    const ir_acquisition_config_t config = ir_acquisition_expand(&g_config);
    return ir_acquisition_capture(&config, read_channel, clock_us, 0, p_frame)
               ? E_OK
               : E_PAR;
}

/** @brief Close an open ADC device and clear its handle. A closed device
 * requires no action.
 */
void ir_sensor_close (void)
{
    if (gh_adc > 0)
    {
        (void)tk_cls_dev(gh_adc, 0);
        gh_adc = 0;
    }
}

/** @brief Read one 12-bit ADC sample from the selected channel. A device
 * error, short read or value above 4095 leaves the caller's raw output
 * untouched. The context is unused.
 */
static int32_t read_channel (void * p_context, uint8_t channel,
                             uint16_t * p_raw)
{
    UW sample = 0U;
    SZ actual = 0;
    (void)p_context;
    ER result = tk_srea_dev(gh_adc, channel, &sample, 1, &actual);

    if (result < E_OK)
    {
        return (int32_t)result;
    }

    if ((actual != 1) || (sample > 4095U))
    {
        return E_IO;
    }

    /* The completed read has already bounded the ADC count to 0..4095. */
    *p_raw = (uint16_t)sample;
    return E_OK;
}

/** @brief Read the low 32 bits of the hardware timer in microseconds. The
 * acquisition callback context is unused.
 */
static uint32_t clock_us (void * p_context)
{
    (void)p_context;
    return in_w(TIMER_TIMERAWL);
}

/* End of file. */

