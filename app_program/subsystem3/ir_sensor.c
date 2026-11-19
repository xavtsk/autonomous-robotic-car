/* Buddy 3: only this adapter touches the kernel ADC and physical pads. */
#include "ir_sensor.h"
#include <tk/tkernel.h>
#include <sys/sysdef.h>
#include "../monotonic_clock.h"
static ID g_h_adc;
static s3_acquisition_stored_config_t g_config;

int32_t ir_sensor_init (const s3_acquisition_config_t * p_config)
{
    unsigned int value_i;
    ID handle;
    if (!s3_acquisition_config_valid(p_config))
    {
        return (int32_t)E_PAR;
    }
    if (g_h_adc > 0)
    {
        return (int32_t)E_OBJ;
    }
    app_clock_start();
    handle = tk_opn_dev((const UB *)"adca", TD_READ | TD_EXCL);
    if (handle < E_OK)
    {
        return (int32_t)handle;
    }
    for (value_i = 0U; value_i < S3_SENSOR_COUNT; value_i++)
    {
        if ((p_config->enabled_mask & S3_SENSOR_BIT(value_i)) != 0U)
        {
            uint32_t pin = 26U + p_config->channel[value_i];
            out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
            clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);
        }
    }
    g_config = (s3_acquisition_stored_config_t){
        p_config->channel[0], p_config->channel[1], p_config->channel[2],
        p_config->enabled_mask};
    g_h_adc = handle;
    return (int32_t)E_OK;
}

static int32_t read_channel (void * p_context, uint8_t channel,
                             uint16_t * p_raw)
{
    UW sample = 0U;
    SZ actual = 0;
    ER result;
    (void)p_context;
    result = tk_srea_dev(g_h_adc, channel, &sample, 1, &actual);
    if (result < E_OK)
    {
        return (int32_t)result;
    }
    if ((actual != 1) || (sample > 4095U))
    {
        return (int32_t)E_IO;
    }
    *p_raw = (uint16_t)sample;
    return (int32_t)E_OK;
}

static uint32_t clock_us (void * p_context)
{
    (void)p_context;
    return in_w(TIMER_TIMERAWL);
}

int32_t ir_sensor_read_frame (s3_sensor_frame_t * p_frame)
{
    if (p_frame == 0)
    {
        return (int32_t)E_PAR;
    }
    *p_frame = (s3_sensor_frame_t){0};
    if (g_h_adc <= 0)
    {
        return (int32_t)E_OBJ;
    }
    const s3_acquisition_config_t config = s3_acquisition_expand(&g_config);
    return s3_acquisition_capture(&config, read_channel, clock_us, 0, p_frame)
               ? (int32_t)E_OK
               : (int32_t)E_PAR;
}

void ir_sensor_close (void)
{
    if (g_h_adc > 0)
    {
        (void)tk_cls_dev(g_h_adc, 0);
        g_h_adc = 0;
    }
}
