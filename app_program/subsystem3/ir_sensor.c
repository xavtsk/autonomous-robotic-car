/* Buddy 3: only this adapter touches the kernel ADC and physical pads. */
#include "ir_sensor.h"
#include <tk/tkernel.h>
#include <sys/sysdef.h>
static ID g_h_adc;
static s3_acquisition_config_t g_config;
int32_t ir_sensor_init(const s3_acquisition_config_t *config)
{
    unsigned int i;
    ID handle;
    if (!s3_acquisition_config_valid(config)) { return (int32_t)E_PAR; }
    if (g_h_adc > 0) { return (int32_t)E_OBJ; }
    handle = tk_opn_dev((const UB *)"adca", TD_READ | TD_EXCL);
    if (handle < E_OK) { return (int32_t)handle; }
    for (i = 0U; i < S3_SENSOR_COUNT; i++) {
        if ((config->enabled_mask & S3_SENSOR_BIT(i)) != 0U) {
            uint32_t pin = 26U + config->channel[i];
            out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
            clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);
        }
    }
    g_config = *config;
    g_h_adc = handle;
    return (int32_t)E_OK;
}
static int32_t read_channel(void *context, uint8_t channel, uint16_t *raw)
{
    UW sample = 0U;
    SZ actual = 0;
    ER result;
    (void)context;
    result = tk_srea_dev(g_h_adc, channel, &sample, 1, &actual);
    if (result < E_OK) { return (int32_t)result; }
    if ((actual != 1) || (sample > 4095U)) { return (int32_t)E_IO; }
    *raw = (uint16_t)sample;
    return (int32_t)E_OK;
}
static uint32_t clock_us(void *context)
{
    (void)context;
    return in_w(TIMER_TIMERAWL);
}
int32_t ir_sensor_read_frame(s3_sensor_frame_t *frame)
{
    if (frame == 0) { return (int32_t)E_PAR; }
    *frame = (s3_sensor_frame_t){0};
    if (g_h_adc <= 0) { return (int32_t)E_OBJ; }
    return s3_acquisition_capture(&g_config, read_channel, clock_us, 0, frame)
        ? (int32_t)E_OK : (int32_t)E_PAR;
}
void ir_sensor_close(void)
{
    if (g_h_adc > 0) { (void)tk_cls_dev(g_h_adc, 0); g_h_adc = 0; }
}
