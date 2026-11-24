/** @file mag_capture.c
 * @brief Emit bounded USB diagnostics for the mounted car's compass.
 */
#include "mag_capture.h"

#if APP_MAG_CAL_CAPTURE
#include <stddef.h>
#include <tm/tmonitor.h>

/* USB-only BSP API. The producer appends to a bounded ring. TinyUSB runs
 * in its own service task. Link state three means a host asserted DTR. */
extern UW   tm_usb_state (void);
extern void tm_usb_console_put (const UB * p_buffer, INT size);

/** @brief Format each accepted vector into a single best-effort USB write.
 * @see mag_capture.h for valid inputs, units, rate and task ownership.
 */
void mag_capture_sample (const imu_sensor_sample_t *  p_sample,
                         const terr_data_odometry_t * p_odometry)
{
    static uint32_t last_us;
    const uint32_t  period_us = 100000U;
    if (((!p_sample->b_is_valid) || (!p_sample->b_is_mag_valid)) ||
        ((p_sample->timestamp_us - last_us) < period_us))
    {
        return;
    }

    if (((NULL == p_odometry) || (!p_odometry->b_is_valid)) ||
        ((!p_odometry->b_is_stationary) || (3U != tm_usb_state())))
    {
        return;
    }

    last_us = p_sample->timestamp_us;
    /* Sensor counts are -2048..2047 and the smallest divisor is 980
     * counts/G. Converting to integer milligauss truncates less than 1 mG.
     * Even the longest line fits: ten timestamp digits and three signed
     * four-digit values. The 32-bit timestamp fits the target unsigned. */
    UB        line[48];
    const INT size = tm_sprintf(line, (const UB *)"[MAGCAL] %u %d %d %d\n",
                                (unsigned)p_sample->timestamp_us,
                                (int)(p_sample->mag_gauss[0] * 1000),
                                (int)(p_sample->mag_gauss[1] * 1000),
                                (int)(p_sample->mag_gauss[2] * 1000));
    if (size > 0)
    {
        tm_usb_console_put(line, size);
    }
}
#endif

/* End of file. */
