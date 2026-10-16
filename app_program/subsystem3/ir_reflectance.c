#include "ir_reflectance.h"

#define IR_ADC_MAX_COUNT (4095U)
#define IR_DARKNESS_MAX  (1000)

void ir_reflectance_reset(ir_reflectance_t *p_sensor)
{
    if (p_sensor != 0)
    {
        p_sensor->surface = IR_SURFACE_UNKNOWN;
    }
}

bool ir_reflectance_init(ir_reflectance_t *p_sensor,
                        const ir_calibration_t *p_calibration)
{
    if (p_sensor == 0)
    {
        return false;
    }
    p_sensor->configured = false;
    ir_reflectance_reset(p_sensor);
    if ((p_calibration == 0) ||
        (p_calibration->white_raw > IR_ADC_MAX_COUNT) ||
        (p_calibration->black_raw > IR_ADC_MAX_COUNT) ||
        (p_calibration->white_raw == p_calibration->black_raw) ||
        (p_calibration->white_limit >= p_calibration->black_limit) ||
        (p_calibration->black_limit > IR_DARKNESS_MAX))
    {
        return false;
    }
    p_sensor->calibration = *p_calibration;
    p_sensor->configured = true;
    return true;
}

bool ir_reflectance_update(ir_reflectance_t *p_sensor, uint16_t raw,
                          uint16_t *p_darkness)
{
    int32_t span;
    int32_t darkness;

    if ((p_sensor == 0) || (p_darkness == 0))
    {
        return false;
    }
    if ((!p_sensor->configured) || (raw > IR_ADC_MAX_COUNT))
    {
        ir_reflectance_reset(p_sensor);
        return false;
    }

    span = (int32_t)p_sensor->calibration.black_raw -
           (int32_t)p_sensor->calibration.white_raw;
    darkness = (((int32_t)raw - (int32_t)p_sensor->calibration.white_raw) *
                IR_DARKNESS_MAX) / span;
    if (darkness < 0)
    {
        darkness = 0;
    }
    else if (darkness > IR_DARKNESS_MAX)
    {
        darkness = IR_DARKNESS_MAX;
    }

    *p_darkness = (uint16_t)darkness;
    if (*p_darkness <= p_sensor->calibration.white_limit)
    {
        p_sensor->surface = IR_SURFACE_WHITE;
    }
    else if (*p_darkness >= p_sensor->calibration.black_limit)
    {
        p_sensor->surface = IR_SURFACE_BLACK;
    }
    return true;
}
