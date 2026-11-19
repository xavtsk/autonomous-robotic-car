/** @file ir_calibration.c
 * @brief Collect labelled reflectance samples and derive validated sensor
 * calibration.
 */
#include "ir_calibration.h"
#include "../app_types.h"

void s3_calibration_begin (s3_calibration_collector_t * p_collector)
{
    if (p_collector != 0)
    {
        *p_collector = (s3_calibration_collector_t){0};
        p_collector->white.minimum = 4095U;
        p_collector->black.minimum = 4095U;
    }
}

bool s3_calibration_add (s3_calibration_collector_t * p_collector,
                         ir_surface_t labelled_surface, uint16_t raw)
{
    s3_calibration_range_t * p_range;
    if ((p_collector == 0) || (raw > 4095U) ||
        ((labelled_surface != IR_SURFACE_WHITE) &&
         (labelled_surface != IR_SURFACE_BLACK)))
    {
        return false;
    }
    p_range = (labelled_surface == IR_SURFACE_WHITE) ? &p_collector->white
                                                     : &p_collector->black;
    if (p_range->count >= 65535U)
    {
        return false;
    }
    p_range->count++;
    p_range->sum += raw;
    if (raw < p_range->minimum)
    {
        p_range->minimum = raw;
    }
    if (raw > p_range->maximum)
    {
        p_range->maximum = raw;
    }
    return true;
}

bool s3_calibration_finish (const s3_calibration_collector_t * p_collector,
                            ir_calibration_t * p_active)
{
    ir_calibration_t candidate;
    ir_reflectance_t check;
    if ((p_collector == 0) || (p_active == 0) ||
        (p_collector->white.count < 10U) || (p_collector->black.count < 10U))
    {
        return false;
    }
    if (!((p_collector->white.maximum < p_collector->black.minimum) ||
          (p_collector->black.maximum < p_collector->white.minimum)))
    {
        return false;
    }
    candidate = *p_active;
    candidate.white_raw =
        (uint16_t)(p_collector->white.sum / p_collector->white.count);
    candidate.black_raw =
        (uint16_t)(p_collector->black.sum / p_collector->black.count);
    if (!ir_reflectance_init(&check, &candidate))
    {
        return false;
    }
    *p_active = candidate;
    return true;
}

s3_calibration_summary_t
s3_calibration_summary (const s3_calibration_range_t * p_range)
{
    s3_calibration_summary_t summary = {0};
    if ((p_range != 0) && (p_range->count > 0U))
    {
        summary.count = p_range->count;
        summary.minimum = p_range->minimum;
        summary.maximum = p_range->maximum;
        summary.average = (float32_t)p_range->sum / (float32_t)p_range->count;
        summary.available = true;
    }
    return summary;
}

/* End of file. */

