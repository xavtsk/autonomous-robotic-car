#include "ir_calibration.h"
void s3_calibration_begin(s3_calibration_collector_t *collector)
{
    if (collector != 0) {
        *collector = (s3_calibration_collector_t){0};
        collector->white.minimum = 4095U;
        collector->black.minimum = 4095U;
    }
}
bool s3_calibration_add(s3_calibration_collector_t *collector,
                        ir_surface_t labelled_surface, uint16_t raw)
{
    s3_calibration_range_t *range;
    if ((collector == 0) || (raw > 4095U) ||
        ((labelled_surface != IR_SURFACE_WHITE) && (labelled_surface != IR_SURFACE_BLACK))) {
        return false;
    }
    range = (labelled_surface == IR_SURFACE_WHITE) ? &collector->white : &collector->black;
    if (range->count >= 65535U) { return false; }
    range->count++;
    range->sum += raw;
    if (raw < range->minimum) { range->minimum = raw; }
    if (raw > range->maximum) { range->maximum = raw; }
    return true;
}
bool s3_calibration_finish(const s3_calibration_collector_t *collector,
                           ir_calibration_t *active)
{
    ir_calibration_t candidate;
    ir_reflectance_t check;
    if ((collector == 0) || (active == 0) ||
        (collector->white.count < 10U) || (collector->black.count < 10U)) { return false; }
    if (!((collector->white.maximum < collector->black.minimum) ||
          (collector->black.maximum < collector->white.minimum))) { return false; }
    candidate = *active;
    candidate.white_raw = (uint16_t)(collector->white.sum / collector->white.count);
    candidate.black_raw = (uint16_t)(collector->black.sum / collector->black.count);
    if (!ir_reflectance_init(&check, &candidate)) { return false; }
    *active = candidate;
    return true;
}

s3_calibration_summary_t s3_calibration_summary(const s3_calibration_range_t *range)
{
    s3_calibration_summary_t summary = {0};
    if ((range != 0) && (range->count > 0U)) {
        summary.count = range->count;
        summary.minimum = range->minimum;
        summary.maximum = range->maximum;
        summary.average = (float)range->sum / (float)range->count;
        summary.available = true;
    }
    return summary;
}
