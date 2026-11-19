/** @file ir_calibration.c
 * @brief Collect labelled reflectance samples and derive validated sensor
 * calibration.
 */
#include "ir_calibration.h"
#include "../app_types.h"

/** @brief Clear both labelled ranges for a new calibration collection.
 * @param p_collector Writable caller-owned collector, or null for a no-op.
 * @post Counts, sums and maxima are zero; both minimum sentinels are 4095.
 * @note No active sensor calibration changes. The sampling owner serializes
 * all access to the collector; this function acquires no lock.
 */
void s3_calibration_begin (s3_calibration_collector_t * p_collector)
{
    if (p_collector != 0)
    {
        *p_collector = (s3_calibration_collector_t){0};
        p_collector->white.minimum = 4095U;
        p_collector->black.minimum = 4095U;
    }
}

/** @brief Accumulate one labelled 12-bit ADC sample without saturation.
 * @param p_collector Collector initialized by begin and owned by the caller.
 * @param labelled_surface Exactly IR_SURFACE_WHITE or IR_SURFACE_BLACK.
 * @param raw ADC count in 0..4095; wider uint16 inputs are rejected unchanged.
 * @return True after updating count, sum and bounds of the selected range.
 * False for null, invalid label/raw or count 65535 leaves ranges unchanged.
 * @note The caller serializes access. The other labelled range is untouched;
 * counters stop accepting samples rather than wrapping or replacing history.
 */
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

/** @brief Commit integer-mean raw endpoints after collection validation.
 * @param p_collector Readable begin/add collector, stable and caller-owned.
 * @param p_active Writable calibration with existing normalized limits.
 * @return True after replacing raw endpoints with integer-truncated means.
 * Nulls, fewer than ten samples per label, overlapping/touching ranges, or
 * rejected candidate calibration return false and leave p_active unchanged.
 * @note Either ADC polarity is accepted. Existing white/black limits are kept.
 * No sensor state is installed here; the owner applies success through
 * ir_reflectance_init and serializes collector/calibration access.
 */
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

/** @brief Copy a descriptive range summary without requiring ten samples.
 * @param p_range Readable labelled range, or null for an unavailable result.
 * @return Zero/unavailable for null or zero count; otherwise copies count and
 * raw ADC extrema and computes sum/count using the existing float32 precision.
 * @note No range validation or mutation. Caller ensures a stable snapshot;
 * this function neither locks nor commits an active calibration.
 */
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

