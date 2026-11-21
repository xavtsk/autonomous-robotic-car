/** @file ir_calibration.h
 * @brief Collect labelled reflectance samples and derive validated sensor
 * calibration.
 */
#ifndef BUDDY3_IR_CALIBRATION_H
#define BUDDY3_IR_CALIBRATION_H
#include "../app_types.h"
#include "ir_reflectance.h"

typedef struct __attribute__((packed))
{
    uint32_t count : 16;
    uint32_t sum : 28; /* max 65535 * 4095 = 268365825 */
    uint32_t minimum : 12;
    uint32_t maximum : 12;
} ir_calibration_range_t;

typedef struct
{
    ir_calibration_range_t white;
    ir_calibration_range_t black;
} ir_calibration_collector_t;

typedef struct
{
    uint32_t  count;
    uint16_t  minimum;
    uint16_t  maximum;
    float32_t average;
    bool      b_is_available;
} ir_calibration_summary_t;

/* Descriptive results even before enough samples exist to commit calibration.
 */
/** @brief Copy a descriptive range summary without requiring ten samples.
 * @param p_range Readable labelled range, or null for an unavailable result.
 * @return Zero/unavailable for null or zero count; otherwise copies count and
 * raw ADC extrema and computes sum/count using the existing float32 precision.
 * @note No range validation or mutation. Caller ensures a stable snapshot;
 * this function neither locks nor commits an active calibration.
 */
ir_calibration_summary_t
ir_calibration_summary (const ir_calibration_range_t * p_range);
/** @brief Clear both labelled ranges for a new calibration collection.
 * @param p_collector Writable caller-owned collector, or null for a no-op.
 * @post Counts, sums and maxima are zero; both minimum sentinels are 4095.
 * @note No active sensor calibration changes. The sampling owner serializes
 * all access to the collector; this function acquires no lock.
 */
void ir_calibration_begin (ir_calibration_collector_t * p_collector);
/** @brief Accumulate one labelled 12-bit ADC sample without saturation.
 * @param p_collector Collector initialized by begin and owned by the caller.
 * @param labelled_surface Exactly IR_SURFACE_WHITE or IR_SURFACE_BLACK.
 * @param raw ADC count in 0..4095; wider uint16 inputs are rejected unchanged.
 * @return True after updating count, sum and bounds of the selected range.
 * False for null, invalid label/raw or count 65535 leaves ranges unchanged.
 * @note The caller serializes access. The other labelled range is untouched;
 * counters stop accepting samples rather than wrapping or replacing history.
 */
bool ir_calibration_add (ir_calibration_collector_t * p_collector,
                         ir_reflectance_surface_t     labelled_surface,
                         uint16_t                     raw);
/* >=10 samples per surface, strictly non-overlapping ranges. Leaves active
 * unchanged on rejection. Apply success using ir_reflectance_init. */
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
bool ir_calibration_finish (const ir_calibration_collector_t * p_collector,
                            ir_reflectance_calibration_t *     p_active);
#endif

/* End of file. */

