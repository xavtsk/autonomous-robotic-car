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
} s3_calibration_range_t;

typedef struct
{
    s3_calibration_range_t white;
    s3_calibration_range_t black;
} s3_calibration_collector_t;

typedef struct
{
    uint32_t count;
    uint16_t minimum, maximum;
    float32_t average;
    bool available;
} s3_calibration_summary_t;

/* Descriptive results even before enough samples exist to commit calibration.
 */
s3_calibration_summary_t
s3_calibration_summary (const s3_calibration_range_t * p_range);
void s3_calibration_begin (s3_calibration_collector_t * p_collector);
bool s3_calibration_add (s3_calibration_collector_t * p_collector,
                         ir_surface_t labelled_surface, uint16_t raw);
/* >=10 samples per surface, strictly non-overlapping ranges. Leaves active
 * unchanged on rejection. Apply success using ir_reflectance_init. */
bool s3_calibration_finish (const s3_calibration_collector_t * p_collector,
                            ir_calibration_t * p_active);
#endif

/* End of file. */

