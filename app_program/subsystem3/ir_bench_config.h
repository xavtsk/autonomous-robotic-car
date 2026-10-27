/* Measured 2026-10-27; remeasure after height/material/lighting changes. */
#ifndef BUDDY3_IR_BENCH_CONFIG_H
#define BUDDY3_IR_BENCH_CONFIG_H
#include "ir_acquisition.h"
/* Build with IR_CHANNELS=3 for all three connected sensors. Check the GP28
 * battery-sensing jumper before wiring GP28. Disabled inputs are not read. */
#ifndef APP_IR_CHANNELS
#define APP_IR_CHANNELS 1
#endif
#if APP_IR_CHANNELS == 3
#define IR_BENCH_ENABLED_MASK (7U)
#else
#define IR_BENCH_ENABLED_MASK (1U)
#endif
/* Same physical mapping as the integrated controller (ir_acquisition.h). */
#define IR_BENCH_BARCODE_CHANNEL S3_ADC_CHANNEL_BARCODE /* GP28 */
#define IR_BENCH_LEFT_CHANNEL    S3_ADC_CHANNEL_LEFT    /* GP27 */
#define IR_BENCH_RIGHT_CHANNEL   S3_ADC_CHANNEL_RIGHT   /* GP26 */
/* Midpoints of controller/builtin_calibration.h measured ranges.
 * builtin_test.c checks that boot and bench references remain identical. */
#define IR_BENCH_WHITE_RAW    (156U)
#define IR_BENCH_BLACK_RAW    (305U)
#define IR_BENCH_LEFT_WHITE_RAW  (144U)
#define IR_BENCH_LEFT_BLACK_RAW  (1501U)
#define IR_BENCH_RIGHT_WHITE_RAW (182U)
#define IR_BENCH_RIGHT_BLACK_RAW (3068U)
#define IR_BENCH_WHITE_LIMIT  (400U)
#define IR_BENCH_BLACK_LIMIT  (600U)
#define IR_BENCH_CALIBRATION_LABEL "MEASURED_2026-10-27"
#define IR_BENCH_SAMPLE_MS (10U)
#define IR_BENCH_REPORT_MS (200U)
#endif
