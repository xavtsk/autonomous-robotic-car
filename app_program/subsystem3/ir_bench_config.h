/* Development references only. Final height/material/lighting need calibration. */
#ifndef BUDDY3_IR_BENCH_CONFIG_H
#define BUDDY3_IR_BENCH_CONFIG_H
/* Explicitly change 1 to 7 for all three connected sensors. Check the GP28
 * battery-sensing jumper before wiring GP28. Disabled inputs are not read. */
#define IR_BENCH_ENABLED_MASK (1U)
#define IR_BENCH_BARCODE_CHANNEL (0U) /* GP26: Grove 6 WHITE wire -> AO */
#define IR_BENCH_LEFT_CHANNEL    (1U) /* GP27 */
#define IR_BENCH_RIGHT_CHANNEL   (2U) /* GP28 */
#define IR_BENCH_WHITE_RAW    (150U)
#define IR_BENCH_BLACK_RAW    (280U)
#define IR_BENCH_LEFT_WHITE_RAW  (150U)
#define IR_BENCH_LEFT_BLACK_RAW  (280U)
#define IR_BENCH_RIGHT_WHITE_RAW (150U)
#define IR_BENCH_RIGHT_BLACK_RAW (280U)
#define IR_BENCH_WHITE_LIMIT  (400U)
#define IR_BENCH_BLACK_LIMIT  (600U)
#define IR_BENCH_CALIBRATION_LABEL "PROVISIONAL"
#define IR_BENCH_SAMPLE_MS (10U)
#define IR_BENCH_REPORT_MS (200U)
#endif
