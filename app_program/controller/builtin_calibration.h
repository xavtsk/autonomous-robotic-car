/*
 * Built-in IR calibration for the S3_MOTION profile, so the car is ready to
 * START right after power-up. Holding GP20 for 2 s at READY still recalibrates
 * by hand (e.g. different lighting, mat or sensor height on demo day).
 *
 * Measured 2026-10-27 at the mounted height, using raw button-collection
 * summaries (101 samples per surface per round; ADC raw, 0..4095):
 *   front-left  (Grove 6 GP27): white 115..174, black path 1242..1761, 3 rounds
 *   front-right (Grove 5 GP26): white 146..218, black path 2960..3176, 4 rounds
 *   barcode     (Grove 7 GP28): white paper 134..179, printed black 275..335,
 *                              3 valid rounds
 * Barcode round 2 used black path material and is excluded. See
 * testing/subsystem3/IR_CALIBRATION_2026-10-27.md and its raw range logs.
 * Combined-range midpoint references are LEFT 144/1501, RIGHT 182/3068,
 * BARCODE 156/305. Keep the bench reference macros in sync with this table.
 * These values are ONLY valid for that mounting height and surface. If a
 * sensor is moved, recalibrate with the buttons and update this table.
 * They still pass the normal calibration checks (>=10 samples per surface,
 * non-overlapping white/black ranges) before being accepted at boot.
 *
 * Set S3_BUILTIN_CALIBRATION to 0 to restore the original rule that every
 * power-up requires a fresh button calibration.
 */
#ifndef S3_BUILTIN_CALIBRATION_H
#define S3_BUILTIN_CALIBRATION_H

#ifndef S3_BUILTIN_CALIBRATION
#define S3_BUILTIN_CALIBRATION 1
#endif

/* {white_min, white_max, black_min, black_max}, indexed by s3_sensor_id_t. */
#define S3_BUILTIN_CAL_BARCODE {134U, 179U, 275U, 335U}
#define S3_BUILTIN_CAL_LEFT    {115U, 174U, 1242U, 1761U}
#define S3_BUILTIN_CAL_RIGHT   {146U, 218U, 2960U, 3176U}

#endif
