/*
 * Built-in IR calibration for the S3_MOTION profile, so the car is ready to
 * START right after power-up. Holding GP20 for 2 s at READY still recalibrates
 * by hand (e.g. different lighting, mat or sensor height on demo day).
 *
 * Measured 2026-10-26 on the team course mat with the current sensor mounting
 * (ADC raw, 0..4095; head fully on white mat / fully on black tape or bar):
 *   front-left  (GP26): white ~130-180, black ~1600 (bench, fully on tape)
 *   front-right (GP28): white ~155-228, black ~3130-3250
 *   barcode     (GP27): white ~130-186, black ~1890-1930 (fully on a bar)
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
#define S3_BUILTIN_CAL_BARCODE {130U, 186U, 1880U, 1930U}
#define S3_BUILTIN_CAL_LEFT    {130U, 180U, 1560U, 1640U}
#define S3_BUILTIN_CAL_RIGHT   {155U, 228U, 3130U, 3250U}

#endif
