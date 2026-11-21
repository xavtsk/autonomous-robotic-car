/** @file builtin_calibration.h
 * @brief Supply the integrated motion profile's initial IR calibration.
 *
 * These references cover the mounted sensor geometry, wiring and surfaces
 * recorded below. Holding GP20 for two seconds at READY permits manual
 * recalibration after a change in lighting, material or mounting height.
 *
 * On 27 October 2026, the button collector recorded 101 samples per
 * surface per round. Its 12-bit ADC readings were as follows.
 *   front-left  (Grove 6 GP27): white 115..174, black path 1242..1761, 3 rounds
 *   front-right (Grove 5 GP26): white 146..218, black path 2960..3176, 4 rounds
 *   barcode     (Grove 7 GP28): white paper 134..179, printed black 275..335,
 *                              3 valid rounds
 * Barcode round 2 used black path material and is excluded. See
 * testing/subsystem3/IR_CALIBRATION_2026-10-27.md and its raw range logs.
 * The barcode sensor moved and switched to its AO output. The 21:48
 * course run on 27 October measured 170..260 on white and 3250..3850
 * on black bars. Those values superseded the earlier barcode ranges.
 * Replay of that run decoded at raw cut-offs from 1000 to 3000.
 * Combined-range midpoint references are LEFT 144/1501, RIGHT 182/3068,
 * BARCODE 215/3550. Keep the bench reference macros in sync with this table.
 * The boot path accepts the values through the regular calibration checks.
 * Each surface needs at least ten samples, and the white and black ranges
 * must be disjoint. Recalibrate and update this table if a sensor moves.
 *
 * Set S3_BUILTIN_CALIBRATION to 0 to restore the original rule that every
 * power-up requires a fresh button calibration.
 */
#ifndef S3_BUILTIN_CALIBRATION_H
#define S3_BUILTIN_CALIBRATION_H

#ifndef S3_BUILTIN_CALIBRATION
#define S3_BUILTIN_CALIBRATION 1
#endif

/* {white_min, white_max, black_min, black_max}, indexed by
 * ir_acquisition_sensor_id_t. */
#define S3_BUILTIN_CAL_BARCODE                                                 \
    {                                                                          \
        170U, 260U, 3250U, 3850U                                               \
    }
#define S3_BUILTIN_CAL_LEFT                                                    \
    {                                                                          \
        115U, 174U, 1242U, 1761U                                               \
    }
#define S3_BUILTIN_CAL_RIGHT                                                   \
    {                                                                          \
        146U, 218U, 2960U, 3176U                                               \
    }

#endif

/* End of file. */

