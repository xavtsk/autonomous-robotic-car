/** @file ir_reflectance.h
 * @brief Buddy 3: hardware-independent, per-sensor reflectance classification.
 */
#ifndef BUDDY3_IR_REFLECTANCE_H
#define BUDDY3_IR_REFLECTANCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IR_SURFACE_UNKNOWN = 0,
    IR_SURFACE_WHITE,
    IR_SURFACE_BLACK
} ir_reflectance_surface_t;

typedef struct
{
    uint16_t white_raw;
    uint16_t black_raw;
    uint16_t white_limit; /* Normalised 0..1000: enter WHITE at/below this. */
    uint16_t
        black_limit; /* Enter BLACK at/above this; must exceed white_limit. */
} ir_reflectance_calibration_t;

/* Validated retained values only. The public input above stays wide so an
 * invalid 4096 cannot truncate into an apparently valid ADC zero. */
typedef struct __attribute__((packed))
{
    uint32_t white_raw : 12;
    uint32_t black_raw : 12;
    uint32_t white_limit : 10;
    uint32_t black_limit : 10;
} ir_reflectance_stored_cal_t;

typedef struct
{
    ir_reflectance_stored_cal_t calibration;
    uint8_t                     surface : 2;
    uint8_t                     b_is_configured : 1;
} ir_reflectance_t;

/* One state object per sensor, owned by the sampling task. Either ADC polarity
 * is supported. Invalid calibration leaves the object unconfigured/UNKNOWN. */
/** @brief Validate wide calibration values before retaining compact fields.
 * @param p_sensor Writable sensor state, or null for rejection.
 * @param p_calibration Readable calibration: raw endpoints 0..4095, unequal;
 * normalized limits satisfy 0 <= white_limit < black_limit <= 1000.
 * @return True after copying accepted endpoints/limits and setting configured.
 * False on invalid calibration leaves nonnull state unconfigured and UNKNOWN,
 * but preserves its old stored calibration. Null p_sensor changes nothing.
 * @note Either raw polarity is supported. The sampling owner serializes access;
 * no physical calibration measurement or peripheral operation occurs here.
 */
bool ir_reflectance_init (ir_reflectance_t *                   p_sensor,
                          const ir_reflectance_calibration_t * p_calibration);
/** @brief Forget only the previously classified surface.
 * @param p_sensor Writable caller-owned sensor state, or null for a no-op.
 * @post A nonnull state's surface is UNKNOWN; configuration and endpoints stay.
 * @note Sampling-owner operation with no synchronization or peripheral access.
 */
void ir_reflectance_reset (ir_reflectance_t * p_sensor);

/* Returns false for invalid arguments/raw counts or unconfigured state.
 * Valid readings are clamped to 0 (white)..1000 (black). Inside the hysteresis
 * band, retain the previous surface; initially this is UNKNOWN. On invalid
 * raw input, reset the surface to UNKNOWN and leave p_darkness unchanged.
 * This cannot detect a disconnected wire or validate real track calibration. */
/** @brief Normalize one ADC reading and update the retained hysteresis state.
 * @param p_sensor State successfully initialized by ir_reflectance_init.
 * @param raw Wide ADC input; only 0..4095 is accepted.
 * @param p_darkness Writable normalized output in 0..1000 on success.
 * @return True after the original integer division and clamping. Null arguments
 * return false without mutation. Unconfigured state or invalid raw resets only
 * the surface to UNKNOWN and returns false without writing p_darkness.
 * @note Values at/below white_limit enter WHITE; at/above black_limit enter
 * BLACK. Between the limits the previous surface, including UNKNOWN, persists.
 * Caller owns synchronization. This does not detect a disconnected sensor.
 */
bool ir_reflectance_update (ir_reflectance_t * p_sensor, uint16_t raw,
                            uint16_t * p_darkness);

#endif /* BUDDY3_IR_REFLECTANCE_H */

/* End of file. */

