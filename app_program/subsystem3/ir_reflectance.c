/** @file ir_reflectance.c
 * @brief Buddy 3: hardware-independent, per-sensor reflectance classification.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 */
#include "ir_reflectance.h"

#define IR_ADC_MAX_COUNT (4095U)
#define IR_DARKNESS_MAX (1000)

/** @brief Forget only the previously classified surface.
 * @param p_sensor Writable caller-owned sensor state, or null for a no-op.
 * @post A nonnull state's surface is UNKNOWN; configuration and endpoints stay.
 * @note Sampling-owner operation with no synchronization or peripheral access.
 */
void ir_reflectance_reset (ir_reflectance_t * p_sensor)
{
    if (p_sensor != 0)
    {
        p_sensor->surface = IR_SURFACE_UNKNOWN;
    }
}

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
                          const ir_reflectance_calibration_t * p_calibration)
{
    if (0 == p_sensor)
    {
        return false;
    }

    p_sensor->b_is_configured = false;
    ir_reflectance_reset(p_sensor);

    if ((((((0 == p_calibration) ||
            (p_calibration->white_raw > IR_ADC_MAX_COUNT)) ||
           (p_calibration->black_raw > IR_ADC_MAX_COUNT)) ||
          (p_calibration->white_raw == p_calibration->black_raw)) ||
         (p_calibration->white_limit >= p_calibration->black_limit)) ||
        ((int)p_calibration->black_limit > IR_DARKNESS_MAX))
    {
        return false;
    }

    /* clang-format off */
    p_sensor->calibration     = (ir_reflectance_stored_cal_t){
        p_calibration->white_raw, p_calibration->black_raw,
        p_calibration->white_limit, p_calibration->black_limit};
    p_sensor->b_is_configured = true;
    /* clang-format on */
    return true;
}

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
                            uint16_t * p_darkness)
{

    if ((0 == p_sensor) || (0 == p_darkness))
    {
        return false;
    }

    if ((!p_sensor->b_is_configured) || (raw > IR_ADC_MAX_COUNT))
    {
        ir_reflectance_reset(p_sensor);
        return false;
    }

    /* A validated 12-bit endpoint fits in int32_t. Signed subtraction
     * preserves either sensor polarity and the scaled delta fits in int32_t.
     */
    const int32_t span = (int32_t)p_sensor->calibration.black_raw -
                         (int32_t)p_sensor->calibration.white_raw;
    int32_t darkness =
        (((int32_t)raw - (int32_t)p_sensor->calibration.white_raw) *
         IR_DARKNESS_MAX) /
        span;

    if (darkness < 0)
    {
        darkness = 0;
    }
    else if (darkness > IR_DARKNESS_MAX)
    {
        darkness = IR_DARKNESS_MAX;
    }
    else
    {
        /* The normalized result is already in range. */
    }

    /* The clamp bounds the value to 0..1000 before unsigned conversion. */
    *p_darkness = (uint16_t)darkness;

    if (*p_darkness <= p_sensor->calibration.white_limit)
    {
        p_sensor->surface = IR_SURFACE_WHITE;
    }
    else if (*p_darkness >= p_sensor->calibration.black_limit)
    {
        p_sensor->surface = IR_SURFACE_BLACK;
    }
    else
    {
        /* Between the limits, retain the previous surface classification. */
    }

    return true;
}

/* End of file. */

