/** @file codec_motion.c
 * @brief Pack and unpack the shared motion snapshot.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 */
#include "codec_motion.h"
#include "../build_assert.h"

/* Compile-time guarantee: motion snapshot layout. */
BUILD_ASSERT(sizeof(codec_motion_snapshot_t) == 4U, codec_motion_c_check_a_t);
/* Each state and fault must fit its three-bit wire field. */
BUILD_ASSERT(MOTION_FAULT < 8 && MOTION_FAULT_TIMEOUT < 8,
             codec_motion_c_check_b_t);
/* Every manoeuvre code must fit its two-bit field. */
BUILD_ASSERT(MANOEUVRE_U_TURN < 4, codec_motion_c_check_c_t);

static uint32_t encode_snapshot_word (const codec_motion_snapshot_t * p_value);
static uint32_t speed_code (int32_t mms);
static int16_t  speed_value (uint32_t code);

/** @brief Validate state and fault and, when valid, both wheel speeds. Encode
 * four little-endian bytes. Failure leaves the caller's output
 * untouched and absent speeds encode as zero.
 */
bool codec_motion_pack (codec_motion_snapshot_t * p_out, motion_state_t state,
                        motion_fault_t fault, bool b_is_busy,
                        bool b_is_speed_valid, int32_t left_mms,
                        int32_t right_mms)
{

    /* Valid enum codes are nonnegative. A negative representation maps
     * above the valid range in the unsigned check. */
    if (((0 == p_out) || ((unsigned)state > (unsigned int)MOTION_FAULT)) ||
        ((unsigned)fault > (unsigned int)MOTION_FAULT_TIMEOUT))
    {
        return false;
    }

    if (b_is_speed_valid && ((((left_mms < -MOTION_REPORT_MAX_MMS) ||
                               (left_mms > MOTION_REPORT_MAX_MMS)) ||
                              (right_mms < -MOTION_REPORT_MAX_MMS)) ||
                             (right_mms > MOTION_REPORT_MAX_MMS)))
    {
        return false;
    }

    uint32_t packed = b_is_speed_valid
                          ? speed_code(left_mms) | (speed_code(right_mms) << 9)
                          : 0U;
    /* Validated state and fault fit three bits. Each flag is zero or
     * one. Widen to uint32_t before placing the fields. */
    packed |= ((uint32_t)state << 18) | ((uint32_t)fault << 21) |
              ((uint32_t)b_is_busy << 24) | ((uint32_t)b_is_speed_valid << 25);

    unsigned byte_index;
    for (byte_index = 0; byte_index < MOTION_SNAPSHOT_BYTES; ++byte_index)
    {
        /* Keep only the selected eight low bits after the right shift. */
        p_out->bytes[byte_index] = (uint8_t)(packed >> (byte_index * 8U));
    }

    return true;
}

/** @brief Check reserved bits, enum ranges and speed encodings. Reject missing
 * input, noncanonical negative zero and nonzero speeds when the valid
 * flag is clear.
 */
bool codec_motion_valid (const codec_motion_snapshot_t * p_value)
{

    if (0 == p_value)
    {
        return false;
    }

    const uint32_t packed = encode_snapshot_word(p_value);
    const uint32_t left   = packed & 511U;
    const uint32_t right  = (packed >> 9) & 511U;

    if ((((packed >> 26) != 0U) ||
         (((packed >> 18) & 7U) > (unsigned int)MOTION_FAULT)) ||
        (((packed >> 21) & 7U) > (unsigned int)MOTION_FAULT_TIMEOUT))
    {
        return false;
    }

    if ((packed & (1U << 25)) == 0U)
    {
        return ((0U == left) && (0U == right));
    }

    return (left != 256U) && (right != 256U); /* Reject negative zero. */
}

/** @brief Extract the three-bit motion state from a non-null snapshot. Callers
 * validating external bytes first use codec_motion_valid.
 */
motion_state_t codec_motion_state (const codec_motion_snapshot_t * p_value)
{
    /* Preserve the raw code 0..7. Callers check enum validity separately. */
    return (motion_state_t)((encode_snapshot_word(p_value) >> 18) & 7U);
}

/** @brief Extract the three-bit fault field from the supplied non-null
 * snapshot.
 */
motion_fault_t codec_motion_fault (const codec_motion_snapshot_t * p_value)
{
    /* Preserve the raw code 0..7. Callers check enum validity separately. */
    return (motion_fault_t)((encode_snapshot_word(p_value) >> 21) & 7U);
}

/** @brief Read the busy flag from a non-null motion snapshot.
 */
bool codec_motion_busy (const codec_motion_snapshot_t * p_value)
{
    return (encode_snapshot_word(p_value) & (1U << 24)) != 0U;
}

/** @brief Read the wheel-speed validity flag from a non-null motion snapshot.
 */
bool codec_motion_speed_valid (const codec_motion_snapshot_t * p_value)
{
    return (encode_snapshot_word(p_value) & (1U << 25)) != 0U;
}

/** @brief Decode the left wheel field as signed millimetres per second. The
 * caller checks snapshot and speed validity when required.
 */
int16_t codec_motion_left_mms (const codec_motion_snapshot_t * p_value)
{
    return speed_value(encode_snapshot_word(p_value) & 511U);
}

/** @brief Decode the right wheel field as signed millimetres per second. The
 * caller checks snapshot and speed validity when required.
 */
int16_t codec_motion_right_mms (const codec_motion_snapshot_t * p_value)
{
    return speed_value((encode_snapshot_word(p_value) >> 9) & 511U);
}

/** @brief Reconstruct the 32-bit motion snapshot from four little-endian
 * bytes. The caller supplies a non-null snapshot.
 */
static uint32_t encode_snapshot_word (const codec_motion_snapshot_t * p_value)
{
    /* Convert each byte from 0..255 before a shift of up to 24 places. */
    return (uint32_t)p_value->bytes[0] | ((uint32_t)p_value->bytes[1] << 8) |
           ((uint32_t)p_value->bytes[2] << 16) |
           ((uint32_t)p_value->bytes[3] << 24);
}

/** @brief Encode a previously range-checked signed speed in millimetres per
 * second into sign and magnitude. Quantize to two millimetres per
 * second with ties away from zero.
 */
static uint32_t speed_code (int32_t mms)
{
    /* The validated speed is within +/-510 mm/s. Negation is defined,
     * and its nonnegative magnitude fits in uint32_t. */
    uint32_t mag = (uint32_t)((mms < 0) ? -mms : mms);
    mag = (mag + 1U) / 2U; /* Round to 2 mm/s with ties away from zero. */
    return mag | ((uint32_t)(mms < 0) << 8);
}

/** @brief Decode a nine-bit sign/magnitude field into signed millimetres per
 * second. The magnitude is at most 510 mm/s after scaling.
 */
static int16_t speed_value (uint32_t code)
{
    const int32_t mag = (int32_t)(code & 255U) * 2;
    return (int16_t)((code & 256U) ? -mag : mag);
}

/* End of file. */

