/** @file motion_pack.c
 * @brief Minimum-bit representation of the data Buddy 2 shares.
 *
 * @par Ownership
 * Buddy 2 - Motion Control
 *
 * Packed with shifts and masks (not C bit-fields), so the layout is the same
 * on every compiler and the laptop decoder can follow DATA_FORMAT.md exactly.
 */

#include <stddef.h>

#include "motion_pack.h"
#include "../app_types.h"

/* These widths define the packed fields. */
#define STATE_BITS (3U)
#define FAULT_BITS (3U) /* Encode the report fault code. */
#define FAULT_MASK_BITS (MOTION_FAULT_BITS_WIDTH) /* bits (status) */
#define ID_BITS (8U)
#define OUTCOME_BITS (2U)
#define TYPE_BITS (3U)
#define MANOEUVRE_BITS (2U)
#define MOVE_SPEED_BITS (MOTION_PACK_MOVE_SPEED_BITS)
#define SPEED_BITS (MOTION_PACK_SPEED_MAG_BITS + 1U)
#define DIST_BITS (MOTION_PACK_DIST_MAG_BITS + 1U)
#define DRIVE_BITS (MOTION_PACK_DRIVE_MAG_BITS + 1U)
#define ANGLE_BITS (MOTION_PACK_ANGLE_MAG_BITS + 1U)

/* The status uses all 64 bits. See DATA_FORMAT.md. */
#define ST_STATE_LSB (0U)
#define ST_FAULT_LSB (ST_STATE_LSB + STATE_BITS)     /*  3 */
#define ST_ID_LSB (ST_FAULT_LSB + FAULT_MASK_BITS)   /*  8 */
#define ST_SPEED_L_LSB (ST_ID_LSB + ID_BITS)         /* 16 */
#define ST_SPEED_R_LSB (ST_SPEED_L_LSB + SPEED_BITS) /* 26 */
#define ST_DIST_LSB (ST_SPEED_R_LSB + SPEED_BITS)    /* 36 */
#define ST_HEADING_LSB (ST_DIST_LSB + DIST_BITS)     /* 52 */
#define ST_STOPPED_BIT (ST_HEADING_LSB + MOTION_PACK_HEADING_BITS)
#define ST_BUSY_BIT (ST_STOPPED_BIT + 1U) /* 62 */
#define ST_VALID_BIT (ST_BUSY_BIT + 1U)   /* 63 */

/* The report uses a 16-bit word. */
#define RP_ID_LSB (0U)
#define RP_OUTCOME_LSB (RP_ID_LSB + ID_BITS)         /*  8 */
#define RP_FAULT_LSB (RP_OUTCOME_LSB + OUTCOME_BITS) /* 10 */
#define RP_LOST_BIT (RP_FAULT_LSB + FAULT_BITS)      /* 13 */

/* The command type uses bits 0 through 2. The largest payload is the
 * 14-bit drive field plus the 9-bit speed field. No command needs more than
 * 26 bits, so each positioned field can be converted to uint32_t safely. */
#define CMD_ARG_LSB (TYPE_BITS)

#define HEADING_TURN_DEG (360)
#define HEADING_LIMIT_DEG                                                      \
    (1000000.0f) /* Bound the heading before conversion. */

/** @brief Apply the existing sign comparison used for magnitude conversion.
 * @param operand Floating value, including signed zero or non-finite input.
 * @return Negated input when it compares below zero, otherwise the input.
 * @note Negative zero retains its sign; NaN is passed through. No state
 * changes.
 */
static float32_t abs_f (float32_t operand)
{
    return ((operand < 0.0f) ? -operand : operand);
}

/** @brief Create a low-bit mask without shifting by 32 or more.
 * @param bits Requested bit count; all uint32_t values are accepted.
 * @return Zero for zero bits, the low-bit mask for 1..31, or UINT32_MAX for
 * 32 or more. Used only as a value; no stored object is modified.
 */
static uint32_t mask_of (uint32_t bits)
{
    return ((bits >= 32U) ? 0xFFFFFFFFU : ((1UL << bits) - 1U));
}

/** @brief Extract a field from a packed 64-bit value.
 * @param word Source word.
 * @param lsb First bit position, below 64.
 * @param bits Width from 0 through 32; caller keeps lsb plus width at most 64.
 * @return The selected low-order field, with all other bits cleared.
 * @note Internal layout constants supply the shift bounds; no checks occur.
 */
static uint32_t field_get (uint64_t word, uint32_t lsb, uint32_t bits)
{
    /* Select at most 32 bits, then discard shifted bits above the field. */
    return ((uint32_t)(word >> lsb) & mask_of(bits));
}

/** @brief Position the masked portion of a field in a 64-bit value.
 * @param value Field value; bits above the declared width are discarded.
 * @param lsb Destination bit position, below 64.
 * @param bits Width from 0 through 32; caller keeps lsb plus width at most 64.
 * @return Shifted field suitable for OR into an initially cleared word.
 * @note Does not modify an existing word or validate layout constants.
 */
static uint64_t field_put (uint32_t value, uint32_t lsb, uint32_t bits)
{
    /* Widen the masked word before shifting through positions 0..63. */
    return ((uint64_t)(value & mask_of(bits)) << lsb);
}

/** @brief Test finiteness using the retained floating arithmetic checks.
 * @param operand Value to classify.
 * @return True for finite values, including both zeros; false for NaN/infinity
 * under the project's floating compiler semantics.
 * @note Uses self-comparison and subtraction; floating exception flags are not
 * managed here. This documents the existing classification, not a replacement.
 */
static bool is_finite (float32_t operand)
{
    return (operand == operand) &&
           ((operand - operand) == 0.0f); /* Reject NaN and infinity. */
}

/* Round the magnitude to whole steps and saturate it. NaN and negative
 * inputs return zero before any conversion to unsigned.
 */
/** @brief Round a magnitude into a saturated unsigned step count.
 * @param magnitude Magnitude in the same unit as step; non-finite is allowed.
 * @param step Positive finite quantization step from the module constants.
 * @param bits Magnitude width, 1..15 for the present callers.
 * @return Truncated magnitude/step plus 0.5, bounded to the field. NaN,
 * negative input and values whose rounded expression is below one give zero;
 * positive infinity or a value at/above the field maximum gives that maximum.
 * @note Bounds are tested before the unsigned conversion; no state changes.
 */
static uint32_t to_steps (float32_t magnitude, float32_t step, uint32_t bits)
{
    const uint32_t  max_steps = mask_of(bits);
    const float32_t steps     = (magnitude / step) + 0.5f;

    if (!(steps >= 1.0f))
    {
        return 0U;
    }

    /* The widest magnitude has 15 bits, so float32_t represents this
     * limit exactly. Saturation leaves only finite, positive steps in
     * uint32_t range for the conversion below. */
    if (steps >= (float32_t)max_steps)
    {
        return max_steps;
    }

    return (uint32_t)steps;
}

/* Store a zero magnitude without a negative sign. */
/** @brief Encode a direction bit and a rounded magnitude.
 * @param value Signed quantity; finite and non-finite inputs are handled.
 * @param step Positive finite step in the quantity's unit.
 * @param mag_bits Magnitude width, 1..15 for the present callers.
 * @return Saturated magnitude with direction at bit mag_bits. Direction is
 * set only for a negative value whose rounded magnitude is nonzero.
 * @note NaN encodes zero; infinities saturate. Both input zeros encode zero.
 */
static uint32_t sm_pack (float32_t value, float32_t step, uint32_t mag_bits)
{
    const uint32_t mag = to_steps(abs_f(value), step, mag_bits);
    const uint32_t dir = ((value < 0.0f) && (mag != 0U)) ? 1U : 0U;

    return (dir << mag_bits) | mag;
}

/** @brief Decode a direction bit and magnitude into a floating value.
 * @param field Raw sign-magnitude bits; higher bits are ignored.
 * @param step Positive finite step in the decoded quantity's unit.
 * @param mag_bits Magnitude width, 1..15 for the present callers.
 * @return Low magnitude times step, negated when the direction bit is set.
 * @note A raw direction bit with zero magnitude produces negative zero;
 * this decoder does not require the packer's canonical zero representation.
 */
static float32_t sm_unpack (uint32_t field, float32_t step, uint32_t mag_bits)
{
    /* A masked magnitude of at most 32767 converts exactly to float32_t. */
    const float32_t value = (float32_t)(field & mask_of(mag_bits)) * step;

    return (((field >> mag_bits) & 1U) != 0U) ? -value : value;
}

/** @brief Round and wrap a status heading to whole degrees.
 * @param degrees Heading in degrees; all floating inputs are handled.
 * @return Zero for NaN. Otherwise clamp to +/-1000000, round halfway away
 * from zero, and reduce modulo 360 to a result from 0 through 359.
 * @note The clamp bounds the signed integer conversion, including infinities.
 */
static uint32_t heading_pack (float32_t degrees)
{

    if (degrees != degrees)
    {
        return 0U;
    }

    if (degrees > HEADING_LIMIT_DEG)
    {
        degrees = HEADING_LIMIT_DEG;
    }

    if (degrees < -HEADING_LIMIT_DEG)
    {
        degrees = -HEADING_LIMIT_DEG;
    }

    int32_t whole =
        (int32_t)((degrees < 0.0f) ? (degrees - 0.5f) : (degrees + 0.5f));
    whole %= HEADING_TURN_DEG;

    if (whole < 0)
    {
        whole += HEADING_TURN_DEG;
    }

    /* The modulo and negative correction leave a result in 0..359. */
    return (uint32_t)whole;
}

/* ---- Fault bits ---------------------------------------------------------- */

/** @brief Select the lowest numbered fault represented by a mask.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
motion_fault_t motion_pack_fault_first (uint8_t fault_bits)
{
    uint32_t bit_index;

    for (bit_index = 0U; bit_index < MOTION_FAULT_BITS_WIDTH; bit_index++)
    {
        if ((((uint32_t)fault_bits >> bit_index) & 1U) != 0U)
        {
            return (motion_fault_t)(bit_index + 1U);
        }
    }

    return MOTION_FAULT_NONE;
}

/* ---- Status -------------------------------------------------------------- */

/** @brief Quantize a status snapshot into the 64-bit status layout.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint64_t motion_pack_status (const motion_status_t * p_status)
{
    uint64_t word = 0U;

    if (NULL == p_status)
    {
        return 0U;
    }

    word |= field_put((uint32_t)p_status->state, ST_STATE_LSB, STATE_BITS);
    word |= field_put((uint32_t)p_status->fault_bits |
                          (uint32_t)motion_pack_fault_bit(p_status->fault),
                      ST_FAULT_LSB, FAULT_MASK_BITS);
    word |= field_put(p_status->command_id, ST_ID_LSB, ID_BITS);
    word |=
        field_put(sm_pack(p_status->speed_left_mps, MOTION_PACK_SPEED_STEP_MPS,
                          MOTION_PACK_SPEED_MAG_BITS),
                  ST_SPEED_L_LSB, SPEED_BITS);
    word |=
        field_put(sm_pack(p_status->speed_right_mps, MOTION_PACK_SPEED_STEP_MPS,
                          MOTION_PACK_SPEED_MAG_BITS),
                  ST_SPEED_R_LSB, SPEED_BITS);
    word |= field_put(sm_pack(p_status->distance_m, MOTION_PACK_DIST_STEP_M,
                              MOTION_PACK_DIST_MAG_BITS),
                      ST_DIST_LSB, DIST_BITS);
    word |= field_put(heading_pack(p_status->heading_deg), ST_HEADING_LSB,
                      MOTION_PACK_HEADING_BITS);
    word |= field_put(p_status->b_is_stopped ? 1U : 0U, ST_STOPPED_BIT, 1U);
    word |= field_put(p_status->b_is_busy ? 1U : 0U, ST_BUSY_BIT, 1U);
    word |= field_put(p_status->b_is_status_valid ? 1U : 0U, ST_VALID_BIT, 1U);

    return word;
}

/** @brief Decode every field of a raw 64-bit status word.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
void motion_pack_read_status (uint64_t word, motion_status_t * p_status)
{
    if (NULL == p_status)
    {
        return;
    }

    /* Decode the raw three-bit state. The fault mask and ID widths are
     * at most eight bits each. */
    /* clang-format off */
    p_status->state           =
        (motion_state_t)field_get(word, ST_STATE_LSB, STATE_BITS);
    p_status->fault_bits      =
        (uint8_t)field_get(word, ST_FAULT_LSB, FAULT_MASK_BITS);
    p_status->fault           = motion_pack_fault_first(p_status->fault_bits);
    p_status->command_id      = (uint8_t)field_get(word, ST_ID_LSB, ID_BITS);
    p_status->speed_left_mps  =
        sm_unpack(field_get(word, ST_SPEED_L_LSB, SPEED_BITS),
                  MOTION_PACK_SPEED_STEP_MPS, MOTION_PACK_SPEED_MAG_BITS);
    p_status->speed_right_mps =
        sm_unpack(field_get(word, ST_SPEED_R_LSB, SPEED_BITS),
                  MOTION_PACK_SPEED_STEP_MPS, MOTION_PACK_SPEED_MAG_BITS);
    p_status->distance_m      =
        sm_unpack(field_get(word, ST_DIST_LSB, DIST_BITS),
                  MOTION_PACK_DIST_STEP_M, MOTION_PACK_DIST_MAG_BITS);
    /* clang-format on */
    /* All nine-bit heading values convert exactly to float32_t. */
    /* clang-format off */
    p_status->heading_deg       =
        (float32_t)field_get(word, ST_HEADING_LSB, MOTION_PACK_HEADING_BITS);
    p_status->b_is_stopped      = (field_get(word, ST_STOPPED_BIT, 1U) != 0U);
    p_status->b_is_busy         = (field_get(word, ST_BUSY_BIT, 1U) != 0U);
    p_status->b_is_status_valid = (field_get(word, ST_VALID_BIT, 1U) != 0U);
    /* clang-format on */
}

/* ---- Report -------------------------------------------------------------- */

/** @brief Encode a completion report in the low 14 bits of a word.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint16_t motion_pack_report (const motion_report_t * p_report)
{
    uint64_t word = 0U;

    if (NULL == p_report)
    {
        return 0U;
    }

    word |= field_put(p_report->command_id, RP_ID_LSB, ID_BITS);
    word |=
        field_put((uint32_t)p_report->outcome, RP_OUTCOME_LSB, OUTCOME_BITS);
    word |= field_put((uint32_t)p_report->fault, RP_FAULT_LSB, FAULT_BITS);
    word |= field_put(p_report->b_was_lost ? 1U : 0U, RP_LOST_BIT, 1U);

    /* The report uses bits 0..13, all representable in uint16_t. */
    return (uint16_t)word;
}

/** @brief Decode the report fields without consuming a report queue.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
void motion_pack_read_report (uint16_t word, motion_report_t * p_report)
{
    if (NULL == p_report)
    {
        return;
    }

    /* The ID field is eight bits. Store its full range, including zero. */
    /* clang-format off */
    p_report->command_id = (uint8_t)field_get(word, RP_ID_LSB, ID_BITS);
    p_report->outcome    =
        (motion_outcome_t)field_get(word, RP_OUTCOME_LSB, OUTCOME_BITS);
    p_report->fault      =
        (motion_fault_t)field_get(word, RP_FAULT_LSB, FAULT_BITS);
    p_report->b_was_lost = (field_get(word, RP_LOST_BIT, 1U) != 0U);
    /* clang-format on */
}

/* ---- Commands ------------------------------------------------------------ */

/* Count the used type and payload bits. Unused bits must be zero. */
#define CMD_VELOCITY_BITS (TYPE_BITS + (2U * SPEED_BITS)) /* 23 */
#define CMD_DRIVE_BITS (TYPE_BITS + DRIVE_BITS + MOVE_SPEED_BITS)
#define CMD_TURN_BITS (TYPE_BITS + ANGLE_BITS + MOVE_SPEED_BITS)
#define CMD_MANOEUVRE_BITS (TYPE_BITS + MANOEUVRE_BITS) /*  5 */
#define CMD_CANCEL_BITS (TYPE_BITS + ID_BITS)           /* 11 */

/** @brief Place a command type in the low three bits.
 * @param type Command type; conversion is masked to three bits.
 * @return Type bits with all payload bits clear; no validity check is made.
 */
static uint32_t cmd_type (motion_pack_cmd_type_t type)
{
    return (uint32_t)field_put((uint32_t)type, 0U, TYPE_BITS);
}

/* Accept only a finite value whose rounded magnitude fits the field. */
/** @brief Check whether a finite signed value rounds into a magnitude field.
 * @param value Signed quantity to test.
 * @param step Positive finite quantization step.
 * @param mag_bits Magnitude width, 9 or 13 for current command callers.
 * @return True exactly when value is finite and the floating expression
 * abs(value)/step + 0.5 is below two to the power mag_bits.
 * @note Does not quantize or change output; thresholds follow float
 * arithmetic.
 */
static bool value_fits_field (float32_t value, float32_t step,
                              uint32_t mag_bits)
{
    return (is_finite(value)) && (((abs_f(value) / step) + 0.5f) <
                                  (float32_t)(mask_of(mag_bits) + 1U));
}

/* A drive or turn speed must fit the field and reach at least one step. */
/** @brief Validate the rounded positive drive/turn speed field.
 * @param speed_mps Requested wheel-speed magnitude in metres per second.
 * @return True for finite input rounding to 1..511 steps of 0.002 m/s.
 * @note Rejects negative, zero and rounded-zero speed; a positive value below
 * one complete step can pass when it rounds up. No controller limits are read.
 */
static bool move_speed_ok (float32_t speed_mps)
{
    return (value_fits_field(speed_mps, MOTION_PACK_SPEED_STEP_MPS,
                             MOVE_SPEED_BITS)) &&
           (to_steps(speed_mps, MOTION_PACK_SPEED_STEP_MPS, MOVE_SPEED_BITS) !=
            0U);
}

/** @brief Build a quantized two-wheel velocity command.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint32_t motion_pack_velocity (float32_t left_mps, float32_t right_mps)
{
    const uint32_t left  = sm_pack(left_mps, MOTION_PACK_SPEED_STEP_MPS,
                                   MOTION_PACK_SPEED_MAG_BITS);
    const uint32_t right = sm_pack(right_mps, MOTION_PACK_SPEED_STEP_MPS,
                                   MOTION_PACK_SPEED_MAG_BITS);

    if ((!value_fits_field(left_mps, MOTION_PACK_SPEED_STEP_MPS,
                           MOTION_PACK_SPEED_MAG_BITS)) ||
        (!value_fits_field(right_mps, MOTION_PACK_SPEED_STEP_MPS,
                           MOTION_PACK_SPEED_MAG_BITS)))
    {
        return MOTION_CMD_INVALID_WORD;
    }

    return cmd_type(MOTION_CMD_VELOCITY) |
           (uint32_t)field_put(left, CMD_ARG_LSB, SPEED_BITS) |
           (uint32_t)field_put(right, CMD_ARG_LSB + SPEED_BITS, SPEED_BITS);
}

/** @brief Build a signed-distance drive command with positive speed.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint32_t motion_pack_drive (float32_t meters, float32_t speed_mps)
{
    const uint32_t dist =
        sm_pack(meters, MOTION_PACK_DIST_STEP_M, MOTION_PACK_DRIVE_MAG_BITS);
    const uint32_t speed =
        to_steps(speed_mps, MOTION_PACK_SPEED_STEP_MPS, MOVE_SPEED_BITS);

    if ((!value_fits_field(meters, MOTION_PACK_DIST_STEP_M,
                           MOTION_PACK_DRIVE_MAG_BITS)) ||
        (!move_speed_ok(speed_mps)))
    {
        return MOTION_CMD_INVALID_WORD;
    }

    return cmd_type(MOTION_CMD_DRIVE) |
           (uint32_t)field_put(dist, CMD_ARG_LSB, DRIVE_BITS) |
           (uint32_t)field_put(speed, CMD_ARG_LSB + DRIVE_BITS,
                               MOVE_SPEED_BITS);
}

/** @brief Build a signed-angle turn command with positive wheel speed.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint32_t motion_pack_turn (float32_t degrees, float32_t speed_mps)
{
    /* Positive angles turn left. Right turns set the direction bit. */
    const uint32_t angle = sm_pack(degrees, MOTION_PACK_ANGLE_STEP_DEG,
                                   MOTION_PACK_ANGLE_MAG_BITS);
    const uint32_t speed =
        to_steps(speed_mps, MOTION_PACK_SPEED_STEP_MPS, MOVE_SPEED_BITS);

    if ((!value_fits_field(degrees, MOTION_PACK_ANGLE_STEP_DEG,
                           MOTION_PACK_ANGLE_MAG_BITS)) ||
        (!move_speed_ok(speed_mps)))
    {
        return MOTION_CMD_INVALID_WORD;
    }

    return cmd_type(MOTION_CMD_TURN) |
           (uint32_t)field_put(angle, CMD_ARG_LSB, ANGLE_BITS) |
           (uint32_t)field_put(speed, CMD_ARG_LSB + ANGLE_BITS,
                               MOVE_SPEED_BITS);
}

/** @brief Build a command for one of the four named manoeuvres.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint32_t motion_pack_manoeuvre (motion_manoeuvre_t manoeuvre)
{
    /* The unsigned comparison also rejects a negative code because it
     * converts above the manoeuvre range. */
    if ((uint32_t)manoeuvre > (uint32_t)MANOEUVRE_U_TURN)
    {
        return MOTION_CMD_INVALID_WORD;
    }

    return cmd_type(MOTION_CMD_MANOEUVRE) |
           (uint32_t)field_put((uint32_t)manoeuvre, CMD_ARG_LSB,
                               MANOEUVRE_BITS);
}

/** @brief Build a STOP command with no payload.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint32_t motion_pack_stop (void)
{
    return cmd_type(MOTION_CMD_STOP);
}

/** @brief Build a cancellation word for an eight-bit command ID.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint32_t motion_cmd_cancel (uint8_t command_id)
{
    return cmd_type(MOTION_CMD_CANCEL) |
           (uint32_t)field_put(command_id, CMD_ARG_LSB, ID_BITS);
}

/** @brief Build a RESET command with no payload.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint32_t motion_pack_reset (void)
{
    return cmd_type(MOTION_CMD_RESET);
}

/* Return the valid word width for the command. */
/** @brief Return the envelope width permitted for a command type.
 * @param type Numeric type; current decoding supplies a three-bit value.
 * @return Type plus payload width for payload-bearing commands, or three for
 * STOP, RESET and any unrecognized value. The caller rejects unknown types.
 */
static uint32_t cmd_bits (uint32_t type)
{
    uint32_t bits;

    switch (type)
    {
    case MOTION_CMD_VELOCITY:
        bits = CMD_VELOCITY_BITS;
    break;
    case MOTION_CMD_DRIVE:
        bits = CMD_DRIVE_BITS;
    break;
    case MOTION_CMD_TURN:
        bits = CMD_TURN_BITS;
    break;
    case MOTION_CMD_MANOEUVRE:
        bits = CMD_MANOEUVRE_BITS;
    break;
    case MOTION_CMD_CANCEL:
        bits = CMD_CANCEL_BITS;
    break;
    default:
        bits = TYPE_BITS;
    break;
    }

    return bits;
}

/** @brief Validate the command envelope and decode its payload.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
bool motion_pack_decode (uint32_t word, motion_pack_cmd_t * p_cmd)
{
    const uint32_t type = field_get(word, 0U, TYPE_BITS);

    /* Reject an unknown type or nonzero unused bits. */
    if (((NULL == p_cmd) || (type > (uint32_t)MOTION_CMD_RESET)) ||
        ((word & ~mask_of(cmd_bits(type))) != 0U))
    {
        return false;
    }
    /* A drive or turn requires a speed of at least one step. */
    if ((((uint32_t)MOTION_CMD_DRIVE == type) &&
         (field_get(word, CMD_ARG_LSB + DRIVE_BITS, MOVE_SPEED_BITS) == 0U)) ||
        (((uint32_t)MOTION_CMD_TURN == type) &&
         (field_get(word, CMD_ARG_LSB + ANGLE_BITS, MOVE_SPEED_BITS) == 0U)))
    {
        return false;
    }

    p_cmd->type       = (motion_pack_cmd_type_t)type;
    p_cmd->left_mps   = 0.0f;
    p_cmd->right_mps  = 0.0f;
    p_cmd->meters     = 0.0f;
    p_cmd->degrees    = 0.0f;
    p_cmd->speed_mps  = 0.0f;
    p_cmd->manoeuvre  = MANOEUVRE_STRAIGHT;
    p_cmd->command_id = 0U;

    switch (p_cmd->type)
    {
    case MOTION_CMD_VELOCITY:
        p_cmd->left_mps =
            sm_unpack(field_get(word, CMD_ARG_LSB, SPEED_BITS),
                      MOTION_PACK_SPEED_STEP_MPS, MOTION_PACK_SPEED_MAG_BITS);
        p_cmd->right_mps =
            sm_unpack(field_get(word, CMD_ARG_LSB + SPEED_BITS, SPEED_BITS),
                      MOTION_PACK_SPEED_STEP_MPS, MOTION_PACK_SPEED_MAG_BITS);
    break;

    case MOTION_CMD_DRIVE:
        /* The unsigned move-speed field fits exactly in float32_t. */
        p_cmd->meters =
            sm_unpack(field_get(word, CMD_ARG_LSB, DRIVE_BITS),
                      MOTION_PACK_DIST_STEP_M, MOTION_PACK_DRIVE_MAG_BITS);
        p_cmd->speed_mps = (float32_t)field_get(word, CMD_ARG_LSB + DRIVE_BITS,
                                                MOVE_SPEED_BITS) *
                           MOTION_PACK_SPEED_STEP_MPS;
    break;

    case MOTION_CMD_TURN:
        /* Turn speed uses the same exact unsigned field as drive speed. */
        p_cmd->degrees =
            sm_unpack(field_get(word, CMD_ARG_LSB, ANGLE_BITS),
                      MOTION_PACK_ANGLE_STEP_DEG, MOTION_PACK_ANGLE_MAG_BITS);
        p_cmd->speed_mps = (float32_t)field_get(word, CMD_ARG_LSB + ANGLE_BITS,
                                                MOVE_SPEED_BITS) *
                           MOTION_PACK_SPEED_STEP_MPS;
    break;

    case MOTION_CMD_MANOEUVRE:
        p_cmd->manoeuvre =
            (motion_manoeuvre_t)field_get(word, CMD_ARG_LSB, MANOEUVRE_BITS);
    break;

    case MOTION_CMD_CANCEL:
        /* The cancellation payload is an eight-bit command identifier. */
        p_cmd->command_id = (uint8_t)field_get(word, CMD_ARG_LSB, ID_BITS);
    break;

    default: /* STOP and RESET carry only their type. */
    break;
    }

    return true;
}

/* End of file. */

