/** @file motion_pack.c
 * @brief Minimum-bit representation of the data Buddy 2 shares.
 */
/*
 * File: motion_pack.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Minimum-bit representation of the data Buddy 2 shares.
 *
 * Packed with shifts and masks (not C bit-fields), so the layout is the same
 * on every compiler and the laptop decoder can follow DATA_FORMAT.md exactly.
 */

#include <stddef.h>

#include "motion_pack.h"
#include "../app_types.h"

/* Field widths */
#define STATE_BITS (3U)
#define FAULT_BITS (3U)                           /* fault code (report) */
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

/* Status word, 64 bits, all used (DATA_FORMAT.md) */
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

/* Report word, 16 bits */
#define RP_ID_LSB (0U)
#define RP_OUTCOME_LSB (RP_ID_LSB + ID_BITS)         /*  8 */
#define RP_FAULT_LSB (RP_OUTCOME_LSB + OUTCOME_BITS) /* 10 */
#define RP_LOST_BIT (RP_FAULT_LSB + FAULT_BITS)      /* 13 */

/* Command word, 32 bits: type in bits 0-2, payload from bit 3 */
#define CMD_ARG_LSB (TYPE_BITS)

#define HEADING_TURN_DEG (360)
#define HEADING_LIMIT_DEG (1000000.0f) /* keeps the int cast safe */

/** @brief Apply the existing sign comparison used for magnitude conversion.
 * @param value_x Floating value, including signed zero or non-finite input.
 * @return Negated input when it compares below zero, otherwise the input.
 * @note Negative zero retains its sign; NaN is passed through. No state
 * changes.
 */
static float32_t abs_f (float32_t value_x)
{
    return (value_x < 0.0f) ? -value_x : value_x;
}

/** @brief Create a low-bit mask without shifting by 32 or more.
 * @param bits Requested bit count; all uint32_t values are accepted.
 * @return Zero for zero bits, the low-bit mask for 1..31, or UINT32_MAX for
 * 32 or more. Used only as a value; no stored object is modified.
 */
static uint32_t mask_of (uint32_t bits)
{
    return (bits >= 32U) ? 0xFFFFFFFFU : ((1UL << bits) - 1U);
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
    return (uint32_t)(word >> lsb) & mask_of(bits);
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
    return (uint64_t)(value & mask_of(bits)) << lsb;
}

/** @brief Test finiteness using the retained floating arithmetic checks.
 * @param value_x Value to classify.
 * @return True for finite values, including both zeros; false for NaN/infinity
 * under the project's floating compiler semantics.
 * @note Uses self-comparison and subtraction; floating exception flags are not
 * managed here. This documents the existing classification, not a replacement.
 */
static bool is_finite (float32_t value_x)
{
    return (value_x == value_x) &&
           ((value_x - value_x) == 0.0f); /* rejects NaN and infinity */
}

/* Rounded, saturated magnitude in steps; NaN or negative gives 0 (no cast
 * of a negative float to unsigned) */
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
    uint32_t max_steps = mask_of(bits);
    float32_t steps = (magnitude / step) + 0.5f;

    if (!(steps >= 1.0f))
    {
        return 0U;
    }
    if (steps >= (float32_t)max_steps)
    {
        return max_steps;
    }

    return (uint32_t)steps;
}

/* Direction bit + magnitude; never a "negative zero" */
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
    uint32_t mag = to_steps(abs_f(value), step, mag_bits);
    uint32_t dir = ((value < 0.0f) && (mag != 0U)) ? 1U : 0U;

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
    float32_t value = (float32_t)(field & mask_of(mag_bits)) * step;

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
    int32_t whole;

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
    whole = (int32_t)((degrees < 0.0f) ? (degrees - 0.5f) : (degrees + 0.5f));
    whole %= HEADING_TURN_DEG;
    if (whole < 0)
    {
        whole += HEADING_TURN_DEG;
    }

    return (uint32_t)whole;
}

/* ---- Fault bits ---------------------------------------------------------- */

/** @brief Select the lowest numbered fault represented by a mask.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
motion_fault_t motion_fault_first (uint8_t fault_bits)
{
    uint32_t value_i;

    for (value_i = 0U; value_i < MOTION_FAULT_BITS_WIDTH; value_i++)
    {
        if ((((uint32_t)fault_bits >> value_i) & 1U) != 0U)
        {
            return (motion_fault_t)(value_i + 1U);
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

    if (p_status == NULL)
    {
        return 0U;
    }
    word |= field_put((uint32_t)p_status->state, ST_STATE_LSB, STATE_BITS);
    word |= field_put((uint32_t)p_status->fault_bits |
                          (uint32_t)motion_fault_bit(p_status->fault),
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
    word |= field_put(p_status->b_stopped ? 1U : 0U, ST_STOPPED_BIT, 1U);
    word |= field_put(p_status->b_busy ? 1U : 0U, ST_BUSY_BIT, 1U);
    word |= field_put(p_status->b_valid ? 1U : 0U, ST_VALID_BIT, 1U);

    return word;
}

/** @brief Decode every field of a raw 64-bit status word.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
void motion_unpack_status (uint64_t word, motion_status_t * p_status)
{
    if (p_status == NULL)
    {
        return;
    }
    p_status->state = (motion_state_t)field_get(word, ST_STATE_LSB, STATE_BITS);
    p_status->fault_bits =
        (uint8_t)field_get(word, ST_FAULT_LSB, FAULT_MASK_BITS);
    p_status->fault = motion_fault_first(p_status->fault_bits);
    p_status->command_id = (uint8_t)field_get(word, ST_ID_LSB, ID_BITS);
    p_status->speed_left_mps =
        sm_unpack(field_get(word, ST_SPEED_L_LSB, SPEED_BITS),
                  MOTION_PACK_SPEED_STEP_MPS, MOTION_PACK_SPEED_MAG_BITS);
    p_status->speed_right_mps =
        sm_unpack(field_get(word, ST_SPEED_R_LSB, SPEED_BITS),
                  MOTION_PACK_SPEED_STEP_MPS, MOTION_PACK_SPEED_MAG_BITS);
    p_status->distance_m =
        sm_unpack(field_get(word, ST_DIST_LSB, DIST_BITS),
                  MOTION_PACK_DIST_STEP_M, MOTION_PACK_DIST_MAG_BITS);
    p_status->heading_deg =
        (float32_t)field_get(word, ST_HEADING_LSB, MOTION_PACK_HEADING_BITS);
    p_status->b_stopped = (field_get(word, ST_STOPPED_BIT, 1U) != 0U);
    p_status->b_busy = (field_get(word, ST_BUSY_BIT, 1U) != 0U);
    p_status->b_valid = (field_get(word, ST_VALID_BIT, 1U) != 0U);
}

/* ---- Report -------------------------------------------------------------- */

/** @brief Encode a completion report in the low 14 bits of a word.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint16_t motion_pack_report (const motion_report_t * p_report)
{
    uint64_t word = 0U;

    if (p_report == NULL)
    {
        return 0U;
    }
    word |= field_put(p_report->command_id, RP_ID_LSB, ID_BITS);
    word |=
        field_put((uint32_t)p_report->outcome, RP_OUTCOME_LSB, OUTCOME_BITS);
    word |= field_put((uint32_t)p_report->fault, RP_FAULT_LSB, FAULT_BITS);
    word |= field_put(p_report->b_lost ? 1U : 0U, RP_LOST_BIT, 1U);

    return (uint16_t)word;
}

/** @brief Decode the report fields without consuming a report queue.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
void motion_unpack_report (uint16_t word, motion_report_t * p_report)
{
    if (p_report == NULL)
    {
        return;
    }
    p_report->command_id = (uint8_t)field_get(word, RP_ID_LSB, ID_BITS);
    p_report->outcome =
        (motion_outcome_t)field_get(word, RP_OUTCOME_LSB, OUTCOME_BITS);
    p_report->fault = (motion_fault_t)field_get(word, RP_FAULT_LSB, FAULT_BITS);
    p_report->b_lost = (field_get(word, RP_LOST_BIT, 1U) != 0U);
}

/* ---- Commands ------------------------------------------------------------ */

/* Bits each command type uses (type + payload); others must be 0 */
#define CMD_VELOCITY_BITS (TYPE_BITS + (2U * SPEED_BITS)) /* 23 */
#define CMD_DRIVE_BITS (TYPE_BITS + DRIVE_BITS + MOVE_SPEED_BITS)
#define CMD_TURN_BITS (TYPE_BITS + ANGLE_BITS + MOVE_SPEED_BITS)
#define CMD_MANOEUVRE_BITS (TYPE_BITS + MANOEUVRE_BITS) /*  5 */
#define CMD_CANCEL_BITS (TYPE_BITS + ID_BITS)           /* 11 */

/** @brief Place a command type in the low three bits.
 * @param type Command type; conversion is masked to three bits.
 * @return Type bits with all payload bits clear; no validity check is made.
 */
static uint32_t cmd_type (motion_cmd_type_t type)
{
    return (uint32_t)field_put((uint32_t)type, 0U, TYPE_BITS);
}

/* Finite and within the field after rounding (no silent saturation) */
/** @brief Check whether a finite signed value rounds into a magnitude field.
 * @param value Signed quantity to test.
 * @param step Positive finite quantization step.
 * @param mag_bits Magnitude width, 9 or 13 for current command callers.
 * @return True exactly when value is finite and the floating expression
 * abs(value)/step + 0.5 is below two to the power mag_bits.
 * @note Does not quantize or change output; thresholds follow float
 * arithmetic.
 */
static bool fits (float32_t value, float32_t step, uint32_t mag_bits)
{
    return is_finite(value) && (((abs_f(value) / step) + 0.5f) <
                                (float32_t)(mask_of(mag_bits) + 1U));
}

/* A drive / turn speed: finite, at least one step, within the field */
/** @brief Validate the rounded positive drive/turn speed field.
 * @param speed_mps Requested wheel-speed magnitude in metres per second.
 * @return True for finite input rounding to 1..511 steps of 0.002 m/s.
 * @note Rejects negative, zero and rounded-zero speed; a positive value below
 * one complete step can pass when it rounds up. No controller limits are read.
 */
static bool move_speed_ok (float32_t speed_mps)
{
    return fits(speed_mps, MOTION_PACK_SPEED_STEP_MPS, MOVE_SPEED_BITS) &&
           (to_steps(speed_mps, MOTION_PACK_SPEED_STEP_MPS, MOVE_SPEED_BITS) !=
            0U);
}

/** @brief Build a quantized two-wheel velocity command.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint32_t motion_cmd_velocity (float32_t left_mps, float32_t right_mps)
{
    uint32_t left = sm_pack(left_mps, MOTION_PACK_SPEED_STEP_MPS,
                            MOTION_PACK_SPEED_MAG_BITS);
    uint32_t right = sm_pack(right_mps, MOTION_PACK_SPEED_STEP_MPS,
                             MOTION_PACK_SPEED_MAG_BITS);

    if (!fits(left_mps, MOTION_PACK_SPEED_STEP_MPS,
              MOTION_PACK_SPEED_MAG_BITS) ||
        !fits(right_mps, MOTION_PACK_SPEED_STEP_MPS,
              MOTION_PACK_SPEED_MAG_BITS))
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
uint32_t motion_cmd_drive (float32_t meters, float32_t speed_mps)
{
    uint32_t dist =
        sm_pack(meters, MOTION_PACK_DIST_STEP_M, MOTION_PACK_DRIVE_MAG_BITS);
    uint32_t speed =
        to_steps(speed_mps, MOTION_PACK_SPEED_STEP_MPS, MOVE_SPEED_BITS);

    if (!fits(meters, MOTION_PACK_DIST_STEP_M, MOTION_PACK_DRIVE_MAG_BITS) ||
        !move_speed_ok(speed_mps))
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
uint32_t motion_cmd_turn (float32_t degrees, float32_t speed_mps)
{
    /* Positive = left; a negative angle (right) sets the direction bit */
    uint32_t angle = sm_pack(degrees, MOTION_PACK_ANGLE_STEP_DEG,
                             MOTION_PACK_ANGLE_MAG_BITS);
    uint32_t speed =
        to_steps(speed_mps, MOTION_PACK_SPEED_STEP_MPS, MOVE_SPEED_BITS);

    if (!fits(degrees, MOTION_PACK_ANGLE_STEP_DEG,
              MOTION_PACK_ANGLE_MAG_BITS) ||
        !move_speed_ok(speed_mps))
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
uint32_t motion_cmd_manoeuvre (manoeuvre_t value_m)
{
    /* Out-of-range values must not be masked into a different manoeuvre */
    if ((uint32_t)value_m > (uint32_t)MANOEUVRE_U_TURN)
    {
        return MOTION_CMD_INVALID_WORD;
    }

    return cmd_type(MOTION_CMD_MANOEUVRE) |
           (uint32_t)field_put((uint32_t)value_m, CMD_ARG_LSB, MANOEUVRE_BITS);
}

/** @brief Build a STOP command with no payload.
 * @see motion_pack.h for the units, validation and ownership contract.
 */
uint32_t motion_cmd_stop (void)
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
uint32_t motion_cmd_reset (void)
{
    return cmd_type(MOTION_CMD_RESET);
}

/* Number of bits a valid word of this type may use */
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
bool motion_unpack_cmd (uint32_t word, motion_cmd_t * p_cmd)
{
    uint32_t type = field_get(word, 0U, TYPE_BITS);

    /* Unknown type, or bits set outside the type's fields: not a command */
    if ((p_cmd == NULL) || (type > (uint32_t)MOTION_CMD_RESET) ||
        ((word & ~mask_of(cmd_bits(type))) != 0U))
    {
        return false;
    }
    /* A drive / turn needs a speed of at least one step */
    if (((type == (uint32_t)MOTION_CMD_DRIVE) &&
         (field_get(word, CMD_ARG_LSB + DRIVE_BITS, MOVE_SPEED_BITS) == 0U)) ||
        ((type == (uint32_t)MOTION_CMD_TURN) &&
         (field_get(word, CMD_ARG_LSB + ANGLE_BITS, MOVE_SPEED_BITS) == 0U)))
    {
        return false;
    }
    p_cmd->type = (motion_cmd_type_t)type;
    p_cmd->left_mps = 0.0f;
    p_cmd->right_mps = 0.0f;
    p_cmd->meters = 0.0f;
    p_cmd->degrees = 0.0f;
    p_cmd->speed_mps = 0.0f;
    p_cmd->manoeuvre = MANOEUVRE_STRAIGHT;
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
        p_cmd->meters =
            sm_unpack(field_get(word, CMD_ARG_LSB, DRIVE_BITS),
                      MOTION_PACK_DIST_STEP_M, MOTION_PACK_DRIVE_MAG_BITS);
        p_cmd->speed_mps = (float32_t)field_get(word, CMD_ARG_LSB + DRIVE_BITS,
                                                MOVE_SPEED_BITS) *
                           MOTION_PACK_SPEED_STEP_MPS;
        break;

    case MOTION_CMD_TURN:
        p_cmd->degrees =
            sm_unpack(field_get(word, CMD_ARG_LSB, ANGLE_BITS),
                      MOTION_PACK_ANGLE_STEP_DEG, MOTION_PACK_ANGLE_MAG_BITS);
        p_cmd->speed_mps = (float32_t)field_get(word, CMD_ARG_LSB + ANGLE_BITS,
                                                MOVE_SPEED_BITS) *
                           MOTION_PACK_SPEED_STEP_MPS;
        break;

    case MOTION_CMD_MANOEUVRE:
        p_cmd->manoeuvre =
            (manoeuvre_t)field_get(word, CMD_ARG_LSB, MANOEUVRE_BITS);
        break;

    case MOTION_CMD_CANCEL:
        p_cmd->command_id = (uint8_t)field_get(word, CMD_ARG_LSB, ID_BITS);
        break;

    default: /* STOP, RESET: no payload */
        break;
    }

    return true;
}

/* End of file. */

