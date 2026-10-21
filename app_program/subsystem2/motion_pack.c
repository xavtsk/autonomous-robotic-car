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

/* Field widths */
#define STATE_BITS              (3U)
#define FAULT_BITS              (3U)                /* fault code (report) */
#define FAULT_MASK_BITS         (MOTION_FAULT_BITS_WIDTH) /* bits (status) */
#define ID_BITS                 (8U)
#define OUTCOME_BITS            (2U)
#define TYPE_BITS               (3U)
#define MANOEUVRE_BITS          (2U)
#define MOVE_SPEED_BITS         (MOTION_PACK_MOVE_SPEED_BITS)
#define SPEED_BITS              (MOTION_PACK_SPEED_MAG_BITS + 1U)
#define DIST_BITS               (MOTION_PACK_DIST_MAG_BITS + 1U)
#define DRIVE_BITS              (MOTION_PACK_DRIVE_MAG_BITS + 1U)
#define ANGLE_BITS              (MOTION_PACK_ANGLE_MAG_BITS + 1U)

/* Status word, 64 bits, all used (DATA_FORMAT.md) */
#define ST_STATE_LSB            (0U)
#define ST_FAULT_LSB            (ST_STATE_LSB + STATE_BITS)         /*  3 */
#define ST_ID_LSB               (ST_FAULT_LSB + FAULT_MASK_BITS)    /*  8 */
#define ST_SPEED_L_LSB          (ST_ID_LSB + ID_BITS)               /* 16 */
#define ST_SPEED_R_LSB          (ST_SPEED_L_LSB + SPEED_BITS)       /* 26 */
#define ST_DIST_LSB             (ST_SPEED_R_LSB + SPEED_BITS)       /* 36 */
#define ST_HEADING_LSB          (ST_DIST_LSB + DIST_BITS)           /* 52 */
#define ST_STOPPED_BIT          (ST_HEADING_LSB + MOTION_PACK_HEADING_BITS)
#define ST_BUSY_BIT             (ST_STOPPED_BIT + 1U)               /* 62 */
#define ST_VALID_BIT            (ST_BUSY_BIT + 1U)                  /* 63 */

/* Report word, 16 bits */
#define RP_ID_LSB               (0U)
#define RP_OUTCOME_LSB          (RP_ID_LSB + ID_BITS)               /*  8 */
#define RP_FAULT_LSB            (RP_OUTCOME_LSB + OUTCOME_BITS)     /* 10 */
#define RP_LOST_BIT             (RP_FAULT_LSB + FAULT_BITS)         /* 13 */

/* Command word, 32 bits: type in bits 0-2, payload from bit 3 */
#define CMD_ARG_LSB             (TYPE_BITS)

#define HEADING_TURN_DEG        (360)
#define HEADING_LIMIT_DEG       (1000000.0f)    /* keeps the int cast safe */

static float abs_f(float x)
{
    return (x < 0.0f) ? -x : x;
}

static uint32_t mask_of(uint32_t bits)
{
    return (bits >= 32U) ? 0xFFFFFFFFU : ((1UL << bits) - 1U);
}

static uint32_t field_get(uint64_t word, uint32_t lsb, uint32_t bits)
{
    return (uint32_t)(word >> lsb) & mask_of(bits);
}

static uint64_t field_put(uint32_t value, uint32_t lsb, uint32_t bits)
{
    return (uint64_t)(value & mask_of(bits)) << lsb;
}

/* Rounded, saturated magnitude in steps; NaN gives 0 */
static uint32_t to_steps(float magnitude, float step, uint32_t bits)
{
    uint32_t max_steps = mask_of(bits);
    float    steps = (magnitude / step) + 0.5f;

    if (steps != steps)
    {
        return 0U;
    }
    if (steps >= (float)max_steps)
    {
        return max_steps;
    }

    return (uint32_t)steps;
}

/* Direction bit + magnitude; never a "negative zero" */
static uint32_t sm_pack(float value, float step, uint32_t mag_bits)
{
    uint32_t mag = to_steps(abs_f(value), step, mag_bits);
    uint32_t dir = ((value < 0.0f) && (mag != 0U)) ? 1U : 0U;

    return (dir << mag_bits) | mag;
}

static float sm_unpack(uint32_t field, float step, uint32_t mag_bits)
{
    float value = (float)(field & mask_of(mag_bits)) * step;

    return (((field >> mag_bits) & 1U) != 0U) ? -value : value;
}

static uint32_t heading_pack(float degrees)
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

motion_fault_t motion_fault_first(uint8_t fault_bits)
{
    uint32_t i;

    for (i = 0U; i < MOTION_FAULT_BITS_WIDTH; i++)
    {
        if ((((uint32_t)fault_bits >> i) & 1U) != 0U)
        {
            return (motion_fault_t)(i + 1U);
        }
    }

    return MOTION_FAULT_NONE;
}

/* ---- Status -------------------------------------------------------------- */

uint64_t motion_pack_status(const motion_status_t *p_status)
{
    uint64_t word = 0U;

    if (p_status == NULL)
    {
        return 0U;
    }
    word |= field_put((uint32_t)p_status->state, ST_STATE_LSB, STATE_BITS);
    word |= field_put((uint32_t)p_status->fault_bits
                      | (uint32_t)MOTION_FAULT_BIT(p_status->fault),
                      ST_FAULT_LSB, FAULT_MASK_BITS);
    word |= field_put(p_status->command_id, ST_ID_LSB, ID_BITS);
    word |= field_put(sm_pack(p_status->speed_left_mps,
                              MOTION_PACK_SPEED_STEP_MPS,
                              MOTION_PACK_SPEED_MAG_BITS),
                      ST_SPEED_L_LSB, SPEED_BITS);
    word |= field_put(sm_pack(p_status->speed_right_mps,
                              MOTION_PACK_SPEED_STEP_MPS,
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

void motion_unpack_status(uint64_t word, motion_status_t *p_status)
{
    if (p_status == NULL)
    {
        return;
    }
    p_status->state = (motion_state_t)field_get(word, ST_STATE_LSB,
                                                STATE_BITS);
    p_status->fault_bits = (uint8_t)field_get(word, ST_FAULT_LSB,
                                              FAULT_MASK_BITS);
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
    p_status->heading_deg = (float)field_get(word, ST_HEADING_LSB,
                                             MOTION_PACK_HEADING_BITS);
    p_status->b_stopped = (field_get(word, ST_STOPPED_BIT, 1U) != 0U);
    p_status->b_busy = (field_get(word, ST_BUSY_BIT, 1U) != 0U);
    p_status->b_valid = (field_get(word, ST_VALID_BIT, 1U) != 0U);
}

/* ---- Report -------------------------------------------------------------- */

uint16_t motion_pack_report(const motion_report_t *p_report)
{
    uint64_t word = 0U;

    if (p_report == NULL)
    {
        return 0U;
    }
    word |= field_put(p_report->command_id, RP_ID_LSB, ID_BITS);
    word |= field_put((uint32_t)p_report->outcome, RP_OUTCOME_LSB,
                      OUTCOME_BITS);
    word |= field_put((uint32_t)p_report->fault, RP_FAULT_LSB, FAULT_BITS);
    word |= field_put(p_report->b_lost ? 1U : 0U, RP_LOST_BIT, 1U);

    return (uint16_t)word;
}

void motion_unpack_report(uint16_t word, motion_report_t *p_report)
{
    if (p_report == NULL)
    {
        return;
    }
    p_report->command_id = (uint8_t)field_get(word, RP_ID_LSB, ID_BITS);
    p_report->outcome = (motion_outcome_t)field_get(word, RP_OUTCOME_LSB,
                                                    OUTCOME_BITS);
    p_report->fault = (motion_fault_t)field_get(word, RP_FAULT_LSB,
                                                FAULT_BITS);
    p_report->b_lost = (field_get(word, RP_LOST_BIT, 1U) != 0U);
}

/* ---- Commands ------------------------------------------------------------ */

static uint32_t cmd_type(motion_cmd_type_t type)
{
    return (uint32_t)field_put((uint32_t)type, 0U, TYPE_BITS);
}

uint32_t motion_cmd_velocity(float left_mps, float right_mps)
{
    uint32_t left = sm_pack(left_mps, MOTION_PACK_SPEED_STEP_MPS,
                            MOTION_PACK_SPEED_MAG_BITS);
    uint32_t right = sm_pack(right_mps, MOTION_PACK_SPEED_STEP_MPS,
                             MOTION_PACK_SPEED_MAG_BITS);

    return cmd_type(MOTION_CMD_VELOCITY)
           | (uint32_t)field_put(left, CMD_ARG_LSB, SPEED_BITS)
           | (uint32_t)field_put(right, CMD_ARG_LSB + SPEED_BITS,
                                 SPEED_BITS);
}

uint32_t motion_cmd_drive(float meters, float speed_mps)
{
    uint32_t dist = sm_pack(meters, MOTION_PACK_DIST_STEP_M,
                            MOTION_PACK_DRIVE_MAG_BITS);
    uint32_t speed = to_steps(speed_mps, MOTION_PACK_SPEED_STEP_MPS,
                              MOVE_SPEED_BITS);

    return cmd_type(MOTION_CMD_DRIVE)
           | (uint32_t)field_put(dist, CMD_ARG_LSB, DRIVE_BITS)
           | (uint32_t)field_put(speed, CMD_ARG_LSB + DRIVE_BITS,
                                 MOVE_SPEED_BITS);
}

uint32_t motion_cmd_turn(float degrees, float speed_mps)
{
    /* Positive = left; a negative angle (right) sets the direction bit */
    uint32_t angle = sm_pack(degrees, MOTION_PACK_ANGLE_STEP_DEG,
                             MOTION_PACK_ANGLE_MAG_BITS);
    uint32_t speed = to_steps(speed_mps, MOTION_PACK_SPEED_STEP_MPS,
                              MOVE_SPEED_BITS);

    return cmd_type(MOTION_CMD_TURN)
           | (uint32_t)field_put(angle, CMD_ARG_LSB, ANGLE_BITS)
           | (uint32_t)field_put(speed, CMD_ARG_LSB + ANGLE_BITS,
                                 MOVE_SPEED_BITS);
}

uint32_t motion_cmd_manoeuvre(manoeuvre_t m)
{
    return cmd_type(MOTION_CMD_MANOEUVRE)
           | (uint32_t)field_put((uint32_t)m, CMD_ARG_LSB, MANOEUVRE_BITS);
}

uint32_t motion_cmd_stop(void)
{
    return cmd_type(MOTION_CMD_STOP);
}

uint32_t motion_cmd_cancel(uint8_t command_id)
{
    return cmd_type(MOTION_CMD_CANCEL)
           | (uint32_t)field_put(command_id, CMD_ARG_LSB, ID_BITS);
}

uint32_t motion_cmd_reset(void)
{
    return cmd_type(MOTION_CMD_RESET);
}

bool motion_unpack_cmd(uint32_t word, motion_cmd_t *p_cmd)
{
    uint32_t type = field_get(word, 0U, TYPE_BITS);

    if ((p_cmd == NULL) || (type > (uint32_t)MOTION_CMD_RESET))
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
        p_cmd->left_mps = sm_unpack(field_get(word, CMD_ARG_LSB, SPEED_BITS),
                                    MOTION_PACK_SPEED_STEP_MPS,
                                    MOTION_PACK_SPEED_MAG_BITS);
        p_cmd->right_mps =
            sm_unpack(field_get(word, CMD_ARG_LSB + SPEED_BITS, SPEED_BITS),
                      MOTION_PACK_SPEED_STEP_MPS, MOTION_PACK_SPEED_MAG_BITS);
        break;

    case MOTION_CMD_DRIVE:
        p_cmd->meters = sm_unpack(field_get(word, CMD_ARG_LSB, DRIVE_BITS),
                                  MOTION_PACK_DIST_STEP_M,
                                  MOTION_PACK_DRIVE_MAG_BITS);
        p_cmd->speed_mps =
            (float)field_get(word, CMD_ARG_LSB + DRIVE_BITS, MOVE_SPEED_BITS)
            * MOTION_PACK_SPEED_STEP_MPS;
        break;

    case MOTION_CMD_TURN:
        p_cmd->degrees = sm_unpack(field_get(word, CMD_ARG_LSB, ANGLE_BITS),
                                   MOTION_PACK_ANGLE_STEP_DEG,
                                   MOTION_PACK_ANGLE_MAG_BITS);
        p_cmd->speed_mps =
            (float)field_get(word, CMD_ARG_LSB + ANGLE_BITS, MOVE_SPEED_BITS)
            * MOTION_PACK_SPEED_STEP_MPS;
        break;

    case MOTION_CMD_MANOEUVRE:
        p_cmd->manoeuvre = (manoeuvre_t)field_get(word, CMD_ARG_LSB,
                                                  MANOEUVRE_BITS);
        break;

    case MOTION_CMD_CANCEL:
        p_cmd->command_id = (uint8_t)field_get(word, CMD_ARG_LSB, ID_BITS);
        break;

    default:                                /* STOP, RESET: no payload */
        break;
    }

    return true;
}
