/*
 * File: motion_pack.h
 * Owner: Buddy 2 - Motion Control
 * Purpose: Minimum-bit representation of the data Buddy 2 shares
 *          (status / telemetry, reports, commands).
 *
 * Layouts, ranges and reasons are in DATA_FORMAT.md (Proposed, to agree with
 * the team). Signed values use a direction bit + magnitude (1 = negative:
 * backward, or right for turns). Values are rounded to the step and
 * saturate at the largest magnitude; they never wrap. Buddy 2 keeps full
 * precision internally; only these words leave it.
 *
 * Faults are individual bits (MOTION_FAULT_BIT()): Buddy 2's 5 bits of the
 * shared fault bitmask. The status word carries them; where they sit in the
 * team's shared mask is still to be agreed. A report carries the 3-bit code
 * of the fault that ended its command.
 */

#ifndef BUDDY2_MOTION_PACK_H
#define BUDDY2_MOTION_PACK_H

#include <stdbool.h>
#include <stdint.h>
#include "motion.h"

/* Steps and magnitude widths (change here if the team agrees otherwise) */
#define MOTION_PACK_SPEED_STEP_MPS  (0.002f)    /* 2 mm/s                     */
#define MOTION_PACK_SPEED_MAG_BITS  (9U)        /* 0 ... 1022 mm/s            */
#define MOTION_PACK_MOVE_SPEED_BITS (9U)        /* 0 ... 1022 mm/s (command)  */
#define MOTION_PACK_DIST_STEP_M     (0.001f)    /* 1 mm                       */
#define MOTION_PACK_DIST_MAG_BITS   (15U)       /* 0 ... 32.767 m (status)    */
#define MOTION_PACK_DRIVE_MAG_BITS  (13U)       /* 0 ... 8.191 m (command)    */
#define MOTION_PACK_ANGLE_STEP_DEG  (1.0f)
#define MOTION_PACK_ANGLE_MAG_BITS  (9U)        /* 0 ... 511 deg (command)    */
#define MOTION_PACK_HEADING_BITS    (9U)        /* 0 ... 359 deg (status)     */

/* Buddy 2's fault bits: fault code n is bit n - 1 (stall = bit 0 ...
 * timeout = bit 4). Several can be set at once. */
#define MOTION_FAULT_BITS_WIDTH     (5U)
#define MOTION_FAULT_BIT(f)         ((uint8_t)(((f) == MOTION_FAULT_NONE) ? 0U \
                                     : (1U << ((uint32_t)(f) - 1U))))

/* Lowest set fault bit as a fault code; MOTION_FAULT_NONE if none is set */
motion_fault_t motion_fault_first(uint8_t fault_bits);

/* Command types (bits 0-2 of a command word) */
typedef enum
{
    MOTION_CMD_VELOCITY = 0,
    MOTION_CMD_DRIVE,
    MOTION_CMD_TURN,
    MOTION_CMD_MANOEUVRE,
    MOTION_CMD_STOP,
    MOTION_CMD_CANCEL,
    MOTION_CMD_RESET
} motion_cmd_type_t;

/* A command word unpacked */
typedef struct
{
    motion_cmd_type_t type;
    float             left_mps;     /* VELOCITY                          */
    float             right_mps;    /* VELOCITY                          */
    float             meters;       /* DRIVE, negative = backward        */
    float             degrees;      /* TURN, positive = left             */
    float             speed_mps;    /* DRIVE, TURN                       */
    manoeuvre_t       manoeuvre;    /* MANOEUVRE                         */
    uint8_t           command_id;   /* CANCEL                            */
} motion_cmd_t;

/* Status / telemetry: 64 bits */
uint64_t motion_pack_status(const motion_status_t *p_status);
void     motion_unpack_status(uint64_t word, motion_status_t *p_status);

/* Done / cancelled / fault report: 16 bits */
uint16_t motion_pack_report(const motion_report_t *p_report);
void     motion_unpack_report(uint16_t word, motion_report_t *p_report);

/* Type 7 is not a command: what the builders return for invalid input */
#define MOTION_CMD_INVALID_WORD     (7U)

/* Commands (Vehicle Controller side): 32 bits. Send them with
 * motion_request_packed(). Valid input:
 *   speeds   finite, |v| <= 1.022 m/s (velocity: either sign);
 *            drive / turn speed > 0 (at least 2 mm/s)
 *   meters   finite, |m| <= 8.191 m          degrees  finite, |d| <= 511
 *   manoeuvre MANOEUVRE_LEFT ... MANOEUVRE_U_TURN
 * Anything else (NaN, infinity, negative or zero move speed, out of range,
 * unknown manoeuvre) gives MOTION_CMD_INVALID_WORD, which
 * motion_request_packed() rejects with MOTION_ERR_BAD_ARG: never a
 * clipped or different command. */
uint32_t motion_cmd_velocity(float left_mps, float right_mps);
uint32_t motion_cmd_drive(float meters, float speed_mps);
uint32_t motion_cmd_turn(float degrees, float speed_mps);
uint32_t motion_cmd_manoeuvre(manoeuvre_t m);
uint32_t motion_cmd_stop(void);
uint32_t motion_cmd_cancel(uint8_t command_id);
uint32_t motion_cmd_reset(void);

/* False if the type is unknown, any bit outside the type's fields is set,
 * or a drive / turn has speed 0 */
bool     motion_unpack_cmd(uint32_t word, motion_cmd_t *p_cmd);

#endif /* BUDDY2_MOTION_PACK_H */
