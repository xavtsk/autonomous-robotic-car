/** @file motion_pack.h
 * @brief Minimum-bit representation of the data Buddy 2 shares (status /
 * telemetry, reports, commands).
 *
 * @par Ownership
 * Buddy 2 - Motion Control
 *
 * Layouts, ranges and reasons are in DATA_FORMAT.md (Proposed, to agree with
 * the team). Signed values use a direction bit + magnitude (1 = negative:
 * backward, or right for turns). Values are rounded to the step and
 * saturate at the largest magnitude; they never wrap. Buddy 2 keeps full
 * precision internally; only these words leave it.
 *
 * Faults are individual bits (motion_pack_fault_bit()): Buddy 2's 5 bits of the
 * shared fault bitmask. The status word carries them; where they sit in the
 * team's shared mask is still to be agreed. A report carries the 3-bit code
 * of the fault that ended its command.
 */

#ifndef BUDDY2_MOTION_PACK_H
#define BUDDY2_MOTION_PACK_H

#include "../app_types.h"
#include <stdbool.h>
#include <stdint.h>
#include "motion.h"

/* These constants define resolution and widths for packed values. */
#define MOTION_PACK_SPEED_STEP_MPS (0.002f) /* Speed resolves to 2 mm/s. */
/* Encode magnitude up to 1022 mm/s. */
#define MOTION_PACK_SPEED_MAG_BITS (9U)
#define MOTION_PACK_MOVE_SPEED_BITS (9U) /* Command up to 1022 mm/s. */
#define MOTION_PACK_DIST_STEP_M (0.001f) /* Distance resolves to 1 mm. */
#define MOTION_PACK_DIST_MAG_BITS (15U)  /* 0 ... 32.767 m (status)    */
#define MOTION_PACK_DRIVE_MAG_BITS (13U) /* 0 ... 8.191 m (command)    */
#define MOTION_PACK_ANGLE_STEP_DEG (1.0f)
/* Command angles up to 511 degrees. */
#define MOTION_PACK_ANGLE_MAG_BITS (9U)
#define MOTION_PACK_HEADING_BITS (9U) /* Status angles range from 0 to 359. */

/* Buddy 2's fault bits: fault code n is bit n - 1 (stall = bit 0 ...
 * timeout = bit 4). Several can be set at once. */
#define MOTION_FAULT_BITS_WIDTH (5U)

/** Return the bit for a valid fault code, evaluating the argument once. */
/** @brief Map a valid fault code to its one-bit status representation.
 * @param fault Code from MOTION_FAULT_NONE through MOTION_FAULT_TIMEOUT.
 * @return Zero for NONE; otherwise bit fault-minus-one, within the low five
 * bits.
 * @pre The caller supplies a valid code; out-of-domain shift counts are not
 * validated. The inline function evaluates its argument once and changes no
 * state.
 */
static inline uint8_t motion_pack_fault_bit (motion_fault_t fault)
{
    /* NONE is zero in the unsigned comparison domain. Codes through TIMEOUT
     * produce zero or one of the five fault bits, all within uint8_t range. */
    return (uint8_t)(((unsigned int)MOTION_FAULT_NONE == fault)
                         ? 0U
                         : (1U << ((uint32_t)fault - 1U)));
}

/* Return the lowest set fault or MOTION_FAULT_NONE. */
/** @brief Select the lowest numbered fault represented by a mask.
 * @param fault_bits Mask whose low five bits represent the defined faults.
 * @return The code for the lowest set low-five bit, or MOTION_FAULT_NONE.
 * @note Ignores the upper three bits; does not modify the mask or shared
 * state.
 */
motion_fault_t motion_pack_fault_first (uint8_t fault_bits);

/* Store the command type in bits 0 through 2. */
typedef enum
{
    MOTION_CMD_VELOCITY = 0,
    MOTION_CMD_DRIVE,
    MOTION_CMD_TURN,
    MOTION_CMD_MANOEUVRE,
    MOTION_CMD_STOP,
    MOTION_CMD_CANCEL,
    MOTION_CMD_RESET
} motion_pack_cmd_type_t;

/* These fields hold a decoded command. */
typedef struct
{
    motion_pack_cmd_type_t type;
    float32_t              left_mps;   /* VELOCITY                          */
    float32_t              right_mps;  /* VELOCITY                          */
    float32_t              meters;     /* Negative distance drives backward. */
    float32_t              degrees;    /* Positive angle turns left. */
    float32_t              speed_mps;  /* DRIVE, TURN                       */
    motion_manoeuvre_t     manoeuvre;  /* MANOEUVRE                         */
    uint8_t                command_id; /* CANCEL                            */
} motion_pack_cmd_t;

/* Encode the status in 64 bits. */
/** @brief Quantize a status snapshot into the 64-bit status layout.
 * @param p_status Readable caller-owned snapshot, or null to return zero.
 * Its fault code must be NONE through TIMEOUT for motion_pack_fault_bit().
 * @return Packed word; null returns zero. State and fault-mask bits are masked
 * to their field widths, and the individual fault code is ORed into the mask.
 * @note Wheel speeds use 0.002 m/s steps and 9 magnitude bits; distance uses
 * 0.001 m steps and 15 magnitude bits. Magnitudes round by adding half a step
 * then truncating, saturate on overflow/infinity, and encode NaN as zero.
 * Zero magnitude has no direction bit. Heading NaN becomes zero; other values
 * clamp to +/-1000000 degrees, round halfway away from zero, then wrap to
 * 0..359.
 * No lock or snapshot acquisition occurs; the caller owns all source access.
 */
uint64_t motion_pack_status (const motion_status_t * p_status);
/** @brief Decode every field of a raw 64-bit status word.
 * @param word Raw word; all bit patterns are accepted without semantic checks.
 * @param p_status Writable output, or null for no action.
 * @post Writes every status member when non-null, including the lowest fault
 * code derived from the five fault bits. Speeds are m/s and distance metres.
 * @note Raw state values 6/7 and heading values 360..511 remain representable;
 * they are not rejected or normalized. Raw sign-magnitude zero can become
 * negative floating zero. The caller serializes output access; no lock is
 * taken.
 */
void motion_pack_read_status (uint64_t word, motion_status_t * p_status);

/* Encode the outcome report in 16 bits. */
/** @brief Encode a completion report in the low 14 bits of a word.
 * @param p_report Readable caller-owned report, or null to return zero.
 * @return Command ID, two-bit outcome, three-bit fault and lost flag; bits
 * 14..15 are zero. Enum values are masked, not semantically validated.
 * @note No queue access, report consumption or synchronization occurs.
 */
uint16_t motion_pack_report (const motion_report_t * p_report);
/** @brief Decode the report fields without consuming a report queue.
 * @param word Raw 16-bit word; the upper two bits are ignored.
 * @param p_report Writable caller-owned output, or null for no action.
 * @post Writes all four report members when non-null. Raw fault values 6/7
 * are retained rather than rejected; no semantic validation or lock occurs.
 */
void motion_pack_read_report (uint16_t word, motion_report_t * p_report);

/* The builders return type 7 for invalid input. */
#define MOTION_CMD_INVALID_WORD (7U)

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
/** @brief Build a quantized two-wheel velocity command.
 * @param left_mps Signed left wheel speed in metres per second.
 * @param right_mps Signed right wheel speed in metres per second.
 * @return Packed command, or MOTION_CMD_INVALID_WORD for non-finite input or
 * a magnitude whose computed abs(value)/0.002 + 0.5 is not below 512.
 * @note Decoded speeds span -1.022..+1.022 m/s in 0.002 m/s steps. Acceptance
 * uses the floating rounding expression, not a separate nominal-range clamp.
 * Zero magnitude is canonical positive zero. This only builds a word; it does
 * not submit a request, change motion state or reserve a command ID.
 */
uint32_t motion_pack_velocity (float32_t left_mps, float32_t right_mps);
/** @brief Build a signed-distance drive command with positive speed.
 * @param meters Signed distance in metres; negative requests backward travel.
 * @param speed_mps Wheel-speed magnitude in metres per second.
 * @return Packed command, or MOTION_CMD_INVALID_WORD unless both inputs are
 * finite, distance rounds within 13 magnitude bits at 0.001 m per step, and
 * speed rounds to 1..511 steps of 0.002 m/s without overflowing that field.
 * @note Zero distance is representable. Positive sub-step speed that rounds to
 * one step is accepted; zero, negative and rounded-zero speeds are rejected.
 * The word encodes a magnitude up to 8.191 m; it does not start the drive or
 * validate the motion controller's current state, calibration or deadline.
 */
uint32_t motion_pack_drive (float32_t meters, float32_t speed_mps);
/** @brief Build a signed-angle turn command with positive wheel speed.
 * @param degrees Angle in degrees; positive turns left, negative turns right.
 * @param speed_mps Wheel-speed magnitude in metres per second.
 * @return Packed command, or MOTION_CMD_INVALID_WORD unless both inputs are
 * finite, angle rounds within 9 magnitude bits at one degree per step, and
 * speed rounds to 1..511 steps of 0.002 m/s without overflowing that field.
 * @note Zero angle and positive sub-step speed rounding to one step are
 * representable. The encoded angle magnitude is at most 511 degrees; it is not
 * wrapped like status heading. This builds a word without submitting motion.
 */
uint32_t motion_pack_turn (float32_t degrees, float32_t speed_mps);
/** @brief Build a command for one of the four named manoeuvres.
 * @param manoeuvre MANOEUVRE_LEFT, RIGHT, STRAIGHT or U_TURN.
 * @return Packed command, or MOTION_CMD_INVALID_WORD for an out-of-range code.
 * @note The unsigned range check also rejects a negative code. No manoeuvre
 * is started and no motion-controller state is read.
 */
uint32_t motion_pack_manoeuvre (motion_manoeuvre_t manoeuvre);
/** @brief Build a STOP command with no payload.
 * @return The canonical STOP word with all non-type bits clear.
 * @note Building the word does not brake hardware; the receiver executes it.
 */
uint32_t motion_pack_stop (void);
/** @brief Build a cancellation word for an eight-bit command ID.
 * @param command_id ID copied verbatim, including zero.
 * @return Packed CANCEL word; no ID is rejected by the builder.
 * @note The eventual receiver decides whether the ID names an active command;
 * this function neither cancels a command nor consumes its report.
 */
uint32_t motion_cmd_cancel (uint8_t command_id);
/** @brief Build a RESET command with no payload.
 * @return The canonical RESET word with all non-type bits clear.
 * @note Building the word does not clear faults; the receiver decides
 * readiness.
 */
uint32_t motion_pack_reset (void);

/* Return false for an unknown type, bits outside the selected type's fields,
 * or a drive or turn with zero speed.
 */
/** @brief Validate the command envelope and decode its payload.
 * @param word Raw command word, including malformed input.
 * @param p_cmd Writable caller-owned output; null is rejected.
 * @return False for null output, type 7, nonzero unused bits, or a drive/turn
 * with zero encoded speed. Failure leaves all output bytes unchanged.
 * True means the representation is valid, not that motion will accept it.
 * @post On success, initializes every member, then decodes the selected
 * payload. Unused numeric members are zero, manoeuvre defaults to STRAIGHT,
 * and command_id defaults to zero. Raw sign-magnitude negative zero is
 * retained.
 * @note No command submission, locking, calibration or controller-state check
 * occurs. The caller owns output access and handles the receiver's result.
 */
bool motion_pack_decode (uint32_t word, motion_pack_cmd_t * p_cmd);

#endif /* BUDDY2_MOTION_PACK_H */

/* End of file. */

