/** @file wheel_encoder.h
 * @brief Wheel encoder acquisition (quadrature, decoded in PIO0).
 *
 * @par Ownership
 * Buddy 2 - Motion Control
 *
 * Counts are in quadrature edges (4 per encoder line). A positive count means
 * the wheel turned in the direction that drives the robot forward.
 *
 * Counts are WHEEL_COUNT_BITS-bit two's complement values (DATA_FORMAT.md):
 * +-524 287 counts = +-42.5 m per wheel, more than the 30 m longest run.
 * Store them as WHEEL_COUNT_BITS-bit fields (wheel_encoder_bits()) and
 * subtract with wheel_encoder_delta(), which is correct across the wrap.
 *
 * One reader: only the motion task reads the PIO, with
 * wheel_encoder_sample() once per control period (and during calibration).
 * Every other call returns that snapshot: one 32-bit load per wheel, safe
 * from any task, never touching the PIO FIFO.
 */

#ifndef BUDDY2_WHEEL_ENCODER_H
#define BUDDY2_WHEEL_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/* Buddy 2 modules share these wheel identifiers. */
typedef enum
{
    WHEEL_LEFT = 0, /* Pair motor M1 with encoder ENC_L. */
    WHEEL_RIGHT,    /* Pair motor M2 with encoder ENC_R. */
    WHEEL_COUNT
} wheel_encoder_id_t;

#define WHEEL_COUNT_BITS (20U)
#define WHEEL_COUNT_MASK ((1UL << WHEEL_COUNT_BITS) - 1U)

/* Store a count such as the start of a move. */
typedef struct
{
    uint32_t bits : WHEEL_COUNT_BITS;
} wheel_encoder_count_t;

/*
 * Configure ENC_L / ENC_R as inputs with pull-ups and start one PIO0 state
 * machine per encoder. No interrupt is used. Returns E_OK (0), or E_OBJ if
 * PIO0 is already in use; counts then stay at 0.
 */
int32_t wheel_encoder_init (bool b_is_left_inverted, bool b_is_right_inverted);

/* Only the motion task refreshes the snapshot. */
void wheel_encoder_sample (void);

/* Any task can read the last sampled value. */
int32_t
wheel_encoder_count (wheel_encoder_id_t wheel); /* Positive means forward. */
uint32_t wheel_encoder_bits (
    wheel_encoder_id_t wheel); /* Read the same count as a raw 20-bit field. */

/* Subtract two counts modulo the 20-bit counter period. */
int32_t wheel_encoder_delta (uint32_t now_bits, uint32_t then_bits);

bool wheel_encoder_is_running (void); /* Report whether PIO is counting. */

/* Return zero because the PIO program ignores invalid transitions. */
uint32_t wheel_encoder_glitches (wheel_encoder_id_t wheel);

/* Calibration configures count direction. */
void wheel_encoder_set_inverted (wheel_encoder_id_t wheel, bool b_is_inverted);
bool wheel_encoder_is_inverted (wheel_encoder_id_t wheel);

#endif /* BUDDY2_WHEEL_ENCODER_H */

/* End of file. */

