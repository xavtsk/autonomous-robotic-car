/*
 * File: wheel_encoder.h
 * Owner: Buddy 2 - Motion Control
 * Purpose: Wheel encoder acquisition (quadrature, decoded in PIO0).
 *
 * Counts are in quadrature edges (4 per encoder line). A positive count means
 * the wheel turned in the direction that drives the robot forward.
 *
 * Counts are WHEEL_COUNT_BITS-bit two's complement values (DATA_FORMAT.md):
 * +-524 287 counts = +-42.5 m per wheel, more than the 30 m longest run.
 * Store them as WHEEL_COUNT_BITS-bit fields (wheel_encoder_bits()) and
 * subtract with wheel_encoder_delta(), which is correct across the wrap.
 */

#ifndef BUDDY2_WHEEL_ENCODER_H
#define BUDDY2_WHEEL_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/* Wheel identifiers, shared by the Buddy 2 modules */
typedef enum
{
    WHEEL_LEFT = 0,     /* left DC motor (M1) + left wheel encoder (ENC_L)   */
    WHEEL_RIGHT,        /* right DC motor (M2) + right wheel encoder (ENC_R) */
    WHEEL_COUNT
} wheel_id_t;

#define WHEEL_COUNT_BITS    (20U)
#define WHEEL_COUNT_MASK    ((1UL << WHEEL_COUNT_BITS) - 1U)

/* A stored count (e.g. the start of a move) */
typedef struct
{
    uint32_t bits : WHEEL_COUNT_BITS;
} wheel_count_t;

/*
 * Configure ENC_L / ENC_R as inputs with pull-ups and start one PIO0 state
 * machine per encoder. No interrupt is used. Returns E_OK (0), or E_OBJ if
 * PIO0 is already in use; counts then stay at 0.
 */
int32_t  wheel_encoder_init(bool b_left_inverted, bool b_right_inverted);

int32_t  wheel_encoder_count(wheel_id_t wheel);       /* + = forward        */
uint32_t wheel_encoder_bits(wheel_id_t wheel);        /* same, 20-bit field */

/* now - then for two stored counts, correct across the 20-bit wrap */
int32_t  wheel_encoder_delta(uint32_t now_bits, uint32_t then_bits);

bool     wheel_encoder_is_running(void);              /* PIO counting       */

/* Invalid transitions; always 0, as the PIO program ignores them */
uint32_t wheel_encoder_glitches(wheel_id_t wheel);

/* Count direction; used by calibration */
void     wheel_encoder_set_inverted(wheel_id_t wheel, bool b_inverted);
bool     wheel_encoder_is_inverted(wheel_id_t wheel);

#endif /* BUDDY2_WHEEL_ENCODER_H */
