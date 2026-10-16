/*
 * File: wheel_encoder.h
 * Owner: Buddy 2 - Motion Control
 * Purpose: Wheel encoder acquisition (quadrature, interrupt driven).
 *
 * Counts are in quadrature edges (4 per encoder line). A positive count means
 * the wheel turned in the direction that drives the robot forward.
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

/*
 * Configure ENC_L / ENC_R as inputs with pull-ups and enable their edge
 * interrupts. Call it from the motion task: the interrupt is serviced by the
 * core that calls this function. Returns E_OK (0) or a negative kernel error.
 */
int32_t  wheel_encoder_init(bool b_left_inverted, bool b_right_inverted);

int32_t  wheel_encoder_count(wheel_id_t wheel);       /* + = forward        */
uint32_t wheel_encoder_glitches(wheel_id_t wheel);    /* invalid transitions */

/* Count direction; used by calibration */
void     wheel_encoder_set_inverted(wheel_id_t wheel, bool b_inverted);
bool     wheel_encoder_is_inverted(wheel_id_t wheel);

#endif /* BUDDY2_WHEEL_ENCODER_H */
