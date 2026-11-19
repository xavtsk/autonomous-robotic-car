/** @file ctrl_motion.h
 * @brief Motion Control statechart, straight-line correction and encoder-based
 * turning.
 */
/*
 * File: ctrl_motion.h
 * Owner: Buddy 2 - Motion Control
 * Purpose: Motion Control statechart, straight-line correction and
 *          encoder-based turning.
 *
 * Not thread-safe on its own: motion.c calls every function below with the
 * Buddy 2 mutex held. Only motion_controller_step() and
 * motion_controller_make_safe() drive the motors, and only the motion task
 * calls them.
 */

#ifndef BUDDY2_MOTION_CONTROLLER_H
#define BUDDY2_MOTION_CONTROLLER_H

#include "../app_types.h"
#include <stdbool.h>
#include <stdint.h>
#include "motion.h"

/* Called for every done / cancelled / fault report, with the Buddy 2 mutex
 * held. It must not block. */
typedef void (*motion_report_sink_t)(const motion_report_t * p_report);

void motion_controller_init (float32_t max_speed_mps,
                             float32_t stop_lead_drive_m,
                             float32_t stop_lead_turn_m,
                             motion_report_sink_t p_sink);

/* One control period: speed control, statechart, fault checks */
void motion_controller_step (float32_t dt_s, uint32_t now_us);

/* Brake now if no motion is active (stop or fault since the last period) */
void motion_controller_make_safe (void);

/* Requests (see motion.h). sign_left / sign_right: +1 forward, -1 backward;
 * dist_m is the distance each wheel travels. */
motion_result_t motion_controller_set_velocity (float32_t left_mps,
                                                float32_t right_mps,
                                                uint32_t now_us);
motion_result_t
motion_controller_request_move (motion_state_t kind, int32_t sign_left,
                                int32_t sign_right, float32_t dist_m,
                                float32_t speed_mps, uint32_t now_us);
void motion_controller_stop (void);
/* Motion task only, under its mutex: stale navigation owner. */
void motion_controller_nav_timeout (void);
motion_result_t motion_controller_cancel (uint8_t command_id);
bool motion_controller_reset (void);
motion_state_t motion_controller_poll (void);
motion_state_t motion_controller_state (void);
motion_fault_t motion_controller_fault (void); /* lowest fault bit */
uint8_t motion_controller_fault_bits (void);   /* motion_fault_bit() */
uint8_t motion_controller_command_id (void);
void motion_controller_ack_complete (uint8_t command_id);
bool motion_controller_is_stopped (void);
float32_t motion_controller_speed_cap (void);
/* Read-only; caller holds the Buddy 2 mutex. wheel: 0 left, 1 right. */
float32_t motion_controller_setpoint (unsigned int wheel);

/* While busy (calibration), requests are rejected and the step does nothing */
void motion_controller_set_busy (bool b_busy);
bool motion_controller_is_busy (void);

#endif /* BUDDY2_MOTION_CONTROLLER_H */

/* End of file. */

