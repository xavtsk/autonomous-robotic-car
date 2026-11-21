/** @file ctrl_motion.h
 * @brief Motion Control statechart, straight-line correction and encoder-based
 * turning.
 *
 * @par Ownership
 * Buddy 2 - Motion Control
 *
 * Not thread-safe on its own: motion.c calls every function below with the
 * Buddy 2 mutex held. The motion owner writes motor outputs during startup,
 * periodic control, navigation timeout handling and pending-brake service.
 */

#ifndef BUDDY2_MOTION_CONTROLLER_H
#define BUDDY2_MOTION_CONTROLLER_H

#include "../app_types.h"
#include <stdbool.h>
#include <stdint.h>
#include "motion.h"

/* Called for every done / cancelled / fault report, with the Buddy 2 mutex
 * held. It must not block. */
typedef void ( * ctrl_motion_report_sink_t)(const motion_report_t * p_report);

void ctrl_motion_init (float32_t max_speed_mps, float32_t stop_lead_drive_m,
                       float32_t                 stop_lead_turn_m,
                       ctrl_motion_report_sink_t p_sink);

/* Advance speed, state and faults by one period. */
void ctrl_motion_step (float32_t dt_s, uint32_t now_us);

/* Brake when the active motion has ended. */
void ctrl_motion_make_safe (void);

/* Requests (see motion.h). sign_left / sign_right: +1 forward, -1 backward;
 * dist_m is the distance each wheel travels. */
motion_result_t ctrl_motion_set_velocity (float32_t left_mps,
                                          float32_t right_mps, uint32_t now_us);
motion_result_t ctrl_motion_request_move (motion_state_t kind,
                                          int32_t sign_left, int32_t sign_right,
                                          float32_t dist_m, float32_t speed_mps,
                                          uint32_t now_us);
void            ctrl_motion_stop (void);
/* Motion task only, under its mutex: stale navigation owner. */
void            ctrl_motion_nav_timeout (void);
motion_result_t ctrl_motion_cancel (uint8_t command_id);
bool            ctrl_motion_reset (void);
motion_state_t  ctrl_motion_poll (void);
motion_state_t  ctrl_motion_state (void);
motion_fault_t  ctrl_motion_fault (void); /* Return the lowest fault bit. */
uint8_t         ctrl_motion_fault_bits (void); /* motion_pack_fault_bit() */
uint8_t         ctrl_motion_command_id (void);
void            ctrl_motion_ack_complete (uint8_t command_id);
bool            ctrl_motion_is_stopped (void);
float32_t       ctrl_motion_speed_cap (void);
/* Read-only; caller holds the Buddy 2 mutex. wheel: 0 left, 1 right. */
float32_t ctrl_motion_setpoint (unsigned int wheel);

/* While reserved, a step still tracks glitches and stillness. */
void ctrl_motion_set_busy (bool b_is_busy);
bool ctrl_motion_is_busy (void);

#endif /* BUDDY2_MOTION_CONTROLLER_H */

/* End of file. */

