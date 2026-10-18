/*
 * File: motion_controller.h
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

#include <stdbool.h>
#include <stdint.h>
#include "motion.h"

void motion_controller_init(float max_speed_mps, float stop_lead_drive_m,
                            float stop_lead_turn_m);

/* One control period: speed control, statechart, fault checks */
void motion_controller_step(float dt_s, uint32_t now_us);

/* Brake now if no motion is active (stop or fault since the last period) */
void motion_controller_make_safe(void);

/* Requests (see motion.h). sign_left / sign_right: +1 forward, -1 backward;
 * dist_m is the distance each wheel travels. */
motion_result_t motion_controller_request_velocity(float left_mps,
                                                   float right_mps,
                                                   uint32_t now_us);
motion_result_t motion_controller_request_move(motion_state_t kind,
                                               int32_t sign_left,
                                               int32_t sign_right,
                                               float dist_m, float speed_mps,
                                               uint32_t now_us);
void            motion_controller_stop(void);
/* Motion task only, under its mutex: stale navigation owner. */
void            motion_controller_navigation_timeout(void);
bool            motion_controller_reset(void);
motion_state_t  motion_controller_poll(void);
motion_state_t  motion_controller_state(void);
motion_fault_t  motion_controller_fault(void);
float           motion_controller_speed_cap(void);

/* While busy (calibration), requests are rejected and the step does nothing */
void            motion_controller_set_busy(bool b_busy);
bool            motion_controller_is_busy(void);

#endif /* BUDDY2_MOTION_CONTROLLER_H */
