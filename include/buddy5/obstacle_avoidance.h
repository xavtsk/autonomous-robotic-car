/*
 * File: obstacle_avoidance.h
 * Owner: Buddy 5 - Ultrasonic Obstacle Scanning and Avoidance
 * Purpose: Public obstacle avoidance subsystem interface.
 */

#ifndef BUDDY5_OBSTACLE_AVOIDANCE_H
#define BUDDY5_OBSTACLE_AVOIDANCE_H

/* TODO: Agree on obstacle profiles, avoidance and recovery requests, and lifecycle error reporting. */

/*
 * Proposed lifecycle API; declarations only, not implemented.
 * TODO: Define initialization prerequisites, failure reporting, process timing,
 *       and micro T-Kernel task, queue, and synchronization contracts.
 * Integration must not call these functions until definitions are provided.
 */
void obstacle_avoidance_init(void);
void obstacle_avoidance_process(void);

#endif /* BUDDY5_OBSTACLE_AVOIDANCE_H */
