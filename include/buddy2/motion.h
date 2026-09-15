/*
 * File: motion.h
 * Owner: Buddy 2 - Motion Control
 * Purpose: Public motion subsystem interface.
 */

#ifndef BUDDY2_MOTION_H
#define BUDDY2_MOTION_H

/* TODO: Agree on movement requests, motion feedback, units, and lifecycle error reporting. */

/*
 * Proposed lifecycle API; declarations only, not implemented.
 * TODO: Define initialization prerequisites, failure reporting, process timing,
 *       and micro T-Kernel task, queue, and synchronization contracts.
 * Integration must not call these functions until definitions are provided.
 */
void motion_init(void);
void motion_process(void);

#endif /* BUDDY2_MOTION_H */
