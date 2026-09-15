/*
 * File: communication.h
 * Owner: Buddy 1 - WiFi Communication, Command and Telemetry
 * Purpose: Public communication subsystem interface.
 */

#ifndef BUDDY1_COMMUNICATION_H
#define BUDDY1_COMMUNICATION_H

/* TODO: Agree on command input, telemetry output, and lifecycle error reporting. */

/*
 * Proposed lifecycle API; declarations only, not implemented.
 * TODO: Define initialization prerequisites, failure reporting, process timing,
 *       and micro T-Kernel task, queue, and synchronization contracts.
 * Integration must not call these functions until definitions are provided.
 */
void communication_init(void);
void communication_process(void);

#endif /* BUDDY1_COMMUNICATION_H */
