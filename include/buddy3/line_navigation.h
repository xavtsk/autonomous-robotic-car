/*
 * File: line_navigation.h
 * Owner: Buddy 3 - IR Line Following and Barcode Decoding
 * Purpose: Public line navigation subsystem interface.
 */

#ifndef BUDDY3_LINE_NAVIGATION_H
#define BUDDY3_LINE_NAVIGATION_H

/* TODO: Agree on line and barcode observations, navigation requests, and lifecycle error reporting. */

/*
 * Proposed lifecycle API; declarations only, not implemented.
 * TODO: Define initialization prerequisites, failure reporting, process timing,
 *       and micro T-Kernel task, queue, and synchronization contracts.
 * Integration must not call these functions until definitions are provided.
 */
void line_navigation_init(void);
void line_navigation_process(void);

#endif /* BUDDY3_LINE_NAVIGATION_H */
