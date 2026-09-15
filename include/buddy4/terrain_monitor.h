/*
 * File: terrain_monitor.h
 * Owner: Buddy 4 - IMU Motion and Terrain Monitoring
 * Purpose: Public terrain monitoring subsystem interface.
 */

#ifndef BUDDY4_TERRAIN_MONITOR_H
#define BUDDY4_TERRAIN_MONITOR_H

/* TODO: Agree on terrain measurements, motion events, and lifecycle error reporting. */

/*
 * Proposed lifecycle API; declarations only, not implemented.
 * TODO: Define initialization prerequisites, failure reporting, process timing,
 *       and micro T-Kernel task, queue, and synchronization contracts.
 * Integration must not call these functions until definitions are provided.
 */
void terrain_monitor_init(void);
void terrain_monitor_process(void);

#endif /* BUDDY4_TERRAIN_MONITOR_H */
