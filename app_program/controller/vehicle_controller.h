/*
 * Team facade over the single s3_integration/s3_drive behaviour owner.
 * This adapter mirrors telemetry and routes UI requests; it owns no task,
 * motion calls or completion-report consumer.
 */
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "../app.h"
#include "../subsystem3/line_navigation.h"
#include "s3_integration.h"

typedef enum {
	VC_WAITING = 0,
	VC_LINE_FOLLOW,
	VC_TURNING,
	VC_STOPPED,
	VC_FAULT
} VEHICLE_CONTROLLER_STATE;

/* Reset adapter state. start() is a compatibility no-op, not a second task. */
ER vehicle_controller_init(void);
ER vehicle_controller_start(void);

/* Legacy entry point: returns E_NOSPT; no second sensor producer is accepted. */
ER vehicle_controller_submit_navigation(const s3_navigation_output_t *output);

/* Queue START/STOP to the sampling owner. E_OK means queued, not executed.
 * Read status for readiness/calibration gates and execution outcome. No
 * physical start source is installed by this facade. */
ER vehicle_controller_begin(void);
ER vehicle_controller_request_stop(void);

VEHICLE_CONTROLLER_STATE vehicle_controller_state(void);
/* Sole sampling-owner adapters, not additional tasks or motion callers. */
bool vehicle_controller_prepare_start(s3_navigation_t *navigation);
/* START was rejected after prepare_start(): undo the map freeze only if that
 * prepare_start() newly took it (no run ever used the map). A freeze from an
 * earlier accepted run is kept until reboot. */
void vehicle_controller_cancel_start(void);
/* START was accepted: the freeze now belongs to the run. */
void vehicle_controller_commit_start(void);
bool vehicle_controller_apply_mapping(const s3_nav_action_t map[26]);
void vehicle_controller_observe(const s3_integration_status_t *status);

#endif /* VEHICLE_CONTROLLER_H */
