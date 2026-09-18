/*
 * mission_controller.h - pure mission-level decision logic.
 *
 * This module owns WHAT the robot should do next. The buddy subsystems own
 * HOW sensing and actuation are performed. There are no Pico SDK, MQTT, RTOS
 * or hardware calls here, so every transition can be host-tested.
 */
#ifndef CONTROLLER_MISSION_CONTROLLER_H
#define CONTROLLER_MISSION_CONTROLLER_H

#include <stdbool.h>

#include "shared/robot_types.h"

typedef enum
{
    MISSION_EVENT_START = 0,
    MISSION_EVENT_BARCODE_DECODED,
    MISSION_EVENT_JUNCTION_DETECTED,
    MISSION_EVENT_TURN_COMPLETED,
    MISSION_EVENT_HUMP_DETECTED,
    MISSION_EVENT_HUMP_COMPLETED,
    MISSION_EVENT_OBSTACLE_DETECTED,
    MISSION_EVENT_MOTION_STOPPED,
    MISSION_EVENT_OBSTACLE_PROFILE_READY,
    MISSION_EVENT_BYPASS_COMPLETED,
    MISSION_EVENT_LINE_LOST,
    MISSION_EVENT_LINE_REACQUIRED,
    MISSION_EVENT_STOP_REQUESTED,
    MISSION_EVENT_RESUME_REQUESTED,
    MISSION_EVENT_IMPACT_DETECTED,
    MISSION_EVENT_TIMEOUT,
    MISSION_EVENT_COUNT
} mission_event_kind_t;

/*
 * One event from a buddy, shared lifecycle adapter or safety input.
 *
 * Only BARCODE_DECODED uses barcode/nav_action/was_reversed. Only
 * OBSTACLE_PROFILE_READY uses avoid_plan. Only TIMEOUT uses timeout_state,
 * which identifies the operation that armed the timer so a late timeout from
 * an old operation cannot fault the current one. The fixed structure avoids
 * malloc and keeps the RTOS message size deterministic.
 */
typedef struct
{
    mission_event_kind_t kind;
    char                 barcode;
    nav_action_t         nav_action;
    bool                 was_reversed;
    avoid_plan_t         avoid_plan;
    mission_state_t      timeout_state;
} mission_event_t;

/* A command for Buddy 2. Parameters travel separately in the decision. */
typedef enum
{
    MISSION_MOTION_NONE = 0,
    MISSION_MOTION_START_LINE_FOLLOW,
    MISSION_MOTION_STOP,
    MISSION_MOTION_SET_SLOW_SPEED,
    MISSION_MOTION_RESTORE_NORMAL_SPEED,
    MISSION_MOTION_EXECUTE_TURN,
    MISSION_MOTION_EXECUTE_BYPASS,
    MISSION_MOTION_START_LINE_SEARCH,
    MISSION_MOTION_COUNT
} mission_motion_command_t;

typedef enum
{
    MISSION_RESULT_ACCEPTED = 0,
    MISSION_RESULT_IGNORED,
    MISSION_RESULT_INVALID_ARGUMENT
} mission_result_t;

typedef struct
{
    mission_state_t          previous_state;
    mission_state_t          current_state;
    mission_motion_command_t motion_command;
    nav_action_t             nav_action;
    avoid_plan_t             avoid_plan;
    bool                     start_obstacle_scan;
    bool                     state_changed;
} mission_decision_t;

/*
 * Controller context. Allocate one statically and let only the future mission
 * task mutate it. Other tasks send events rather than writing these fields.
 */
typedef struct
{
    mission_state_t state;
    mission_state_t state_before_stop;
    mission_state_t state_before_obstacle;
    nav_action_t    pending_nav_action;
    char            pending_barcode;
    bool            pending_barcode_reversed;
    bool            has_pending_nav_action;
} mission_controller_t;

/* Reset the controller to BOOT with no pending barcode action. */
void mission_controller_init(mission_controller_t * controller);

/*
 * Process exactly one event and describe the resulting decision.
 *
 * ACCEPTED means the event was valid in the current state. IGNORED means the
 * event was known but out of order or unsafe to apply. INVALID_ARGUMENT means
 * a pointer or relevant enum value was invalid; decision must not be inspected
 * in that case. On IGNORED, controller state is left unchanged and the
 * decision requests no action.
 */
mission_result_t mission_controller_handle(
    mission_controller_t * controller,
    const mission_event_t * event,
    mission_decision_t * decision);

/* Names for tests, telemetry adapters and serial debugging.
   mission_state_name() is declared in robot_types.h with the enum itself. */
const char * mission_event_name(mission_event_kind_t event);
const char * mission_motion_command_name(mission_motion_command_t command);

#endif /* CONTROLLER_MISSION_CONTROLLER_H */
