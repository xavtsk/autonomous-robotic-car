/*
 * robot_types.h - shared, hardware-independent vocabulary for all buddies.
 *
 * Keep data types here, not drivers or behaviour. This lets every subsystem
 * agree on event parameters without depending on MQTT, the Pico SDK or an
 * RTOS. Add a value only after the integration contract is updated.
 */
#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

/* The central controller's active behaviour; barcode decoding is an event. */
typedef enum
{
    MISSION_STATE_BOOT = 0,
    MISSION_STATE_LINE_FOLLOW,
    MISSION_STATE_AT_JUNCTION,
    MISSION_STATE_HUMP,
    MISSION_STATE_OBSTACLE_STOPPING,
    MISSION_STATE_OBSTACLE_SCAN,
    MISSION_STATE_BYPASS,
    MISSION_STATE_LINE_SEARCH,
    MISSION_STATE_STOPPED,
    MISSION_STATE_FAULT,
    MISSION_STATE_COUNT
} mission_state_t;

/* Fixed actions whose letter mapping can be replaced over MQTT at run time. */
typedef enum
{
    NAV_NONE = 0,
    NAV_LEFT,
    NAV_RIGHT,
    NAV_STRAIGHT,
    NAV_UTURN,
    NAV_COUNT
} nav_action_t;

/* Buddy 4 motion classifications named in the project write-up. */
typedef enum
{
    IMU_EVENT_STATIONARY = 0,
    IMU_EVENT_ACCELERATING,
    IMU_EVENT_TURNING,
    IMU_EVENT_CLIMBING,
    IMU_EVENT_DESCENDING,
    IMU_EVENT_IMPACT,
    IMU_EVENT_COUNT
} imu_event_t;

/* Buddy 5 plans; deliberately separate from barcode navigation actions. */
typedef enum
{
    AVOID_NONE = 0,
    AVOID_STOP,
    AVOID_CONTINUE,
    AVOID_LEFT,
    AVOID_RIGHT,
    AVOID_REVERSE,
    AVOID_COUNT
} avoid_plan_t;

/* -------------------------------------------------------------------------
 * Names for the shared vocabulary
 *
 * Declared beside the enums they describe, and implemented ONCE in
 * robot_types.c.
 *
 * They used to be implemented twice - telemetry.c and mission_controller.c
 * each kept their own table. Both were correct, and nothing kept them that
 * way: a renamed state would have been published over MQTT under one name
 * and printed to the serial log under another, with no warning. Sharing the
 * enum without sharing its names was the actual defect; a test that the two
 * copies agree only detected it.
 * ------------------------------------------------------------------------- */

const char * mission_state_name(mission_state_t state);
const char * nav_action_name(nav_action_t action);
const char * imu_event_name(imu_event_t event);
const char * avoid_plan_name(avoid_plan_t plan);

#endif /* ROBOT_TYPES_H */
