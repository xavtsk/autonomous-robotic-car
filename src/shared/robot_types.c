/*
 * robot_types.c - the one implementation of the shared vocabulary's names.
 *
 * Every subsystem that needs to print a mission state, a navigation action,
 * an IMU event or an avoidance plan calls these. There is deliberately no
 * second copy anywhere: telemetry and the mission controller must never be
 * able to disagree about what a state is called.
 *
 * No hardware, no RTOS, no MQTT - so this links into the host tests, the
 * mission tests and the Pico firmware alike.
 */

#include "shared/robot_types.h"

#include <stddef.h>

static const char * const g_state_names[] =
{
    "BOOT", "LINE_FOLLOW", "AT_JUNCTION", "HUMP", "OBSTACLE_STOPPING",
    "OBSTACLE_SCAN", "BYPASS", "LINE_SEARCH", "STOPPED", "FAULT"
};

static const char * const g_action_names[] =
{
    "NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN"
};

static const char * const g_imu_event_names[] =
{
    "STATIONARY", "ACCELERATING", "TURNING",
    "CLIMBING", "DESCENDING", "IMPACT"
};

static const char * const g_avoid_plan_names[] =
{
    "NONE", "STOP", "CONTINUE", "LEFT", "RIGHT", "REVERSE"
};

/*
 * A table that has fallen out of step with its enum is a silent defect: the
 * wrong name gets printed forever and nobody notices. C11 has _Static_assert,
 * but Barr-C rule 1.1a requires C99, so use the portable idiom - an array
 * whose size goes negative when the condition fails, which no C compiler will
 * accept.
 */
#define ROBOT_COMPILE_ASSERT(condition, name) \
    typedef char robot_assert_##name[(condition) ? 1 : -1]

ROBOT_COMPILE_ASSERT(
    (sizeof g_state_names / sizeof g_state_names[0]) == (size_t)MISSION_STATE_COUNT,
    state_names_match_enum);
ROBOT_COMPILE_ASSERT(
    (sizeof g_action_names / sizeof g_action_names[0]) == (size_t)NAV_COUNT,
    action_names_match_enum);
ROBOT_COMPILE_ASSERT(
    (sizeof g_imu_event_names / sizeof g_imu_event_names[0]) == (size_t)IMU_EVENT_COUNT,
    imu_event_names_match_enum);
ROBOT_COMPILE_ASSERT(
    (sizeof g_avoid_plan_names / sizeof g_avoid_plan_names[0]) == (size_t)AVOID_COUNT,
    avoid_plan_names_match_enum);

/* Each range-checks before indexing: a corrupt value must not read past the
   end of the table. Compare unsigned against unsigned - Barr-C 5.3c. */

const char * mission_state_name(mission_state_t state)
{
    if ((unsigned)state >= (unsigned)MISSION_STATE_COUNT) { return "INVALID"; }
    return g_state_names[(unsigned)state];
}

const char * nav_action_name(nav_action_t action)
{
    if ((unsigned)action >= (unsigned)NAV_COUNT) { return "INVALID"; }
    return g_action_names[(unsigned)action];
}

const char * imu_event_name(imu_event_t event)
{
    if ((unsigned)event >= (unsigned)IMU_EVENT_COUNT) { return "INVALID"; }
    return g_imu_event_names[(unsigned)event];
}

const char * avoid_plan_name(avoid_plan_t plan)
{
    if ((unsigned)plan >= (unsigned)AVOID_COUNT) { return "INVALID"; }
    return g_avoid_plan_names[(unsigned)plan];
}
