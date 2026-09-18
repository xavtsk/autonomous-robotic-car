/*
 * mission_controller.c - deterministic, hardware-independent mission logic.
 */

#include "controller/mission_controller.h"

#include <stddef.h>

/* Mission state names live in robot_types.c, beside the enum, so telemetry
   and this module cannot report different names for the same state. */

static const char * const g_mission_event_names[] =
{
    "START", "BARCODE_DECODED", "JUNCTION_DETECTED", "TURN_COMPLETED",
    "HUMP_DETECTED", "HUMP_COMPLETED", "OBSTACLE_DETECTED", "MOTION_STOPPED",
    "OBSTACLE_PROFILE_READY", "BYPASS_COMPLETED", "LINE_LOST",
    "LINE_REACQUIRED", "STOP_REQUESTED", "RESUME_REQUESTED", "IMPACT_DETECTED",
    "TIMEOUT"
};

static const char * const g_mission_motion_names[] =
{
    "NONE", "START_LINE_FOLLOW", "STOP", "SET_SLOW_SPEED",
    "RESTORE_NORMAL_SPEED", "EXECUTE_TURN", "EXECUTE_BYPASS",
    "START_LINE_SEARCH"
};

#define MISSION_COMPILE_ASSERT(condition, name) \
    typedef char mission_assert_##name[(condition) ? 1 : -1]

MISSION_COMPILE_ASSERT(
    (sizeof g_mission_event_names / sizeof g_mission_event_names[0]) ==
        (size_t)MISSION_EVENT_COUNT,
    event_names_match_enum);

MISSION_COMPILE_ASSERT(
    (sizeof g_mission_motion_names / sizeof g_mission_motion_names[0]) ==
        (size_t)MISSION_MOTION_COUNT,
    motion_names_match_enum);

static void decision_reset(const mission_controller_t * controller,
                           mission_decision_t * decision)
{
    decision->previous_state      = controller->state;
    decision->current_state       = controller->state;
    decision->motion_command      = MISSION_MOTION_NONE;
    decision->nav_action          = NAV_NONE;
    decision->avoid_plan          = AVOID_NONE;
    decision->start_obstacle_scan = false;
    decision->state_changed       = false;
}

static void enter_state(mission_controller_t * controller,
                        mission_decision_t * decision,
                        mission_state_t next_state)
{
    controller->state       = next_state;
    decision->current_state = next_state;
    decision->state_changed = (decision->previous_state != next_state);
}

static bool state_can_detect_barcode(mission_state_t state)
{
    return ((MISSION_STATE_LINE_FOLLOW == state) ||
            (MISSION_STATE_HUMP == state));
}

static bool state_can_detect_obstacle(mission_state_t state)
{
    return ((MISSION_STATE_LINE_FOLLOW == state) ||
            (MISSION_STATE_AT_JUNCTION == state) ||
            (MISSION_STATE_HUMP == state) ||
            (MISSION_STATE_BYPASS == state) ||
            (MISSION_STATE_LINE_SEARCH == state));
}

static bool state_can_lose_line(mission_state_t state)
{
    return ((MISSION_STATE_LINE_FOLLOW == state) ||
            (MISSION_STATE_AT_JUNCTION == state) ||
            (MISSION_STATE_HUMP == state));
}

static bool state_has_timeout(mission_state_t state)
{
    return ((MISSION_STATE_AT_JUNCTION == state) ||
            (MISSION_STATE_OBSTACLE_STOPPING == state) ||
            (MISSION_STATE_OBSTACLE_SCAN == state) ||
            (MISSION_STATE_BYPASS == state) ||
            (MISSION_STATE_LINE_SEARCH == state));
}

static mission_result_t handle_resume(mission_controller_t * controller,
                                      mission_decision_t * decision)
{
    mission_state_t resume_state;

    if (MISSION_STATE_STOPPED != controller->state)
    {
        return MISSION_RESULT_IGNORED;
    }

    resume_state = controller->state_before_stop;

    if ((MISSION_STATE_LINE_FOLLOW == resume_state) ||
        (MISSION_STATE_HUMP == resume_state))
    {
        enter_state(controller, decision, MISSION_STATE_LINE_FOLLOW);
        decision->motion_command = MISSION_MOTION_START_LINE_FOLLOW;
        return MISSION_RESULT_ACCEPTED;
    }

    if ((MISSION_STATE_AT_JUNCTION == resume_state) ||
        (MISSION_STATE_OBSTACLE_STOPPING == resume_state) ||
        (MISSION_STATE_OBSTACLE_SCAN == resume_state) ||
        (MISSION_STATE_BYPASS == resume_state) ||
        (MISSION_STATE_LINE_SEARCH == resume_state))
    {
        /* Never blindly restart a partially completed turn or bypass. */
        enter_state(controller, decision, MISSION_STATE_LINE_SEARCH);
        decision->motion_command = MISSION_MOTION_START_LINE_SEARCH;
        return MISSION_RESULT_ACCEPTED;
    }

    return MISSION_RESULT_IGNORED;
}

void mission_controller_init(mission_controller_t * controller)
{
    if (NULL != controller)
    {
        controller->state                    = MISSION_STATE_BOOT;
        controller->state_before_stop        = MISSION_STATE_BOOT;
        controller->state_before_obstacle    = MISSION_STATE_BOOT;
        controller->pending_nav_action       = NAV_NONE;
        controller->pending_barcode          = '?';
        controller->pending_barcode_reversed = false;
        controller->has_pending_nav_action   = false;
    }
}

mission_result_t mission_controller_handle(
    mission_controller_t * controller,
    const mission_event_t * event,
    mission_decision_t * decision)
{
    if ((NULL == controller) || (NULL == event) || (NULL == decision))
    {
        return MISSION_RESULT_INVALID_ARGUMENT;
    }

    if (((unsigned)controller->state >= (unsigned)MISSION_STATE_COUNT) ||
        ((unsigned)event->kind >= (unsigned)MISSION_EVENT_COUNT))
    {
        return MISSION_RESULT_INVALID_ARGUMENT;
    }

    decision_reset(controller, decision);

    /* Safety events pre-empt the state-specific behaviour below. */
    if (MISSION_EVENT_IMPACT_DETECTED == event->kind)
    {
        enter_state(controller, decision, MISSION_STATE_FAULT);
        decision->motion_command = MISSION_MOTION_STOP;
        return MISSION_RESULT_ACCEPTED;
    }

    if (MISSION_EVENT_STOP_REQUESTED == event->kind)
    {
        if (MISSION_STATE_FAULT == controller->state)
        {
            /* A stop command reinforces a fault; it must not make it resumable. */
            decision->motion_command = MISSION_MOTION_STOP;
            return MISSION_RESULT_ACCEPTED;
        }

        if (MISSION_STATE_STOPPED != controller->state)
        {
            controller->state_before_stop = controller->state;
        }
        enter_state(controller, decision, MISSION_STATE_STOPPED);
        decision->motion_command = MISSION_MOTION_STOP;
        return MISSION_RESULT_ACCEPTED;
    }

    if (MISSION_EVENT_RESUME_REQUESTED == event->kind)
    {
        return handle_resume(controller, decision);
    }

    if (MISSION_EVENT_TIMEOUT == event->kind)
    {
        if ((unsigned)event->timeout_state >= (unsigned)MISSION_STATE_COUNT)
        {
            return MISSION_RESULT_INVALID_ARGUMENT;
        }

        if ((event->timeout_state == controller->state) &&
            state_has_timeout(controller->state))
        {
            enter_state(controller, decision, MISSION_STATE_FAULT);
            decision->motion_command = MISSION_MOTION_STOP;
            return MISSION_RESULT_ACCEPTED;
        }
        /* A previous operation's timer may expire after its state has ended. */
        return MISSION_RESULT_IGNORED;
    }

    /* Barcode decoding records a future action without interrupting motion. */
    if (MISSION_EVENT_BARCODE_DECODED == event->kind)
    {
        if ((unsigned)event->nav_action >= (unsigned)NAV_COUNT)
        {
            return MISSION_RESULT_INVALID_ARGUMENT;
        }

        if ((!state_can_detect_barcode(controller->state)) ||
            (event->barcode < 'A') || (event->barcode > 'Z') ||
            (NAV_NONE == event->nav_action))
        {
            return MISSION_RESULT_IGNORED;
        }

        controller->pending_nav_action       = event->nav_action;
        controller->pending_barcode          = event->barcode;
        controller->pending_barcode_reversed = event->was_reversed;
        controller->has_pending_nav_action   = true;
        return MISSION_RESULT_ACCEPTED;
    }

    if (MISSION_EVENT_OBSTACLE_DETECTED == event->kind)
    {
        if (!state_can_detect_obstacle(controller->state))
        {
            return MISSION_RESULT_IGNORED;
        }

        controller->state_before_obstacle = controller->state;
        enter_state(controller, decision, MISSION_STATE_OBSTACLE_STOPPING);
        decision->motion_command = MISSION_MOTION_STOP;
        return MISSION_RESULT_ACCEPTED;
    }

    if (MISSION_EVENT_LINE_LOST == event->kind)
    {
        if (!state_can_lose_line(controller->state))
        {
            return MISSION_RESULT_IGNORED;
        }

        enter_state(controller, decision, MISSION_STATE_LINE_SEARCH);
        decision->motion_command = MISSION_MOTION_START_LINE_SEARCH;
        return MISSION_RESULT_ACCEPTED;
    }

    switch (controller->state)
    {
        case MISSION_STATE_BOOT:
            if (MISSION_EVENT_START == event->kind)
            {
                enter_state(controller, decision, MISSION_STATE_LINE_FOLLOW);
                decision->motion_command = MISSION_MOTION_START_LINE_FOLLOW;
                return MISSION_RESULT_ACCEPTED;
            }
            break;

        case MISSION_STATE_LINE_FOLLOW:
            if (MISSION_EVENT_JUNCTION_DETECTED == event->kind)
            {
                if (!controller->has_pending_nav_action)
                {
                    enter_state(controller, decision, MISSION_STATE_FAULT);
                    decision->motion_command = MISSION_MOTION_STOP;
                    return MISSION_RESULT_ACCEPTED;
                }

                enter_state(controller, decision, MISSION_STATE_AT_JUNCTION);
                decision->motion_command = MISSION_MOTION_EXECUTE_TURN;
                decision->nav_action     = controller->pending_nav_action;

                controller->pending_nav_action     = NAV_NONE;
                controller->has_pending_nav_action = false;
                return MISSION_RESULT_ACCEPTED;
            }

            if (MISSION_EVENT_HUMP_DETECTED == event->kind)
            {
                enter_state(controller, decision, MISSION_STATE_HUMP);
                decision->motion_command = MISSION_MOTION_SET_SLOW_SPEED;
                return MISSION_RESULT_ACCEPTED;
            }
            break;

        case MISSION_STATE_AT_JUNCTION:
            if (MISSION_EVENT_TURN_COMPLETED == event->kind)
            {
                enter_state(controller, decision, MISSION_STATE_LINE_FOLLOW);
                decision->motion_command = MISSION_MOTION_START_LINE_FOLLOW;
                return MISSION_RESULT_ACCEPTED;
            }
            break;

        case MISSION_STATE_HUMP:
            if (MISSION_EVENT_HUMP_COMPLETED == event->kind)
            {
                enter_state(controller, decision, MISSION_STATE_LINE_FOLLOW);
                decision->motion_command = MISSION_MOTION_RESTORE_NORMAL_SPEED;
                return MISSION_RESULT_ACCEPTED;
            }
            break;

        case MISSION_STATE_OBSTACLE_STOPPING:
            if (MISSION_EVENT_MOTION_STOPPED == event->kind)
            {
                enter_state(controller, decision,
                            MISSION_STATE_OBSTACLE_SCAN);
                decision->start_obstacle_scan = true;
                return MISSION_RESULT_ACCEPTED;
            }
            break;

        case MISSION_STATE_OBSTACLE_SCAN:
            if (MISSION_EVENT_OBSTACLE_PROFILE_READY == event->kind)
            {
                if ((unsigned)event->avoid_plan >= (unsigned)AVOID_COUNT)
                {
                    return MISSION_RESULT_INVALID_ARGUMENT;
                }

                if ((AVOID_LEFT == event->avoid_plan) ||
                    (AVOID_RIGHT == event->avoid_plan) ||
                    (AVOID_REVERSE == event->avoid_plan))
                {
                    enter_state(controller, decision, MISSION_STATE_BYPASS);
                    decision->motion_command = MISSION_MOTION_EXECUTE_BYPASS;
                    decision->avoid_plan     = event->avoid_plan;
                    return MISSION_RESULT_ACCEPTED;
                }

                if (AVOID_CONTINUE == event->avoid_plan)
                {
                    if (MISSION_STATE_LINE_FOLLOW ==
                        controller->state_before_obstacle)
                    {
                        enter_state(controller, decision,
                                    MISSION_STATE_LINE_FOLLOW);
                        decision->motion_command =
                            MISSION_MOTION_START_LINE_FOLLOW;
                    }
                    else
                    {
                        /* An interrupted manoeuvre is not safe to replay. */
                        enter_state(controller, decision,
                                    MISSION_STATE_LINE_SEARCH);
                        decision->motion_command =
                            MISSION_MOTION_START_LINE_SEARCH;
                    }
                    return MISSION_RESULT_ACCEPTED;
                }

                if (AVOID_STOP == event->avoid_plan)
                {
                    controller->state_before_stop = MISSION_STATE_OBSTACLE_SCAN;
                    enter_state(controller, decision, MISSION_STATE_STOPPED);
                    decision->motion_command = MISSION_MOTION_STOP;
                    return MISSION_RESULT_ACCEPTED;
                }
            }
            break;

        case MISSION_STATE_BYPASS:
            if (MISSION_EVENT_BYPASS_COMPLETED == event->kind)
            {
                enter_state(controller, decision, MISSION_STATE_LINE_SEARCH);
                decision->motion_command = MISSION_MOTION_START_LINE_SEARCH;
                return MISSION_RESULT_ACCEPTED;
            }

            break;

        case MISSION_STATE_LINE_SEARCH:
            if (MISSION_EVENT_LINE_REACQUIRED == event->kind)
            {
                enter_state(controller, decision, MISSION_STATE_LINE_FOLLOW);
                decision->motion_command = MISSION_MOTION_START_LINE_FOLLOW;
                return MISSION_RESULT_ACCEPTED;
            }
            break;

        case MISSION_STATE_STOPPED:
        case MISSION_STATE_FAULT:
            break;

        case MISSION_STATE_COUNT:
        default:
            return MISSION_RESULT_INVALID_ARGUMENT;
    }

    return MISSION_RESULT_IGNORED;
}

const char * mission_event_name(mission_event_kind_t event)
{
    if ((unsigned)event >= (unsigned)MISSION_EVENT_COUNT)
    {
        return "INVALID";
    }
    return g_mission_event_names[(unsigned)event];
}

const char * mission_motion_command_name(mission_motion_command_t command)
{
    if ((unsigned)command >= (unsigned)MISSION_MOTION_COUNT)
    {
        return "INVALID";
    }
    return g_mission_motion_names[(unsigned)command];
}
