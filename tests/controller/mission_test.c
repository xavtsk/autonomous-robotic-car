/* Host tests for the pure mission controller. */

#include "controller/mission_controller.h"

#include <stdio.h>
#include <string.h>

static unsigned g_checks;
static unsigned g_failures;

static void check(bool condition, const char * description)
{
    g_checks++;
    if (condition)
    {
        printf("  ok    %s\n", description);
    }
    else
    {
        printf("  FAIL  %s\n", description);
        g_failures++;
    }
}

static mission_event_t event_of(mission_event_kind_t kind)
{
    mission_event_t event =
    {
        .kind = kind,
        .barcode = '?',
        .nav_action = NAV_NONE,
        .was_reversed = false,
        .avoid_plan = AVOID_NONE,
        .timeout_state = MISSION_STATE_BOOT
    };

    return event;
}

static mission_result_t send_event(mission_controller_t * controller,
                                   mission_event_t event,
                                   mission_decision_t * decision)
{
    return mission_controller_handle(controller, &event, decision);
}

static void start_robot(mission_controller_t * controller,
                        mission_decision_t * decision)
{
    mission_event_t event;

    mission_controller_init(controller);
    event = event_of(MISSION_EVENT_START);
    (void)send_event(controller, event, decision);
}

static void test_startup(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;
    mission_result_t     result;

    printf("\nstartup\n");
    mission_controller_init(&controller);
    check(MISSION_STATE_BOOT == controller.state, "initial state is BOOT");
    check(!controller.has_pending_nav_action, "there is no pending barcode action");

    event = event_of(MISSION_EVENT_START);
    result = send_event(&controller, event, &decision);
    check(MISSION_RESULT_ACCEPTED == result, "START is accepted in BOOT");
    check(MISSION_STATE_LINE_FOLLOW == controller.state, "START enters LINE_FOLLOW");
    check(MISSION_MOTION_START_LINE_FOLLOW == decision.motion_command,
          "START requests line following");
    check(decision.state_changed, "START reports the state transition");

    result = send_event(&controller, event, &decision);
    check(MISSION_RESULT_IGNORED == result, "a repeated START is ignored");
    check(MISSION_MOTION_NONE == decision.motion_command, "ignored event requests no motion");
}

static void test_barcode_and_junction(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;
    mission_result_t     result;

    printf("\nbarcode then junction\n");
    start_robot(&controller, &decision);

    event = event_of(MISSION_EVENT_BARCODE_DECODED);
    event.barcode      = 'C';
    event.nav_action   = NAV_RIGHT;
    event.was_reversed = true;
    result = send_event(&controller, event, &decision);

    check(MISSION_RESULT_ACCEPTED == result, "valid barcode is accepted");
    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "barcode does not interrupt line following");
    check(controller.has_pending_nav_action, "navigation action is stored for the junction");
    check(NAV_RIGHT == controller.pending_nav_action, "stored action is RIGHT");
    check('C' == controller.pending_barcode, "decoded letter is retained");
    check(controller.pending_barcode_reversed, "reverse-scan evidence is retained");
    check(MISSION_MOTION_NONE == decision.motion_command,
          "barcode alone requests no turn");

    event = event_of(MISSION_EVENT_JUNCTION_DETECTED);
    result = send_event(&controller, event, &decision);
    check(MISSION_RESULT_ACCEPTED == result, "junction with a pending action is accepted");
    check(MISSION_STATE_AT_JUNCTION == controller.state, "controller enters AT_JUNCTION");
    check(MISSION_MOTION_EXECUTE_TURN == decision.motion_command,
          "junction requests a turn");
    check(NAV_RIGHT == decision.nav_action, "the requested turn uses the stored action");
    check(!controller.has_pending_nav_action, "pending action is consumed once");

    event = event_of(MISSION_EVENT_TURN_COMPLETED);
    result = send_event(&controller, event, &decision);
    check(MISSION_RESULT_ACCEPTED == result, "turn completion is accepted");
    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "turn completion restores LINE_FOLLOW");
}

static void test_barcode_validation(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nbarcode validation\n");
    start_robot(&controller, &decision);

    event = event_of(MISSION_EVENT_BARCODE_DECODED);
    event.barcode = 'c';
    event.nav_action = NAV_LEFT;
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "lowercase barcode is ignored");

    event.barcode = 'A';
    event.nav_action = NAV_NONE;
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "unassigned barcode is ignored");
    check(!controller.has_pending_nav_action, "invalid barcodes do not create pending work");
}

static void test_junction_without_barcode(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\njunction without a command\n");
    start_robot(&controller, &decision);

    event = event_of(MISSION_EVENT_JUNCTION_DETECTED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "unsafe junction is handled rather than ignored");
    check(MISSION_STATE_FAULT == controller.state,
          "junction without a barcode enters FAULT");
    check(MISSION_MOTION_STOP == decision.motion_command,
          "unknown route requests an immediate stop");
}

static void test_hump(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nhump handling\n");
    start_robot(&controller, &decision);

    event = event_of(MISSION_EVENT_HUMP_DETECTED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "hump detection is accepted during line following");
    check(MISSION_STATE_HUMP == controller.state, "controller enters HUMP");
    check(MISSION_MOTION_SET_SLOW_SPEED == decision.motion_command,
          "hump requests reduced speed");

    event = event_of(MISSION_EVENT_HUMP_COMPLETED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "hump completion is accepted");
    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "hump completion returns to LINE_FOLLOW");
    check(MISSION_MOTION_RESTORE_NORMAL_SPEED == decision.motion_command,
          "normal speed is restored after the hump");
}

static void test_obstacle_bypass_and_recovery(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nobstacle bypass and line recovery\n");
    start_robot(&controller, &decision);

    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "obstacle detection is accepted");
    check(MISSION_STATE_OBSTACLE_STOPPING == controller.state,
          "controller enters OBSTACLE_STOPPING");
    check(MISSION_MOTION_STOP == decision.motion_command,
          "robot stops before scanning");
    check(!decision.start_obstacle_scan,
          "Buddy 5 is not asked to scan while the car may still be rolling");

    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = AVOID_LEFT;
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "an obstacle profile arriving before the stop is ignored");
    check(MISSION_STATE_OBSTACLE_STOPPING == controller.state,
          "an early profile cannot skip the motor-stop handshake");

    event = event_of(MISSION_EVENT_MOTION_STOPPED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "Buddy 2 confirms that braking completed");
    check(MISSION_STATE_OBSTACLE_SCAN == controller.state,
          "motor-stop confirmation enters OBSTACLE_SCAN");
    check(decision.start_obstacle_scan,
          "Buddy 5 is requested to scan only after the stop confirmation");

    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = AVOID_LEFT;
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "usable obstacle plan is accepted");
    check(MISSION_STATE_BYPASS == controller.state, "controller enters BYPASS");
    check(MISSION_MOTION_EXECUTE_BYPASS == decision.motion_command,
          "Buddy 2 is requested to execute the bypass");
    check(AVOID_LEFT == decision.avoid_plan, "left avoidance plan is forwarded");

    event = event_of(MISSION_EVENT_LINE_REACQUIRED);
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "an early line signal does not cancel an unfinished bypass");
    check(MISSION_STATE_BYPASS == controller.state,
          "controller waits for Buddy 2 to complete the bypass");

    event = event_of(MISSION_EVENT_BYPASS_COMPLETED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "bypass completion is accepted");
    check(MISSION_STATE_LINE_SEARCH == controller.state,
          "bypass completion starts LINE_SEARCH");

    event = event_of(MISSION_EVENT_LINE_REACQUIRED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "line reacquisition is accepted");
    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "line reacquisition restores LINE_FOLLOW");
}

static void test_obstacle_alternative_plans(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nobstacle plan alternatives\n");
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_MOTION_STOPPED);
    (void)send_event(&controller, event, &decision);

    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = AVOID_CONTINUE;
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "CONTINUE plan is accepted");
    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "CONTINUE returns to line following");

    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_MOTION_STOPPED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = AVOID_STOP;
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "STOP plan is accepted");
    check(MISSION_STATE_STOPPED == controller.state, "STOP plan enters STOPPED");

    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_MOTION_STOPPED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = AVOID_NONE;
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "missing obstacle plan is ignored");
    check(MISSION_STATE_OBSTACLE_SCAN == controller.state,
          "invalid plan leaves the robot stopped in scan state");

    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_LINE_LOST);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_MOTION_STOPPED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = AVOID_CONTINUE;
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "CONTINUE is accepted after an obstacle interrupts line search");
    check(MISSION_STATE_LINE_SEARCH == controller.state,
          "CONTINUE does not falsely claim that the lost line was recovered");
}

static void test_line_loss(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nline loss and recovery\n");
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_LINE_LOST);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "line loss is accepted");
    check(MISSION_STATE_LINE_SEARCH == controller.state, "line loss enters LINE_SEARCH");
    check(MISSION_MOTION_START_LINE_SEARCH == decision.motion_command,
          "line loss requests search motion");

    event = event_of(MISSION_EVENT_LINE_REACQUIRED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "reacquisition returns to LINE_FOLLOW");
}

static void test_stop_and_resume(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nstop and safe resume\n");
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_STOP_REQUESTED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "STOP is accepted from LINE_FOLLOW");
    check(MISSION_STATE_STOPPED == controller.state, "STOP enters STOPPED");
    check(MISSION_MOTION_STOP == decision.motion_command, "STOP requests motor stop");

    event = event_of(MISSION_EVENT_RESUME_REQUESTED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "RESUME is accepted after a normal stop");
    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "normal stop resumes line following");

    /* Stop midway through a bypass: never replay a half-completed manoeuvre. */
    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_MOTION_STOPPED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = AVOID_RIGHT;
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_STOP_REQUESTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_RESUME_REQUESTED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "RESUME is accepted after an interrupted bypass");
    check(MISSION_STATE_LINE_SEARCH == controller.state,
          "interrupted bypass resumes with line search");
    check(MISSION_MOTION_START_LINE_SEARCH == decision.motion_command,
          "resume does not blindly repeat the bypass");
}

static void test_safety_faults(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nsafety faults and timeouts\n");
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_IMPACT_DETECTED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "impact is accepted from an active state");
    check(MISSION_STATE_FAULT == controller.state, "impact enters FAULT");
    check(MISSION_MOTION_STOP == decision.motion_command, "impact stops the motors");

    event = event_of(MISSION_EVENT_STOP_REQUESTED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "STOP is still acknowledged while faulted");
    check(MISSION_STATE_FAULT == controller.state,
          "STOP cannot downgrade FAULT into a resumable state");
    check(MISSION_MOTION_STOP == decision.motion_command,
          "STOP while faulted reinforces the motor stop");

    event = event_of(MISSION_EVENT_RESUME_REQUESTED);
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "FAULT cannot be cleared by an ordinary RESUME");

    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_LINE_LOST);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_TIMEOUT);
    event.timeout_state = MISSION_STATE_LINE_SEARCH;
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "line-search timeout is accepted");
    check(MISSION_STATE_FAULT == controller.state, "line-search timeout enters FAULT");
    check(MISSION_MOTION_STOP == decision.motion_command, "timeout stops the motors");

    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_TIMEOUT);
    event.timeout_state = MISSION_STATE_LINE_FOLLOW;
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "timeout is ignored when no timed manoeuvre is active");

    /* A turn timer can expire after an obstacle has pre-empted the turn. */
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_BARCODE_DECODED);
    event.barcode = 'B';
    event.nav_action = NAV_LEFT;
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_JUNCTION_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_TIMEOUT);
    event.timeout_state = MISSION_STATE_AT_JUNCTION;
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "a stale timeout from an interrupted turn is ignored");
    check(MISSION_STATE_OBSTACLE_STOPPING == controller.state,
          "a stale timer cannot fault the current obstacle-stop handshake");
}

static void test_obstacle_preempts_hump(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nbehaviour priority\n");
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_HUMP_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "obstacle pre-empts hump handling");
    check(MISSION_STATE_OBSTACLE_STOPPING == controller.state,
          "higher-priority obstacle becomes the active state");
    check(MISSION_MOTION_STOP == decision.motion_command,
          "obstacle pre-emption stops the robot");
}

static void test_invalid_and_out_of_order_events(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\ninvalid and out-of-order input\n");
    start_robot(&controller, &decision);

    event = event_of(MISSION_EVENT_TURN_COMPLETED);
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "turn completion is ignored when no turn is active");
    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "out-of-order event leaves state unchanged");

    event = event_of((mission_event_kind_t)MISSION_EVENT_COUNT);
    check(MISSION_RESULT_INVALID_ARGUMENT == send_event(&controller, event, &decision),
          "out-of-range event is rejected");

    event = event_of(MISSION_EVENT_BARCODE_DECODED);
    event.barcode = 'A';
    event.nav_action = (nav_action_t)NAV_COUNT;
    check(MISSION_RESULT_INVALID_ARGUMENT ==
          send_event(&controller, event, &decision),
          "out-of-range barcode action is rejected");

    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_MOTION_STOPPED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = (avoid_plan_t)AVOID_COUNT;
    check(MISSION_RESULT_INVALID_ARGUMENT ==
          send_event(&controller, event, &decision),
          "out-of-range avoidance plan is rejected");

    event = event_of(MISSION_EVENT_TIMEOUT);
    event.timeout_state = (mission_state_t)MISSION_STATE_COUNT;
    check(MISSION_RESULT_INVALID_ARGUMENT ==
          send_event(&controller, event, &decision),
          "out-of-range timeout owner is rejected");

    controller.state = (mission_state_t)MISSION_STATE_COUNT;
    event = event_of(MISSION_EVENT_START);
    check(MISSION_RESULT_INVALID_ARGUMENT ==
          send_event(&controller, event, &decision),
          "out-of-range controller state is rejected");

    mission_controller_init(&controller);
    check(MISSION_RESULT_INVALID_ARGUMENT ==
          mission_controller_handle(NULL, &event, &decision),
          "NULL controller is rejected");
    check(MISSION_RESULT_INVALID_ARGUMENT ==
          mission_controller_handle(&controller, NULL, &decision),
          "NULL event is rejected");
    check(MISSION_RESULT_INVALID_ARGUMENT ==
          mission_controller_handle(&controller, &event, NULL),
          "NULL decision is rejected");
}

static void test_names(void)
{
    printf("\nname tables\n");
    check(0 == strcmp("LINE_FOLLOW", mission_state_name(MISSION_STATE_LINE_FOLLOW)),
          "state name is readable");
    check(0 == strcmp("BARCODE_DECODED",
                      mission_event_name(MISSION_EVENT_BARCODE_DECODED)),
          "event name is readable");
    check(0 == strcmp("EXECUTE_BYPASS",
                      mission_motion_command_name(MISSION_MOTION_EXECUTE_BYPASS)),
          "motion command name is readable");
    check(0 == strcmp("INVALID", mission_state_name(MISSION_STATE_COUNT)),
          "invalid state name is bounded");
    check(0 == strcmp("INVALID", mission_event_name(MISSION_EVENT_COUNT)),
          "invalid event name is bounded");
    check(0 == strcmp("INVALID", mission_motion_command_name(MISSION_MOTION_COUNT)),
          "invalid motion name is bounded");
}


/* -------------------------------------------------------------------------
 * Characterisation tests for two OPEN team decisions.
 *
 * These pin down what the controller does today. They are written so that if
 * the team decides differently, the test fails loudly and points at the
 * decision - rather than the behaviour changing unnoticed. Neither is
 * asserted to be correct; both are recorded as open decisions 8 and 9 in
 * INTEGRATION_CONTRACT.md.
 * ------------------------------------------------------------------------- */

static void test_line_lost_during_a_manoeuvre(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nline loss DURING a manoeuvre (open decision 8)\n");

/*
     * Physical reality: while the car pivots through a junction, its line
     * sensors leave the line. If Buddy 3 reports that as LINE_LOST, the turn
     * is abandoned midway and the later TURN_COMPLETED is discarded.
     *
     * Either Buddy 3 must suppress LINE_LOST while a turn is in progress, or
     * the controller must ignore it in AT_JUNCTION. That is a team decision,
     * so this test records today's behaviour rather than changing it.
     */
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_BARCODE_DECODED);
    event.barcode = 'C';
    event.nav_action = NAV_RIGHT;
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_JUNCTION_DETECTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_AT_JUNCTION == controller.state, "a turn is in progress");

    event = event_of(MISSION_EVENT_LINE_LOST);
    check(MISSION_RESULT_ACCEPTED == send_event(&controller, event, &decision),
          "TODAY: line loss during a turn is accepted");
    check(MISSION_STATE_LINE_SEARCH == controller.state,
          "TODAY: the unfinished turn is abandoned for LINE_SEARCH");

    event = event_of(MISSION_EVENT_TURN_COMPLETED);
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "TODAY: the turn's own completion is then discarded");

    /* The same question applies while crossing a hump, where one wheel is
       raised and the sensors may momentarily leave the line. */
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_HUMP_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_LINE_LOST);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_LINE_SEARCH == controller.state,
          "TODAY: line loss during a hump also abandons the hump");

    event = event_of(MISSION_EVENT_HUMP_COMPLETED);
    check(MISSION_RESULT_IGNORED == send_event(&controller, event, &decision),
          "TODAY: the hump's completion is then discarded");
}

static void test_slow_speed_is_not_restored_on_every_exit(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nhump speed restoration (open decision 9)\n");

    /* The intended path does restore normal speed. */
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_HUMP_DETECTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_MOTION_SET_SLOW_SPEED == decision.motion_command,
          "entering HUMP asks Buddy 2 to slow down");
    event = event_of(MISSION_EVENT_HUMP_COMPLETED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_MOTION_RESTORE_NORMAL_SPEED == decision.motion_command,
          "completing the hump restores normal speed");

    /*
     * But HUMP can also be left by a stop, or by an obstacle. On those paths
     * no RESTORE_NORMAL_SPEED is ever issued.
     *
     * Whether that matters depends on an unanswered ownership question: does
     * START_LINE_FOLLOW already imply normal speed (making RESTORE redundant),
     * or is speed a separate axis Buddy 2 keeps until told otherwise (in which
     * case the car finishes the course at hump speed)? The contract says a
     * motion command "replaces the previous behaviour", which is suggestive
     * but not decisive.
     */
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_HUMP_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_STOP_REQUESTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_RESUME_REQUESTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "stop then resume during a hump returns to LINE_FOLLOW");
    check(MISSION_MOTION_RESTORE_NORMAL_SPEED != decision.motion_command,
          "TODAY: that path never asks for normal speed to be restored");
    check(MISSION_MOTION_START_LINE_FOLLOW == decision.motion_command,
          "TODAY: it issues START_LINE_FOLLOW instead");

    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_HUMP_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_MOTION_STOPPED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = AVOID_LEFT;
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_BYPASS_COMPLETED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_LINE_REACQUIRED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "an obstacle during a hump ends back in LINE_FOLLOW");
    check(MISSION_MOTION_RESTORE_NORMAL_SPEED != decision.motion_command,
          "TODAY: that path never restores normal speed either");
}

static void test_pending_barcode_survives_a_detour(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\na stored barcode action survives an obstacle detour\n");

    /*
     * The barcode is printed before the junction, so an obstacle between the
     * two must not lose the action - otherwise the car reaches the junction
     * with nothing pending and (per open decision 4) faults.
     */
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_BARCODE_DECODED);
    event.barcode = 'C';
    event.nav_action = NAV_RIGHT;
    (void)send_event(&controller, event, &decision);
    check(controller.has_pending_nav_action, "the action is stored");

    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_MOTION_STOPPED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = AVOID_LEFT;
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_BYPASS_COMPLETED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_LINE_REACQUIRED);
    (void)send_event(&controller, event, &decision);

    check(MISSION_STATE_LINE_FOLLOW == controller.state,
          "the car is back on the line after the detour");
    check(controller.has_pending_nav_action,
          "the stored action survived the whole detour");

    event = event_of(MISSION_EVENT_JUNCTION_DETECTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_AT_JUNCTION == controller.state,
          "the junction is still handled as a turn, not a fault");
    check(NAV_RIGHT == decision.nav_action,
          "and it is the action decoded before the obstacle");
}

static void test_stop_during_every_moving_state(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;

    printf("\nsafety stop is honoured from every moving state\n");

    /* The contract says "any moving state --STOP--> STOPPED". Check each one
       rather than trusting the arbitration order to have covered them all. */
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_STOP_REQUESTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_STOPPED == controller.state, "stop works from LINE_FOLLOW");

    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_HUMP_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_STOP_REQUESTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_STOPPED == controller.state, "stop works from HUMP");

    /* AT_JUNCTION is a moving state too - the turn is being executed - so it
       has to be covered here or the claim is not true. */
    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_BARCODE_DECODED);
    event.barcode = 'C';
    event.nav_action = NAV_RIGHT;
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_JUNCTION_DETECTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_AT_JUNCTION == controller.state, "a turn is executing");
    event = event_of(MISSION_EVENT_STOP_REQUESTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_STOPPED == controller.state, "stop works from AT_JUNCTION");
    check(MISSION_MOTION_STOP == decision.motion_command,
          "and it commands Buddy 2 to stop mid-turn");

    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_STOP_REQUESTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_STOPPED == controller.state,
          "stop works from OBSTACLE_STOPPING");

    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_DETECTED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_MOTION_STOPPED);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_OBSTACLE_PROFILE_READY);
    event.avoid_plan = AVOID_RIGHT;
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_BYPASS == controller.state, "a bypass is in progress");
    event = event_of(MISSION_EVENT_STOP_REQUESTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_STOPPED == controller.state, "stop works from BYPASS");

    start_robot(&controller, &decision);
    event = event_of(MISSION_EVENT_LINE_LOST);
    (void)send_event(&controller, event, &decision);
    event = event_of(MISSION_EVENT_STOP_REQUESTED);
    (void)send_event(&controller, event, &decision);
    check(MISSION_STATE_STOPPED == controller.state, "stop works from LINE_SEARCH");
}

int main(void)
{
    printf("INF2004 central mission controller\n");

    test_startup();
    test_barcode_and_junction();
    test_barcode_validation();
    test_junction_without_barcode();
    test_hump();
    test_obstacle_bypass_and_recovery();
    test_obstacle_alternative_plans();
    test_line_loss();
    test_stop_and_resume();
    test_safety_faults();
    test_obstacle_preempts_hump();
    test_invalid_and_out_of_order_events();
    test_line_lost_during_a_manoeuvre();
    test_slow_speed_is_not_restored_on_every_exit();
    test_pending_barcode_survives_a_detour();
    test_stop_during_every_moving_state();
    test_names();

    printf("\n%u checks, %u failures\n", g_checks, g_failures);
    if (0u == g_failures)
    {
        printf("ALL MISSION TESTS PASS\n");
        return 0;
    }

    printf("MISSION TESTS FAILED\n");
    return 1;
}
