/*
 * consistency_test.c - cross-module agreement checks.
 *
 * The other two suites each test one module in isolation. This one exists to
 * catch disagreements BETWEEN modules, which neither of them can see.
 *
 * Why it is a separate binary: telemetry.c and mission_controller.c are
 * deliberately independent - communications must not depend on mission logic,
 * and the controller must not depend on MQTT. Linking them together inside
 * either suite would create exactly the coupling the architecture avoids. So
 * they are joined only here, in test code, where it costs nothing.
 *
 * This is the executable form of the "Design Consistency" section the Week 6
 * template asks for: evidence that the artefacts describe one system rather
 * than several that happen to compile.
 */

#include "buddy1/telemetry.h"
#include "controller/mission_controller.h"

#include <stdio.h>
#include <string.h>

/* telemetry.c needs these; the tests are single-threaded and use no clock. */
uint32_t tele_now_ms(void) { return 0u; }
void     tele_lock(void)   { }
void     tele_unlock(void) { }

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

/*
 * HISTORY, because it changes what this test is worth.
 *
 * telemetry.c and mission_controller.c each used to keep their own array of
 * state names for the SAME enum. They agreed, and nothing made them keep
 * agreeing - rename a state in one and MQTT would report a different name
 * from the serial log, silently. This test was written to catch that drift.
 *
 * The tables have since been merged into robot_types.c, next to the enum, so
 * the drift is now impossible rather than merely detected. tele_state_name()
 * is an alias for mission_state_name(), so the comparison below is close to
 * tautological TODAY.
 *
 * It is kept because it stops being tautological the moment anyone
 * reintroduces a second table - which is exactly the change that would
 * reopen the hole. A cheap regression guard, not a live defence.
 */
static void test_state_names_agree(void)
{
    unsigned index;
    bool     all_agree = true;

    printf("\nthe two state-name tables agree\n");

    for (index = 0u; index < (unsigned)MISSION_STATE_COUNT; index++)
    {
        const char * from_telemetry = tele_state_name((tele_state_t)index);
        const char * from_mission   = mission_state_name((mission_state_t)index);

        if (0 != strcmp(from_telemetry, from_mission))
        {
            printf("        index %u: telemetry says \"%s\", mission says \"%s\"\n",
                   index, from_telemetry, from_mission);
            all_agree = false;
        }
    }

    check(all_agree,
          "telemetry and the controller still resolve every state identically");

    /* Both must also reject an out-of-range value the same way, or a corrupt
       state would be reported as a real one on one side and not the other. */
    check(0 == strcmp("INVALID", tele_state_name((tele_state_t)MISSION_STATE_COUNT)),
          "telemetry rejects an out-of-range state");
    check(0 == strcmp("INVALID", mission_state_name(MISSION_STATE_COUNT)),
          "the controller rejects an out-of-range state");
}

/*
 * nav_action_t is shared: the controller decides which action a barcode means,
 * and telemetry publishes the name of that action. If the two disagree, the
 * dashboard shows the car doing something other than what it is doing.
 */
static void test_action_names_are_shared(void)
{
    printf("\nnavigation actions mean the same thing on both sides\n");

    check(0 == strcmp("LEFT", nav_action_name(NAV_LEFT)), "LEFT");
    check(0 == strcmp("RIGHT", nav_action_name(NAV_RIGHT)), "RIGHT");
    check(0 == strcmp("STRAIGHT", nav_action_name(NAV_STRAIGHT)), "STRAIGHT");
    check(0 == strcmp("UTURN", nav_action_name(NAV_UTURN)), "UTURN");

    /* The controller refuses NAV_NONE as a barcode action, and telemetry
       still has to be able to name it - they must not disagree about whether
       it is a legal value. */
    check(0 == strcmp("NONE", nav_action_name(NAV_NONE)),
          "NAV_NONE is nameable even though it is not a legal barcode action");
}

/*
 * A decision produced by the controller must be publishable by telemetry
 * without translation. This walks one real scenario end to end and confirms
 * every value that crosses the boundary is one telemetry accepts.
 */
static void test_a_decision_can_be_published(void)
{
    mission_controller_t controller;
    mission_decision_t   decision;
    mission_event_t      event;
    tele_msg_t           msg;

    printf("\na controller decision can be published unchanged\n");

    tele_init();
    mission_controller_init(&controller);

    memset(&event, 0, sizeof event);
    event.kind = MISSION_EVENT_START;
    (void)mission_controller_handle(&controller, &event, &decision);

    /* The state the controller just entered is published directly - no
       mapping table, because tele_state_t IS mission_state_t. */
    check(tele_report_state(decision.current_state),
          "the new mission state is accepted by telemetry");
    check(tele_queue_pop(&msg), "and produces a message");
    check(NULL != strstr(msg.payload, "LINE_FOLLOW"),
          "carrying the same state name the controller reports");

    /* Now a barcode, whose action comes back out in the decision. */
    memset(&event, 0, sizeof event);
    event.kind = MISSION_EVENT_BARCODE_DECODED;
    event.barcode = 'C';
    event.nav_action = NAV_RIGHT;
    (void)mission_controller_handle(&controller, &event, &decision);

    memset(&event, 0, sizeof event);
    event.kind = MISSION_EVENT_JUNCTION_DETECTED;
    (void)mission_controller_handle(&controller, &event, &decision);

    check(NAV_RIGHT == decision.nav_action, "the junction decision carries the action");
    check(tele_report_barcode('C', decision.nav_action, false),
          "telemetry accepts that action without translation");
    while (tele_queue_pop(&msg)) { /* reach the barcode message */ }
    check(0u == tele_queue_depth(), "queue drained");
}

int main(void)
{
    printf("INF2004 cross-module consistency\n");

    test_state_names_agree();
    test_action_names_are_shared();
    test_a_decision_can_be_published();

    printf("\n%u checks, %u failures\n", g_checks, g_failures);

    if (0u == g_failures)
    {
        printf("ALL CONSISTENCY TESTS PASS\n");
        return 0;
    }

    printf("CONSISTENCY TESTS FAILED\n");
    return 1;
}
