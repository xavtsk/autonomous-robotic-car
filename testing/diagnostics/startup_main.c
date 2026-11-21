/** @file startup_main.c
 * @brief Actual app_main.c startup, with task execution and hardware replaced
 * by bounded event observations.
 *
 * This does not simulate an RTOS scheduler.
 */
#include "startup_main.h"
#include "../../app_program/app_main.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static jmp_buf  g_boundary;
static unsigned g_events[32];
static unsigned g_event_count;
static unsigned g_initialized;
static unsigned g_started;
static unsigned g_delays;
static unsigned g_reports;
static unsigned g_priority_changes;
static ER       g_init_result;
static ER       g_priority_result;
static PRI      g_current_priority;
static int      g_diagnostic_attempted;

static void event (unsigned value);
static ER   subsystem_init (unsigned index);
static ER   subsystem_start (unsigned index);
static void run_case (ER diagnostic_error, ER priority_error);

/** @brief Record the telemetry initialization at position 1 in the startup
 * fixture and return its result.
 */
ER subsystem1_init (void)
{
    return subsystem_init(1);
}

/** @brief Record the motion initialization at position 2 in the startup
 * fixture and return its result.
 */
ER subsystem2_init (void)
{
    return subsystem_init(2);
}

/** @brief Record the navigation initialization at position 3 in the startup
 * fixture and return its result.
 */
ER subsystem3_init (void)
{
    return subsystem_init(3);
}

/** @brief Record the terrain initialization at position 4 in the startup
 * fixture and return its result.
 */
ER subsystem4_init (void)
{
    return subsystem_init(4);
}

/** @brief Record the obstacle initialization at position 5 in the startup
 * fixture and return its result.
 */
ER subsystem5_init (void)
{
    return subsystem_init(5);
}

/** @brief Record the vehicle controller initialization at position 6 in the
 * startup fixture and return its result.
 */
ER vehicle_controller_init (void)
{
    return subsystem_init(6);
}

/** @brief Record the telemetry start with subsystem ID 1 after
 * initialization and return its result.
 */
ER subsystem1_start (void)
{
    return subsystem_start(1);
}

/** @brief Record the motion start with subsystem ID 2 after initialization
 * and return its result.
 */
ER subsystem2_start (void)
{
    return subsystem_start(2);
}

/** @brief Record the navigation start with subsystem ID 3 after
 * initialization and return its result.
 */
ER subsystem3_start (void)
{
    return subsystem_start(3);
}

/** @brief Record the terrain start with subsystem ID 4 after initialization
 * and return its result.
 */
ER subsystem4_start (void)
{
    return subsystem_start(4);
}

/** @brief Record the obstacle start with subsystem ID 5 after initialization
 * and return its result.
 */
ER subsystem5_start (void)
{
    return subsystem_start(5);
}

/** @brief Record the vehicle controller start with subsystem ID 6 after
 * initialization and return its result.
 */
ER vehicle_controller_start (void)
{
    return subsystem_start(6);
}

/** @brief Record the first startup event before any subsystem starts and
 * return the injected diagnostic initialization result.
 */
ER robot_diagnostics_init (void)
{
    assert(0U == g_initialized && 0U == g_started);
    g_diagnostic_attempted = 1;
    event(1);
    return g_init_result;
}

/** @brief Require priority 23, one delay and six started subsystems before
 * counting and recording a report.
 */
void robot_diagnostics_report (void)
{
    assert(23 == g_current_priority && 1U == g_delays && 63U == g_started);
    g_reports++;
    event(41);
}

/** @brief Require the current task to request priority 23 after startup.
 * Record the attempt and change fixture priority only on success.
 */
ER tk_chg_pri (ID h_task, PRI priority)
{
    assert(TSK_SELF == h_task && 23 == priority && 63U == g_started);
    g_priority_changes++;
    event(30);

    if (E_OK == g_priority_result)
    {
        g_current_priority = priority;
    }

    return g_priority_result;
}

/** @brief Check the one-second diagnostic cadence and return control to the
 * test on its second delay.
 */
ER tk_dly_tsk (UW milliseconds)
{
    assert(23 == g_current_priority && 1000U == milliseconds);
    g_delays++;
    event(40);

    if (2U == g_delays)
    {
        longjmp(g_boundary, 1);
    }

    return E_OK;
}

/** @brief Check for an indefinite sleep before any report. Append the sleep
 * event and return control to the fixture through its saved jump
 * target.
 */
ER tk_slp_tsk (TMO timeout)
{
    assert(TMO_FEVR == timeout && 0U == g_reports);
    event(50);
    longjmp(g_boundary, 2);
}

/** @brief Write simulated specification version 0x300 and product version
 * one to the caller and return success.
 */
ER tk_ref_ver (T_RVER * p_version)
{
    p_version->spver = 0x300U;
    p_version->prver = 1U;
    return E_OK;
}

/** @brief Report processor one to the simulated kernel caller.
 */
INT tk_get_prc (void)
{
    return 1;
}

/** @brief Fail the test if the runtime takes this unused task-management
 * path.
 */
ID tk_cre_tsk (const T_CTSK * p_task)
{
    (void)p_task;
    assert(0);
    return -1;
}

/** @brief Fail the test if the runtime takes this unused task-management
 * path.
 */
ER tk_sta_tsk (ID h_task, INT value)
{
    (void)h_task;
    (void)value;
    assert(0);
    return -1;
}

/** @brief Fail the test if the runtime takes this unused task-management
 * path.
 */
ER tk_del_tsk (ID h_task)
{
    (void)h_task;
    assert(0);
    return -1;
}

/** @brief Format a startup message into bounded scratch space and return its
 * character count. Diagnostic messages also check the task priority.
 */
INT tm_printf (const UB * p_format, ...)
{
    char    scratch[256];
    va_list arguments;

    if (strncmp((const char *)p_format, "[DIAG]", 6U) == 0)
    {
        assert(23 == g_current_priority);
    }

    va_start(arguments, p_format);
    int count =
        vsnprintf(scratch, sizeof(scratch), (const char *)p_format, arguments);
    va_end(arguments);
    assert(count >= 0 && (unsigned)count < sizeof(scratch));
    return count;
}

/** @brief Execute the startup fixture for successful and failed diagnostic
 * initialization or priority changes and check the resulting task
 * order.
 */
int main (void)
{
    run_case(E_OK, E_OK);
    run_case(-91, E_OK);
    run_case(E_OK, -92);
    puts("PASS: actual diagnostic startup, all original subsystem hooks, "
         "low-priority delay/report, and non-fatal init/priority failures");
    return 0;
}

/** @brief Append one event identifier to the startup trace. An assertion
 * guards its 32 slots.
 */
static void event (unsigned value)
{
    assert(g_event_count < 32U);
    g_events[g_event_count++] = value;
}

/** @brief Require diagnostics to have been attempted and all tasks still
 * stopped. Mark the one-based subsystem as initialized and append
 * its event.
 */
static ER subsystem_init (unsigned index)
{
    assert(g_diagnostic_attempted && 0U == g_started);
    event(10U + index);
    g_initialized |= 1U << (index - 1U);
    return E_OK;
}

/** @brief Require all six initializations to have finished. Mark the
 * one-based subsystem as started and append its event.
 */
static ER subsystem_start (unsigned index)
{
    assert(g_diagnostic_attempted && 63U == g_initialized);
    event(20U + index);
    g_started |= 1U << (index - 1U);
    return E_OK;
}

/** @brief Run application startup with the supplied diagnostic and priority
 * results. Check initialization order, priority changes, reporting
 * and delays. The fixture links no motion-start or polling entry
 * point.
 */
static void run_case (ER diagnostic_error, ER priority_error)
{
    static const unsigned startup[] = {1,  11, 12, 13, 14, 15, 16,
                                       21, 22, 26, 23, 24, 25};
    /* clang-format off */
    g_event_count          = g_initialized = g_started = g_delays = g_reports =
        g_priority_changes                               = 0U;
    g_diagnostic_attempted = 0;
    g_current_priority     = 1;
    g_init_result          = diagnostic_error;
    g_priority_result      = priority_error;
    /* clang-format on */
    int result = setjmp(g_boundary);

    if (0 == result)
    {
        (void)usermain();
        assert(0);
    }

    assert(63U == g_initialized && 63U == g_started);
    assert(g_event_count >= sizeof(startup) / sizeof(startup[0]));
    assert(memcmp(g_events, startup, sizeof(startup)) == 0);

    if (diagnostic_error != E_OK)
    {
        assert(2 == result && 0U == g_priority_changes && 0U == g_delays &&
               0U == g_reports);
        assert(14U == g_event_count && 50U == g_events[13]);
    }
    else if (priority_error != E_OK)
    {
        assert(2 == result && 1U == g_priority_changes && 0U == g_delays &&
               0U == g_reports);
        assert(15U == g_event_count && 30U == g_events[13] &&
               50U == g_events[14]);
    }
    else
    {
        assert(1 == result && 1U == g_priority_changes && 2U == g_delays &&
               1U == g_reports);
        assert(17U == g_event_count && 30U == g_events[13] &&
               40U == g_events[14] && 41U == g_events[15] &&
               40U == g_events[16]);
    }
}

/* End of file. */

