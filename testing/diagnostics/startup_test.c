/* Actual app_main.c startup, with task execution and hardware replaced by
 * bounded event observations. This does not simulate an RTOS scheduler. */
#include "firmware_test_kernel.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_TEST 0
#define APP_IR_BENCH 0
#define TK_SUPPORT_SMP 1
#define EXPORT
#define IMPORT extern
#define TASK_STACK_SIZE 2048U
#define TA_HLNG 1U
#define TA_RNG3 2U
#define TA_ASSPRC 4U
#define TP_PRC1 1U
#define TSK_SELF 0
#define TMO_FEVR (-1)
typedef int PRI;
typedef uint32_t UW;
typedef void (*FP)(void);
typedef struct { FP task; PRI itskpri; UW stksz, tskatr, assprc; } T_CTSK;
typedef struct { unsigned spver, prver; } T_RVER;

static jmp_buf boundary;
static unsigned events[32], event_count, initialized, started;
static unsigned delays, reports, priority_changes;
static ER init_result, priority_result;
static PRI current_priority;
static int diagnostic_attempted;

static void event(unsigned value)
{ assert(event_count < 32U); events[event_count++] = value; }
static ER subsystem_init(unsigned index)
{
    assert(diagnostic_attempted && started == 0U);
    event(10U + index); initialized |= 1U << (index - 1U); return E_OK;
}
static ER subsystem_start(unsigned index)
{
    assert(diagnostic_attempted && initialized == 63U);
    event(20U + index); started |= 1U << (index - 1U); return E_OK;
}
ER subsystem1_init(void) { return subsystem_init(1); }
ER subsystem2_init(void) { return subsystem_init(2); }
ER subsystem3_init(void) { return subsystem_init(3); }
ER subsystem4_init(void) { return subsystem_init(4); }
ER subsystem5_init(void) { return subsystem_init(5); }
ER vehicle_controller_init(void) { return subsystem_init(6); }
ER subsystem1_start(void) { return subsystem_start(1); }
ER subsystem2_start(void) { return subsystem_start(2); }
ER subsystem3_start(void) { return subsystem_start(3); }
ER subsystem4_start(void) { return subsystem_start(4); }
ER subsystem5_start(void) { return subsystem_start(5); }
ER vehicle_controller_start(void) { return subsystem_start(6); }
ER robot_diagnostics_init(void)
{ assert(initialized == 0U && started == 0U); diagnostic_attempted = 1; event(1); return init_result; }
void robot_diagnostics_report(void)
{ assert(current_priority == 23 && delays == 1U && started == 63U); reports++; event(41); }
ER tk_chg_pri(ID id, PRI priority)
{
    assert(id == TSK_SELF && priority == 23 && started == 63U);
    priority_changes++; event(30);
    if (priority_result == E_OK) { current_priority = priority; }
    return priority_result;
}
ER tk_dly_tsk(UW milliseconds)
{
    assert(current_priority == 23 && milliseconds == 1000U);
    delays++; event(40);
    if (delays == 2U) { longjmp(boundary, 1); }
    return E_OK;
}
ER tk_slp_tsk(TMO timeout)
{ assert(timeout == TMO_FEVR && reports == 0U); event(50); longjmp(boundary, 2); }
ER tk_ref_ver(T_RVER *version)
{ version->spver = 0x300U; version->prver = 1U; return E_OK; }
INT tk_get_prc(void) { return 1; }
ID tk_cre_tsk(const T_CTSK *task) { (void)task; assert(0); return -1; }
ER tk_sta_tsk(ID id, INT value) { (void)id; (void)value; assert(0); return -1; }
ER tk_del_tsk(ID id) { (void)id; assert(0); return -1; }
INT tm_printf(const UB *format, ...)
{
    char scratch[256];
    va_list arguments;
    if (strncmp((const char *)format, "[DIAG]", 6U) == 0) {
        assert(current_priority == 23);
    }
    va_start(arguments, format);
    int count = vsnprintf(scratch, sizeof(scratch), (const char *)format, arguments);
    va_end(arguments);
    assert(count >= 0 && (unsigned)count < sizeof(scratch));
    return count;
}
#define CHECK(expression, name) do { ER e_ = (expression); if (e_ < E_OK) \
    { tm_printf((const UB *)"ERROR %s: %d\n", (name), e_); } } while (0)

/* No begin, movement or motion_poll symbol exists in this executable. */
#include "../../app_program/app_main.c"

static void run_case(ER diagnostic_error, ER priority_error)
{
    static const unsigned startup[] = {1, 11, 12, 13, 14, 15, 16,
                                         21, 22, 26, 23, 24, 25};
    event_count = initialized = started = delays = reports = priority_changes = 0U;
    diagnostic_attempted = 0; current_priority = 1;
    init_result = diagnostic_error; priority_result = priority_error;
    int result = setjmp(boundary);
    if (result == 0) { (void)usermain(); assert(0); }
    assert(initialized == 63U && started == 63U);
    assert(event_count >= sizeof(startup) / sizeof(startup[0]));
    assert(memcmp(events, startup, sizeof(startup)) == 0);
    if (diagnostic_error != E_OK) {
        assert(result == 2 && priority_changes == 0U && delays == 0U && reports == 0U);
        assert(event_count == 14U && events[13] == 50U);
    } else if (priority_error != E_OK) {
        assert(result == 2 && priority_changes == 1U && delays == 0U && reports == 0U);
        assert(event_count == 15U && events[13] == 30U && events[14] == 50U);
    } else {
        assert(result == 1 && priority_changes == 1U && delays == 2U && reports == 1U);
        assert(event_count == 17U && events[13] == 30U && events[14] == 40U
               && events[15] == 41U && events[16] == 40U);
    }
}
int main(void)
{
    run_case(E_OK, E_OK);
    run_case(-91, E_OK);
    run_case(E_OK, -92);
    puts("PASS: actual diagnostic startup, all original subsystem hooks, low-priority delay/report, and non-fatal init/priority failures");
    return 0;
}
