/* Actual test runner with fake time/buttons/motion boundaries; no hardware. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#define APP_H
#define APP_MOTION_SMOKE 1
#define E_OK 0
#define TSK_SELF 0
#define GPIO_MODE_IN 0
#define GPIO(pin) (pin)
#define GPIO_PDE 1U
#define GPIO_PUE 2U
#define clr_w(address, bits) ((void)(address), (void)(bits))
#define set_w(address, bits) ((void)(address), (void)(bits))
typedef int ER, INT;
typedef unsigned char UB;
typedef struct { int32_t hi; uint32_t lo; } SYSTIM;
#include "../../app_program/subsystem2/motion.h"
#include "../../app_program/subsystem2/motion_codec.h"

enum { COMPLETE, STOP_COUNTDOWN, STOP_RUN, TIMEOUT, FAULT, REJECT,
       STOP_PRIORITY, HELD_BOOT, NO_PRESS, NOT_READY, CLOCK_FAIL, WAIT_FAIL,
       UNEXPECTED, HELD_ONLY, CLOCK_FAIL_RUN, GPIO_FAIL, PRIORITY_FAIL };
static unsigned scenario, requests, stops;
static uint32_t clock_value, base, request_at;
static char output[4096];
static size_t used;
static jmp_buf finished;

ER tk_chg_pri(INT task, INT priority)
{ assert(task == TSK_SELF && priority == 15); return scenario == PRIORITY_FAIL ? -1 : E_OK; }
ER gpio_set_pin(unsigned pin, unsigned mode)
{ assert((pin == 20U || pin == 21U) && mode == GPIO_MODE_IN); return scenario == GPIO_FAIL ? -1 : E_OK; }
ER tk_get_otm(SYSTIM *value)
{
    if (scenario == CLOCK_FAIL || (scenario == CLOCK_FAIL_RUN && requests)) { return -1; }
    value->hi = 0; value->lo = clock_value; return E_OK;
}
ER tk_dly_tsk(unsigned delay)
{
    if (scenario == WAIT_FAIL && requests) { return -1; }
    clock_value += delay;
    if ((uint32_t)(clock_value-base) > 12000U) { longjmp(finished, 1); }
    return E_OK;
}
unsigned gpio_get_val(unsigned pin)
{
    uint32_t elapsed = clock_value-base;
    bool down;
    if (pin == 21U) {
        down = (scenario == STOP_PRIORITY && elapsed >= 650U) ||
               (scenario == STOP_COUNTDOWN && elapsed >= 1000U) ||
               (scenario == STOP_RUN && requests && (uint32_t)(clock_value-request_at) >= 100U);
    } else {
        down = scenario == HELD_ONLY || (scenario == HELD_BOOT && elapsed < 800U) ||
               (scenario == HELD_BOOT ? (elapsed >= 1000U && elapsed < 1100U) :
                (scenario != NO_PRESS && elapsed >= 650U && elapsed < 750U));
    }
    return down ? 0U : 1U;
}
INT tm_printf(const UB *format, ...)
{
    va_list args; va_start(args, format);
    int n = vsnprintf(output+used, sizeof(output)-used, (const char *)format, args);
    va_end(args); assert(n >= 0 && (size_t)n < sizeof(output)-used); used += (size_t)n;
    return n;
}
void stop(void) { stops++; }
int32_t enc_l_count(void) { return 100; }
int32_t enc_r_count(void) { return 200; }
motion_fault_t motion_get_fault(void) { return scenario == FAULT ? MOTION_FAULT_ENCODER : MOTION_FAULT_NONE; }
bool motion_is_calibrated(void) { return true; }
bool motion_read_snapshot(motion_snapshot_t *out)
{
    return motion_snapshot_pack(out, MOTION_IDLE, MOTION_FAULT_NONE,
        scenario == NOT_READY, true, 0, 0);
}
motion_result_t motion_request_drive(float distance, float speed)
{
    assert(distance == 0.10f && speed == 0.05f && requests == 0U);
    assert((uint32_t)(clock_value-base) >= 3680U);
    requests++; request_at = clock_value;
    return scenario == REJECT ? MOTION_ERR_BUSY : MOTION_OK;
}
motion_state_t motion_poll(void)
{
    if (scenario == FAULT) { return MOTION_FAULT; }
    if (scenario == UNEXPECTED) { return MOTION_IDLE; }
    if (scenario != TIMEOUT && (uint32_t)(clock_value-request_at) >= 400U) { return MOTION_COMPLETE; }
    return MOTION_DRIVE;
}
#include "../../app_program/diagnostics/motion_smoke_test.c"

static void run(unsigned kind, const char *reason, bool moves, uint32_t starting_clock)
{
    scenario=kind; requests=stops=0U; used=0U; output[0]='\0';
    base=clock_value=starting_clock;
    int expired = setjmp(finished);
    if (!expired) { motion_smoke_run(); }
    if (kind == NO_PRESS || kind == HELD_ONLY) {
        assert(expired && requests == 0U); return;
    }
    assert(!expired && requests == (moves ? 1U : 0U));
    if (kind == GPIO_FAIL || kind == PRIORITY_FAIL) { assert(requests == 0U); return; }
    assert(strstr(output, reason) && strstr(output, "FINISHED") && stops >= 2U);
    if (kind == TIMEOUT) { assert((uint32_t)(clock_value-request_at) == 3050U); }
}
int main(void)
{
    run(COMPLETE,"encoder target reached",true,0U);
    run(COMPLETE,"encoder target reached",true,UINT32_MAX-2000U);
    run(STOP_COUNTDOWN,"cancelled in countdown",false,0U);
    run(STOP_RUN,"GP21 stop",true,0U);
    run(TIMEOUT,"software timeout",true,0U);
    run(FAULT,"Buddy 2 fault",true,0U);
    run(REJECT,"rejected",true,0U);
    run(STOP_PRIORITY,"cancelled before start",false,0U);
    run(HELD_BOOT,"encoder target reached",true,0U);
    run(NO_PRESS,"",false,0U);
    run(HELD_ONLY,"",false,0U);
    run(NOT_READY,"not ready",false,0U);
    run(CLOCK_FAIL,"not ready",false,0U);
    run(CLOCK_FAIL_RUN,"clock failed",true,0U);
    run(WAIT_FAIL,"wait failed",true,0U);
    run(UNEXPECTED,"unexpected motion state",true,0U);
    run(GPIO_FAIL,"",false,0U);
    run(PRIORITY_FAIL,"",false,0U);
    puts("PASS: one-shot motion test: explicit start, held boot/no auto-start, stop priority/countdown/run, timeout, completion, faults, rejection, clock wrap/failure, wait/GPIO/priority failure");
}
