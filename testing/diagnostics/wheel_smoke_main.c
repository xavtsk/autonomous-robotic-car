/** @file wheel_smoke_main.c
 * @brief Actual test runner with fake time/buttons/motion boundaries; no
 * hardware.
 * Unsigned casts spell out the original modulo conversion.
 */
#include <assert.h>
#include "../../app_program/app_types.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include "wheel_smoke_main.h"
#include "../../app_program/diagnostics/smoke_motion.h"
#include "../../app_program/subsystem2/codec_motion.h"

typedef enum
{
    COMPLETE,
    STOP_COUNTDOWN,
    STOP_RUN,
    TIMEOUT,
    FAULT,
    REJECT,
    STOP_PRIORITY,
    HELD_BOOT,
    NO_PRESS,
    NOT_READY,
    CLOCK_FAIL,
    WAIT_FAIL,
    UNEXPECTED,
    HELD_ONLY,
    CLOCK_FAIL_RUN,
    GPIO_FAIL,
    PRIORITY_FAIL
} wheel_smoke_scenario_t;

static unsigned g_scenario;
static unsigned g_requests;
static unsigned g_stops;
static uint32_t g_clock_value;
static uint32_t g_base;
static uint32_t g_request_at;
static char     g_output[4096];
static size_t   g_used;
static jmp_buf  g_finished;

static void run (unsigned kind, const char * p_reason, bool b_should_move,
                 uint32_t starting_clock);

/** @brief Check that the smoke task requests priority 15 for itself. Return
 * the error selected by the priority-failure scenario.
 */
ER tk_chg_pri (INT task, INT priority)
{
    assert(TSK_SELF == task && 15 == priority);
    return (unsigned)PRIORITY_FAIL == g_scenario ? -1 : E_OK;
}

/** @brief Check that GP20 or GP21 is configured as an input and return the
 * simulated setup result.
 */
ER gpio_set_pin (unsigned pin, unsigned mode)
{
    assert((20U == pin || 21U == pin) && GPIO_MODE_IN == mode);
    return (unsigned)GPIO_FAIL == g_scenario ? -1 : E_OK;
}

/** @brief Return the fixture clock in the low time word and zero in the high
 * word. A selected startup or running clock failure leaves both
 * words untouched.
 */
ER tk_get_otm (SYSTIM * p_value)
{
    if (((unsigned)CLOCK_FAIL == g_scenario) ||
        (((unsigned)CLOCK_FAIL_RUN == g_scenario) && g_requests))
    {
        return -1;
    }

    p_value->hi = 0;
    p_value->lo = g_clock_value;
    return E_OK;
}

/** @brief Inject the selected wait failure after a request, otherwise advance
 * the clock until the smoke-test cutoff.
 */
ER tk_dly_tsk (unsigned delay)
{
    if (((unsigned)WAIT_FAIL == g_scenario) && g_requests)
    {
        return -1;
    }

    g_clock_value += delay;

    if ((uint32_t)(g_clock_value - g_base) > 12000U)
    {
        longjmp(g_finished, 1);
    }

    return E_OK;
}

/** @brief Return the active-low GP20 or GP21 level for the current simulated
 * time and button scenario.
 */
unsigned gpio_get_val (unsigned pin)
{
    uint32_t elapsed = g_clock_value - g_base;
    bool     b_is_button_down;

    if (21U == pin)
    {
        b_is_button_down =
            ((((unsigned)STOP_PRIORITY == g_scenario) && (elapsed >= 650U)) ||
             (((unsigned)STOP_COUNTDOWN == g_scenario) &&
              (elapsed >= 1000U))) ||
            ((((unsigned)STOP_RUN == g_scenario) && g_requests) &&
             ((uint32_t)(g_clock_value - g_request_at) >= 100U));
    }
    else
    {
        b_is_button_down =
            (((unsigned)HELD_ONLY == g_scenario) ||
             (((unsigned)HELD_BOOT == g_scenario) && (elapsed < 800U))) ||
            ((unsigned)HELD_BOOT == g_scenario
                 ? ((elapsed >= 1000U) && (elapsed < 1100U))
                 : (((g_scenario != (unsigned)NO_PRESS) && (elapsed >= 650U)) &&
                    (elapsed < 750U)));
    }

    return b_is_button_down ? 0U : 1U;
}

/** @brief Append a bounded formatted smoke-test message to the fixture
 * console and return the character count.
 */
INT tm_printf (const UB * p_format, ...)
{
    va_list args;
    va_start(args, p_format);
    int written = vsnprintf(g_output + g_used, sizeof(g_output) - g_used,
                            (const char *)p_format, args);
    va_end(args);
    assert(written >= 0 && (size_t)written < sizeof(g_output) - g_used);
    g_used += (size_t)written;
    return written;
}

/** @brief Count one motor STOP request in the smoke-test fixture.
 */
void stop (void)
{
    g_stops++;
}

/** @brief Return the fixed left-wheel encoder count of 100.
 */
int32_t enc_l_count (void)
{
    return 100;
}

/** @brief Return the fixed right-wheel encoder count of 200.
 */
int32_t enc_r_count (void)
{
    return 200;
}

/** @brief Return the encoder fault in the fault scenario and the clear fault
 * code otherwise.
 */
motion_fault_t motion_get_fault (void)
{
    return (unsigned)FAULT == g_scenario ? MOTION_FAULT_ENCODER
                                         : MOTION_FAULT_NONE;
}

/** @brief Report that the simulated motion controller already has
 * calibration.
 */
bool motion_is_calibrated (void)
{
    return true;
}

/** @brief Pack an idle, fault-free snapshot with both speeds zero. The
 * not-ready case sets the busy input.
 */
bool snapshot_motion_read (codec_motion_snapshot_t * p_out)
{
    return codec_motion_pack(p_out, MOTION_IDLE, MOTION_FAULT_NONE,
                             (unsigned)NOT_READY == g_scenario, true, 0, 0);
}

/** @brief Check the initial delay and the single 0.10 m drive request at
 * 0.05 m/s. Record its time even when the selected scenario returns
 * busy.
 */
motion_result_t motion_request_drive (float32_t distance, float32_t speed)
{
    assert(0.10f == distance && 0.05f == speed && 0U == g_requests);
    assert((uint32_t)(g_clock_value - g_base) >= 3680U);
    g_requests++;
    g_request_at = g_clock_value;
    return (unsigned)REJECT == g_scenario ? MOTION_ERR_BUSY : MOTION_OK;
}

/** @brief Return the injected fault or unexpected idle state when selected.
 * A normal drive completes after 400 ms. The timeout scenario stays
 * in drive.
 */
motion_state_t motion_poll (void)
{
    if ((unsigned)FAULT == g_scenario)
    {
        return MOTION_FAULT;
    }

    if ((unsigned)UNEXPECTED == g_scenario)
    {
        return MOTION_IDLE;
    }

    if ((g_scenario != (unsigned)TIMEOUT) &&
        ((uint32_t)(g_clock_value - g_request_at) >= 400U))
    {
        return MOTION_COMPLETE;
    }

    return MOTION_DRIVE;
}

/** @brief Run the motion smoke scenarios with simulated buttons, clocks and
 * requests. Check completion and failure paths without driving
 * hardware.
 */
int main (void)
{
    run(COMPLETE, "encoder target reached", true, 0U);
    run(COMPLETE, "encoder target reached", true, UINT32_MAX - 2000U);
    run(STOP_COUNTDOWN, "cancelled in countdown", false, 0U);
    run(STOP_RUN, "GP21 stop", true, 0U);
    run(TIMEOUT, "software timeout", true, 0U);
    run(FAULT, "Buddy 2 fault", true, 0U);
    run(REJECT, "rejected", true, 0U);
    run(STOP_PRIORITY, "cancelled before start", false, 0U);
    run(HELD_BOOT, "encoder target reached", true, 0U);
    run(NO_PRESS, "", false, 0U);
    run(HELD_ONLY, "", false, 0U);
    run(NOT_READY, "not ready", false, 0U);
    run(CLOCK_FAIL, "not ready", false, 0U);
    run(CLOCK_FAIL_RUN, "clock failed", true, 0U);
    run(WAIT_FAIL, "wait failed", true, 0U);
    run(UNEXPECTED, "unexpected motion state", true, 0U);
    run(GPIO_FAIL, "", false, 0U);
    run(PRIORITY_FAIL, "", false, 0U);
    puts("PASS: one-shot motion test: explicit start, held boot/no "
         "auto-start, stop priority/countdown/run, timeout, completion, "
         "faults, rejection, clock wrap/failure, wait/GPIO/priority failure");
}

/** @brief Start the requested smoke scenario at the supplied clock, execute
 * the task to the fixture cutoff and check the observed requests and
 * stops.
 */
static void run (unsigned kind, const char * p_reason, bool b_should_move,
                 uint32_t starting_clock)
{
    /* clang-format off */
    g_scenario  = kind;
    g_requests  = g_stops = 0U;
    g_used      = 0U;
    g_output[0] = '\0';
    g_base      = g_clock_value = starting_clock;
    /* clang-format on */
    int expired = setjmp(g_finished);

    if (!expired)
    {
        motion_smoke_run();
    }

    if (((unsigned)NO_PRESS == kind) || ((unsigned)HELD_ONLY == kind))
    {
        assert(expired && 0U == g_requests);
        return;
    }

    assert(!expired && g_requests == (b_should_move ? 1U : 0U));

    if (((unsigned)GPIO_FAIL == kind) || ((unsigned)PRIORITY_FAIL == kind))
    {
        assert(0U == g_requests);
        return;
    }

    assert(strstr(g_output, p_reason) && strstr(g_output, "FINISHED") &&
           g_stops >= 2U);

    if ((unsigned)TIMEOUT == kind)
    {
        assert((uint32_t)(g_clock_value - g_request_at) == 3050U);
    }
}

/* End of file. */

