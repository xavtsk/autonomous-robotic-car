/** @file smoke_motion.c
 * @brief Run the separately enabled, manually armed hardware motion smoke test.
 */
/* Isolated, manually armed hardware test. No changes to Buddy 2 algorithms. */
#include "smoke_motion.h"
#if APP_MOTION_SMOKE
#include "../app.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/codec_motion.h"
#include <sys/sysdef.h>

#define START_PIN 20U
#define STOP_PIN 21U
#define POLL_MS 10U
#define RELEASE_MS 100U
#define DEBOUNCE_MS 30U
#define COUNTDOWN_MS 3000U
#define LIMIT_MS 3000U
#define TEST_DISTANCE_M 0.10f
#define TEST_SPEED_MPS 0.05f

/** @brief Read the low word of the kernel operating time in milliseconds.
 * Leave the caller's output untouched if the time request fails.
 */
static bool now_ms (uint32_t * p_out)
{
    SYSTIM time;
    if (tk_get_otm(&time) != E_OK)
    {
        return false;
    }
    *p_out = time.lo;
    return true;
}

/** @brief Read the selected active-low GPIO. Return true when the button is
 * pressed.
 */
static bool pressed (unsigned pin)
{
    return gpio_get_val(pin) == 0U;
}

/** @brief Delay one polling interval and report whether the kernel accepted
 * the delay.
 */
static bool pause_poll (void)
{
    return tk_dly_tsk(POLL_MS) == E_OK;
}

/** @brief Request a stop through the motion owner, delay for the final
 * readings and print the reason, encoder counts and fault.
 */
static void finish (const char * p_reason)
{
    /* Output changes stay with Buddy 2's motion task. No direct PWM writes. */
    stop();
    (void)tk_dly_tsk(50U);
    tm_printf((const UB *)"[SMOKE] %s; left=%d right=%d fault=%u\n", p_reason,
              (int)enc_l_count(), (int)enc_r_count(),
              (unsigned)motion_get_fault());
    tm_printf((const UB *)"[SMOKE] FINISHED: no automatic retry; power off "
                          "before repositioning.\n");
}

/** @brief Run a single manually armed 100 mm drive at 50 mm/s. Check
 * readiness, button release, countdown and stop conditions before and
 * during motion. Print the result without automatically starting a
 * second run.
 */
void motion_smoke_run (void)
{
    uint32_t                now;
    uint32_t                since;
    unsigned                released = 0U;
    unsigned                held     = 0U;
    unsigned                waiting  = 0U;
    codec_motion_snapshot_t snapshot;
    const char *            p_result = "software timeout";

    /* Initial task otherwise has priority 1. Let motion (14) run first. */
    if (tk_chg_pri(TSK_SELF, 15) != E_OK)
    {
        return;
    }
    if (gpio_set_pin(START_PIN, GPIO_MODE_IN) != E_OK ||
        gpio_set_pin(STOP_PIN, GPIO_MODE_IN) != E_OK)
    {
        return;
    }
    clr_w(GPIO(START_PIN), GPIO_PDE);
    clr_w(GPIO(STOP_PIN), GPIO_PDE);
    set_w(GPIO(START_PIN), GPIO_PUE);
    set_w(GPIO(STOP_PIN), GPIO_PUE);
    if (tk_dly_tsk(500U) != E_OK)
    {
        finish("startup delay failed");
        return;
    }
    stop();
    if (!now_ms(&now) || !motion_is_calibrated() ||
        !snapshot_motion_read(&snapshot) || !codec_motion_valid(&snapshot) ||
        codec_motion_state(&snapshot) != MOTION_IDLE ||
        codec_motion_fault(&snapshot) != MOTION_FAULT_NONE ||
        codec_motion_busy(&snapshot) || !codec_motion_speed_valid(&snapshot))
    {
        finish("not ready: no drive requested");
        return;
    }
    tm_printf((const UB *)"[SMOKE] TEST ONLY: 100 mm requested at 50 mm/s; "
                          "existing calibration is NOT physically verified.\n");
    tm_printf(
        (const UB *)"[SMOKE] Raise wheels for FIRST check. Keep power switch "
                    "reachable. Do not press M1/M2 test buttons.\n");
    tm_printf((const UB *)"[SMOKE] GP20=START; GP21=STOP. Release BOTH buttons "
                          "first. No movement on boot.\n");

    /* A held-at-boot start cannot arm. Stop wins if buttons overlap. */
    while (released < RELEASE_MS)
    {
        released = (!pressed(START_PIN) && !pressed(STOP_PIN))
                       ? released + POLL_MS
                       : 0U;
        if (!pause_poll())
        {
            finish("wait failed");
            return;
        }
    }
    tm_printf((const UB *)"[SMOKE] READY: press GP20 once. GP21 cancels this "
                          "attempt.\n");
    while (held < DEBOUNCE_MS)
    {
        if (pressed(STOP_PIN))
        {
            finish("cancelled before start");
            return;
        }
        held = pressed(START_PIN) ? held + POLL_MS : 0U;
        if (!pause_poll())
        {
            finish("wait failed");
            return;
        }
        waiting += POLL_MS;
        if (waiting >= 1000U)
        {
            tm_printf(
                (const UB *)"[SMOKE] READY: GP20 starts once; GP21 cancels.\n");
            waiting = 0U;
        }
    }
    tm_printf((const UB *)"[SMOKE] START accepted: 3 second countdown. GP21 "
                          "cancels.\n");
    if (!now_ms(&since))
    {
        finish("clock failed");
        return;
    }
    do
    {
        if (pressed(STOP_PIN))
        {
            finish("cancelled in countdown");
            return;
        }
        if (!pause_poll() || !now_ms(&now))
        {
            finish("clock/wait failed");
            return;
        }
    } while ((now - since) < COUNTDOWN_MS);

    if (pressed(STOP_PIN) || !now_ms(&since))
    {
        finish("cancelled at start");
        return;
    }
    if (motion_request_drive(TEST_DISTANCE_M, TEST_SPEED_MPS) != MOTION_OK)
    {
        finish("drive request rejected");
        return;
    }
    /* No printing in the running loop. Timeout uses kernel monotonic time,
     * not the separate hardware timestamp whose earlier readings disagreed.
     * This and GP21 are software stops, not an independent emergency cutoff. */
    for (;;)
    {
        motion_state_t state;
        if (pressed(STOP_PIN))
        {
            p_result = "GP21 stop requested";
            break;
        }
        if (!now_ms(&now))
        {
            p_result = "clock failed";
            break;
        }
        if ((now - since) >= LIMIT_MS)
        {
            break;
        }
        state = motion_poll();
        if (MOTION_COMPLETE == state)
        {
            p_result = "encoder target reached (not ruler-verified distance)";
            break;
        }
        if (MOTION_FAULT == state)
        {
            p_result = "Buddy 2 fault";
            break;
        }
        if (state != MOTION_DRIVE)
        {
            p_result = "unexpected motion state";
            break;
        }
        if (!pause_poll())
        {
            p_result = "wait failed";
            break;
        }
    }
    finish(p_result);
}
#else
/* ISO C requires a declaration even when this hardware test is disabled. */
typedef void smoke_motion_disabled_t;
#endif

/* End of file. */

