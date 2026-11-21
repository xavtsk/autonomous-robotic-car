/** @file buddy2.c
 * @brief RTOS lifecycle: the motion task and its 10 ms tick.
 *
 * @par Ownership
 * Buddy 2 - Motion control using motors and wheel encoders.
 *
 * A cyclic handler wakes the motion task every S2_PERIOD_MS. The task owns
 * the motor PWM and the encoder PIO, and runs motion_process() once
 * per wake-up. stop() also wakes it, so the motors brake at once.
 */

#include "buddy2.h"
#include "motion.h"
#include "../diagnostics/robot_diagnostics.h"

static ID gh_motion_task  = 0;
static ID gh_motion_cycle = 0;

static void subsystem2_tick (void * p_exinf);
static void subsystem2_task (INT stacd, void * p_exinf);

/** @brief Create the motion mutex and queue, then the 10 ms cyclic wakeup.
 * Return the first creation error, or E_OK.
 */
ER subsystem2_init (void)
{
    T_CCYC ccyc = {0};

    const int32_t value_er =
        motion_create(); /* Create the mutex and report queue. */
    if (value_er < E_OK)
    {
        return (ER)value_er;
    }

    /* T_CCYC uses the kernel FP address type. The cyclic handler receives
     * one void * extended-information argument. See tk/syscall.h. */
    ccyc.cycatr     = TA_HLNG; /* Start the task in subsystem2_start(). */
    ccyc.cychdr     = (FP)subsystem2_tick;
    ccyc.cyctim     = S2_PERIOD_MS;
    ccyc.cycphs     = S2_PERIOD_MS;
    gh_motion_cycle = tk_cre_cyc(&ccyc);

    return (gh_motion_cycle < E_OK) ? gh_motion_cycle : E_OK;
}

/** @brief Start the motion task on its configured core, attach the task handle
 * and enable cyclic wakeups. Return the task or timer startup result.
 */
ER subsystem2_start (void)
{
    gh_motion_task = app_start_task((FP)subsystem2_task, S2_PRI_MAIN,
                                    S2_MOTION_CORE, "motion");

    if (gh_motion_task < E_OK)
    {
        return gh_motion_task;
    }

    motion_attach_task((int32_t)gh_motion_task);

    return tk_sta_cyc(gh_motion_cycle);
}

/** @brief Wake the motion task on a cyclic timer tick if its handle is valid.
 * The callback argument is unused.
 */
static void subsystem2_tick (void * p_exinf)
{
    (void)p_exinf;

    if (gh_motion_task > 0)
    {
        (void)tk_wup_tsk(gh_motion_task);
    }
}

/** @brief Initialize the motion hardware, then service one motion update per
 * wake. The optional diagnostics record the execution interval in
 * microseconds. Kernel entry arguments are unused.
 */
static void subsystem2_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    motion_init(); /* Initialize motor PWM and encoder PIO. */
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
#if APP_DIAGNOSTICS
        uint32_t started_us = robot_diagnostics_now_us();
#endif
        motion_process();
#if APP_DIAGNOSTICS
        robot_diagnostics_record(ROBOT_DIAG_MOTION, started_us,
                                 robot_diagnostics_now_us());
#endif
    }
}

/* End of file. */

