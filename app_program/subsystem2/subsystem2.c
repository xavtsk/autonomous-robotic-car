/*
 * File: subsystem2.c
 * Owner: Buddy 2 - Motion control using motors and wheel encoders.
 * Purpose: RTOS lifecycle: the motion task and its 10 ms tick.
 *
 * A cyclic handler wakes the motion task every S2_PERIOD_MS. The task owns
 * the motor PWM and the encoder PIO, and runs motion_process() once
 * per wake-up. stop() also wakes it, so the motors brake at once.
 */

#include "subsystem2.h"
#include "motion.h"
#include "../diagnostics/robot_diagnostics.h"

static ID g_task_id = 0;
static ID g_cycle_id = 0;

static void subsystem2_tick(void *p_exinf)
{
    (void)p_exinf;
    if (g_task_id > 0)
    {
        (void)tk_wup_tsk(g_task_id);
    }
}

static void subsystem2_task(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    motion_init();                      /* motor PWM, encoder PIO           */
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

ER subsystem2_init(void)
{
    T_CCYC  ccyc = { 0 };
    int32_t er;

    er = motion_create();               /* mutex, report queue */
    if (er < (int32_t)E_OK)
    {
        return (ER)er;
    }

    ccyc.cycatr = TA_HLNG;              /* started in subsystem2_start() */
    ccyc.cychdr = (FP)subsystem2_tick;
    ccyc.cyctim = S2_PERIOD_MS;
    ccyc.cycphs = S2_PERIOD_MS;
    g_cycle_id = tk_cre_cyc(&ccyc);

    return (g_cycle_id < E_OK) ? (ER)g_cycle_id : E_OK;
}

ER subsystem2_start(void)
{
    g_task_id = app_start_task((FP)subsystem2_task, S2_PRI_MAIN,
                               S2_MOTION_CORE, "motion");
    if (g_task_id < E_OK)
    {
        return (ER)g_task_id;
    }
    motion_attach_task((int32_t)g_task_id);

    return tk_sta_cyc(g_cycle_id);
}
