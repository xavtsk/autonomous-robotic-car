/** @file app_main.c
 * @brief Application entry point, adapted from INF2004_ProjectTemplate.
 * @see docs/TEMPLATE_MIGRATION.md for the original project and revisions.
 *
 * The kernel starts usermain() in its initial task. Initialize all subsystem
 * objects before starting any tasks, since the second core may run immediately.
 * IR_BENCH builds start only subsystem 3 for stationary sensor setup.
 */
#include "app_main.h"
#include "diagnostics/robot_diagnostics.h"
#include "diagnostics/smoke_motion.h"
#include "controller/op_panel.h"

#if APP_DIAGNOSTICS
#if APP_TEST || APP_IR_BENCH
#error "Diagnostics must use the normal TEST=0 IR_BENCH=0 profile"
#endif
#include "subsystem2/buddy2.h"
#include "subsystem3/buddy3.h"
#define ROBOT_DIAG_REPORT_PRIORITY (23)
#define ROBOT_DIAG_REPORT_MS (1000U)
#endif

static inline void app_log_failure (INT result, const char * p_name);

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
/** @brief Create and start a kernel task with the supplied entry point,
 * priority and optional core affinity. Use the name for error
 * reporting. Return its handle on success. Delete a newly created task
 * if startup fails.
 */
ID app_start_task (FP p_entry, PRI pri, UW prc, const char * p_name)
{
    T_CTSK ctsk = {0};
    ID     h_task;
    ER     value_er;

    ctsk.task    = p_entry;         /* Set the task entry point. */
    ctsk.itskpri = pri;             /* Priority one is highest. */
    ctsk.stksz   = TASK_STACK_SIZE; /* The kernel allocates the stack. */
    ctsk.tskatr  = TA_HLNG          /* Use the high-level task entry. */
                  | TA_RNG3;        /* Run at user protection level. */
#if TK_SUPPORT_SMP
    if (prc != 0U)
    {
        ctsk.tskatr |= TA_ASSPRC; /* Use the configured core affinity. */
        ctsk.assprc  = prc;       /* Select the allowed processor set. */
    }
#else
    (void)prc; /* Single-core builds omit affinity. */
#endif

    /* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
    h_task = tk_cre_tsk(&ctsk);

    if (h_task < E_OK)
    {
        app_log_failure(h_task, p_name);
        return h_task;
    }

    /* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
    value_er = tk_sta_tsk(h_task, 0);
    app_log_failure(value_er, p_name);

    if (value_er < E_OK)
    {
        (void)tk_del_tsk(h_task);
        return (ID)value_er;
    }

    return h_task;
}

/** @brief Start the configured subsystem profile in the kernel startup task.
 * @details The integrated profile stops its startup sequence on the first
 * error. The compatibility profile calls every initialization hook before
 * calling its start hooks and logs each failure. The IR bench uses its own
 * startup path. When enabled, diagnostics reuse this task at a lower priority
 * to observe subsystem timing. The initial task otherwise sleeps forever.
 */
INT usermain (void)
{
    T_RVER ver;
#if APP_DIAGNOSTICS
    ER diagnostic_status;
#endif

    /* Read the kernel version for the boot report. */
    tk_ref_ver(&ver);
    tm_printf((UB *)"\n=== micro T-Kernel on Raspberry Pi Pico W"
                    " (spec 0x%04x, product 0x%04x) ===\n",
              ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    tm_printf((UB *)"dual-core (SMP) build, usermain on core %d\n",
              tk_get_prc() - 1); /* Processor numbers start at one. */
#else
    tm_printf((UB *)"single-core build\n");
#endif

#if APP_DIAGNOSTICS
    /* Create before either timing producer starts. A diagnostic failure must
     * not prevent ordinary subsystem startup or change motor behaviour. */
    diagnostic_status = robot_diagnostics_init();
    app_log_failure(diagnostic_status, "robot_diagnostics_init");
#endif

#if APP_S4_BENCH
    {
        ER er = subsystem4_init();
        if (E_OK == er)
        {
            er = subsystem4_start();
        }
        app_log_failure(er, "S4 bench startup");
        tm_printf((UB *)"S4 BENCH: keep board level/still for calibration; "
                        "other application tasks disabled.\n");
    }
#elif APP_MOTION_SMOKE
    /* No Vehicle Controller, IR, networking or other application tasks. */
    {
        ER er = subsystem2_init();
        app_log_failure(er, "smoke subsystem2_init");
        if (E_OK == er)
        {
            er = subsystem2_start();
            app_log_failure(er, "smoke subsystem2_start");
            if (E_OK == er)
            {
                motion_smoke_run();
            }
        }
    }
#elif APP_IR_BENCH
    /* Isolated bench profile: never initialise/start the motion subsystem. */
    {
        ER er = subsystem3_init();
        app_log_failure(er, "subsystem3_init");
        if (E_OK == er)
        {
            app_log_failure(subsystem3_start(), "subsystem3_start");
        }
    }
    tm_printf((UB *)"IR BENCH: other application tasks and tests disabled.\n");
#elif APP_S3_MOTION
    /* Explicit team profile: one upstream controller owner, Buddy 1 mirror. */
    {
        ER value_er = subsystem1_init();
        app_log_failure(value_er, "telemetry init");

        if (E_OK == value_er)
        {
            value_er = vehicle_controller_init();
        }

        if (E_OK == value_er)
        {
            value_er = subsystem2_init();
        }

        app_log_failure(value_er, "motion init");

        if (E_OK == value_er)
        {
            value_er = subsystem3_init();
            app_log_failure(value_er, "integration init");
        }
#if APP_S4
        if (E_OK == value_er)
        {
            value_er = subsystem4_init();
            app_log_failure(value_er, "IMU init");
        }
#endif
#if APP_S5
        if (E_OK == value_er)
        {
            value_er = subsystem5_init();
            app_log_failure(value_er,
                            "obstacle init (confirm hardware configuration)");
        }
#endif
        if (E_OK == value_er)
        {
            value_er = subsystem1_start();
            app_log_failure(value_er, "telemetry start");
        }

        if (E_OK == value_er)
        {
            value_er = subsystem2_start();
            app_log_failure(value_er, "motion start");
        }

        if (E_OK == value_er)
        {
            value_er = subsystem3_start();
            app_log_failure(value_er, "integration start");
        }
#if APP_S4
        if (E_OK == value_er)
        {
            value_er = subsystem4_start();
            app_log_failure(value_er, "IMU start");
        }
#endif
#if APP_S5
        if (E_OK == value_er)
        {
            value_er = subsystem5_start();
            app_log_failure(value_er, "obstacle start");
        }
#endif
        if (E_OK == value_er)
        {
            value_er = op_panel_start();
            app_log_failure(value_er, "operator panel start");
        }

        tm_printf((UB *)"Integrated controller: DISARMED; calibrate with GP20, "
                        "then GP20 = START, GP21 = STOP. status=%d\n",
                  (int)value_er);
    }
#else
    /* Create all subsystem kernel objects before starting their tasks. */
    app_log_failure(subsystem1_init(), "subsystem1_init");
    app_log_failure(subsystem2_init(), "subsystem2_init");
    app_log_failure(subsystem3_init(), "subsystem3_init");
    app_log_failure(subsystem4_init(), "subsystem4_init");
    app_log_failure(subsystem5_init(), "subsystem5_init");
    app_log_failure(vehicle_controller_init(), "vehicle_controller_init");

    /* Compatibility facade has no task; navigation is opt-in S3_MOTION=1. */
    app_log_failure(subsystem1_start(), "subsystem1_start");
    app_log_failure(subsystem2_start(), "subsystem2_start");
    app_log_failure(vehicle_controller_start(), "vehicle_controller_start");
    app_log_failure(subsystem3_start(), "subsystem3_start");
    app_log_failure(subsystem4_start(), "subsystem4_start");
    app_log_failure(subsystem5_start(), "subsystem5_start");

    tm_printf((UB *)"All subsystem startup hooks called.\n\n");

#if APP_TEST
    /* Test build (make TEST=1): run every subsystem's tests in turn,
       here in the initial task, while the system is running.  Tests may
       call tk_dly_tsk() to let the other tasks work for a while. */
    tm_printf((UB *)"[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    tm_printf((UB *)"[TEST] === test run end ===\n");
#endif
#endif /* APP_IR_BENCH */

#if APP_DIAGNOSTICS
    if (E_OK == diagnostic_status)
    {
        /* Reuse this otherwise-sleeping initial task, not an extra task/stack.
         * Never print periodic diagnostics at its initial priority of 1. */
        ER error = tk_chg_pri(TSK_SELF, ROBOT_DIAG_REPORT_PRIORITY);
        app_log_failure(error, "diagnostic reporter priority");
        if (E_OK == error)
        {
            tm_printf((UB *)"[DIAG] mode=observation_only motion_period_ms=%u "
                            "ir_period_ms=%u reporter_priority=%u\n",
                      (unsigned)S2_PERIOD_MS, (unsigned)S3_SAMPLE_MS,
                      (unsigned)ROBOT_DIAG_REPORT_PRIORITY);
            tm_printf(
                (UB *)"[DIAG] no automatic drive start; USB output can affect "
                      "timing; stack/heap high-water not measured\n");
            for (;;)
            {
                (void)tk_dly_tsk(ROBOT_DIAG_REPORT_MS);
                robot_diagnostics_report();
            }
        }
    }
#endif

    /* Sleep forever.  Returning from usermain() would shut down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/** @brief Print the startup result only when a kernel call fails. */
static inline void app_log_failure (INT result, const char * p_name)
{
    if (result < E_OK)
    {
        tm_printf((UB *)"ERROR %s: %d\n", p_name, result);
    }
}

/* End of file. */

