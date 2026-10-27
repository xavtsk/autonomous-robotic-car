/*
 * Application entry point, adapted from INF2004_ProjectTemplate.
 * The kernel starts usermain() in its initial task. Initialize all subsystem
 * objects before starting any tasks, since the second core may run immediately.
 * IR_BENCH builds start only subsystem 3 for stationary sensor setup.
 */
#include "app.h"
#include "diagnostics/robot_diagnostics.h"
#include "diagnostics/motion_smoke_test.h"
#include "controller/operator_panel.h"

#if APP_DIAGNOSTICS
#if APP_TEST || APP_IR_BENCH
#error "Diagnostics must use the normal TEST=0 IR_BENCH=0 profile"
#endif
#include "subsystem2/subsystem2.h"
#include "subsystem3/subsystem3.h"
#define ROBOT_DIAG_REPORT_PRIORITY (23)
#define ROBOT_DIAG_REPORT_MS (1000U)
#endif

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	if (er < E_OK) {
		(void)tk_del_tsk(tid);
		return (ID)er;
	}
	return tid;
}

EXPORT INT usermain(void)
{
	T_RVER	ver;
#if APP_DIAGNOSTICS
	ER diagnostic_status;
#endif

	/* tk_ref_ver: ask the kernel who it is */
	tk_ref_ver(&ver);
	tm_printf((UB*)"\n=== micro T-Kernel on Raspberry Pi Pico W"
		  " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
		  tk_get_prc() - 1);	/* tk_get_prc() counts from 1 */
#else
	tm_printf((UB*)"single-core build\n");
#endif

#if APP_DIAGNOSTICS
	/* Create before either timing producer starts. A diagnostic failure must
	 * not prevent ordinary subsystem startup or change motor behaviour. */
	diagnostic_status = robot_diagnostics_init();
	CHECK(diagnostic_status, "robot_diagnostics_init");
#endif

#if APP_MOTION_SMOKE
	/* No Vehicle Controller, IR, networking or other application tasks. */
	{
		ER er = subsystem2_init();
		CHECK(er, "smoke subsystem2_init");
		if (er == E_OK) {
			er = subsystem2_start();
			CHECK(er, "smoke subsystem2_start");
			if (er == E_OK) { motion_smoke_run(); }
		}
	}
#elif APP_IR_BENCH
	/* Isolated bench profile: never initialise/start the motion subsystem. */
	{
		ER er = subsystem3_init();
		CHECK(er, "subsystem3_init");
		if (er == E_OK) {
			CHECK(subsystem3_start(), "subsystem3_start");
		}
	}
	tm_printf((UB*)"IR BENCH: other application tasks and tests disabled.\n");
#elif APP_S3_MOTION
    /* Explicit team profile: one upstream controller owner, Buddy 1 mirror. */
    {
        ER er = subsystem1_init();
        CHECK(er, "telemetry init");
        if (er == E_OK) { er = vehicle_controller_init(); }
        if (er == E_OK) { er = subsystem2_init(); }
        CHECK(er, "motion init");
        if (er == E_OK)
        {
            er = subsystem3_init();
            CHECK(er, "integration init");
        }
        if (er == E_OK)
        {
            er = subsystem1_start();
            CHECK(er, "telemetry start");
        }
        if (er == E_OK)
        {
            er = subsystem2_start();
            CHECK(er, "motion start");
        }
        if (er == E_OK)
        {
            er = subsystem3_start();
            CHECK(er, "integration start");
        }
        if (er == E_OK)
        {
            er = operator_panel_start();
            CHECK(er, "operator panel start");
        }
        tm_printf((UB *)"Buddies 1-3 integration: DISARMED; calibrate with GP20, "
                        "then GP20 = START, GP21 = STOP. status=%d\n", (int)er);
    }
#else
	/* Phase 1: every subsystem creates its kernel objects */
	CHECK(subsystem1_init(), "subsystem1_init");
	CHECK(subsystem2_init(), "subsystem2_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem4_init(), "subsystem4_init");
	CHECK(subsystem5_init(), "subsystem5_init");
	CHECK(vehicle_controller_init(), "vehicle_controller_init");

	/* Compatibility facade has no task; navigation is opt-in S3_MOTION=1. */
	CHECK(subsystem1_start(), "subsystem1_start");
	CHECK(subsystem2_start(), "subsystem2_start");
	CHECK(vehicle_controller_start(), "vehicle_controller_start");
	CHECK(subsystem3_start(), "subsystem3_start");
	CHECK(subsystem4_start(), "subsystem4_start");
	CHECK(subsystem5_start(), "subsystem5_start");

	tm_printf((UB*)"All subsystem startup hooks called.\n\n");

#if APP_TEST
	/* Test build (make TEST=1): run every subsystem's tests in turn,
	   here in the initial task, while the system is running.  Tests may
	   call tk_dly_tsk() to let the other tasks work for a while. */
	tm_printf((UB*)"[TEST] === test run start ===\n");
	subsystem1_test();
	subsystem2_test();
	subsystem3_test();
	subsystem4_test();
	subsystem5_test();
	tm_printf((UB*)"[TEST] === test run end ===\n");
#endif
#endif /* APP_IR_BENCH */

#if APP_DIAGNOSTICS
	if (diagnostic_status == E_OK) {
		/* Reuse this otherwise-sleeping initial task, not an extra task/stack.
		 * Never print periodic diagnostics at its initial priority of 1. */
		ER error = tk_chg_pri(TSK_SELF, ROBOT_DIAG_REPORT_PRIORITY);
		CHECK(error, "diagnostic reporter priority");
		if (error == E_OK) {
			tm_printf((UB *)"[DIAG] mode=observation_only motion_period_ms=%u ir_period_ms=%u reporter_priority=%u\n",
				(unsigned)S2_PERIOD_MS, (unsigned)S3_SAMPLE_MS,
				(unsigned)ROBOT_DIAG_REPORT_PRIORITY);
			tm_printf((UB *)"[DIAG] no automatic drive start; USB output can affect timing; stack/heap high-water not measured\n");
			for (;;) {
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
