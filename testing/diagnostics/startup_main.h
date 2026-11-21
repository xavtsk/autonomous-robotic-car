/** @file startup_main.h
 * @brief RTOS types used by the startup-order fixture.
 *
 * The fake tasks and subsystem entry points record startup events for the
 * application entry point linked with this fixture.
 */
#ifndef TESTING_DIAGNOSTICS_STARTUP_MAIN_H
#define TESTING_DIAGNOSTICS_STARTUP_MAIN_H

#include "diag_kernel.h"

#define APP_TEST 0
#define APP_IR_BENCH 0
#define TK_SUPPORT_SMP 1
#define TASK_STACK_SIZE 2048U
#define TA_HLNG 1U
#define TA_RNG3 2U
#define TA_ASSPRC 4U
#define TP_PRC1 1U
#define TSK_SELF 0
#define TMO_FEVR (-1)
typedef int      PRI;
typedef uint32_t UW;
typedef void     ( * FP)(void);

typedef struct
{
    FP  task;
    PRI itskpri;
    UW  stksz;
    UW  tskatr;
    UW  assprc;
} T_CTSK;

typedef struct
{
    unsigned spver;
    unsigned prver;
} T_RVER;

ER  subsystem1_init (void);
ER  subsystem2_init (void);
ER  subsystem3_init (void);
ER  subsystem4_init (void);
ER  subsystem5_init (void);
ER  vehicle_controller_init (void);
ER  subsystem1_start (void);
ER  subsystem2_start (void);
ER  subsystem3_start (void);
ER  subsystem4_start (void);
ER  subsystem5_start (void);
ER  vehicle_controller_start (void);
ER  tk_chg_pri (ID h_task, PRI priority);
ER  tk_dly_tsk (UW milliseconds);
ER  tk_slp_tsk (TMO timeout);
ER  tk_ref_ver (T_RVER * p_version);
INT tk_get_prc (void);
ID  tk_cre_tsk (const T_CTSK * p_task_entry);
ER  tk_sta_tsk (ID h_task, INT value);
ER  tk_del_tsk (ID h_task);

/* Record startup calls without enabling a real diagnostics owner. */
ER   robot_diagnostics_init (void);
void robot_diagnostics_report (void);

#define CHECK(expression, name)                                                \
    do                                                                         \
    {                                                                          \
        ER e_ = (expression);                                                  \
        if (e_ < E_OK)                                                         \
        {                                                                      \
            tm_printf((const UB *)"ERROR %s: %d\n", (name), e_);               \
        }                                                                      \
    } while (0)

#endif /* TESTING_DIAGNOSTICS_STARTUP_MAIN_H */

/* End of file. */

