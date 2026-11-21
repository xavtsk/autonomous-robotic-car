/** @file app.h
 * @brief Expose shared RTOS definitions and subsystem lifecycle contracts.
 */
/*
 *  app.h
 *  Definitions shared by the whole team.  Changed only by team agreement.
 *
 *  Anything that belongs to ONE subsystem goes in that subsystem's own
 *  header instead (app_program/subsystemN/buddyN.h).
 */

#ifndef APP_H
#define APP_H

#include "system_config.h"

#include <tk/tkernel.h>  /* Use the micro T-Kernel service API. */
#include <tm/tmonitor.h> /* Use T-Monitor for console output. */
#include <bsp/libbsp.h>  /* Board helpers: gpio_set_pin/val, pwm_... */

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task.  Each subsystem
 *  has its own band (TEAM.md section 4), defined in its own header.
 * --------------------------------------------------------------------- */
#define TASK_STACK_SIZE 2048 /* Reserve about 1 KB for tm_printf. */

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
extern ID app_start_task (FP p_entry, PRI pri, UW prc, const char * p_name);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
extern ER subsystem1_init (void);
extern ER subsystem2_init (void);
extern ER subsystem3_init (void);
extern ER subsystem4_init (void);
extern ER subsystem5_init (void);
extern ER vehicle_controller_init (void);

extern ER subsystem1_start (void);
extern ER subsystem2_start (void);
extern ER subsystem3_start (void);
extern ER subsystem4_start (void);
extern ER subsystem5_start (void);
extern ER vehicle_controller_start (void);

#if APP_TEST
extern void subsystem1_test (void);
extern void subsystem2_test (void);
extern void subsystem3_test (void);
extern void subsystem4_test (void);
extern void subsystem5_test (void);
#endif

#endif /* APP_H */

/* End of file. */

