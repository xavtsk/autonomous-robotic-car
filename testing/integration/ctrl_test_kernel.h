/** @file ctrl_test_kernel.h
 * @brief Host-only RTOS declarations for the real controller integration test.
 */
#ifndef CONTROLLER_TEST_KERNEL_H
#define CONTROLLER_TEST_KERNEL_H
#include <stdint.h>
#include <stddef.h> /* Supply NULL for controller code built with this shim. */
typedef uint8_t      UB;
typedef uint16_t     UH;
typedef int16_t      H;
typedef uint32_t     UW;
typedef int          INT;
typedef int          ID;
typedef int          ER;
typedef int          PRI;
typedef int          TMO;
typedef unsigned int UINT;
typedef unsigned int BOOL;
typedef void         ( * FP)(void);

typedef struct
{
    unsigned mtxatr;
} T_CMTX;

#define IMPORT extern
#define TRUE 1U
#define FALSE 0U
#define E_OK 0
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define E_NOSPT (-9)
#define TA_INHERIT 1U
#define TMO_FEVR (-1)
#define TMO_POL 0
#define TP_PRC1 1U
ID tk_cre_mtx (const T_CMTX * p_value);
ER tk_loc_mtx (ID h_mutex, TMO timeout);
ER tk_unl_mtx (ID h_mutex);
ER tk_wup_tsk (ID h_task);
ER tk_slp_tsk (TMO timeout);
ID app_start_task (FP p_entry, PRI priority, UW core, const char * p_name);

/* Boundaries implemented by the fixtures that drive periodic sampling. */
ER  tk_sta_cyc (ID h_cycle);
ER  tk_stp_cyc (ID h_cycle);
ER  tk_del_cyc (ID h_cycle);
ER  tk_del_tsk (ID h_task);
ER  tk_ter_tsk (ID h_task);
ER  tk_del_mtx (ID h_mutex);
ER  tk_can_wup (ID h_task);
ER  tk_dly_tsk (unsigned milliseconds);
int tm_printf (const UB * p_format, ...);
#endif

/* End of file. */

