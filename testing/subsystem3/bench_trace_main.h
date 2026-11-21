/** @file bench_trace_main.h
 * @brief Acquisition interface for the stationary IR bench trace.
 *
 * The runner supplies sensor initialization, frame reads and close while
 * controlling the number of sampling iterations.
 */
#ifndef TESTING_SUBSYSTEM3_BENCH_TRACE_MAIN_H
#define TESTING_SUBSYSTEM3_BENCH_TRACE_MAIN_H

#include <stdint.h>

#include "ir_acquisition.h"

typedef int      ID;
typedef int      INT;
typedef int      ER;
typedef int      PRI;
typedef unsigned UW;
typedef uint8_t  UB;
typedef void     ( * FP)(void);

typedef struct
{
    int mtxatr;
} T_CMTX;

typedef struct
{
    int      cycatr;
    FP       cychdr;
    unsigned cyctim;
    unsigned cycphs;
} T_CCYC;

#define E_OK 0
#define E_OBJ (-41)
#define E_PAR (-17)
#define E_NOSPT (-9)
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define TA_INHERIT 1
#define TA_HLNG 2
#define TP_PRC1 1U

ER tk_loc_mtx (ID h_mutex, int timeout);
ER tk_unl_mtx (ID h_mutex);
ID tk_cre_mtx (const T_CMTX * p_mutex_spec);
ID tk_cre_cyc (const T_CCYC * p_cycle_spec);
ER tk_sta_cyc (ID h_cycle);
ER tk_stp_cyc (ID h_cycle);
ER tk_del_cyc (ID h_cycle);
ER tk_del_mtx (ID h_mutex);
ER tk_ter_tsk (ID h_task);
ER tk_del_tsk (ID h_task);
ER tk_wup_tsk (ID h_task);
ER tk_can_wup (ID h_task);
ER tk_slp_tsk (int timeout);
ER tk_dly_tsk (unsigned duration);
ID app_start_task (FP p_entry, PRI priority, UW processor, const char * p_name);
int tm_printf (const UB * p_format, ...);
ER  subsystem3_init (void);
ER  subsystem3_start (void);

#endif /* TESTING_SUBSYSTEM3_BENCH_TRACE_MAIN_H */

/* End of file. */

