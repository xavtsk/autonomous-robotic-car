/* Host-only RTOS declarations for the real controller integration test. */
#ifndef CONTROLLER_TEST_KERNEL_H
#define CONTROLLER_TEST_KERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef unsigned char UB;
typedef unsigned short UH;
typedef short H;
typedef uint32_t UW;
typedef int INT, ID, ER, PRI, TMO;
typedef unsigned int UINT, BOOL;
typedef void (*FP)(void);
typedef struct { unsigned mtxatr; } T_CMTX;
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
#define TP_PRC1 1U
ID tk_cre_mtx(const T_CMTX *value);
ER tk_loc_mtx(ID id, TMO timeout);
ER tk_unl_mtx(ID id);
ER tk_wup_tsk(ID id);
ER tk_slp_tsk(TMO timeout);
ID app_start_task(FP entry, PRI priority, UW core, const char *name);
#endif
