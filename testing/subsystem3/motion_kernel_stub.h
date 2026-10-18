/* Minimal kernel boundary for testing production motion.c on the host. */
#ifndef TEST_MOTION_KERNEL_H
#define TEST_MOTION_KERNEL_H
#define APP_H
#include <stdint.h>
typedef int32_t ID;
typedef int32_t ER;
typedef struct
{
    unsigned int mtxatr;
} T_CMTX;
#define E_OK 0
#define TMO_FEVR (-1)
#define TA_INHERIT 1U
#define TSK_SELF 0
#define TIMER_TIMERAWL 0U
#define TP_PRC1 1U
ID tk_cre_mtx(const T_CMTX *config);
ER tk_loc_mtx(ID id, int timeout);
ER tk_unl_mtx(ID id);
ER tk_wup_tsk(ID id);
ER tk_dly_tsk(uint32_t delay);
ER tk_can_wup(ID id);
uint32_t in_w(uint32_t address);
#endif
