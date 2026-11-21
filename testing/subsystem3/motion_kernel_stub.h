/** @file motion_kernel_stub.h
 * @brief Minimal kernel boundary for testing production motion.c on the host.
 */
#ifndef TEST_MOTION_KERNEL_H
#define TEST_MOTION_KERNEL_H
#define APP_H
#include <stddef.h> /* Supply NULL for motion, including the fixed baseline. */
#include <stdint.h>
typedef int32_t INT;
typedef int32_t SZ;
typedef int32_t TMO;

typedef struct
{
    unsigned int mbfatr;
    SZ           bufsz;
    INT          maxmsz;
} T_CMBF;

#define TA_TFIFO 0U
#define TMO_POL 0
#define RESETS_RESET 4U
#define RESETS_RESET_DONE 8U
#define RESETS_RESET_TIMER 1U
/* The shared timer uses this name from the imported board header. */
#define XOSC_MHz 12U

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
ID tk_cre_mbf (const T_CMBF * p_config);
ER tk_snd_mbf (ID h_message_buffer, const void * p_data, INT size, TMO timeout);
INT      tk_rcv_mbf (ID h_message_buffer, void * p_data, TMO timeout);
void     out_w (uint32_t address, uint32_t value);
void     clr_w (uint32_t address, uint32_t value);
ID       tk_cre_mtx (const T_CMTX * p_config);
ER       tk_loc_mtx (ID h_mutex, int timeout);
ER       tk_unl_mtx (ID h_mutex);
ER       tk_wup_tsk (ID h_task);
ER       tk_dly_tsk (uint32_t delay);
ER       tk_can_wup (ID h_task);
uint32_t in_w (uint32_t address);
#endif

/* End of file. */

