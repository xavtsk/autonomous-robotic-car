/* Host-only model of the small RTOS surface used by Buddy 1. Build with
 * -DAPP_H -include host_kernel.h; production builds use the real kernel. */
#ifndef BUDDY1_HOST_KERNEL_H
#define BUDDY1_HOST_KERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef unsigned char UB;
typedef unsigned short UH;
typedef short H;
typedef uint32_t UW;
typedef int INT;
typedef unsigned int UINT, BOOL;
typedef int ID, ER, PRI, TMO;
#define IMPORT extern
#define EXPORT
#define LOCAL static
#define TRUE 1U
#define FALSE 0U
#define E_OK 0
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_SYS (-5)
#define E_TMOUT (-50)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_TFIFO 0
#define TA_INHERIT 1
typedef struct { unsigned mbfatr; int bufsz, maxmsz; } T_CMBF;
typedef struct { unsigned mtxatr; } T_CMTX;
ID tk_cre_mbf(const T_CMBF *p);
ER tk_del_mbf(ID id);
ID tk_cre_mtx(const T_CMTX *p);
ER tk_snd_mbf(ID id, const void *data, INT length, TMO timeout);
INT tk_rcv_mbf(ID id, void *data, TMO timeout);
ER tk_loc_mtx(ID id, TMO timeout);
ER tk_unl_mtx(ID id);
ID app_start_task(void (*entry)(INT, void *), PRI priority, UW core, const char *name);
int tm_printf(const UB *format, ...);
ER subsystem1_init(void);
void subsystem1_test(void);
#endif
