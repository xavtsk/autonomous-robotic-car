/* Host-only declarations; production keeps the real micro T-Kernel headers. */
#ifndef DIAGNOSTICS_TEST_KERNEL_H
#define DIAGNOSTICS_TEST_KERNEL_H
#include <stdint.h>
typedef int ID, ER, INT, TMO;
typedef unsigned char UB;
typedef struct { unsigned mtxatr; } T_CMTX;
#define E_OK 0
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define TA_INHERIT 1U
#define TMO_POL 0
ID tk_cre_mtx(const T_CMTX *value);
ER tk_loc_mtx(ID id, TMO timeout);
ER tk_unl_mtx(ID id);
INT tm_printf(const UB *format, ...);
#endif
