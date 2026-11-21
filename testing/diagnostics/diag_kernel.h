/** @file diag_kernel.h
 * @brief Host-only declarations; production keeps the real micro T-Kernel
 * headers.
 */
#ifndef DIAGNOSTICS_TEST_KERNEL_H
#define DIAGNOSTICS_TEST_KERNEL_H
#include <stdint.h>
typedef int     ID;
typedef int     ER;
typedef int     INT;
typedef int     TMO;
typedef uint8_t UB;

typedef struct
{
    unsigned mtxatr;
} T_CMTX;

#define E_OK 0
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define TA_INHERIT 1U
#define TMO_POL 0
ID  tk_cre_mtx (const T_CMTX * p_value);
ER  tk_loc_mtx (ID h_mutex, TMO timeout);
ER  tk_unl_mtx (ID h_mutex);
INT tm_printf (const UB * p_format, ...);
#endif

/* End of file. */

