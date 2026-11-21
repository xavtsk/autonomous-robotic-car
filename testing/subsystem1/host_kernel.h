/** @file host_kernel.h
 * @brief Host-only model of the small RTOS surface used by Buddy 1.
 *
 * Build with
 * -DAPP_H -include host_kernel.h; production builds use the real kernel.
 */
#ifndef BUDDY1_HOST_KERNEL_H
#define BUDDY1_HOST_KERNEL_H
#include <stdint.h>
#include <stddef.h> /* Supply NULL for command and MQTT bridge sources. */
typedef uint8_t      UB;
typedef uint16_t     UH;
typedef int16_t      H;
typedef uint32_t     UW;
typedef int          INT;
typedef unsigned int UINT;
typedef unsigned int BOOL;
typedef int          ID;
typedef int          ER;
typedef int          PRI;
typedef int          TMO;
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

typedef struct
{
    unsigned mbfatr;
    int      bufsz;
    int      maxmsz;
} T_CMBF;

typedef struct
{
    unsigned mtxatr;
} T_CMTX;

typedef struct
{
    void * exinf;
    ID     wtsk;
    ID     stsk;
    INT    msgsz;
    int    frbufsz;
    INT    maxmsz;
} T_RMBF;

ID   tk_cre_mbf (const T_CMBF * p_message_buffer_config);
ER   tk_del_mbf (ID h_message_buffer);
ID   tk_cre_mtx (const T_CMTX * p_mutex_config);
ER   tk_snd_mbf (ID h_message_buffer, const void * p_data, INT length,
                 TMO timeout);
INT  tk_rcv_mbf (ID h_message_buffer, void * p_data, TMO timeout);
ER   tk_ref_mbf (ID h_message_buffer, T_RMBF * p_reference);
ER   tk_dly_tsk (TMO time);
ER   tk_loc_mtx (ID h_mutex, TMO timeout);
ER   tk_unl_mtx (ID h_mutex);
ID   app_start_task (void ( * p_entry)(INT start_code, void * p_extra_info),
                     PRI priority, UW core, const char * p_name);
int  tm_printf (const UB * p_format, ...);
ER   subsystem1_init (void);
void subsystem1_test (void);
#endif

/* End of file. */

