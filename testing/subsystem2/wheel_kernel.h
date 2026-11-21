/** @file wheel_kernel.h
 * @brief Host-only MMIO/RTOS boundary.
 *
 * Firmware still includes the real app.h.
 */
#ifndef BUDDY2_WHEEL_KERNEL_H
#define BUDDY2_WHEEL_KERNEL_H
#include <stdint.h>
#include <stddef.h> /* Supply NULL for motion sources built with this shim. */
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

typedef struct
{
    unsigned intatr;
    FP       inthdr;
} T_DINT;

#define IMPORT extern
#define TRUE 1U
#define FALSE 0U
#define E_OK 0
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define TA_INHERIT 1U
#define TA_HLNG 0U
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define TP_PRC1 1U
#define SYSCLK 125U
#define TIMER_TIMERAWL 0x40054028U
#define IO_BANK0_BASE 0x40014000U
#define SIO_BASE 0xd0000000U
#define GPIO_IN (SIO_BASE + 4U)
#define GPIO_MODE_IN 0U
#define GPIO_PDE 4U
#define GPIO_PUE 8U
#define GPIO(pin) (0x4001c004U + (pin) * 4U)
#define PWM_BASE 0x40050000U
/* The motor driver uses the imported RP2040 name for this 0x04 offset. */
#define PWM_CHx_DIV 4U
ID  tk_cre_mtx (const T_CMTX * p_spec);
ER  tk_loc_mtx (ID h_mutex, TMO timeout);
ER  tk_unl_mtx (ID h_mutex);
ER  tk_def_int (UINT intno, const T_DINT * p_spec);
ER  tk_dly_tsk (INT delay_ms);
ER  tk_wup_tsk (ID h_task);
ER  tk_slp_tsk (TMO timeout);
ID  app_start_task (FP p_entry, PRI priority, UW core, const char * p_name);
INT tk_can_wup (ID h_task);
uint32_t in_w (uint32_t address);
void     out_w (uint32_t address, uint32_t value);
void     clr_w (uint32_t address, uint32_t mask);
void     set_w (uint32_t address, uint32_t mask);
void     EnableInt (UINT intno, INT priority);
int      gpio_set_pin (uint32_t pin, uint32_t mode);
int      pwm_set_pin (uint32_t pin);
int      pwm_set_wrap (uint32_t pin, uint32_t wrap);
int      pwm_set_cc (uint32_t pin, uint32_t value);
int      pwm_set_enabled (uint32_t pin, BOOL enabled);
int      tm_printf (const UB * p_format, ...);
#endif

/* End of file. */

