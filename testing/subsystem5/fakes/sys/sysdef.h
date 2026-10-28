#ifndef S5_FAKE_SYSDEF_H
#define S5_FAKE_SYSDEF_H
#include <stdint.h>
#define TIMER_TIMERAWL 0x40054028U
#define RESETS_RESET 0x4000c000U
#define RESETS_RESET_DONE 0x4000c008U
#define RESETS_RESET_PIO0 (1U<<10)
#define RESETS_RESET_PWM (1U<<14)
#define PWM_BASE 0x40050000U
#define PWM_CHx_DIV 4U
#define SYSCLK 125U
uint32_t fake_read(uint32_t address);
void fake_write(uint32_t address,uint32_t value);
#define in_w fake_read
#define out_w fake_write
#define clr_w(a,v) fake_write((a),fake_read(a)&~(v))
#endif
