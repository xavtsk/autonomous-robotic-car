#ifndef INTEGRATION_RUNTIME_SYSDEF_H
#define INTEGRATION_RUNTIME_SYSDEF_H
#define TIMER_TIMERAWL 0U
uint32_t runtime_read_clock(uint32_t address);
#define in_w runtime_read_clock
#endif
