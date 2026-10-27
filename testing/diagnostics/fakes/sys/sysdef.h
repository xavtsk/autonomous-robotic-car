#ifndef DIAGNOSTICS_TEST_SYSDEF_H
#define DIAGNOSTICS_TEST_SYSDEF_H
#include <stdint.h>
#define TIMER_TIMERAWL UINT32_C(0x40054028)
uint32_t diagnostics_test_in_w(uint32_t address);
#define in_w diagnostics_test_in_w
#endif
