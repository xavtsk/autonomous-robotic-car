/** @file sys.h
 * @brief Expose the fake lwIP clock interface used by MQTT host tests.
 */
#ifndef HOST_LWIP_SYS_H
#define HOST_LWIP_SYS_H
#include "lwip/apps/mqtt.h"
u32_t sys_now (void);
#endif

/* End of file. */

