/** @file host_main.h
 * @brief MQTT client callbacks implemented by the network fixture.
 *
 * The fake client records publication and dispatches acknowledgements and
 * received fragments through the lwIP callback types.
 */
#ifndef TESTING_SUBSYSTEM1_HOST_MAIN_H
#define TESTING_SUBSYSTEM1_HOST_MAIN_H

#include "host_kernel.h"
#include "lwip/apps/mqtt.h"

/* Clock and timeout services supplied to the fake MQTT client. */
u32_t sys_now (void);
void  sys_timeout (unsigned delay_ms, void ( * p_callback)(void * p_context),
                   void * p_arg);
void  sys_untimeout (void ( * p_callback)(void * p_context), void * p_arg);

int host_main_run (void);
#ifdef TEST_MQTT_RECEIVE
ID   host_main_command_buffer (void);
void host_main_receive (uint32_t announced, uint16_t actual, uint16_t split,
                        const uint8_t * p_bytes);
#endif

#endif /* TESTING_SUBSYSTEM1_HOST_MAIN_H */

/* End of file. */

