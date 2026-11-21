/** @file timeouts.h
 * @brief Declare fake lwIP timeout scheduling hooks for MQTT tests.
 */
#ifndef TESTING_SUBSYSTEM1_FAKES_LWIP_TIMEOUTS_H
#define TESTING_SUBSYSTEM1_FAKES_LWIP_TIMEOUTS_H

void sys_timeout (unsigned ms, void ( * p_callback)(void * p_context),
                  void * p_argument);
void sys_untimeout (void ( * p_callback)(void * p_context), void * p_argument);

#endif /* TESTING_SUBSYSTEM1_FAKES_LWIP_TIMEOUTS_H */

/* End of file. */

