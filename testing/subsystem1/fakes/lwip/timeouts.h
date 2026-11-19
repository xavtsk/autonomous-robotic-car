/** @file timeouts.h
 * @brief Declare fake lwIP timeout scheduling hooks for MQTT tests.
 */
#ifndef TESTING_SUBSYSTEM1_FAKES_LWIP_TIMEOUTS_H
#define TESTING_SUBSYSTEM1_FAKES_LWIP_TIMEOUTS_H

void sys_timeout (unsigned ms, void (*callback)(void *), void * arg);
void sys_untimeout (void (*callback)(void *), void * arg);

#endif /* TESTING_SUBSYSTEM1_FAKES_LWIP_TIMEOUTS_H */

/* End of file. */

