/** @file timeouts.h
 * @brief Declare fake lwIP timeout scheduling hooks for MQTT tests.
 */
void sys_timeout (unsigned ms, void (*callback)(void *), void * arg);
void sys_untimeout (void (*callback)(void *), void * arg);

/* End of file. */

