/** @file mqtt_bridge.h
 * @brief Adapter between the kernel-owned MQTT port and Buddy 1 queues.
 *
 * The team-owned mqtt_utk.h declares the shared callback interface. Its public
 * names connect these strong implementations to the network port's weak hooks.
 */
#ifndef BUDDY1_MQTT_BRIDGE_H
#define BUDDY1_MQTT_BRIDGE_H

#if defined(TM_MQTT) && TM_MQTT
#include "buddy1.h"
#include "mqtt_utk.h"
#endif

#endif /* BUDDY1_MQTT_BRIDGE_H */

/* End of file. */

