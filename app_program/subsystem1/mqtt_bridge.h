/** @file mqtt_bridge.h
 * @brief Adapter between the kernel-owned MQTT port and Buddy 1 queues.
 *
 * MQTT callback names and signatures are defined by the upstream port.
 */
#ifndef BUDDY1_MQTT_BRIDGE_H
#define BUDDY1_MQTT_BRIDGE_H

#if defined(TM_MQTT) && TM_MQTT
#include "buddy1.h"
#include "mqtt_utk.h"
#endif

#endif /* BUDDY1_MQTT_BRIDGE_H */
