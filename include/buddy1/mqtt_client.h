/*
 * File: mqtt_client.h
 * Owner: Buddy 1 - WiFi Communication, Command and Telemetry
 * Purpose: MQTT session, publishing, subscription and bounded buffering.
 */

#ifndef BUDDY1_MQTT_CLIENT_H
#define BUDDY1_MQTT_CLIENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BUDDY1_MQTT_TOPIC_MAX    96u
#define BUDDY1_MQTT_PAYLOAD_MAX 512u

typedef enum
{
    BUDDY1_MQTT_WAIT_NETWORK = 0,
    BUDDY1_MQTT_CONNECTING,
    BUDDY1_MQTT_ONLINE,
    BUDDY1_MQTT_BACKOFF,
    BUDDY1_MQTT_STATE_COUNT
} buddy1_mqtt_state_t;

/* Call after wifi_connection_init(), because lwIP must already exist. */
bool buddy1_mqtt_init(void);

/* Advance the MQTT session. Never waits for the broker. */
void buddy1_mqtt_process(uint32_t now_ms, bool network_available);

bool buddy1_mqtt_is_online(void);
buddy1_mqtt_state_t buddy1_mqtt_state(void);
const char * buddy1_mqtt_state_name(buddy1_mqtt_state_t state);
uint32_t buddy1_mqtt_reconnect_count(void);
uint32_t buddy1_mqtt_command_drop_count(void);

/* Submit an ordinary publish. True means accepted by lwIP, not necessarily
 * already delivered across the radio. */
bool buddy1_mqtt_publish(const char * topic,
                         const char * payload,
                         uint8_t qos,
                         bool retain);

/* Retain one critical QoS 1 reply until its request completes. */
bool buddy1_mqtt_queue_reply(const char * topic, const char * payload);
bool buddy1_mqtt_reply_pending(void);

/* Copy one complete subscribed command into caller-owned storage. A command
 * remains pending while a previous critical reply owns the reply slot. */
bool buddy1_mqtt_take_command(char * topic_out,
                              size_t topic_size,
                              char * payload_out,
                              size_t payload_size,
                              bool * overflowed);

#endif /* BUDDY1_MQTT_CLIENT_H */
