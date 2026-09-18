/*
 * File: wifi_connection.h
 * Owner: Buddy 1 - WiFi Communication, Command and Telemetry
 * Purpose: Non-blocking WiFi association and connection recovery.
 */

#ifndef BUDDY1_WIFI_CONNECTION_H
#define BUDDY1_WIFI_CONNECTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    WIFI_CONNECTION_BOOT = 0,
    WIFI_CONNECTION_JOINING,
    WIFI_CONNECTION_ONLINE,
    WIFI_CONNECTION_BACKOFF,
    WIFI_CONNECTION_STATE_COUNT
} wifi_connection_state_t;

/* Initialize the Pico W networking hardware. Call once before process(). */
bool wifi_connection_init(void);

/* Advance association/recovery without blocking. Call about every 10 ms. */
void wifi_connection_process(uint32_t now_ms);

bool wifi_connection_is_online(void);
wifi_connection_state_t wifi_connection_state(void);
const char * wifi_connection_state_name(wifi_connection_state_t state);
uint32_t wifi_connection_reconnect_count(void);

/* Returns false when RSSI is unavailable because the interface is offline. */
bool wifi_connection_get_rssi(int32_t * rssi_dbm);

#endif /* BUDDY1_WIFI_CONNECTION_H */
