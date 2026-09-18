/*
 * File: wifi_connection.c
 * Owner: Buddy 1 - WiFi Communication, Command and Telemetry
 * Purpose: Non-blocking WiFi association and connection recovery.
 */

#include "buddy1/wifi_connection.h"

#include "pico/cyw43_arch.h"

#include <stddef.h>

#ifndef WIFI_SSID
#define WIFI_SSID       "SET_ME"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD   "SET_ME"
#endif

#define WIFI_BACKOFF_START_MS  1000u
#define WIFI_BACKOFF_MAX_MS   16000u

static wifi_connection_state_t g_state = WIFI_CONNECTION_BOOT;
static uint32_t g_reconnects;
static uint32_t g_backoff_ms = WIFI_BACKOFF_START_MS;
static uint32_t g_retry_at_ms;

static void enter_backoff(uint32_t now_ms)
{
    g_state       = WIFI_CONNECTION_BACKOFF;
    g_retry_at_ms = now_ms + g_backoff_ms;
    g_reconnects++;

    g_backoff_ms *= 2u;
    if (g_backoff_ms > WIFI_BACKOFF_MAX_MS)
    {
        g_backoff_ms = WIFI_BACKOFF_MAX_MS;
    }
}

bool wifi_connection_init(void)
{
    if (0 != cyw43_arch_init())
    {
        return false;
    }

    cyw43_arch_enable_sta_mode();
    g_state       = WIFI_CONNECTION_BOOT;
    g_reconnects  = 0u;
    g_backoff_ms  = WIFI_BACKOFF_START_MS;
    g_retry_at_ms = 0u;
    return true;
}

void wifi_connection_process(uint32_t now_ms)
{
    int link;

    switch (g_state)
    {
        case WIFI_CONNECTION_BOOT:
            if (0 == cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD,
                                                   CYW43_AUTH_WPA2_AES_PSK))
            {
                g_state = WIFI_CONNECTION_JOINING;
            }
            else
            {
                enter_backoff(now_ms);
            }
            break;

        case WIFI_CONNECTION_JOINING:
            link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
            if (CYW43_LINK_UP == link)
            {
                g_state      = WIFI_CONNECTION_ONLINE;
                g_backoff_ms = WIFI_BACKOFF_START_MS;
            }
            else if ((CYW43_LINK_FAIL == link) ||
                     (CYW43_LINK_NONET == link) ||
                     (CYW43_LINK_BADAUTH == link))
            {
                enter_backoff(now_ms);
            }
            else
            {
                /* Association is still in progress. */
            }
            break;

        case WIFI_CONNECTION_ONLINE:
            link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
            if (CYW43_LINK_UP != link)
            {
                enter_backoff(now_ms);
            }
            break;

        case WIFI_CONNECTION_BACKOFF:
            if ((int32_t)(now_ms - g_retry_at_ms) >= 0)
            {
                g_state = WIFI_CONNECTION_BOOT;
            }
            break;

        case WIFI_CONNECTION_STATE_COUNT:
        default:
            g_state = WIFI_CONNECTION_BOOT;
            break;
    }
}

bool wifi_connection_is_online(void)
{
    return (WIFI_CONNECTION_ONLINE == g_state);
}

wifi_connection_state_t wifi_connection_state(void)
{
    return g_state;
}

const char * wifi_connection_state_name(wifi_connection_state_t state)
{
    static const char * const names[] =
    {
        "BOOT",
        "JOINING",
        "ONLINE",
        "BACKOFF"
    };

    if ((unsigned)state >= (unsigned)WIFI_CONNECTION_STATE_COUNT)
    {
        return "INVALID";
    }
    return names[state];
}

uint32_t wifi_connection_reconnect_count(void)
{
    return g_reconnects;
}

bool wifi_connection_get_rssi(int32_t * rssi_dbm)
{
    if ((NULL == rssi_dbm) || !wifi_connection_is_online())
    {
        return false;
    }

    return (0 == cyw43_wifi_get_rssi(&cyw43_state, rssi_dbm));
}
