/*
 * File: mqtt_client.c
 * Owner: Buddy 1 - WiFi Communication, Command and Telemetry
 * Purpose: MQTT session, publishing, subscription and bounded buffering.
 */

#include "buddy1/mqtt_client.h"

#include "buddy1/telemetry.h"

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#ifndef MQTT_BROKER_IP
#define MQTT_BROKER_IP     "192.168.1.100"
#endif

#ifndef MQTT_BROKER_PORT
#define MQTT_BROKER_PORT   1883
#endif

#define MQTT_TOPIC_STATUS       TELE_TOPIC_ROOT "/status"
#define MQTT_TOPIC_COMMANDS     TELE_TOPIC_ROOT "/cmd/#"
#define MQTT_BACKOFF_START_MS   1000u
#define MQTT_BACKOFF_MAX_MS    16000u
#define MQTT_REPLY_MAX           192u

static mqtt_client_t * g_client;
static ip_addr_t       g_broker_address;
static volatile buddy1_mqtt_state_t g_state = BUDDY1_MQTT_WAIT_NETWORK;
static volatile uint32_t g_reconnects;
static volatile uint32_t g_backoff_ms = MQTT_BACKOFF_START_MS;
static volatile uint32_t g_retry_at_ms;

static char     g_command_topic[BUDDY1_MQTT_TOPIC_MAX];
static char     g_command_payload[BUDDY1_MQTT_PAYLOAD_MAX];
static uint16_t g_command_length;
static bool     g_command_overflow;
static bool     g_command_discard;
static volatile bool g_command_pending;
static volatile uint32_t g_command_dropped;

static char g_reply_topic[BUDDY1_MQTT_TOPIC_MAX];
static char g_reply_payload[MQTT_REPLY_MAX];
static volatile bool g_reply_pending;
static volatile bool g_reply_inflight;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void enter_backoff(uint32_t timestamp_ms)
{
    g_state          = BUDDY1_MQTT_BACKOFF;
    g_retry_at_ms    = timestamp_ms + g_backoff_ms;
    g_reply_inflight = false;
    g_reconnects++;

    g_backoff_ms *= 2u;
    if (g_backoff_ms > MQTT_BACKOFF_MAX_MS)
    {
        g_backoff_ms = MQTT_BACKOFF_MAX_MS;
    }
}

static void on_publish_done(void * arg, err_t err)
{
    (void)arg;
    (void)err;
}

static void on_reply_done(void * arg, err_t err)
{
    (void)arg;
    g_reply_inflight = false;
    if (ERR_OK == err)
    {
        g_reply_pending = false;
    }
}

static void on_subscribe_done(void * arg, err_t err)
{
    (void)arg;
    (void)err;
    /* The next reconnect repeats the subscription. Connection/status
     * telemetry exposes failures without doing work in this IRQ callback. */
}

static void on_incoming_topic(void * arg, const char * topic, u32_t total_len)
{
    (void)arg;
    (void)total_len;

    if (g_command_pending)
    {
        g_command_discard = true;
        g_command_dropped++;
        return;
    }

    g_command_discard  = false;
    g_command_length   = 0u;
    g_command_overflow = false;
    (void)snprintf(g_command_topic, sizeof g_command_topic, "%s", topic);
}

static void on_incoming_data(void * arg,
                             const u8_t * data,
                             u16_t length,
                             u8_t flags)
{
    (void)arg;

    if (g_command_discard)
    {
        if (0u != (flags & MQTT_DATA_FLAG_LAST))
        {
            g_command_discard = false;
        }
        return;
    }

    if (((size_t)g_command_length + (size_t)length) <
        sizeof g_command_payload)
    {
        memcpy(&g_command_payload[g_command_length], data, length);
        g_command_length = (uint16_t)(g_command_length + length);
    }
    else
    {
        g_command_overflow = true;
    }

    if (0u != (flags & MQTT_DATA_FLAG_LAST))
    {
        g_command_payload[g_command_length] = '\0';
        g_command_pending = true;
    }
}

static void on_connection(mqtt_client_t * client,
                          void * arg,
                          mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_state      = BUDDY1_MQTT_ONLINE;
        g_backoff_ms = MQTT_BACKOFF_START_MS;

        mqtt_set_inpub_callback(g_client, on_incoming_topic,
                                on_incoming_data, NULL);
        (void)mqtt_subscribe(g_client, MQTT_TOPIC_COMMANDS, 1u,
                             on_subscribe_done, NULL);
    }
    else
    {
        enter_backoff(now_ms());
    }
}

static void start_connection(uint32_t timestamp_ms)
{
    struct mqtt_connect_client_info_t client_info;
    err_t err;

    memset(&client_info, 0, sizeof client_info);
    client_info.client_id   = TELE_TOPIC_ROOT;
    client_info.keep_alive  = MQTT_CONNECT_KEEPALIVE;
    client_info.will_topic  = MQTT_TOPIC_STATUS;
    client_info.will_msg    = "{\"up\":false,\"reason\":\"link lost\"}";
    client_info.will_qos    = 1u;
    client_info.will_retain = 1u;

    g_state = BUDDY1_MQTT_CONNECTING;
    cyw43_arch_lwip_begin();
    err = mqtt_client_connect(g_client, &g_broker_address, MQTT_BROKER_PORT,
                              on_connection, NULL, &client_info);
    cyw43_arch_lwip_end();

    if (ERR_OK != err)
    {
        enter_backoff(timestamp_ms);
    }
}

static void publish_pending_reply(void)
{
    err_t err;

    if (!g_reply_pending || g_reply_inflight ||
        (BUDDY1_MQTT_ONLINE != g_state))
    {
        return;
    }

    g_reply_inflight = true;
    cyw43_arch_lwip_begin();
    err = mqtt_publish(g_client, g_reply_topic, g_reply_payload,
                       (u16_t)strlen(g_reply_payload), 1u, 0u,
                       on_reply_done, NULL);
    cyw43_arch_lwip_end();

    if (ERR_OK != err)
    {
        g_reply_inflight = false;
    }
}

bool buddy1_mqtt_init(void)
{
    if (0 == ipaddr_aton(MQTT_BROKER_IP, &g_broker_address))
    {
        return false;
    }

    cyw43_arch_lwip_begin();
    g_client = mqtt_client_new();
    cyw43_arch_lwip_end();
    if (NULL == g_client)
    {
        return false;
    }

    g_state            = BUDDY1_MQTT_WAIT_NETWORK;
    g_reconnects       = 0u;
    g_backoff_ms       = MQTT_BACKOFF_START_MS;
    g_retry_at_ms      = 0u;
    g_command_pending  = false;
    g_command_dropped  = 0u;
    g_command_discard  = false;
    g_command_overflow = false;
    g_reply_pending    = false;
    g_reply_inflight   = false;
    return true;
}

void buddy1_mqtt_process(uint32_t timestamp_ms, bool network_available)
{
    u8_t connected;

    if (!network_available)
    {
        if (BUDDY1_MQTT_ONLINE == g_state)
        {
            cyw43_arch_lwip_begin();
            mqtt_disconnect(g_client);
            cyw43_arch_lwip_end();
        }
        g_state          = BUDDY1_MQTT_WAIT_NETWORK;
        g_reply_inflight = false;
        return;
    }

    switch (g_state)
    {
        case BUDDY1_MQTT_WAIT_NETWORK:
            start_connection(timestamp_ms);
            break;

        case BUDDY1_MQTT_CONNECTING:
            /* Completion is reported asynchronously by on_connection(). */
            break;

        case BUDDY1_MQTT_ONLINE:
            cyw43_arch_lwip_begin();
            connected = mqtt_client_is_connected(g_client);
            cyw43_arch_lwip_end();
            if (0u == connected)
            {
                enter_backoff(timestamp_ms);
            }
            else
            {
                publish_pending_reply();
            }
            break;

        case BUDDY1_MQTT_BACKOFF:
            if ((int32_t)(timestamp_ms - g_retry_at_ms) >= 0)
            {
                start_connection(timestamp_ms);
            }
            break;

        case BUDDY1_MQTT_STATE_COUNT:
        default:
            g_state = BUDDY1_MQTT_WAIT_NETWORK;
            break;
    }
}

bool buddy1_mqtt_is_online(void)
{
    return (BUDDY1_MQTT_ONLINE == g_state);
}

buddy1_mqtt_state_t buddy1_mqtt_state(void)
{
    return g_state;
}

const char * buddy1_mqtt_state_name(buddy1_mqtt_state_t state)
{
    static const char * const names[] =
    {
        "WAIT_NETWORK",
        "CONNECTING",
        "ONLINE",
        "BACKOFF"
    };

    if ((unsigned)state >= (unsigned)BUDDY1_MQTT_STATE_COUNT)
    {
        return "INVALID";
    }
    return names[state];
}

uint32_t buddy1_mqtt_reconnect_count(void)
{
    return g_reconnects;
}

uint32_t buddy1_mqtt_command_drop_count(void)
{
    return g_command_dropped;
}

bool buddy1_mqtt_publish(const char * topic,
                         const char * payload,
                         uint8_t qos,
                         bool retain)
{
    size_t length;
    err_t  err;

    if ((NULL == topic) || (NULL == payload) || (qos > 2u) ||
        !buddy1_mqtt_is_online())
    {
        return false;
    }

    length = strlen(payload);
    if (length > (size_t)UINT16_MAX)
    {
        return false;
    }

    cyw43_arch_lwip_begin();
    err = mqtt_publish(g_client, topic, payload, (u16_t)length, qos,
                       retain ? 1u : 0u, on_publish_done, NULL);
    cyw43_arch_lwip_end();
    return (ERR_OK == err);
}

bool buddy1_mqtt_queue_reply(const char * topic, const char * payload)
{
    int topic_written;
    int payload_written;

    if ((NULL == topic) || (NULL == payload) || g_reply_pending)
    {
        return false;
    }

    topic_written = snprintf(g_reply_topic, sizeof g_reply_topic, "%s", topic);
    payload_written = snprintf(g_reply_payload, sizeof g_reply_payload,
                               "%s", payload);
    if ((topic_written < 0) || (payload_written < 0) ||
        ((size_t)topic_written >= sizeof g_reply_topic) ||
        ((size_t)payload_written >= sizeof g_reply_payload))
    {
        return false;
    }

    g_reply_pending  = true;
    g_reply_inflight = false;
    return true;
}

bool buddy1_mqtt_reply_pending(void)
{
    return g_reply_pending;
}

bool buddy1_mqtt_take_command(char * topic_out,
                              size_t topic_size,
                              char * payload_out,
                              size_t payload_size,
                              bool * overflowed)
{
    int topic_written;
    int payload_written;

    if ((NULL == topic_out) || (0u == topic_size) ||
        (NULL == payload_out) || (0u == payload_size) ||
        (NULL == overflowed) || !g_command_pending || g_reply_pending)
    {
        return false;
    }

    topic_written = snprintf(topic_out, topic_size, "%s", g_command_topic);
    payload_written = snprintf(payload_out, payload_size, "%s",
                               g_command_payload);
    if ((topic_written < 0) || (payload_written < 0) ||
        ((size_t)topic_written >= topic_size) ||
        ((size_t)payload_written >= payload_size))
    {
        return false;
    }

    *overflowed       = g_command_overflow;
    g_command_pending = false;
    g_command_overflow = false;
    return true;
}
