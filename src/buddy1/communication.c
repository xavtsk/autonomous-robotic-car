/*
 * File: communication.c
 * Owner: Buddy 1 - WiFi Communication, Command and Telemetry
 * Purpose: Coordinate WiFi, MQTT, commands and telemetry without blocking.
 */

#include "buddy1/communication.h"

#include "buddy1/command_receiver.h"
#include "buddy1/mqtt_client.h"
#include "buddy1/telemetry.h"
#include "buddy1/wifi_connection.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#define TOPIC_STATUS      TELE_TOPIC_ROOT "/status"
#define TOPIC_REPLY_MAP   TELE_TOPIC_ROOT "/reply/barcode_map"
#define TOPIC_REPLY_SNAP  TELE_TOPIC_ROOT "/reply/snapshot"
#define STATUS_PERIOD_MS  2000u
#define PUBLISH_BUDGET    4u

static robot_config_t * g_config;
static tele_msg_t g_pending_telemetry;
static bool g_has_pending_telemetry;
static bool g_was_online;
static bool g_initialized;
static uint32_t g_next_status_ms;

static void queue_map_reply(cmd_map_reply_t result)
{
    char reply[192];

    if (!cmd_format_barcode_map_reply(reply, sizeof reply, tele_now_ms(),
                                      g_config, result))
    {
        (void)tele_report_log("barcode map reply formatting failed");
        return;
    }

    if (!buddy1_mqtt_queue_reply(TOPIC_REPLY_MAP, reply))
    {
        (void)tele_report_log("barcode map reply queue busy");
    }
}

static void handle_command(void)
{
    char topic[BUDDY1_MQTT_TOPIC_MAX];
    char payload[BUDDY1_MQTT_PAYLOAD_MAX];
    bool overflowed;
    cmd_kind_t kind;

    if (!buddy1_mqtt_take_command(topic, sizeof topic,
                                  payload, sizeof payload, &overflowed))
    {
        return;
    }

    kind = cmd_kind_from_topic(topic);
    if (overflowed)
    {
        /* A map request still receives the promised structured rejection,
         * even when its body exceeded the bounded input buffer. */
        if (CMD_BARCODE_MAP == kind)
        {
            queue_map_reply(CMD_MAP_REPLY_MALFORMED);
        }
        (void)tele_report_log("command too long - ignored");
        return;
    }

    switch (kind)
    {
        case CMD_BARCODE_MAP:
        {
            nav_action_t parsed[ROBOT_BARCODE_COUNT];
            robot_config_result_t result;

            if (!cmd_parse_barcode_map(payload, parsed))
            {
                queue_map_reply(CMD_MAP_REPLY_MALFORMED);
                (void)tele_report_log("barcode map rejected: malformed");
                break;
            }

            result = robot_config_apply_barcode_map(g_config, parsed);
            if (ROBOT_CONFIG_APPLIED == result)
            {
                queue_map_reply(CMD_MAP_REPLY_ACCEPTED);
                (void)tele_report_log("barcode map accepted");
            }
            else if (ROBOT_CONFIG_LOCKED == result)
            {
                queue_map_reply(CMD_MAP_REPLY_LOCKED);
                (void)tele_report_log("barcode map rejected: locked");
            }
            else
            {
                queue_map_reply(CMD_MAP_REPLY_INVALID);
                (void)tele_report_log("barcode map rejected: invalid");
            }
            break;
        }

        case CMD_SNAPSHOT:
        {
            uint32_t replayed = tele_publish_snapshot();
            char reply[96];

            (void)snprintf(reply, sizeof reply,
                           "{\"t\":%" PRIu32 ",\"replayed\":%" PRIu32 "}",
                           tele_now_ms(), replayed);
            if (!buddy1_mqtt_queue_reply(TOPIC_REPLY_SNAP, reply))
            {
                (void)tele_report_log("snapshot reply queue busy");
            }
            break;
        }

        case CMD_UNKNOWN:
            (void)tele_report_log("unknown command received");
            break;

        case CMD_NONE:
        default:
            break;
    }
}

static void publish_status(bool retain)
{
    char payload[224];
    int32_t rssi = 0;
    uint32_t reconnects;

    (void)wifi_connection_get_rssi(&rssi);
    reconnects = wifi_connection_reconnect_count() +
                 buddy1_mqtt_reconnect_count();

    (void)snprintf(payload, sizeof payload,
                   "{\"up\":true,\"uptime_s\":%" PRIu32 ",\"rssi\":%" PRId32 ","
                   "\"phase\":\"%s\",\"dropped\":%" PRIu32 ","
                   "\"coalesced\":%" PRIu32 ",\"qdepth\":%" PRIu32 ","
                   "\"reconnects\":%" PRIu32 ",\"cmd_dropped\":%" PRIu32 "}",
                   tele_now_ms() / 1000u,
                   rssi,
                   run_phase_name(g_config->phase),
                   tele_dropped_count(),
                   tele_coalesced_count(),
                   tele_queue_depth(),
                   reconnects,
                   buddy1_mqtt_command_drop_count());

    (void)buddy1_mqtt_publish(TOPIC_STATUS, payload, 1u, retain);
}

static void drain_telemetry(void)
{
    unsigned attempt;

    for (attempt = 0u; attempt < PUBLISH_BUDGET; attempt++)
    {
        if (!g_has_pending_telemetry)
        {
            if (!tele_queue_pop(&g_pending_telemetry))
            {
                break;
            }
            g_has_pending_telemetry = true;
        }

        if (buddy1_mqtt_publish(g_pending_telemetry.topic,
                                g_pending_telemetry.payload,
                                0u, false))
        {
            g_has_pending_telemetry = false;
        }
        else
        {
            break;
        }
    }
}

bool communication_init(robot_config_t * config)
{
    if (NULL == config)
    {
        return false;
    }

    g_config = config;
    tele_init();
    if (!wifi_connection_init())
    {
        return false;
    }
    if (!buddy1_mqtt_init())
    {
        return false;
    }

    g_has_pending_telemetry = false;
    g_was_online            = false;
    g_next_status_ms        = 0u;
    g_initialized           = true;
    return true;
}

void communication_process(void)
{
    uint32_t timestamp_ms;
    bool online;

    if (!g_initialized)
    {
        return;
    }

    timestamp_ms = tele_now_ms();
    wifi_connection_process(timestamp_ms);
    buddy1_mqtt_process(timestamp_ms, wifi_connection_is_online());
    online = communication_is_online();

    if (!online)
    {
        g_was_online = false;
        return;
    }

    if (!g_was_online)
    {
        publish_status(true);
        g_next_status_ms = timestamp_ms + STATUS_PERIOD_MS;
        g_was_online = true;
    }

    handle_command();

    /* If command handling queued a critical reply, submit it before ordinary
     * QoS 0 telemetry consumes more of lwIP's bounded output buffer. */
    buddy1_mqtt_process(timestamp_ms, true);
    drain_telemetry();

    if ((int32_t)(timestamp_ms - g_next_status_ms) >= 0)
    {
        publish_status(true);
        g_next_status_ms = timestamp_ms + STATUS_PERIOD_MS;
    }
}

bool communication_is_online(void)
{
    return wifi_connection_is_online() && buddy1_mqtt_is_online();
}

uint32_t communication_reconnect_count(void)
{
    return wifi_connection_reconnect_count() +
           buddy1_mqtt_reconnect_count();
}
