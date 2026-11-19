/** @file mqtt_bridge.c
 * @brief Adapter between the kernel-owned MQTT port and Buddy 1 queues.
 */
#include "mqtt_bridge.h"
#include "../build_assert.h"

#if defined(TM_MQTT) && TM_MQTT

#include "buddy1.h"
#include "mqtt_utk.h"
#if APP_S3_MOTION
#include "../controller/remote_control.h"
extern unsigned int lwip_utk_rand (void);

static int remote_control_topic (const char * p_topic)
{
    static const char wanted[] = "car/cmd/control";
    unsigned value_i = 0;
    while (wanted[value_i] && p_topic[value_i] == wanted[value_i])
    {
        ++value_i;
    }
    return wanted[value_i] == '\0' && p_topic[value_i] == '\0';
}

static int remote_take (T_MQTT_UTK_APP_MSG * p_out, int status_allowed)
{
    unsigned value_n =
        s3_remote_reply_take(p_out->payload, MQTT_UTK_APP_PAYLOAD_MAX);
    const char * p_topic = "car/reply/control";
    if (!value_n && status_allowed)
    {
        value_n =
            s3_remote_status_take(p_out->payload, MQTT_UTK_APP_PAYLOAD_MAX);
        p_topic = "car/telemetry/control";
    }
    if (!value_n)
    {
        return 0;
    }
    for (unsigned value_i = 0; value_i < MQTT_UTK_APP_TOPIC_MAX; value_i++)
    {
        p_out->topic[value_i] = p_topic[value_i];
        if (!p_topic[value_i])
        {
            break;
        }
    }
    p_out->topic[MQTT_UTK_APP_TOPIC_MAX - 1] = '\0';
    p_out->payload_len = (unsigned short)value_n;
    p_out->qos = value_n == REMOTE_REPLY_SIZE ? 1 : 0;
    p_out->retain = 0;
    p_out->routine = value_n == REMOTE_STATUS_SIZE;
    return 1;
}
#endif

/* Compile-time guarantee: fallback heartbeat and application versions must agree. */
BUILD_ASSERT(BUDDY1_WIRE_VERSION == MQTT_UTK_APP_WIRE_VERSION,
             mqtt_bridge_c_check_a_t);
/* Compile-time guarantee: network staging must hold a complete binary snapshot. */
BUILD_ASSERT(BUDDY1_PAYLOAD_MAX <= MQTT_UTK_APP_PAYLOAD_MAX,
             mqtt_bridge_c_check_b_t);

static int has_command_prefix (const char * p_topic)
{
    static const char prefix[] = "car/cmd/";
    INT value_i;
    for (value_i = 0; prefix[value_i] != '\0'; value_i++)
    {
        if (p_topic[value_i] != prefix[value_i])
        {
            return 0;
        }
    }
    return 1;
}

int mqtt_utk_app_take (T_MQTT_UTK_APP_MSG * p_out)
{
#if APP_S3_MOTION
    static int status_turn = 1;
#endif
    buddy1_telemetry_msg_t message;
    INT value_i;
    const char * p_topic;

    if (p_out == NULL)
    {
        return 0;
    }
#if APP_S3_MOTION
    if (remote_take(p_out, status_turn))
    {
        if (p_out->routine)
        {
            status_turn = 0;
        }
        return 1;
    }
#endif
    if (!buddy1_outbox_take(&message))
    {
#if APP_S3_MOTION
        const int taken = remote_take(p_out, 1);
        if (taken && p_out->routine)
        {
            status_turn = 0;
        }
        return taken;
#else
        return 0;
#endif
    }
#if APP_S3_MOTION
    /* On a slow link a timer alone could select status on every call. Give
     * ordinary queued telemetry a turn between recurring control packets. */
    status_turn = 1;
#endif
    p_topic = buddy1_topic_name(message.type);
    if (p_topic == NULL)
    {
        return 0;
    }
    for (value_i = 0; value_i < MQTT_UTK_APP_TOPIC_MAX; value_i++)
    {
        p_out->topic[value_i] = p_topic[value_i];
        if (p_topic[value_i] == '\0')
        {
            break;
        }
    }
    p_out->topic[MQTT_UTK_APP_TOPIC_MAX - 1] = '\0';
    p_out->payload_len = (unsigned short)buddy1_payload_size(message.type);
    if (p_out->payload_len > MQTT_UTK_APP_PAYLOAD_MAX)
    {
        return 0;
    }
    for (value_i = 0; value_i < (INT)p_out->payload_len; value_i++)
    {
        p_out->payload[value_i] = message.payload[value_i];
    }
    p_out->qos = (message.type == BUDDY1_TOPIC_SNAPSHOT ||
                  message.type == BUDDY1_TOPIC_MAP_REPLY)
                     ? 1
                     : 0;
    p_out->retain = 0;
    /* Only recurring samples yield slots to barcode uploads. Faults, state,
     * barcode events and replies retain priority without duplicating topic
     * names or application enums in the network layer. */
    p_out->routine = (message.type == BUDDY1_TOPIC_MOTION ||
                      message.type == BUDDY1_TOPIC_POSE ||
                      message.type == BUDDY1_TOPIC_TERRAIN ||
                      message.type == BUDDY1_TOPIC_AVOIDANCE);
    return 1;
}

unsigned short mqtt_utk_app_status (unsigned char * p_payload,
                                    unsigned short capacity)
{
    buddy1_snapshot_t snapshot;
    if (g_mtx_buddy1_state <= 0 || !buddy1_snapshot(&snapshot))
    {
        return 0;
    }
    return (unsigned short)buddy1_encode(BUDDY1_TOPIC_STATUS, &snapshot,
                                         p_payload, capacity);
}

int mqtt_utk_app_link (int value_up)
{
    ER value_er = buddy1_update_faults(BUDDY1_FAULT_WIFI,
                                       value_up ? 0 : BUDDY1_FAULT_WIFI);
    /* E_TMOUT still means the fault mirror was updated. Only the live report
     * was dropped; the next heartbeat/snapshot carries the current mask. */
    return value_er == E_OK || value_er == E_TMOUT;
}

void mqtt_utk_app_connection (int value_up)
{
#if APP_S3_MOTION
    /* Broker reconnects can happen while Wi-Fi stays up. Each connection
     * needs its own freshness namespace, including that case. */
    s3_remote_link(value_up ? lwip_utk_rand() : 0U, value_up != 0);
#else
    (void)value_up;
#endif
}

void mqtt_utk_app_receive (const char * p_topic,
                           const unsigned char * p_payload,
                           unsigned short payload_len)
{
    if (p_topic == NULL || p_payload == NULL || !has_command_prefix(p_topic))
    {
        return;
    }
#if APP_S3_MOTION
    /* Bypass the ordinary command FIFO: STOP must replace pending START even
     * when the snapshot/map queue or telemetry outbox is full. */
    if (remote_control_topic(p_topic))
    {
        s3_remote_receive(p_payload, payload_len);
        return;
    }
#endif
    (void)buddy1_command_enqueue(p_topic, p_payload, payload_len);
}

#endif

/* End of file. */

