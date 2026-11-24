/** @file mqtt_bridge.c
 * @brief Adapter between the kernel-owned MQTT port and Buddy 1 queues.
 * @see docs/REMOTE_CONTROL_PROTOCOL.md for the dashboard request path.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 * Byte and half-word values fit in int, and uint32_t fits in int64_t.
 * The casts make the existing arithmetic promotions explicit.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 */
#include "mqtt_bridge.h"
#include "../build_assert.h"

#if defined(TM_MQTT) && TM_MQTT

#if APP_S3_MOTION
#include "../controller/remote_control.h"
extern unsigned int lwip_utk_rand (void);

#endif

/* Compile-time guarantee: fallback heartbeat and application versions must
 * agree. */
BUILD_ASSERT(BUDDY1_WIRE_VERSION == MQTT_UTK_APP_WIRE_VERSION,
             mqtt_bridge_c_check_a_t);
/* Compile-time guarantee: network staging must hold a complete binary snapshot.
 */
BUILD_ASSERT(BUDDY1_PAYLOAD_MAX <= MQTT_UTK_APP_PAYLOAD_MAX,
             mqtt_bridge_c_check_b_t);

#if APP_S3_MOTION
static int mqtt_bridge_is_control_topic (const char * p_topic);
static int mqtt_bridge_take_remote (T_MQTT_UTK_APP_MSG * p_out,
                                    int                  status_allowed);
#endif
static int mqtt_bridge_has_command_prefix (const char * p_topic);

/** @brief Select one application packet using the existing queue priorities.
 * @param[out] p_out Caller-owned writable packet; NULL returns zero.
 * @return One for a prepared packet, zero otherwise. Unknown queued topics
 * are consumed; an oversized payload may leave topic and length modified.
 * @note Network-owner task only; uses the queue/remote helpers' existing
 * synchronization. Recurring control status alternates with ordinary packets;
 * remote replies take precedence. No pointer is retained and no retry is added.
 */
int mqtt_utk_app_take (T_MQTT_UTK_APP_MSG * p_out)
{
#if APP_S3_MOTION
    static int status_turn = 1;
#endif
    buddy1_telemetry_msg_t message;
    INT                    byte_index;
    const char *           p_topic = NULL;

    if (NULL == p_out)
    {
        return 0;
    }
#if APP_S3_MOTION
    if (mqtt_bridge_take_remote(p_out, status_turn))
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
        const int taken = mqtt_bridge_take_remote(p_out, 1);

        if (taken && (p_out->routine))
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
    p_topic = telemetry_codec_topic_name(message.type);

    if (NULL == p_topic)
    {
        return 0;
    }

    for (byte_index = 0; byte_index < MQTT_UTK_APP_TOPIC_MAX; byte_index++)
    {
        p_out->topic[byte_index] = p_topic[byte_index];

        if ('\0' == (int)p_topic[byte_index])
        {
            break;
        }
    }

    p_out->topic[MQTT_UTK_APP_TOPIC_MAX - 1] = '\0';

    /* The selected layout rounds a byte-sized bit count up to full bytes
     * and adds one version byte. The total fits the MQTT length field. */
    p_out->payload_len = (uint16_t)telemetry_codec_version_size(
        message.type, message.payload[0]);

    if ((!p_out->payload_len) ||
        ((int)p_out->payload_len > MQTT_UTK_APP_PAYLOAD_MAX))
    {
        return 0;
    }

    for (byte_index = 0; byte_index < (INT)p_out->payload_len; byte_index++)
    {
        p_out->payload[byte_index] = message.payload[byte_index];
    }

    p_out->qos    = ((BUDDY1_TOPIC_SNAPSHOT == (int)message.type) ||
                  (BUDDY1_TOPIC_MAP_REPLY == (int)message.type))
                        ? 1
                        : 0;
    p_out->retain = 0;
    /* Only recurring samples yield slots to barcode uploads. Faults, state,
     * barcode events and replies retain priority without duplicating topic
     * names or application enums in the network layer. */
    p_out->routine = ((((BUDDY1_TOPIC_MOTION == (int)message.type) ||
                        (BUDDY1_TOPIC_POSE == (int)message.type)) ||
                       (BUDDY1_TOPIC_TERRAIN == (int)message.type)) ||
                      (BUDDY1_TOPIC_AVOIDANCE == (int)message.type));
    return 1;
}

/** @brief Encode a current Buddy 1 status snapshot for the heartbeat.
 * @param[out] p_payload Caller-owned buffer passed to buddy1_encode.
 * @param capacity Buffer capacity in bytes (0 through 65535).
 * @return Encoded length, or zero if state is unavailable or encoding fails.
 * @note Network-owner task; snapshot locking is delegated to buddy1_snapshot.
 * Snapshot failure leaves output untouched. Encoding validation and possible
 * partial output on failure are delegated to buddy1_encode; no pointer is kept.
 */
uint16_t mqtt_utk_app_status (uint8_t * p_payload, uint16_t capacity)
{
    telemetry_codec_snapshot_t snapshot;

    if ((gh_buddy1_state_mutex <= 0) || (!buddy1_snapshot(&snapshot)))
    {
        return 0;
    }

    /* Encoding returns zero or the fixed size of the status payload. */
    return (uint16_t)buddy1_encode(BUDDY1_TOPIC_STATUS, &snapshot, p_payload,
                                   capacity);
}

/** @brief Mirror the Wi-Fi state into Buddy 1's fault state.
 * @param b_is_up Zero means down; any nonzero value means up.
 * @return One when the mirror was updated, including E_TMOUT when its live
 * report was dropped; zero for other failures, permitting the caller to retry.
 * @note Network-owner task; uses buddy1_update_faults synchronization.
 */
int mqtt_utk_app_link (int b_is_up)
{
    ER value_er = buddy1_update_faults(BUDDY1_FAULT_WIFI,
                                       b_is_up ? 0U : BUDDY1_FAULT_WIFI);
    /* E_TMOUT still means the fault mirror was updated. Only the live report
     * was dropped; the next heartbeat/snapshot carries the current mask. */
    return (E_OK == value_er) || (E_TMOUT == value_er);
}

/** @brief Forward broker connection changes to remote-control freshness state.
 * @param b_is_connected Zero disconnects; nonzero obtains a new random epoch.
 * @note Network-owner task; synchronization is delegated to
 * remote_control_link. The hook is a no-op when motion integration is disabled.
 */
void mqtt_utk_app_connection (int b_is_connected)
{
#if APP_S3_MOTION
    /* Broker reconnects can happen while Wi-Fi stays up. Each connection
     * needs its own freshness namespace, including that case. */
    remote_control_link(b_is_connected ? lwip_utk_rand() : 0U,
                        b_is_connected != 0);
#else
    (void)b_is_connected;
#endif
}

/** @brief Dispatch complete commands, preserving remote STOP precedence.
 * @param p_topic Borrowed NUL-terminated topic; NULL is ignored.
 * @param p_payload Borrowed payload_len bytes; NULL is ignored.
 * @param payload_len Payload byte count (0 through 65535), not a buffer size.
 * @note Network-owner task only; input pointers are never retained. Remote
 * control bypasses the ordinary queue. Other commands use its polling enqueue;
 * non-command topics, malformed input and failed enqueues are silently dropped.
 */
void mqtt_utk_app_receive (const char * p_topic, const uint8_t * p_payload,
                           uint16_t payload_len)
{
    if (((NULL == p_topic) || (NULL == p_payload)) ||
        (!mqtt_bridge_has_command_prefix(p_topic)))
    {
        return;
    }
#if APP_S3_MOTION
    /* Bypass the ordinary command FIFO: STOP must replace pending START even
     * when the snapshot/map queue or telemetry outbox is full. */
    if (mqtt_bridge_is_control_topic(p_topic))
    {
        remote_control_receive(p_payload, payload_len);
        return;
    }
#endif
    (void)buddy1_command_enqueue(p_topic, p_payload, payload_len);
}

#if APP_S3_MOTION
/** @brief Match the exact remote-control topic, without changing input.
 * @param p_topic Non-NULL NUL-terminated string, borrowed for this call.
 * @return One for an exact match, zero otherwise. No shared state or locking.
 */
static int mqtt_bridge_is_control_topic (const char * p_topic)
{
    static const char wanted[]   = "car/cmd/control";
    unsigned          char_index = 0;

    while ((wanted[char_index]) && (p_topic[char_index] == wanted[char_index]))
    {
        ++char_index;
    }

    return ('\0' == (int)wanted[char_index]) &&
           ('\0' == (int)p_topic[char_index]);
}

/** @brief Take a remote reply first, then an eligible recurring status.
 * @param[out] p_out Non-NULL caller-owned packet; no pointer is retained.
 * @param status_allowed Nonzero permits status when no reply was taken.
 * @return One after filling a packet, zero with output unchanged otherwise.
 * @note Network-owner task only. The remote helpers attempt their existing
 * mutex without waiting; success consumes a reply or records a status send.
 */
static int mqtt_bridge_take_remote (T_MQTT_UTK_APP_MSG * p_out,
                                    int                  status_allowed)
{
    unsigned payload_len =
        remote_control_reply_take(p_out->payload, MQTT_UTK_APP_PAYLOAD_MAX);
    const char * p_topic = "car/reply/control";

    if ((!payload_len) && status_allowed)
    {
        payload_len = remote_control_status_take(p_out->payload,
                                                 MQTT_UTK_APP_PAYLOAD_MAX);
        p_topic     = "car/telemetry/control";
    }

    if (!payload_len)
    {
        return 0;
    }

    for (unsigned char_index = 0;
         char_index < (unsigned int)MQTT_UTK_APP_TOPIC_MAX; char_index++)
    {
        p_out->topic[char_index] = p_topic[char_index];

        if (!p_topic[char_index])
        {
            break;
        }
    }

    /* The remote reader returns a reply or status length, both within the
     * byte capacity supplied above. Its length fits uint16_t. */
    p_out->topic[MQTT_UTK_APP_TOPIC_MAX - 1] = '\0';
    p_out->payload_len                       = (uint16_t)payload_len;

    p_out->qos     = REMOTE_REPLY_SIZE == payload_len ? 1 : 0;
    p_out->retain  = 0;
    p_out->routine = REMOTE_STATUS_SIZE == payload_len;
    return 1;
}
#endif

/** @brief Test the command namespace prefix without changing input.
 * @param p_topic Non-NULL NUL-terminated string, borrowed for this call.
 * @return One when the prefix matches, zero otherwise. No locking or I/O.
 */
static int mqtt_bridge_has_command_prefix (const char * p_topic)
{
    static const char prefix[] = "car/cmd/";
    INT               char_index;

    for (char_index = 0; (int)prefix[char_index] != '\0'; char_index++)
    {
        if (p_topic[char_index] != prefix[char_index])
        {
            return 0;
        }
    }

    return 1;
}

#endif

/* End of file. */

