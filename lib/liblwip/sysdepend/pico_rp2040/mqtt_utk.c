/** @file mqtt_utk.c
 * @brief Run MQTT connection and callback processing within the network-owning
 * task.
 * The platform flag is nonnegative. Convert it to unsigned before testing
 * register or packet bits.
 * A topic character and the staged byte counts fit in int. lwIP error
 * values and MQTT connection statuses also fit the monitor's %d argument.
 * Console strings have static storage and the monitor reads them without
 * writing through its historical mutable pointer type.
 */
/* cyw43_utk_task() in lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c owns
 * CYW43439, lwIP and MQTT together. Each linked poll calls mqtt_utk_start()
 * and mqtt_utk_poll(). The connection guards retain an active or pending
 * attempt. lwIP keepalive, retry and heartbeat timeouts share the owner's
 * sys_check_timeouts() poll with DHCP and TCP.
 *
 * lwip/sys.h declares the clock. Retain the toolchain size_t alongside the
 * kernel monitor types as in sys_arch.c. Publish callbacks stay on the
 * network owner.
 */

#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "../../../../app_program/build_assert.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_HEARTBEAT_MS 5000
#define MQTT_UTK_KEEPALIVE_S 60
#define MQTT_UTK_RETRY_MS 3000
#define MQTT_UTK_PUBLISH_GAP_MS 20U
#define MQTT_UTK_PUBLISH_RETRY_MS 100U
#define MQTT_UTK_COMMAND_TOPIC "car/cmd/#"

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks. Preserve raw bytes, including zero. Reject the
 * entire packet if its topic/length is oversized or fragments do not match. */
#define MQTT_UTK_RX_TOPIC_MAX 64
#define MQTT_UTK_RX_DATA_MAX MQTT_UTK_APP_PAYLOAD_MAX

/* One fresh application packet may wait here during a capture turn. Once
 * either source has been attempted, retain that source until completion or a
 * permanent error, even across reconnects. Never interleave retries. */
typedef enum
{
    PUBLISH_NONE,
    PUBLISH_APPLICATION,
    PUBLISH_CAPTURE
} mqtt_utk_publish_kind_t;

#if APP_IR_DEBUG
/* Give fresh controller observations a bounded turn even when motion/terrain
 * fill the application FIFO. On a slow link, two successful data packets must
 * still progress between previews. Never overwrite a staged BCAP packet. */
#define MQTT_UTK_CONTROL_PERIOD_MS 200U
#define MQTT_UTK_CONTROL_DATA_GAP 2U
#endif
#if APP_IMU_DEBUG
#define MQTT_UTK_IMU_DATA_GAP 2U
#endif

/* Every field below is owned by the existing CYW43/lwIP task. Callback
 * reentrancy stays on that owner; no ISR/shared volatile flags are packed here.
 * Wide hook inputs are validated before these bounded fields are updated.
 * Native layout is never sent over MQTT. */
typedef struct
{
    bool     b_is_connecting : 1;
    bool     b_is_rx_truncated : 1;
    bool     b_has_pending_application : 1;
    bool     b_has_inflight_packet : 1;
    bool     b_has_warned_publish : 1;
    bool     b_is_publish_paced : 1;
    bool     b_is_capture_turn : 1;
    bool     b_has_capture_packet : 1;
    uint32_t retry_source : 2;     /**< NONE/APPLICATION/CAPTURE, 0..2. */
    uint32_t rx_len : 5;           /**< Accumulated bytes, saturated at 17. */
    uint32_t rx_expected : 5;      /**< Valid advertised bytes, 0..17. */
    uint32_t debug_len : 8;        /**< Validated capture bytes, 1..164 or 0. */
    uint32_t publish_interval : 2; /**< Exact interval encoding, 0..2. */
#if APP_IR_DEBUG
    bool     b_has_taken_control : 1;
    uint32_t control_data_since : 2;
#endif
#if APP_IMU_DEBUG
    uint32_t imu_data_since : 2;
#endif
} mqtt_utk_state_t;

typedef enum
{
    MQTT_UTK_WAIT_INITIAL = 0,
    MQTT_UTK_WAIT_GAP,
    MQTT_UTK_WAIT_RETRY
} mqtt_utk_wait_state_t;

BUILD_ASSERT(MQTT_UTK_RX_DATA_MAX <= 31U, mqtt_utk_rx_capacity_t);
BUILD_ASSERT(MQTT_UTK_DEBUG_PAYLOAD_MAX <= 255U, mqtt_utk_debug_capacity_t);
BUILD_ASSERT((unsigned)PUBLISH_CAPTURE <= 3U, mqtt_utk_source_capacity_t);
#if defined(__arm__)
/* Target allocation is separate from the host compiler's bitfield ABI. */
BUILD_ASSERT(sizeof(mqtt_utk_state_t) <= 8U, mqtt_utk_state_allocation_t);
#endif
BUILD_ASSERT(MQTT_UTK_PUBLISH_GAP_MS <= UINT16_MAX, mqtt_utk_gap_range_t);
BUILD_ASSERT(MQTT_UTK_PUBLISH_RETRY_MS <= UINT16_MAX, mqtt_utk_retry_range_t);
#if APP_IR_DEBUG
BUILD_ASSERT(MQTT_UTK_CONTROL_DATA_GAP <= 3U, mqtt_utk_control_gap_t);
#endif
#if APP_IMU_DEBUG
BUILD_ASSERT(MQTT_UTK_IMU_DATA_GAP <= 3U, mqtt_utk_imu_gap_t);
#endif

static mqtt_client_t * gp_client = NULL;
static ip_addr_t       g_broker_address;

static char g_rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB   g_rx_data[MQTT_UTK_RX_DATA_MAX];

/* A fixed, single-owner retry slot, not another queue or a larger lwIP pool.
 * Retain the complete packet until its publish callback succeeds. At most one
 * application publish is in flight, leaving request slots for subscriptions
 * and heartbeat. The bounded application FIFO applies backpressure behind it.
 * No blocking waits or extra timeout objects are needed. */
static T_MQTT_UTK_APP_MSG g_pending_message;
static u32_t              g_last_publish_ms;

/* Separate capture storage keeps a held application packet intact. */
static uint8_t g_debug_payload[MQTT_UTK_DEBUG_PAYLOAD_MAX];
#if APP_IMU_DEBUG
static uint8_t g_preview_payload[MQTT_UTK_IMU_DEBUG_MAX];
#elif APP_IR_DEBUG
static uint8_t g_preview_payload[MQTT_UTK_CONTROL_DEBUG_MAX];
#endif
#if APP_IR_DEBUG
static u32_t g_last_control_ms;
#endif

static mqtt_utk_state_t g_mqtt_state = {
    .publish_interval = MQTT_UTK_WAIT_INITIAL,
#if APP_IR_DEBUG
    .control_data_since = MQTT_UTK_CONTROL_DATA_GAP,
#endif
#if APP_IMU_DEBUG
    .imu_data_since = MQTT_UTK_IMU_DATA_GAP,
#endif
};

static u16_t mqtt_utk_publish_wait_ms (void);

static void pace_publish (u16_t wait_ms);

static void warn_pending (err_t result);

static void data_publish_completed (void);

static void application_publish_done_cb (void * p_context, err_t result);

static void debug_publish_done_cb (void * p_context, err_t result);

#if APP_IR_DEBUG || APP_IMU_DEBUG
static void preview_publish_done_cb (void * p_context, err_t result);

static void publish_preview (const char * p_topic, unsigned int size);

#endif
static void incoming_publish_cb (void * p_context, const char * p_topic,
                                 u32_t tot_len);

static void incoming_data_cb (void * p_context, const u8_t * p_data, u16_t len,
                              u8_t flags);

static void subscribe_done_cb (void * p_context, err_t result);

static void command_subscribe_done_cb (void * p_context, err_t result);

static void publish_done_cb (void * p_context, err_t result);

static void heartbeat_timeout (void * p_context);

static void retry_timeout (void * p_context);

static void connection_cb (mqtt_client_t * p_client, void * p_context,
                           mqtt_connection_status_t status);

/** @brief Default barcode hook: supply no packet and leave output unchanged.
 * @param p_bytes Ignored borrowed output buffer; may be NULL.
 * @param size Ignored byte capacity. No state, ownership transfer or locking.
 * @return Zero. The application may override this network-task hook.
 */
__attribute__((weak)) unsigned int mqtt_utk_debug_take (uint8_t *    p_bytes,
                                                        unsigned int size)
{
    (void)p_bytes;
    (void)size;
    return 0;
}

/** @brief Default IR hook: supply no packet and leave output unchanged.
 * @param p_bytes Ignored borrowed output buffer; may be NULL.
 * @param size Ignored byte capacity. No state, ownership transfer or locking.
 * @return Zero. The application may override this network-task hook.
 */
__attribute__((weak)) unsigned int mqtt_utk_ir_debug_take (uint8_t *    p_bytes,
                                                           unsigned int size)
{
    (void)p_bytes;
    (void)size;
    return 0U;
}

/** @brief Default controller hook: supply no packet or modify output.
 * @param p_bytes Ignored borrowed output buffer; may be NULL.
 * @param size Ignored byte capacity. No state, ownership transfer or locking.
 * @return Zero. The application may override this network-task hook.
 */
__attribute__((weak)) unsigned int
mqtt_utk_control_debug_take (uint8_t * p_bytes, unsigned int size)
{
    (void)p_bytes;
    (void)size;
    return 0U;
}

/** @brief Default IMU hook: supply no packet and leave output unchanged.
 * @param p_bytes Ignored borrowed output buffer; may be NULL.
 * @param size Ignored byte capacity. No state, ownership transfer or locking.
 * @return Zero. The application may override this network-task hook.
 */
__attribute__((weak)) unsigned int mqtt_utk_imu_debug_take (uint8_t * p_bytes,
                                                            unsigned int size)
{
    (void)p_bytes;
    (void)size;
    return 0U;
}

/** @brief Default application hook: leave output unchanged and take nothing.
 * @param p_out Ignored borrowed packet pointer; may be NULL.
 * @return Zero. No state or locking; called by the network-owner task.
 */
__attribute__((weak)) int mqtt_utk_app_take (T_MQTT_UTK_APP_MSG * p_out)
{
    (void)p_out;
    return 0;
}

/** @brief Default receive hook: ignore a completed inbound publication.
 * @param p_topic Ignored topic pointer; may be NULL and is not retained.
 * @param p_payload Ignored payload pointer; may be NULL and is not retained.
 * @param payload_len Ignored byte count (0 through 65535).
 * @note Network-owner task; no state changes, output or locking.
 */
__attribute__((weak)) void mqtt_utk_app_receive (const char *    p_topic,
                                                 const uint8_t * p_payload,
                                                 uint16_t        payload_len)
{
    (void)p_topic;
    (void)p_payload;
    (void)payload_len;
}

/** @brief Encode the default two-byte heartbeat without application state.
 * @param[out] p_payload Writable capacity-byte buffer; must be non-NULL when
 * capacity is at least two. No pointer is retained.
 * @param capacity Buffer capacity in bytes (0 through 65535).
 * @return Two after writing the version/alive bytes, or zero with output
 * unchanged when capacity is below two. Network-owner task; no locking.
 */
__attribute__((weak)) uint16_t mqtt_utk_app_status (uint8_t * p_payload,
                                                    uint16_t  capacity)
{
    if (capacity < 2U)
    {
        return 0;
    }

    p_payload[0] = MQTT_UTK_APP_WIRE_VERSION;
    p_payload[1] = 1; /* Report a heartbeat with unknown faults. */
    return 2;
}

/** @brief Default link hook: accept the observation without side effects.
 * @param b_is_up Ignored Wi-Fi state (zero down, nonzero up).
 * @return One. Network-owner task; no shared state or locking.
 */
__attribute__((weak)) int mqtt_utk_app_link (int b_is_up)
{
    (void)b_is_up;
    return 1;
}

/** @brief Default broker hook: ignore the connection change.
 * @param b_is_connected Ignored broker state (zero disconnected, nonzero
 * connected).
 * @note Network-owner task; no shared state, output or locking.
 */
__attribute__((weak)) void mqtt_utk_app_connection (int b_is_connected)
{
    (void)b_is_connected;
}

/** @brief Start an IPv4 broker connection from the network-owner task. Leave an
 * existing or pending connection alone. Log allocation, address and connection
 * errors without changing the reconnect cadence. */
void mqtt_utk_start (void)
{
    struct mqtt_connect_client_info_t client_info = {0};
    err_t                             err;

    if ((g_mqtt_state.b_is_connecting) ||
        ((gp_client != NULL) && (mqtt_client_is_connected(gp_client))))
    {
        return;
    }

    if (NULL == gp_client)
    {
        gp_client = mqtt_client_new();

        if (NULL == gp_client)
        {
            tm_printf((UB *)"mqtt: mqtt_client_new() failed\n");
            return;
        }
    }

    if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&g_broker_address)))
    {
        tm_printf((UB *)"mqtt: bad MQTT_BROKER_IP\n");
        return;
    }
    IP_SET_TYPE(&g_broker_address, IPADDR_TYPE_V4);

    client_info.client_id  = MQTT_CLIENT_ID;
    client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

    /* clang-format off */
    g_mqtt_state.b_is_connecting = true;
    err                          =
        mqtt_client_connect(gp_client, &g_broker_address, MQTT_BROKER_PORT,
                              connection_cb, NULL, &client_info);
    /* clang-format on */

    if (err != ERR_OK)
    {
        tm_printf((UB *)"mqtt: mqtt_client_connect() failed, err=%d\n",
                  (int)err);
        g_mqtt_state.b_is_connecting = false;
    }
}

/** @brief Advance the single in-flight broker publication from the
 * network-owner task. Keep application and capture retry data until the
 * retained attempt resolves. Skip a disconnected client or an outstanding
 * pacing delay. */
void mqtt_utk_poll (void)
{
    err_t        err;
    unsigned int payload_length;
    unsigned int capture_checked = 0;

    if ((NULL == gp_client) || (!mqtt_client_is_connected(gp_client)))
    {
        g_mqtt_state.b_has_inflight_packet = false;
        return;
    }

    /* Keep elapsed milliseconds modulo 2^32, including across clock wrap. */
    if ((g_mqtt_state.b_has_inflight_packet) ||
        ((g_mqtt_state.b_is_publish_paced) &&
         ((sys_now() - g_last_publish_ms) < mqtt_utk_publish_wait_ms())))
    {
        return;
    }

    /* A timeout/disconnect can leave delivery ambiguous. Finish the exact
     * attempted packet before taking another packet from either source.
     * The two-bit source fits in int in both comparisons. */
    if (PUBLISH_APPLICATION == (int)(g_mqtt_state.retry_source))
    {
        goto publish_application;
    }

    if (PUBLISH_CAPTURE == (int)(g_mqtt_state.retry_source))
    {
        goto publish_capture;
    }

    bool b_did_take_packet = false;

    if (!g_mqtt_state.b_has_pending_application)
    {
        g_pending_message.routine =
            0; /* Hooks can leave the scheduling hint unset. */
        b_did_take_packet = 0 != mqtt_utk_app_take(&g_pending_message);
    }

    if (b_did_take_packet)
    {
        g_pending_message.topic[MQTT_UTK_APP_TOPIC_MAX - 1] = '\0';

        if (((((int)(g_pending_message.payload_len) >
               MQTT_UTK_APP_PAYLOAD_MAX) ||
              ('\0' == (int)(g_pending_message.topic[0]))) ||
             (g_pending_message.qos > 2U)) ||
            (g_pending_message.retain > 1U))
        {
            tm_printf((UB *)"mqtt: invalid application packet discarded\n");
            pace_publish(MQTT_UTK_PUBLISH_GAP_MS);
            return;
        }

        g_mqtt_state.b_has_pending_application = true;
        g_mqtt_state.b_has_warned_publish      = false;
    }

    /* Do not reorder the application FIFO. Its head event/reply takes
     * priority, while recurring samples share slots one-for-one with BCAP.
     * Urgent events behind a held sample still wait for the existing FIFO. */
    if ((g_mqtt_state.b_has_pending_application) &&
        (!g_pending_message.routine))
    {
        goto publish_application;
    }
#if APP_IR_DEBUG || APP_IMU_DEBUG
    /* With no queued application packet, check for BCAP before treating the
     * link as idle. Diagnostics use separate storage from the staged capture.
     */
    if ((!g_mqtt_state.b_has_pending_application) &&
        (!g_mqtt_state.b_has_capture_packet))
    {
        /* clang-format off */
        payload_length  =
            mqtt_utk_debug_take(g_debug_payload, sizeof g_debug_payload);
        capture_checked = 1;
        /* clang-format on */

        if (payload_length > sizeof g_debug_payload)
        {
            return;
        }

        if (payload_length != 0U)
        {
            /* The buffer-size guard bounds this length before narrowing. */
            g_mqtt_state.debug_len            = (u16_t)payload_length;
            g_mqtt_state.b_has_capture_packet = true;
        }
    }
#endif
#if APP_IR_DEBUG
    if (((!g_mqtt_state.b_has_taken_control) ||
         ((sys_now() - g_last_control_ms) >= MQTT_UTK_CONTROL_PERIOD_MS)) &&
        (((!g_mqtt_state.b_has_pending_application) &&
          (!g_mqtt_state.b_has_capture_packet)) ||
         (g_mqtt_state.control_data_since >= MQTT_UTK_CONTROL_DATA_GAP)))
    {
        payload_length = mqtt_utk_control_debug_take(
            g_preview_payload, MQTT_UTK_CONTROL_DEBUG_MAX);

        if (payload_length > MQTT_UTK_CONTROL_DEBUG_MAX)
        {
            return;
        }

        if (payload_length != 0U)
        {
            g_last_control_ms                = sys_now();
            g_mqtt_state.b_has_taken_control = true;
            g_mqtt_state.control_data_since  = 0;
            publish_preview(MQTT_UTK_CONTROL_DEBUG_TOPIC, payload_length);
            return;
        }
    }
#endif
#if APP_IMU_DEBUG
    if ((!g_mqtt_state.b_has_pending_application &&
         !g_mqtt_state.b_has_capture_packet) ||
        g_mqtt_state.imu_data_since >= MQTT_UTK_IMU_DATA_GAP)
    {
        payload_length =
            mqtt_utk_imu_debug_take(g_preview_payload, MQTT_UTK_IMU_DEBUG_MAX);
        if (payload_length > MQTT_UTK_IMU_DEBUG_MAX)
        {
            return;
        }
        if (payload_length != 0U)
        {
            g_mqtt_state.imu_data_since = 0;
            publish_preview(MQTT_UTK_IMU_DEBUG_TOPIC, payload_length);
            return;
        }
    }
#endif
    if ((g_mqtt_state.b_has_pending_application) &&
        (!g_mqtt_state.b_is_capture_turn))
    {
        goto publish_application;
    }

    if ((!g_mqtt_state.b_has_capture_packet) && (!capture_checked))
    {
        payload_length =
            mqtt_utk_debug_take(g_debug_payload, sizeof g_debug_payload);

        if (payload_length > sizeof g_debug_payload)
        {
            return;
        }

        if (payload_length != 0U)
        {
            /* The buffer-size guard bounds this length before narrowing. */
            g_mqtt_state.debug_len            = (u16_t)payload_length;
            g_mqtt_state.b_has_capture_packet = true;
        }
    }

    if (g_mqtt_state.b_has_capture_packet)
    {
        goto publish_capture;
    }

    if (g_mqtt_state.b_has_pending_application)
    {
        goto publish_application;
    }

#if APP_IR_DEBUG
    /* IR remains spare-capacity only. */
    payload_length =
        mqtt_utk_ir_debug_take(g_preview_payload, MQTT_UTK_IR_DEBUG_MAX);

    if ((payload_length != 0U) && (payload_length <= MQTT_UTK_IR_DEBUG_MAX))
    {
        publish_preview(MQTT_UTK_IR_DEBUG_TOPIC, payload_length);
    }
#endif
    return;

publish_capture:
    g_mqtt_state.retry_source          = PUBLISH_CAPTURE;
    g_mqtt_state.b_has_inflight_packet = true;
    pace_publish(MQTT_UTK_PUBLISH_GAP_MS);
    err =
        mqtt_publish(gp_client, MQTT_UTK_DEBUG_TOPIC, g_debug_payload,
                     g_mqtt_state.debug_len, 0, 0, debug_publish_done_cb, NULL);

    if (err != ERR_OK)
    {
        g_mqtt_state.b_has_inflight_packet = false;

        if ((ERR_MEM == err) || (ERR_CONN == err))
        {
            pace_publish(MQTT_UTK_PUBLISH_RETRY_MS);
        }
        else
        {
            g_mqtt_state.b_has_capture_packet =
                false; /* Retrying invalid arguments gives the same error. */
            g_mqtt_state.retry_source      = PUBLISH_NONE;
            g_mqtt_state.b_is_capture_turn = false;
        }
    }

    return;

publish_application:
    /* Set state before calling lwIP; test ports may complete synchronously.
     * The next packet cannot overtake this one or consume another request slot.
     */
    g_mqtt_state.retry_source          = PUBLISH_APPLICATION;
    g_mqtt_state.b_has_inflight_packet = true;
    pace_publish(MQTT_UTK_PUBLISH_GAP_MS);
    /* The validated application length is at most 17 bytes and fits u16_t
     * for lwIP. The same check applies to a retained retry. */
    err = mqtt_publish(
        gp_client, g_pending_message.topic, g_pending_message.payload,
        (u16_t)g_pending_message.payload_len, g_pending_message.qos,
        g_pending_message.retain, application_publish_done_cb, NULL);

    if (err != ERR_OK)
    {
        g_mqtt_state.b_has_inflight_packet = false;

        if ((ERR_MEM == err) || (ERR_CONN == err))
        {
            warn_pending(err);
            pace_publish(MQTT_UTK_PUBLISH_RETRY_MS);
        }
        else
        {
            /* Bad arguments cannot improve with a retry. Drop this one packet
             * explicitly, then allow the next FIFO item on a later poll. */
            tm_printf((UB *)"mqtt: application packet discarded, err=%d\n",
                      (int)err);
            g_mqtt_state.b_has_pending_application = false;
            g_mqtt_state.b_has_warned_publish      = false;
            g_mqtt_state.retry_source              = PUBLISH_NONE;
        }
    }
}

/** @brief Decode the exact retained publish interval in milliseconds.
 * @return Zero before pacing, 20 for normal pacing or 100 for retry pacing.
 * @note Called only by the network owner; no state or clock is changed.
 */
static u16_t mqtt_utk_publish_wait_ms (void)
{
    switch (g_mqtt_state.publish_interval)
    {
    case MQTT_UTK_WAIT_GAP:
        return MQTT_UTK_PUBLISH_GAP_MS;
    case MQTT_UTK_WAIT_RETRY:
        return MQTT_UTK_PUBLISH_RETRY_MS;
    default:
        return 0U;
    }
}

/** @brief Start the existing publish pacing interval on the network owner.
 * @param wait_ms One of MQTT_UTK_PUBLISH_GAP_MS or MQTT_UTK_PUBLISH_RETRY_MS.
 * @note Updates the same timestamp before recording the interval.
 * Every private caller supplies one of the two named constants. This
 * representation does not accept a new caller-selected timeout or change the
 * public MQTT hook's validation behavior.
 */
static void pace_publish (u16_t wait_ms)
{
    g_last_publish_ms = sys_now();
    /* All private callers supply one of these two unchanged intervals. */
    g_mqtt_state.publish_interval   = (MQTT_UTK_PUBLISH_GAP_MS == wait_ms)
                                          ? MQTT_UTK_WAIT_GAP
                                          : MQTT_UTK_WAIT_RETRY;
    g_mqtt_state.b_is_publish_paced = true;
}

/** @brief Print the first delayed application-publish warning for the retained
 * packet and remember that it was reported.
 */
static void warn_pending (err_t result)
{
    if (!g_mqtt_state.b_has_warned_publish)
    {
        tm_printf((UB *)"mqtt: application publish delayed, err=%d; retaining "
                        "packet\n",
                  (int)result);
        g_mqtt_state.b_has_warned_publish = true;
    }
}

/** @brief Advance each enabled preview stream's spacing counter up to its
 * configured data-packet gap.
 */
static void data_publish_completed (void)
{
#if APP_IR_DEBUG
    if (g_mqtt_state.control_data_since < MQTT_UTK_CONTROL_DATA_GAP)
    {
        ++g_mqtt_state.control_data_since;
    }
#endif
#if APP_IMU_DEBUG
    if (g_mqtt_state.imu_data_since < MQTT_UTK_IMU_DATA_GAP)
    {
        ++g_mqtt_state.imu_data_since;
    }
#endif
}

/** @brief Release the in-flight application send. Success consumes the pending
 * packet and advances pacing. An error retains the packet for retry,
 * which can produce duplicate delivery.
 */
static void application_publish_done_cb (void * p_context, err_t result)
{
    (void)p_context;
    g_mqtt_state.b_has_inflight_packet = false;

    if (ERR_OK == result)
    {
        g_mqtt_state.b_has_pending_application = false;
        g_mqtt_state.b_has_warned_publish      = false;
        g_mqtt_state.retry_source              = PUBLISH_NONE;
        g_mqtt_state.b_is_capture_turn         = true;
        data_publish_completed();
    }
    else
    {
        /* A request timeout is not proof that the broker never received it.
         * Retry is at-least-once best effort; consumers must tolerate repeats.
         */
        warn_pending(result);
        pace_publish(MQTT_UTK_PUBLISH_RETRY_MS);
    }
}

/** @brief Release the in-flight capture publish. Success consumes the retained
 * packet and advances scheduling. Failure keeps it eligible for a
 * paced retry.
 */
static void debug_publish_done_cb (void * p_context, err_t result)
{
    (void)p_context;
    g_mqtt_state.b_has_inflight_packet = false;

    if (ERR_OK == result)
    {
        g_mqtt_state.b_has_capture_packet = false;
        g_mqtt_state.retry_source         = PUBLISH_NONE;
        g_mqtt_state.b_is_capture_turn    = false;
        data_publish_completed();
    }
    else
    {
        pace_publish(MQTT_UTK_PUBLISH_RETRY_MS);
    }
}

#if APP_IR_DEBUG || APP_IMU_DEBUG
/** @brief Complete a live preview send and release the in-flight slot. The
 * lwIP callback context is unused.
 */
static void preview_publish_done_cb (void * p_context, err_t result)
{
    (void)p_context;
    (void)result;
    g_mqtt_state.b_has_inflight_packet =
        false; /* Drop the old preview after any completion. */
}

/** @brief Submit the prepared live preview bytes on the supplied topic and
 * update in-flight state according to lwIP acceptance.
 */
static void publish_preview (const char * p_topic, unsigned int size)
{
    g_mqtt_state.b_has_inflight_packet = true;
    pace_publish(MQTT_UTK_PUBLISH_GAP_MS);
    /* Each preview caller bounds its packet to 16, 70 or 156 bytes.
     * Every accepted length fits the u16_t publish argument. */
    err_t err = mqtt_publish(gp_client, p_topic, g_preview_payload, (u16_t)size,
                             0, 0, preview_publish_done_cb, NULL);

    if (err != ERR_OK)
    {
        g_mqtt_state.b_has_inflight_packet = false;
    }
}

#endif
/** @brief Start reassembly of a newly announced inbound message.
 * @param p_context Unused lwIP callback cookie.
 * @param p_topic NUL-terminated topic borrowed for the duration of this call.
 * @param tot_len Announced payload length in bytes.
 * @details The network owner copies the topic and resets the fragment
 * cursor here. Overlong topics and payloads leave the rejection flag set.
 */
static void incoming_publish_cb (void * p_context, const char * p_topic,
                                 u32_t tot_len)
{
    u32_t byte_index;

    (void)p_context;

    for (byte_index = 0;
         (byte_index < ((unsigned int)(MQTT_UTK_RX_TOPIC_MAX - 1))) &&
         ((int)(p_topic[byte_index]) != '\0');
         byte_index++)
    {
        g_rx_topic[byte_index] = p_topic[byte_index];
    }

    g_rx_topic[byte_index] = '\0';

    /* clang-format off */
    g_mqtt_state.rx_len            = 0;
    g_mqtt_state.b_is_rx_truncated =
        (((int)(p_topic[byte_index]) != '\0') ||
         (tot_len > (unsigned int)MQTT_UTK_RX_DATA_MAX));
    /* clang-format on */
    /* Retain only lengths that fit the 17-byte inbound buffer. This range
     * fits both u16_t and the five-bit field. Other lengths store zero
     * and leave the rejection flag set. */
    g_mqtt_state.rx_expected = (tot_len <= (unsigned int)MQTT_UTK_RX_DATA_MAX)
                                   ? (u16_t)tot_len
                                   : (u16_t)0;
}

/** @brief Append incoming fragments within the receive buffer.
 * @param p_context Unused lwIP callback cookie.
 * @param p_data Borrowed fragment, readable for len bytes during this call.
 * @param len Fragment size in bytes, possibly zero.
 * @param flags The lwIP last-fragment flag controls dispatch.
 * @details lwIP calls this after the topic announcement on the same network
 * owner. A truncated or length-mismatched message is dropped. The
 * application callback borrows the copied data until it returns.
 */
static void incoming_data_cb (void * p_context, const u8_t * p_data, u16_t len,
                              u8_t flags)
{
    u16_t byte_index;

    (void)p_context;

    for (byte_index = 0; byte_index < len; byte_index++)
    {
        if ((int)(g_mqtt_state.rx_len) < MQTT_UTK_RX_DATA_MAX)
        {
            g_rx_data[g_mqtt_state.rx_len++] = p_data[byte_index];
        }
        else
        {
            g_mqtt_state.b_is_rx_truncated = true;
        }
    }

    /* The last-fragment mask is nonnegative. Interpret it as unsigned
     * while testing it against the received byte. */
    if ((flags & (unsigned)MQTT_DATA_FLAG_LAST) != 0U)
    {
        if (g_mqtt_state.rx_len != g_mqtt_state.rx_expected)
        {
            g_mqtt_state.b_is_rx_truncated = true;
        }

        /* The stored 0..17 count fits the monitor's unsigned argument. */
        tm_printf((UB *)"mqtt: RX [%s] %u binary bytes%s\n", g_rx_topic,
                  (unsigned)g_mqtt_state.rx_len,
                  g_mqtt_state.b_is_rx_truncated ? " (rejected)" : "");

        if (!g_mqtt_state.b_is_rx_truncated)
        {
            mqtt_utk_app_receive(g_rx_topic, g_rx_data, g_mqtt_state.rx_len);
        }
    }
}

/** @brief Report the lwIP result for subscription to the configured telemetry
 * topic.
 */
static void subscribe_done_cb (void * p_context, err_t result)
{
    (void)p_context;

    if (ERR_OK == result)
    {
        tm_printf((UB *)"mqtt: subscribed to " MQTT_TOPIC "\n");
    }
    else
    {
        tm_printf((UB *)"mqtt: subscribe failed, err=%d\n", (int)result);
    }
}

/** @brief Report the lwIP result for subscription to the command topic.
 */
static void command_subscribe_done_cb (void * p_context, err_t result)
{
    (void)p_context;

    if (ERR_OK == result)
    {
        tm_printf((UB *)"mqtt: subscribed to " MQTT_UTK_COMMAND_TOPIC "\n");
    }
    else
    {
        tm_printf((UB *)"mqtt: command subscribe failed, err=%d\n",
                  (int)result);
    }
}

/** @brief Print a diagnostic only when the heartbeat publish callback reports
 * an error.
 */
static void publish_done_cb (void * p_context, err_t result)
{
    (void)p_context;

    if (result != ERR_OK)
    {
        tm_printf((UB *)"mqtt: publish failed, err=%d\n", (int)result);
    }
}

/** @brief On a connected client, obtain and publish the current status or
 * fallback heartbeat and schedule the next heartbeat according to the
 * adapter policy.
 */
static void heartbeat_timeout (void * p_context)
{
    UB       payload[MQTT_UTK_APP_PAYLOAD_MAX];
    uint16_t len;

    (void)p_context;

    if (mqtt_client_is_connected(gp_client))
    {
        len = mqtt_utk_app_status(payload, sizeof(payload));

        if ((len > 0U) && (len <= sizeof(payload)))
        {
            /* The buffer check bounds len. uint16_t and u16_t have the
             * same unsigned 16-bit range. This heartbeat ignores the
             * submit return and keeps its periodic schedule. */
            (void)mqtt_publish(gp_client, MQTT_TOPIC, payload, (u16_t)len,
                               0 /* qos */, 0 /* retain */, publish_done_cb,
                               NULL);
        }
    }

    /* Re-arms itself every call -- mqtt.c's connection callback (below)
     * only starts this chain once, on the transition into MQTT_CONNECT_
     * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
    sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

/** @brief Re-enter the MQTT connection startup path after the retry timer
 * expires.
 */
static void retry_timeout (void * p_context)
{
    (void)p_context;
    g_mqtt_state.b_is_connecting = false;
    mqtt_utk_start();
}

/** @brief Notify the application of link transitions. On acceptance install
 * receive handling, subscribe and replace the heartbeat timer. On
 * disconnect retain pending application data and schedule
 * reconnection.
 */
static void connection_cb (mqtt_client_t * p_client, void * p_context,
                           mqtt_connection_status_t status)
{
    (void)p_client;
    (void)p_context;

    if ((unsigned int)MQTT_CONNECT_ACCEPTED == status)
    {
        err_t err;
        mqtt_utk_app_connection(1);

        /* No longer *connecting* -- clear the guard so that if this
         * connection later drops, the poll loop's mqtt_utk_start() call
         * is allowed to reconnect. Leaving it set would wedge the client
         * permanently after the first disconnect. */
        g_mqtt_state.b_is_connecting       = false;
        g_mqtt_state.b_has_inflight_packet = false;

        tm_printf((UB *)"mqtt: connected to " MQTT_BROKER_IP "\n");

        /* Subscribe to the same topic this client publishes to, so every
         * heartbeat comes back from the broker and is printed by
         * incoming_data_cb(). A matching RX for each TX is end-to-end
         * proof of the round trip without needing mosquitto_sub on the
         * broker host. Callbacks must be registered before subscribing,
         * or the first delivery has nowhere to go. */
        mqtt_set_inpub_callback(gp_client, incoming_publish_cb,
                                incoming_data_cb, NULL);

        err = mqtt_subscribe(gp_client, MQTT_TOPIC, 0 /* qos */,
                             subscribe_done_cb, NULL);

        if (err != ERR_OK)
        {
            tm_printf((UB *)"mqtt: mqtt_subscribe() failed, err=%d\n",
                      (int)err);
        }

        err = mqtt_subscribe(gp_client, MQTT_UTK_COMMAND_TOPIC, 1 /* qos */,
                             command_subscribe_done_cb, NULL);

        if (err != ERR_OK)
        {
            tm_printf((UB *)"mqtt: command mqtt_subscribe() failed, err=%d\n",
                      (int)err);
        }

        /* Drop any chain left over from a previous connection before
         * arming a new one. Each reconnect would otherwise add another
         * self-perpetuating heartbeat, and every one of them holds a
         * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
        sys_untimeout(heartbeat_timeout, NULL);
        sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
    }
    else
    {
        mqtt_utk_app_connection(0);
        /* lwIP clears requests without completion callbacks on close.
         * Keep our application packet for the next connection and schedule
         * the existing connection retry without adding a publish timer. */
        tm_printf((UB *)"mqtt: connection status %d, retrying in %ds\n",
                  (int)status, MQTT_UTK_RETRY_MS / 1000);
        g_mqtt_state.b_has_inflight_packet = false;
        sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
    }
}

/* End of file. */

