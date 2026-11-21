/** @file host_main.c
 * @brief Exercise MQTT telemetry, command handling and debug publication
 * fairness.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "buddy1.h"
#include "mqtt_utk.h"
#include "host_main.h"
#include "lwip/ip_addr.h"
#include <assert.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define DISCONNECTED_POLL_COUNT 20U

#define QUEUED_REPORTS 4U
#define FIRST_DRAIN_POLLS 24U
#define CONTROL_REPLY_POLLS 5U
#define RECONNECT_DRAIN_POLLS 60U
#define FINAL_DRAIN_POLLS 12U
#define NO_CLOCK_RETRY_POLLS 1000U
#define FAILED_LINK_RETRY_POLLS 1000U
#define REJECTED_UPLOAD_POLLS 1000U
#define SAMPLES_PER_PRODUCER 1000U
#define MIN_ECHO_MM 30
#define MAX_ECHO_MM 4000U
#define MAX_EDGE_OFFSET_MM 4000
#define MAX_TILT_TENTHS_DEG 900
#define MAX_TEST_RATE_TENTHS 360
#define MIN_SCAN_ANGLE_DEG 30
#define MAX_SCAN_ANGLE_DEG 150U
#define HUMP_HEIGHT_CODES 512U
#define MOTION_PACKET_BYTES 4U
#define MAX_MOTION_SPEED_MMS 510
#define HEADING_CODE_COUNT 360U
#define BYTE_CODE_COUNT 256U
#define TEST_FAULT_BIT_COUNT 8U

typedef struct
{
    unsigned capacity;
    unsigned size;
    unsigned head;
    unsigned tail;
    unsigned count;
    UB       slot_bytes[4][20];
} host_message_buffer_t;

static unsigned        g_failures;
static unsigned        g_delayed_logs;
static unsigned        g_discarded_logs;
static pthread_mutex_t g_cache_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_queue_lock = PTHREAD_MUTEX_INITIALIZER;

static host_message_buffer_t g_buffers[2];

static unsigned g_created;
static ID       gh_observed_outbox;
static ID       gh_observed_commands;

/* Fake sleep for the command task's reply retry: counts calls and, when
 * sleep_drains is set, lets the "network" take one queued message. */
static unsigned g_reply_sleeps;
static unsigned g_sleep_drains;

static mqtt_client_t             g_fake_client;
static mqtt_host_connection_fn_t gp_connected = NULL;
static mqtt_host_publish_fn_t    gp_incoming  = NULL;
static mqtt_host_data_fn_t       gp_fragment  = NULL;

static void ( * gp_heartbeat)(void * p_context) = NULL;

static unsigned            g_publish_count;
static char                g_published_topic[64];
static UB                  g_published[MQTT_UTK_DEBUG_PAYLOAD_MAX];
static u16_t               g_published_len;
static u8_t                g_published_qos;
static u8_t                g_published_retain;
static unsigned            g_publish_attempts;
static u32_t               g_clock_ms;
static err_t               g_publish_result;
static int                 g_defer_completion;
static mqtt_host_done_fn_t gp_deferred_done = NULL;
static void *              gp_deferred_arg  = NULL;
static T_MQTT_UTK_APP_MSG  g_attempted;
static UB                  g_attempted_bytes[MQTT_UTK_DEBUG_PAYLOAD_MAX];
static unsigned            g_heartbeat_pending;

/* Binary diagnostic hook: payloads contain embedded zero bytes, not strings. */
static const uint8_t * g_debug_queue[16];
static unsigned        g_debug_size[16];
static const uint8_t   g_debug_summary[16] = {1};
#if APP_IR_DEBUG
static const uint8_t g_debug_end[6] = {1, 0, 0, 0, 0, 3};
#endif
static unsigned g_debug_head;
static unsigned g_debug_count;
static unsigned g_debug_asks;

#if APP_IR_DEBUG
static unsigned g_ir_frames;
static unsigned g_control_frames;
#endif

#if APP_IMU_DEBUG
static unsigned g_imu_batches;
#endif

static void empty_outbox (void);

static void poll_after (u32_t delay_ms);

static unsigned outbox_count (void);

static void complete_publish (err_t result);

static void test_debug_lines (void);

static void queue_debug (unsigned count, const uint8_t * p_bytes,
                         unsigned size);

static void test_capture_fairness (void);

static void test_capture_event_priority (void);

static void test_capture_mixed_retries (void);

#if APP_IR_DEBUG
static void test_ir_preview (void);

static void test_control_preview (void);

static void test_control_preview_fairness (unsigned completion_ms);

static void test_preview_waits_for_retry (void);

#endif
#if APP_IMU_DEBUG
static void test_imu_stream (void);

#endif
static void test_publish_backpressure (void);

static void test_pose (void);

static int same_avoidance (const telemetry_codec_avoidance_t * p_first,
                           const telemetry_codec_avoidance_t * p_second);

static int same_scan (const telemetry_codec_scan_t * p_first,
                      const telemetry_codec_scan_t * p_second);

static void test_buddy5 (void);

static int same_terrain (const telemetry_codec_terrain_t * p_first,
                         const telemetry_codec_terrain_t * p_second);

static void test_terrain (void);

static int same_sample (const telemetry_codec_scan_sample_t * p_first,
                        const telemetry_codec_scan_sample_t * p_second);

static void test_scan_sample (void);

static void test_reply_retry (void);

static void * producer (void * p_producer_id);

static void emit_vectors (void);

static void test_terrain_widths (void);

/** @brief Allocate a simulated message buffer and derive its capacity from
 * the requested message size and aligned slot overhead.
 */
ID tk_cre_mbf (const T_CMBF * p_message_buffer_config)
{
    unsigned buffer_index = g_created++;
    assert(buffer_index < 2U && p_message_buffer_config->maxmsz <= 20);
    /* clang-format off */
    g_buffers[buffer_index].size     =
        (unsigned)p_message_buffer_config->maxmsz;
    g_buffers[buffer_index].capacity =
        (unsigned)p_message_buffer_config->bufsz /
        (4U + ((g_buffers[buffer_index].size + 3U) & ~3U));
    /* clang-format on */

    if (sizeof(buddy1_telemetry_msg_t) == g_buffers[buffer_index].size)
    {
        gh_observed_outbox = (ID)buffer_index + 1;
    }

    if (sizeof(buddy1_command_msg_t) == g_buffers[buffer_index].size)
    {
        gh_observed_commands = (ID)buffer_index + 1;
    }

    return (ID)buffer_index + 1;
}

/** @brief Verify the priority-inheritance attribute and return the configured
 * fake mutex ID.
 */
ID tk_cre_mtx (const T_CMTX * p_mutex_config)
{
    assert(TA_INHERIT == p_mutex_config->mtxatr);
    return 3;
}

/** @brief Accept deletion of either registered message buffer.
 */
ER tk_del_mbf (ID h_message_buffer)
{
    assert(1 == h_message_buffer || 2 == h_message_buffer);
    return E_OK;
}

/** @brief Lock the simulated snapshot mutex. Return an object error for any
 * other ID and a system error on pthread failure.
 */
ER tk_loc_mtx (ID h_mutex, TMO timeout)
{
    (void)timeout;

    if (h_mutex != 3)
    {
        return E_OBJ;
    }

    return pthread_mutex_lock(&g_cache_lock) == 0 ? E_OK : E_SYS;
}

/** @brief Release the host cache mutex and translate a pthread failure to the
 * kernel error code.
 */
ER tk_unl_mtx (ID h_mutex)
{
    assert(3 == h_mutex);
    return pthread_mutex_unlock(&g_cache_lock) == 0 ? E_OK : E_SYS;
}

/** @brief Append one fixed-size message under the queue lock or return the
 * fixture's full-buffer error.
 */
ER tk_snd_mbf (ID h_message_buffer, const void * p_data, INT length,
               TMO timeout)
{
    unsigned buffer_index = (unsigned)(h_message_buffer - 1);
    assert(buffer_index < 2U && TMO_POL == timeout &&
           (unsigned)length == g_buffers[buffer_index].size);
    (void)pthread_mutex_lock(&g_queue_lock);

    if (g_buffers[buffer_index].count == g_buffers[buffer_index].capacity)
    {
        (void)pthread_mutex_unlock(&g_queue_lock);
        return E_TMOUT;
    }

    memcpy(g_buffers[buffer_index].slot_bytes[g_buffers[buffer_index].head],
           p_data, (size_t)length);
    g_buffers[buffer_index].head =
        (g_buffers[buffer_index].head + 1U) % g_buffers[buffer_index].capacity;
    ++g_buffers[buffer_index].count;
    (void)pthread_mutex_unlock(&g_queue_lock);
    return E_OK;
}

/** @brief Count one reply retry delay. When draining is enabled, remove an
 * outbox item under the queue lock if available.
 */
ER tk_dly_tsk (TMO time)
{
    unsigned buffer_index = 0;
    assert(BUDDY1_REPLY_RETRY_MS == time);
    ++g_reply_sleeps;

    if (g_sleep_drains)
    {
        (void)pthread_mutex_lock(&g_queue_lock);

        if (g_buffers[buffer_index].count)
        {
            g_buffers[buffer_index].tail = (g_buffers[buffer_index].tail + 1U) %
                                           g_buffers[buffer_index].capacity;
            --g_buffers[buffer_index].count;
        }

        (void)pthread_mutex_unlock(&g_queue_lock);
    }

    return E_OK;
}

/** @brief Report the next message size under the queue lock, using zero for
 * an empty buffer.
 */
ER tk_ref_mbf (ID h_message_buffer, T_RMBF * p_buffer_info)
{
    unsigned buffer_index = (unsigned)(h_message_buffer - 1);
    assert(buffer_index < 2U && p_buffer_info);
    (void)pthread_mutex_lock(&g_queue_lock);
    p_buffer_info->msgsz =
        g_buffers[buffer_index].count ? (INT)g_buffers[buffer_index].size : 0;
    (void)pthread_mutex_unlock(&g_queue_lock);
    return E_OK;
}

/** @brief Pop the oldest fixed-size message under the queue lock. Return a
 * timeout when no message is available.
 */
INT tk_rcv_mbf (ID h_message_buffer, void * p_data, TMO timeout)
{
    unsigned buffer_index = (unsigned)(h_message_buffer - 1);
    assert(buffer_index < 2U && TMO_POL == timeout);
    (void)pthread_mutex_lock(&g_queue_lock);

    if (!g_buffers[buffer_index].count)
    {
        (void)pthread_mutex_unlock(&g_queue_lock);
        return E_TMOUT;
    }

    memcpy(p_data,
           g_buffers[buffer_index].slot_bytes[g_buffers[buffer_index].tail],
           g_buffers[buffer_index].size);
    g_buffers[buffer_index].tail =
        (g_buffers[buffer_index].tail + 1U) % g_buffers[buffer_index].capacity;
    --g_buffers[buffer_index].count;
    (void)pthread_mutex_unlock(&g_queue_lock);
    return (INT)g_buffers[buffer_index].size;
}

/** @brief Return the fixture task ID without scheduling an entry point.
 */
ID app_start_task (void ( * p_task_entry)(INT start_code, void * p_extra_info),
                   PRI priority, UW core, const char * p_name)
{
    (void)p_task_entry;
    (void)priority;
    (void)core;
    (void)p_name;
    return 4;
}

/** @brief Write a bounded diagnostic line, count matching discard messages
 * and return the formatter result.
 */
int tm_printf (const UB * p_format, ...)
{
    char    text[512];
    va_list args;
    int     len;
    va_start(args, p_format);
    len = vsnprintf(text, sizeof(text), (const char *)p_format, args);
    va_end(args);

    if (strstr(text, " FAIL "))
    {
        ++g_failures;
    }

    if (strstr(text, "application publish delayed"))
    {
        ++g_delayed_logs;
    }

    if (strstr(text, "application packet discarded"))
    {
        ++g_discarded_logs;
    }

    fputs(text, stdout);
    return len;
}

/** @brief Return the fixture clock in the millisecond units expected by lwIP.
 */
u32_t sys_now (void)
{
    return g_clock_ms;
}

/** @brief Return the fixture-owned MQTT client reused across connection
 * attempts.
 */
mqtt_client_t * mqtt_client_new (void)
{
    return &g_fake_client;
}

/** @brief Return whether the supplied client exists and has its connected
 * flag set.
 */
int mqtt_client_is_connected (mqtt_client_t * p_client)
{
    return p_client && (p_client->connected);
}

/** @brief Save the connection callback for later injection and return
 * success without contacting a broker.
 */
err_t mqtt_client_connect (
    mqtt_client_t * p_client, const void * p_address, u16_t port,
    mqtt_host_connection_fn_t p_connection_callback, void * p_context,
    const struct mqtt_connect_client_info_t * p_client_info)
{
    (void)p_client;
    (void)p_address;
    (void)port;
    (void)p_context;
    (void)p_client_info;
    gp_connected = p_connection_callback;
    return ERR_OK;
}

/** @brief Store the incoming publication and fragment callbacks for the test
 * driver.
 */
void mqtt_set_inpub_callback (mqtt_client_t *        p_client,
                              mqtt_host_publish_fn_t p_incoming,
                              mqtt_host_data_fn_t p_fragment, void * p_context)
{
    (void)p_client;
    (void)p_context;
    gp_incoming = p_incoming;
    gp_fragment = p_fragment;
}

/** @brief Acknowledge the simulated subscription immediately through the
 * supplied callback.
 */
err_t mqtt_subscribe (mqtt_client_t * p_client, const char * p_topic, u8_t qos,
                      mqtt_host_done_fn_t p_done_callback, void * p_context)
{
    (void)p_client;
    (void)p_topic;
    (void)qos;
    p_done_callback(p_context, ERR_OK);
    return ERR_OK;
}

/** @brief Capture the attempted topic and payload. Apply the injected submit
 * result and either defer or invoke a successful completion
 * callback.
 */
err_t mqtt_publish (mqtt_client_t * p_client, const char * p_topic,
                    const void * p_payload, u16_t len, u8_t qos, u8_t retain,
                    mqtt_host_done_fn_t p_done_callback, void * p_context)
{
    assert(p_client->connected && len <= sizeof(g_published));
    ++g_publish_attempts;
    strcpy(g_attempted.topic, p_topic);
    memcpy(g_attempted.payload, p_payload,
           len < sizeof g_attempted.payload ? len : sizeof g_attempted.payload);
    g_attempted.payload_len = len;
    g_attempted.qos         = qos;
    g_attempted.retain      = retain;
    memcpy(g_attempted_bytes, p_payload, len);

    if (g_publish_result != ERR_OK)
    {
        return g_publish_result;
    }

    strcpy(g_published_topic, p_topic);
    memcpy(g_published, p_payload, len);
    g_published_len    = len;
    g_published_qos    = qos;
    g_published_retain = retain;
    ++g_publish_count;

    if (g_defer_completion)
    {
        assert(!gp_deferred_done);
        gp_deferred_done = p_done_callback;
        gp_deferred_arg  = p_context;
    }
    else
    {
        p_done_callback(p_context, ERR_OK);
    }

    return ERR_OK;
}

/** @brief Return a successful parse and write the fixture's zero address
 * regardless of the supplied text.
 */
int ip4addr_aton (const char * p_text, ip_addr_t * p_address)
{
    (void)p_text;
    *p_address = 0;
    return 1;
}

/** @brief Record a five-second heartbeat callback and assert that another
 * heartbeat is not already pending.
 */
void sys_timeout (unsigned milliseconds, void ( * p_callback)(void * p_context),
                  void * p_context)
{
    (void)p_context;

    if (5000U == milliseconds)
    {
        assert(!g_heartbeat_pending);
        g_heartbeat_pending = 1;
        gp_heartbeat        = p_callback;
    }
}

/** @brief Clear the pending heartbeat flag when the callback matches the
 * registered heartbeat.
 */
void sys_untimeout (void ( * p_callback)(void * p_context), void * p_context)
{
    (void)p_context;

    if (p_callback == gp_heartbeat)
    {
        g_heartbeat_pending = 0;
    }
}

/** @brief Copy the next staged binary debug payload into the supplied
 * buffer. Return its byte length or zero when exhausted.
 */
unsigned int mqtt_utk_debug_take (uint8_t * p_bytes, unsigned int size)
{
    unsigned debug_length;
    ++g_debug_asks;

    if (g_debug_head >= g_debug_count)
    {
        return 0;
    }

    debug_length = g_debug_size[g_debug_head];
    assert(debug_length <= size);
    memcpy(p_bytes, g_debug_queue[g_debug_head++], debug_length);
    return debug_length;
}

#if APP_IR_DEBUG
/** @brief Produce a 70-byte fixture control frame while frames remain, or
 * return zero without writing.
 */
unsigned int mqtt_utk_control_debug_take (uint8_t * p_bytes, unsigned int size)
{
    if (!g_control_frames)
    {
        return 0U;
    }

    assert(size >= 70U);
    memset(p_bytes, 0, 70U);
    p_bytes[0]  = 1U;
    p_bytes[69] = 0x3fU;
    --g_control_frames;
    return 70U;
}

/** @brief Produce a 16-byte fixture IR frame while frames remain, or return
 * zero without writing.
 */
unsigned int mqtt_utk_ir_debug_take (uint8_t * p_bytes, unsigned int size)
{
    if (!g_ir_frames)
    {
        return 0U;
    }

    assert(size >= 16U);
    memset(p_bytes, 0, 16U);
    p_bytes[0]  = 1U;
    p_bytes[15] = 0x3fU;
    g_ir_frames--;
    return 16U;
}

#endif
#if APP_IMU_DEBUG
/** @brief Write one synthetic IMU batch into an exactly sized buffer. Return
 * zero when the batches are exhausted.
 */
unsigned int mqtt_utk_imu_debug_take (uint8_t * p_bytes, unsigned int size)
{
    if (0U == g_imu_batches)
    {
        return 0U;
    }

    assert(MQTT_UTK_IMU_DEBUG_MAX == size);
    memset(p_bytes, 0, size);
    p_bytes[0]         = 1U;
    p_bytes[1]         = 8U;
    p_bytes[size - 1U] = 0x5aU;
    --g_imu_batches;
    return size;
}

#endif
#ifdef TEST_MQTT_RECEIVE
/** @brief Return the command buffer allocated by the simulated kernel. */
ID host_main_command_buffer (void)
{
    assert(gh_observed_commands > 0);
    return gh_observed_commands;
}

/** @brief Deliver a map publish and its two fragments to registered callbacks.
 * @pre The suite has registered both callbacks and split is at most actual.
 */
void host_main_receive (uint32_t announced, uint16_t actual, uint16_t split,
                        const uint8_t * p_bytes)
{
    gp_incoming(NULL, "car/cmd/barcode_map", announced);
    gp_fragment(NULL, p_bytes, split, 0U);
    /* The subtraction is nonnegative and bounded by the uint16_t actual. */
    gp_fragment(NULL, p_bytes + split, (uint16_t)(actual - split),
                MQTT_DATA_FLAG_LAST);
}

#endif
/** @brief Exercise the host MQTT service and leave its callbacks registered. */
int host_main_run (void)
{
    telemetry_codec_snapshot_t snapshot = {0};
    telemetry_codec_snapshot_t decoded;
    buddy1_command_msg_t       command;
    UB                         packet[32];
    UB                         map_packet[4];
    telemetry_codec_map_t      map = {'A', 'B', 'C', 'D'};
    pthread_t                  workers[8];
    unsigned                   index;
    unsigned                   before;
    int                        speed;
    assert(subsystem1_init() == E_OK);
    assert(buddy1_config_snapshot_close(NULL) == E_PAR);
    assert(buddy1_configuration_allowed());
    assert(buddy1_barcode_action('D') == (unsigned)BUDDY1_ACTION_UTURN);
    subsystem1_test();
    assert(0U == g_failures);
    {
        telemetry_codec_map_t frozen;
        assert(buddy1_config_snapshot_close(&frozen) == E_OK);
        assert(!buddy1_configuration_allowed());
        assert(buddy1_barcode_action(frozen.forward) ==
               (unsigned)BUDDY1_ACTION_FORWARD);
        assert(buddy1_barcode_action(frozen.left) ==
               (unsigned)BUDDY1_ACTION_LEFT);
        assert(buddy1_barcode_action(frozen.right) ==
               (unsigned)BUDDY1_ACTION_RIGHT);
        assert(buddy1_barcode_action(frozen.uturn) ==
               (unsigned)BUDDY1_ACTION_UTURN);
        assert(buddy1_apply_barcode_map(&map) == E_OBJ);
    }
    /* Independent expected rounding across the complete speed range. */
    for (speed = -MAX_MOTION_SPEED_MMS; speed <= MAX_MOTION_SPEED_MMS; ++speed)
    {
        int expected =
            speed >= 0 ? ((speed + 1) / 2) * 2 : -(((-speed + 1) / 2) * 2);
        snapshot.left_mms  = (int16_t)speed;
        snapshot.right_mms = (int16_t)-speed;
        assert(buddy1_encode(BUDDY1_TOPIC_MOTION, &snapshot, packet,
                             sizeof(packet)) == 4U);
        assert(buddy1_decode(BUDDY1_TOPIC_MOTION, packet, 4, &decoded));
        assert(decoded.left_mms == expected && decoded.right_mms == -expected);
    }

    for (index = 0; index < HEADING_CODE_COUNT; ++index)
    {
        snapshot.heading_deg = (H)index;
        assert(buddy1_encode(BUDDY1_TOPIC_HEADING, &snapshot, packet,
                             sizeof(packet)) == 3U);
        assert(buddy1_decode(BUDDY1_TOPIC_HEADING, packet, 3, &decoded) &&
               decoded.heading_deg == (H)index);
    }

    for (index = BUDDY1_STATE_STOPPED; index <= (unsigned)BUDDY1_STATE_ERROR;
         ++index)
    {
        snapshot.robot_state = (telemetry_codec_state_t)index;
        assert(buddy1_encode(BUDDY1_TOPIC_STATE, &snapshot, packet,
                             sizeof(packet)) == 2U);
        assert(buddy1_decode(BUDDY1_TOPIC_STATE, packet, 2, &decoded) &&
               decoded.robot_state == snapshot.robot_state);
    }

    for (index = 0; index < BYTE_CODE_COUNT; ++index)
    {
        packet[0] = BUDDY1_WIRE_VERSION;
        packet[1] = (UB)index;
        assert(telemetry_codec_payload_valid(BUDDY1_TOPIC_BARCODE, packet, 2) ==
               ((index & 31U) <= 25U &&
                (index >> 5) <= (unsigned)BUDDY1_ACTION_UTURN));
    }
    /* All fault combinations cross all three layouts, including IR in the
     * former snapshot padding bit and heartbeat's most significant bit. */
    snapshot.faults_valid = 1;

    for (index = 0; index <= BUDDY1_FAULT_MASK; ++index)
    {
        const UB topics[] = {BUDDY1_TOPIC_FAULTS, BUDDY1_TOPIC_SNAPSHOT,
                             BUDDY1_TOPIC_STATUS};
        unsigned topic_index;
        snapshot.faults = (UB)index;

        for (topic_index = 0; topic_index < sizeof(topics); ++topic_index)
        {
            unsigned len = buddy1_encode(topics[topic_index], &snapshot, packet,
                                         sizeof(packet));
            assert(len &&
                   buddy1_decode(topics[topic_index], packet, len, &decoded));
            assert(decoded.faults_valid && decoded.faults == index);
            packet[0] = 1;
            assert(!buddy1_decode(topics[topic_index], packet, len, &decoded));
        }
    }

    for (index = 0; index < TEST_FAULT_BIT_COUNT; ++index)
    {
        assert(pthread_create(&workers[index], NULL, producer,
                              (void *)(uintptr_t)index) == 0);
    }

    for (index = 0; index < TEST_FAULT_BIT_COUNT; ++index)
    {
        assert(pthread_join(workers[index], NULL) == 0);
    }

    assert(buddy1_snapshot(&decoded) && 279 == decoded.heading_deg &&
           999 == (int)decoded.obstacle_distance_mm &&
           1000 == (int)decoded.obstacle_width_mm &&
           BUDDY1_FAULT_MASK == decoded.faults);
    empty_outbox();

    mqtt_utk_start();
    g_fake_client.connected = 1;
    gp_connected(&g_fake_client, NULL, MQTT_CONNECT_ACCEPTED);
    assert(buddy1_map_pack(&map, map_packet));
    /* Force zero bytes and fragmented delivery through the ACTUAL mqtt_utk.c.
     */
    gp_incoming(NULL, "car/cmd/barcode_map", 4);
    gp_fragment(NULL, map_packet, 2, 0);
    gp_fragment(NULL, map_packet + 2, 2, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(gh_observed_commands, &command, TMO_POL) ==
           (INT)sizeof(command));
    assert(memcmp(command.payload, map_packet, 4) == 0);
    gp_incoming(NULL, "car/cmd/snapshot", 1);
    packet[0] = BUDDY1_WIRE_VERSION;
    gp_fragment(NULL, packet, 1, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(gh_observed_commands, &command, TMO_POL) ==
           (INT)sizeof(command));
    assert(buddy1_command_process(&command) == E_OK);
    before = g_publish_count;
    mqtt_utk_poll();
    assert(g_publish_count == before + 1U);
    assert(BUDDY1_PAYLOAD_MAX == g_published_len &&
           strcmp(g_published_topic, "car/reply/snapshot") == 0 &&
           buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, g_published, g_published_len,
                         &decoded));
    gp_incoming(NULL, "car/cmd/snapshot", 2);
    gp_fragment(NULL, packet, 1, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(gh_observed_commands, &command, TMO_POL) == E_TMOUT);
    gp_incoming(NULL, "car/cmd/barcode_map", 17);
    gp_fragment(NULL, map_packet, 4, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(gh_observed_commands, &command, TMO_POL) == E_TMOUT);
    gp_incoming(
        NULL,
        "car/cmd/snapshot/this/topic/is/intentionally/far/too/long/to/be/"
        "accepted",
        1);
    gp_fragment(NULL, packet, 1, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(gh_observed_commands, &command, TMO_POL) == E_TMOUT);
    assert(mqtt_utk_app_link(0));
    assert(buddy1_snapshot(&decoded) && (decoded.faults & BUDDY1_FAULT_WIFI));
    assert(mqtt_utk_app_link(1));
    assert(buddy1_snapshot(&decoded) &&
           decoded.faults == (BUDDY1_FAULT_MASK & ~BUDDY1_FAULT_WIFI));
    empty_outbox();
    assert(buddy1_report_motion(0, 0) == E_OK);
    poll_after(20);
    assert(4 == (int)g_published_len && 0 == (int)g_published[1] &&
           0 == (int)g_published[2] && 0 == (int)g_published[3]);
    g_heartbeat_pending = 0;
    gp_heartbeat(NULL);
    assert(2 == (int)g_published_len &&
           buddy1_decode(BUDDY1_TOPIC_STATUS, g_published, g_published_len,
                         &decoded));
    test_publish_backpressure();
    test_debug_lines();
    test_capture_fairness();
    test_capture_event_priority();
    test_capture_mixed_retries();
    test_pose();
    test_terrain();
    test_scan_sample();
    test_reply_retry();
    test_buddy5();
    test_terrain_widths();
#if APP_IR_DEBUG
    test_ir_preview();
    test_control_preview();
    test_control_preview_fairness(0);
    test_control_preview_fairness(250);
    test_preview_waits_for_retry();
#endif
#if APP_IMU_DEBUG
    test_imu_stream();
#endif
    emit_vectors();
    puts("HOST PASS: binary FIFO, full speed range, 64 fault masks, eight "
         "concurrent producers, real MQTT callbacks and bridge");
    return 0;
}

#ifndef TEST_MQTT_RECEIVE
/** @brief Return the outcome of the shared telemetry and network host
 * fixture.
 */
int main (void)
{
    return host_main_run();
}

#endif

/** @brief Drain all messages available from the Buddy 1 telemetry outbox.
 */
static void empty_outbox (void)
{
    buddy1_telemetry_msg_t msg;

    while (buddy1_outbox_take(&msg))
    {
        /* Each take removes one queued publication. */
    }
}

/** @brief Advance simulated network time by the supplied milliseconds and
 * poll the MQTT integration once.
 */
static void poll_after (u32_t delay_ms)
{
    g_clock_ms += delay_ms;
    mqtt_utk_poll();
}

/** @brief Return the number of messages currently queued in the simulated
 * telemetry outbox.
 */
static unsigned outbox_count (void)
{
    assert(gh_observed_outbox > 0);
    return g_buffers[gh_observed_outbox - 1].count;
}

/** @brief Remove the pending publish callback from its slot and deliver the
 * requested completion code.
 */
static void complete_publish (err_t result)
{
    mqtt_host_done_fn_t p_done_callback = gp_deferred_done;
    void *              p_context       = gp_deferred_arg;
    assert(p_done_callback);
    gp_deferred_done = NULL;
    gp_deferred_arg  = NULL;
    p_done_callback(p_context, result);
}

/** @brief Exercise exact-length binary capture publication, ordering, pacing
 * and retry while preserving embedded zero bytes.
 */
static void test_debug_lines (void)
{
    static uint8_t longest[MQTT_UTK_DEBUG_PAYLOAD_MAX];
    unsigned       count;
    memset(longest, 0xa5, sizeof longest);
    longest[10] = 0;
    empty_outbox();
    poll_after(1000);
    g_debug_queue[0] = g_debug_summary;
    g_debug_size[0]  = sizeof g_debug_summary;
    g_debug_queue[1] = longest;
    g_debug_size[1]  = sizeof longest;
    g_debug_head     = 0;
    g_debug_count    = 2;
    /* Telemetry first: a queued report goes out before any debug line. */
    assert(buddy1_report_heading(12) == E_OK);
    count = g_publish_count;
    poll_after(20);
    assert(g_publish_count == count + 1U &&
           strcmp(g_published_topic, "car/debug/barcode") != 0);
    /* Then one debug line per paced slot, on the debug topic, QoS 0. */
    poll_after(20);
    assert(g_publish_count == count + 2U &&
           strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC) == 0 &&
           0 == (int)g_published_qos && 0 == (int)g_published_retain &&
           g_published_len == g_debug_size[0] &&
           memcmp(g_published, g_debug_queue[0], g_published_len) == 0);
    /* A transient rejection keeps the same line and retries it later. */
    g_publish_result = ERR_MEM;
    poll_after(20);
    assert(g_publish_count == count + 2U && 2U == g_debug_head);
    g_publish_result = ERR_OK;
    poll_after(100);
    assert(g_publish_count == count + 3U && 164U == g_published_len &&
           memcmp(g_published, longest, sizeof longest) == 0);
    /* Nothing left: no more publishes. */
    poll_after(100);
    poll_after(100);
    assert(g_publish_count == count + 3U);
    g_debug_count = 0;
    g_debug_head  = 0;
    puts("HOST PASS: exact 164-byte binary capture, event priority, zero bytes "
         "preserved, paced and retried");
}

/** @brief Stage the requested number of identical fixture payloads for
 * subsequent debug reads.
 */
static void queue_debug (unsigned count, const uint8_t * p_bytes, unsigned size)
{
    assert(count <= sizeof g_debug_queue / sizeof g_debug_queue[0]);
    g_debug_head  = 0;
    g_debug_count = count;

    for (unsigned queue_index = 0; queue_index < count; ++queue_index)
    {
        g_debug_queue[queue_index] = p_bytes;
        g_debug_size[queue_index]  = size;
    }
}

/** @brief Check progress for ordinary telemetry and barcode capture under
 * contention with synchronous completions.
 */
static void test_capture_fairness (void)
{
    telemetry_codec_snapshot_t decoded;
    telemetry_codec_terrain_t  terrain = {0};
    T_MQTT_UTK_APP_MSG         message;
    unsigned                   count;
    unsigned                   reports = 0;
    empty_outbox();
    /* Scheduling metadata is local: only recurring samples yield a turn. */
    assert(buddy1_report_motion(2, 4) == E_OK);
    assert(mqtt_utk_app_take(&message) && message.routine);
    assert(buddy1_report_pose(12, 34) == E_OK);
    assert(mqtt_utk_app_take(&message) && message.routine);
    assert(buddy1_report_terrain(&terrain) == E_OK);
    assert(mqtt_utk_app_take(&message) && message.routine);

    /* Start after a capture completion, then keep the four-slot app FIFO
     * replenished throughout twelve captures. Both sources must progress,
     * including with the fake port's synchronous completion callbacks. */
    queue_debug(1, g_debug_summary, sizeof g_debug_summary);
    poll_after(20);
    queue_debug(12, g_debug_summary, sizeof g_debug_summary);

    for (unsigned report_index = 1; report_index <= QUEUED_REPORTS;
         ++report_index)
    {
        assert(buddy1_report_motion((H)(2U * report_index),
                                    (H)(2U * report_index)) == E_OK);
    }

    count = g_publish_count;

    for (unsigned poll_index = 0; poll_index < FIRST_DRAIN_POLLS; ++poll_index)
    {
        poll_after(20);
        assert(g_publish_count == count + poll_index + 1U);

        if ((poll_index & 1U) == 0U)
        {
            ++reports;
            assert(!strcmp(g_published_topic, "car/telemetry/motion") &&
                   buddy1_decode(BUDDY1_TOPIC_MOTION, g_published,
                                 g_published_len, &decoded) &&
                   decoded.left_mms == (H)(2U * reports));
            assert(buddy1_report_motion((H)(2U * (reports + 4U)), 0) == E_OK);
        }
        else
        {
            assert(!strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC));
        }
    }

    assert(12U == g_debug_head && 12U == reports);
    /* The capture turn held one fresh app packet; drain that slot before
     * dropping the rest of this synthetic workload. */
    poll_after(20);
    assert(!strcmp(g_published_topic, "car/telemetry/motion") &&
           buddy1_decode(BUDDY1_TOPIC_MOTION, g_published, g_published_len,
                         &decoded) &&
           26 == decoded.left_mms);
    empty_outbox();
    g_debug_head = g_debug_count = 0;
    puts("HOST PASS: saturated FIFO and BCAP each progress every other slot, "
         "app order intact, synchronous completion safe");
}

/** @brief Check event and reply precedence over new barcode captures,
 * including QoS and application FIFO order.
 */
static void test_capture_event_priority (void)
{
    buddy1_command_msg_t snapshot = {.kind    = BUDDY1_COMMAND_SNAPSHOT,
                                     .payload = {BUDDY1_WIRE_VERSION}};
    const char * topics[]    = {"car/telemetry/faults", "car/telemetry/state",
                                "car/telemetry/barcode", "car/reply/snapshot",
                                "car/reply/barcode_map"};
    UB           map_reply[] = {BUDDY1_WIRE_VERSION, 0};
    unsigned     asks;
    empty_outbox();
    queue_debug(1, g_debug_summary, sizeof g_debug_summary);
    assert(buddy1_update_faults(BUDDY1_FAULT_WIFI, 0) == E_OK);
    assert(buddy1_report_robot_state(BUDDY1_STATE_STOPPED) == E_OK);
    assert(buddy1_report_barcode('A') == E_OK);
    assert(buddy1_command_process(&snapshot) == E_OK);
    asks = g_debug_asks;

    for (unsigned poll_index = 0; poll_index < CONTROL_REPLY_POLLS;
         ++poll_index)
    {
        poll_after(20);
        assert(!strcmp(g_published_topic, topics[poll_index]) &&
               g_debug_asks == asks);
        assert((int)g_published_qos == (poll_index >= 3U ? 1 : 0));

        if (0U == poll_index)
        {
            assert(buddy1_queue_publish(BUDDY1_TOPIC_MAP_REPLY, map_reply,
                                        sizeof map_reply) == E_OK);
        }
    }

    poll_after(20);
    assert(!strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC));

    /* Priority applies at the head: never reorder an urgent event ahead of
     * an already staged routine sample, even if that sample yields a slot. */
    assert(buddy1_report_heading(22) == E_OK);
    poll_after(20);
    queue_debug(2, g_debug_summary, sizeof g_debug_summary);
    assert(buddy1_report_motion(44, 46) == E_OK);
    assert(buddy1_update_faults(BUDDY1_FAULT_WIFI, 0) == E_OK);
    poll_after(20);
    assert(!strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC));
    poll_after(20);
    assert(!strcmp(g_published_topic, "car/telemetry/motion"));
    poll_after(20);
    assert(!strcmp(g_published_topic, "car/telemetry/faults"));
    poll_after(20);
    assert(!strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC));
    g_debug_head = g_debug_count = 0;
    puts("HOST PASS: faults/state/barcode/replies ahead of fresh captures, QoS "
         "preserved, no app FIFO reordering");
}

/** @brief Exercise retryable and permanent errors, reconnects and in-flight
 * ownership with full-size capture data.
 */
static void test_capture_mixed_retries (void)
{
    static uint8_t full[MQTT_UTK_DEBUG_PAYLOAD_MAX];
    unsigned       attempts;
    unsigned       asks;
    memset(full, 0xa6, sizeof full);
    full[15] = 0;
    empty_outbox();
    queue_debug(2, full, sizeof full);
    assert(buddy1_report_motion(48, 50) == E_OK);
    g_publish_result = ERR_MEM;
    poll_after(20);
    assert(!strcmp(g_attempted.topic, "car/telemetry/motion") &&
           0U == g_debug_head);
    assert(buddy1_report_heading(23) == E_OK);
    g_publish_result = ERR_OK;
    poll_after(100);
    assert(!strcmp(g_published_topic, "car/telemetry/motion") &&
           0U == g_debug_head);
    poll_after(20);
    assert(!strcmp(g_published_topic, "car/telemetry/heading"));

    /* Capture rejection retains all 164 bytes; a newly arrived event cannot
     * replace it. Retry pacing and only one request apply across both sources.
     */
    assert(buddy1_report_motion(60, 62) == E_OK);
    g_publish_result = ERR_MEM;
    poll_after(20);
    assert(1U == g_debug_head && g_attempted.payload_len == sizeof full &&
           !strcmp(g_attempted.topic, MQTT_UTK_DEBUG_TOPIC) &&
           !memcmp(g_attempted_bytes, full, sizeof full));
    assert(buddy1_report_heading(24) == E_OK);
    attempts = g_publish_attempts;
    asks     = g_debug_asks;
    poll_after(99);
    assert(g_publish_attempts == attempts);
    g_publish_result   = ERR_OK;
    g_defer_completion = 1;
    poll_after(1);
    assert(g_publish_attempts == attempts + 1U && outbox_count() == 1U &&
           g_debug_asks == asks && !memcmp(g_published, full, sizeof full));
    poll_after(200);
    assert(g_publish_attempts == attempts + 1U);
    complete_publish(ERR_TIMEOUT);
    poll_after(99);
    assert(g_publish_attempts == attempts + 1U);
    poll_after(1);
    assert(g_publish_attempts == attempts + 2U && g_debug_asks == asks &&
           outbox_count() == 1U && !memcmp(g_published, full, sizeof full));

    /* lwIP clears the callback on disconnect. Retry the same capture after
     * reconnect, keep the waiting event, and keep one heartbeat timer chain. */
    g_fake_client.connected = 0;
    gp_deferred_done        = NULL;
    gp_deferred_arg         = NULL;
    gp_connected(&g_fake_client, NULL, 256);
    poll_after(100);
    assert(g_publish_attempts == attempts + 2U && 1U == g_heartbeat_pending);
    g_fake_client.connected = 1;
    g_defer_completion      = 0;
    gp_connected(&g_fake_client, NULL, MQTT_CONNECT_ACCEPTED);
    mqtt_utk_poll();
    assert(g_publish_attempts == attempts + 3U && g_debug_asks == asks &&
           1U == g_heartbeat_pending && outbox_count() == 1U &&
           !strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC) &&
           !memcmp(g_published, full, sizeof full));
    /* The routine sample staged before the capture still precedes the event. */
    poll_after(20);
    assert(!strcmp(g_published_topic, "car/telemetry/motion"));
    poll_after(20);
    assert(!strcmp(g_published_topic, "car/telemetry/heading"));

    /* A permanent capture error frees its slot; fresh telemetry progresses. */
    g_publish_result = ERR_ARG;
    poll_after(20);
    assert(2U == g_debug_head);
    assert(buddy1_report_motion(52, 54) == E_OK);
    g_publish_result = ERR_OK;
    poll_after(20);
    assert(!strcmp(g_published_topic, "car/telemetry/motion"));
    g_debug_head = g_debug_count = 0;
    puts("HOST PASS: mixed app/capture exact retries, 164 bytes, one "
         "in-flight, reconnect, one heartbeat chain, permanent-error recovery");
}

#if APP_IR_DEBUG
/** @brief Check IR preview length, priority, pacing and drop behavior when
 * the publisher is busy.
 */
static void test_ir_preview (void)
{
    unsigned count;
    empty_outbox();
    g_debug_head     = 0;
    g_debug_count    = 1;
    g_debug_queue[0] = g_debug_end;
    g_debug_size[0]  = sizeof g_debug_end;
    g_ir_frames      = 1;
    assert(buddy1_report_heading(40) == E_OK);
    count = g_publish_count;
    poll_after(20);
    assert(g_publish_count == count + 1U && 1U == g_ir_frames &&
           strcmp(g_published_topic, MQTT_UTK_IR_DEBUG_TOPIC) != 0);
    poll_after(20);
    assert(g_publish_count == count + 2U && 1U == g_ir_frames &&
           strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC) == 0);
    poll_after(20);
    assert(g_publish_count == count + 3U && 0U == g_ir_frames &&
           strcmp(g_published_topic, MQTT_UTK_IR_DEBUG_TOPIC) == 0);
    assert(16 == (int)g_published_len && 1 == (int)g_published[0] &&
           0 == (int)g_published[1] && 0 == (int)g_published_qos &&
           0 == (int)g_published_retain);
    g_ir_frames      = 1;
    g_publish_result = ERR_MEM;
    poll_after(20);
    assert(g_publish_count == count + 3U && 0U == g_ir_frames);
    g_publish_result = ERR_OK;
    poll_after(100);
    assert(g_publish_count == count + 3U); /* Drop a rejected preview. */
    g_ir_frames        = 1;
    g_defer_completion = 1;
    poll_after(20);
    assert(g_publish_count == count + 4U);
    assert(buddy1_report_heading(41) == E_OK);
    poll_after(20);
    assert(g_publish_count == count + 4U);
    g_defer_completion = 0;
    complete_publish(ERR_OK);
    poll_after(20);
    assert(g_publish_count == count + 5U &&
           strcmp(g_published_topic, "car/telemetry/heading") == 0);
    g_debug_head = g_debug_count = 0;
    puts("HOST PASS: IR preview below telemetry/capture priority, binary "
         "length, QoS0, pacing, drop-on-backpressure and one in-flight slot");
}

/** @brief Check the 70-byte control preview under event priority, staged
 * barcode data and publish rejection.
 */
static void test_control_preview (void)
{
    empty_outbox();
    g_debug_head     = 0;
    g_debug_count    = 1;
    g_debug_queue[0] = g_debug_end;
    g_debug_size[0]  = sizeof g_debug_end;
    g_control_frames = g_ir_frames = 1;
    assert(buddy1_report_heading(42) == E_OK);
    poll_after(20);
    assert(!strcmp(g_published_topic, "car/telemetry/heading") &&
           1U == g_control_frames);
    poll_after(200);
    assert(!strcmp(g_published_topic, MQTT_UTK_CONTROL_DEBUG_TOPIC) &&
           0U == g_control_frames && 1U == g_ir_frames);
    assert(70 == (int)g_published_len && 0 == (int)g_published_qos &&
           0 == (int)g_published_retain);
    poll_after(20);
    assert(!strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC) &&
           g_published_len == sizeof g_debug_end &&
           !memcmp(g_published, g_debug_end, sizeof g_debug_end));
    poll_after(20);
    assert(!strcmp(g_published_topic, MQTT_UTK_IR_DEBUG_TOPIC) &&
           0U == g_ir_frames);
    unsigned count   = g_publish_count;
    g_control_frames = 1;
    g_publish_result = ERR_MEM;
    poll_after(200);
    assert(0U == g_control_frames && g_publish_count == count);
    g_publish_result = ERR_OK;
    poll_after(100);
    assert(g_publish_count == count);
    g_control_frames   = 1;
    g_defer_completion = 1;
    poll_after(200);
    assert(g_publish_count == count + 1U && 70 == (int)g_published_len);
    g_ir_frames = 1;
    poll_after(20);
    assert(g_publish_count == count + 1U && 1U == g_ir_frames);
    g_defer_completion = 0;
    complete_publish(ERR_OK);
    poll_after(20);
    assert(0U == g_ir_frames && g_publish_count == count + 2U);
    g_debug_head = g_debug_count = 0;
    puts("HOST PASS: bounded control diagnostics after events, staged BCAP "
         "intact, exact 70 bytes, drop-on-reject and one in-flight buffer");
}

/** @brief Simulate publish completions at the supplied millisecond interval
 * and check preview pacing amid saturated data traffic.
 */
static void test_control_preview_fairness (unsigned completion_ms)
{
    unsigned                   apps             = 0;
    unsigned                   captures         = 0;
    unsigned                   previews         = 0;
    unsigned                   data_since       = 2;
    u32_t                      previous_preview = 0;
    telemetry_codec_snapshot_t decoded;
    empty_outbox();
    g_control_frames = g_ir_frames = 0;
    /* Establish a capture turn boundary and two completed data packets. */
    queue_debug(2, g_debug_summary, sizeof g_debug_summary);
    poll_after(200);
    poll_after(20);
    assert(!strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC));
    queue_debug(1, g_debug_summary, sizeof g_debug_summary);

    for (unsigned report_index = 1; report_index <= QUEUED_REPORTS;
         ++report_index)
    {
        assert(buddy1_report_motion((H)(2U * report_index), 0) == E_OK);
    }

    g_control_frames   = 100;
    g_defer_completion = completion_ms != 0U;

    for (unsigned poll_index = 0; poll_index < RECONNECT_DRAIN_POLLS;
         ++poll_index)
    {
        unsigned before = g_publish_count;
        poll_after(20);
        assert(g_publish_count == before + 1U);

        if (!strcmp(g_published_topic, MQTT_UTK_CONTROL_DEBUG_TOPIC))
        {
            assert(data_since >= 2U);

            if (previews)
            {
                assert((u32_t)(g_clock_ms - previous_preview) >= 200U);
            }

            previous_preview = g_clock_ms;
            ++previews;
            data_since = 0;
        }
        else if (!strcmp(g_published_topic, "car/telemetry/motion"))
        {
            ++apps;
            ++data_since;
            assert(buddy1_decode(BUDDY1_TOPIC_MOTION, g_published,
                                 g_published_len, &decoded));
            assert(decoded.left_mms == (H)(2U * apps));
            assert(buddy1_report_motion((H)(2U * (apps + 4U)), 0) == E_OK);
        }
        else
        {
            assert(!strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC));
            assert(
                g_published_len == sizeof g_debug_summary &&
                !memcmp(g_published, g_debug_summary, sizeof g_debug_summary));
            ++captures;
            ++data_since;
            queue_debug(1, g_debug_summary, sizeof g_debug_summary);
        }

        if (completion_ms)
        {
            unsigned attempts = g_publish_attempts;
            poll_after(completion_ms);
            assert(g_publish_attempts ==
                   attempts); /* Keep one packet in flight. */
            complete_publish(ERR_OK);
        }
    }

    assert(previews > 0U && apps > 0U && captures > 0U);
    assert(apps == captures || apps == captures + 1U);

    if (completion_ms >= 250U)
    {
        assert(20U == previews && 20U == apps && 20U == captures);
    }
    else
    {
        assert(previews <= 6U && apps >= 27U && captures >= 27U);
    }
    /* A fresh preview must not bypass a FIFO-head event, even when overdue. */
    /* clang-format off */
    g_control_frames = 0;
    g_debug_head     = g_debug_count = 0;
    /* clang-format on */
    empty_outbox();
    g_defer_completion = 0;
    poll_after(200);
    empty_outbox();
    assert(buddy1_report_heading(43) == E_OK);
    poll_after(20);
    assert(buddy1_report_heading(44) == E_OK);
    poll_after(20);
    g_control_frames = 1;
    assert(buddy1_report_heading(45) == E_OK);
    poll_after(200);
    assert(!strcmp(g_published_topic, "car/telemetry/heading") &&
           1U == g_control_frames);
    poll_after(20);
    assert(!strcmp(g_published_topic, MQTT_UTK_CONTROL_DEBUG_TOPIC));
    g_control_frames = 0;
    printf("HOST PASS: controller preview progresses with saturated "
           "routine/BCAP, >=2 data packets and <=5Hz; completion=%ums\n",
           completion_ms);
}

/** @brief Check that a preview waits for an outstanding barcode retry,
 * publish completion and queued event.
 */
static void test_preview_waits_for_retry (void)
{
    uint8_t  full[MQTT_UTK_DEBUG_PAYLOAD_MAX];
    unsigned attempts;
    memset(full, 0xa7, sizeof full);
    full[5] = 0;
    empty_outbox();
    g_control_frames = g_ir_frames = 0;
    queue_debug(1, full, sizeof full);
    g_publish_result = ERR_MEM;
    poll_after(200);
    assert(!strcmp(g_attempted.topic, MQTT_UTK_DEBUG_TOPIC));
    g_control_frames = 1;
    assert(buddy1_report_heading(46) == E_OK);
    attempts = g_publish_attempts;
    poll_after(99);
    assert(g_publish_attempts == attempts && 1U == g_control_frames);
    g_publish_result   = ERR_OK;
    g_defer_completion = 1;
    poll_after(1);
    assert(g_publish_attempts == attempts + 1U &&
           !strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC) &&
           g_published_len == sizeof full &&
           !memcmp(g_published, full, sizeof full) && 1U == g_control_frames);
    poll_after(250);
    assert(g_publish_attempts == attempts + 1U && 1U == g_control_frames);
    complete_publish(ERR_TIMEOUT);
    poll_after(100);
    assert(g_publish_attempts == attempts + 2U &&
           !strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC) &&
           g_published_len == sizeof full &&
           !memcmp(g_published, full, sizeof full) && 1U == g_control_frames);
    complete_publish(ERR_OK);
    g_defer_completion = 0;
    poll_after(20);
    assert(!strcmp(g_published_topic, "car/telemetry/heading") &&
           1U == g_control_frames);
    poll_after(20);
    assert(!strcmp(g_published_topic, MQTT_UTK_CONTROL_DEBUG_TOPIC) &&
           0U == g_control_frames);
    g_debug_head = g_debug_count = 0;
    puts("HOST PASS: overdue controller preview waits for exact BCAP retries, "
         "in-flight completion and queued event");
}

#endif
#if APP_IMU_DEBUG
/** @brief Check IMU batch size and traffic scheduling under load, including
 * the handling of a rejected diagnostic publish.
 */
static void test_imu_stream (void)
{
    unsigned apps       = 0;
    unsigned captures   = 0;
    unsigned batches    = 0;
    unsigned data_since = 0;
    empty_outbox();
    /* clang-format off */
    g_debug_head  = g_debug_count = 0;
    g_imu_batches = 1;
    /* clang-format on */
    assert(buddy1_report_heading(64) == E_OK);
    poll_after(200);
    assert(!strcmp(g_published_topic, "car/telemetry/heading") &&
           1U == g_imu_batches);
#if APP_IR_DEBUG
    g_control_frames = 1;
    poll_after(200);
    assert(!strcmp(g_published_topic, MQTT_UTK_CONTROL_DEBUG_TOPIC) &&
           1U == g_imu_batches);
#endif
    poll_after(20);
    assert(!strcmp(g_published_topic, MQTT_UTK_IMU_DEBUG_TOPIC) &&
           0U == g_imu_batches);
    assert(MQTT_UTK_IMU_DEBUG_MAX == g_published_len &&
           0 == (int)g_published_qos && 0 == (int)g_published_retain);
    assert(1U == g_published[0] && 8U == g_published[1] &&
           0U == g_published[2] && 0x5aU == g_published[g_published_len - 1U]);

    /* A busy link grants no more than one IMU slot per two data publishes,
     * preserving the app/capture alternation and progress for all sources. */
    g_imu_batches = 12;
    queue_debug(4, g_debug_summary, sizeof g_debug_summary);

    for (unsigned report_index = 0; report_index < QUEUED_REPORTS;
         ++report_index)
    {
        assert(buddy1_report_motion(40, 41) == E_OK);
    }

    unsigned before = g_publish_count;

    for (unsigned poll_index = 0; poll_index < FINAL_DRAIN_POLLS; ++poll_index)
    {
        poll_after(20);
        assert(g_publish_count == before + poll_index + 1U);

        if (!strcmp(g_published_topic, MQTT_UTK_IMU_DEBUG_TOPIC))
        {
            assert(data_since >= 2U);
            ++batches;
            data_since = 0;
        }
        else if (!strcmp(g_published_topic, MQTT_UTK_DEBUG_TOPIC))
        {
            ++captures;
            ++data_since;
        }
        else
        {
            assert(!strcmp(g_published_topic, "car/telemetry/motion"));
            assert(buddy1_report_motion(40, 41) == E_OK);
            ++apps;
            ++data_since;
        }
    }

    assert(4U == apps && 4U == captures && 4U == batches);

    /* Drain any already staged routine packet then demonstrate that a
     * rejected IMU packet never delays a later FIFO-head event with a retry. */
    g_imu_batches = 0;
    empty_outbox();
    poll_after(20);
    g_imu_batches    = 1;
    g_publish_result = ERR_MEM;
    poll_after(20);
    assert(!strcmp(g_attempted.topic, MQTT_UTK_IMU_DEBUG_TOPIC) &&
           0U == g_imu_batches);
    g_publish_result = ERR_OK;
    assert(buddy1_report_heading(65) == E_OK);
    poll_after(20);
    assert(!strcmp(g_published_topic, "car/telemetry/heading"));
    g_debug_head = g_debug_count = 0;
    puts("HOST PASS: bounded raw IMU batches, legacy traffic priority and "
         "fairness, no diagnostic retry");
}

#endif
/** @brief Inject deferred completions, queue pressure, reconnects and clock
 * rollover. Verify retry pacing and permanent error recovery.
 * @details A transient publish rejection retains the payload and QoS.
 * Successful submission occupies an in-flight slot until completion.
 * Disconnect clears the fake lwIP callback while the transport retains the
 * pending packet. Confirm that invalid arguments release the slot and
 * that a retry deadline survives the millisecond timer wrapping.
 */
static void test_publish_backpressure (void)
{
    buddy1_command_msg_t       command = {.kind    = BUDDY1_COMMAND_SNAPSHOT,
                                          .payload = {BUDDY1_WIRE_VERSION}};
    telemetry_codec_snapshot_t decoded;
    UB                         expected[BUDDY1_PAYLOAD_MAX];
    unsigned                   count;
    unsigned                   attempts;
    unsigned                   logs;
    unsigned                   index;

    /* ERR_MEM keeps the exact QoS 1 reply, rather than popping and losing it.
     * Retry rate/logging are bounded and the following FIFO item stays queued.
     */
    empty_outbox();
    assert(buddy1_snapshot_packet(expected));
    assert(buddy1_command_process(&command) == E_OK);
    assert(buddy1_report_heading(47) == E_OK);
    g_publish_result = ERR_MEM;
    count            = g_publish_count;
    logs             = g_delayed_logs;
    poll_after(20);
    attempts = g_publish_attempts;
    assert(outbox_count() == 1U && g_publish_count == count &&
           g_delayed_logs == logs + 1U);
    assert(strcmp(g_attempted.topic, "car/reply/snapshot") == 0 &&
           1 == (int)g_attempted.qos && 0 == (int)g_attempted.retain &&
           g_attempted.payload_len == sizeof(expected) &&
           memcmp(g_attempted.payload, expected, sizeof(expected)) == 0);

    for (index = 0; index < NO_CLOCK_RETRY_POLLS; ++index)
    {
        mqtt_utk_poll();
    }

    poll_after(99);
    assert(g_publish_attempts == attempts);
    poll_after(1);
    assert(g_publish_attempts == attempts + 1U && g_delayed_logs == logs + 1U);
    g_publish_result = ERR_OK;
    poll_after(100);
    assert(g_publish_count == count + 1U && outbox_count() == 1U &&
           1 == (int)g_published_qos && 0 == (int)g_published_retain &&
           memcmp(g_published, expected, sizeof(expected)) == 0);
    mqtt_utk_poll();
    poll_after(19);
    assert(g_publish_count == count + 1U);
    poll_after(1);
    assert(g_publish_count == count + 2U && outbox_count() == 0U &&
           0 == (int)g_published_qos &&
           buddy1_decode(BUDDY1_TOPIC_HEADING, g_published, g_published_len,
                         &decoded) &&
           47 == decoded.heading_deg);

    /* A rejected packet survives a disconnected period and is first on
     * reconnect. */
    assert(buddy1_report_motion(156, -74) == E_OK);
    assert(buddy1_report_heading(88) == E_OK);
    g_publish_result = ERR_CONN;
    poll_after(20);
    attempts                = g_publish_attempts;
    g_fake_client.connected = 0;

    for (index = 0; index < DISCONNECTED_POLL_COUNT; ++index)
    {
        poll_after(100);
    }

    assert(g_publish_attempts == attempts && outbox_count() == 1U);
    g_publish_result        = ERR_OK;
    g_fake_client.connected = 1;
    gp_connected(&g_fake_client, NULL, MQTT_CONNECT_ACCEPTED);
    mqtt_utk_poll();
    assert(buddy1_decode(BUDDY1_TOPIC_MOTION, g_published, g_published_len,
                         &decoded) &&
           156 == decoded.left_mms && decoded.right_mms == -74 &&
           outbox_count() == 1U);
    poll_after(20);
    assert(outbox_count() == 0U);

    /* Accepted != completed. Only one app request consumes a lwIP slot until
     * completion, even after thousands of network polls. A callback timeout
     * retains this packet and retries before the next FIFO item. */
    assert(buddy1_report_motion(56, 58) == E_OK);
    assert(buddy1_report_heading(89) == E_OK);
    g_defer_completion = 1;
    poll_after(20);
    attempts = g_publish_attempts;

    for (index = 0; index < FAILED_LINK_RETRY_POLLS; ++index)
    {
        poll_after(20);
    }

    assert(g_publish_attempts == attempts && outbox_count() == 1U);
    complete_publish(ERR_TIMEOUT);
    poll_after(99);
    assert(g_publish_attempts == attempts);
    poll_after(1);
    assert(g_publish_attempts == attempts + 1U && outbox_count() == 1U &&
           buddy1_decode(BUDDY1_TOPIC_MOTION, g_published, g_published_len,
                         &decoded) &&
           56 == decoded.left_mms && 58 == decoded.right_mms);
    complete_publish(ERR_OK);
    g_defer_completion = 0;
    poll_after(20);
    assert(outbox_count() == 0U &&
           buddy1_decode(BUDDY1_TOPIC_HEADING, g_published, g_published_len,
                         &decoded) &&
           89 == decoded.heading_deg);

    /* Real lwIP drops in-flight callback registrations on disconnect. Do not
     * get stuck waiting forever or lose that packet. Duplicate delivery after
     * an ambiguous disconnect is possible and intentionally not "exactly once".
     */
    assert(buddy1_report_motion(60, -62) == E_OK);
    assert(buddy1_report_heading(90) == E_OK);
    g_defer_completion = 1;
    poll_after(20);
    attempts                = g_publish_attempts;
    g_fake_client.connected = 0;
    gp_deferred_done        = NULL;
    gp_deferred_arg         = NULL;
    gp_connected(&g_fake_client, NULL, 256);
    poll_after(100);
    assert(g_publish_attempts == attempts && outbox_count() == 1U);
    g_fake_client.connected = 1;
    g_defer_completion      = 0;
    gp_connected(&g_fake_client, NULL, MQTT_CONNECT_ACCEPTED);
    mqtt_utk_poll();
    assert(g_publish_attempts == attempts + 1U && outbox_count() == 1U &&
           buddy1_decode(BUDDY1_TOPIC_MOTION, g_published, g_published_len,
                         &decoded) &&
           60 == decoded.left_mms && decoded.right_mms == -62);
    poll_after(20);
    assert(outbox_count() == 0U);

    /* A permanent bad argument is explicitly discarded once, not retried in
     * a tight loop; the next packet can progress on the next paced poll. */
    assert(buddy1_report_motion(12, 14) == E_OK);
    assert(buddy1_report_heading(91) == E_OK);
    g_publish_result = ERR_ARG;
    logs             = g_discarded_logs;
    poll_after(20);
    attempts = g_publish_attempts;

    for (index = 0; index < REJECTED_UPLOAD_POLLS; ++index)
    {
        mqtt_utk_poll();
    }

    assert(g_publish_attempts == attempts && g_discarded_logs == logs + 1U &&
           outbox_count() == 1U);
    g_publish_result = ERR_OK;
    poll_after(20);
    assert(outbox_count() == 0U &&
           buddy1_decode(BUDDY1_TOPIC_HEADING, g_published, g_published_len,
                         &decoded) &&
           91 == decoded.heading_deg);

    /* Millisecond-clock rollover does not cancel the retry delay. */
    assert(buddy1_report_motion(16, 18) == E_OK);
    g_clock_ms       = UINT32_MAX - 5U;
    g_publish_result = ERR_MEM;
    mqtt_utk_poll();
    attempts = g_publish_attempts;
    poll_after(99);
    assert(g_publish_attempts == attempts);
    g_publish_result = ERR_OK;
    poll_after(1);
    assert(g_publish_attempts == attempts + 1U &&
           buddy1_decode(BUDDY1_TOPIC_MOTION, g_published, g_published_len,
                         &decoded) &&
           16 == decoded.left_mms && 18 == decoded.right_mms);
    puts("HOST PASS: MQTT retained FIFO retry, exact binary/QoS, bounded "
         "pacing/logging, deferred completion, disconnect recovery, permanent "
         "error, clock rollover");
}

/** @brief Check the four-byte route pose, field limits, separate QoS 0
 * publication and snapshot bits 102 through 123.
 */
static void test_pose (void)
{
    telemetry_codec_snapshot_t snapshot = {0};
    telemetry_codec_snapshot_t decoded;
    telemetry_codec_snapshot_t before;
    telemetry_codec_snapshot_t after;
    UB                         payload[32];
    snapshot.pose_distance_cm = 4095;
    snapshot.pose_heading_deg = 359;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &snapshot, payload,
                         sizeof payload) == 4U);
    assert(BUDDY1_WIRE_VERSION == payload[0] && 0xffU == payload[1] &&
           0x7fU == payload[2] && 0x16U == payload[3]);
    assert(buddy1_decode(BUDDY1_TOPIC_POSE, payload, 4U, &decoded) &&
           decoded.pose_valid && 4095U == decoded.pose_distance_cm &&
           359 == decoded.pose_heading_deg);
    snapshot.pose_distance_cm = 0;
    snapshot.pose_heading_deg = 0;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &snapshot, payload,
                         sizeof payload) == 4U &&
           0U == payload[1] && 0U == payload[2] && 0U == payload[3]);
    snapshot.pose_distance_cm = 4096;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.pose_distance_cm = 1;
    snapshot.pose_heading_deg = 360;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.pose_heading_deg = -1;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &snapshot, payload,
                         sizeof payload) == 0U);
    payload[0] = BUDDY1_WIRE_VERSION;
    payload[1] = 0;
    payload[2] = 0x80;
    payload[3] = 0x16; /* heading 360 */
    assert(!buddy1_decode(BUDDY1_TOPIC_POSE, payload, 4U, &decoded));
    payload[2] = 0;
    payload[3] = 0x20; /* Set a reserved padding bit. */
    assert(!buddy1_decode(BUDDY1_TOPIC_POSE, payload, 4U, &decoded));
    payload[3] = 0;
    assert(
        !buddy1_decode(BUDDY1_TOPIC_POSE, payload, 3U, &decoded)); /* length */
    assert(telemetry_codec_payload_size(BUDDY1_TOPIC_POSE) == 4U &&
           telemetry_codec_payload_bits(BUDDY1_TOPIC_POSE) == 21U);
    assert(buddy1_invalidate(BUDDY1_TOPIC_POSE) ==
           E_PAR); /* Use an unsupported topic bit. */
    empty_outbox();
    assert(buddy1_snapshot(&before));
    assert(buddy1_report_pose(4096, 0) == E_PAR &&
           buddy1_report_pose(0, 360) == E_PAR);
    assert(buddy1_snapshot(&after) &&
           memcmp(&before, &after, sizeof before) ==
               0); /* Leave the output intact on rejection. */
    assert(buddy1_report_pose(1234, 270) == E_OK);
    assert(buddy1_snapshot(&after) && after.pose_valid &&
           1234U == after.pose_distance_cm && 270 == after.pose_heading_deg &&
           after.motion_valid == before.motion_valid &&
           after.faults == before.faults &&
           after.hump_count == before.hump_count);
    /* Snapshot layout: the original 102 bits unchanged, pose valid at bit 102,
     * distance bits 103-114, heading bits 115-123, version + 16 body bytes. */
    {
        UB                         snap[BUDDY1_PAYLOAD_MAX];
        telemetry_codec_snapshot_t zero_pose = {0};
        assert(telemetry_codec_payload_bits(BUDDY1_TOPIC_SNAPSHOT) == 124U &&
               telemetry_codec_payload_size(BUDDY1_TOPIC_SNAPSHOT) == 17U);
        zero_pose.pose_valid       = 1;
        zero_pose.pose_distance_cm = 4095;
        zero_pose.pose_heading_deg = 359;
        assert(buddy1_encode(BUDDY1_TOPIC_SNAPSHOT, &zero_pose, snap,
                             sizeof snap) == 17U);
        assert(0U == snap[1] && 0xC0U == snap[13] && 0xFFU == snap[14] &&
               0x3FU == snap[15] && 0x0BU == snap[16]);
        assert(buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, snap, 17U, &decoded) &&
               decoded.pose_valid && 4095U == decoded.pose_distance_cm &&
               359 == decoded.pose_heading_deg && !decoded.motion_valid);
        snap[16] = 0x1BU;
        assert(!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, snap, 17U,
                              &decoded)); /* padding */
        snap[16]  = 0x0FU;
        snap[15] |= 0xF8U;
        assert(!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, snap, 17U,
                              &decoded)); /* heading 511 */
        zero_pose.pose_valid = 0;
        zero_pose.pose_distance_cm =
            77; /* Zero an invalid field with stale data. */
        assert(buddy1_encode(BUDDY1_TOPIC_SNAPSHOT, &zero_pose, snap,
                             sizeof snap) == 17U &&
               0U == snap[13] && 0U == snap[14] && 0U == snap[15] &&
               0U == snap[16]);
        snap[14] = 1U;
        assert(!buddy1_decode(
            BUDDY1_TOPIC_SNAPSHOT, snap, 17U,
            &decoded)); /* Supply data without its validity bit. */
        assert(!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, snap, 14U,
                              &decoded)); /* Use the old packet length. */
    }
    poll_after(20);
    assert(strcmp(g_published_topic, "car/telemetry/pose") == 0 &&
           4U == g_published_len &&
           buddy1_decode(BUDDY1_TOPIC_POSE, g_published, g_published_len,
                         &decoded) &&
           1234U == decoded.pose_distance_cm &&
           270 == decoded.pose_heading_deg);
    puts("HOST PASS: route pose 21 bits/4 bytes, 4095/359 limits, "
         "padding/length rejection, own topic; snapshot pose section (bits "
         "102-123), rejected values never cached");
}

/* Buddy 4 terrain: live-only, 59 bits/9 bytes (19 Nov 2026: + compass turn
 * rate and lifted/overturned), every pitch/roll, height and turn code
 * round-trips, unavailable means canonical zero, never cached. */
/* Buddy 5: live avoidance (38 bits/6 bytes) and FR6 scan profile (95 bits/
 * 13 bytes); every field boundary round-trips, unusable values are zero,
 * out-of-range and noncanonical packets are rejected, never cached. */
/** @brief Compare the named avoidance fields. Native padding can differ in
 * otherwise equal stack copies, as observed with arm64 GCC on 14
 * November 2026.
 */
static int same_avoidance (const telemetry_codec_avoidance_t * p_first,
                           const telemetry_codec_avoidance_t * p_second)
{
    return (((((((((p_first->range_valid == p_second->range_valid) &&
                   (p_first->no_echo == p_second->no_echo)) &&
                  (p_first->range_mm == p_second->range_mm)) &&
                 (p_first->servo_deg == p_second->servo_deg)) &&
                (p_first->state == p_second->state)) &&
               (p_first->fault == p_second->fault)) &&
              (p_first->route == p_second->route)) &&
             (p_first->leg == p_second->leg)) &&
            (p_first->pass == p_second->pass)) &&
           (p_first->swapped == p_second->swapped);
}

/** @brief Compare the validity, identity and measured geometry fields of two
 * obstacle scan reports.
 */
static int same_scan (const telemetry_codec_scan_t * p_first,
                      const telemetry_codec_scan_t * p_second)
{
    return ((((((((p_first->valid == p_second->valid) &&
                  (p_first->scan_id == p_second->scan_id)) &&
                 (p_first->closest_mm == p_second->closest_mm)) &&
                (p_first->width_mm == p_second->width_mm)) &&
               (p_first->left_clear_mm == p_second->left_clear_mm)) &&
              (p_first->right_clear_mm == p_second->right_clear_mm)) &&
             (p_first->forward_mm == p_second->forward_mm)) &&
            (p_first->left_edge_mm == p_second->left_edge_mm)) &&
           (p_first->right_edge_mm == p_second->right_edge_mm);
}

/** @brief Check avoidance and scan wire boundaries, rejection of malformed
 * packets and publication on the expected topics.
 */
static void test_buddy5 (void)
{
    telemetry_codec_snapshot_t  snapshot;
    telemetry_codec_snapshot_t  decoded;
    telemetry_codec_snapshot_t  before;
    telemetry_codec_snapshot_t  after;
    telemetry_codec_avoidance_t avoidance_case;
    telemetry_codec_scan_t      scan_case;
    UB                          payload[32];
    unsigned                    millimetres;
    unsigned                    deg;
    int                         edge_mm;
    assert(telemetry_codec_payload_size(BUDDY1_TOPIC_AVOIDANCE) == 6U &&
           telemetry_codec_payload_bits(BUDDY1_TOPIC_AVOIDANCE) == 38U);
    assert(telemetry_codec_payload_size(BUDDY1_TOPIC_SCAN) == 13U &&
           telemetry_codec_payload_bits(BUDDY1_TOPIC_SCAN) == 95U);
    assert(telemetry_codec_payload_size(BUDDY1_TOPIC_SCAN) <=
           BUDDY1_PAYLOAD_MAX);

    for (millimetres  = MIN_ECHO_MM; millimetres <= MAX_ECHO_MM;
         millimetres += 1U)
    {
        /* clang-format off */
        deg                = millimetres % 181U;
        snapshot           = (telemetry_codec_snapshot_t){0};
        snapshot.avoidance =
            (telemetry_codec_avoidance_t){1,
                                          0,
                                          (uint16_t)millimetres,
                                          (uint8_t)deg,
                                          (uint8_t)(millimetres % 7U),
                                          (uint8_t)(millimetres % 8U),
                                          (uint8_t)(millimetres % 3U),
                                          (uint8_t)(millimetres % 16U),
                                          (uint8_t)(millimetres % 5U),
                                          (uint8_t)(millimetres % 2U)};
        /* clang-format on */
        assert(buddy1_encode(BUDDY1_TOPIC_AVOIDANCE, &snapshot, payload,
                             sizeof payload) == 6U &&
               ((unsigned)payload[5] >> 6U) == 0U);
        assert(buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, payload, 6U, &decoded) &&
               decoded.avoidance_valid &&
               same_avoidance(&decoded.avoidance, &snapshot.avoidance));
    }

    snapshot           = (telemetry_codec_snapshot_t){0};
    snapshot.avoidance = (telemetry_codec_avoidance_t){
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0}; /* Report that the sensor got no echo. */
    assert(buddy1_encode(BUDDY1_TOPIC_AVOIDANCE, &snapshot, payload,
                         sizeof payload) == 6U &&
           buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, payload, 6U, &decoded) &&
           decoded.avoidance.no_echo && !decoded.avoidance.range_valid);
    {
        const telemetry_codec_avoidance_t bad[] = {
            {1, 0, 29, 90, 0, 0, 0, 0, 0, 0},
            {1, 0, 4001, 90, 0, 0, 0, 0, 0, 0},
            {0, 0, 500, 90, 0, 0, 0, 0, 0, 0},
            {1, 1, 500, 90, 0, 0, 0, 0, 0, 0},
            {1, 0, 500, 181, 0, 0, 0, 0, 0, 0},
            {1, 0, 500, 90, 7, 0, 0, 0, 0, 0},
            {1, 0, 500, 90, 0, 0, 3, 0, 0, 0},
            {1, 0, 500, 90, 0, 0, 0, 0, 5, 0},
            {1, 0, 500, 90, 0, 0, 0, 16, 0, 0},
            {1, 0, 500, 90, 0, 0, 0, 0, 0, 2}};

        for (unsigned case_index = 0; case_index < (sizeof bad / sizeof bad[0]);
             ++case_index)
        {
            snapshot.avoidance = bad[case_index];
            assert(buddy1_encode(BUDDY1_TOPIC_AVOIDANCE, &snapshot, payload,
                                 sizeof payload) == 0U);
        }
    }
    payload[0] = 3;
    payload[1] = 0x04;
    payload[2] = 0;
    payload[3] = 0;
    payload[4] = 0;
    payload[5] = 0; /* Set a distance on an invalid range. */
    assert(!buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, payload, 6U, &decoded));
    payload[1] = 0;
    payload[2] = 0;
    payload[5] = 0x40;
    assert(!buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, payload, 6U,
                          &decoded)); /* padding */
    payload[5] = 0;
    assert(!buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, payload, 5U,
                          &decoded)); /* length */
    for (edge_mm  = -MAX_EDGE_OFFSET_MM; edge_mm <= MAX_EDGE_OFFSET_MM;
         edge_mm += 1)
    {
        snapshot      = (telemetry_codec_snapshot_t){0};
        snapshot.scan = (telemetry_codec_scan_t){
            1,
            (uint8_t)((unsigned)edge_mm & 0xFFU),
            (uint16_t)(30U + ((unsigned)(edge_mm + 4000) % 3971U)),
            (uint16_t)((unsigned)(edge_mm + 4000) % 4096U),
            (uint16_t)((unsigned)(edge_mm + 4000) % 4001U),
            (uint16_t)((unsigned)(4000 - edge_mm) % 4001U),
            (uint16_t)((unsigned)(edge_mm + 4000) / 2U),
            4000,
            (int16_t)edge_mm};
        assert(buddy1_encode(BUDDY1_TOPIC_SCAN, &snapshot, payload,
                             sizeof payload) == 13U &&
               ((unsigned)payload[12] >> 7U) == 0U);
        assert(buddy1_decode(BUDDY1_TOPIC_SCAN, payload, 13U, &decoded) &&
               decoded.scan_valid && same_scan(&decoded.scan, &snapshot.scan));
    }

    snapshot              = (telemetry_codec_snapshot_t){0};
    snapshot.scan.scan_id = 9; /* Mark the scan as failed. */
    assert(buddy1_encode(BUDDY1_TOPIC_SCAN, &snapshot, payload,
                         sizeof payload) == 13U &&
           buddy1_decode(BUDDY1_TOPIC_SCAN, payload, 13U, &decoded) &&
           !decoded.scan.valid && 9U == decoded.scan.scan_id);
    {
        const telemetry_codec_scan_t bad[] = {
            {1, 0, 29, 0, 0, 0, 0, 0, 0},     {1, 0, 4001, 0, 0, 0, 0, 0, 0},
            {1, 0, 100, 4096, 0, 0, 0, 0, 0}, {1, 0, 100, 0, 4001, 0, 0, 0, 0},
            {1, 0, 100, 0, 0, 4001, 0, 0, 0}, {1, 0, 100, 0, 0, 0, 4001, 0, 0},
            {1, 0, 100, 0, 0, 0, 0, 4001, 0}, {1, 0, 100, 0, 0, 0, 0, 0, -4001},
            {1, 0, 100, 0, 0, 0, 0, -10, 10}, {0, 0, 100, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 5, 0},      {2, 0, 100, 0, 0, 0, 0, 0, 0}};

        for (unsigned case_index = 0; case_index < (sizeof bad / sizeof bad[0]);
             ++case_index)
        {
            snapshot.scan = bad[case_index];
            assert(buddy1_encode(BUDDY1_TOPIC_SCAN, &snapshot, payload,
                                 sizeof payload) == 0U);
        }
    }
    snapshot.scan = (telemetry_codec_scan_t){0};
    assert(buddy1_encode(BUDDY1_TOPIC_SCAN, &snapshot, payload,
                         sizeof payload) == 13U);
    payload[12] = 0x80U;
    assert(!buddy1_decode(BUDDY1_TOPIC_SCAN, payload, 13U,
                          &decoded)); /* padding */
    payload[12] = 0;
    payload[2]  = 0x02U;
    assert(
        !buddy1_decode(BUDDY1_TOPIC_SCAN, payload, 13U,
                       &decoded)); /* Set a distance on an invalid reading. */
    assert(!buddy1_decode(BUDDY1_TOPIC_SCAN, payload, 12U, &decoded));
    assert(buddy1_invalidate(BUDDY1_TOPIC_AVOIDANCE) == E_PAR &&
           buddy1_invalidate(BUDDY1_TOPIC_SCAN) == E_PAR);
    empty_outbox();
    assert(buddy1_snapshot(&before));
    assert(buddy1_report_avoidance(0) == E_PAR &&
           buddy1_report_scan(0) == E_PAR);
    avoidance_case =
        (telemetry_codec_avoidance_t){1, 0, 20, 90, 0, 0, 0, 0, 0, 0};
    assert(buddy1_report_avoidance(&avoidance_case) == E_PAR);
    avoidance_case =
        (telemetry_codec_avoidance_t){1, 0, 412, 150, 3, 0, 2, 3, 3, 1};
    assert(buddy1_report_avoidance(&avoidance_case) == E_OK);
    poll_after(20);
    assert(strcmp(g_published_topic, "car/telemetry/avoidance") == 0 &&
           6U == g_published_len &&
           buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, g_published, g_published_len,
                         &decoded) &&
           same_avoidance(&decoded.avoidance, &avoidance_case));
    scan_case =
        (telemetry_codec_scan_t){1, 7, 250, 400, 180, 320, 380, 150, -250};
    assert(buddy1_report_scan(&scan_case) == E_OK);
    poll_after(20);
    assert(strcmp(g_published_topic, "car/telemetry/scan") == 0 &&
           13U == g_published_len &&
           buddy1_decode(BUDDY1_TOPIC_SCAN, g_published, g_published_len,
                         &decoded) &&
           same_scan(&decoded.scan, &scan_case));
    assert(buddy1_snapshot(&after) &&
           memcmp(&before, &after, sizeof before) == 0);
    puts("HOST PASS: Buddy 5 avoidance 38 bits/6 bytes and FR6 scan 95 bits/13 "
         "bytes: all range/edge codes, servo/state/route/leg/pass limits, "
         "unavailable=zero, padding/length rejection, live-only, own topics");
}

/** @brief Compare tilt, terrain state, hump, turn and safety values in two
 * decoded reports.
 */
static int same_terrain (const telemetry_codec_terrain_t * p_first,
                         const telemetry_codec_terrain_t * p_second)
{
    return ((((((((((p_first->tilt_valid == p_second->tilt_valid) &&
                    (p_first->pitch_tenths_deg ==
                     p_second->pitch_tenths_deg)) &&
                   (p_first->roll_tenths_deg == p_second->roll_tenths_deg)) &&
                  (p_first->state == p_second->state)) &&
                 (p_first->on_hump == p_second->on_hump)) &&
                (p_first->climb_mm == p_second->climb_mm)) &&
               (p_first->peak_valid == p_second->peak_valid)) &&
              (p_first->peak_mm == p_second->peak_mm)) &&
             (p_first->turn_valid == p_second->turn_valid)) &&
            (p_first->turn_dps == p_second->turn_dps)) &&
           (p_first->safety == p_second->safety);
}

/** @brief Check packing and validation of tilt, hump and safety reports and
 * their publication as live terrain data.
 */
static void test_terrain (void)
{
    telemetry_codec_snapshot_t snapshot = {0};
    telemetry_codec_snapshot_t decoded;
    telemetry_codec_snapshot_t before;
    telemetry_codec_snapshot_t after;
    telemetry_codec_terrain_t  terrain_case;
    UB                         payload[32];
    int                        angle_tenths_deg;
    assert(telemetry_codec_payload_size(BUDDY1_TOPIC_TERRAIN) == 9U &&
           telemetry_codec_payload_bits(BUDDY1_TOPIC_TERRAIN) == 59U);

    for (angle_tenths_deg = -MAX_TILT_TENTHS_DEG;
         angle_tenths_deg <= MAX_TILT_TENTHS_DEG; ++angle_tenths_deg)
    {
        snapshot         = (telemetry_codec_snapshot_t){0};
        snapshot.terrain = (telemetry_codec_terrain_t){
            1,
            (int16_t)angle_tenths_deg,
            (int16_t)(-angle_tenths_deg),
            (uint8_t)((unsigned)(angle_tenths_deg + 900) % 8U),
            1,
            (uint16_t)((unsigned)(angle_tenths_deg + 900) % 501U),
            1,
            (uint16_t)((unsigned)(900 - angle_tenths_deg) % 501U),
            1,
            (int16_t)(angle_tenths_deg * 2 / 5),
            (uint8_t)((unsigned)(angle_tenths_deg + 900) % 3U)};
        assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                             sizeof payload) == 9U &&
               ((unsigned)payload[8] >> 3U) == 0U);
        assert(buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 9U, &decoded) &&
               decoded.terrain_valid &&
               same_terrain(&decoded.terrain, &snapshot.terrain));
    }

    for (angle_tenths_deg = -MAX_TEST_RATE_TENTHS;
         angle_tenths_deg <= MAX_TEST_RATE_TENTHS; angle_tenths_deg += 720)
    {
        /* Test both turn-rate endpoints. */
        snapshot.terrain = (telemetry_codec_terrain_t){
            1, 0, 0, 4, 0, 0, 0, 0, 1, (int16_t)angle_tenths_deg, 0};
        assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                             sizeof payload) == 9U &&
               buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 9U, &decoded) &&
               same_terrain(&decoded.terrain, &snapshot.terrain));
    }

    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 0, 0, 0, 0, 0, 0, 0, 1, 361, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 0, 0, 0, 0, 0, 0, 0, 1, -361, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 901, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 0, -901, 0, 0, 0, 0, 0, 0, 0, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 0, 0, 0, 1, 501, 0, 0, 0, 0, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 0, 0, 0, 0, 0, 1, 501, 0, 0, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    snapshot.terrain =
        (telemetry_codec_terrain_t){1, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 0U);
    /* Everything unavailable: terrain state still sent, all values zero. */
    snapshot.terrain =
        (telemetry_codec_terrain_t){0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &snapshot, payload,
                         sizeof payload) == 9U &&
           0U == payload[1] && 0U == payload[2] && 0U == payload[3] &&
           0x01U == payload[4] && 0U == payload[5] && 0U == payload[6] &&
           0U == payload[7] && 0U == payload[8]);
    assert(buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 9U, &decoded) &&
           !decoded.terrain.tilt_valid && 2U == decoded.terrain.state &&
           !decoded.terrain.turn_valid &&
           BUDDY1_SAFETY_NONE == decoded.terrain.safety);
    payload[1] = 0x02U;
    assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 9U,
                          &decoded)); /* Set an angle with unavailable tilt. */
    payload[1] = 0xF1U;
    payload[2] = 0x07U;
    payload[4] = 0U; /* pitch +1016 */
    assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 9U, &decoded));
    payload[1] = 0;
    payload[2] = 0;
    payload[4] = 0x08U;
    assert(!buddy1_decode(
        BUDDY1_TOPIC_TERRAIN, payload, 9U,
        &decoded)); /* Set climb height without an active hump. */
    payload[4] = 0;
    payload[7] = 0x01U;
    assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 9U,
                          &decoded)); /* Set a rate without its validity bit. */
    payload[6] = 0xC0U;
    payload[7] = 0xB4U;
    assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 9U,
                          &decoded)); /* Set a valid rate beyond 360 deg/s. */
    payload[6] = 0x40U;
    payload[7] = 0xB4U;
    assert(buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 9U, &decoded) &&
           360 == decoded.terrain.turn_dps); /* +360 */
    payload[6] = 0;
    payload[7] = 0;
    payload[8] = 0x06U;
    assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 9U,
                          &decoded)); /* safety 3 */
    payload[8] = 0x08U;
    assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 9U,
                          &decoded)); /* padding */
    payload[8] = 0;
    assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, payload, 8U,
                          &decoded)); /* length */
    assert(buddy1_invalidate(BUDDY1_TOPIC_TERRAIN) ==
           E_PAR); /* Omit the required snapshot section. */
    empty_outbox();
    assert(buddy1_snapshot(&before));
    assert(buddy1_report_terrain(0) == E_PAR);
    terrain_case =
        (telemetry_codec_terrain_t){1, 901, 0, 1, 0, 0, 0, 0, 0, 0, 0};
    assert(buddy1_report_terrain(&terrain_case) == E_PAR);
    terrain_case = (telemetry_codec_terrain_t){
        1, -305, 42, 5, 1, 17, 1, 33, 1, -123, BUDDY1_SAFETY_LIFTED};
    assert(buddy1_report_terrain(&terrain_case) == E_OK);
    assert(buddy1_snapshot(&after) &&
           memcmp(&before, &after, sizeof before) == 0);
    poll_after(20);
    assert(strcmp(g_published_topic, "car/telemetry/terrain") == 0 &&
           9U == g_published_len &&
           buddy1_decode(BUDDY1_TOPIC_TERRAIN, g_published, g_published_len,
                         &decoded) &&
           same_terrain(&decoded.terrain, &terrain_case));
    puts("HOST PASS: Buddy 4 terrain 59 bits/9 bytes, all pitch/roll codes, "
         "+/-90.0, 0..500 mm and +/-360 deg/s limits, lifted/overturned, "
         "unavailable=zero, padding/length rejection, live-only, own topic");
}

/** @brief Compare the scan identity, position, angle, validity and distance
 * of two scan samples.
 */
static int same_sample (const telemetry_codec_scan_sample_t * p_first,
                        const telemetry_codec_scan_sample_t * p_second)
{
    return ((((((p_first->scan_id == p_second->scan_id) &&
                (p_first->peek == p_second->peek)) &&
               (p_first->index == p_second->index)) &&
              (p_first->count == p_second->count)) &&
             (p_first->angle_deg == p_second->angle_deg)) &&
            (p_first->valid == p_second->valid)) &&
           (p_first->distance_mm == p_second->distance_mm);
}

/** @brief Check the six-byte wire format and its 35 used bits, including
 * angle, index, count, range boundaries and the canonical zero for a
 * missing reading.
 */
static void test_scan_sample (void)
{
    telemetry_codec_snapshot_t    input = {0};
    telemetry_codec_snapshot_t    decoded;
    telemetry_codec_snapshot_t    before;
    telemetry_codec_snapshot_t    after;
    telemetry_codec_scan_sample_t valid_sample;
    UB                            packet[32];
    unsigned                      angle;
    unsigned                      count;
    unsigned                      index;
    unsigned                      range_index;
    static const unsigned         ranges_mm[] = {30, 31, 999, 3999, 4000};
    /* Field order is scan ID, peek, index, count, angle in degrees,
     * echo validity and range in millimetres. */
    static const telemetry_codec_scan_sample_t invalid_samples[] = {
        {1, 0, 0, 0, 90, 1, 200},  /* Supply a zero count. */
        {1, 0, 0, 17, 90, 1, 200}, /* Set count above the maximum. */
        {1, 0, 1, 1, 90, 1, 200},  /* Set the index to the count. */
        {1, 0, 0, 1, 25, 1, 200},  /* Set an angle below the minimum. */
        {1, 0, 0, 1, 155, 1, 200}, /* Set an angle above the maximum. */
        {1, 0, 0, 1, 92, 1, 200},  /* Set a non-grid angle. */
        {1, 0, 0, 1, 90, 1, 29},   /* Set a distance below the minimum. */
        {1, 0, 0, 1, 90, 1, 4001}, /* Set a distance above the maximum. */
        {1, 2, 0, 1, 90, 1, 200},  /* Use an invalid peek flag. */
        {1, 0, 0, 1, 90, 2, 200},  /* Use an invalid echo flag. */
        {1, 0, 0, 1, 90, 0, 200},  /* Set a distance without an echo. */
    };

    assert(telemetry_codec_payload_size(BUDDY1_TOPIC_SCAN_SAMPLE) == 6U &&
           telemetry_codec_payload_bits(BUDDY1_TOPIC_SCAN_SAMPLE) == 35U &&
           strcmp(telemetry_codec_topic_name(BUDDY1_TOPIC_SCAN_SAMPLE),
                  "car/telemetry/scan_sample") == 0);

    for (angle = MIN_SCAN_ANGLE_DEG; angle <= MAX_SCAN_ANGLE_DEG; angle += 5U)
    {
        for (count = 1; count <= BUDDY1_SCAN_SAMPLES_MAX; ++count)
        {
            for (index = 0; index < count; ++index)
            {
                for (range_index = 0;
                     range_index <= (sizeof ranges_mm / sizeof ranges_mm[0]);
                     ++range_index)
                {
                    input.scan_sample = (telemetry_codec_scan_sample_t){
                        (UB)(angle + count),
                        (UB)(1U == count),
                        (UB)index,
                        (UB)count,
                        (UB)angle,
                        (UB)(range_index <
                             (sizeof ranges_mm / sizeof ranges_mm[0])),
                        (UH)(range_index <
                                     (sizeof ranges_mm / sizeof ranges_mm[0])
                                 ? ranges_mm[range_index]
                                 : 0U)};
                    assert(buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &input,
                                         packet, sizeof packet) == 6U &&
                           ((unsigned)packet[5] >> 3U) == 0U);
                    assert(
                        buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, packet, 6U,
                                      &decoded) &&
                        decoded.scan_sample_valid &&
                        same_sample(&decoded.scan_sample, &input.scan_sample));
                }
            }
        }
    }

    valid_sample = (telemetry_codec_scan_sample_t){1, 0, 0, 1, 90, 1, 200};

    for (size_t case_index = 0U;
         case_index < (sizeof invalid_samples / sizeof invalid_samples[0]);
         case_index++)
    {
        input.scan_sample = invalid_samples[case_index];
        assert(0U == buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &input, packet,
                                   sizeof packet));
    }

    input.scan_sample = valid_sample;
    assert(buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &input, packet,
                         sizeof packet) == 6U);
    packet[3] |= 0x3EU;
    assert(!buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, packet, 6U,
                          &decoded)); /* Use reserved angle index 31. */
    assert(buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &input, packet,
                         sizeof packet) == 6U);
    packet[2] = 0x02U;
    assert(!buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, packet, 6U,
                          &decoded)); /* Index one is outside a count of one. */
    assert(buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &input, packet,
                         sizeof packet) == 6U);
    packet[5] |= 0x08U;
    assert(!buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, packet, 6U,
                          &decoded)); /* padding */
    packet[5] &= 0x07U;
    assert(!buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, packet, 5U,
                          &decoded)); /* length */
    assert(buddy1_invalidate(BUDDY1_TOPIC_SCAN_SAMPLE) ==
           E_PAR); /* Omit the required snapshot section. */
    empty_outbox();
    assert(buddy1_snapshot(&before));
    assert(buddy1_report_scan_sample(0) == E_PAR);
    valid_sample.angle_deg = 151;
    assert(buddy1_report_scan_sample(&valid_sample) == E_PAR);
    assert(buddy1_outbox_idle());
    valid_sample = (telemetry_codec_scan_sample_t){200, 1, 0, 1, 135, 1, 287};
    assert(buddy1_report_scan_sample(&valid_sample) == E_OK);
    assert(!buddy1_outbox_idle()); /* Keep later readings in the queue. */
    assert(buddy1_snapshot(&after) &&
           memcmp(&before, &after, sizeof before) == 0);
    poll_after(20);
    assert(strcmp(g_published_topic, "car/telemetry/scan_sample") == 0 &&
           6U == g_published_len &&
           buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, g_published, g_published_len,
                         &decoded) &&
           same_sample(&decoded.scan_sample, &valid_sample));
    puts("HOST PASS: Buddy 5 scan sample 35 bits/6 bytes: every angle 30..150, "
         "every index/count 1..16, 30/4000 mm limits, no reading=zero, "
         "padding/length rejection, live-only, own topic, outbox-idle gate");
}

/** @brief Fill the outbox and verify that a pending map or snapshot reply is
 * retried and that its command executes once.
 */
static void test_reply_retry (void)
{
    buddy1_command_msg_t   snapshot = {.kind    = BUDDY1_COMMAND_SNAPSHOT,
                                       .payload = {BUDDY1_WIRE_VERSION}};
    buddy1_command_msg_t   map      = {.kind = BUDDY1_COMMAND_BARCODE_MAP};
    telemetry_codec_map_t  wanted   = {'F', 'L', 'R', 'U'};
    buddy1_telemetry_msg_t msg;
    telemetry_codec_map_t  original;
    unsigned               index;
    unsigned               before;
    const BOOL             b_was_open = buddy1_configuration_allowed();
    assert(buddy1_configuration_claim(&original, NULL) == E_OK &&
           buddy1_configuration_reopen() == E_OK);
    assert(buddy1_map_pack(&wanted, map.payload));
    empty_outbox();

    for (index = 0; index < (unsigned)BUDDY1_QUEUE_MESSAGES; ++index)
    {
        assert(buddy1_report_heading((H)index) == E_OK);
    }

    assert(buddy1_command_process(&snapshot) ==
           E_TMOUT); /* Attempt the lock once without sleeping. */
    g_reply_sleeps = 0;
    g_sleep_drains = 0;
    assert(buddy1_command_process_retry(&snapshot, 3U) == E_TMOUT &&
           2U == g_reply_sleeps);
    g_reply_sleeps = 0;
    g_sleep_drains = 1;
    assert(buddy1_command_process_retry(&map, BUDDY1_REPLY_TRIES) == E_OK &&
           1U == g_reply_sleeps);
    assert(buddy1_barcode_action('L') ==
           (unsigned)BUDDY1_ACTION_LEFT); /* Apply the request once. */
    before = outbox_count();

    while (buddy1_outbox_take(&msg))
    {
        /* The last dequeued message is checked below. */
    }

    assert(BUDDY1_QUEUE_MESSAGES == before &&
           BUDDY1_TOPIC_MAP_REPLY == (int)msg.type &&
           BUDDY1_REPLY_OK == (int)msg.payload[1]);

    for (index = 0; index < (unsigned)BUDDY1_QUEUE_MESSAGES; ++index)
    {
        assert(buddy1_report_heading((H)index) == E_OK);
    }

    g_reply_sleeps = 0;
    assert(buddy1_command_process_retry(&snapshot, BUDDY1_REPLY_TRIES) ==
               E_OK &&
           1U == g_reply_sleeps);
    g_sleep_drains = 0;
    empty_outbox();
    assert(buddy1_apply_barcode_map(&original) ==
           E_OK); /* Restore the map and its latch. */
    if (!b_was_open)
    {
        assert(buddy1_configuration_close() == E_OK);
    }

    puts("HOST PASS: map/snapshot replies wait for outbox space (10 ms "
         "retries, bounded), command applied once");
}

/** @brief Run one of the concurrent telemetry producers selected by the
 * fixture argument for 1,000 iterations.
 */
static void * producer (void * p_producer_id)
{
    unsigned index;
    unsigned producer_index = (unsigned)(uintptr_t)p_producer_id;

    for (index = 0; index < SAMPLES_PER_PRODUCER; ++index)
    {
        if (0U == producer_index)
        {
            (void)buddy1_report_heading((H)(index % 360U));
        }
        else if (1U == producer_index)
        {
            (void)buddy1_report_obstacle((UH)index, (UH)(index + 1U),
                                         BUDDY1_ROUTE_LEFT);
        }
        else
        {
            (void)buddy1_update_faults((UB)(1U << (producer_index - 2U)),
                                       (UB)(1U << (producer_index - 2U)));
        }
    }

    return NULL;
}

/** @brief Encode fixture telemetry vectors and print topic labels and
 * payload bytes for comparison.
 */
static void emit_vectors (void)
{
    telemetry_codec_snapshot_t snapshot = {0};
    UB                         packet[32];
    unsigned                   topic;
    unsigned                   index;
    unsigned                   len;
    /* clang-format off */
    snapshot.motion_valid   = snapshot.barcode_valid = snapshot.hump_valid =
        snapshot.heading_valid                     = 1;
    snapshot.obstacle_valid = snapshot.robot_state_valid =
        snapshot.faults_valid = 1;
    /* clang-format on */

    /* clang-format off */
    snapshot.left_mms             = 120;
    snapshot.right_mms            = -118;
    snapshot.barcode_letter       = 'W';
    snapshot.barcode_action       = BUDDY1_ACTION_LEFT;
    snapshot.hump_count           = 2;
    snapshot.hump_height_mm       = 45;
    snapshot.heading_deg          = 359;
    snapshot.obstacle_distance_mm = 180;
    snapshot.obstacle_width_mm    = 140;
    snapshot.obstacle_route       = BUDDY1_ROUTE_LEFT;
    snapshot.robot_state          = BUDDY1_STATE_LINE_FOLLOWING;
    snapshot.faults               = 0x31;
    snapshot.pose_valid           = 1;
    snapshot.pose_distance_cm     = 2049;
    snapshot.pose_heading_deg     = 90;
    snapshot.terrain              = (telemetry_codec_terrain_t){
        1, 123, -51, 5, 1, 23, 1, 41, 1, -87, BUDDY1_SAFETY_NONE};
    snapshot.avoidance            =
        (telemetry_codec_avoidance_t){1, 0, 412, 150, 3, 0, 2, 3, 3, 1};
    snapshot.scan                 =
        (telemetry_codec_scan_t){1, 7, 250, 400, 180, 320, 380, 150, -250};
    snapshot.scan_sample          =
        (telemetry_codec_scan_sample_t){7, 0, 3, 13, 75, 1, 412};
    /* clang-format on */

    for (topic = 0; topic < (unsigned)BUDDY1_TOPIC_COUNT; ++topic)
    {
        if ((unsigned)BUDDY1_TOPIC_MAP_REPLY == topic)
        {
            continue;
        }

        len = buddy1_encode((UB)topic, &snapshot, packet, sizeof(packet));
        assert(len);
        printf("VECTOR %s ", telemetry_codec_topic_name((UB)topic));

        for (index = 0; index < len; ++index)
        {
            printf("%02x", packet[index]);
        }

        puts("");
    }
}

/** @brief Check every encoded hump height and sensor boundary, including
 * rejection of invalid values without changing cached data.
 */
static void test_terrain_widths (void)
{
    telemetry_codec_snapshot_t snapshot = {0};
    telemetry_codec_snapshot_t decoded;
    telemetry_codec_snapshot_t before;
    UB                         payload[BUDDY1_PAYLOAD_MAX];
    assert(BUDDY1_PAYLOAD_MAX == 17 &&
           telemetry_codec_payload_bits(BUDDY1_TOPIC_HUMP) == 25U &&
           telemetry_codec_payload_bits(BUDDY1_TOPIC_OBSTACLE) == 26U);

    for (unsigned height = 0; height < HUMP_HEIGHT_CODES; ++height)
    {
        snapshot.hump_count     = 65535;
        snapshot.hump_height_mm = (UH)height;
        unsigned encoded_length = buddy1_encode(BUDDY1_TOPIC_HUMP, &snapshot,
                                                payload, sizeof payload);
        assert((encoded_length != 0U) == (height <= 500U));
        payload[0] = BUDDY1_WIRE_VERSION;
        payload[1] = 255;
        payload[2] = 255;
        payload[3] = (UB)height;
        payload[4] = (UB)(height >> 8);
        assert((buddy1_decode(BUDDY1_TOPIC_HUMP, payload, 5, &decoded) != 0) ==
               (height <= 500U));

        if (height <= 500U)
        {
            assert(decoded.hump_height_mm == height &&
                   65535 == (int)decoded.hump_count);
        }
    }

    static const unsigned distances[] = {0, 29, 30, 4000, 4001, 4095};
    snapshot.obstacle_width_mm        = 4095;
    snapshot.obstacle_route           = BUDDY1_ROUTE_RIGHT;

    for (unsigned case_index = 0;
         case_index < (sizeof distances / sizeof distances[0]); ++case_index)
    {
        unsigned distance             = distances[case_index];
        snapshot.obstacle_distance_mm = (UH)distance;
        assert((buddy1_encode(BUDDY1_TOPIC_OBSTACLE, &snapshot, payload,
                              sizeof payload) != 0U) ==
               (distance >= 30U && distance <= 4000U));
        uint32_t body = distance | (4095U << 12) | (2U << 24);
        payload[0]    = BUDDY1_WIRE_VERSION;

        for (unsigned byte_index = 0; byte_index < MOTION_PACKET_BYTES;
             ++byte_index)
        {
            payload[1U + byte_index] = (UB)(body >> (8U * byte_index));
        }

        assert((buddy1_decode(BUDDY1_TOPIC_OBSTACLE, payload, 5, &decoded) !=
                0) == (distance >= 30U && distance <= 4000U));
    }

    assert(buddy1_snapshot(&before));
    assert(buddy1_report_hump(0, 501) == E_PAR && buddy1_snapshot(&decoded) &&
           decoded.hump_height_mm == before.hump_height_mm);
    /* A same-length v2 packet must never be reinterpreted as v3. */
    snapshot.hump_height_mm = 1;
    assert(buddy1_encode(BUDDY1_TOPIC_HUMP, &snapshot, payload,
                         sizeof payload) == 5U);
    payload[0] = 2;
    assert(!buddy1_decode(BUDDY1_TOPIC_HUMP, payload, 5, &decoded));
    puts("HOST PASS: v3 9/12-bit terrain fields, exhaustive height codes, "
         "sensor boundaries, no clipping/cache corruption, v2 rejection");
}

/* End of file. */

