/** @file mqtt_utk.h
 * @brief Declare the kernel MQTT lifecycle, publishing and application callback
 * interface.
 */
/* These declarations connect the MQTT client to micro T-Kernel on RP2040
 * with NO_SYS set to 1.
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

#include <stdint.h>

/* The fallback heartbeat uses application wire version 3. */
#define MQTT_UTK_APP_WIRE_VERSION 3U
#define MQTT_UTK_APP_TOPIC_MAX 64
/* The version and 124-bit snapshot fit in 17 bytes. */
#define MQTT_UTK_APP_PAYLOAD_MAX 17

/** @brief Application packet copied into the network owner's retry slot.
 * Topic is NUL-terminated; payload_len is a byte count, at most
 * MQTT_UTK_APP_PAYLOAD_MAX for publication. qos/retain are MQTT options.
 * This structure is local storage, never a native-layout wire packet.
 */
typedef struct
{
    char     topic[MQTT_UTK_APP_TOPIC_MAX];
    uint8_t  payload[MQTT_UTK_APP_PAYLOAD_MAX];
    uint16_t payload_len;
    uint8_t  qos;
    uint8_t  retain;
    /* Local scheduling metadata, never transmitted. Zero preserves priority
     * for events/replies; recurring samples may share slots with captures. */
    uint8_t routine;
} T_MQTT_UTK_APP_MSG;

/** @brief Start the client from the network-owner task after Wi-Fi connects.
 * Call once, from cyw43_utk_task() the first time the link reaches
 * CYW43_LINK_UP (see cyw43_utk.c). Starts the MQTT connection to
 * MQTT_BROKER_IP:MQTT_BROKER_PORT (config/wifi_secrets.h) and, once
 * connected, a periodic heartbeat publish on MQTT_TOPIC. Safe to call more
 * than once -- a second call while already connecting/connected is a no-op.
 */
void mqtt_utk_start (void);

/** @brief Advance publication/retry scheduling from the network-owner task.
 * Network-owner task only. Attempts at most one application publish per call,
 * spaced by at least 20 ms (100 ms after transient rejection). One fixed retry
 * slot retains the packet until completion, including across disconnects;
 * no later FIFO packet is removed while it is pending. Routine telemetry
 * alternates with waiting barcode packets. Events/replies at the FIFO head
 * precede a fresh barcode packet; an attempted packet always finishes/retries
 * first. No blocking wait.
 * A disconnect/timeout may make delivery ambiguous: retries can duplicate a
 * packet, and QoS 0 still does not provide broker acknowledgement. */
void mqtt_utk_poll (void);

/** @brief Take one application packet from the network-owner task.
 * @param[out] p_out Writable caller-owned packet; not retained by the hook.
 * @return One for a packet, zero otherwise. The weak default writes nothing;
 * the Buddy 1 implementation also rejects NULL. Its rejected message may
 * already be dequeued and may leave partially written output.
 */
int mqtt_utk_app_take (T_MQTT_UTK_APP_MSG * p_out);

/* Optional binary barcode capture packets on MQTT_UTK_DEBUG_TOPIC. Alternate
 * with routine application samples while both are waiting; other FIFO-head
 * events/replies take priority. Same single in-flight slot and pacing. Return
 * the packet length, or 0 when there is nothing to send. The weak default
 * sends nothing. Called from the network task. */
#define MQTT_UTK_DEBUG_TOPIC "car/debug/barcode"
/* Reserve room for a binary capture with up to 24 samples. */
#define MQTT_UTK_DEBUG_PAYLOAD_MAX 164
/** @brief Take a barcode capture packet from the network-owner task.
 * @param[out] p_bytes Caller-owned writable buffer of size bytes.
 * @param size Buffer capacity in bytes; the hook must not exceed it.
 * @return Packet byte count, or zero when no packet is supplied. The weak
 * default returns zero without touching the buffer. No pointer is retained.
 */
unsigned int mqtt_utk_debug_take (uint8_t * p_bytes, unsigned int size);

/* Best-effort diagnostics. IR uses spare capacity. Controller observations
 * get at most 5 Hz, then raw IMU batches get a turn. On a busy link each
 * requires two successful application/capture packets between its own sends.
 * FIFO-head events and exact pending retries keep priority. A rejected
 * diagnostic is dropped; the receiver uses the IMU sequence to find gaps. */
#define MQTT_UTK_IR_DEBUG_TOPIC "car/debug/ir"
#define MQTT_UTK_IR_DEBUG_MAX 16U
/** @brief Take a best-effort IR observation from the network-owner task.
 * @param[out] p_bytes Caller-owned writable buffer of size bytes.
 * @param size Buffer capacity in bytes; the hook must not exceed it.
 * @return Packet byte count, or zero when unavailable. The weak default leaves
 * output unchanged. No pointer is retained; publication rejection drops it.
 */
unsigned int mqtt_utk_ir_debug_take (uint8_t * p_bytes, unsigned int size);
#define MQTT_UTK_CONTROL_DEBUG_TOPIC "car/debug/controller"
#define MQTT_UTK_CONTROL_DEBUG_MAX 70U
/** @brief Take a controller observation from the network-owner task.
 * @param[out] p_bytes Caller-owned writable buffer of size bytes.
 * @param size Buffer capacity in bytes; the hook must not exceed it.
 * @return Packet byte count, or zero when unavailable. The weak default leaves
 * output unchanged. No pointer is retained; publication rejection drops it.
 */
unsigned int mqtt_utk_control_debug_take (uint8_t * p_bytes, unsigned int size);
#define MQTT_UTK_IMU_DEBUG_TOPIC "car/debug/imu"
#define MQTT_UTK_IMU_DEBUG_MAX 156U
/** @brief Take a raw IMU batch from the network-owner task.
 * @param[out] p_bytes Caller-owned writable buffer of size bytes.
 * @param size Buffer capacity in bytes; the hook must not exceed it.
 * @return Packet byte count, or zero when unavailable. The weak default leaves
 * output unchanged. No pointer is retained; publication rejection drops it.
 */
unsigned int mqtt_utk_imu_debug_take (uint8_t * p_bytes, unsigned int size);

/** @brief Handle a complete inbound payload in the network-owner task.
 * @param p_topic Borrowed NUL-terminated topic, valid during this call only.
 * @param p_payload Borrowed payload_len bytes, valid during this call only.
 * @param payload_len Complete payload length in bytes (0 through 65535);
 * the network receiver currently accepts at most MQTT_UTK_APP_PAYLOAD_MAX.
 * @note The weak default ignores input. Buddy 1 rejects NULL pointers and
 * non-command topics, copies ordinary permitted commands to its RTOS queue,
 * and dispatches remote control separately. Queue/validation failures drop
 * the command; neither implementation retains the input pointers.
 */
void mqtt_utk_app_receive (const char * p_topic, const uint8_t * p_payload,
                           uint16_t payload_len);

/** @brief Encode a heartbeat from the network-owner task.
 * @param[out] p_payload Caller-owned writable buffer of capacity bytes.
 * @param capacity Available byte count (0 through 65535).
 * @return Encoded byte count, or zero when unavailable/too small. The weak
 * default requires a non-NULL buffer when capacity is at least two; it writes
 * two fallback bytes then. Buddy 1 supplies its snapshot encoder instead.
 * @note No pointer is retained. Failure/output effects are implementation
 * specific; the Buddy 1 encoder may leave partial output on rejection.
 */
uint16_t mqtt_utk_app_status (uint8_t * p_payload, uint16_t capacity);
/** @brief Observe Wi-Fi link state from the network-owner task.
 * @param b_is_up Zero means down; any nonzero value means up.
 * @return Nonzero once the observation is accepted; zero asks the caller to
 * retry when application objects are unavailable. The weak default accepts.
 */
int mqtt_utk_app_link (int b_is_up);
/** @brief Observe a broker connection change from the network-owner task.
 * @param b_is_connected Zero means disconnected; any nonzero value means
 * connected.
 * @note Distinct from Wi-Fi link state. The weak default is a no-op;
 * the application may reset remote-control connection freshness state.
 */
void mqtt_utk_app_connection (int b_is_connected);

#endif /* MQTT_UTK_H */

/* End of file. */

