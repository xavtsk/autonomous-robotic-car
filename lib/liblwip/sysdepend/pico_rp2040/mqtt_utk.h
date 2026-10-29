/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

#define MQTT_UTK_APP_WIRE_VERSION 3U /* binary fallback heartbeat contract */
#define MQTT_UTK_APP_TOPIC_MAX    64
#define MQTT_UTK_APP_PAYLOAD_MAX 14

typedef struct {
	char		topic[MQTT_UTK_APP_TOPIC_MAX];
	unsigned char	payload[MQTT_UTK_APP_PAYLOAD_MAX];
	unsigned short	payload_len;
	unsigned char	qos;
	unsigned char	retain;
} T_MQTT_UTK_APP_MSG;

/*
 * Call once, from cyw43_utk_task() the first time the link reaches
 * CYW43_LINK_UP (see cyw43_utk.c). Starts the MQTT connection to
 * MQTT_BROKER_IP:MQTT_BROKER_PORT (config/wifi_secrets.h) and, once
 * connected, a periodic heartbeat publish on MQTT_TOPIC. Safe to call more
 * than once -- a second call while already connecting/connected is a no-op.
 */
void mqtt_utk_start(void);

/* Network-owner task only. Attempts at most one application publish per call,
 * spaced by at least 20 ms (100 ms after transient rejection). One fixed retry
 * slot retains the packet until completion, including across disconnects;
 * no later FIFO packet is removed while it is pending. No blocking wait.
 * A disconnect/timeout may make delivery ambiguous: retries can duplicate a
 * packet, and QoS 0 still does not provide broker acknowledgement. */
void mqtt_utk_poll(void);

/* Application hook. The weak default returns no message; Buddy 1 overrides
 * it in app_program/subsystem1/mqtt_bridge.c. */
int mqtt_utk_app_take(T_MQTT_UTK_APP_MSG *out);

/* Optional low-priority binary barcode capture packets on
 * MQTT_UTK_DEBUG_TOPIC. Asked for one packet only when no application message
 * is waiting, so telemetry always goes first; same single in-flight slot and
 * pacing. Return the packet length, or 0 when there is nothing to send. The
 * weak default sends nothing. Called from the network task. */
#define MQTT_UTK_DEBUG_TOPIC       "car/debug/barcode"
#if APP_IR_DEBUG
#define MQTT_UTK_DEBUG_PAYLOAD_MAX 70 /* shared with controller preview */
#else
#define MQTT_UTK_DEBUG_PAYLOAD_MAX 47 /* maximum binary capture packet */
#endif
unsigned int mqtt_utk_debug_take(unsigned char *bytes, unsigned int size);

/* Optional IR_DEBUG=1 preview, after ordinary telemetry and barcode uploads.
 * Latest-only binary diagnostics; stale/rejected previews are dropped, never
 * retried ahead of application messages. No new incoming robot command. */
#define MQTT_UTK_IR_DEBUG_TOPIC "car/debug/ir"
#define MQTT_UTK_IR_DEBUG_MAX 16U
unsigned int mqtt_utk_ir_debug_take(unsigned char *bytes, unsigned int size);
#define MQTT_UTK_CONTROL_DEBUG_TOPIC "car/debug/controller"
#define MQTT_UTK_CONTROL_DEBUG_MAX 70U
unsigned int mqtt_utk_control_debug_take(unsigned char *bytes, unsigned int size);

/* Called after a complete inbound MQTT payload has arrived. The weak default
 * ignores it; Buddy 1 overrides it and copies permitted commands into its
 * RTOS message buffer. */
void mqtt_utk_app_receive(const char *topic,
			  const unsigned char *payload,
			  unsigned short payload_len);

/* Application binary heartbeat and link observation hooks, network task only.
 * Link returns zero while application objects are not ready; caller retries. */
unsigned short mqtt_utk_app_status(unsigned char *payload, unsigned short capacity);
int mqtt_utk_app_link(int up);

#endif /* MQTT_UTK_H */
