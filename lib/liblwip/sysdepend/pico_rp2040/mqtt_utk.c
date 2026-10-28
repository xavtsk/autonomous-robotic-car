/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	The lwIP clock declaration needs lwip/sys.h. Keep the toolchain size_t
 *	definition alongside the kernel's monitor types, as in sys_arch.c.
 *	No kernel or GPIO work is done by publish callbacks.
 */

#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_HEARTBEAT_MS   5000
#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000
#define MQTT_UTK_PUBLISH_GAP_MS    20U
#define MQTT_UTK_PUBLISH_RETRY_MS 100U
#define MQTT_UTK_COMMAND_TOPIC  "car/cmd/#"

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks. Preserve raw bytes, including zero. Reject the
 * entire packet if its topic/length is oversized or fragments do not match. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    MQTT_UTK_APP_PAYLOAD_MAX

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_truncated;
static u16_t             rx_expected;

/* A fixed, single-owner retry slot, not another queue or a larger lwIP pool.
 * Retain the complete packet until its publish callback succeeds. At most one
 * application publish is in flight, leaving request slots for subscriptions
 * and heartbeat. The bounded application FIFO applies backpressure behind it.
 * No blocking waits or extra timeout objects are needed. */
static T_MQTT_UTK_APP_MSG pending;
static u32_t last_publish_ms;
static u16_t publish_wait_ms;
static u8_t pending_valid, pending_inflight, pending_warned, publish_paced;
/* Diagnostic text slot: used only while no application packet is pending. */
static char debug_line[MQTT_UTK_DEBUG_PAYLOAD_MAX];
static u16_t debug_len;
static u8_t debug_valid;

static void heartbeat_timeout(void *arg);

static void pace_publish(u16_t wait_ms)
{
	last_publish_ms = sys_now();
	publish_wait_ms = wait_ms;
	publish_paced = 1;
}

static void warn_pending(err_t result)
{
	if(!pending_warned) {
		tm_printf((UB*)"mqtt: application publish delayed, err=%d; retaining packet\n",
		          (int)result);
		pending_warned = 1;
	}
}

static void application_publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	pending_inflight = 0;
	if(result == ERR_OK) {
		pending_valid = 0;
		pending_warned = 0;
	} else {
		/* A request timeout is not proof that the broker never received it.
		 * Retry is at-least-once best effort; consumers must tolerate repeats. */
		warn_pending(result);
		pace_publish(MQTT_UTK_PUBLISH_RETRY_MS);
	}
}

static void debug_publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	pending_inflight = 0;
	if(result == ERR_OK) {
		debug_valid = 0;
	} else {
		pace_publish(MQTT_UTK_PUBLISH_RETRY_MS);
	}
}

__attribute__((weak)) unsigned int mqtt_utk_debug_take(char *line, unsigned int size)
{
	(void)line;
	(void)size;
	return 0;
}

__attribute__((weak)) unsigned int mqtt_utk_ir_debug_take(unsigned char *bytes, unsigned int size)
{
	(void)bytes;
	(void)size;
	return 0U;
}
__attribute__((weak)) unsigned int mqtt_utk_control_debug_take(unsigned char *bytes, unsigned int size)
{
	(void)bytes; (void)size; return 0U;
}

#if APP_IR_DEBUG
static void ir_debug_publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	(void)result;
	pending_inflight = 0; /* best effort: never retry an old preview */
}
#endif

__attribute__((weak)) int mqtt_utk_app_take(T_MQTT_UTK_APP_MSG *out)
{
	(void)out;
	return 0;
}

__attribute__((weak)) void mqtt_utk_app_receive(const char *topic,
					 const unsigned char *payload,
					 unsigned short payload_len)
{
	(void)topic;
	(void)payload;
	(void)payload_len;
}

__attribute__((weak)) unsigned short mqtt_utk_app_status(unsigned char *payload,
                                                       unsigned short capacity)
{
	if (capacity < 2) return 0;
	payload[0] = MQTT_UTK_APP_WIRE_VERSION; payload[1] = 1; /* alive, faults unknown */
	return 2;
}

__attribute__((weak)) int mqtt_utk_app_link(int up)
{
	(void)up;
	return 1;
}

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;

	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_truncated = (topic[i] != '\0' || tot_len > MQTT_UTK_RX_DATA_MAX);
	rx_expected = (tot_len <= MQTT_UTK_RX_DATA_MAX) ? (u16_t)tot_len : 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	for(i = 0; i < len; i++) {
		if(rx_len < MQTT_UTK_RX_DATA_MAX) {
			rx_data[rx_len++] = data[i];
		} else {
			rx_truncated = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		if(rx_len != rx_expected) rx_truncated = 1;
		tm_printf((UB*)"mqtt: RX [%s] %u binary bytes%s\n", rx_topic,
		          (unsigned)rx_len, rx_truncated? " (rejected)": "");
		if(!rx_truncated) {
			mqtt_utk_app_receive(rx_topic, rx_data, rx_len);
		}
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " MQTT_TOPIC "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void command_subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " MQTT_UTK_COMMAND_TOPIC "\n");
	} else {
		tm_printf((UB*)"mqtt: command subscribe failed, err=%d\n",
			  (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
	}
}

static void heartbeat_timeout(void *arg)
{
	UB payload[MQTT_UTK_APP_PAYLOAD_MAX];
	unsigned short len;

	(void)arg;

	if(mqtt_client_is_connected(client)) {
		len = mqtt_utk_app_status(payload, sizeof(payload));
		if (len > 0 && len <= sizeof(payload))
		(void)mqtt_publish(client, MQTT_TOPIC, payload, (u16_t)len,
				   0 /* qos */, 0 /* retain */,
				   publish_done_cb, NULL);
	}

	/* Re-arms itself every call -- mqtt.c's connection callback (below)
	 * only starts this chain once, on the transition into MQTT_CONNECT_
	 * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
	sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;
		pending_inflight = 0;

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Subscribe to the same topic this client publishes to, so every
		 * heartbeat comes back from the broker and is printed by
		 * incoming_data_cb(). A matching RX for each TX is end-to-end
		 * proof of the round trip without needing mosquitto_sub on the
		 * broker host. Callbacks must be registered before subscribing,
		 * or the first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, MQTT_TOPIC, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		err = mqtt_subscribe(client, MQTT_UTK_COMMAND_TOPIC, 1 /* qos */,
				     command_subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: command mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating heartbeat, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(heartbeat_timeout, NULL);
		sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
	} else {
		/* lwIP clears requests without completion callbacks on close.
		 * Keep our application packet for the next connection and schedule
		 * the existing connection retry without adding a publish timer. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		pending_inflight = 0;
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}

void mqtt_utk_poll(void)
{
	err_t err;

	if(client == NULL || !mqtt_client_is_connected(client)) {
		pending_inflight = 0;
		return;
	}
	if(pending_inflight ||
	   (publish_paced && (u32_t)(sys_now() - last_publish_ms) < publish_wait_ms)) {
		return;
	}

	if(!pending_valid) {
		if(!mqtt_utk_app_take(&pending)) {
			/* Nothing for telemetry: send one diagnostic line, if any. */
			if(!debug_valid) {
				unsigned int n = mqtt_utk_debug_take(debug_line, sizeof debug_line);
				if(n == 0U) {
#if APP_IR_DEBUG
					const char *topic = MQTT_UTK_CONTROL_DEBUG_TOPIC;
					n = mqtt_utk_control_debug_take((unsigned char *)debug_line, MQTT_UTK_CONTROL_DEBUG_MAX);
					if(n > MQTT_UTK_CONTROL_DEBUG_MAX) return;
					if(n == 0U) {
						topic = MQTT_UTK_IR_DEBUG_TOPIC;
						n = mqtt_utk_ir_debug_take((unsigned char *)debug_line, MQTT_UTK_IR_DEBUG_MAX);
						if(n > MQTT_UTK_IR_DEBUG_MAX) return;
					}
					if(n == 0U) return;
					pending_inflight = 1;
					pace_publish(MQTT_UTK_PUBLISH_GAP_MS);
					err = mqtt_publish(client, topic, debug_line,
					                  (u16_t)n, 0, 0, ir_debug_publish_done_cb, NULL);
					if(err != ERR_OK) pending_inflight = 0;
#endif
					return;
				}
				if(n > sizeof debug_line) return;
				debug_len = (u16_t)n;
				debug_valid = 1;
			}
			pending_inflight = 1;
			pace_publish(MQTT_UTK_PUBLISH_GAP_MS);
			err = mqtt_publish(client, MQTT_UTK_DEBUG_TOPIC, debug_line,
			                   debug_len, 0, 0, debug_publish_done_cb, NULL);
			if(err != ERR_OK) {
				pending_inflight = 0;
				if(err == ERR_MEM || err == ERR_CONN) {
					pace_publish(MQTT_UTK_PUBLISH_RETRY_MS);
				} else {
					debug_valid = 0; /* cannot improve with a retry */
				}
			}
			return;
		}
		pending.topic[MQTT_UTK_APP_TOPIC_MAX - 1] = '\0';
		if(pending.payload_len > MQTT_UTK_APP_PAYLOAD_MAX ||
		   pending.topic[0] == '\0' || pending.qos > 2 || pending.retain > 1) {
			tm_printf((UB*)"mqtt: invalid application packet discarded\n");
			pace_publish(MQTT_UTK_PUBLISH_GAP_MS);
			return;
		}
		pending_valid = 1;
		pending_warned = 0;
	}

	/* Set state before calling lwIP; test ports may complete synchronously.
	 * The next packet cannot overtake this one or consume another request slot. */
	pending_inflight = 1;
	pace_publish(MQTT_UTK_PUBLISH_GAP_MS);
	err = mqtt_publish(client, pending.topic, pending.payload,
	                   (u16_t)pending.payload_len, pending.qos, pending.retain,
	                   application_publish_done_cb, NULL);
	if(err != ERR_OK) {
		pending_inflight = 0;
		if(err == ERR_MEM || err == ERR_CONN) {
			warn_pending(err);
			pace_publish(MQTT_UTK_PUBLISH_RETRY_MS);
		} else {
			/* Bad arguments cannot improve with a retry. Drop this one packet
			 * explicitly, then allow the next FIFO item on a later poll. */
			tm_printf((UB*)"mqtt: application packet discarded, err=%d\n", (int)err);
			pending_valid = 0;
			pending_warned = 0;
		}
	}
}
