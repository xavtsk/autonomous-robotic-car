#if defined(TM_MQTT) && TM_MQTT

#include "subsystem1.h"
#include "mqtt_utk.h"
#if APP_S3_MOTION
#include "../controller/remote_control.h"
extern unsigned int lwip_utk_rand(void);

static int remote_control_topic(const char *topic)
{
	static const char wanted[]="car/cmd/control";
	unsigned i=0;
	while (wanted[i] && topic[i]==wanted[i]) ++i;
	return wanted[i]=='\0' && topic[i]=='\0';
}

static int remote_take(T_MQTT_UTK_APP_MSG *out,int status_allowed)
{
	unsigned n=s3_remote_reply_take(out->payload,MQTT_UTK_APP_PAYLOAD_MAX);
	const char *topic="car/reply/control";
	if (!n && status_allowed) { n=s3_remote_status_take(out->payload,MQTT_UTK_APP_PAYLOAD_MAX); topic="car/telemetry/control"; }
	if (!n) return 0;
	for (unsigned i=0;i<MQTT_UTK_APP_TOPIC_MAX;i++) {
		out->topic[i]=topic[i];
		if (!topic[i]) break;
	}
	out->topic[MQTT_UTK_APP_TOPIC_MAX-1]='\0';
	out->payload_len=(unsigned short)n;
	out->qos=n==REMOTE_REPLY_SIZE?1:0; out->retain=0;
	out->routine=n==REMOTE_STATUS_SIZE;
	return 1;
}
#endif

_Static_assert(BUDDY1_WIRE_VERSION == MQTT_UTK_APP_WIRE_VERSION,
               "fallback heartbeat and application versions must agree");
_Static_assert(BUDDY1_PAYLOAD_MAX <= MQTT_UTK_APP_PAYLOAD_MAX,
               "network staging must hold a complete binary snapshot");

static int has_command_prefix(const char *topic)
{
	static const char prefix[] = "car/cmd/";
	INT i;
	for (i = 0; prefix[i] != '\0'; i++) {
		if (topic[i] != prefix[i]) return 0;
	}
	return 1;
}

int mqtt_utk_app_take(T_MQTT_UTK_APP_MSG *out)
{
#if APP_S3_MOTION
	static int status_turn=1;
#endif
	BUDDY1_TELEMETRY_MSG message;
	INT i;
	const char *topic;

	if (out == NULL) return 0;
#if APP_S3_MOTION
	if (remote_take(out,status_turn)) {
		if (out->routine) status_turn=0;
		return 1;
	}
#endif
	if (!buddy1_outbox_take(&message)) {
#if APP_S3_MOTION
		const int taken=remote_take(out,1);
		if (taken && out->routine) status_turn=0;
		return taken;
#else
		return 0;
#endif
	}
#if APP_S3_MOTION
	/* On a slow link a timer alone could select status on every call. Give
	 * ordinary queued telemetry a turn between recurring control packets. */
	status_turn=1;
#endif
	topic = buddy1_topic_name(message.type);
	if (topic == NULL) return 0;
	for (i = 0; i < MQTT_UTK_APP_TOPIC_MAX; i++) {
		out->topic[i] = topic[i];
		if (topic[i] == '\0') break;
	}
	out->topic[MQTT_UTK_APP_TOPIC_MAX - 1] = '\0';
	out->payload_len = (unsigned short)buddy1_payload_size(message.type);
	if (out->payload_len > MQTT_UTK_APP_PAYLOAD_MAX) return 0;
	for (i = 0; i < (INT)out->payload_len; i++) {
		out->payload[i] = message.payload[i];
	}
	out->qos = (message.type == BUDDY1_TOPIC_SNAPSHOT ||
	            message.type == BUDDY1_TOPIC_MAP_REPLY) ? 1 : 0;
	out->retain = 0;
	/* Only recurring samples yield slots to barcode uploads. Faults, state,
	 * barcode events and replies retain priority without duplicating topic
	 * names or application enums in the network layer. */
	out->routine = (message.type == BUDDY1_TOPIC_MOTION ||
	                message.type == BUDDY1_TOPIC_POSE ||
	                message.type == BUDDY1_TOPIC_TERRAIN);
	return 1;
}

unsigned short mqtt_utk_app_status(unsigned char *payload, unsigned short capacity)
{
	BUDDY1_SNAPSHOT snapshot;
	if (mtx_buddy1_state <= 0 || !buddy1_snapshot(&snapshot)) return 0;
	return (unsigned short)buddy1_encode(BUDDY1_TOPIC_STATUS, &snapshot, payload, capacity);
}

int mqtt_utk_app_link(int up)
{
	ER er = buddy1_update_faults(BUDDY1_FAULT_WIFI, up ? 0 : BUDDY1_FAULT_WIFI);
	/* E_TMOUT still means the fault mirror was updated. Only the live report
	 * was dropped; the next heartbeat/snapshot carries the current mask. */
	return er == E_OK || er == E_TMOUT;
}

void mqtt_utk_app_connection(int up)
{
#if APP_S3_MOTION
	/* Broker reconnects can happen while Wi-Fi stays up. Each connection
	 * needs its own freshness namespace, including that case. */
	s3_remote_link(up?lwip_utk_rand():0U,up!=0);
#else
	(void)up;
#endif
}

void mqtt_utk_app_receive(const char *topic,
			  const unsigned char *payload,
			  unsigned short payload_len)
{
	if (topic == NULL || payload == NULL || !has_command_prefix(topic)) return;
#if APP_S3_MOTION
	/* Bypass the ordinary command FIFO: STOP must replace pending START even
	 * when the snapshot/map queue or telemetry outbox is full. */
	if (remote_control_topic(topic)) {
		s3_remote_receive(payload,payload_len);
		return;
	}
#endif
	(void)buddy1_command_enqueue(topic, payload, payload_len);
}

#endif
