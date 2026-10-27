#if defined(TM_MQTT) && TM_MQTT

#include "subsystem1.h"
#include "mqtt_utk.h"

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
	BUDDY1_TELEMETRY_MSG message;
	INT i;
	const char *topic;

	if (out == NULL || !buddy1_outbox_take(&message)) return 0;
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

void mqtt_utk_app_receive(const char *topic,
			  const unsigned char *payload,
			  unsigned short payload_len)
{
	if (topic == NULL || payload == NULL || !has_command_prefix(topic)) return;
	(void)buddy1_command_enqueue(topic, payload, payload_len);
}

#endif
