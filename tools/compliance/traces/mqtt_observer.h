/* Observe deterministic network callbacks after the concurrent producer test joins. */
static host_publish_cb trace_original_incoming;
static host_data_cb trace_original_fragment;

static void trace_incoming(void *context, const char *topic, u32_t length)
{
    printf("MQTT_RX %u %s %u\n", fake_now, topic, length);
    trace_original_incoming(context, topic, length);
    printf("MQTT_QUEUE incoming %u %u\n", buffers[0].count, buffers[1].count);
}

static void trace_fragment(void *context, const u8_t *data, u16_t length, u8_t flags)
{
    printf("MQTT_FRAGMENT %u %u %u ", fake_now, length, flags);
    for (unsigned index=0; index<length; ++index)
    {
        printf("%02x", data[index]);
    }
    putchar('\n');
    trace_original_fragment(context, data, length, flags);
    printf("MQTT_QUEUE fragment %u %u\n", buffers[0].count, buffers[1].count);
}

static void trace_publish(const char *topic, const void *payload, u16_t length,
                          u8_t qos, u8_t retain)
{
    const u8_t *bytes=payload;
    printf("MQTT_TX %u %s %u %u %u outcome=%d ", fake_now, topic, length,
           qos, retain, publish_result);
    for (unsigned index=0; index<length; ++index)
    {
        printf("%02x", bytes[index]);
    }
    putchar('\n');
}
