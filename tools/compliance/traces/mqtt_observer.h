/** @file mqtt_observer.h
 * @brief Observe deterministic network callbacks after the concurrent producer
 * test joins.
 */
#ifndef TOOLS_COMPLIANCE_TRACES_MQTT_OBSERVER_H
#define TOOLS_COMPLIANCE_TRACES_MQTT_OBSERVER_H

/** @brief Observe one announcement, then delegate to the registered callback.
 */
static void trace_incoming (void * p_context, const char * p_topic,
                            u32_t length)
{
    printf("MQTT_RX %u %s %u\n", g_clock_ms, p_topic, length);
    gp_incoming(p_context, p_topic, length);
    printf("MQTT_QUEUE incoming %u %u\n", g_buffers[0].count,
           g_buffers[1].count);
}

/** @brief Trace the fragment bytes and queue counts around their delivery. */
static void trace_fragment (void * p_context, const u8_t * p_data, u16_t length,
                            u8_t flags)
{
    printf("MQTT_FRAGMENT %u %u %u ", g_clock_ms, length, flags);
    for (unsigned index = 0; index < length; ++index)
    {
        printf("%02x", p_data[index]);
    }
    putchar('\n');
    gp_fragment(p_context, p_data, length, flags);
    printf("MQTT_QUEUE fragment %u %u\n", g_buffers[0].count,
           g_buffers[1].count);
}

/** @brief Record an outbound payload and the fake broker's publish result. */
static void trace_publish (const char * p_topic, const void * p_payload,
                           u16_t length, u8_t qos, u8_t retain)
{
    const u8_t * p_bytes = p_payload;
    printf("MQTT_TX %u %s %u %u %u outcome=%d ", g_clock_ms, p_topic, length,
           qos, retain, g_publish_result);
    for (unsigned index = 0; index < length; ++index)
    {
        printf("%02x", p_bytes[index]);
    }
    putchar('\n');
}
#endif /* TOOLS_COMPLIANCE_TRACES_MQTT_OBSERVER_H */
/* End of file. */

