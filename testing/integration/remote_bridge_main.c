/** @file remote_bridge_main.c
 * @brief Production MQTT bridge routing; remote owner is stubbed, never motors.
 * The selected narrow integer operands fit in int.
 */
#include "../subsystem1/host_kernel.h"
#include "../../app_program/subsystem1/buddy1.h"
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"
#include "remote_bridge_main.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
ID              gh_buddy1_state_mutex = 1;
static unsigned g_ordinary;
static unsigned g_received;
static unsigned g_reply_ready;
static unsigned g_status_ready;
static unsigned g_outbox_ready;
static uint32_t g_linked_epoch;
static bool     gb_is_linked_up;

/** @brief Return the fixed fixture word used to make the simulated network
 * epoch reproducible.
 */
unsigned int lwip_utk_rand (void)
{
    return 0x11223344U;
}

/** @brief Save the supplied epoch and link state for inspection by the
 * bridge test.
 */
void remote_control_link (uint32_t epoch, bool b_is_up)
{
    g_linked_epoch  = epoch;
    gb_is_linked_up = b_is_up;
}

/** @brief Require a version-three STOP packet of the expected size and count
 * its delivery.
 */
void remote_control_receive (const uint8_t * p_packet, unsigned length)
{
    assert(REMOTE_COMMAND_SIZE == length && 3 == (int)p_packet[0] &&
           REMOTE_STOP == (int)p_packet[1]);
    g_received++;
}

/** @brief Consume a staged compact reply or return zero if none is ready.
 * Check the supplied buffer capacity.
 */
unsigned remote_control_reply_take (uint8_t * p_packet, unsigned capacity)
{
    assert(capacity >= REMOTE_REPLY_SIZE);

    if (!g_reply_ready)
    {
        return 0;
    }

    g_reply_ready = 0;
    memset(p_packet, 0, REMOTE_REPLY_SIZE);
    p_packet[0] = REMOTE_REPLY_VERSION;
    return REMOTE_REPLY_SIZE;
}

/** @brief Consume a staged version-three status or return zero if none is
 * ready. Check the supplied buffer capacity.
 */
unsigned remote_control_status_take (uint8_t * p_packet, unsigned capacity)
{
    assert(capacity >= REMOTE_STATUS_SIZE);

    if (!g_status_ready)
    {
        return 0;
    }

    g_status_ready = 0;
    memset(p_packet, 0, REMOTE_STATUS_SIZE);
    p_packet[0] = 3;
    return REMOTE_STATUS_SIZE;
}

/** @brief Pop one staged motion publication or return zero when the
 * simulated outbox is empty.
 */
BOOL buddy1_outbox_take (buddy1_telemetry_msg_t * p_message)
{
    if (!g_outbox_ready)
    {
        return 0;
    }

    --g_outbox_ready;
    *p_message =
        (buddy1_telemetry_msg_t){.type = BUDDY1_TOPIC_MOTION, .payload = {3}};
    return 1;
}

/** @brief Count an ordinary command and return the fixture's timeout result.
 */
ER buddy1_command_enqueue (const char * p_topic, const UB * p_packet, UH length)
{
    (void)p_topic;
    (void)p_packet;
    (void)length;
    g_ordinary++;
    return E_TMOUT;
}

/** @brief Acknowledge fault updates without retaining the changed bits.
 */
ER buddy1_update_faults (UB mask, UB value)
{
    (void)mask;
    (void)value;
    return E_OK;
}

/** @brief Report that a telemetry snapshot is unavailable and leave the
 * destination untouched.
 */
BOOL buddy1_snapshot (telemetry_codec_snapshot_t * p_state)
{
    (void)p_state;
    return 0;
}

/** @brief Route example MQTT control packets through the bridge and verify
 * response priority and delivery through the fake owner.
 */
int main (void)
{
    T_MQTT_UTK_APP_MSG message;
    uint8_t            packet[REMOTE_COMMAND_SIZE] = {3, REMOTE_STOP};
    mqtt_utk_app_receive("car/cmd/control", packet, sizeof packet);
    assert(1U == g_received &&
           0U == g_ordinary); /* Bypass the full ordinary queue. */
    mqtt_utk_app_receive("car/cmd/snapshot", packet, 1);
    assert(1U == g_ordinary);
    mqtt_utk_app_receive("car/telemetry/control", packet, sizeof packet);
    assert(1U == g_received);
    mqtt_utk_app_receive("car/cmd/control-extra", packet, sizeof packet);
    assert(1U == g_received && 2U == g_ordinary);
    assert(mqtt_utk_app_link(1) && !gb_is_linked_up);
    mqtt_utk_app_connection(1);
    assert(gb_is_linked_up && 0x11223344U == g_linked_epoch);
    mqtt_utk_app_connection(0);
    assert(!gb_is_linked_up);
    g_reply_ready = g_status_ready = 1;
    assert(mqtt_utk_app_take(&message) &&
           !strcmp(message.topic, "car/reply/control") &&
           REMOTE_REPLY_SIZE == message.payload_len && 1 == (int)message.qos &&
           !message.retain && !message.routine);
    assert(mqtt_utk_app_take(&message) &&
           !strcmp(message.topic, "car/telemetry/control") &&
           14 == (int)message.payload_len && 0 == (int)message.qos &&
           !message.retain && message.routine);
    assert(!mqtt_utk_app_take(&message));
    /* Status continuously due on a slow link cannot starve ordinary data. */
    g_status_ready = 1;
    g_outbox_ready = 2;
    assert(mqtt_utk_app_take(&message) &&
           !strcmp(message.topic, "car/telemetry/motion"));
    assert(mqtt_utk_app_take(&message) &&
           !strcmp(message.topic, "car/telemetry/control"));
    g_status_ready = 1;
    assert(mqtt_utk_app_take(&message) &&
           !strcmp(message.topic, "car/telemetry/motion"));
    assert(mqtt_utk_app_take(&message) &&
           !strcmp(message.topic, "car/telemetry/control"));
    puts("PASS: production remote bridge FIFO bypass, reconnect hook, reply "
         "priority and routine non-retained status");
    return 0;
}

/* End of file. */

