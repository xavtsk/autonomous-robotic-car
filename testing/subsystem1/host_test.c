#include "subsystem1.h"
#include "mqtt_utk.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include <assert.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static unsigned failures;
static unsigned delayed_logs, discarded_logs;
static pthread_mutex_t cache_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t queue_lock = PTHREAD_MUTEX_INITIALIZER;
static struct { unsigned capacity, size, head, tail, count; UB data[4][17]; } buffers[2];
static unsigned created;

ID tk_cre_mbf(const T_CMBF *p)
{
    unsigned id = created++;
    assert(id < 2 && p->maxmsz <= 17);
    buffers[id].size = (unsigned)p->maxmsz;
    buffers[id].capacity = (unsigned)p->bufsz / (4U + ((buffers[id].size + 3U) & ~3U));
    return (ID)id + 1;
}
ID tk_cre_mtx(const T_CMTX *p) { assert(p->mtxatr == TA_INHERIT); return 3; }
ER tk_del_mbf(ID id) { assert(id == 1 || id == 2); return E_OK; }
ER tk_loc_mtx(ID id, TMO timeout)
{ (void)timeout; if (id != 3) return E_OBJ; return pthread_mutex_lock(&cache_lock) == 0 ? E_OK : E_SYS; }
ER tk_unl_mtx(ID id)
{ assert(id == 3); return pthread_mutex_unlock(&cache_lock) == 0 ? E_OK : E_SYS; }
ER tk_snd_mbf(ID id, const void *data, INT length, TMO timeout)
{
    unsigned b = (unsigned)(id - 1);
    assert(b < 2 && timeout == TMO_POL && (unsigned)length == buffers[b].size);
    (void)pthread_mutex_lock(&queue_lock);
    if (buffers[b].count == buffers[b].capacity) {
        (void)pthread_mutex_unlock(&queue_lock); return E_TMOUT;
    }
    memcpy(buffers[b].data[buffers[b].head], data, (size_t)length);
    buffers[b].head = (buffers[b].head + 1) % buffers[b].capacity;
    ++buffers[b].count;
    (void)pthread_mutex_unlock(&queue_lock); return E_OK;
}
INT tk_rcv_mbf(ID id, void *data, TMO timeout)
{
    unsigned b = (unsigned)(id - 1);
    assert(b < 2 && timeout == TMO_POL);
    (void)pthread_mutex_lock(&queue_lock);
    if (!buffers[b].count) { (void)pthread_mutex_unlock(&queue_lock); return E_TMOUT; }
    memcpy(data, buffers[b].data[buffers[b].tail], buffers[b].size);
    buffers[b].tail = (buffers[b].tail + 1) % buffers[b].capacity;
    --buffers[b].count;
    (void)pthread_mutex_unlock(&queue_lock); return (INT)buffers[b].size;
}
ID app_start_task(void (*entry)(INT, void *), PRI priority, UW core, const char *name)
{ (void)entry; (void)priority; (void)core; (void)name; return 4; }
int tm_printf(const UB *format, ...)
{
    char text[512]; va_list args; int len;
    va_start(args, format); len = vsnprintf(text, sizeof(text), (const char *)format, args); va_end(args);
    if (strstr(text, " FAIL ")) ++failures;
    if (strstr(text, "application publish delayed")) ++delayed_logs;
    if (strstr(text, "application packet discarded")) ++discarded_logs;
    fputs(text, stdout); return len;
}

static mqtt_client_t fake_client;
static host_connection_cb connected;
static host_publish_cb incoming;
static host_data_cb fragment;
static void (*heartbeat)(void *);
static unsigned publish_count;
static char published_topic[64];
static UB published[MQTT_UTK_DEBUG_PAYLOAD_MAX];
static u16_t published_len;
static u8_t published_qos, published_retain;
static unsigned publish_attempts;
static u32_t fake_now;
static err_t publish_result;
static int defer_completion;
static host_done_cb deferred_done;
static void *deferred_arg;
static T_MQTT_UTK_APP_MSG attempted;

u32_t sys_now(void) { return fake_now; }

mqtt_client_t *mqtt_client_new(void) { return &fake_client; }
int mqtt_client_is_connected(mqtt_client_t *c) { return c && c->connected; }
err_t mqtt_client_connect(mqtt_client_t *c, const void *addr, unsigned short port,
                          host_connection_cb cb, void *arg, const struct mqtt_connect_client_info_t *info)
{ (void)c; (void)addr; (void)port; (void)arg; (void)info; connected = cb; return ERR_OK; }
void mqtt_set_inpub_callback(mqtt_client_t *c, host_publish_cb p, host_data_cb d, void *arg)
{ (void)c; (void)arg; incoming = p; fragment = d; }
err_t mqtt_subscribe(mqtt_client_t *c, const char *topic, u8_t qos, host_done_cb cb, void *arg)
{ (void)c; (void)topic; (void)qos; cb(arg, ERR_OK); return ERR_OK; }
err_t mqtt_publish(mqtt_client_t *c, const char *topic, const void *payload, u16_t len,
                   u8_t qos, u8_t retain, host_done_cb cb, void *arg)
{
    assert(c->connected && len <= sizeof(published));
    ++publish_attempts;
    strcpy(attempted.topic, topic);
    memcpy(attempted.payload, payload, len < sizeof attempted.payload ? len : sizeof attempted.payload);
    attempted.payload_len = len; attempted.qos = qos; attempted.retain = retain;
    if (publish_result != ERR_OK) return publish_result;
    strcpy(published_topic, topic); memcpy(published, payload, len);
    published_len = len; published_qos = qos; published_retain = retain;
    ++publish_count;
    if (defer_completion) {
        assert(!deferred_done); deferred_done = cb; deferred_arg = arg;
    } else cb(arg, ERR_OK);
    return ERR_OK;
}
int ip4addr_aton(const char *text, ip_addr_t *addr) { (void)text; *addr = 0; return 1; }
void sys_timeout(unsigned ms, void (*callback)(void *), void *arg)
{ (void)arg; if (ms == 5000) heartbeat = callback; }
void sys_untimeout(void (*callback)(void *), void *arg) { (void)callback; (void)arg; }

static void empty_outbox(void) { BUDDY1_TELEMETRY_MSG msg; while (buddy1_outbox_take(&msg)) { } }

static void poll_after(u32_t ms) { fake_now += ms; mqtt_utk_poll(); }
static unsigned outbox_count(void) { return buffers[mbf_buddy1_outbox - 1].count; }
static void complete_publish(err_t result)
{
    host_done_cb cb = deferred_done;
    void *arg = deferred_arg;
    assert(cb); deferred_done = NULL; deferred_arg = NULL; cb(arg, result);
}

/* Diagnostic text hook: a strong override of mqtt_utk_debug_take(). */
static const char *debug_queue[4];
static unsigned debug_head, debug_count, debug_asks;
unsigned int mqtt_utk_debug_take(char *line, unsigned int size)
{
    unsigned n;
    ++debug_asks;
    if (debug_head >= debug_count) return 0;
    n = (unsigned)strlen(debug_queue[debug_head]);
    assert(n < size);
    memcpy(line, debug_queue[debug_head++], n);
    return n;
}
static void test_debug_lines(void)
{
    static char longest[MQTT_UTK_DEBUG_PAYLOAD_MAX];
    unsigned count;
    memset(longest, 'x', sizeof longest - 1U);
    empty_outbox();
    poll_after(1000);
    debug_queue[0] = "[BCAP] run summary: episodes=1";
    debug_queue[1] = longest;
    debug_head = 0; debug_count = 2;
    /* Telemetry first: a queued report goes out before any debug line. */
    assert(buddy1_report_heading(12) == E_OK);
    count = publish_count;
    poll_after(20);
    assert(publish_count == count + 1 && strcmp(published_topic, "car/debug/barcode") != 0);
    /* Then one debug line per paced slot, on the debug topic, QoS 0. */
    poll_after(20);
    assert(publish_count == count + 2 && strcmp(published_topic, MQTT_UTK_DEBUG_TOPIC) == 0 &&
           published_qos == 0 && published_retain == 0 &&
           published_len == strlen(debug_queue[0]) &&
           memcmp(published, debug_queue[0], published_len) == 0);
    /* A transient rejection keeps the same line and retries it later. */
    publish_result = ERR_MEM;
    poll_after(20);
    assert(publish_count == count + 2 && debug_head == 2);
    publish_result = ERR_OK;
    poll_after(100);
    assert(publish_count == count + 3 && published_len == sizeof longest - 1U);
    /* Nothing left: no more publishes. */
    poll_after(100); poll_after(100);
    assert(publish_count == count + 3);
    debug_count = 0; debug_head = 0;
    puts("HOST PASS: diagnostic text lines only after telemetry, own topic, paced, retried, bounded");
}

#if APP_IR_DEBUG
static unsigned ir_frames;
static unsigned control_frames;
unsigned int mqtt_utk_control_debug_take(unsigned char *bytes, unsigned int size)
{
    if (!control_frames) return 0U;
    assert(size>=70U); memset(bytes,0,70U);bytes[0]=1U;bytes[69]=0x3fU;
    --control_frames;return 70U;
}
unsigned int mqtt_utk_ir_debug_take(unsigned char *bytes, unsigned int size)
{
    if (!ir_frames) return 0U;
    assert(size>=16U);
    memset(bytes,0,16U);bytes[0]=1U;bytes[15]=0x3fU;
    ir_frames--;
    return 16U;
}
static void test_ir_preview(void)
{
    unsigned count;
    empty_outbox();
    debug_head=0;debug_count=1;debug_queue[0]="[BCAP] end";
    ir_frames=1;
    assert(buddy1_report_heading(40)==E_OK);
    count=publish_count;
    poll_after(20);
    assert(publish_count==count+1&&ir_frames==1&&strcmp(published_topic,MQTT_UTK_IR_DEBUG_TOPIC)!=0);
    poll_after(20);
    assert(publish_count==count+2&&ir_frames==1&&strcmp(published_topic,MQTT_UTK_DEBUG_TOPIC)==0);
    poll_after(20);
    assert(publish_count==count+3&&ir_frames==0&&strcmp(published_topic,MQTT_UTK_IR_DEBUG_TOPIC)==0);
    assert(published_len==16&&published[0]==1&&published[1]==0&&published_qos==0&&published_retain==0);
    ir_frames=1;publish_result=ERR_MEM;poll_after(20);
    assert(publish_count==count+3&&ir_frames==0);
    publish_result=ERR_OK;poll_after(100);
    assert(publish_count==count+3); /* rejected preview is not retried */
    ir_frames=1;defer_completion=1;poll_after(20);
    assert(publish_count==count+4);
    assert(buddy1_report_heading(41)==E_OK);
    poll_after(20);assert(publish_count==count+4);
    defer_completion=0;complete_publish(ERR_OK);poll_after(20);
    assert(publish_count==count+5&&strcmp(published_topic,"car/telemetry/heading")==0);
    debug_head=debug_count=0;
    puts("HOST PASS: IR preview below telemetry/capture priority, binary length, QoS0, pacing, drop-on-backpressure and one in-flight slot");
}
static void test_control_preview(void)
{
    empty_outbox();debug_head=0;debug_count=1;debug_queue[0]="[BCAP] end";
    control_frames=ir_frames=1;
    assert(buddy1_report_heading(42)==E_OK);
    poll_after(20);assert(!strcmp(published_topic,"car/telemetry/heading")&&control_frames==1);
    poll_after(20);assert(!strcmp(published_topic,MQTT_UTK_DEBUG_TOPIC)&&control_frames==1);
    poll_after(20);assert(!strcmp(published_topic,MQTT_UTK_CONTROL_DEBUG_TOPIC)&&control_frames==0&&ir_frames==1);
    assert(published_len==70&&published_qos==0&&published_retain==0);
    poll_after(20);assert(!strcmp(published_topic,MQTT_UTK_IR_DEBUG_TOPIC)&&ir_frames==0);
    unsigned count=publish_count;
    control_frames=1;publish_result=ERR_MEM;poll_after(20);assert(control_frames==0&&publish_count==count);
    publish_result=ERR_OK;poll_after(100);assert(publish_count==count);
    control_frames=1;defer_completion=1;poll_after(20);
    assert(publish_count==count+1&&published_len==70);
    ir_frames=1;poll_after(20);assert(publish_count==count+1&&ir_frames==1);
    defer_completion=0;complete_publish(ERR_OK);poll_after(20);assert(ir_frames==0&&publish_count==count+2);
    debug_head=debug_count=0;
    puts("HOST PASS: control diagnostics below telemetry/BCAP, before IR, exact 70 bytes, drop-on-reject and stable in-flight buffer");
}
#endif
static void test_publish_backpressure(void)
{
    BUDDY1_COMMAND_MSG command = {.kind = BUDDY1_COMMAND_SNAPSHOT,
                                  .payload = {BUDDY1_WIRE_VERSION}};
    BUDDY1_SNAPSHOT decoded;
    UB expected[16];
    unsigned count, attempts, logs, i;

    /* ERR_MEM keeps the exact QoS 1 reply, rather than popping and losing it.
     * Retry rate/logging are bounded and the following FIFO item stays queued. */
    empty_outbox();
    assert(buddy1_snapshot_packet(expected));
    assert(buddy1_command_process(&command) == E_OK);
    assert(buddy1_report_heading(47) == E_OK);
    publish_result = ERR_MEM; count = publish_count; logs = delayed_logs;
    poll_after(20); attempts = publish_attempts;
    assert(outbox_count() == 1 && publish_count == count && delayed_logs == logs + 1);
    assert(strcmp(attempted.topic, "car/reply/snapshot") == 0 && attempted.qos == 1 &&
           attempted.retain == 0 && attempted.payload_len == sizeof(expected) &&
           memcmp(attempted.payload, expected, sizeof(expected)) == 0);
    for (i = 0; i < 1000; ++i) mqtt_utk_poll();
    poll_after(99); assert(publish_attempts == attempts);
    poll_after(1); assert(publish_attempts == attempts + 1 && delayed_logs == logs + 1);
    publish_result = ERR_OK; poll_after(100);
    assert(publish_count == count + 1 && outbox_count() == 1 && published_qos == 1 &&
           published_retain == 0 && memcmp(published, expected, sizeof(expected)) == 0);
    mqtt_utk_poll(); poll_after(19); assert(publish_count == count + 1);
    poll_after(1); assert(publish_count == count + 2 && outbox_count() == 0 &&
                         published_qos == 0 &&
                         buddy1_decode(BUDDY1_TOPIC_HEADING, published, published_len, &decoded) &&
                         decoded.heading_deg == 47);

    /* A rejected packet survives a disconnected period and is first on reconnect. */
    assert(buddy1_report_motion(156, -74) == E_OK);
    assert(buddy1_report_heading(88) == E_OK);
    publish_result = ERR_CONN; poll_after(20); attempts = publish_attempts;
    fake_client.connected = 0;
    for (i = 0; i < 20; ++i) poll_after(100);
    assert(publish_attempts == attempts && outbox_count() == 1);
    publish_result = ERR_OK; fake_client.connected = 1;
    connected(&fake_client, NULL, MQTT_CONNECT_ACCEPTED); mqtt_utk_poll();
    assert(buddy1_decode(BUDDY1_TOPIC_MOTION, published, published_len, &decoded) &&
           decoded.left_mms == 156 && decoded.right_mms == -74 && outbox_count() == 1);
    poll_after(20); assert(outbox_count() == 0);

    /* Accepted != completed. Only one app request consumes a lwIP slot until
     * completion, even after thousands of network polls. A callback timeout
     * retains this packet and retries before the next FIFO item. */
    assert(buddy1_report_motion(56, 58) == E_OK);
    assert(buddy1_report_heading(89) == E_OK);
    defer_completion = 1; poll_after(20); attempts = publish_attempts;
    for (i = 0; i < 1000; ++i) poll_after(20);
    assert(publish_attempts == attempts && outbox_count() == 1);
    complete_publish(ERR_TIMEOUT); poll_after(99);
    assert(publish_attempts == attempts); poll_after(1);
    assert(publish_attempts == attempts + 1 && outbox_count() == 1 &&
           buddy1_decode(BUDDY1_TOPIC_MOTION, published, published_len, &decoded) &&
           decoded.left_mms == 56 && decoded.right_mms == 58);
    complete_publish(ERR_OK); defer_completion = 0; poll_after(20);
    assert(outbox_count() == 0 &&
           buddy1_decode(BUDDY1_TOPIC_HEADING, published, published_len, &decoded) &&
           decoded.heading_deg == 89);

    /* Real lwIP drops in-flight callback registrations on disconnect. Do not
     * get stuck waiting forever or lose that packet. Duplicate delivery after
     * an ambiguous disconnect is possible and intentionally not "exactly once". */
    assert(buddy1_report_motion(60, -62) == E_OK);
    assert(buddy1_report_heading(90) == E_OK);
    defer_completion = 1; poll_after(20); attempts = publish_attempts;
    fake_client.connected = 0; deferred_done = NULL; deferred_arg = NULL;
    connected(&fake_client, NULL, 256); poll_after(100);
    assert(publish_attempts == attempts && outbox_count() == 1);
    fake_client.connected = 1; defer_completion = 0;
    connected(&fake_client, NULL, MQTT_CONNECT_ACCEPTED); mqtt_utk_poll();
    assert(publish_attempts == attempts + 1 && outbox_count() == 1 &&
           buddy1_decode(BUDDY1_TOPIC_MOTION, published, published_len, &decoded) &&
           decoded.left_mms == 60 && decoded.right_mms == -62);
    poll_after(20); assert(outbox_count() == 0);

    /* A permanent bad argument is explicitly discarded once, not retried in
     * a tight loop; the next packet can progress on the next paced poll. */
    assert(buddy1_report_motion(12, 14) == E_OK);
    assert(buddy1_report_heading(91) == E_OK);
    publish_result = ERR_ARG; logs = discarded_logs; poll_after(20);
    attempts = publish_attempts;
    for (i = 0; i < 1000; ++i) mqtt_utk_poll();
    assert(publish_attempts == attempts && discarded_logs == logs + 1 && outbox_count() == 1);
    publish_result = ERR_OK; poll_after(20);
    assert(outbox_count() == 0 &&
           buddy1_decode(BUDDY1_TOPIC_HEADING, published, published_len, &decoded) &&
           decoded.heading_deg == 91);

    /* Millisecond-clock rollover does not cancel the retry delay. */
    assert(buddy1_report_motion(16, 18) == E_OK);
    fake_now = UINT32_MAX - 5U; publish_result = ERR_MEM;
    mqtt_utk_poll(); attempts = publish_attempts;
    poll_after(99); assert(publish_attempts == attempts);
    publish_result = ERR_OK; poll_after(1);
    assert(publish_attempts == attempts + 1 &&
           buddy1_decode(BUDDY1_TOPIC_MOTION, published, published_len, &decoded) &&
           decoded.left_mms == 16 && decoded.right_mms == 18);
    puts("HOST PASS: MQTT retained FIFO retry, exact binary/QoS, bounded pacing/logging, deferred completion, disconnect recovery, permanent error, clock rollover");
}

/* Route pose: live-only, exact 4-byte layout, strict ranges, never touches the
 * snapshot cache, published on its own topic at QoS 0. */
static void test_pose(void)
{
    BUDDY1_SNAPSHOT v = {0}, d, before, after;
    UB p[16];
    v.pose_distance_cm = 4095; v.pose_heading_deg = 359;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &v, p, sizeof p) == 4U);
    assert(p[0] == 2U && p[1] == 0xffU && p[2] == 0x7fU && p[3] == 0x16U);
    assert(buddy1_decode(BUDDY1_TOPIC_POSE, p, 4U, &d) && d.pose_valid &&
           d.pose_distance_cm == 4095U && d.pose_heading_deg == 359);
    v.pose_distance_cm = 0; v.pose_heading_deg = 0;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &v, p, sizeof p) == 4U &&
           p[1] == 0U && p[2] == 0U && p[3] == 0U);
    v.pose_distance_cm = 4096; assert(buddy1_encode(BUDDY1_TOPIC_POSE, &v, p, sizeof p) == 0U);
    v.pose_distance_cm = 1; v.pose_heading_deg = 360;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &v, p, sizeof p) == 0U);
    v.pose_heading_deg = -1; assert(buddy1_encode(BUDDY1_TOPIC_POSE, &v, p, sizeof p) == 0U);
    p[0] = 2; p[1] = 0; p[2] = 0x80; p[3] = 0x16;              /* heading 360 */
    assert(!buddy1_decode(BUDDY1_TOPIC_POSE, p, 4U, &d));
    p[2] = 0; p[3] = 0x20;                                     /* padding bit */
    assert(!buddy1_decode(BUDDY1_TOPIC_POSE, p, 4U, &d));
    p[3] = 0; assert(!buddy1_decode(BUDDY1_TOPIC_POSE, p, 3U, &d)); /* length */
    assert(buddy1_payload_size(BUDDY1_TOPIC_POSE) == 4U &&
           buddy1_payload_bits(BUDDY1_TOPIC_POSE) == 21U);
    assert(buddy1_invalidate(BUDDY1_TOPIC_POSE) == E_PAR); /* no snapshot section */
    empty_outbox();
    assert(buddy1_snapshot(&before));
    assert(buddy1_report_pose(4096, 0) == E_PAR && buddy1_report_pose(0, 360) == E_PAR);
    assert(buddy1_report_pose(1234, 270) == E_OK);
    assert(buddy1_snapshot(&after) && memcmp(&before, &after, sizeof before) == 0);
    poll_after(20);
    assert(strcmp(published_topic, "car/telemetry/pose") == 0 && published_len == 4U &&
           buddy1_decode(BUDDY1_TOPIC_POSE, published, published_len, &d) &&
           d.pose_distance_cm == 1234U && d.pose_heading_deg == 270);
    puts("HOST PASS: route pose 21 bits/4 bytes, 4095/359 limits, padding/length rejection, live-only (snapshot unchanged), own topic");
}
static void *producer(void *which)
{
    unsigned i, n = (unsigned)(uintptr_t)which;
    for (i = 0; i < 1000; ++i) {
        if (n == 0) (void)buddy1_report_heading((H)(i % 360));
        else if (n == 1) (void)buddy1_report_obstacle((UH)i, (UH)(i + 1), BUDDY1_ROUTE_LEFT);
        else (void)buddy1_update_faults((UB)(1U << (n - 2)), (UB)(1U << (n - 2)));
    }
    return NULL;
}

static void emit_vectors(void)
{
    BUDDY1_SNAPSHOT v = {0};
    UB packet[16]; unsigned topic, i, len;
    v.motion_valid = v.barcode_valid = v.hump_valid = v.heading_valid = 1;
    v.obstacle_valid = v.robot_state_valid = v.faults_valid = 1;
    v.left_mms = 120; v.right_mms = -118;
    v.barcode_letter = 'W'; v.barcode_action = BUDDY1_ACTION_LEFT;
    v.hump_count = 2; v.hump_height_mm = 45; v.heading_deg = 359;
    v.obstacle_distance_mm = 180; v.obstacle_width_mm = 140; v.obstacle_route = BUDDY1_ROUTE_LEFT;
    v.robot_state = BUDDY1_STATE_LINE_FOLLOWING; v.faults = 0x31;
    v.pose_distance_cm = 2049; v.pose_heading_deg = 90;
    for (topic = 0; topic < BUDDY1_TOPIC_COUNT; ++topic) {
        if (topic == BUDDY1_TOPIC_MAP_REPLY) continue;
        len = buddy1_encode((UB)topic, &v, packet, sizeof(packet)); assert(len);
        printf("VECTOR %s ", buddy1_topic_name((UB)topic));
        for (i = 0; i < len; ++i) printf("%02x", packet[i]);
        puts("");
    }
}

int main(void)
{
    BUDDY1_SNAPSHOT v = {0}, decoded;
    BUDDY1_COMMAND_MSG command;
    UB packet[16], map_packet[4];
    BUDDY1_BARCODE_MAP map = {'A', 'B', 'C', 'D'};
    pthread_t workers[8]; unsigned i, before;
    int speed;
    assert(subsystem1_init() == E_OK);
    assert(buddy1_configuration_snapshot_and_close(NULL) == E_PAR);
    assert(buddy1_configuration_allowed());
    assert(buddy1_barcode_action('D') == BUDDY1_ACTION_UTURN);
    subsystem1_test();
    assert(failures == 0);
    {
        BUDDY1_BARCODE_MAP frozen;
        assert(buddy1_configuration_snapshot_and_close(&frozen) == E_OK);
        assert(!buddy1_configuration_allowed());
        assert(buddy1_barcode_action(frozen.forward) == BUDDY1_ACTION_FORWARD);
        assert(buddy1_barcode_action(frozen.left) == BUDDY1_ACTION_LEFT);
        assert(buddy1_barcode_action(frozen.right) == BUDDY1_ACTION_RIGHT);
        assert(buddy1_barcode_action(frozen.uturn) == BUDDY1_ACTION_UTURN);
        assert(buddy1_apply_barcode_map(&map) == E_OBJ);
    }
    /* Independent expected rounding across the complete speed range. */
    for (speed = -510; speed <= 510; ++speed) {
        int expected = speed >= 0 ? ((speed + 1) / 2) * 2 : -(((-speed + 1) / 2) * 2);
        v.left_mms = (int16_t)speed; v.right_mms = (int16_t)-speed;
        assert(buddy1_encode(BUDDY1_TOPIC_MOTION, &v, packet, sizeof(packet)) == 4);
        assert(buddy1_decode(BUDDY1_TOPIC_MOTION, packet, 4, &decoded));
        assert(decoded.left_mms == expected && decoded.right_mms == -expected);
    }
    for (i = 0; i < 360; ++i) {
        v.heading_deg = (H)i;
        assert(buddy1_encode(BUDDY1_TOPIC_HEADING, &v, packet, sizeof(packet)) == 3);
        assert(buddy1_decode(BUDDY1_TOPIC_HEADING, packet, 3, &decoded) && decoded.heading_deg == (H)i);
    }
    for (i = BUDDY1_STATE_STOPPED; i <= BUDDY1_STATE_ERROR; ++i) {
        v.robot_state = (BUDDY1_ROBOT_STATE)i;
        assert(buddy1_encode(BUDDY1_TOPIC_STATE, &v, packet, sizeof(packet)) == 2);
        assert(buddy1_decode(BUDDY1_TOPIC_STATE, packet, 2, &decoded) && decoded.robot_state == v.robot_state);
    }
    for (i = 0; i < 256; ++i) {
        packet[0] = BUDDY1_WIRE_VERSION; packet[1] = (UB)i;
        assert(buddy1_payload_valid(BUDDY1_TOPIC_BARCODE, packet, 2) ==
               ((i & 31) <= 25 && (i >> 5) <= BUDDY1_ACTION_UTURN));
    }
    /* All fault combinations cross all three layouts, including IR in the
     * former snapshot padding bit and heartbeat's most significant bit. */
    v.faults_valid = 1;
    for (i = 0; i <= BUDDY1_FAULT_MASK; ++i) {
        const UB topics[] = {BUDDY1_TOPIC_FAULTS, BUDDY1_TOPIC_SNAPSHOT, BUDDY1_TOPIC_STATUS};
        unsigned n;
        v.faults = (UB)i;
        for (n = 0; n < sizeof(topics); ++n) {
            unsigned len = buddy1_encode(topics[n], &v, packet, sizeof(packet));
            assert(len && buddy1_decode(topics[n], packet, len, &decoded));
            assert(decoded.faults_valid && decoded.faults == i);
            packet[0] = 1;
            assert(!buddy1_decode(topics[n], packet, len, &decoded));
        }
    }
    for (i = 0; i < 8; ++i) assert(pthread_create(&workers[i], NULL, producer, (void *)(uintptr_t)i) == 0);
    for (i = 0; i < 8; ++i) assert(pthread_join(workers[i], NULL) == 0);
    assert(buddy1_snapshot(&decoded) && decoded.heading_deg == 279 &&
           decoded.obstacle_distance_mm == 999 && decoded.obstacle_width_mm == 1000 &&
           decoded.faults == BUDDY1_FAULT_MASK);
    empty_outbox();

    mqtt_utk_start(); fake_client.connected = 1; connected(&fake_client, NULL, MQTT_CONNECT_ACCEPTED);
    assert(buddy1_map_pack(&map, map_packet));
    /* Force zero bytes and fragmented delivery through the ACTUAL mqtt_utk.c. */
    incoming(NULL, "car/cmd/barcode_map", 4);
    fragment(NULL, map_packet, 2, 0); fragment(NULL, map_packet + 2, 2, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(mbf_buddy1_commands, &command, TMO_POL) == (INT)sizeof(command));
    assert(memcmp(command.payload, map_packet, 4) == 0);
    incoming(NULL, "car/cmd/snapshot", 1); packet[0] = BUDDY1_WIRE_VERSION;
    fragment(NULL, packet, 1, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(mbf_buddy1_commands, &command, TMO_POL) == (INT)sizeof(command));
    assert(buddy1_command_process(&command) == E_OK);
    before = publish_count; mqtt_utk_poll(); assert(publish_count == before + 1);
    assert(published_len == 16 && strcmp(published_topic, "car/reply/snapshot") == 0 &&
           buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, published, published_len, &decoded));
    incoming(NULL, "car/cmd/snapshot", 2); fragment(NULL, packet, 1, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(mbf_buddy1_commands, &command, TMO_POL) == E_TMOUT);
    incoming(NULL, "car/cmd/barcode_map", 17); fragment(NULL, map_packet, 4, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(mbf_buddy1_commands, &command, TMO_POL) == E_TMOUT);
    incoming(NULL, "car/cmd/snapshot/this/topic/is/intentionally/far/too/long/to/be/accepted", 1);
    fragment(NULL, packet, 1, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(mbf_buddy1_commands, &command, TMO_POL) == E_TMOUT);
    assert(mqtt_utk_app_link(0));
    assert(buddy1_snapshot(&decoded) && (decoded.faults & BUDDY1_FAULT_WIFI));
    assert(mqtt_utk_app_link(1));
    assert(buddy1_snapshot(&decoded) && decoded.faults == (BUDDY1_FAULT_MASK & ~BUDDY1_FAULT_WIFI));
    empty_outbox();
    assert(buddy1_report_motion(0, 0) == E_OK);
    poll_after(20); assert(published_len == 4 && published[1] == 0 && published[2] == 0 && published[3] == 0);
    heartbeat(NULL); assert(published_len == 2 &&
        buddy1_decode(BUDDY1_TOPIC_STATUS, published, published_len, &decoded));
    test_publish_backpressure();
    test_debug_lines();
    test_pose();
#if APP_IR_DEBUG
    test_ir_preview();
    test_control_preview();
#endif
    emit_vectors();
    puts("HOST PASS: binary FIFO, full speed range, 64 fault masks, eight concurrent producers, real MQTT callbacks and bridge");
    return 0;
}
