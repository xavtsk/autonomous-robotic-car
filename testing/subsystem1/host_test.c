#include "buddy1.h"
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
static struct { unsigned capacity, size, head, tail, count; UB data[4][20]; } buffers[2];
static unsigned created;

ID tk_cre_mbf(const T_CMBF *p)
{
    unsigned id = created++;
    assert(id < 2 && p->maxmsz <= 20);
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
/* Fake sleep for the command task's reply retry: counts calls and, when
 * sleep_drains is set, lets the "network" take one queued message. */
static unsigned sleeps, sleep_drains;
ER tk_dly_tsk(TMO time)
{
    unsigned b = 0;
    assert(time == BUDDY1_REPLY_RETRY_MS);
    ++sleeps;
    if (sleep_drains) {
        (void)pthread_mutex_lock(&queue_lock);
        if (buffers[b].count) { buffers[b].tail = (buffers[b].tail + 1) % buffers[b].capacity; --buffers[b].count; }
        (void)pthread_mutex_unlock(&queue_lock);
    }
    return E_OK;
}
ER tk_ref_mbf(ID id, T_RMBF *ref)
{
    unsigned b = (unsigned)(id - 1);
    assert(b < 2 && ref);
    (void)pthread_mutex_lock(&queue_lock);
    ref->msgsz = buffers[b].count ? (INT)buffers[b].size : 0;
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
static UB attempted_bytes[MQTT_UTK_DEBUG_PAYLOAD_MAX];
static unsigned heartbeat_pending;

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
    memcpy(attempted_bytes, payload, len);
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
{
    (void)arg;
    if (ms == 5000) {
        assert(!heartbeat_pending); heartbeat_pending = 1; heartbeat = callback;
    }
}
void sys_untimeout(void (*callback)(void *), void *arg)
{ (void)arg; if (callback == heartbeat) heartbeat_pending = 0; }

static void empty_outbox(void) { buddy1_telemetry_msg_t msg; while (buddy1_outbox_take(&msg)) { } }

static void poll_after(u32_t ms) { fake_now += ms; mqtt_utk_poll(); }
static unsigned outbox_count(void) { return buffers[g_mbf_buddy1_outbox - 1].count; }
static void complete_publish(err_t result)
{
    host_done_cb cb = deferred_done;
    void *arg = deferred_arg;
    assert(cb); deferred_done = NULL; deferred_arg = NULL; cb(arg, result);
}

/* Binary diagnostic hook: payloads contain embedded zero bytes, not strings. */
static const unsigned char *debug_queue[16];
static unsigned debug_size[16];
static const unsigned char debug_summary[16]={1};
#if APP_IR_DEBUG
static const unsigned char debug_end[6]={1,0,0,0,0,3};
#endif
static unsigned debug_head, debug_count, debug_asks;
unsigned int mqtt_utk_debug_take(unsigned char *bytes, unsigned int size)
{
    unsigned n;
    ++debug_asks;
    if (debug_head >= debug_count) return 0;
    n = debug_size[debug_head];
    assert(n <= size);
    memcpy(bytes, debug_queue[debug_head++], n);
    return n;
}
static void test_debug_lines(void)
{
    static unsigned char longest[MQTT_UTK_DEBUG_PAYLOAD_MAX];
    unsigned count;
    memset(longest, 0xa5, sizeof longest); longest[10]=0;
    empty_outbox();
    poll_after(1000);
    debug_queue[0] = debug_summary; debug_size[0]=sizeof debug_summary;
    debug_queue[1] = longest; debug_size[1]=sizeof longest;
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
           published_len == debug_size[0] &&
           memcmp(published, debug_queue[0], published_len) == 0);
    /* A transient rejection keeps the same line and retries it later. */
    publish_result = ERR_MEM;
    poll_after(20);
    assert(publish_count == count + 2 && debug_head == 2);
    publish_result = ERR_OK;
    poll_after(100);
    assert(publish_count == count + 3 && published_len == 164U &&
           memcmp(published, longest, sizeof longest) == 0);
    /* Nothing left: no more publishes. */
    poll_after(100); poll_after(100);
    assert(publish_count == count + 3);
    debug_count = 0; debug_head = 0;
    puts("HOST PASS: exact 164-byte binary capture, event priority, zero bytes preserved, paced and retried");
}

static void queue_debug(unsigned count, const unsigned char *bytes, unsigned size)
{
    assert(count <= sizeof debug_queue / sizeof debug_queue[0]);
    debug_head = 0; debug_count = count;
    for (unsigned i = 0; i < count; ++i) {
        debug_queue[i] = bytes; debug_size[i] = size;
    }
}

static void test_capture_fairness(void)
{
    buddy1_snapshot_t decoded;
    buddy1_terrain_t terrain = {0};
    T_MQTT_UTK_APP_MSG message;
    unsigned count, reports = 0;
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
    queue_debug(1, debug_summary, sizeof debug_summary); poll_after(20);
    queue_debug(12, debug_summary, sizeof debug_summary);
    for (unsigned i = 1; i <= 4; ++i)
        assert(buddy1_report_motion((H)(2U*i), (H)(2U*i)) == E_OK);
    count = publish_count;
    for (unsigned i = 0; i < 24; ++i) {
        poll_after(20);
        assert(publish_count == count + i + 1);
        if ((i & 1U) == 0) {
            ++reports;
            assert(!strcmp(published_topic, "car/telemetry/motion") &&
                   buddy1_decode(BUDDY1_TOPIC_MOTION, published, published_len, &decoded) &&
                   decoded.left_mms == (H)(2U*reports));
            assert(buddy1_report_motion((H)(2U*(reports+4U)), 0) == E_OK);
        } else {
            assert(!strcmp(published_topic, MQTT_UTK_DEBUG_TOPIC));
        }
    }
    assert(debug_head == 12 && reports == 12);
    /* The capture turn held one fresh app packet; drain that slot before
     * dropping the rest of this synthetic workload. */
    poll_after(20);
    assert(!strcmp(published_topic, "car/telemetry/motion") &&
           buddy1_decode(BUDDY1_TOPIC_MOTION, published, published_len, &decoded) &&
           decoded.left_mms == 26);
    empty_outbox(); debug_head = debug_count = 0;
    puts("HOST PASS: saturated FIFO and BCAP each progress every other slot, app order intact, synchronous completion safe");
}

static void test_capture_event_priority(void)
{
    buddy1_command_msg_t snapshot = {.kind = BUDDY1_COMMAND_SNAPSHOT,
                                   .payload = {BUDDY1_WIRE_VERSION}};
    const char *topics[] = {"car/telemetry/faults", "car/telemetry/state",
                           "car/telemetry/barcode", "car/reply/snapshot",
                           "car/reply/barcode_map"};
    UB map_reply[] = {BUDDY1_WIRE_VERSION, 0};
    unsigned asks;
    empty_outbox();
    queue_debug(1, debug_summary, sizeof debug_summary);
    assert(buddy1_update_faults(BUDDY1_FAULT_WIFI, 0) == E_OK);
    assert(buddy1_report_robot_state(BUDDY1_STATE_STOPPED) == E_OK);
    assert(buddy1_report_barcode('A') == E_OK);
    assert(buddy1_command_process(&snapshot) == E_OK);
    asks = debug_asks;
    for (unsigned i = 0; i < 5; ++i) {
        poll_after(20);
        assert(!strcmp(published_topic, topics[i]) && debug_asks == asks);
        assert(published_qos == (i >= 3 ? 1 : 0));
        if (i == 0)
            assert(buddy1_queue_publish(BUDDY1_TOPIC_MAP_REPLY, map_reply, sizeof map_reply) == E_OK);
    }
    poll_after(20); assert(!strcmp(published_topic, MQTT_UTK_DEBUG_TOPIC));

    /* Priority applies at the head: never reorder an urgent event ahead of
     * an already staged routine sample, even if that sample yields a slot. */
    assert(buddy1_report_heading(22) == E_OK); poll_after(20);
    queue_debug(2, debug_summary, sizeof debug_summary);
    assert(buddy1_report_motion(44, 46) == E_OK);
    assert(buddy1_update_faults(BUDDY1_FAULT_WIFI, 0) == E_OK);
    poll_after(20); assert(!strcmp(published_topic, MQTT_UTK_DEBUG_TOPIC));
    poll_after(20); assert(!strcmp(published_topic, "car/telemetry/motion"));
    poll_after(20); assert(!strcmp(published_topic, "car/telemetry/faults"));
    poll_after(20); assert(!strcmp(published_topic, MQTT_UTK_DEBUG_TOPIC));
    debug_head = debug_count = 0;
    puts("HOST PASS: faults/state/barcode/replies ahead of fresh captures, QoS preserved, no app FIFO reordering");
}

static void test_capture_mixed_retries(void)
{
    static unsigned char full[MQTT_UTK_DEBUG_PAYLOAD_MAX];
    unsigned attempts, asks;
    memset(full, 0xa6, sizeof full); full[15] = 0;
    empty_outbox();
    queue_debug(2, full, sizeof full);
    assert(buddy1_report_motion(48, 50) == E_OK);
    publish_result = ERR_MEM; poll_after(20);
    assert(!strcmp(attempted.topic, "car/telemetry/motion") && debug_head == 0);
    assert(buddy1_report_heading(23) == E_OK);
    publish_result = ERR_OK; poll_after(100);
    assert(!strcmp(published_topic, "car/telemetry/motion") && debug_head == 0);
    poll_after(20); assert(!strcmp(published_topic, "car/telemetry/heading"));

    /* Capture rejection retains all 164 bytes; a newly arrived event cannot
     * replace it. Retry pacing and only one request apply across both sources. */
    assert(buddy1_report_motion(60, 62) == E_OK);
    publish_result = ERR_MEM; poll_after(20);
    assert(debug_head == 1 && attempted.payload_len == sizeof full &&
           !strcmp(attempted.topic, MQTT_UTK_DEBUG_TOPIC) &&
           !memcmp(attempted_bytes, full, sizeof full));
    assert(buddy1_report_heading(24) == E_OK);
    attempts = publish_attempts; asks = debug_asks;
    poll_after(99); assert(publish_attempts == attempts);
    publish_result = ERR_OK; defer_completion = 1; poll_after(1);
    assert(publish_attempts == attempts + 1 && outbox_count() == 1 &&
           debug_asks == asks && !memcmp(published, full, sizeof full));
    poll_after(200); assert(publish_attempts == attempts + 1);
    complete_publish(ERR_TIMEOUT); poll_after(99);
    assert(publish_attempts == attempts + 1);
    poll_after(1);
    assert(publish_attempts == attempts + 2 && debug_asks == asks &&
           outbox_count() == 1 && !memcmp(published, full, sizeof full));

    /* lwIP clears the callback on disconnect. Retry the same capture after
     * reconnect, keep the waiting event, and keep one heartbeat timer chain. */
    fake_client.connected = 0; deferred_done = NULL; deferred_arg = NULL;
    connected(&fake_client, NULL, 256); poll_after(100);
    assert(publish_attempts == attempts + 2 && heartbeat_pending == 1);
    fake_client.connected = 1; defer_completion = 0;
    connected(&fake_client, NULL, MQTT_CONNECT_ACCEPTED); mqtt_utk_poll();
    assert(publish_attempts == attempts + 3 && debug_asks == asks &&
           heartbeat_pending == 1 && outbox_count() == 1 &&
           !strcmp(published_topic, MQTT_UTK_DEBUG_TOPIC) &&
           !memcmp(published, full, sizeof full));
    /* The routine sample staged before the capture still precedes the event. */
    poll_after(20); assert(!strcmp(published_topic, "car/telemetry/motion"));
    poll_after(20); assert(!strcmp(published_topic, "car/telemetry/heading"));

    /* A permanent capture error frees its slot; fresh telemetry progresses. */
    publish_result = ERR_ARG; poll_after(20); assert(debug_head == 2);
    assert(buddy1_report_motion(52, 54) == E_OK);
    publish_result = ERR_OK; poll_after(20);
    assert(!strcmp(published_topic, "car/telemetry/motion"));
    debug_head = debug_count = 0;
    puts("HOST PASS: mixed app/capture exact retries, 164 bytes, one in-flight, reconnect, one heartbeat chain, permanent-error recovery");
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
    debug_head=0;debug_count=1;debug_queue[0]=debug_end;debug_size[0]=sizeof debug_end;
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
    empty_outbox();debug_head=0;debug_count=1;debug_queue[0]=debug_end;debug_size[0]=sizeof debug_end;
    control_frames=ir_frames=1;
    assert(buddy1_report_heading(42)==E_OK);
    poll_after(20);assert(!strcmp(published_topic,"car/telemetry/heading")&&control_frames==1);
    poll_after(200);assert(!strcmp(published_topic,MQTT_UTK_CONTROL_DEBUG_TOPIC)&&control_frames==0&&ir_frames==1);
    assert(published_len==70&&published_qos==0&&published_retain==0);
    poll_after(20);assert(!strcmp(published_topic,MQTT_UTK_DEBUG_TOPIC)&&published_len==sizeof debug_end&&
                         !memcmp(published,debug_end,sizeof debug_end));
    poll_after(20);assert(!strcmp(published_topic,MQTT_UTK_IR_DEBUG_TOPIC)&&ir_frames==0);
    unsigned count=publish_count;
    control_frames=1;publish_result=ERR_MEM;poll_after(200);assert(control_frames==0&&publish_count==count);
    publish_result=ERR_OK;poll_after(100);assert(publish_count==count);
    control_frames=1;defer_completion=1;poll_after(200);
    assert(publish_count==count+1&&published_len==70);
    ir_frames=1;poll_after(20);assert(publish_count==count+1&&ir_frames==1);
    defer_completion=0;complete_publish(ERR_OK);poll_after(20);assert(ir_frames==0&&publish_count==count+2);
    debug_head=debug_count=0;
    puts("HOST PASS: bounded control diagnostics after events, staged BCAP intact, exact 70 bytes, drop-on-reject and one in-flight buffer");
}
static void test_control_preview_fairness(unsigned completion_ms)
{
    unsigned apps=0, captures=0, previews=0, data_since=2;
    u32_t previous_preview=0;
    buddy1_snapshot_t decoded;
    empty_outbox(); control_frames=ir_frames=0;
    /* Establish a capture turn boundary and two completed data packets. */
    queue_debug(2,debug_summary,sizeof debug_summary);
    poll_after(200); poll_after(20);
    assert(!strcmp(published_topic,MQTT_UTK_DEBUG_TOPIC));
    queue_debug(1,debug_summary,sizeof debug_summary);
    for(unsigned i=1;i<=4;++i) assert(buddy1_report_motion((H)(2*i),0)==E_OK);
    control_frames=100;
    defer_completion=completion_ms!=0;
    for(unsigned i=0;i<60;++i) {
        unsigned before=publish_count;
        poll_after(20);
        assert(publish_count==before+1);
        if(!strcmp(published_topic,MQTT_UTK_CONTROL_DEBUG_TOPIC)) {
            assert(data_since>=2);
            if(previews) assert((u32_t)(fake_now-previous_preview)>=200U);
            previous_preview=fake_now; ++previews; data_since=0;
        } else if(!strcmp(published_topic,"car/telemetry/motion")) {
            ++apps; ++data_since;
            assert(buddy1_decode(BUDDY1_TOPIC_MOTION,published,published_len,&decoded));
            assert(decoded.left_mms==(H)(2*apps));
            assert(buddy1_report_motion((H)(2*(apps+4)),0)==E_OK);
        } else {
            assert(!strcmp(published_topic,MQTT_UTK_DEBUG_TOPIC));
            assert(published_len==sizeof debug_summary &&
                   !memcmp(published,debug_summary,sizeof debug_summary));
            ++captures; ++data_since;
            queue_debug(1,debug_summary,sizeof debug_summary);
        }
        if(completion_ms) {
            unsigned attempts=publish_attempts;
            poll_after(completion_ms);
            assert(publish_attempts==attempts); /* no second in-flight packet */
            complete_publish(ERR_OK);
        }
    }
    assert(previews>0 && apps>0 && captures>0);
    assert(apps==captures || apps==captures+1);
    if(completion_ms>=250) assert(previews==20 && apps==20 && captures==20);
    else assert(previews<=6 && apps>=27 && captures>=27);
    /* A fresh preview must not bypass a FIFO-head event, even when overdue. */
    control_frames=0; debug_head=debug_count=0; empty_outbox();
    defer_completion=0; poll_after(200); empty_outbox();
    assert(buddy1_report_heading(43)==E_OK); poll_after(20);
    assert(buddy1_report_heading(44)==E_OK); poll_after(20);
    control_frames=1;
    assert(buddy1_report_heading(45)==E_OK); poll_after(200);
    assert(!strcmp(published_topic,"car/telemetry/heading") && control_frames==1);
    poll_after(20); assert(!strcmp(published_topic,MQTT_UTK_CONTROL_DEBUG_TOPIC));
    control_frames=0;
    printf("HOST PASS: controller preview progresses with saturated routine/BCAP, >=2 data packets and <=5Hz; completion=%ums\n",completion_ms);
}
static void test_control_preview_waits_for_retry(void)
{
    unsigned char full[MQTT_UTK_DEBUG_PAYLOAD_MAX];
    unsigned attempts;
    memset(full,0xa7,sizeof full); full[5]=0;
    empty_outbox(); control_frames=ir_frames=0;
    queue_debug(1,full,sizeof full);
    publish_result=ERR_MEM; poll_after(200);
    assert(!strcmp(attempted.topic,MQTT_UTK_DEBUG_TOPIC));
    control_frames=1;
    assert(buddy1_report_heading(46)==E_OK);
    attempts=publish_attempts;
    poll_after(99); assert(publish_attempts==attempts && control_frames==1);
    publish_result=ERR_OK; defer_completion=1; poll_after(1);
    assert(publish_attempts==attempts+1 && !strcmp(published_topic,MQTT_UTK_DEBUG_TOPIC) &&
           published_len==sizeof full && !memcmp(published,full,sizeof full) && control_frames==1);
    poll_after(250); assert(publish_attempts==attempts+1 && control_frames==1);
    complete_publish(ERR_TIMEOUT); poll_after(100);
    assert(publish_attempts==attempts+2 && !strcmp(published_topic,MQTT_UTK_DEBUG_TOPIC) &&
           published_len==sizeof full && !memcmp(published,full,sizeof full) && control_frames==1);
    complete_publish(ERR_OK); defer_completion=0;
    poll_after(20); assert(!strcmp(published_topic,"car/telemetry/heading") && control_frames==1);
    poll_after(20); assert(!strcmp(published_topic,MQTT_UTK_CONTROL_DEBUG_TOPIC) && control_frames==0);
    debug_head=debug_count=0;
    puts("HOST PASS: overdue controller preview waits for exact BCAP retries, in-flight completion and queued event");
}
#endif

#if APP_IMU_DEBUG
static unsigned imu_batches;

unsigned int mqtt_utk_imu_debug_take(unsigned char *bytes, unsigned int size)
{
    if (imu_batches == 0U) return 0U;
    assert(size == MQTT_UTK_IMU_DEBUG_MAX);
    memset(bytes, 0, size);
    bytes[0] = 1U;
    bytes[1] = 8U;
    bytes[size - 1U] = 0x5aU;
    --imu_batches;
    return size;
}

static void test_imu_stream(void)
{
    unsigned apps = 0, captures = 0, batches = 0, data_since = 0;
    empty_outbox();
    debug_head = debug_count = 0;
    imu_batches = 1;
    assert(buddy1_report_heading(64) == E_OK);
    poll_after(200);
    assert(!strcmp(published_topic, "car/telemetry/heading") && imu_batches == 1);
#if APP_IR_DEBUG
    control_frames = 1;
    poll_after(200);
    assert(!strcmp(published_topic, MQTT_UTK_CONTROL_DEBUG_TOPIC) && imu_batches == 1);
#endif
    poll_after(20);
    assert(!strcmp(published_topic, MQTT_UTK_IMU_DEBUG_TOPIC) && imu_batches == 0);
    assert(published_len == MQTT_UTK_IMU_DEBUG_MAX && published_qos == 0 && published_retain == 0);
    assert(published[0] == 1U && published[1] == 8U && published[2] == 0U &&
           published[published_len - 1U] == 0x5aU);

    /* A busy link grants no more than one IMU slot per two data publishes,
     * preserving the app/capture alternation and progress for all sources. */
    imu_batches = 12;
    queue_debug(4, debug_summary, sizeof debug_summary);
    for (unsigned i = 0; i < 4; ++i) assert(buddy1_report_motion(40, 41) == E_OK);
    unsigned before = publish_count;
    for (unsigned i = 0; i < 12; ++i) {
        poll_after(20);
        assert(publish_count == before + i + 1U);
        if (!strcmp(published_topic, MQTT_UTK_IMU_DEBUG_TOPIC)) {
            assert(data_since >= 2U);
            ++batches;
            data_since = 0;
        } else if (!strcmp(published_topic, MQTT_UTK_DEBUG_TOPIC)) {
            ++captures;
            ++data_since;
        } else {
            assert(!strcmp(published_topic, "car/telemetry/motion"));
            assert(buddy1_report_motion(40, 41) == E_OK);
            ++apps;
            ++data_since;
        }
    }
    assert(apps == 4U && captures == 4U && batches == 4U);

    /* Drain any already staged routine packet then demonstrate that a
     * rejected IMU packet never delays a later FIFO-head event with a retry. */
    imu_batches = 0;
    empty_outbox();
    poll_after(20);
    imu_batches = 1;
    publish_result = ERR_MEM;
    poll_after(20);
    assert(!strcmp(attempted.topic, MQTT_UTK_IMU_DEBUG_TOPIC) && imu_batches == 0);
    publish_result = ERR_OK;
    assert(buddy1_report_heading(65) == E_OK);
    poll_after(20);
    assert(!strcmp(published_topic, "car/telemetry/heading"));
    debug_head = debug_count = 0;
    puts("HOST PASS: bounded raw IMU batches, legacy traffic priority and fairness, no diagnostic retry");
}
#endif

static void test_publish_backpressure(void)
{
    buddy1_command_msg_t command = {.kind = BUDDY1_COMMAND_SNAPSHOT,
                                  .payload = {BUDDY1_WIRE_VERSION}};
    buddy1_snapshot_t decoded;
    UB expected[BUDDY1_PAYLOAD_MAX];
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

/* Route pose: exact 4-byte layout, strict ranges, published on its own topic
 * at QoS 0; since 19 Nov 2026 also kept in the snapshot (bits 102-123). */
static void test_pose(void)
{
    buddy1_snapshot_t v = {0}, d, before, after;
    UB p[32];
    v.pose_distance_cm = 4095; v.pose_heading_deg = 359;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &v, p, sizeof p) == 4U);
    assert(p[0] == BUDDY1_WIRE_VERSION && p[1] == 0xffU && p[2] == 0x7fU && p[3] == 0x16U);
    assert(buddy1_decode(BUDDY1_TOPIC_POSE, p, 4U, &d) && d.pose_valid &&
           d.pose_distance_cm == 4095U && d.pose_heading_deg == 359);
    v.pose_distance_cm = 0; v.pose_heading_deg = 0;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &v, p, sizeof p) == 4U &&
           p[1] == 0U && p[2] == 0U && p[3] == 0U);
    v.pose_distance_cm = 4096; assert(buddy1_encode(BUDDY1_TOPIC_POSE, &v, p, sizeof p) == 0U);
    v.pose_distance_cm = 1; v.pose_heading_deg = 360;
    assert(buddy1_encode(BUDDY1_TOPIC_POSE, &v, p, sizeof p) == 0U);
    v.pose_heading_deg = -1; assert(buddy1_encode(BUDDY1_TOPIC_POSE, &v, p, sizeof p) == 0U);
    p[0] = BUDDY1_WIRE_VERSION; p[1] = 0; p[2] = 0x80; p[3] = 0x16; /* heading 360 */
    assert(!buddy1_decode(BUDDY1_TOPIC_POSE, p, 4U, &d));
    p[2] = 0; p[3] = 0x20;                                     /* padding bit */
    assert(!buddy1_decode(BUDDY1_TOPIC_POSE, p, 4U, &d));
    p[3] = 0; assert(!buddy1_decode(BUDDY1_TOPIC_POSE, p, 3U, &d)); /* length */
    assert(buddy1_payload_size(BUDDY1_TOPIC_POSE) == 4U &&
           buddy1_payload_bits(BUDDY1_TOPIC_POSE) == 21U);
    assert(buddy1_invalidate(BUDDY1_TOPIC_POSE) == E_PAR); /* not one of the seven */
    empty_outbox();
    assert(buddy1_snapshot(&before));
    assert(buddy1_report_pose(4096, 0) == E_PAR && buddy1_report_pose(0, 360) == E_PAR);
    assert(buddy1_snapshot(&after) && memcmp(&before, &after, sizeof before) == 0); /* rejected: unchanged */
    assert(buddy1_report_pose(1234, 270) == E_OK);
    assert(buddy1_snapshot(&after) && after.pose_valid && after.pose_distance_cm == 1234U &&
           after.pose_heading_deg == 270 && after.motion_valid == before.motion_valid &&
           after.faults == before.faults && after.hump_count == before.hump_count);
    /* Snapshot layout: the original 102 bits unchanged, pose valid at bit 102,
     * distance bits 103-114, heading bits 115-123, version + 16 body bytes. */
    {
        UB snap[BUDDY1_PAYLOAD_MAX];
        buddy1_snapshot_t z = {0};
        assert(buddy1_payload_bits(BUDDY1_TOPIC_SNAPSHOT) == 124U && buddy1_payload_size(BUDDY1_TOPIC_SNAPSHOT) == 17U);
        z.pose_valid = 1; z.pose_distance_cm = 4095; z.pose_heading_deg = 359;
        assert(buddy1_encode(BUDDY1_TOPIC_SNAPSHOT, &z, snap, sizeof snap) == 17U);
        assert(snap[1] == 0U && snap[13] == 0xC0U && snap[14] == 0xFFU && snap[15] == 0x3FU && snap[16] == 0x0BU);
        assert(buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, snap, 17U, &d) && d.pose_valid &&
               d.pose_distance_cm == 4095U && d.pose_heading_deg == 359 && !d.motion_valid);
        snap[16] = 0x1BU; assert(!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, snap, 17U, &d));        /* padding */
        snap[16] = 0x0FU; snap[15] |= 0xF8U; assert(!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, snap, 17U, &d)); /* heading 511 */
        z.pose_valid = 0; z.pose_distance_cm = 77;          /* stale value, invalid: zero bits */
        assert(buddy1_encode(BUDDY1_TOPIC_SNAPSHOT, &z, snap, sizeof snap) == 17U &&
               snap[13] == 0U && snap[14] == 0U && snap[15] == 0U && snap[16] == 0U);
        snap[14] = 1U; assert(!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, snap, 17U, &d));          /* data w/o valid */
        assert(!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, snap, 14U, &d));                        /* old length */
    }
    poll_after(20);
    assert(strcmp(published_topic, "car/telemetry/pose") == 0 && published_len == 4U &&
           buddy1_decode(BUDDY1_TOPIC_POSE, published, published_len, &d) &&
           d.pose_distance_cm == 1234U && d.pose_heading_deg == 270);
    puts("HOST PASS: route pose 21 bits/4 bytes, 4095/359 limits, padding/length rejection, own topic; snapshot pose section (bits 102-123), rejected values never cached");
}
/* Buddy 4 terrain: live-only, 59 bits/9 bytes (19 Nov 2026: + compass turn
 * rate and lifted/overturned), every pitch/roll, height and turn code
 * round-trips, unavailable means canonical zero, never cached. */
/* Buddy 5: live avoidance (38 bits/6 bytes) and FR6 scan profile (95 bits/
 * 13 bytes); every field boundary round-trips, unusable values are zero,
 * out-of-range and noncanonical packets are rejected, never cached. */
/* Field compares: these structs have padding, which memcmp would compare too
 * (garbage on stack copies; failed with arm64 Linux gcc 11, 14 Nov 2026). */
static int same_avoidance(const buddy1_avoidance_t *a, const buddy1_avoidance_t *b)
{
    return a->range_valid == b->range_valid && a->no_echo == b->no_echo && a->range_mm == b->range_mm &&
           a->servo_deg == b->servo_deg && a->state == b->state && a->fault == b->fault &&
           a->route == b->route && a->leg == b->leg && a->pass == b->pass && a->swapped == b->swapped;
}
static int same_scan(const buddy1_scan_t *a, const buddy1_scan_t *b)
{
    return a->valid == b->valid && a->scan_id == b->scan_id && a->closest_mm == b->closest_mm &&
           a->width_mm == b->width_mm && a->left_clear_mm == b->left_clear_mm &&
           a->right_clear_mm == b->right_clear_mm && a->forward_mm == b->forward_mm &&
           a->left_edge_mm == b->left_edge_mm && a->right_edge_mm == b->right_edge_mm;
}
static void test_buddy5(void)
{
    buddy1_snapshot_t v, d, before, after;
    buddy1_avoidance_t a;
    buddy1_scan_t q;
    UB p[32];
    unsigned mm, deg;
    int e;
    assert(buddy1_payload_size(BUDDY1_TOPIC_AVOIDANCE) == 6U && buddy1_payload_bits(BUDDY1_TOPIC_AVOIDANCE) == 38U);
    assert(buddy1_payload_size(BUDDY1_TOPIC_SCAN) == 13U && buddy1_payload_bits(BUDDY1_TOPIC_SCAN) == 95U);
    assert(buddy1_payload_size(BUDDY1_TOPIC_SCAN) <= BUDDY1_PAYLOAD_MAX);
    for (mm = 30; mm <= 4000; mm += 1) {
        deg = mm % 181U;
        v = (buddy1_snapshot_t){0};
        v.avoidance = (buddy1_avoidance_t){1, 0, (uint16_t)mm, (uint8_t)deg, (uint8_t)(mm % 7U), (uint8_t)(mm % 8U),
                                         (uint8_t)(mm % 3U), (uint8_t)(mm % 16U), (uint8_t)(mm % 5U), (uint8_t)(mm % 2U)};
        assert(buddy1_encode(BUDDY1_TOPIC_AVOIDANCE, &v, p, sizeof p) == 6U && (p[5] >> 6) == 0U);
        assert(buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, p, 6U, &d) && d.avoidance_valid &&
               same_avoidance(&d.avoidance, &v.avoidance));
    }
    v = (buddy1_snapshot_t){0};
    v.avoidance = (buddy1_avoidance_t){0, 1, 0, 0, 0, 0, 0, 0, 0, 0};          /* no echo */
    assert(buddy1_encode(BUDDY1_TOPIC_AVOIDANCE, &v, p, sizeof p) == 6U &&
           buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, p, 6U, &d) && d.avoidance.no_echo && !d.avoidance.range_valid);
    {
        const buddy1_avoidance_t bad[] = {
            {1, 0, 29, 90, 0, 0, 0, 0, 0, 0}, {1, 0, 4001, 90, 0, 0, 0, 0, 0, 0}, {0, 0, 500, 90, 0, 0, 0, 0, 0, 0},
            {1, 1, 500, 90, 0, 0, 0, 0, 0, 0}, {1, 0, 500, 181, 0, 0, 0, 0, 0, 0}, {1, 0, 500, 90, 7, 0, 0, 0, 0, 0},
            {1, 0, 500, 90, 0, 0, 3, 0, 0, 0}, {1, 0, 500, 90, 0, 0, 0, 0, 5, 0}, {1, 0, 500, 90, 0, 0, 0, 16, 0, 0},
            {1, 0, 500, 90, 0, 0, 0, 0, 0, 2}};
        for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; ++i) {
            v.avoidance = bad[i]; assert(buddy1_encode(BUDDY1_TOPIC_AVOIDANCE, &v, p, sizeof p) == 0U);
        }
    }
    p[0] = 3; p[1] = 0x04; p[2] = 0; p[3] = 0; p[4] = 0; p[5] = 0;         /* invalid range with mm */
    assert(!buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, p, 6U, &d));
    p[1] = 0; p[2] = 0; p[5] = 0x40; assert(!buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, p, 6U, &d)); /* padding */
    p[5] = 0; assert(!buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, p, 5U, &d));                       /* length */
    for (e = -4000; e <= 4000; e += 1) {
        v = (buddy1_snapshot_t){0};
        v.scan = (buddy1_scan_t){1, (uint8_t)(e & 0xFF), (uint16_t)(30 + (unsigned)(e + 4000) % 3971U),
                               (uint16_t)((unsigned)(e + 4000) % 4096U), (uint16_t)((unsigned)(e + 4000) % 4001U),
                               (uint16_t)((unsigned)(4000 - e) % 4001U), (uint16_t)((unsigned)(e + 4000) / 2U),
                               4000, (int16_t)e};
        assert(buddy1_encode(BUDDY1_TOPIC_SCAN, &v, p, sizeof p) == 13U && (p[12] >> 7) == 0U);
        assert(buddy1_decode(BUDDY1_TOPIC_SCAN, p, 13U, &d) && d.scan_valid &&
               same_scan(&d.scan, &v.scan));
    }
    v = (buddy1_snapshot_t){0}; v.scan.scan_id = 9;                             /* failed scan */
    assert(buddy1_encode(BUDDY1_TOPIC_SCAN, &v, p, sizeof p) == 13U &&
           buddy1_decode(BUDDY1_TOPIC_SCAN, p, 13U, &d) && !d.scan.valid && d.scan.scan_id == 9U);
    {
        const buddy1_scan_t bad[] = {
            {1, 0, 29, 0, 0, 0, 0, 0, 0}, {1, 0, 4001, 0, 0, 0, 0, 0, 0}, {1, 0, 100, 4096, 0, 0, 0, 0, 0},
            {1, 0, 100, 0, 4001, 0, 0, 0, 0}, {1, 0, 100, 0, 0, 4001, 0, 0, 0}, {1, 0, 100, 0, 0, 0, 4001, 0, 0},
            {1, 0, 100, 0, 0, 0, 0, 4001, 0}, {1, 0, 100, 0, 0, 0, 0, 0, -4001}, {1, 0, 100, 0, 0, 0, 0, -10, 10},
            {0, 0, 100, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 5, 0}, {2, 0, 100, 0, 0, 0, 0, 0, 0}};
        for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; ++i) {
            v.scan = bad[i]; assert(buddy1_encode(BUDDY1_TOPIC_SCAN, &v, p, sizeof p) == 0U);
        }
    }
    v.scan = (buddy1_scan_t){0}; assert(buddy1_encode(BUDDY1_TOPIC_SCAN, &v, p, sizeof p) == 13U);
    p[12] = 0x80U; assert(!buddy1_decode(BUDDY1_TOPIC_SCAN, p, 13U, &d));                     /* padding */
    p[12] = 0; p[2] = 0x02U; assert(!buddy1_decode(BUDDY1_TOPIC_SCAN, p, 13U, &d));          /* invalid with mm */
    assert(!buddy1_decode(BUDDY1_TOPIC_SCAN, p, 12U, &d));
    assert(buddy1_invalidate(BUDDY1_TOPIC_AVOIDANCE) == E_PAR && buddy1_invalidate(BUDDY1_TOPIC_SCAN) == E_PAR);
    empty_outbox();
    assert(buddy1_snapshot(&before));
    assert(buddy1_report_avoidance(0) == E_PAR && buddy1_report_scan(0) == E_PAR);
    a = (buddy1_avoidance_t){1, 0, 20, 90, 0, 0, 0, 0, 0, 0}; assert(buddy1_report_avoidance(&a) == E_PAR);
    a = (buddy1_avoidance_t){1, 0, 412, 150, 3, 0, 2, 3, 3, 1}; assert(buddy1_report_avoidance(&a) == E_OK);
    poll_after(20);
    assert(strcmp(published_topic, "car/telemetry/avoidance") == 0 && published_len == 6U &&
           buddy1_decode(BUDDY1_TOPIC_AVOIDANCE, published, published_len, &d) &&
           same_avoidance(&d.avoidance, &a));
    q = (buddy1_scan_t){1, 7, 250, 400, 180, 320, 380, 150, -250}; assert(buddy1_report_scan(&q) == E_OK);
    poll_after(20);
    assert(strcmp(published_topic, "car/telemetry/scan") == 0 && published_len == 13U &&
           buddy1_decode(BUDDY1_TOPIC_SCAN, published, published_len, &d) && same_scan(&d.scan, &q));
    assert(buddy1_snapshot(&after) && memcmp(&before, &after, sizeof before) == 0);
    puts("HOST PASS: Buddy 5 avoidance 38 bits/6 bytes and FR6 scan 95 bits/13 bytes: all range/edge codes, servo/state/route/leg/pass limits, unavailable=zero, padding/length rejection, live-only, own topics");
}
static int same_terrain(const buddy1_terrain_t *a, const buddy1_terrain_t *b)
{
    return a->tilt_valid == b->tilt_valid && a->pitch_x10 == b->pitch_x10 &&
           a->roll_x10 == b->roll_x10 && a->state == b->state && a->on_hump == b->on_hump &&
           a->climb_mm == b->climb_mm && a->peak_valid == b->peak_valid && a->peak_mm == b->peak_mm &&
           a->turn_valid == b->turn_valid && a->turn_dps == b->turn_dps && a->safety == b->safety;
}
static void test_terrain(void)
{
    buddy1_snapshot_t v = {0}, d, before, after;
    buddy1_terrain_t t;
    UB p[32];
    int x;
    assert(buddy1_payload_size(BUDDY1_TOPIC_TERRAIN) == 9U &&
           buddy1_payload_bits(BUDDY1_TOPIC_TERRAIN) == 59U);
    for (x = -900; x <= 900; ++x) {
        v = (buddy1_snapshot_t){0};
        v.terrain = (buddy1_terrain_t){1, (int16_t)x, (int16_t)(-x), (uint8_t)((unsigned)(x + 900) % 8U),
                                     1, (uint16_t)((unsigned)(x + 900) % 501U), 1, (uint16_t)((unsigned)(900 - x) % 501U),
                                     1, (int16_t)(x * 2 / 5), (uint8_t)((unsigned)(x + 900) % 3U)};
        assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 9U && (p[8] >> 3) == 0U);
        assert(buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d) && d.terrain_valid &&
               same_terrain(&d.terrain, &v.terrain));
    }
    for (x = -360; x <= 360; x += 720) { /* turn-rate extremes */
        v.terrain = (buddy1_terrain_t){1, 0, 0, 4, 0, 0, 0, 0, 1, (int16_t)x, 0};
        assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 9U &&
               buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d) && same_terrain(&d.terrain, &v.terrain));
    }
    v.terrain = (buddy1_terrain_t){1, 0, 0, 0, 0, 0, 0, 0, 1, 361, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){1, 0, 0, 0, 0, 0, 0, 0, 1, -361, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){1, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){1, 901, 0, 0, 0, 0, 0, 0, 0, 0, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){1, 0, -901, 0, 0, 0, 0, 0, 0, 0, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){1, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){1, 0, 0, 0, 1, 501, 0, 0, 0, 0, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){1, 0, 0, 0, 0, 0, 1, 501, 0, 0, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){1, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    v.terrain = (buddy1_terrain_t){1, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0}; assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 0U);
    /* Everything unavailable: terrain state still sent, all values zero. */
    v.terrain = (buddy1_terrain_t){0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0};
    assert(buddy1_encode(BUDDY1_TOPIC_TERRAIN, &v, p, sizeof p) == 9U && p[1] == 0U && p[2] == 0U &&
           p[3] == 0U && p[4] == 0x01U && p[5] == 0U && p[6] == 0U && p[7] == 0U && p[8] == 0U);
    assert(buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d) && !d.terrain.tilt_valid && d.terrain.state == 2U &&
           !d.terrain.turn_valid && d.terrain.safety == BUDDY1_SAFETY_NONE);
    p[1] = 0x02U; assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d));  /* unavailable + angle */
    p[1] = 0xF1U; p[2] = 0x07U; p[4] = 0U;                                  /* pitch +1016 */
    assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d));
    p[1] = 0; p[2] = 0; p[4] = 0x08U; assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d)); /* climb w/o hump */
    p[4] = 0; p[7] = 0x01U; assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d));            /* turn w/o valid */
    p[6] = 0xC0U; p[7] = 0xB4U; assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d)); /* valid, +361 deg/s */
    p[6] = 0x40U; p[7] = 0xB4U; assert(buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d) && d.terrain.turn_dps == 360); /* +360 */
    p[6] = 0; p[7] = 0; p[8] = 0x06U; assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d)); /* safety 3 */
    p[8] = 0x08U; assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 9U, &d));                      /* padding */
    p[8] = 0; assert(!buddy1_decode(BUDDY1_TOPIC_TERRAIN, p, 8U, &d));                          /* length */
    assert(buddy1_invalidate(BUDDY1_TOPIC_TERRAIN) == E_PAR); /* no snapshot section */
    empty_outbox();
    assert(buddy1_snapshot(&before));
    assert(buddy1_report_terrain(0) == E_PAR);
    t = (buddy1_terrain_t){1, 901, 0, 1, 0, 0, 0, 0, 0, 0, 0}; assert(buddy1_report_terrain(&t) == E_PAR);
    t = (buddy1_terrain_t){1, -305, 42, 5, 1, 17, 1, 33, 1, -123, BUDDY1_SAFETY_LIFTED}; assert(buddy1_report_terrain(&t) == E_OK);
    assert(buddy1_snapshot(&after) && memcmp(&before, &after, sizeof before) == 0);
    poll_after(20);
    assert(strcmp(published_topic, "car/telemetry/terrain") == 0 && published_len == 9U &&
           buddy1_decode(BUDDY1_TOPIC_TERRAIN, published, published_len, &d) &&
           same_terrain(&d.terrain, &t));
    puts("HOST PASS: Buddy 4 terrain 59 bits/9 bytes, all pitch/roll codes, +/-90.0, 0..500 mm and +/-360 deg/s limits, lifted/overturned, unavailable=zero, padding/length rejection, live-only, own topic");
}
static int same_sample(const buddy1_scan_sample_t *a, const buddy1_scan_sample_t *b)
{
    return a->scan_id == b->scan_id && a->peek == b->peek && a->index == b->index &&
           a->count == b->count && a->angle_deg == b->angle_deg && a->valid == b->valid &&
           a->distance_mm == b->distance_mm;
}
/* Buddy 5 raw scan readings (19 Nov 2026): 35 bits/6 bytes, every angle,
 * index/count pair and range limit round-trips; no reading = canonical zero. */
static void test_scan_sample(void)
{
    buddy1_snapshot_t v = {0}, d, before, after;
    buddy1_scan_sample_t q;
    UB p[32];
    unsigned angle, count, index, k;
    static const unsigned mm[] = {30, 31, 999, 3999, 4000};
    assert(buddy1_payload_size(BUDDY1_TOPIC_SCAN_SAMPLE) == 6U &&
           buddy1_payload_bits(BUDDY1_TOPIC_SCAN_SAMPLE) == 35U &&
           strcmp(buddy1_topic_name(BUDDY1_TOPIC_SCAN_SAMPLE), "car/telemetry/scan_sample") == 0);
    for (angle = 30; angle <= 150; angle += 5)
        for (count = 1; count <= BUDDY1_SCAN_SAMPLES_MAX; ++count)
            for (index = 0; index < count; ++index)
                for (k = 0; k <= sizeof mm / sizeof mm[0]; ++k) {
                    v.scan_sample = (buddy1_scan_sample_t){(UB)(angle + count), (UB)(count == 1U), (UB)index,
                        (UB)count, (UB)angle, (UB)(k < sizeof mm / sizeof mm[0]),
                        (UH)(k < sizeof mm / sizeof mm[0] ? mm[k] : 0U)};
                    assert(buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &v, p, sizeof p) == 6U && (p[5] >> 3) == 0U);
                    assert(buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, p, 6U, &d) && d.scan_sample_valid &&
                           same_sample(&d.scan_sample, &v.scan_sample));
                }
    q = (buddy1_scan_sample_t){1, 0, 0, 1, 90, 1, 200};
#define REJECT(field, value) do { v.scan_sample = q; v.scan_sample.field = (value); \
        assert(buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &v, p, sizeof p) == 0U); } while (0)
    REJECT(count, 0); REJECT(count, 17); REJECT(index, 1); REJECT(angle_deg, 25);
    REJECT(angle_deg, 155); REJECT(angle_deg, 92); REJECT(distance_mm, 29);
    REJECT(distance_mm, 4001); REJECT(peek, 2); REJECT(valid, 2);
    v.scan_sample = q; v.scan_sample.valid = 0; assert(buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &v, p, sizeof p) == 0U);
#undef REJECT
    v.scan_sample = q; assert(buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &v, p, sizeof p) == 6U);
    p[3] |= 0x3EU; assert(!buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, p, 6U, &d)); /* angle index 31 */
    assert(buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &v, p, sizeof p) == 6U);
    p[2] = 0x02U; assert(!buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, p, 6U, &d));                 /* index 1 of 1 */
    assert(buddy1_encode(BUDDY1_TOPIC_SCAN_SAMPLE, &v, p, sizeof p) == 6U);
    p[5] |= 0x08U; assert(!buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, p, 6U, &d));                /* padding */
    p[5] &= 0x07U; assert(!buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, p, 5U, &d));                /* length */
    assert(buddy1_invalidate(BUDDY1_TOPIC_SCAN_SAMPLE) == E_PAR); /* no snapshot section */
    empty_outbox();
    assert(buddy1_snapshot(&before));
    assert(buddy1_report_scan_sample(0) == E_PAR);
    q.angle_deg = 151; assert(buddy1_report_scan_sample(&q) == E_PAR);
    assert(buddy1_outbox_idle());
    q = (buddy1_scan_sample_t){200, 1, 0, 1, 135, 1, 287}; assert(buddy1_report_scan_sample(&q) == E_OK);
    assert(!buddy1_outbox_idle());                    /* readings wait behind it */
    assert(buddy1_snapshot(&after) && memcmp(&before, &after, sizeof before) == 0);
    poll_after(20);
    assert(strcmp(published_topic, "car/telemetry/scan_sample") == 0 && published_len == 6U &&
           buddy1_decode(BUDDY1_TOPIC_SCAN_SAMPLE, published, published_len, &d) &&
           same_sample(&d.scan_sample, &q));
    puts("HOST PASS: Buddy 5 scan sample 35 bits/6 bytes: every angle 30..150, every index/count 1..16, 30/4000 mm limits, no reading=zero, padding/length rejection, live-only, own topic, outbox-idle gate");
}
/* A full outbox must delay a map/snapshot reply, not drop it, and the
 * command itself runs once. */
static void test_reply_retry(void)
{
    buddy1_command_msg_t snapshot = {.kind = BUDDY1_COMMAND_SNAPSHOT, .payload = {BUDDY1_WIRE_VERSION}};
    buddy1_command_msg_t map = {.kind = BUDDY1_COMMAND_BARCODE_MAP};
    buddy1_barcode_map_t wanted = {'F', 'L', 'R', 'U'};
    buddy1_telemetry_msg_t msg;
    buddy1_barcode_map_t original;
    unsigned i, before;
    const BOOL was_open = buddy1_configuration_allowed();
    assert(buddy1_configuration_claim(&original, NULL) == E_OK && buddy1_configuration_reopen() == E_OK);
    assert(buddy1_map_pack(&wanted, map.payload));
    empty_outbox();
    for (i = 0; i < BUDDY1_QUEUE_MESSAGES; ++i) assert(buddy1_report_heading((H)i) == E_OK);
    assert(buddy1_command_process(&snapshot) == E_TMOUT);          /* one attempt, no sleep */
    sleeps = 0; sleep_drains = 0;
    assert(buddy1_command_process_retry(&snapshot, 3U) == E_TMOUT && sleeps == 2U);
    sleeps = 0; sleep_drains = 1;
    assert(buddy1_command_process_retry(&map, BUDDY1_REPLY_TRIES) == E_OK && sleeps == 1U);
    assert(buddy1_barcode_action('L') == BUDDY1_ACTION_LEFT);      /* applied once */
    before = outbox_count();
    while (buddy1_outbox_take(&msg)) { }
    assert(before == BUDDY1_QUEUE_MESSAGES && msg.type == BUDDY1_TOPIC_MAP_REPLY &&
           msg.payload[1] == BUDDY1_REPLY_OK);
    for (i = 0; i < BUDDY1_QUEUE_MESSAGES; ++i) assert(buddy1_report_heading((H)i) == E_OK);
    sleeps = 0;
    assert(buddy1_command_process_retry(&snapshot, BUDDY1_REPLY_TRIES) == E_OK && sleeps == 1U);
    sleep_drains = 0;
    empty_outbox();
    assert(buddy1_apply_barcode_map(&original) == E_OK);            /* restore map and latch */
    if (!was_open) assert(buddy1_configuration_close() == E_OK);
    puts("HOST PASS: map/snapshot replies wait for outbox space (10 ms retries, bounded), command applied once");
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
    buddy1_snapshot_t v = {0};
    UB packet[32]; unsigned topic, i, len;
    v.motion_valid = v.barcode_valid = v.hump_valid = v.heading_valid = 1;
    v.obstacle_valid = v.robot_state_valid = v.faults_valid = 1;
    v.left_mms = 120; v.right_mms = -118;
    v.barcode_letter = 'W'; v.barcode_action = BUDDY1_ACTION_LEFT;
    v.hump_count = 2; v.hump_height_mm = 45; v.heading_deg = 359;
    v.obstacle_distance_mm = 180; v.obstacle_width_mm = 140; v.obstacle_route = BUDDY1_ROUTE_LEFT;
    v.robot_state = BUDDY1_STATE_LINE_FOLLOWING; v.faults = 0x31;
    v.pose_valid = 1; v.pose_distance_cm = 2049; v.pose_heading_deg = 90;
    v.terrain = (buddy1_terrain_t){1, 123, -51, 5, 1, 23, 1, 41, 1, -87, BUDDY1_SAFETY_NONE};
    v.avoidance = (buddy1_avoidance_t){1, 0, 412, 150, 3, 0, 2, 3, 3, 1};
    v.scan = (buddy1_scan_t){1, 7, 250, 400, 180, 320, 380, 150, -250};
    v.scan_sample = (buddy1_scan_sample_t){7, 0, 3, 13, 75, 1, 412};
    for (topic = 0; topic < BUDDY1_TOPIC_COUNT; ++topic) {
        if (topic == BUDDY1_TOPIC_MAP_REPLY) continue;
        len = buddy1_encode((UB)topic, &v, packet, sizeof(packet)); assert(len);
        printf("VECTOR %s ", buddy1_topic_name((UB)topic));
        for (i = 0; i < len; ++i) printf("%02x", packet[i]);
        puts("");
    }
}

static void test_terrain_widths(void)
{
    buddy1_snapshot_t v={0},d,before;
    UB p[BUDDY1_PAYLOAD_MAX];
    assert(BUDDY1_PAYLOAD_MAX==17 && buddy1_payload_bits(BUDDY1_TOPIC_HUMP)==25 &&
           buddy1_payload_bits(BUDDY1_TOPIC_OBSTACLE)==26);
    for(unsigned height=0;height<512;++height) {
        v.hump_count=65535;v.hump_height_mm=(UH)height;
        unsigned n=buddy1_encode(BUDDY1_TOPIC_HUMP,&v,p,sizeof p);
        assert((n!=0)==(height<=500));
        p[0]=BUDDY1_WIRE_VERSION;p[1]=255;p[2]=255;
        p[3]=(UB)height;p[4]=(UB)(height>>8);
        assert((buddy1_decode(BUDDY1_TOPIC_HUMP,p,5,&d)!=0)==(height<=500));
        if(height<=500)assert(d.hump_height_mm==height&&d.hump_count==65535);
    }
    static const unsigned distances[]={0,29,30,4000,4001,4095};
    v.obstacle_width_mm=4095;v.obstacle_route=BUDDY1_ROUTE_RIGHT;
    for(unsigned i=0;i<sizeof distances/sizeof distances[0];++i) {
        unsigned distance=distances[i];v.obstacle_distance_mm=(UH)distance;
        assert((buddy1_encode(BUDDY1_TOPIC_OBSTACLE,&v,p,sizeof p)!=0)==(distance>=30&&distance<=4000));
        uint32_t body=distance|(4095U<<12)|(2U<<24);
        p[0]=BUDDY1_WIRE_VERSION;
        for(unsigned b=0;b<4;++b)p[1+b]=(UB)(body>>(8*b));
        assert((buddy1_decode(BUDDY1_TOPIC_OBSTACLE,p,5,&d)!=0)==(distance>=30&&distance<=4000));
    }
    assert(buddy1_snapshot(&before));
    assert(buddy1_report_hump(0,501)==E_PAR&&buddy1_snapshot(&d)&&
           d.hump_height_mm==before.hump_height_mm);
    /* A same-length v2 packet must never be reinterpreted as v3. */
    v.hump_height_mm=1;assert(buddy1_encode(BUDDY1_TOPIC_HUMP,&v,p,sizeof p)==5);
    p[0]=2;assert(!buddy1_decode(BUDDY1_TOPIC_HUMP,p,5,&d));
    puts("HOST PASS: v3 9/12-bit terrain fields, exhaustive height codes, sensor boundaries, no clipping/cache corruption, v2 rejection");
}
int main(void)
{
    buddy1_snapshot_t v = {0}, decoded;
    buddy1_command_msg_t command;
    UB packet[32], map_packet[4];
    buddy1_barcode_map_t map = {'A', 'B', 'C', 'D'};
    pthread_t workers[8]; unsigned i, before;
    int speed;
    assert(subsystem1_init() == E_OK);
    assert(buddy1_config_snapshot_close(NULL) == E_PAR);
    assert(buddy1_configuration_allowed());
    assert(buddy1_barcode_action('D') == BUDDY1_ACTION_UTURN);
    subsystem1_test();
    assert(failures == 0);
    {
        buddy1_barcode_map_t frozen;
        assert(buddy1_config_snapshot_close(&frozen) == E_OK);
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
        v.robot_state = (buddy1_robot_state_t)i;
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
    assert(tk_rcv_mbf(g_mbf_buddy1_commands, &command, TMO_POL) == (INT)sizeof(command));
    assert(memcmp(command.payload, map_packet, 4) == 0);
    incoming(NULL, "car/cmd/snapshot", 1); packet[0] = BUDDY1_WIRE_VERSION;
    fragment(NULL, packet, 1, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(g_mbf_buddy1_commands, &command, TMO_POL) == (INT)sizeof(command));
    assert(buddy1_command_process(&command) == E_OK);
    before = publish_count; mqtt_utk_poll(); assert(publish_count == before + 1);
    assert(published_len == BUDDY1_PAYLOAD_MAX && strcmp(published_topic, "car/reply/snapshot") == 0 &&
           buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, published, published_len, &decoded));
    incoming(NULL, "car/cmd/snapshot", 2); fragment(NULL, packet, 1, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(g_mbf_buddy1_commands, &command, TMO_POL) == E_TMOUT);
    incoming(NULL, "car/cmd/barcode_map", 17); fragment(NULL, map_packet, 4, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(g_mbf_buddy1_commands, &command, TMO_POL) == E_TMOUT);
    incoming(NULL, "car/cmd/snapshot/this/topic/is/intentionally/far/too/long/to/be/accepted", 1);
    fragment(NULL, packet, 1, MQTT_DATA_FLAG_LAST);
    assert(tk_rcv_mbf(g_mbf_buddy1_commands, &command, TMO_POL) == E_TMOUT);
    assert(mqtt_utk_app_link(0));
    assert(buddy1_snapshot(&decoded) && (decoded.faults & BUDDY1_FAULT_WIFI));
    assert(mqtt_utk_app_link(1));
    assert(buddy1_snapshot(&decoded) && decoded.faults == (BUDDY1_FAULT_MASK & ~BUDDY1_FAULT_WIFI));
    empty_outbox();
    assert(buddy1_report_motion(0, 0) == E_OK);
    poll_after(20); assert(published_len == 4 && published[1] == 0 && published[2] == 0 && published[3] == 0);
    heartbeat_pending = 0; heartbeat(NULL); assert(published_len == 2 &&
        buddy1_decode(BUDDY1_TOPIC_STATUS, published, published_len, &decoded));
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
    test_control_preview_waits_for_retry();
#endif
#if APP_IMU_DEBUG
    test_imu_stream();
#endif
    emit_vectors();
    puts("HOST PASS: binary FIFO, full speed range, 64 fault masks, eight concurrent producers, real MQTT callbacks and bridge");
    return 0;
}
