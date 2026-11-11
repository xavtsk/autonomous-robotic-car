/* Opt-in runtime. One sampling owner serialises all navigation/control calls.
 */
#include "s3_integration.h"
#include "vehicle_controller.h"
#include "integration_codec.h"
#include "../diagnostics/robot_diagnostics.h"
#include "../subsystem3/ir_sensor.h"
#include "../subsystem3/subsystem3.h"
#include "builtin_calibration.h"
#include "course_profile.h"
#include "barcode_capture.h"
#include "ir_debug.h"
#include "control_debug.h"
#include "terrain_integration.h"
#include "remote_control.h"
#if APP_S4
#include "../subsystem4/subsystem4.h"
#endif
#if APP_S5
#include "../subsystem5/s5_api.h"
#endif
#include <sys/sysdef.h>
#if APP_S3_MOTION
#ifndef S3_BARCODE_CAPTURE
#define S3_BARCODE_CAPTURE 1 /* barcode-sensor trace, printed over USB after a run */
#endif
#if S3_BARCODE_CAPTURE
static bcap_t g_bcap; /* written by the sampling owner, read by report; g_mutex */
static bool g_bcap_valid;
/* Wi-Fi copy of the trace: sent in binary packets on MQTT_UTK_DEBUG_TOPIC after
 * a run ends, sharing publish slots with routine telemetry. The flag is a cheap
 * hint read by the network task every poll; the cursor is used under g_mutex. */
static volatile bool g_bcap_net_pending;
static bcap_cursor_t g_bcap_net_cursor;
#if defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
extern volatile UW tm_usb_link_state; /* 3 = USB terminal open (DTR) */
#define BCAP_TERMINAL_OPEN() (tm_usb_link_state == 3U)
#else
#define BCAP_TERMINAL_OPEN() (true)
#endif
#endif
#define S3_COLLECTION_US (1000000U)
#define S3_EMPTY_RANGE_MINIMUM (4095U) /* 12-bit ADC maximum */
static ID g_task, g_report, g_cycle, g_mutex;
static s3_drive_t g_drive;
static s3_status_packet_t g_status;
static s3_control_packet_t g_command;
static struct { unsigned pending:1, stop:1, status_ready:1, reset_pending:1; } g_shared;
static bool g_reset_waiting; /* sampling-owner-only completion state */
static bool g_reset_remote;
static s3_calibration_collector_t g_collector[S3_SENSOR_COUNT];
static bool g_collecting;
#if APP_IR_DEBUG
/* One latest-only packet under g_mutex; no queue, task or peripheral owner. */
static ir_debug_packet_t g_ir_debug;
static bool g_ir_ready, g_ir_seen;
static uint32_t g_ir_last;
static uint8_t g_ir_sequence;
static control_debug_packet_t g_control_debug;
static bool g_control_ready, g_control_seen;
static uint32_t g_control_last;
static uint8_t g_control_sequence, g_last_barcode_status;
static uint16_t g_debug_events[3]; /* saturating counts since boot */
#endif
static s3_sensor_id_t g_sensor;
static ir_surface_t g_label;
static uint32_t g_collect_since;
static uint32_t now_us(void) { return in_w(TIMER_TIMERAWL); }
/* g_mutex protects the bounded remote mailbox and correlated result ring. */
static struct {
    uint32_t epoch, last_id, sampled_us, published_us, stop_sequence;
    uint8_t flags, head, count;
    bool connected, published, physical_stop, inflight;
    remote_request_t pending, reset;
    uint8_t reply[6][REMOTE_REPLY_SIZE];
} g_remote;
static void remote_reply(const remote_request_t *r,uint8_t result,uint8_t reason)
{
    if (!r->valid) return;
    if (g_remote.count==6U) { g_remote.head=(g_remote.head+1U)%6U; --g_remote.count; }
    uint8_t *p=g_remote.reply[(g_remote.head+g_remote.count)%6U];
    p[0]=REMOTE_VERSION; p[1]=r->action; remote_put32(p+2,r->id);
    p[6]=result; p[7]=reason;
    remote_put32(p+8,r->action==REMOTE_STOP?g_remote.epoch:r->epoch); ++g_remote.count;
}
static void remote_cancel(remote_request_t *r)
{
    remote_reply(r,REMOTE_CANCELLED,REMOTE_SUPERSEDED); r->valid=false;
}
static void remote_finish(const remote_request_t *r,bool ok,uint8_t reason)
{
    if (!r->valid || tk_loc_mtx(g_mutex,TMO_FEVR)!=E_OK) return;
    remote_reply(r,ok?REMOTE_EXECUTED:REMOTE_REJECTED,ok?REMOTE_NONE:reason);
    g_remote.inflight=false;
    (void)tk_unl_mtx(g_mutex);
}
void s3_remote_link(uint32_t epoch,bool up)
{
    if (g_mutex<=0 || tk_loc_mtx(g_mutex,TMO_FEVR)!=E_OK) return;
    if (g_remote.pending.valid && g_remote.pending.action!=REMOTE_STOP) {
        remote_cancel(&g_remote.pending); g_shared.pending=false; g_shared.reset_pending=false;
    }
    remote_cancel(&g_remote.reset); /* owner finishes safe reset housekeeping */
    g_remote.connected=up;
    if (up) { g_remote.epoch=epoch?epoch:1U; g_remote.last_id=0U; }
    g_remote.published=false;
    (void)tk_unl_mtx(g_mutex);
}
void s3_remote_receive(const uint8_t *p,unsigned length)
{
    if (!p || length!=REMOTE_COMMAND_SIZE || p[0]!=REMOTE_VERSION ||
        p[1]<REMOTE_START || p[1]>REMOTE_RESET || g_mutex<=0) return;
    const remote_request_t r={.action=p[1],.id=remote_get32(p+2),
        .epoch=remote_get32(p+6),.time_us=remote_get32(p+10),.valid=true};
    if (tk_loc_mtx(g_mutex,TMO_FEVR)!=E_OK) return;
    uint8_t reason=REMOTE_NONE;
    const bool fresh=g_remote.connected && r.epoch==g_remote.epoch &&
        (uint32_t)(now_us()-r.time_us)<=REMOTE_MAX_AGE_US;
    if (r.action!=REMOTE_STOP && !fresh) reason=REMOTE_STALE;
    else if (r.action!=REMOTE_STOP && !remote_newer(r.id,g_remote.last_id)) reason=REMOTE_REPLAY;
    else if (r.action!=REMOTE_STOP && g_remote.physical_stop) reason=REMOTE_NOT_READY;
    else if (r.action!=REMOTE_STOP && (g_shared.pending || g_shared.stop || g_shared.reset_pending || g_remote.inflight)) reason=REMOTE_BUSY;
    /* Newer rejected IDs are consumed, too: no older START after a newer STOP. */
    if (r.epoch==g_remote.epoch && remote_newer(r.id,g_remote.last_id)) g_remote.last_id=r.id;
    if (reason!=REMOTE_NONE) remote_reply(&r,REMOTE_REJECTED,reason);
    else {
        const s3_control_request_t c={.kind=r.action==REMOTE_START?S3_CONTROL_START:
            r.action==REMOTE_STOP?S3_CONTROL_STOP:S3_CONTROL_RESET};
        if (r.action==REMOTE_STOP) {
            ++g_remote.stop_sequence;
            remote_cancel(&g_remote.pending); remote_cancel(&g_remote.reset);
            g_shared.stop=true; g_shared.pending=false; g_shared.reset_pending=false;
        } else {
            (void)s3_control_pack(&c,&g_command);
            g_shared.pending=true; g_shared.reset_pending=r.action==REMOTE_RESET;
        }
        g_remote.pending=r; remote_reply(&r,REMOTE_QUEUED,REMOTE_NONE);
    }
    (void)tk_unl_mtx(g_mutex);
}
unsigned s3_remote_reply_take(uint8_t *p,unsigned capacity)
{
    unsigned n=0;
    if (!p || capacity<REMOTE_REPLY_SIZE || g_mutex<=0 || tk_loc_mtx(g_mutex,TMO_POL)!=E_OK) return 0;
    if (g_remote.count) {
        for(unsigned i=0;i<REMOTE_REPLY_SIZE;i++) p[i]=g_remote.reply[g_remote.head][i];
        g_remote.head=(g_remote.head+1U)%6U; --g_remote.count; n=REMOTE_REPLY_SIZE;
    }
    (void)tk_unl_mtx(g_mutex); return n;
}
unsigned s3_remote_status_take(uint8_t *p,unsigned capacity)
{
    unsigned n=0;
    if (!p || capacity<REMOTE_STATUS_SIZE || g_mutex<=0 || tk_loc_mtx(g_mutex,TMO_POL)!=E_OK) return 0;
    const uint32_t now=now_us();
    if (g_remote.connected && (!g_remote.published || (uint32_t)(now-g_remote.published_us)>=500000U)) {
        p[0]=REMOTE_VERSION; remote_put32(p+1,g_remote.epoch); remote_put32(p+5,now);
        remote_put32(p+9,g_remote.last_id);
        p[13]=(uint32_t)(now-g_remote.sampled_us)<=200000U?g_remote.flags:0U;
        g_remote.published_us=now; g_remote.published=true; n=REMOTE_STATUS_SIZE;
    }
    (void)tk_unl_mtx(g_mutex); return n;
}
uint32_t s3_remote_stop_sequence(void)
{
    uint32_t sequence=0;
    if (g_mutex>0 && tk_loc_mtx(g_mutex,TMO_FEVR)==E_OK) {
        sequence=g_remote.stop_sequence; (void)tk_unl_mtx(g_mutex);
    }
    return sequence;
}
void s3_remote_physical_stop(bool held)
{
    if (g_mutex>0 && tk_loc_mtx(g_mutex,TMO_FEVR)==E_OK) {
        g_remote.physical_stop=held; (void)tk_unl_mtx(g_mutex);
    }
}
static bool remote_permitted(const remote_request_t *r)
{
    bool permitted=false;
    if (tk_loc_mtx(g_mutex,TMO_FEVR)==E_OK) {
        permitted=g_remote.connected && r->epoch==g_remote.epoch &&
            !g_shared.stop && !g_remote.physical_stop && r->id==g_remote.last_id &&
            (uint32_t)(now_us()-r->time_us)<=REMOTE_MAX_AGE_US;
        (void)tk_unl_mtx(g_mutex);
    }
    return permitted;
}
bool s3_integration_submit(const s3_control_request_t *request)
{
    bool accepted = false;
    s3_control_packet_t packet;
    if ((request == 0) || (g_mutex <= 0) ||
        ((unsigned int)request->kind > S3_CONTROL_RESET) ||
        !s3_control_pack(request, &packet))
    {
        return false;
    }
    if (tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }
    if (request->kind == S3_CONTROL_STOP)
    {
        remote_cancel(&g_remote.pending);
        remote_cancel(&g_remote.reset);
        g_shared.stop = true;
        g_shared.pending = false;
        g_shared.reset_pending = false;
        accepted = true;
    }
    else if (!g_shared.pending && !g_shared.stop && !g_shared.reset_pending)
    {
        g_command = packet;
        g_shared.pending = true;
        g_shared.reset_pending = request->kind == S3_CONTROL_RESET;
        accepted = true;
    }
    (void)tk_unl_mtx(g_mutex);
    return accepted;
}
bool s3_integration_status(s3_integration_status_t *status)
{
    s3_status_packet_t packet;
    bool ready;
    if ((status == 0) || (g_mutex <= 0))
    {
        return false;
    }
    if (tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }
    packet = g_status;
    ready = g_shared.status_ready;
    (void)tk_unl_mtx(g_mutex);
    return ready && s3_status_unpack(&packet, status);
}
static bool execute(const s3_control_request_t *c, uint32_t now)
{
    if (c->kind == S3_CONTROL_STOP)
    {
        s3_drive_stop(&g_drive);
        g_collecting = false;
        return true;
    }
    if (c->kind == S3_CONTROL_RESET)
    {
        if (!terrain_control_reset_ready(now) || !s3_drive_reset(&g_drive, now)) { return false; }
        terrain_control_clear_fault();
        return true;
    }
    if (c->kind == S3_CONTROL_START)
    {
        /* Ineligible STARTs (already driving, FAULT, active lifecycle,
         * collecting, not ready) are refused BEFORE touching the map. */
        if (!terrain_control_ready(now) || g_collecting ||
            ((g_drive.state != S3_DRIVE_OFF) &&
             (g_drive.state != S3_DRIVE_HALTED)) ||
            (g_drive.navigation.lifecycle != S3_RUN_INACTIVE) ||
            (s3_navigation_readiness(&g_drive.navigation, now) != 0U) ||
            !vehicle_controller_prepare_start(&g_drive.navigation))
        {
            return false;
        }
        /* s3_drive_start() also checks Buddy 2 (fresh, IDLE, braked, no fault).
         * If it refuses, undo only a map freeze this attempt newly took. */
        if (!s3_drive_start(&g_drive, now))
        {
            vehicle_controller_cancel_start();
            return false;
        }
        if (!terrain_control_begin(now)) {
            s3_drive_stop(&g_drive);
            vehicle_controller_cancel_start();
            return false;
        }
        vehicle_controller_commit_start();
        return true;
    }
    if ((g_drive.navigation.lifecycle != S3_RUN_INACTIVE) || g_collecting ||
        (g_drive.state == S3_DRIVE_FAULT))
    {
        return false;
    }
    if (c->kind == S3_CONTROL_MAPPING)
    {
        return vehicle_controller_apply_mapping(c->map) &&
               s3_navigation_set_mapping(&g_drive.navigation, c->map);
    }
    if ((unsigned int)c->sensor >= S3_SENSOR_COUNT)
    {
        return false;
    }
    if (c->kind == S3_CONTROL_APPLY_CALIBRATION)
    {
        return s3_navigation_apply_calibration(&g_drive.navigation, c->sensor,
                                               &g_collector[c->sensor]);
    }
    g_sensor = c->sensor;
    g_collecting = true;
    g_collect_since = now;
    g_label = c->kind == S3_CONTROL_WHITE ? IR_SURFACE_WHITE : IR_SURFACE_BLACK;
    /* Same empty-range convention as s3_calibration_begin(): the minimum
     * starts at the 12-bit maximum. Starting it at 0 kept both minima at 0,
     * so every white/black pair looked overlapping and was rejected. */
    if (g_label == IR_SURFACE_WHITE)
    {
        g_collector[g_sensor].white =
            (s3_calibration_range_t){.minimum = S3_EMPTY_RANGE_MINIMUM};
    }
    else
    {
        g_collector[g_sensor].black =
            (s3_calibration_range_t){.minimum = S3_EMPTY_RANGE_MINIMUM};
    }
    return true;
}
static void tick(void *unused)
{
    (void)unused;
    if (g_task > 0)
    {
        (void)tk_wup_tsk(g_task);
    }
}
static bool drive_running(s3_drive_state_t state)
{
    return (state >= S3_DRIVE_FOLLOW) && (state <= S3_DRIVE_REACQUIRE);
}
static void reset_completed(void)
{
    if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
    {
        g_shared.reset_pending = false;
        (void)tk_unl_mtx(g_mutex);
    }
}
#if APP_IR_DEBUG
static bool debug_scaled(float value, float scale, int32_t limit, int32_t *out)
{
    const float v = value * scale;
    /* Ordered comparisons also reject NaN/infinity, without newlib headers
     * (whose size_t conflicts with this kernel's freestanding types). */
    if (!(v >= -(float)limit && v <= (float)limit)) return false;
    *out = (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
    return true;
}
static bool capture_control_debug(const s3_integration_status_t *st,
                                  control_debug_packet_t *packet)
{
    /* Owned by the sampling task. Static scratch avoids growing its stack. */
    static control_debug_sample_t d;
    static motion_debug_snapshot_t m;
    const uint32_t now = st->navigation.timestamp_us;
    static const uint32_t event_bits[3] = {S3_EVENT_BARCODE,S3_EVENT_JUNCTION,S3_EVENT_LINE_LOST};
    for (unsigned i=0; i<3; ++i)
        if ((st->navigation.events & event_bits[i]) && g_debug_events[i] != UINT16_MAX)
            ++g_debug_events[i];
    if (st->navigation.barcode_status != S3_BARCODE_NONE)
        g_last_barcode_status = (uint8_t)st->navigation.barcode_status;
    if (g_control_seen && (uint32_t)(now-g_control_last)<200000U) return false;
    g_control_seen=true; g_control_last=now;
    d=(control_debug_sample_t){0};
    d.timestamp_us=now; d.sequence=g_control_sequence++;
    if (motion_try_debug_snapshot(&m))
    {
        d.motion_available=1; d.motion_timestamp_us=m.timestamp_us;
        d.motion_state=m.state; d.motion_fault=m.fault; d.braked=m.braked; d.busy=m.busy;
        /* Motion can preempt this task after the ADC frame was timestamped.
         * Judge freshness against the current clock, not that older frame. */
        d.motion_valid=m.valid && (uint32_t)(now_us()-m.timestamp_us)<=50000U;
        d.motion_valid = d.motion_valid &&
            debug_scaled(m.setpoint_mps[0],1000,4095,&d.setpoint_left) &&
            debug_scaled(m.setpoint_mps[1],1000,4095,&d.setpoint_right) &&
            debug_scaled(m.measured_mps[0],1000,4095,&d.measured_left) &&
            debug_scaled(m.measured_mps[1],1000,4095,&d.measured_right) &&
            debug_scaled(m.duty[0],100,100,&d.duty_left) &&
            debug_scaled(m.duty[1],100,100,&d.duty_right) &&
            m.encoder[0]>=-524288 && m.encoder[0]<=524287 &&
            m.encoder[1]>=-524288 && m.encoder[1]<=524287;
        if (d.motion_valid) { d.encoder_left=m.encoder[0]; d.encoder_right=m.encoder[1]; }
        else { d.setpoint_left=d.setpoint_right=d.measured_left=d.measured_right=0;
               d.duty_left=d.duty_right=0; }
    }
    d.drive_state=st->state; d.reason=st->reason; d.pending=g_drive.pending;
    d.line_error=st->navigation.line.valid ? st->navigation.line.error : 0;
    d.line_flags=st->navigation.line.valid | st->navigation.line.present<<1 |
        st->navigation.line.junction_candidate<<2 | st->navigation.line.touching<<3 |
        st->navigation.rearming<<4;
    d.barcode_status=g_last_barcode_status;
    d.accepted_calibration=st->navigation.accepted_calibration;
    d.readiness=st->navigation.readiness_blockers;
    d.barcode_events=g_debug_events[0]; d.junction_events=g_debug_events[1]; d.lost_events=g_debug_events[2];
    d.sample_gap_us=st->navigation.sample_gap_us; d.run_max_gap_us=st->run_timing.max_gap_us;
#define DEBUG_CAL(prefix,id) \
    d.prefix##_white=g_drive.navigation.config.calibration[id].white_raw; \
    d.prefix##_black=g_drive.navigation.config.calibration[id].black_raw; \
    d.prefix##_white_limit=g_drive.navigation.config.calibration[id].white_limit; \
    d.prefix##_black_limit=g_drive.navigation.config.calibration[id].black_limit;
    DEBUG_CAL(left,S3_SENSOR_LEFT)
    DEBUG_CAL(right,S3_SENSOR_RIGHT)
    DEBUG_CAL(barcode,S3_SENSOR_BARCODE)
#undef DEBUG_CAL
    d.cal_sensor=st->calibration_sensor; d.collecting=st->collecting;
    d.white_min=st->white.count ? st->white.minimum : 0;
    d.white_max=st->white.count ? st->white.maximum : 0;
    d.black_min=st->black.count ? st->black.minimum : 0;
    d.black_max=st->black.count ? st->black.maximum : 0;
    d.white_count=st->white.count; d.black_count=st->black.count;
    return control_debug_pack(&d,packet);
}
#endif
static void sample(INT code, void *unused)
{
    static s3_sensor_frame_t frame;
    static motion_feedback_t feedback;
    static s3_control_request_t command;
    static s3_integration_status_t working;
    int32_t adc;
    (void)code;
    (void)unused;
    (void)tk_slp_tsk(TMO_FEVR);
    adc = ir_sensor_init(&g_drive.navigation.config.acquisition);
    for (;;)
    {
#if APP_DIAGNOSTICS
        uint32_t started_us = robot_diagnostics_now_us();
#endif
        bool pending = false;
        remote_request_t remote_command={0};
        const bool was_running = drive_running(g_drive.state);
        /* STOP is handled before sampling, even if an earlier command queued.
         */
        if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
        {
            if (g_shared.stop)
            {
                command = (s3_control_request_t){.kind = S3_CONTROL_STOP};
                pending = true;
            }
            else if (g_shared.pending)
            {
                pending = s3_control_unpack(&g_command, &command);
            }
            remote_command=g_remote.pending;
            g_remote.inflight=remote_command.valid;
            g_remote.pending.valid=false;
            g_shared.stop = false;
            g_shared.pending = false;
            (void)tk_unl_mtx(g_mutex);
        }
        if (pending && (command.kind == S3_CONTROL_STOP))
        {
            g_reset_waiting = false;
            g_reset_remote = false;
            const bool ok=execute(&command, now_us());
            if (remote_command.valid) remote_finish(&remote_command,ok,REMOTE_NOT_READY);
            else { working.command_ok=ok; working.command_sequence++; }
            pending = false;
        }
        feedback = (motion_feedback_t){0};
        (void)motion_get_feedback(&feedback);
        working.adc_status = adc;
        if (adc == E_OK)
        {
            working.adc_status = ir_sensor_read_frame(&frame);
        }
        if (working.adc_status != E_OK)
        {
            frame = (s3_sensor_frame_t){0};
            frame.timestamp_us = now_us();
        }
        /* ADC waits may allow a newer IMU publication than the frame start. */
        terrain_control_poll(&g_drive, now_us());
        (void)s3_drive_process(&g_drive, &frame, &feedback);
        /* A reset request is not a reset acknowledgement. The drive owner
         * waits for a newer safe feedback publication, or its 500 ms timeout.
         * Publish exactly one command result after that check, not the old
         * pre-reset FAULT snapshot. STOP above can always cancel the wait. */
        if (g_reset_waiting &&
            (s3_drive_reset_result(&g_drive) != S3_RESET_PENDING))
        {
            const bool ok=s3_drive_reset_result(&g_drive)==S3_RESET_OK;
            remote_request_t reset={0};
            if (tk_loc_mtx(g_mutex,TMO_FEVR)==E_OK) {
                reset=g_remote.reset; g_remote.reset.valid=false;
                (void)tk_unl_mtx(g_mutex);
            }
            if (g_reset_remote) { if (reset.valid) remote_finish(&reset,ok,REMOTE_NOT_READY); }
            else { working.command_ok=ok; working.command_sequence++; }
            g_reset_remote=false;
            g_reset_waiting = false;
            reset_completed();
        }
#if S3_BARCODE_CAPTURE
        if (was_running)
        {
            const bcap_input_t in = {
                frame.timestamp_us,
                frame.sensor[S3_SENSOR_BARCODE].raw,
                g_drive.output.sensor[S3_SENSOR_BARCODE].darkness,
                (uint8_t)g_drive.output.sensor[S3_SENSOR_BARCODE].surface,
                (uint8_t)g_drive.output.barcode_status,
                g_drive.output.barcode.letter,
                (g_drive.navigation.barcode.collecting != 0U) ||
                    s3_barcode_record_active(&g_drive.navigation.barcode_record),
                g_drive.navigation.rearming,
                feedback.valid ? feedback.centre_m : 0.0};
            if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
            {
                bcap_record(&g_bcap, &in);
                if (!drive_running(g_drive.state))
                {
                    bcap_end_run(&g_bcap);
                    g_bcap_net_cursor = (bcap_cursor_t){0};
                    g_bcap_net_pending = true;
                }
                (void)tk_unl_mtx(g_mutex);
            }
        }
#endif
        if (was_running)
        {
            /* Include the interval which caused a halt/fault; do not allow
             * later idle samples to erase it or add startup noise to a run. */
            s3_run_timing_record(&working.run_timing,
                                 g_drive.output.sample_gap_us,
                                 frame.timestamp_us,
                                 g_drive.navigation.config.max_sample_age_us);
        }
        if (g_collecting)
        {
            const s3_raw_sample_t *raw = &frame.sensor[g_sensor];
            if (raw->valid && (raw->error == 0))
            {
                (void)s3_calibration_add(&g_collector[g_sensor], g_label,
                                         raw->raw);
            }
            if ((uint32_t)(frame.timestamp_us - g_collect_since) >=
                S3_COLLECTION_US)
            {
                g_collecting = false;
            }
        }
        if (pending)
        {
            bool permitted=!remote_command.valid || remote_permitted(&remote_command);
            bool ok = permitted && execute(&command, now_us());
            /* START changes owner state/heartbeat only: velocity is sent on
             * the next sample. If STOP/reconnect arrived during that change,
             * cancel before publishing success or reaching a velocity tick. */
            if (ok && remote_command.valid && !remote_permitted(&remote_command)) {
                const s3_control_request_t stop_command={.kind=S3_CONTROL_STOP};
                (void)execute(&stop_command,now_us()); ok=false; permitted=false;
            }
            if ((command.kind == S3_CONTROL_RESET) && ok &&
                (s3_drive_reset_result(&g_drive) == S3_RESET_PENDING))
            {
                g_reset_waiting = true;
                g_reset_remote = remote_command.valid;
                if (remote_command.valid && tk_loc_mtx(g_mutex,TMO_FEVR)==E_OK) {
                    if (!g_shared.stop && !g_remote.physical_stop && g_remote.connected &&
                        remote_command.epoch==g_remote.epoch) g_remote.reset=remote_command;
                    else remote_reply(&remote_command,REMOTE_CANCELLED,REMOTE_SUPERSEDED);
                    g_remote.inflight=false;
                    (void)tk_unl_mtx(g_mutex);
                }
            }
            else
            {
                if (remote_command.valid) remote_finish(&remote_command,ok,permitted?REMOTE_NOT_READY:REMOTE_STALE);
                else { working.command_ok=ok; working.command_sequence++; }
                if (command.kind == S3_CONTROL_RESET) { reset_completed(); }
            }
            if ((command.kind == S3_CONTROL_START) && ok)
            {
                s3_run_timing_begin(&working.run_timing, now_us());
#if S3_BARCODE_CAPTURE
                if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
                {
                    bcap_begin_run(&g_bcap, now_us());
                    g_bcap_valid = true;
                    g_bcap_net_pending = false;
                    (void)tk_unl_mtx(g_mutex);
                }
#endif
            }
        }
        working.navigation = g_drive.output;
        /* Controls can change readiness after this cycle's observation. */
        working.navigation.lifecycle = g_drive.navigation.lifecycle;
        working.navigation.accepted_calibration =
            g_drive.navigation.accepted_calibration;
        working.navigation.readiness_blockers =
            s3_navigation_readiness(&g_drive.navigation, now_us());
        working.motion = feedback;
        working.state = g_drive.state;
        working.reason = g_drive.reason;
        s3_timing_increment(&working.samples);
        if (g_drive.output.sample_gap_us > working.max_gap_us)
        {
            working.max_gap_us = g_drive.output.sample_gap_us;
        }
        if (g_drive.output.sample_gap_us > S3_SAMPLE_PERIOD_US)
        {
            s3_timing_increment(&working.above_target);
        }
        working.collecting = g_collecting;
        working.calibration_sensor = g_sensor;
        working.white = s3_calibration_summary(&g_collector[g_sensor].white);
        working.black = s3_calibration_summary(&g_collector[g_sensor].black);
        s3_status_packet_t packet;
        bool packed = s3_status_pack(&working, &packet);
        if (!packed) { s3_drive_stop(&g_drive); }
#if APP_IR_DEBUG
        ir_debug_packet_t ir_packet;
        control_debug_packet_t control_packet;
        const bool control_packed=capture_control_debug(&working,&control_packet);
        bool ir_packed = false;
        if (ir_debug_due(frame.timestamp_us, &g_ir_last, &g_ir_seen))
        {
            static const s3_sensor_id_t order[3] = {S3_SENSOR_LEFT, S3_SENSOR_RIGHT, S3_SENSOR_BARCODE};
            ir_debug_sample_t preview = {0};
            preview.timestamp_us = frame.timestamp_us;
            preview.sequence = g_ir_sequence++;
            for (unsigned i = 0U; i < 3U; ++i)
            {
                const s3_sensor_id_t id = order[i];
                const s3_raw_sample_t *raw = &frame.sensor[id];
                if (raw->enabled && raw->valid && raw->error == 0)
                {
                    preview.raw_valid |= (uint8_t)(1U << i);
                    preview.raw[i] = raw->raw;
                    if (working.navigation.sensor[id].valid)
                    {
                        preview.normalised_valid |= (uint8_t)(1U << i);
                        preview.darkness[i] = working.navigation.sensor[id].darkness;
                        preview.surface[i] = (uint8_t)working.navigation.sensor[id].surface;
                    }
                }
            }
            ir_packed = ir_debug_pack(&preview, &ir_packet);
        }
#endif
        uint8_t remote_flags=REMOTE_AVAILABLE;
        const uint32_t remote_now=now_us();
        if (drive_running(g_drive.state)) remote_flags|=REMOTE_RUNNING;
        const bool motor_safe=feedback.valid && feedback.braked && feedback.state==MOTION_IDLE &&
            feedback.fault==MOTION_FAULT_NONE && (uint32_t)(remote_now-feedback.timestamp_us)<=200000U;
        if (!g_collecting && (g_drive.state==S3_DRIVE_OFF || g_drive.state==S3_DRIVE_HALTED) &&
            g_drive.navigation.lifecycle==S3_RUN_INACTIVE &&
            s3_navigation_readiness(&g_drive.navigation,remote_now)==0U &&
            terrain_control_ready(remote_now) && motor_safe) remote_flags|=REMOTE_CAN_START;
        if (g_drive.state==S3_DRIVE_FAULT && terrain_control_reset_ready(remote_now)) remote_flags|=REMOTE_CAN_RESET;
        if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
        {
            g_remote.flags=remote_flags; g_remote.sampled_us=remote_now;
            if (g_shared.stop || g_shared.pending || g_shared.reset_pending || g_remote.physical_stop)
                g_remote.flags&=(uint8_t)~(REMOTE_CAN_START|REMOTE_CAN_RESET);
            if (packed) { g_status = packet; }
            g_shared.status_ready = packed;
#if APP_IR_DEBUG
            if (ir_packed) { g_ir_debug = ir_packet; g_ir_ready = true; }
            if (control_packed) { g_control_debug=control_packet; g_control_ready=true; }
#endif
            (void)tk_unl_mtx(g_mutex);
        }
        if (packed) { vehicle_controller_observe(&working); }
#if APP_DIAGNOSTICS
        robot_diagnostics_record(ROBOT_DIAG_IR, started_us,
                                 robot_diagnostics_now_us());
#endif
        (void)tk_can_wup(TSK_SELF);
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
#if APP_IR_DEBUG && defined(TM_MQTT) && TM_MQTT
unsigned int mqtt_utk_control_debug_take(unsigned char *bytes, unsigned int size)
{
    unsigned int count=0;
    if (!bytes || size<CONTROL_DEBUG_PACKET_SIZE || g_mutex<=0 ||
        tk_loc_mtx(g_mutex,TMO_POL)!=E_OK) return 0;
    if (g_control_ready)
    {
        const uint32_t timestamp=(uint32_t)g_control_debug.bytes[1] |
            (uint32_t)g_control_debug.bytes[2]<<8 | (uint32_t)g_control_debug.bytes[3]<<16 |
            (uint32_t)g_control_debug.bytes[4]<<24;
        if ((uint32_t)(now_us()-timestamp)<=400000U)
        {
            for (unsigned i=0;i<CONTROL_DEBUG_PACKET_SIZE;++i) bytes[i]=g_control_debug.bytes[i];
            count=CONTROL_DEBUG_PACKET_SIZE;
        }
        g_control_ready=false;
    }
    (void)tk_unl_mtx(g_mutex);
    return count;
}
/* Called only by the network owner, with a nonblocking mutex attempt. */
unsigned int mqtt_utk_ir_debug_take(unsigned char *bytes, unsigned int size)
{
    unsigned int count = 0U;
    if (!bytes || size < IR_DEBUG_PACKET_SIZE || g_mutex <= 0 ||
        tk_loc_mtx(g_mutex, TMO_POL) != E_OK) return 0U;
    if (g_ir_ready)
    {
        const uint32_t timestamp = (uint32_t)g_ir_debug.bytes[1] |
            (uint32_t)g_ir_debug.bytes[2] << 8 |
            (uint32_t)g_ir_debug.bytes[3] << 16 |
            (uint32_t)g_ir_debug.bytes[4] << 24;
        if ((uint32_t)(now_us() - timestamp) <= 100000U)
        {
            for (unsigned i = 0U; i < IR_DEBUG_PACKET_SIZE; ++i) bytes[i] = g_ir_debug.bytes[i];
            count = IR_DEBUG_PACKET_SIZE;
        }
        g_ir_ready = false;
    }
    (void)tk_unl_mtx(g_mutex);
    return count;
}
#endif
#if S3_BARCODE_CAPTURE
#if defined(TM_MQTT) && TM_MQTT
#include "mqtt_utk.h"
/* Network task: next binary capture packet; never wait on the control owner. */
unsigned int mqtt_utk_debug_take(unsigned char *bytes, unsigned int size)
{
    unsigned int n = 0U;
    if (!bytes || size<BCAP_WIRE_MAX || !g_bcap_net_pending || (g_mutex <= 0))
    {
        return 0U;
    }
    if (tk_loc_mtx(g_mutex, TMO_POL) != E_OK)
    {
        return 0U;
    }
    if (g_bcap_net_pending)
    {
        n = bcap_binary_next(&g_bcap, &g_bcap_net_cursor, bytes, size);
        if (!n) { g_bcap_net_pending = false; }
    }
    (void)tk_unl_mtx(g_mutex);
    return n;
}
#endif
/* After a run, print the barcode trace over USB, a bounded number of lines
 * per report period. Restarts whenever a USB terminal is (re)opened, so the
 * car can run untethered and be plugged in afterwards. */
static void report_barcode_capture(bool running)
{
    static bool was_running, was_open, dumping;
    static bcap_cursor_t cursor;
    static char line[200];
    const bool open = BCAP_TERMINAL_OPEN();
    unsigned int lines;
    if (running)
    {
        dumping = false;
    }
    else if ((was_running || (open && !was_open)) && g_bcap_valid)
    {
        dumping = true;
        cursor = (bcap_cursor_t){0};
        tm_printf((UB *)"[BCAP] barcode sensor trace follows (copy from here "
                        "to [BCAP] end)\n");
    }
    was_running = running;
    was_open = open;
    for (lines = 0U; dumping && open && (lines < 40U); lines++)
    {
        bool more = false;
        if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
        {
            more = bcap_format_next(&g_bcap, &cursor, line, sizeof line);
            (void)tk_unl_mtx(g_mutex);
        }
        if (!more)
        {
            dumping = false;
            break;
        }
        tm_printf((UB *)"%s\n", line);
    }
}
#endif
static void report(INT code, void *unused)
{
    static s3_integration_status_t s;
    (void)code;
    (void)unused;
    for (;;)
    {
        /* 1 s, not 200 ms: keeps operator prompts readable on USB. */
        (void)tk_dly_tsk(1000U);
        if (!s3_integration_status(&s))
        {
            continue;
        }
#if APP_S5
        s5_observation_t obstacle;
        if (subsystem5_snapshot(&obstacle)) {
            unsigned status=subsystem5_controller_status();
            tm_printf((UB *)"[S5] state=%u fault=%u route=%u range_mm=%u valid=%u no_echo=%u scan=%u profile=%u width_mm=%u left_mm=%u right_mm=%u\n",
                status&7U,(status>>3)&7U,status>>6,(unsigned)obstacle.range_mm,
                (unsigned)obstacle.range_valid,(unsigned)obstacle.no_echo,(unsigned)obstacle.scan_id,
                (unsigned)obstacle.profile.valid,(unsigned)obstacle.profile.width_mm,
                (unsigned)obstacle.profile.left_clear_mm,(unsigned)obstacle.profile.right_clear_mm);
        }
#endif
#if APP_S4
        terrain_result_t terrain;
        uint16_t terrain_run, terrain_humps;
        if (subsystem4_read(&terrain, &terrain_run, &terrain_humps)) {
            tm_printf((UB *)"[S4] run=%u state=%u flags=%u pitch_x10=%d roll_x10=%d turn_dps=%d humps=%u peak_mm=%u impact=%u\n",
                (unsigned)terrain_run, (unsigned)terrain.state, (unsigned)terrain.flags,
                (int)(terrain.pitch_deg*10), (int)(terrain.roll_deg*10), (int)terrain.turn_dps,
                (unsigned)terrain_humps, (unsigned)(terrain.peak_m*1000), (unsigned)terrain.impact);
        } else { tm_printf((UB *)"[S4] waiting for sample/reset acknowledgement\n"); }
#endif
        tm_printf(
            (UB *)"[S3+S2] state=%u reason=%u ready=0x%x cal=0x%x motion=%s "
                  "odom_valid=%u adc=%d gap=%u boot_max=%u above10ms=%u "
                  "target_ms=%u\n",
            (unsigned)s.state, (unsigned)s.reason,
            (unsigned)s.navigation.readiness_blockers,
            (unsigned)s.navigation.accepted_calibration,
            motion_state_name(s.motion.state), (unsigned)s.motion.valid,
            (int)s.adc_status, (unsigned)s.navigation.sample_gap_us,
            (unsigned)s.max_gap_us, (unsigned)s.above_target,
            (unsigned)S3_SAMPLE_PERIOD_MS);
        tm_printf((UB *)"  RUN timing: start_us=%u active=%u samples=%u "
                        "max_gap_us=%u above10ms=%u over30ms=%u "
                        "last_over30_us=%u\n",
                  (unsigned)s.run_timing.started_us,
                  (unsigned)drive_running(s.state),
                  (unsigned)s.run_timing.samples,
                  (unsigned)s.run_timing.max_gap_us,
                  (unsigned)s.run_timing.above_target,
                  (unsigned)s.run_timing.above_safety,
                  (unsigned)s.run_timing.last_safety_us);
        tm_printf(
            (UB *)"  motion_id=%u stopped=%u odom_epoch=%u report_losses=%u "
                  "motion_fault=%s\n",
            (unsigned)s.motion.command_id, (unsigned)s.motion.braked,
            (unsigned)s.motion.epoch, (unsigned)s.motion.report_loss_count,
            motion_fault_name(s.motion.fault));
        tm_printf((UB *)"  command=%u ok=%u collecting=%u sensor=%u white n=%u "
                        "[%u,%u] black n=%u [%u,%u]\n",
                  (unsigned)s.command_sequence, (unsigned)s.command_ok,
                  (unsigned)s.collecting, (unsigned)s.calibration_sensor,
                  (unsigned)s.white.count, (unsigned)s.white.minimum,
                  (unsigned)s.white.maximum, (unsigned)s.black.count,
                  (unsigned)s.black.minimum, (unsigned)s.black.maximum);
        /* Live calibrated readings: 0 = white, 1000 = black. Meaningful
         * only after that sensor's calibration has been accepted. */
        tm_printf((UB *)"  IR dark: L(GP%u)=%u R(GP%u)=%u BAR(GP%u)=%u | "
                        "line present=%u error=%d junction=%u\n",
                  S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_LEFT),
                  (unsigned)s.navigation.sensor[S3_SENSOR_LEFT].darkness,
                  S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_RIGHT),
                  (unsigned)s.navigation.sensor[S3_SENSOR_RIGHT].darkness,
                  S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_BARCODE),
                  (unsigned)s.navigation.sensor[S3_SENSOR_BARCODE].darkness,
                  (unsigned)s.navigation.line.present,
                  (int)s.navigation.line.error,
                  (unsigned)s.navigation.line.junction_candidate);
#if S3_BARCODE_CAPTURE
        {
            uint16_t decoded = 0U, rejected = 0U, timing = 0U, overflow = 0U;
            uint8_t episodes = 0U;
            char letters[sizeof g_bcap.letters] = "-";
            if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
            {
                decoded = g_bcap.decoded;
                rejected = g_bcap.rejected;
                timing = g_bcap.timing;
                overflow = g_bcap.overflow;
                episodes = g_bcap.episodes;
                if (g_bcap.letter_count != 0U)
                {
                    for (unsigned int i = 0U; i < sizeof letters; i++)
                    {
                        letters[i] = g_bcap.letters[i];
                    }
                }
                (void)tk_unl_mtx(g_mutex);
            }
            tm_printf((UB *)"  BARCODE this run: decoded=%u letters=%s rejected=%u "
                            "timing_error=%u overflow=%u dark_episodes=%u\n",
                      (unsigned)decoded, letters, (unsigned)rejected,
                      (unsigned)timing, (unsigned)overflow, (unsigned)episodes);
        }
        report_barcode_capture(drive_running(s.state));
#endif
    }
}
static void cleanup(void)
{
    if (g_cycle > 0)
    {
        (void)tk_stp_cyc(g_cycle);
    }
    if (g_task > 0)
    {
        (void)tk_ter_tsk(g_task);
        (void)tk_del_tsk(g_task);
    }
    if (g_report > 0)
    {
        (void)tk_ter_tsk(g_report);
        (void)tk_del_tsk(g_report);
    }
    if (g_cycle > 0)
    {
        (void)tk_del_cyc(g_cycle);
    }
    if (g_mutex > 0)
    {
        (void)tk_del_mtx(g_mutex);
    }
    g_task = g_report = g_cycle = g_mutex = 0;
}
/* Minimum forward wheel speed while line following. On the car (2026-10-26)
 * a strongly-off-centre start made Buddy 3's steering ask the inner wheel for
 * ~0.04 m/s; that wheel never started turning and Buddy 2's stall check
 * (0.6 s without encoder progress) faulted the run. The motor smoke test
 * moved reliably at 0.05 m/s. Raise only small positive requests; zero and
 * reverse requests pass through unchanged, and Buddy 3's steering algorithm
 * itself is untouched. Tune on the car; set to 0 to disable. */
#define S3_MIN_WHEEL_MPS (0.01f)
static float floor_wheel(float mps)
{
    return ((mps > 0.0f) && (mps < S3_MIN_WHEEL_MPS)) ? S3_MIN_WHEEL_MPS
                                                      : mps;
}
static motion_result_t velocity_with_floor(float left_mps, float right_mps)
{
    return motion_request_velocity(floor_wheel(left_mps),
                                   floor_wheel(right_mps));
}
#if S3_BUILTIN_CALIBRATION
/* Feed the measured ranges through the SAME validation as a button
 * calibration (sample count, non-overlap, reflectance init). Returns the
 * mask of sensors accepted; any sensor that fails still needs the buttons. */
static uint32_t apply_builtin_calibration(void)
{
    static const uint16_t table[S3_SENSOR_COUNT][4] = {
        S3_BUILTIN_CAL_BARCODE, S3_BUILTIN_CAL_LEFT, S3_BUILTIN_CAL_RIGHT};
    unsigned int i;
    for (i = 0U; i < S3_SENSOR_COUNT; i++)
    {
        s3_calibration_collector_t c;
        s3_calibration_begin(&c);
        (void)s3_calibration_add(&c, IR_SURFACE_WHITE, table[i][0]);
        (void)s3_calibration_add(&c, IR_SURFACE_BLACK, table[i][2]);
        for (unsigned int k = 0U; k < 10U; k++)
        {
            (void)s3_calibration_add(&c, IR_SURFACE_WHITE,
                                     (uint16_t)((table[i][0] + table[i][1]) / 2U));
            (void)s3_calibration_add(&c, IR_SURFACE_BLACK,
                                     (uint16_t)((table[i][2] + table[i][3]) / 2U));
        }
        (void)s3_calibration_add(&c, IR_SURFACE_WHITE, table[i][1]);
        (void)s3_calibration_add(&c, IR_SURFACE_BLACK, table[i][3]);
        (void)s3_navigation_apply_calibration(&g_drive.navigation,
                                              (s3_sensor_id_t)i, &c);
    }
    return g_drive.navigation.accepted_calibration;
}
#endif
int32_t s3_integration_init(void)
{
    T_CMTX mutex = {0};
    T_CCYC cycle = {0};
    s3_navigation_config_t cfg = s3_navigation_defaults();
    const s3_drive_port_t port = {
        velocity_with_floor,        motion_request_manoeuvre, stop,
        motion_get_report,          motion_command_id,        motion_reset,
        motion_navigation_heartbeat};
    const s3_drive_options_t options = s3_course_options();
    if (g_mutex > 0)
    {
        return E_OBJ;
    }
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    s3_course_navigation(&cfg);
    if (!s3_drive_init(&g_drive, &cfg, &port) ||
        !s3_drive_set_options(&g_drive, &options))
    {
        return E_PAR;
    }
#if S3_BUILTIN_CALIBRATION
    (void)apply_builtin_calibration();
#endif
    mutex.mtxatr = TA_INHERIT;
    g_mutex = tk_cre_mtx(&mutex);
    if (g_mutex < E_OK)
    {
        return g_mutex;
    }
    cycle.cycatr = TA_HLNG;
    cycle.cychdr = (FP)tick;
    cycle.cyctim = S3_SAMPLE_PERIOD_MS;
    cycle.cycphs = S3_SAMPLE_PERIOD_MS;
    g_cycle = tk_cre_cyc(&cycle);
    if (g_cycle < E_OK)
    {
        int32_t er = g_cycle;
        cleanup();
        return er;
    }
    return E_OK;
}
int32_t s3_integration_start(void)
{
    ER er;
    if ((g_mutex <= 0) || (g_cycle <= 0) || (g_task > 0))
    {
        return E_OBJ;
    }
    g_task = app_start_task((FP)sample, S3_PRI_MAIN, TP_PRC1, "s3-motion");
    if (g_task < E_OK)
    {
        er = g_task;
        cleanup();
        return er;
    }
    g_report = app_start_task((FP)report, S3_PRI_REPORT, TP_PRC1, "s3-status");
    if (g_report < E_OK)
    {
        er = g_report;
        cleanup();
        return er;
    }
    er = tk_sta_cyc(g_cycle);
    if (er < E_OK)
    {
        cleanup();
    }
    return er;
}
#else
void s3_remote_link(uint32_t epoch,bool up) { (void)epoch; (void)up; }
void s3_remote_receive(const uint8_t *p,unsigned n) { (void)p; (void)n; }
unsigned s3_remote_reply_take(uint8_t *p,unsigned n) { (void)p; (void)n; return 0; }
unsigned s3_remote_status_take(uint8_t *p,unsigned n) { (void)p; (void)n; return 0; }
uint32_t s3_remote_stop_sequence(void) { return 0; }
void s3_remote_physical_stop(bool held) { (void)held; }
bool s3_integration_submit(const s3_control_request_t *request)
{
    (void)request;
    return false;
}
bool s3_integration_status(s3_integration_status_t *status)
{
    (void)status;
    return false;
}
int32_t s3_integration_init(void) { return E_NOSPT; }
int32_t s3_integration_start(void) { return E_NOSPT; }
#endif
