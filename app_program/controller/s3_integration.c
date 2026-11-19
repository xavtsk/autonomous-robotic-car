/* Opt-in runtime. One sampling owner serialises all navigation/control calls.
 */
#include "s3_integration.h"
#include "../app_types.h"
#include "vehicle_controller.h"
#include "integration_codec.h"
#include "../diagnostics/robot_diagnostics.h"
#include "../subsystem3/ir_sensor.h"
#include "../subsystem3/buddy3.h"
#include "builtin_calibration.h"
#include "course_profile.h"
#include "bc_capture.h"
#include "ir_debug.h"
#include "control_debug.h"
#include "terr_link.h"
#include "remote_control.h"
#if APP_S4
#include "../subsystem4/buddy4.h"
#endif
#if APP_S5
#include "../subsystem5/s5_api.h"
#endif
#include <sys/sysdef.h>
#if APP_S3_MOTION
#ifndef S3_BARCODE_CAPTURE
#define S3_BARCODE_CAPTURE                                                     \
    1 /* barcode-sensor trace, printed over USB after a run */
#endif
#if S3_BARCODE_CAPTURE
static bcap_t
    g_bcap; /* written by the sampling owner, read by report; g_mutex */
static bool g_bcap_valid;
/* Wi-Fi copy of the trace: sent in binary packets on MQTT_UTK_DEBUG_TOPIC after
 * a run ends, sharing publish slots with routine telemetry. The flag is a cheap
 * hint read by the network task every poll; the cursor is used under g_mutex.
 */
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

static struct
{
    unsigned pending : 1, stop : 1, status_ready : 1, reset_pending : 1;
} g_shared;

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

static uint32_t now_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

/* g_mutex protects the bounded remote mailbox and correlated result ring. */
static struct
{
    uint32_t epoch, last_id, sampled_us, published_us, stop_sequence;
    unsigned flags : 4;
    unsigned head : 3;
    unsigned count : 3;
    bool connected : 1;
    bool published : 1;
    bool physical_stop : 1;
    bool inflight : 1;
    remote_request_t pending, reset;
    uint8_t reply[REMOTE_REPLY_RING_SIZE][REMOTE_REPLY_SIZE];
} g_remote;

static void remote_reply (const remote_request_t * p_r, uint8_t result,
                          uint8_t reason)
{
    if (!p_r->valid)
    {
        return;
    }
    if (g_remote.count == REMOTE_REPLY_RING_SIZE)
    {
        g_remote.head = (g_remote.head + 1U) % REMOTE_REPLY_RING_SIZE;
        --g_remote.count;
    }
    uint8_t * p_p =
        g_remote
            .reply[(g_remote.head + g_remote.count) % REMOTE_REPLY_RING_SIZE];
    p_p[0] = REMOTE_REPLY_VERSION;
    /* Internal enum values occupy 2 + 2 + 3 bits, so this cast cannot narrow
     * a valid value. Preserve all 32 request-ID and epoch bits unchanged. */
    p_p[1] = (uint8_t)((uint32_t)p_r->action |
                       ((uint32_t)result << REMOTE_REPLY_ACTION_BITS) |
                       ((uint32_t)reason << (REMOTE_REPLY_ACTION_BITS +
                                             REMOTE_REPLY_RESULT_BITS)));
    remote_put32(p_p + REMOTE_REPLY_ID_OFFSET, p_r->id);
    remote_put32(p_p + REMOTE_REPLY_EPOCH_OFFSET,
                 p_r->action == REMOTE_STOP ? g_remote.epoch : p_r->epoch);
    ++g_remote.count;
}

static void remote_cancel (remote_request_t * p_r)
{
    remote_reply(p_r, REMOTE_CANCELLED, REMOTE_SUPERSEDED);
    p_r->valid = false;
}

static void remote_finish (const remote_request_t * p_r, bool value_ok,
                           uint8_t reason)
{
    if (!p_r->valid || tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK)
    {
        return;
    }
    remote_reply(p_r, value_ok ? REMOTE_EXECUTED : REMOTE_REJECTED,
                 value_ok ? REMOTE_NONE : reason);
    g_remote.inflight = false;
    (void)tk_unl_mtx(g_mutex);
}

void s3_remote_link (uint32_t epoch, bool value_up)
{
    if (g_mutex <= 0 || tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK)
    {
        return;
    }
    if (g_remote.pending.valid && g_remote.pending.action != REMOTE_STOP)
    {
        remote_cancel(&g_remote.pending);
        g_shared.pending = false;
        g_shared.reset_pending = false;
    }
    remote_cancel(&g_remote.reset); /* owner finishes safe reset housekeeping */
    g_remote.connected = value_up;
    if (value_up)
    {
        g_remote.epoch = epoch ? epoch : 1U;
        g_remote.last_id = 0U;
    }
    g_remote.published = false;
    (void)tk_unl_mtx(g_mutex);
}

void s3_remote_receive (const uint8_t * p_p, unsigned length)
{
    if (!p_p || length != REMOTE_COMMAND_SIZE || p_p[0] != REMOTE_VERSION ||
        p_p[1] < REMOTE_START || p_p[1] > REMOTE_RESET || g_mutex <= 0)
    {
        return;
    }
    const remote_request_t value_r = {.action = p_p[1],
                                      .id = remote_get32(p_p + 2),
                                      .epoch = remote_get32(p_p + 6),
                                      .time_us = remote_get32(p_p + 10),
                                      .valid = true};
    if (tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK)
    {
        return;
    }
    uint8_t reason = REMOTE_NONE;
    const bool fresh =
        g_remote.connected && value_r.epoch == g_remote.epoch &&
        (uint32_t)(now_us() - value_r.time_us) <= REMOTE_MAX_AGE_US;
    if (value_r.action != REMOTE_STOP && !fresh)
    {
        reason = REMOTE_STALE;
    }
    else if (value_r.action != REMOTE_STOP &&
             !remote_newer(value_r.id, g_remote.last_id))
    {
        reason = REMOTE_REPLAY;
    }
    else if (value_r.action != REMOTE_STOP && g_remote.physical_stop)
    {
        reason = REMOTE_NOT_READY;
    }
    else if (value_r.action != REMOTE_STOP &&
             (g_shared.pending || g_shared.stop || g_shared.reset_pending ||
              g_remote.inflight))
    {
        reason = REMOTE_BUSY;
    }
    /* Newer rejected IDs are consumed, too: no older START after a newer STOP.
     */
    if (value_r.epoch == g_remote.epoch &&
        remote_newer(value_r.id, g_remote.last_id))
    {
        g_remote.last_id = value_r.id;
    }
    if (reason != REMOTE_NONE)
    {
        remote_reply(&value_r, REMOTE_REJECTED, reason);
    }
    else
    {
        const s3_control_request_t value_c = {
            .kind = value_r.action == REMOTE_START  ? S3_CONTROL_START
                    : value_r.action == REMOTE_STOP ? S3_CONTROL_STOP
                                                    : S3_CONTROL_RESET};
        if (value_r.action == REMOTE_STOP)
        {
            ++g_remote.stop_sequence;
            remote_cancel(&g_remote.pending);
            remote_cancel(&g_remote.reset);
            g_shared.stop = true;
            g_shared.pending = false;
            g_shared.reset_pending = false;
        }
        else
        {
            (void)s3_control_pack(&value_c, &g_command);
            g_shared.pending = true;
            g_shared.reset_pending = value_r.action == REMOTE_RESET;
        }
        g_remote.pending = value_r;
        remote_reply(&value_r, REMOTE_QUEUED, REMOTE_NONE);
    }
    (void)tk_unl_mtx(g_mutex);
}

unsigned s3_remote_reply_take (uint8_t * p_p, unsigned capacity)
{
    unsigned value_n = 0;
    if (!p_p || capacity < REMOTE_REPLY_SIZE || g_mutex <= 0 ||
        tk_loc_mtx(g_mutex, TMO_POL) != E_OK)
    {
        return 0;
    }
    if (g_remote.count)
    {
        for (unsigned value_i = 0; value_i < REMOTE_REPLY_SIZE; value_i++)
        {
            p_p[value_i] = g_remote.reply[g_remote.head][value_i];
        }
        g_remote.head = (g_remote.head + 1U) % REMOTE_REPLY_RING_SIZE;
        --g_remote.count;
        value_n = REMOTE_REPLY_SIZE;
    }
    (void)tk_unl_mtx(g_mutex);
    return value_n;
}

unsigned s3_remote_status_take (uint8_t * p_p, unsigned capacity)
{
    unsigned value_n = 0;
    if (!p_p || capacity < REMOTE_STATUS_SIZE || g_mutex <= 0 ||
        tk_loc_mtx(g_mutex, TMO_POL) != E_OK)
    {
        return 0;
    }
    const uint32_t now = now_us();
    if (g_remote.connected &&
        (!g_remote.published ||
         (uint32_t)(now - g_remote.published_us) >= 500000U))
    {
        p_p[0] = REMOTE_VERSION;
        remote_put32(p_p + 1, g_remote.epoch);
        remote_put32(p_p + 5, now);
        remote_put32(p_p + 9, g_remote.last_id);
        p_p[13] = (uint32_t)(now - g_remote.sampled_us) <= 200000U
                      ? g_remote.flags
                      : 0U;
        g_remote.published_us = now;
        g_remote.published = true;
        value_n = REMOTE_STATUS_SIZE;
    }
    (void)tk_unl_mtx(g_mutex);
    return value_n;
}

uint32_t s3_remote_stop_sequence (void)
{
    uint32_t sequence = 0;
    if (g_mutex > 0 && tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
    {
        sequence = g_remote.stop_sequence;
        (void)tk_unl_mtx(g_mutex);
    }
    return sequence;
}

void s3_remote_physical_stop (bool held)
{
    if (g_mutex > 0 && tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
    {
        g_remote.physical_stop = held;
        (void)tk_unl_mtx(g_mutex);
    }
}

static bool remote_permitted (const remote_request_t * p_r)
{
    bool permitted = false;
    if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
    {
        permitted = g_remote.connected && p_r->epoch == g_remote.epoch &&
                    !g_shared.stop && !g_remote.physical_stop &&
                    p_r->id == g_remote.last_id &&
                    (uint32_t)(now_us() - p_r->time_us) <= REMOTE_MAX_AGE_US;
        (void)tk_unl_mtx(g_mutex);
    }
    return permitted;
}

bool s3_integration_submit (const s3_control_request_t * p_request)
{
    bool accepted = false;
    s3_control_packet_t packet;
    if ((p_request == 0) || (g_mutex <= 0) ||
        ((unsigned int)p_request->kind > S3_CONTROL_RESET) ||
        !s3_control_pack(p_request, &packet))
    {
        return false;
    }
    if (tk_loc_mtx(g_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }
    if (p_request->kind == S3_CONTROL_STOP)
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
        g_shared.reset_pending = p_request->kind == S3_CONTROL_RESET;
        accepted = true;
    }
    (void)tk_unl_mtx(g_mutex);
    return accepted;
}

bool s3_integration_status (s3_integration_status_t * p_status)
{
    s3_status_packet_t packet;
    bool ready;
    if ((p_status == 0) || (g_mutex <= 0))
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
    return ready && s3_status_unpack(&packet, p_status);
}

static bool execute (const s3_control_request_t * p_c, uint32_t now)
{
    if (p_c->kind == S3_CONTROL_STOP)
    {
        s3_drive_stop(&g_drive);
        g_collecting = false;
        return true;
    }
    if (p_c->kind == S3_CONTROL_RESET)
    {
        return terrain_control_reset_ready(now) &&
               s3_drive_reset(&g_drive, now);
    }
    if (p_c->kind == S3_CONTROL_START)
    {
        /* Ineligible STARTs (already driving, FAULT, active lifecycle,
         * collecting, not ready) are refused BEFORE touching the map. */
        if (!terrain_control_ready(now) || g_collecting ||
            ((g_drive.state != S3_DRIVE_OFF) &&
             (g_drive.state != S3_DRIVE_HALTED)) ||
            (g_drive.navigation.lifecycle != S3_RUN_INACTIVE) ||
            (s3_navigation_readiness(&g_drive.navigation, now) != 0U) ||
            !vehicle_controller_prep_start(&g_drive.navigation))
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
        if (!terrain_control_begin(now))
        {
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
    if (p_c->kind == S3_CONTROL_MAPPING)
    {
        return vehicle_controller_set_mapping(p_c->map) &&
               s3_navigation_set_mapping(&g_drive.navigation, p_c->map);
    }
    if ((unsigned int)p_c->sensor >= S3_SENSOR_COUNT)
    {
        return false;
    }
    if (p_c->kind == S3_CONTROL_APPLY_CALIBRATION)
    {
        return s3_navigation_apply_calibration(&g_drive.navigation, p_c->sensor,
                                               &g_collector[p_c->sensor]);
    }
    g_sensor = p_c->sensor;
    g_collecting = true;
    g_collect_since = now;
    g_label =
        p_c->kind == S3_CONTROL_WHITE ? IR_SURFACE_WHITE : IR_SURFACE_BLACK;
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

static void tick (void * p_unused)
{
    (void)p_unused;
    if (g_task > 0)
    {
        (void)tk_wup_tsk(g_task);
    }
}

static bool drive_running (s3_drive_state_t state)
{
    return (state >= S3_DRIVE_FOLLOW) && (state <= S3_DRIVE_REACQUIRE);
}

static void reset_completed (void)
{
    if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
    {
        g_shared.reset_pending = false;
        (void)tk_unl_mtx(g_mutex);
    }
}
#if APP_IR_DEBUG
static bool debug_scaled (float32_t value, float32_t scale, int32_t limit,
                          int32_t * p_out)
{
    const float32_t value_v = value * scale;
    /* Ordered comparisons also reject NaN/infinity, without newlib headers
     * (whose size_t conflicts with this kernel's freestanding types). */
    if (!(value_v >= -(float32_t)limit && value_v <= (float32_t)limit))
    {
        return false;
    }
    *p_out = (int32_t)(value_v >= 0.0f ? value_v + 0.5f : value_v - 0.5f);
    return true;
}

/** Convert with the original rounding/range helper before field encoding.
 * The sampling owner alone uses the builder. A failed conversion leaves its
 * target field untouched, preserving the original short-circuit sequence. */
static bool debug_builder_scaled (control_debug_builder_t * p_builder,
                                  control_debug_field_t field, float32_t value,
                                  float32_t scale, int32_t limit)
{
    int32_t converted = 0;
    if (!debug_scaled(value, scale, limit, &converted))
    {
        return false;
    }
    control_debug_builder_s(p_builder, field, converted);
    return true;
}

static bool capture_control_debug (const s3_integration_status_t * p_st,
                                   control_debug_packet_t * p_packet)
{
    /* Owned by the sampling task. Static scratch avoids growing its stack. */
    static control_debug_builder_t builder;
    bool motion_valid = false;
    static motion_debug_snapshot_t value_m;
    const uint32_t now = p_st->navigation.timestamp_us;
    static const uint32_t event_bits[3] = {S3_EVENT_BARCODE, S3_EVENT_JUNCTION,
                                           S3_EVENT_LINE_LOST};
    for (unsigned value_i = 0; value_i < 3; ++value_i)
    {
        if ((p_st->navigation.events & event_bits[value_i]) &&
            g_debug_events[value_i] != UINT16_MAX)
        {
            ++g_debug_events[value_i];
        }
    }
    if (p_st->navigation.barcode_status != S3_BARCODE_NONE)
    {
        g_last_barcode_status = (uint8_t)p_st->navigation.barcode_status;
    }
    if (g_control_seen && (uint32_t)(now - g_control_last) < 200000U)
    {
        return false;
    }
    g_control_seen = true;
    g_control_last = now;
    control_debug_builder_init(&builder);
    control_debug_builder_u(&builder, CD_TIMESTAMP_US, now);
    control_debug_builder_u(&builder, CD_SEQUENCE, g_control_sequence++);
    if (motion_try_debug_snapshot(&value_m))
    {
        control_debug_builder_u(&builder, CD_MOTION_AVAILABLE, 1);
        control_debug_builder_u(&builder, CD_MOTION_TIMESTAMP_US,
                                value_m.timestamp_us);
        control_debug_builder_u(&builder, CD_MOTION_STATE, value_m.state);
        control_debug_builder_u(&builder, CD_MOTION_FAULT, value_m.fault);
        control_debug_builder_u(&builder, CD_BRAKED, value_m.braked);
        control_debug_builder_u(&builder, CD_BUSY, value_m.busy);
        /* Motion can preempt this task after the ADC frame was timestamped.
         * Judge freshness against the current clock, not that older frame. */
        motion_valid = value_m.valid &&
                       (uint32_t)(now_us() - value_m.timestamp_us) <= 50000U;
        motion_valid =
            motion_valid &&
            debug_builder_scaled(&builder, CD_SETPOINT_LEFT,
                                 value_m.setpoint_mps[0], 1000, 4095) &&
            debug_builder_scaled(&builder, CD_SETPOINT_RIGHT,
                                 value_m.setpoint_mps[1], 1000, 4095) &&
            debug_builder_scaled(&builder, CD_MEASURED_LEFT,
                                 value_m.measured_mps[0], 1000, 4095) &&
            debug_builder_scaled(&builder, CD_MEASURED_RIGHT,
                                 value_m.measured_mps[1], 1000, 4095) &&
            debug_builder_scaled(&builder, CD_DUTY_LEFT, value_m.duty[0], 100,
                                 100) &&
            debug_builder_scaled(&builder, CD_DUTY_RIGHT, value_m.duty[1], 100,
                                 100) &&
            value_m.encoder[0] >= -524288 && value_m.encoder[0] <= 524287 &&
            value_m.encoder[1] >= -524288 && value_m.encoder[1] <= 524287;
        if (motion_valid)
        {
            control_debug_builder_s(&builder, CD_ENCODER_LEFT,
                                    value_m.encoder[0]);
            control_debug_builder_s(&builder, CD_ENCODER_RIGHT,
                                    value_m.encoder[1]);
        }
        else
        {
            control_debug_builder_s(&builder, CD_SETPOINT_LEFT, 0);
            control_debug_builder_s(&builder, CD_SETPOINT_RIGHT, 0);
            control_debug_builder_s(&builder, CD_MEASURED_LEFT, 0);
            control_debug_builder_s(&builder, CD_MEASURED_RIGHT, 0);
            control_debug_builder_s(&builder, CD_DUTY_LEFT, 0);
            control_debug_builder_s(&builder, CD_DUTY_RIGHT, 0);
        }
    }
    control_debug_builder_u(&builder, CD_DRIVE_STATE, p_st->state);
    control_debug_builder_u(&builder, CD_REASON, p_st->reason);
    control_debug_builder_u(&builder, CD_PENDING, g_drive.pending);
    control_debug_builder_s(
        &builder, CD_LINE_ERROR,
        p_st->navigation.line.valid ? p_st->navigation.line.error : 0);
    control_debug_builder_u(&builder, CD_LINE_FLAGS,
                            p_st->navigation.line.valid |
                                p_st->navigation.line.present << 1 |
                                p_st->navigation.line.junction_candidate << 2 |
                                p_st->navigation.line.touching << 3 |
                                p_st->navigation.rearming << 4);
    control_debug_builder_u(&builder, CD_BARCODE_STATUS, g_last_barcode_status);
    control_debug_builder_u(&builder, CD_ACCEPTED_CALIBRATION,
                            p_st->navigation.accepted_calibration);
    control_debug_builder_u(&builder, CD_READINESS,
                            p_st->navigation.readiness_blockers);
    control_debug_builder_u(&builder, CD_BARCODE_EVENTS, g_debug_events[0]);
    control_debug_builder_u(&builder, CD_JUNCTION_EVENTS, g_debug_events[1]);
    control_debug_builder_u(&builder, CD_LOST_EVENTS, g_debug_events[2]);
    control_debug_builder_u(&builder, CD_SAMPLE_GAP_US,
                            p_st->navigation.sample_gap_us);
    control_debug_builder_u(&builder, CD_RUN_MAX_GAP_US,
                            p_st->run_timing.max_gap_us);
    control_debug_builder_u(
        &builder, CD_LEFT_WHITE,
        g_drive.navigation.config.calibration[S3_SENSOR_LEFT].white_raw);
    control_debug_builder_u(
        &builder, CD_LEFT_BLACK,
        g_drive.navigation.config.calibration[S3_SENSOR_LEFT].black_raw);
    control_debug_builder_u(
        &builder, CD_LEFT_WHITE_LIMIT,
        g_drive.navigation.config.calibration[S3_SENSOR_LEFT].white_limit);
    control_debug_builder_u(
        &builder, CD_LEFT_BLACK_LIMIT,
        g_drive.navigation.config.calibration[S3_SENSOR_LEFT].black_limit);
    control_debug_builder_u(
        &builder, CD_RIGHT_WHITE,
        g_drive.navigation.config.calibration[S3_SENSOR_RIGHT].white_raw);
    control_debug_builder_u(
        &builder, CD_RIGHT_BLACK,
        g_drive.navigation.config.calibration[S3_SENSOR_RIGHT].black_raw);
    control_debug_builder_u(
        &builder, CD_RIGHT_WHITE_LIMIT,
        g_drive.navigation.config.calibration[S3_SENSOR_RIGHT].white_limit);
    control_debug_builder_u(
        &builder, CD_RIGHT_BLACK_LIMIT,
        g_drive.navigation.config.calibration[S3_SENSOR_RIGHT].black_limit);
    control_debug_builder_u(
        &builder, CD_BARCODE_WHITE,
        g_drive.navigation.config.calibration[S3_SENSOR_BARCODE].white_raw);
    control_debug_builder_u(
        &builder, CD_BARCODE_BLACK,
        g_drive.navigation.config.calibration[S3_SENSOR_BARCODE].black_raw);
    control_debug_builder_u(
        &builder, CD_BARCODE_WHITE_LIMIT,
        g_drive.navigation.config.calibration[S3_SENSOR_BARCODE].white_limit);
    control_debug_builder_u(
        &builder, CD_BARCODE_BLACK_LIMIT,
        g_drive.navigation.config.calibration[S3_SENSOR_BARCODE].black_limit);
    control_debug_builder_u(&builder, CD_CAL_SENSOR, p_st->calibration_sensor);
    control_debug_builder_u(&builder, CD_COLLECTING, p_st->collecting);
    control_debug_builder_u(&builder, CD_WHITE_MIN,
                            p_st->white.count ? p_st->white.minimum : 0);
    control_debug_builder_u(&builder, CD_WHITE_MAX,
                            p_st->white.count ? p_st->white.maximum : 0);
    control_debug_builder_u(&builder, CD_BLACK_MIN,
                            p_st->black.count ? p_st->black.minimum : 0);
    control_debug_builder_u(&builder, CD_BLACK_MAX,
                            p_st->black.count ? p_st->black.maximum : 0);
    control_debug_builder_u(&builder, CD_WHITE_COUNT, p_st->white.count);
    control_debug_builder_u(&builder, CD_BLACK_COUNT, p_st->black.count);
    control_debug_builder_u(&builder, CD_MOTION_VALID, motion_valid);
    return control_debug_builder_end(&builder, p_packet);
}
#endif
static void sensor_sample_task (INT code, void * p_unused)
{
    static s3_sensor_frame_t frame;
    static motion_feedback_t feedback;
    static s3_control_request_t command;
    static s3_integration_status_t working;
    int32_t adc;
    (void)code;
    (void)p_unused;
    (void)tk_slp_tsk(TMO_FEVR);
    adc = ir_sensor_init(&g_drive.navigation.config.acquisition);
    for (;;)
    {
#if APP_DIAGNOSTICS
        uint32_t started_us = robot_diagnostics_now_us();
#endif
        bool pending = false;
        remote_request_t remote_command = {0};
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
            remote_command = g_remote.pending;
            g_remote.inflight = remote_command.valid;
            g_remote.pending.valid = false;
            g_shared.stop = false;
            g_shared.pending = false;
            (void)tk_unl_mtx(g_mutex);
        }
        if (pending && (command.kind == S3_CONTROL_STOP))
        {
            g_reset_waiting = false;
            g_reset_remote = false;
            const bool value_ok = execute(&command, now_us());
            if (remote_command.valid)
            {
                remote_finish(&remote_command, value_ok, REMOTE_NOT_READY);
            }
            else
            {
                working.command_ok = value_ok;
                working.command_sequence++;
            }
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
            const bool value_ok =
                s3_drive_reset_result(&g_drive) == S3_RESET_OK;
            if (value_ok)
            {
                terrain_control_clear_fault();
            }
            remote_request_t reset = {0};
            if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
            {
                reset = g_remote.reset;
                g_remote.reset.valid = false;
                (void)tk_unl_mtx(g_mutex);
            }
            if (g_reset_remote)
            {
                if (reset.valid)
                {
                    remote_finish(&reset, value_ok, REMOTE_NOT_READY);
                }
            }
            else
            {
                working.command_ok = value_ok;
                working.command_sequence++;
            }
            g_reset_remote = false;
            g_reset_waiting = false;
            reset_completed();
        }
#if S3_BARCODE_CAPTURE
        if (was_running)
        {
            const bcap_input_t value_in = {
                frame.timestamp_us,
                frame.sensor[S3_SENSOR_BARCODE].raw,
                g_drive.output.sensor[S3_SENSOR_BARCODE].darkness,
                (uint8_t)g_drive.output.sensor[S3_SENSOR_BARCODE].surface,
                (uint8_t)g_drive.output.barcode_status,
                g_drive.output.barcode.letter,
                (g_drive.navigation.barcode.collecting != 0U) ||
                    s3_barcode_record_active(
                        &g_drive.navigation.barcode_record),
                g_drive.navigation.rearming,
                feedback.valid ? feedback.centre_m : 0.0};
            if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
            {
                bcap_record(&g_bcap, &value_in);
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
            const s3_raw_sample_t * p_raw = &frame.sensor[g_sensor];
            if (p_raw->valid && (p_raw->error == 0))
            {
                (void)s3_calibration_add(&g_collector[g_sensor], g_label,
                                         p_raw->raw);
            }
            if ((uint32_t)(frame.timestamp_us - g_collect_since) >=
                S3_COLLECTION_US)
            {
                g_collecting = false;
            }
        }
        if (pending)
        {
            bool permitted =
                !remote_command.valid || remote_permitted(&remote_command);
            bool value_ok = permitted && execute(&command, now_us());
            /* START changes owner state/heartbeat only: velocity is sent on
             * the next sample. If STOP/reconnect arrived during that change,
             * cancel before publishing success or reaching a velocity tick. */
            if (value_ok && remote_command.valid &&
                !remote_permitted(&remote_command))
            {
                const s3_control_request_t stop_command = {.kind =
                                                               S3_CONTROL_STOP};
                (void)execute(&stop_command, now_us());
                value_ok = false;
                permitted = false;
            }
            if ((command.kind == S3_CONTROL_RESET) && value_ok &&
                (s3_drive_reset_result(&g_drive) == S3_RESET_PENDING))
            {
                g_reset_waiting = true;
                g_reset_remote = remote_command.valid;
                if (remote_command.valid &&
                    tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
                {
                    if (!g_shared.stop && !g_remote.physical_stop &&
                        g_remote.connected &&
                        remote_command.epoch == g_remote.epoch)
                    {
                        g_remote.reset = remote_command;
                    }
                    else
                    {
                        remote_reply(&remote_command, REMOTE_CANCELLED,
                                     REMOTE_SUPERSEDED);
                    }
                    g_remote.inflight = false;
                    (void)tk_unl_mtx(g_mutex);
                }
            }
            else
            {
                if (remote_command.valid)
                {
                    remote_finish(&remote_command, value_ok,
                                  permitted ? REMOTE_NOT_READY : REMOTE_STALE);
                }
                else
                {
                    working.command_ok = value_ok;
                    working.command_sequence++;
                }
                if (command.kind == S3_CONTROL_RESET)
                {
                    reset_completed();
                }
            }
            if ((command.kind == S3_CONTROL_START) && value_ok)
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
        if (!packed)
        {
            s3_drive_stop(&g_drive);
        }
#if APP_IR_DEBUG
        ir_debug_packet_t ir_packet;
        control_debug_packet_t control_packet;
        const bool control_packed =
            capture_control_debug(&working, &control_packet);
        bool ir_packed = false;
        if (ir_debug_due(frame.timestamp_us, &g_ir_last, &g_ir_seen))
        {
            static const s3_sensor_id_t order[3] = {
                S3_SENSOR_LEFT, S3_SENSOR_RIGHT, S3_SENSOR_BARCODE};
            ir_debug_sample_t preview = {0};
            preview.timestamp_us = frame.timestamp_us;
            preview.sequence = g_ir_sequence++;
            for (unsigned value_i = 0U; value_i < 3U; ++value_i)
            {
                const s3_sensor_id_t value_id = order[value_i];
                const s3_raw_sample_t * p_raw = &frame.sensor[value_id];
                if (p_raw->enabled && p_raw->valid && p_raw->error == 0)
                {
                    preview.raw_valid |= (uint8_t)(1U << value_i);
                    preview.raw[value_i] = p_raw->raw;
                    if (working.navigation.sensor[value_id].valid)
                    {
                        preview.normalised_valid |= (uint8_t)(1U << value_i);
                        preview.darkness[value_i] =
                            working.navigation.sensor[value_id].darkness;
                        preview.surface[value_i] =
                            (uint8_t)working.navigation.sensor[value_id]
                                .surface;
                    }
                }
            }
            ir_packed = ir_debug_pack(&preview, &ir_packet);
        }
#endif
        uint8_t remote_flags = REMOTE_AVAILABLE;
        const uint32_t remote_now = now_us();
        if (drive_running(g_drive.state))
        {
            remote_flags |= REMOTE_RUNNING;
        }
        const bool motor_safe =
            feedback.valid && feedback.braked &&
            feedback.state == MOTION_IDLE &&
            feedback.fault == MOTION_FAULT_NONE &&
            (uint32_t)(remote_now - feedback.timestamp_us) <= 200000U;
        if (!g_collecting &&
            (g_drive.state == S3_DRIVE_OFF ||
             g_drive.state == S3_DRIVE_HALTED) &&
            g_drive.navigation.lifecycle == S3_RUN_INACTIVE &&
            s3_navigation_readiness(&g_drive.navigation, remote_now) == 0U &&
            terrain_control_ready(remote_now) && motor_safe)
        {
            remote_flags |= REMOTE_CAN_START;
        }
        if (g_drive.state == S3_DRIVE_FAULT &&
            terrain_control_reset_ready(remote_now))
        {
            remote_flags |= REMOTE_CAN_RESET;
        }
        if (tk_loc_mtx(g_mutex, TMO_FEVR) == E_OK)
        {
            g_remote.flags = remote_flags;
            g_remote.sampled_us = remote_now;
            if (g_shared.stop || g_shared.pending || g_shared.reset_pending ||
                g_remote.physical_stop)
            {
                g_remote.flags &=
                    (uint8_t)~(REMOTE_CAN_START | REMOTE_CAN_RESET);
            }
            if (packed)
            {
                g_status = packet;
            }
            g_shared.status_ready = packed;
#if APP_IR_DEBUG
            if (ir_packed)
            {
                g_ir_debug = ir_packet;
                g_ir_ready = true;
            }
            if (control_packed)
            {
                g_control_debug = control_packet;
                g_control_ready = true;
            }
#endif
            (void)tk_unl_mtx(g_mutex);
        }
        if (packed)
        {
            vehicle_controller_observe(&working);
        }
#if APP_DIAGNOSTICS
        robot_diagnostics_record(ROBOT_DIAG_IR, started_us,
                                 robot_diagnostics_now_us());
#endif
        (void)tk_can_wup(TSK_SELF);
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
#if APP_IR_DEBUG && defined(TM_MQTT) && TM_MQTT
unsigned int mqtt_utk_control_debug_take (unsigned char * p_bytes,
                                          unsigned int size)
{
    unsigned int count = 0;
    if (!p_bytes || size < CONTROL_DEBUG_PACKET_SIZE || g_mutex <= 0 ||
        tk_loc_mtx(g_mutex, TMO_POL) != E_OK)
    {
        return 0;
    }
    if (g_control_ready)
    {
        const uint32_t timestamp = (uint32_t)g_control_debug.bytes[1] |
                                   (uint32_t)g_control_debug.bytes[2] << 8 |
                                   (uint32_t)g_control_debug.bytes[3] << 16 |
                                   (uint32_t)g_control_debug.bytes[4] << 24;
        if ((uint32_t)(now_us() - timestamp) <= 400000U)
        {
            for (unsigned value_i = 0; value_i < CONTROL_DEBUG_PACKET_SIZE;
                 ++value_i)
            {
                p_bytes[value_i] = g_control_debug.bytes[value_i];
            }
            count = CONTROL_DEBUG_PACKET_SIZE;
        }
        g_control_ready = false;
    }
    (void)tk_unl_mtx(g_mutex);
    return count;
}

/* Called only by the network owner, with a nonblocking mutex attempt. */
unsigned int mqtt_utk_ir_debug_take (unsigned char * p_bytes, unsigned int size)
{
    unsigned int count = 0U;
    if (!p_bytes || size < IR_DEBUG_PACKET_SIZE || g_mutex <= 0 ||
        tk_loc_mtx(g_mutex, TMO_POL) != E_OK)
    {
        return 0U;
    }
    if (g_ir_ready)
    {
        const uint32_t timestamp = (uint32_t)g_ir_debug.bytes[1] |
                                   (uint32_t)g_ir_debug.bytes[2] << 8 |
                                   (uint32_t)g_ir_debug.bytes[3] << 16 |
                                   (uint32_t)g_ir_debug.bytes[4] << 24;
        if ((uint32_t)(now_us() - timestamp) <= 100000U)
        {
            for (unsigned value_i = 0U; value_i < IR_DEBUG_PACKET_SIZE;
                 ++value_i)
            {
                p_bytes[value_i] = g_ir_debug.bytes[value_i];
            }
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
unsigned int mqtt_utk_debug_take (unsigned char * p_bytes, unsigned int size)
{
    unsigned int value_n = 0U;
    if (!p_bytes || size < BCAP_WIRE_MAX || !g_bcap_net_pending ||
        (g_mutex <= 0))
    {
        return 0U;
    }
    if (tk_loc_mtx(g_mutex, TMO_POL) != E_OK)
    {
        return 0U;
    }
    if (g_bcap_net_pending)
    {
        value_n = bcap_binary_next(&g_bcap, &g_bcap_net_cursor, p_bytes, size);
        if (!value_n)
        {
            g_bcap_net_pending = false;
        }
    }
    (void)tk_unl_mtx(g_mutex);
    return value_n;
}
#endif
/* After a run, print the barcode trace over USB, a bounded number of lines
 * per report period. Restarts whenever a USB terminal is (re)opened, so the
 * car can run untethered and be plugged in afterwards. */
static void report_barcode_capture (bool running)
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
static void sensor_report_task (INT code, void * p_unused)
{
    static s3_integration_status_t value_s;
    (void)code;
    (void)p_unused;
    for (;;)
    {
        /* 1 s, not 200 ms: keeps operator prompts readable on USB. */
        (void)tk_dly_tsk(1000U);
        if (!s3_integration_status(&value_s))
        {
            continue;
        }
#if APP_S5
        s5_observation_t obstacle;
        if (subsystem5_snapshot(&obstacle))
        {
            unsigned status = subsystem5_controller_status();
            tm_printf((UB *)"[S5] state=%u fault=%u route=%u range_mm=%u "
                            "valid=%u no_echo=%u scan=%u profile=%u "
                            "width_mm=%u left_mm=%u right_mm=%u\n",
                      status & 7U, (status >> 3) & 7U, (status >> 6) & 3U,
                      (unsigned)obstacle.range_mm,
                      (unsigned)obstacle.range_valid,
                      (unsigned)obstacle.no_echo, (unsigned)obstacle.scan_id,
                      (unsigned)obstacle.profile.valid,
                      (unsigned)obstacle.profile.width_mm,
                      (unsigned)obstacle.profile.left_clear_mm,
                      (unsigned)obstacle.profile.right_clear_mm);
        }
#endif
#if APP_S4
        terrain_result_t terrain;
        uint16_t terrain_run, terrain_humps;
        if (subsystem4_read(&terrain, &terrain_run, &terrain_humps))
        {
            tm_printf((UB *)"[S4] run=%u state=%u flags=%u pitch_x10=%d "
                            "roll_x10=%d turn_dps=%d humps=%u peak_mm=%u "
                            "impact=%u lifted=%u overturned=%u\n",
                      (unsigned)terrain_run, (unsigned)terrain.state,
                      (unsigned)terrain.flags, (int)(terrain.pitch_deg * 10),
                      (int)(terrain.roll_deg * 10), (int)terrain.turn_dps,
                      (unsigned)terrain_humps,
                      (unsigned)(terrain.peak_m * 1000),
                      (unsigned)terrain.impact,
                      (unsigned)(terrain.safety == TERRAIN_SAFETY_LIFTED),
                      (unsigned)(terrain.safety == TERRAIN_SAFETY_OVERTURNED));
        }
        else
        {
            tm_printf((UB *)"[S4] waiting for sample/reset acknowledgement\n");
        }
#endif
        tm_printf(
            (UB *)"[S3+S2] state=%u reason=%u ready=0x%x cal=0x%x motion=%s "
                  "odom_valid=%u adc=%d gap=%u boot_max=%u above_target=%u "
                  "target_ms=%u\n",
            (unsigned)value_s.state, (unsigned)value_s.reason,
            (unsigned)value_s.navigation.readiness_blockers,
            (unsigned)value_s.navigation.accepted_calibration,
            motion_state_name(value_s.motion.state),
            (unsigned)value_s.motion.valid, (int)value_s.adc_status,
            (unsigned)value_s.navigation.sample_gap_us,
            (unsigned)value_s.max_gap_us, (unsigned)value_s.above_target,
            (unsigned)S3_SAMPLE_PERIOD_MS);
        tm_printf((UB *)"  RUN timing: start_us=%u active=%u samples=%u "
                        "max_gap_us=%u above_target=%u over30ms=%u "
                        "last_over30_us=%u\n",
                  (unsigned)value_s.run_timing.started_us,
                  (unsigned)drive_running(value_s.state),
                  (unsigned)value_s.run_timing.samples,
                  (unsigned)value_s.run_timing.max_gap_us,
                  (unsigned)value_s.run_timing.above_target,
                  (unsigned)value_s.run_timing.above_safety,
                  (unsigned)value_s.run_timing.last_safety_us);
        tm_printf(
            (UB *)"  motion_id=%u stopped=%u odom_epoch=%u report_losses=%u "
                  "motion_fault=%s\n",
            (unsigned)value_s.motion.command_id,
            (unsigned)value_s.motion.braked, (unsigned)value_s.motion.epoch,
            (unsigned)value_s.motion.report_loss_count,
            motion_fault_name(value_s.motion.fault));
        tm_printf(
            (UB *)"  command=%u ok=%u collecting=%u sensor=%u white n=%u "
                  "[%u,%u] black n=%u [%u,%u]\n",
            (unsigned)value_s.command_sequence, (unsigned)value_s.command_ok,
            (unsigned)value_s.collecting, (unsigned)value_s.calibration_sensor,
            (unsigned)value_s.white.count, (unsigned)value_s.white.minimum,
            (unsigned)value_s.white.maximum, (unsigned)value_s.black.count,
            (unsigned)value_s.black.minimum, (unsigned)value_s.black.maximum);
        /* Live calibrated readings: 0 = white, 1000 = black. Meaningful
         * only after that sensor's calibration has been accepted. */
        tm_printf(
            (UB *)"  IR dark: L(GP%u)=%u R(GP%u)=%u BAR(GP%u)=%u | "
                  "line present=%u error=%d junction=%u\n",
            S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_LEFT),
            (unsigned)value_s.navigation.sensor[S3_SENSOR_LEFT].darkness,
            S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_RIGHT),
            (unsigned)value_s.navigation.sensor[S3_SENSOR_RIGHT].darkness,
            S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_BARCODE),
            (unsigned)value_s.navigation.sensor[S3_SENSOR_BARCODE].darkness,
            (unsigned)value_s.navigation.line.present,
            (int)value_s.navigation.line.error,
            (unsigned)value_s.navigation.line.junction_candidate);
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
                    for (unsigned int value_i = 0U; value_i < sizeof letters;
                         value_i++)
                    {
                        letters[value_i] = g_bcap.letters[value_i];
                    }
                }
                (void)tk_unl_mtx(g_mutex);
            }
            tm_printf(
                (UB *)"  BARCODE this run: decoded=%u letters=%s rejected=%u "
                      "timing_error=%u overflow=%u dark_episodes=%u\n",
                (unsigned)decoded, letters, (unsigned)rejected,
                (unsigned)timing, (unsigned)overflow, (unsigned)episodes);
        }
        report_barcode_capture(drive_running(value_s.state));
#endif
    }
}

static void cleanup (void)
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

static float32_t floor_wheel (float32_t mps)
{
    return ((mps > 0.0f) && (mps < S3_MIN_WHEEL_MPS)) ? S3_MIN_WHEEL_MPS : mps;
}

static motion_result_t velocity_with_floor (float32_t left_mps, float32_t right_mps)
{
    return motion_request_velocity(floor_wheel(left_mps),
                                   floor_wheel(right_mps));
}
#if S3_BUILTIN_CALIBRATION
/* Feed the measured ranges through the SAME validation as a button
 * calibration (sample count, non-overlap, reflectance init). Returns the
 * mask of sensors accepted; any sensor that fails still needs the buttons. */
static uint32_t apply_builtin_calibration (void)
{
    static const uint16_t table[S3_SENSOR_COUNT][4] = {
        S3_BUILTIN_CAL_BARCODE, S3_BUILTIN_CAL_LEFT, S3_BUILTIN_CAL_RIGHT};
    unsigned int value_i;
    for (value_i = 0U; value_i < S3_SENSOR_COUNT; value_i++)
    {
        s3_calibration_collector_t value_c;
        s3_calibration_begin(&value_c);
        (void)s3_calibration_add(&value_c, IR_SURFACE_WHITE, table[value_i][0]);
        (void)s3_calibration_add(&value_c, IR_SURFACE_BLACK, table[value_i][2]);
        for (unsigned int value_k = 0U; value_k < 10U; value_k++)
        {
            (void)s3_calibration_add(
                &value_c, IR_SURFACE_WHITE,
                (uint16_t)((table[value_i][0] + table[value_i][1]) / 2U));
            (void)s3_calibration_add(
                &value_c, IR_SURFACE_BLACK,
                (uint16_t)((table[value_i][2] + table[value_i][3]) / 2U));
        }
        (void)s3_calibration_add(&value_c, IR_SURFACE_WHITE, table[value_i][1]);
        (void)s3_calibration_add(&value_c, IR_SURFACE_BLACK, table[value_i][3]);
        (void)s3_navigation_apply_calibration(
            &g_drive.navigation, (s3_sensor_id_t)value_i, &value_c);
    }
    return g_drive.navigation.accepted_calibration;
}
#endif
int32_t s3_integration_init (void)
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
        int32_t value_er = g_cycle;
        cleanup();
        return value_er;
    }
    return E_OK;
}

int32_t s3_integration_start (void)
{
    ER value_er;
    if ((g_mutex <= 0) || (g_cycle <= 0) || (g_task > 0))
    {
        return E_OBJ;
    }
    g_task = app_start_task((FP)sensor_sample_task, S3_PRI_MAIN, TP_PRC1,
                            "s3-motion");
    if (g_task < E_OK)
    {
        value_er = g_task;
        cleanup();
        return value_er;
    }
    g_report = app_start_task((FP)sensor_report_task, S3_PRI_REPORT, TP_PRC1,
                              "s3-status");
    if (g_report < E_OK)
    {
        value_er = g_report;
        cleanup();
        return value_er;
    }
    value_er = tk_sta_cyc(g_cycle);
    if (value_er < E_OK)
    {
        cleanup();
    }
    return value_er;
}
#else
void s3_remote_link (uint32_t epoch, bool up)
{
    (void)epoch;
    (void)up;
}

void s3_remote_receive (const uint8_t * p, unsigned n)
{
    (void)p;
    (void)n;
}

unsigned s3_remote_reply_take (uint8_t * p, unsigned n)
{
    (void)p;
    (void)n;
    return 0;
}

unsigned s3_remote_status_take (uint8_t * p, unsigned n)
{
    (void)p;
    (void)n;
    return 0;
}

uint32_t s3_remote_stop_sequence (void)
{
    return 0;
}

void s3_remote_physical_stop (bool held)
{
    (void)held;
}

bool s3_integration_submit (const s3_control_request_t * request)
{
    (void)request;
    return false;
}

bool s3_integration_status (s3_integration_status_t * status)
{
    (void)status;
    return false;
}

int32_t s3_integration_init (void)
{
    return E_NOSPT;
}

int32_t s3_integration_start (void)
{
    return E_NOSPT;
}
#endif
