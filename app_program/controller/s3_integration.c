/** @file s3_integration.c
 * @brief Run sensor sampling and serialized controller commands for the
 * integration profile.
 * @details The sampling task serializes navigation and control. Readers
 * take copies of its published state under the drive mutex.
 * @note Use unsigned masks for byte-sized flags and unsigned comparisons
 * for nonnegative enum limits. Byte and half-word operands fit the target's
 * 32-bit int, and a uint32_t operand fits int64_t when widened for a range
 * check or calculation.
 * @note Each explicitly discarded mutex release follows a successful
 * acquisition. The command, snapshot or reply result retains its existing
 * meaning regardless of the kernel's release status.
 * @see app_program/controller/README.md for the control and report paths.
 * @see docs/REMOTE_CONTROL_PROTOCOL.md for the command mailbox and replies.
 */
#include "s3_integration.h"
#include "../app_types.h"
#include "../build_assert.h"
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
#ifdef S3_CAL_DEFAULTS_TEST
#include "cal_defaults_main.h"
#endif
#ifdef S3_MAP_FREEZE_TEST
#include "map_freeze_main.h"
#endif
#if defined(S3_OPERATOR_TEST)
#include "operator_main.h"
#endif
#if defined(S3_RUNTIME_TEST)
#include "runtime_main.h"
#endif
#ifndef S3_BARCODE_CAPTURE
#define S3_BARCODE_CAPTURE 1 /* Print the recorded barcode trace over USB. */
#endif
#if defined(TM_MQTT) && TM_MQTT && (APP_IR_DEBUG || S3_BARCODE_CAPTURE)
#include "mqtt_utk.h"
#endif
/** Flags shared by the sampling and reporting paths under gh_drive_mutex. */
typedef struct
{
    bool b_has_command : 1;
    bool b_has_stop : 1;
    bool b_has_status : 1;
    bool b_has_reset : 1;
#if APP_IR_DEBUG
    bool b_has_ir_debug : 1;
    bool b_has_control_debug : 1;
#endif
} s3_shared_flags_t;

#define S3_COLLECTION_US (1000000U)
#define BCAP_USB_LINES_PER_POLL (40U)
#define CAL_MIDPOINT_SAMPLES (10U)
#define S3_EMPTY_RANGE_MINIMUM (4095U) /* A 12-bit ADC tops out at 4095. */

/** Calibration and reset state accessed only by the sampling owner. */
typedef struct
{
    uint8_t sensor : 2;
    uint8_t label : 2;
    bool    b_waits_for_reset : 1;
    bool    b_is_remote_reset : 1;
    bool    b_is_collecting : 1;
#if APP_IR_DEBUG
    bool b_has_control_tick : 1;
#endif
} s3_sampling_flags_t;

/* gh_drive_mutex protects the bounded remote mailbox and correlated result
 * ring. */
typedef struct
{
    uint32_t                 epoch;
    uint32_t                 last_id;
    uint32_t                 sampled_us;
    uint32_t                 published_us;
    uint32_t                 stop_sequence;
    unsigned                 flags : 4;
    unsigned                 head : 3;
    unsigned                 count : 3;
    bool                     b_is_connected : 1;
    bool                     b_has_published_status : 1;
    bool                     b_is_stop_held : 1;
    bool                     b_has_active_request : 1;
    remote_control_request_t pending;
    remote_control_request_t reset;
    uint8_t                  reply[REMOTE_REPLY_RING_SIZE][REMOTE_REPLY_SIZE];
} remote_mailbox_t;

BUILD_ASSERT(S3_SENSOR_COUNT <= 4, sampling_sensor_bits_t);
BUILD_ASSERT(IR_SURFACE_WHITE >= 0 && IR_SURFACE_WHITE <= 3 &&
                 IR_SURFACE_BLACK >= 0 && IR_SURFACE_BLACK <= 3,
             sampling_surface_bits_t);

/* Minimum forward wheel speed while line following. On the car (2026-10-26)
 * a strongly-off-centre start made Buddy 3's steering ask the inner wheel for
 * ~0.04 m/s; that wheel never started turning and Buddy 2's stall check
 * (0.6 s without encoder progress) faulted the run. The motor smoke test
 * moved reliably at 0.05 m/s. Raise only small positive requests; zero and
 * reverse requests pass through unchanged, and Buddy 3's steering algorithm
 * itself is untouched. Tune on the car; set to 0 to disable. */
#define S3_MIN_WHEEL_MPS (0.01f)

#if S3_BARCODE_CAPTURE
static bc_capture_t g_bcap; /* Sampling and reporting share gh_drive_mutex. */
static bool         gb_has_capture;
/* Wi-Fi copy of the trace: sent in binary packets on MQTT_UTK_DEBUG_TOPIC after
 * a run ends, sharing publish slots with routine telemetry. The flag is a cheap
 * hint read by the network task every poll; the cursor is used under
 * gh_drive_mutex.
 */
static volatile bool       gb_is_capture_pending;
static bc_capture_cursor_t g_bcap_net_cursor;
#if defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
extern volatile UW
    tm_usb_link_state; /* State three means the USB terminal asserted DTR. */
#endif
#endif
static ID                                 gh_sample_task;
static ID                                 gh_report_task;
static ID                                 gh_sample_cycle;
static ID                                 gh_drive_mutex;
static s3_drive_t                         g_drive;
static integration_codec_status_packet_t  g_status;
static integration_codec_control_packet_t g_command;

static s3_shared_flags_t g_shared;

/* Only the sampling owner reads or writes this storage byte. Sensor values
 * are checked before assignment; labels retain zero, white and black. Keep
 * publication flags in g_shared, whose every access holds gh_drive_mutex. */
static s3_sampling_flags_t g_sampling;

#if defined(__arm__) || defined(__thumb__)
BUILD_ASSERT(sizeof(g_sampling) == 1U, sampling_storage_byte_t);
BUILD_ASSERT(sizeof(g_shared) == 1U, shared_storage_byte_t);
#endif

static ir_calibration_collector_t g_collector[S3_SENSOR_COUNT];
#if APP_IR_DEBUG
/* One latest-only packet under gh_drive_mutex; no queue, task or peripheral
 * owner. */
static ir_debug_packet_t      g_ir_debug;
static bool                   gb_has_ir_tick;
static uint32_t               g_ir_last;
static uint8_t                g_ir_sequence;
static control_debug_packet_t g_control_debug;
static uint32_t               g_control_last;
static uint8_t                g_control_sequence;
static uint8_t                g_last_barcode_status;

static uint16_t g_debug_events[3]; /* Saturate the counters since boot. */
#endif
static uint32_t g_collect_since;

static remote_mailbox_t g_remote;

#if S3_BARCODE_CAPTURE
static inline bool bcap_terminal_open (void);

#endif
static uint32_t now_us (void);

static void remote_reply (const remote_control_request_t * p_request,
                          uint8_t result, uint8_t reason);

static void remote_cancel (remote_control_request_t * p_request);

static void remote_finish (const remote_control_request_t * p_request,
                           bool b_did_succeed, uint8_t reason);

static bool remote_permitted (const remote_control_request_t * p_request);

static bool execute (const s3_integration_request_t * p_control, uint32_t now);

static void wake_sample_task (void * p_unused);

static bool drive_running (s3_drive_state_t state);

static void reset_completed (void);

#if APP_IR_DEBUG
static bool debug_scaled (float32_t value, float32_t scale, int32_t limit,
                          int32_t * p_out);

static bool debug_builder_scaled (control_debug_builder_t * p_builder,
                                  control_debug_field_t field, float32_t value,
                                  float32_t scale, int32_t limit);

static bool capture_control_debug (const s3_integration_status_t * p_st,
                                   control_debug_packet_t *        p_packet);

#endif
static inline void
collect_calibration_sample (const ir_acquisition_frame_t * p_frame);

#if S3_BARCODE_CAPTURE
static inline void start_capture_run (void);
static inline void
append_capture_sample (const bc_capture_input_t * p_capture_input);
#endif
static void sensor_sample_task (INT code, void * p_unused);

#if S3_BARCODE_CAPTURE
static void report_barcode_capture (bool b_is_running);

#endif
static void sensor_report_task (INT code, void * p_unused);

static void cleanup (void);

static float32_t floor_wheel (float32_t mps);

static motion_result_t velocity_with_floor (float32_t left_mps,
                                            float32_t right_mps);

#if S3_BUILTIN_CALIBRATION
static uint32_t apply_builtin_calibration (void);

#endif

/** @brief Update link state under the mailbox mutex. Cancel pending non-STOP
 * commands and remote reset replies. A new connection resets request
 * numbering and schedules a status publication.
 */
void remote_control_link (uint32_t epoch, bool b_is_up)
{
    if ((gh_drive_mutex <= 0) || (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) != E_OK))
    {
        return;
    }

    if ((g_remote.pending.b_is_request_valid) &&
        ((int)g_remote.pending.action != REMOTE_STOP))
    {
        remote_cancel(&g_remote.pending);
        g_shared.b_has_command = false;
        g_shared.b_has_reset   = false;
    }

    remote_cancel(
        &g_remote.reset); /* The owner finishes the reset bookkeeping. */
    g_remote.b_is_connected = b_is_up;

    if (b_is_up)
    {
        g_remote.epoch   = epoch ? epoch : 1U;
        g_remote.last_id = 0U;
    }

    g_remote.b_has_published_status = false;
    (void)tk_unl_mtx(gh_drive_mutex);
}

/** @brief Validate a version-3 request and update the bounded mailbox under
 * its mutex.
 * @param[in] p_packet Borrowed request bytes, readable for length bytes.
 * @param[in] length Available byte count. The request has a fixed length.
 * @details An invalid envelope or unavailable mailbox produces no reply.
 * Otherwise, check freshness, request sequence, physical STOP and mailbox
 * availability in that order for START and RESET. A newer identifier in the
 * current epoch is consumed even when its request is rejected. STOP bypasses
 * those admission checks and cancels queued work and the pending reset.
 * The correlated reply records admission. The sampling owner applies the
 * command and reports its outcome separately.
 * @note Each action byte is in 0..255 and fits int before comparison.
 */
void remote_control_receive (const uint8_t * p_packet, unsigned length)
{
    if ((((((0 == p_packet) || (length != REMOTE_COMMAND_SIZE)) ||
           (p_packet[0] != REMOTE_VERSION)) ||
          ((int)p_packet[1] < REMOTE_START)) ||
         ((int)p_packet[1] > REMOTE_RESET)) ||
        (gh_drive_mutex <= 0))
    {
        return;
    }

    const remote_control_request_t request = {
        .action             = p_packet[1],
        .request_id         = remote_control_get_u32(p_packet + 2),
        .epoch              = remote_control_get_u32(p_packet + 6),
        .time_us            = remote_control_get_u32(p_packet + 10),
        .b_is_request_valid = true};

    if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) != E_OK)
    {
        return;
    }

    uint8_t    reason = REMOTE_NONE;
    const bool b_is_fresh =
        ((g_remote.b_is_connected) && (request.epoch == g_remote.epoch)) &&
        ((now_us() - request.time_us) <= REMOTE_MAX_AGE_US);

    if (((int)request.action != REMOTE_STOP) && (!b_is_fresh))
    {
        reason = REMOTE_STALE;
    }
    else if (((int)request.action != REMOTE_STOP) &&
             (!remote_control_is_newer(request.request_id, g_remote.last_id)))
    {
        reason = REMOTE_REPLAY;
    }
    else if (((int)request.action != REMOTE_STOP) && (g_remote.b_is_stop_held))
    {
        reason = REMOTE_NOT_READY;
    }
    else if (((int)request.action != REMOTE_STOP) &&
             ((((g_shared.b_has_command) || (g_shared.b_has_stop)) ||
               (g_shared.b_has_reset)) ||
              (g_remote.b_has_active_request)))
    {
        reason = REMOTE_BUSY;
    }
    else
    {
        /* The request passed the rejection checks. */
    }
    /* Newer rejected IDs are consumed, too: no older START after a newer STOP.
     */
    if ((request.epoch == g_remote.epoch) &&
        (remote_control_is_newer(request.request_id, g_remote.last_id)))
    {
        g_remote.last_id = request.request_id;
    }

    if ((int)reason != REMOTE_NONE)
    {
        remote_reply(&request, REMOTE_REJECTED, reason);
    }
    else
    {
        const s3_integration_request_t command = {
            .kind = REMOTE_START == (int)request.action ? S3_CONTROL_START
                    : REMOTE_STOP == (int)request.action ? S3_CONTROL_STOP
                                                         : S3_CONTROL_RESET};

        if (REMOTE_STOP == (int)request.action)
        {
            ++g_remote.stop_sequence;
            remote_cancel(&g_remote.pending);
            remote_cancel(&g_remote.reset);
            g_shared.b_has_stop    = true;
            g_shared.b_has_command = false;
            g_shared.b_has_reset   = false;
        }
        else
        {
            (void)integration_codec_pack_cmd(&command, &g_command);
            g_shared.b_has_command = true;
            g_shared.b_has_reset   = REMOTE_RESET == (int)request.action;
        }

        g_remote.pending = request;
        remote_reply(&request, REMOTE_QUEUED, REMOTE_NONE);
    }

    (void)tk_unl_mtx(gh_drive_mutex);
}

/** @brief Poll the mailbox mutex and copy the oldest complete reply to the
 * caller. Return its byte count, or zero without consuming it for a
 * short buffer, empty ring or unavailable lock.
 */
unsigned remote_control_reply_take (uint8_t * p_packet, unsigned capacity)
{
    unsigned reply_size = 0;

    if ((((0 == p_packet) || (capacity < REMOTE_REPLY_SIZE)) ||
         (gh_drive_mutex <= 0)) ||
        (tk_loc_mtx(gh_drive_mutex, TMO_POL) != E_OK))
    {
        return 0;
    }

    if (g_remote.count)
    {
        for (unsigned byte_index = 0; byte_index < REMOTE_REPLY_SIZE;
             byte_index++)
        {
            p_packet[byte_index] = g_remote.reply[g_remote.head][byte_index];
        }

        g_remote.head = (g_remote.head + 1U) % REMOTE_REPLY_RING_SIZE;
        --g_remote.count;
        reply_size = REMOTE_REPLY_SIZE;
    }

    (void)tk_unl_mtx(gh_drive_mutex);
    return reply_size;
}

/** @brief Poll for connected status, initially and then every 500 ms. Return a
 * full version-3 packet or zero bytes. Flags older than 200 ms are
 * reported as zero.
 */
unsigned remote_control_status_take (uint8_t * p_packet, unsigned capacity)
{
    unsigned status_size = 0;

    if ((((0 == p_packet) || (capacity < REMOTE_STATUS_SIZE)) ||
         (gh_drive_mutex <= 0)) ||
        (tk_loc_mtx(gh_drive_mutex, TMO_POL) != E_OK))
    {
        return 0;
    }

    const uint32_t now = now_us();

    if ((g_remote.b_is_connected) &&
        ((!g_remote.b_has_published_status) ||
         ((now - g_remote.published_us) >= 500000U)))
    {
        p_packet[0] = REMOTE_VERSION;
        remote_control_put_u32(p_packet + 1, g_remote.epoch);
        remote_control_put_u32(p_packet + 5, now);
        remote_control_put_u32(p_packet + 9, g_remote.last_id);
        /* clang-format off */
        p_packet[13]                    =
            (now - g_remote.sampled_us) <= 200000U
                                    ? g_remote.flags
                                    : 0U;
        g_remote.published_us           = now;
        g_remote.b_has_published_status = true;
        status_size                     = REMOTE_STATUS_SIZE;
        /* clang-format on */
    }

    (void)tk_unl_mtx(gh_drive_mutex);
    return status_size;
}

/** @brief Read the wrapping accepted remote STOP count under the mailbox
 * mutex. Return zero if the mutex is unavailable.
 */
uint32_t remote_control_stop_sequence (void)
{
    uint32_t sequence = 0;

    if ((gh_drive_mutex > 0) && (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK))
    {
        sequence = g_remote.stop_sequence;
        (void)tk_unl_mtx(gh_drive_mutex);
    }

    return sequence;
}

/** @brief Publish the current physical STOP button state under the mailbox
 * mutex for the remote-control interlock.
 */
void remote_control_physical_stop (bool b_is_held)
{
    if ((gh_drive_mutex > 0) && (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK))
    {
        g_remote.b_is_stop_held = b_is_held;
        (void)tk_unl_mtx(gh_drive_mutex);
    }
}

/** @brief Validate and encode a local control request before locking. STOP
 * supersedes queued work. Other requests need an available mailbox.
 * True means accepted for later processing.
 */
bool s3_integration_submit (const s3_integration_request_t * p_request)
{
    bool                               b_is_accepted = false;
    integration_codec_control_packet_t packet;

    if ((((0 == p_request) || (gh_drive_mutex <= 0)) ||
         ((unsigned int)p_request->kind > (unsigned int)S3_CONTROL_RESET)) ||
        (!integration_codec_pack_cmd(p_request, &packet)))
    {
        return false;
    }

    if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }

    if ((unsigned int)S3_CONTROL_STOP == p_request->kind)
    {
        remote_cancel(&g_remote.pending);
        remote_cancel(&g_remote.reset);
        g_shared.b_has_stop    = true;
        g_shared.b_has_command = false;
        g_shared.b_has_reset   = false;
        b_is_accepted          = true;
    }
    else if (((!g_shared.b_has_command) && (!g_shared.b_has_stop)) &&
             (!g_shared.b_has_reset))
    {
        /* clang-format off */
        g_command              = packet;
        g_shared.b_has_command = true;
        g_shared.b_has_reset   =
            (unsigned int)S3_CONTROL_RESET == p_request->kind;
        /* clang-format on */

        b_is_accepted = true;
    }
    else
    {
        /* Leave the occupied mailbox and its queued work intact. */
    }

    (void)tk_unl_mtx(gh_drive_mutex);
    return b_is_accepted;
}

/** @brief Copy the latest status packet while locked and unpack it for the
 * caller. Return false for a missing output, unavailable mutex,
 * unavailable snapshot or invalid encoding.
 */
bool s3_integration_status (s3_integration_status_t * p_status)
{
    integration_codec_status_packet_t packet;

    if ((0 == p_status) || (gh_drive_mutex <= 0))
    {
        return false;
    }

    if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }

    packet                = g_status;
    const bool b_is_ready = g_shared.b_has_status;
    (void)tk_unl_mtx(gh_drive_mutex);
    return b_is_ready && (integration_codec_unpack_status(&packet, p_status));
}

#if APP_IR_DEBUG && defined(TM_MQTT) && TM_MQTT
/** @brief Take the pending control packet if its age is at most 400 ms.
 * A short buffer or busy mutex leaves the packet pending. Once locked, a
 * stale pending packet is discarded. Return the copied size or zero.
 */
unsigned int mqtt_utk_control_debug_take (uint8_t * p_bytes, unsigned int size)
{
    unsigned int count = 0;

    if ((((0 == p_bytes) || (size < (unsigned int)CONTROL_DEBUG_PACKET_SIZE)) ||
         (gh_drive_mutex <= 0)) ||
        (tk_loc_mtx(gh_drive_mutex, TMO_POL) != E_OK))
    {
        return 0;
    }

    if (g_shared.b_has_control_debug)
    {
        const uint32_t timestamp = (uint32_t)g_control_debug.bytes[1] |
                                   ((uint32_t)g_control_debug.bytes[2] << 8) |
                                   ((uint32_t)g_control_debug.bytes[3] << 16) |
                                   ((uint32_t)g_control_debug.bytes[4] << 24);

        if ((now_us() - timestamp) <= 400000U)
        {
            for (unsigned byte_index = 0;
                 byte_index < (unsigned int)CONTROL_DEBUG_PACKET_SIZE;
                 ++byte_index)
            {
                p_bytes[byte_index] = g_control_debug.bytes[byte_index];
            }

            count = CONTROL_DEBUG_PACKET_SIZE;
        }

        g_shared.b_has_control_debug = false;
    }

    (void)tk_unl_mtx(gh_drive_mutex);
    return count;
}

/* Called only by the network owner, with a nonblocking mutex attempt. */
/** @brief Take the pending IR packet if its age is at most 100 ms.
 * A short buffer or busy mutex leaves the packet pending. Once locked, a
 * stale pending packet is discarded. Return the copied size or zero.
 */
unsigned int mqtt_utk_ir_debug_take (uint8_t * p_bytes, unsigned int size)
{
    unsigned int count = 0U;

    if ((((0 == p_bytes) || (size < IR_DEBUG_PACKET_SIZE)) ||
         (gh_drive_mutex <= 0)) ||
        (tk_loc_mtx(gh_drive_mutex, TMO_POL) != E_OK))
    {
        return 0U;
    }

    if (g_shared.b_has_ir_debug)
    {
        const uint32_t timestamp = (uint32_t)g_ir_debug.bytes[1] |
                                   ((uint32_t)g_ir_debug.bytes[2] << 8) |
                                   ((uint32_t)g_ir_debug.bytes[3] << 16) |
                                   ((uint32_t)g_ir_debug.bytes[4] << 24);

        if ((now_us() - timestamp) <= 100000U)
        {
            for (unsigned byte_index = 0U; byte_index < IR_DEBUG_PACKET_SIZE;
                 ++byte_index)
            {
                p_bytes[byte_index] = g_ir_debug.bytes[byte_index];
            }

            count = IR_DEBUG_PACKET_SIZE;
        }

        g_shared.b_has_ir_debug = false;
    }

    (void)tk_unl_mtx(gh_drive_mutex);
    return count;
}

#endif
#if S3_BARCODE_CAPTURE
#if defined(TM_MQTT) && TM_MQTT
/* Network task: next binary capture packet; never wait on the control owner. */
/** @brief Poll the capture mutex and return the next binary packet length. A
 * busy lock, short buffer or unavailable capture returns zero.
 * Exhaustion clears the pending upload flag.
 */
unsigned int mqtt_utk_debug_take (uint8_t * p_bytes, unsigned int size)
{
    unsigned int packet_size = 0U;

    if ((((0 == p_bytes) || (size < BCAP_WIRE_MAX)) ||
         (!gb_is_capture_pending)) ||
        (gh_drive_mutex <= 0))
    {
        return 0U;
    }

    if (tk_loc_mtx(gh_drive_mutex, TMO_POL) != E_OK)
    {
        return 0U;
    }

    if (gb_is_capture_pending)
    {
        packet_size =
            bc_capture_binary_next(&g_bcap, &g_bcap_net_cursor, p_bytes, size);

        if (!packet_size)
        {
            gb_is_capture_pending = false;
        }
    }

    (void)tk_unl_mtx(gh_drive_mutex);
    return packet_size;
}

#endif
#endif
/** @brief Initialize drive configuration, shared mutex and cyclic wakeup.
 * Return E_OBJ for repeated initialization, E_PAR for invalid drive
 * settings or the kernel creation error.
 */
int32_t s3_integration_init (void)
{
    T_CMTX                   mutex = {0};
    T_CCYC                   cycle = {0};
    line_navigation_config_t cfg   = line_navigation_defaults();
    const s3_drive_port_t    port  = {
        velocity_with_floor,        motion_request_manoeuvre, stop,
        motion_get_report,          motion_command_id,        motion_reset,
        motion_navigation_heartbeat};
    const s3_drive_options_t options = course_profile_options();

    if (gh_drive_mutex > 0)
    {
        return E_OBJ;
    }

    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    course_profile_navigation(&cfg);

    if ((!s3_drive_init(&g_drive, &cfg, &port)) ||
        (!s3_drive_set_options(&g_drive, &options)))
    {
        return E_PAR;
    }
#if S3_BUILTIN_CALIBRATION
    (void)apply_builtin_calibration();
#endif
    mutex.mtxatr   = TA_INHERIT;
    gh_drive_mutex = tk_cre_mtx(&mutex);

    if (gh_drive_mutex < E_OK)
    {
        return gh_drive_mutex;
    }

    /* T_CCYC uses the kernel FP address type. The cyclic handler receives
     * one void * extended-information argument. See tk/syscall.h. */
    cycle.cycatr    = TA_HLNG;
    cycle.cychdr    = (FP)wake_sample_task;
    cycle.cyctim    = S3_SAMPLE_PERIOD_MS;
    cycle.cycphs    = S3_SAMPLE_PERIOD_MS;
    gh_sample_cycle = tk_cre_cyc(&cycle);

    if (gh_sample_cycle < E_OK)
    {
        const int32_t value_er = gh_sample_cycle;
        cleanup();
        return value_er;
    }

    return E_OK;
}

/** @brief Start the sampling and reporting tasks, then enable cyclic wakeups.
 * Return E_OBJ for invalid startup order. Creation or timer start
 * failures release acquired resources.
 */
int32_t s3_integration_start (void)
{
    ER value_er;

    if (((gh_drive_mutex <= 0) || (gh_sample_cycle <= 0)) ||
        (gh_sample_task > 0))
    {
        return E_OBJ;
    }

    /* Both entry points take the INT start code and void * context that
     * app_start_task passes through the kernel's FP entry field. */
    gh_sample_task = app_start_task((FP)sensor_sample_task, S3_PRI_MAIN,
                                    TP_PRC1, "s3-motion");

    if (gh_sample_task < E_OK)
    {
        value_er = gh_sample_task;
        cleanup();
        return value_er;
    }

    gh_report_task = app_start_task((FP)sensor_report_task, S3_PRI_REPORT,
                                    TP_PRC1, "s3-status");

    if (gh_report_task < E_OK)
    {
        value_er = gh_report_task;
        cleanup();
        return value_er;
    }

    value_er = tk_sta_cyc(gh_sample_cycle);

    if (value_er < E_OK)
    {
        cleanup();
    }

    return value_er;
}

#ifdef S3_RUNTIME_TEST
/** @brief Borrow the drive until the next simulated task cycle. */
const s3_drive_t * s3_integration_runtime_test_drive (void)
{
    return &g_drive;
}

/** @brief Run the sampling owner until the fixture's simulated sleep.
 * @pre The fixture has installed a jump target and initialized the owner.
 * The same fake sleep returns to the caller after each sampling cycle.
 */
void s3_integration_runtime_test_cycle (void)
{
    sensor_sample_task(0, NULL);
}

#if S3_BARCODE_CAPTURE
/** @brief Read whether the capture upload can begin in the fixture. */
bool s3_integration_runtime_test_capture_pending (void)
{
    return gb_is_capture_pending;
}
#endif

/** @brief Copy private state after a single-threaded fixture event. The
 * snapshot does not consume a reply or modify a publication flag.
 */
runtime_main_private_view_t s3_integration_runtime_test_view (void)
{
    runtime_main_private_view_t view = {
        .epoch                 = g_remote.epoch,
        .last_id               = g_remote.last_id,
        .stop_sequence         = g_remote.stop_sequence,
        .pending_id            = g_remote.pending.TEST_REMOTE_ID_MEMBER,
        .pending_epoch         = g_remote.pending.epoch,
        .head                  = g_remote.head,
        .count                 = g_remote.count,
        .pending_action        = g_remote.pending.action,
        .b_has_command         = TEST_S3_SHARED_PENDING,
        .b_has_stop            = TEST_S3_SHARED_STOP,
        .b_has_status          = TEST_S3_SHARED_STATUS_READY,
        .b_has_reset           = TEST_S3_SHARED_RESET_PENDING,
        .b_waits_for_reset     = TEST_S3_RESET_WAITING,
        .b_is_remote_reset     = TEST_S3_RESET_REMOTE,
        .b_has_pending_request = g_remote.pending.b_is_request_valid};
    memcpy(view.reply, g_remote.reply, sizeof view.reply);
#if APP_IR_DEBUG
    view.b_has_ir_debug      = TEST_S3_IR_READY;
    view.b_has_control_tick  = TEST_S3_CONTROL_SEEN;
    view.b_has_control_debug = TEST_S3_CONTROL_READY;
    memcpy(view.control_packet, g_control_debug.bytes,
           sizeof view.control_packet);
#endif
    return view;
}

/** @brief Seed the remote status fields for the exact packet check.
 * @param timestamp_us Use the fixture's wrapping microsecond clock.
 * @pre Only the fixture thread is active. The remote mailbox can be seeded
 * without taking its production mutex.
 */
void s3_integration_runtime_test_seed_status (uint32_t timestamp_us)
{
    g_remote.epoch                  = 0x11223344U;
    g_remote.last_id                = 0x12345678U;
    g_remote.flags                  = 3;
    g_remote.sampled_us             = timestamp_us;
    g_remote.b_has_published_status = false;
}

/** @brief Queue one fixture reply through the real bounded mailbox.
 * @param[in] p_request Live request owned by the fixture for this call.
 * @param result One REMOTE_* reply outcome.
 * @param reason One REMOTE_* reply reason.
 * @note The production queue applies the same limit and eviction behavior.
 */
void s3_integration_runtime_test_reply (
    const remote_control_request_t * p_request, uint8_t result, uint8_t reason)
{
    remote_reply(p_request, result, reason);
}

/** @brief Issue START immediately for the slope fixture.
 * @param now_us Current simulated microsecond time, modulo 2^32.
 * @return The real command handler's acceptance result.
 */
bool s3_integration_runtime_test_start (uint32_t now_us)
{
    return execute(&(s3_integration_request_t){.kind = S3_CONTROL_START},
                   now_us);
}

/** @brief Apply one fixture calibration to the drive owner.
 * @param sensor Sensor selector used by navigation calibration.
 * @param[in] p_values Borrowed calibration summary for the duration of the
 * call.
 * @return True if the existing navigation validator accepts the summary.
 */
bool s3_integration_runtime_test_calibration (
    ir_acquisition_sensor_id_t         sensor,
    const ir_calibration_collector_t * p_values)
{
    return line_navigation_set_calibration(&g_drive.navigation, sensor,
                                           p_values);
}
#endif /* S3_RUNTIME_TEST */

#ifdef S3_OPERATOR_TEST
/** @brief Borrow the drive state until the next simulated task cycle. */
const s3_drive_t * s3_integration_operator_test_drive (void)
{
    return &g_drive;
}

/** @brief Enter the real sampling owner; the fake sleep ends its cycle. */
void s3_integration_operator_test_cycle (void)
{
    sensor_sample_task(0, NULL);
}

/** @brief Seed the calibration-time sensor fault in the single-threaded test.
 */
void s3_integration_operator_test_sensor_fault (void)
{
    g_drive.state  = S3_DRIVE_FAULT;
    g_drive.reason = S3_DRIVE_SENSOR_FAULT;
}
#endif

#if defined(S3_CAL_DEFAULTS_TEST) || defined(S3_MAP_FREEZE_TEST)
/** @brief Borrow the drive for fixture reads while the owner is yielded. */
const s3_drive_t * s3_integration_test_drive (void)
{
    return &g_drive;
}

/** @brief Enter the sampling task until the fixture clock yields control. */
void s3_integration_test_sample_period (void)
{
    sensor_sample_task(0, NULL);
}
#endif

#ifdef S3_MAP_FREEZE_TEST
/** @brief Seed the accepted-sensor mask for the map-only test. */
void s3_integration_test_set_calibrated (void)
{
    g_drive.navigation.accepted_calibration = S3_ALL_SENSORS;
}
#endif

#ifdef S3_CAL_DEFAULTS_TEST
/** @brief Return the configured forward floor in metres per second. */
float32_t s3_integration_test_floor_mps (void)
{
    return S3_MIN_WHEEL_MPS;
}

/** @brief Exercise the normal floor and velocity boundary in m/s. */
motion_result_t s3_integration_test_velocity (float32_t left_mps,
                                              float32_t right_mps)
{
    return velocity_with_floor(left_mps, right_mps);
}
#endif

#if S3_BARCODE_CAPTURE
/** @brief Read the USB terminal-open flag, or allow the non-USB console. */
static inline bool bcap_terminal_open (void)
{
#if defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
    return 3U == tm_usb_link_state;
#else
    return true;
#endif
}

#endif
/** @brief Read the low 32 bits of the RP2040 hardware timer in microseconds.
 */
static uint32_t now_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

/** @brief With the mailbox mutex held, append one version-4 result for a valid
 * request. A full ring discards its oldest reply. STOP uses the
 * current link epoch.
 */
static void remote_reply (const remote_control_request_t * p_request,
                          uint8_t result, uint8_t reason)
{
    if (!p_request->b_is_request_valid)
    {
        return;
    }

    if (REMOTE_REPLY_RING_SIZE == g_remote.count)
    {
        g_remote.head = (g_remote.head + 1U) % REMOTE_REPLY_RING_SIZE;
        --g_remote.count;
    }

    /* The promoted sum is nonnegative and below twice the ring size. */
    const unsigned slot =
        (unsigned)(g_remote.head + g_remote.count) % REMOTE_REPLY_RING_SIZE;
    uint8_t * p_packet = g_remote.reply[slot];
    p_packet[0]        = REMOTE_REPLY_VERSION;
    /* Internal enum values occupy 2 + 2 + 3 bits, so this cast cannot narrow
     * a valid value. Preserve all 32 request-ID and epoch bits unchanged. */
    p_packet[1] = (uint8_t)((uint32_t)p_request->action |
                            ((uint32_t)result << REMOTE_REPLY_ACTION_BITS) |
                            ((uint32_t)reason << (REMOTE_REPLY_ACTION_BITS +
                                                  REMOTE_REPLY_RESULT_BITS)));
    remote_control_put_u32(p_packet + REMOTE_REPLY_ID_OFFSET,
                           p_request->request_id);
    remote_control_put_u32(p_packet + REMOTE_REPLY_EPOCH_OFFSET,
                           REMOTE_STOP == (int)p_request->action
                               ? g_remote.epoch
                               : p_request->epoch);
    ++g_remote.count;
}

/** @brief With the mailbox mutex held, queue cancellation of a valid request
 * and clear its valid flag.
 */
static void remote_cancel (remote_control_request_t * p_request)
{
    remote_reply(p_request, REMOTE_CANCELLED, REMOTE_SUPERSEDED);
    p_request->b_is_request_valid = false;
}

/** @brief Report the owner's result and release the in-flight flag under the
 * mailbox mutex. An invalid request or failed lock leaves the mailbox
 * untouched.
 */
static void remote_finish (const remote_control_request_t * p_request,
                           bool b_did_succeed, uint8_t reason)
{
    if ((!p_request->b_is_request_valid) ||
        (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) != E_OK))
    {
        return;
    }

    remote_reply(p_request, b_did_succeed ? REMOTE_EXECUTED : REMOTE_REJECTED,
                 b_did_succeed ? REMOTE_NONE : (int)reason);
    g_remote.b_has_active_request = false;
    (void)tk_unl_mtx(gh_drive_mutex);
}

/** @brief Recheck link, epoch, request ID, STOP interlocks and age in
 * microseconds before the owner acts. Failure to lock denies the
 * request.
 */
static bool remote_permitted (const remote_control_request_t * p_request)
{
    bool b_is_permitted = false;

    if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK)
    {
        b_is_permitted = (((((g_remote.b_is_connected) &&
                             (p_request->epoch == g_remote.epoch)) &&
                            (!g_shared.b_has_stop)) &&
                           (!g_remote.b_is_stop_held)) &&
                          (p_request->request_id == g_remote.last_id)) &&
                         ((now_us() - p_request->time_us) <= REMOTE_MAX_AGE_US);
        (void)tk_unl_mtx(gh_drive_mutex);
    }

    return b_is_permitted;
}

/** @brief Execute one validated control request on the sampling owner. The
 * wrapping timestamp is in microseconds. Return the outcome of the
 * requested drive or calibration action.
 */
static bool execute (const s3_integration_request_t * p_control, uint32_t now)
{
    if ((unsigned int)S3_CONTROL_STOP == p_control->kind)
    {
        s3_drive_stop(&g_drive);
        g_sampling.b_is_collecting = false;
        return true;
    }

    if ((unsigned int)S3_CONTROL_RESET == p_control->kind)
    {
        return (terr_link_reset_ready(now)) && (s3_drive_reset(&g_drive, now));
    }

    if ((unsigned int)S3_CONTROL_START == p_control->kind)
    {
        /* Ineligible STARTs (already driving, FAULT, active lifecycle,
         * collecting, not ready) are refused BEFORE touching the map. */
        if ((((((!terr_link_ready(now)) || (g_sampling.b_is_collecting)) ||
               ((g_drive.state != (unsigned int)S3_DRIVE_OFF) &&
                (g_drive.state != (unsigned int)S3_DRIVE_HALTED))) ||
              ((int)g_drive.navigation.lifecycle != S3_RUN_INACTIVE)) ||
             (line_navigation_readiness(&g_drive.navigation, now) != 0U)) ||
            (!vehicle_controller_prep_start(&g_drive.navigation)))
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

        if (!terr_link_begin(now))
        {
            s3_drive_stop(&g_drive);
            vehicle_controller_cancel_start();
            return false;
        }

        vehicle_controller_commit_start();
        return true;
    }

    if ((((int)g_drive.navigation.lifecycle != S3_RUN_INACTIVE) ||
         (g_sampling.b_is_collecting)) ||
        ((unsigned int)S3_DRIVE_FAULT == g_drive.state))
    {
        return false;
    }

    if ((unsigned int)S3_CONTROL_MAPPING == p_control->kind)
    {
        return (vehicle_controller_set_mapping(p_control->map)) &&
               (line_navigation_set_mapping(&g_drive.navigation,
                                            p_control->map));
    }

    if ((unsigned int)p_control->sensor >= (unsigned int)S3_SENSOR_COUNT)
    {
        return false;
    }

    if ((unsigned int)S3_CONTROL_APPLY_CALIBRATION == p_control->kind)
    {
        return line_navigation_set_calibration(&g_drive.navigation,
                                               p_control->sensor,
                                               &g_collector[p_control->sensor]);
    }

    /* clang-format off */
    g_sampling.sensor          = p_control->sensor;
    g_sampling.b_is_collecting = true;
    g_collect_since            = now;
    g_sampling.label           =
        (unsigned int)S3_CONTROL_WHITE == p_control->kind
                           ? IR_SURFACE_WHITE
                           : IR_SURFACE_BLACK;
    /* clang-format on */
    /* Same empty-range convention as ir_calibration_begin(): the minimum
     * starts at the 12-bit maximum. Starting it at 0 kept both minima at 0,
     * so every white/black pair looked overlapping and was rejected. */
    if (IR_SURFACE_WHITE == (int)g_sampling.label)
    {
        g_collector[g_sampling.sensor].white =
            (ir_calibration_range_t){.minimum = S3_EMPTY_RANGE_MINIMUM};
    }
    else
    {
        g_collector[g_sampling.sensor].black =
            (ir_calibration_range_t){.minimum = S3_EMPTY_RANGE_MINIMUM};
    }

    return true;
}

/** @brief Wake the sampling task if it has been created. The cyclic callback
 * argument is unused.
 */
static void wake_sample_task (void * p_unused)
{
    (void)p_unused;

    if (gh_sample_task > 0)
    {
        (void)tk_wup_tsk(gh_sample_task);
    }
}

/** @brief Return whether the drive state lies in the inclusive FOLLOW through
 * REACQUIRE range used by run capture.
 */
static bool drive_running (s3_drive_state_t state)
{
    return (state >= (unsigned int)S3_DRIVE_FOLLOW) &&
           (state <= (unsigned int)S3_DRIVE_REACQUIRE);
}

/** @brief Clear the shared pending-reset flag after owner processing when the
 * mailbox mutex can be acquired.
 */
static void reset_completed (void)
{
    if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK)
    {
        g_shared.b_has_reset = false;
        (void)tk_unl_mtx(gh_drive_mutex);
    }
}

#if APP_IR_DEBUG
/** @brief Scale a floating-point sample and round ties away from zero. Reject
 * an unordered value or one outside the signed limit before writing
 * the integer output.
 * @pre Current callers use a positive limit of 100 or 4095. The largest
 * rounded magnitude is 4095, which fits int32_t.
 */
static bool debug_scaled (float32_t value, float32_t scale, int32_t limit,
                          int32_t * p_out)
{
    const float32_t scaled = value * scale;
    /* Ordered comparisons also reject NaN/infinity, without newlib headers
     * (whose size_t conflicts with this kernel's freestanding types). */
    if (!((scaled >= -(float32_t)limit) && (scaled <= (float32_t)limit)))
    {
        return false;
    }

    *p_out = (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
    return true;
}

/** @brief Convert and range-check one signed field before encoding it.
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

/** @brief Update the owner's saturating event counters and barcode history,
 * then encode a control-debug packet from the integration snapshot and
 * available motion data.
 * @details The counters describe the observation even if construction of the
 * packet is deferred by the 200 ms throttle. Record motion state and its
 * timestamp when a snapshot is present. Mark motion measurements valid
 * only if the snapshot is fresh and the scaled values and counts fit.
 * Invalid measurements leave the corresponding wire fields at zero.
 * @return True when the throttle permits the packet and its builder succeeds.
 */
static bool capture_control_debug (const s3_integration_status_t * p_st,
                                   control_debug_packet_t *        p_packet)
{
    /* Owned by the sampling task. Static scratch avoids growing its stack. */
    static control_debug_builder_t builder;
    bool                           b_is_motion_valid = false;
    static motion_debug_snapshot_t motion;
    const uint32_t                 now  = p_st->navigation.timestamp_us;
    static const uint32_t event_bits[3] = {S3_EVENT_BARCODE, S3_EVENT_JUNCTION,
                                           S3_EVENT_LINE_LOST};

    for (unsigned event_index = 0;
         event_index < (sizeof event_bits / sizeof event_bits[0]);
         ++event_index)
    {
        if ((p_st->navigation.events & event_bits[event_index]) &&
            ((int)g_debug_events[event_index] != UINT16_MAX))
        {
            ++g_debug_events[event_index];
        }
    }

    if (p_st->navigation.barcode_status != (unsigned int)S3_BARCODE_NONE)
    {
        /* The navigation decoder statuses occupy three bits. */
        g_last_barcode_status = (uint8_t)p_st->navigation.barcode_status;
    }

    if ((g_sampling.b_has_control_tick) && ((now - g_control_last) < 200000U))
    {
        return false;
    }

    g_sampling.b_has_control_tick = true;
    g_control_last                = now;
    control_debug_builder_init(&builder);
    control_debug_builder_u(&builder, CD_TIMESTAMP_US, now);
    control_debug_builder_u(&builder, CD_SEQUENCE, g_control_sequence++);

    if (motion_try_debug_snapshot(&motion))
    {
        control_debug_builder_u(&builder, CD_MOTION_AVAILABLE, 1);
        control_debug_builder_u(&builder, CD_MOTION_TIMESTAMP_US,
                                motion.timestamp_us);
        control_debug_builder_u(&builder, CD_MOTION_STATE, motion.state);
        control_debug_builder_u(&builder, CD_MOTION_FAULT, motion.fault);
        control_debug_builder_u(&builder, CD_BRAKED, motion.b_is_braked);
        control_debug_builder_u(&builder, CD_BUSY, motion.b_is_debug_busy);
        /* Motion can preempt this task after the ADC frame was timestamped.
         * Judge freshness against the current clock, not that older frame. */
        b_is_motion_valid = (motion.b_is_debug_valid) &&
                            ((now_us() - motion.timestamp_us) <= 50000U);
        b_is_motion_valid =
            (((((((((b_is_motion_valid &&
                     (debug_builder_scaled(&builder, CD_SETPOINT_LEFT,
                                           motion.setpoint_mps[0], 1000,
                                           4095))) &&
                    (debug_builder_scaled(&builder, CD_SETPOINT_RIGHT,
                                          motion.setpoint_mps[1], 1000,
                                          4095))) &&
                   (debug_builder_scaled(&builder, CD_MEASURED_LEFT,
                                         motion.measured_mps[0], 1000,
                                         4095))) &&
                  (debug_builder_scaled(&builder, CD_MEASURED_RIGHT,
                                        motion.measured_mps[1], 1000, 4095))) &&
                 (debug_builder_scaled(&builder, CD_DUTY_LEFT, motion.duty[0],
                                       100, 100))) &&
                (debug_builder_scaled(&builder, CD_DUTY_RIGHT, motion.duty[1],
                                      100, 100))) &&
               (motion.encoder[0] >= -524288)) &&
              (motion.encoder[0] <= 524287)) &&
             (motion.encoder[1] >= -524288)) &&
            (motion.encoder[1] <= 524287);

        if (b_is_motion_valid)
        {
            control_debug_builder_s(&builder, CD_ENCODER_LEFT,
                                    motion.encoder[0]);
            control_debug_builder_s(&builder, CD_ENCODER_RIGHT,
                                    motion.encoder[1]);
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
        p_st->navigation.line.b_is_valid ? p_st->navigation.line.error : 0);
    /* Boolean values widen to 0 or 1 before entering the line flag word. */
    control_debug_builder_u(
        &builder, CD_LINE_FLAGS,
        (unsigned)p_st->navigation.line.b_is_valid |
            ((unsigned)p_st->navigation.line.b_is_line_present << 1U) |
            ((unsigned)p_st->navigation.line.b_has_junction_candidate << 2U) |
            ((unsigned)p_st->navigation.line.b_is_touching_tape << 3U) |
            ((unsigned)p_st->navigation.b_is_rearming << 4U));
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
    control_debug_builder_u(&builder, CD_COLLECTING, p_st->b_is_collecting);
    control_debug_builder_u(&builder, CD_WHITE_MIN,
                            p_st->white.count ? (int)p_st->white.minimum : 0);
    control_debug_builder_u(&builder, CD_WHITE_MAX,
                            p_st->white.count ? (int)p_st->white.maximum : 0);
    control_debug_builder_u(&builder, CD_BLACK_MIN,
                            p_st->black.count ? (int)p_st->black.minimum : 0);
    control_debug_builder_u(&builder, CD_BLACK_MAX,
                            p_st->black.count ? (int)p_st->black.maximum : 0);
    control_debug_builder_u(&builder, CD_WHITE_COUNT, p_st->white.count);
    control_debug_builder_u(&builder, CD_BLACK_COUNT, p_st->black.count);
    control_debug_builder_u(&builder, CD_MOTION_VALID, b_is_motion_valid);
    return control_debug_builder_end(&builder, p_packet);
}

#endif
/** @brief Update the active calibration window from the acquired IR frame.
 * @param p_frame Borrowed frame from this sampling cycle. It stays task-owned.
 * @note Invalid samples do not change the collector. The elapsed deadline is
 * still evaluated after each sample using the original unsigned subtraction.
 */
static inline void
collect_calibration_sample (const ir_acquisition_frame_t * p_frame)
{
    if (g_sampling.b_is_collecting)
    {
        const ir_acquisition_sample_t * p_raw =
            &p_frame->sensor[g_sampling.sensor];

        if ((p_raw->b_is_valid) && (0 == p_raw->error))
        {
            (void)ir_calibration_add(&g_collector[g_sampling.sensor],
                                     g_sampling.label, p_raw->raw);
        }

        if ((p_frame->timestamp_us - g_collect_since) >= S3_COLLECTION_US)
        {
            g_sampling.b_is_collecting = false;
        }
    }
}

/** @brief Own ADC acquisition, drive updates and status publication.
 * @details The integrated cyclic wake has a 5 ms period. Copy and clear the
 * shared command under the drive mutex. Service STOP before sampling, then
 * read motion feedback and IR, process terrain and drive state, and complete
 * any reset whose new feedback arrived. Publish copies under the existing
 * mailbox and capture locking rules. Kernel entry arguments are unused.
 * @see app_program/controller/README.md for ownership and reporting cadence.
 */
static void sensor_sample_task (INT code, void * p_unused)
{
    static ir_acquisition_frame_t   frame;
    static motion_feedback_t        feedback;
    static s3_integration_request_t command;
    static s3_integration_status_t  working;
    (void)code;
    (void)p_unused;
    (void)tk_slp_tsk(TMO_FEVR);
    const int32_t adc = ir_sensor_init(&g_drive.navigation.config.acquisition);

    for (;;)
    {
#if APP_DIAGNOSTICS
        uint32_t started_us = robot_diagnostics_now_us();
#endif
        bool                     b_is_pending   = false;
        remote_control_request_t remote_command = {0};
        const bool               b_was_running  = drive_running(g_drive.state);
        /* STOP is handled before sampling, even if an earlier command queued.
         */
        if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK)
        {
            if (g_shared.b_has_stop)
            {
                /* clang-format off */
                command      =
                    (s3_integration_request_t){.kind = S3_CONTROL_STOP};
                b_is_pending = true;
                /* clang-format on */
            }
            else if (g_shared.b_has_command)
            {
                b_is_pending =
                    integration_codec_unpack_cmd(&g_command, &command);
            }
            else
            {
                /* No control command is queued for this sample. */
            }

            /* clang-format off */
            remote_command                      = g_remote.pending;
            g_remote.b_has_active_request       =
                remote_command.b_is_request_valid;
            g_remote.pending.b_is_request_valid = false;
            g_shared.b_has_stop                 = false;
            g_shared.b_has_command              = false;
            /* clang-format on */
            (void)tk_unl_mtx(gh_drive_mutex);
        }

        if (b_is_pending && ((unsigned int)S3_CONTROL_STOP == command.kind))
        {
            g_sampling.b_waits_for_reset    = false;
            g_sampling.b_is_remote_reset    = false;
            const bool b_did_accept_request = execute(&command, now_us());

            if (remote_command.b_is_request_valid)
            {
                remote_finish(&remote_command, b_did_accept_request,
                              REMOTE_NOT_READY);
            }
            else
            {
                working.b_did_command_succeed = b_did_accept_request;
                working.command_sequence++;
            }

            b_is_pending = false;
        }

        /* Availability is carried in the zero-initialized feedback record.
         * The drive can still process the frame when feedback is absent. */
        feedback = (motion_feedback_t){0};
        (void)motion_get_feedback(&feedback);
        working.adc_status = adc;

        if (E_OK == adc)
        {
            working.adc_status = ir_sensor_read_frame(&frame);
        }

        if (working.adc_status != E_OK)
        {
            frame              = (ir_acquisition_frame_t){0};
            frame.timestamp_us = now_us();
        }
        /* ADC waits may allow a newer IMU publication than the frame start. */
        terr_link_poll(&g_drive, now_us());
        /* The drive owns its fault state. Observe it through the update
         * below even if processing this frame returned false. */
        (void)s3_drive_process(&g_drive, &frame, &feedback);
        /* A reset request is not a reset acknowledgement. The drive owner
         * waits for a newer safe feedback publication, or its 500 ms timeout.
         * Publish exactly one command result after that check, not the old
         * pre-reset FAULT snapshot. STOP above can always cancel the wait. */
        if ((g_sampling.b_waits_for_reset) &&
            (s3_drive_reset_result(&g_drive) != (unsigned int)S3_RESET_PENDING))
        {
            const bool b_did_accept_request =
                s3_drive_reset_result(&g_drive) == (unsigned int)S3_RESET_OK;

            if (b_did_accept_request)
            {
                terr_link_clear_fault();
            }

            remote_control_request_t reset = {0};

            if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK)
            {
                reset                             = g_remote.reset;
                g_remote.reset.b_is_request_valid = false;
                (void)tk_unl_mtx(gh_drive_mutex);
            }

            if (g_sampling.b_is_remote_reset)
            {
                if (reset.b_is_request_valid)
                {
                    remote_finish(&reset, b_did_accept_request,
                                  REMOTE_NOT_READY);
                }
            }
            else
            {
                working.b_did_command_succeed = b_did_accept_request;
                working.command_sequence++;
            }

            g_sampling.b_is_remote_reset = false;
            g_sampling.b_waits_for_reset = false;
            reset_completed();
        }
#if S3_BARCODE_CAPTURE
        if (b_was_running)
        {
            /* The surface and status fields use two and three bits.
             * The capture input retains both enum values in bytes. */
            const bc_capture_input_t value_in = {
                frame.timestamp_us,
                frame.sensor[S3_SENSOR_BARCODE].raw,
                g_drive.output.sensor[S3_SENSOR_BARCODE].darkness,
                (uint8_t)g_drive.output.sensor[S3_SENSOR_BARCODE].surface,
                (uint8_t)g_drive.output.barcode_status,
                g_drive.output.barcode.letter,
                (g_drive.navigation.barcode.b_is_collecting != 0U) ||
                    (bc_decoder_record_active(
                        &g_drive.navigation.barcode_record)),
                g_drive.navigation.b_is_rearming,
                feedback.b_is_feedback_valid ? feedback.centre_m : 0.0};

            append_capture_sample(&value_in);
        }
#endif
        if (b_was_running)
        {
            /* Include the interval which caused a halt/fault; do not allow
             * later idle samples to erase it or add startup noise to a run. */
            run_timing_record(&working.run_timing, g_drive.output.sample_gap_us,
                              frame.timestamp_us,
                              g_drive.navigation.config.max_sample_age_us);
        }

        collect_calibration_sample(&frame);

        if (b_is_pending)
        {
            bool b_is_permitted = (!remote_command.b_is_request_valid) ||
                                  (remote_permitted(&remote_command));
            bool b_did_accept_request =
                b_is_permitted && (execute(&command, now_us()));
            /* START changes owner state/heartbeat only: velocity is sent on
             * the next sample. If STOP/reconnect arrived during that change,
             * cancel before publishing success or reaching a velocity tick. */
            if ((b_did_accept_request && (remote_command.b_is_request_valid)) &&
                (!remote_permitted(&remote_command)))
            {
                const s3_integration_request_t stop_command = {
                    .kind = S3_CONTROL_STOP};
                (void)execute(&stop_command, now_us());
                b_did_accept_request = false;
                b_is_permitted       = false;
            }

            if ((((unsigned int)S3_CONTROL_RESET == command.kind) &&
                 b_did_accept_request) &&
                (s3_drive_reset_result(&g_drive) ==
                 (unsigned int)S3_RESET_PENDING))
            {
                g_sampling.b_waits_for_reset = true;
                g_sampling.b_is_remote_reset =
                    remote_command.b_is_request_valid;

                if ((remote_command.b_is_request_valid) &&
                    (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK))
                {
                    if ((((!g_shared.b_has_stop) &&
                          (!g_remote.b_is_stop_held)) &&
                         (g_remote.b_is_connected)) &&
                        (remote_command.epoch == g_remote.epoch))
                    {
                        g_remote.reset = remote_command;
                    }
                    else
                    {
                        remote_reply(&remote_command, REMOTE_CANCELLED,
                                     REMOTE_SUPERSEDED);
                    }

                    g_remote.b_has_active_request = false;
                    (void)tk_unl_mtx(gh_drive_mutex);
                }
            }
            else
            {
                if (remote_command.b_is_request_valid)
                {
                    remote_finish(&remote_command, b_did_accept_request,
                                  b_is_permitted ? REMOTE_NOT_READY
                                                 : REMOTE_STALE);
                }
                else
                {
                    working.b_did_command_succeed = b_did_accept_request;
                    working.command_sequence++;
                }

                if ((unsigned int)S3_CONTROL_RESET == command.kind)
                {
                    reset_completed();
                }
            }

            if (((unsigned int)S3_CONTROL_START == command.kind) &&
                b_did_accept_request)
            {
                run_timing_begin(&working.run_timing, now_us());
#if S3_BARCODE_CAPTURE
                start_capture_run();
#endif
            }
        }

        working.navigation = g_drive.output;
        /* Controls can change readiness after this cycle's observation. */
        /* clang-format off */
        working.navigation.lifecycle            = g_drive.navigation.lifecycle;
        working.navigation.accepted_calibration =
            g_drive.navigation.accepted_calibration;
        working.navigation.readiness_blockers   =
            line_navigation_readiness(&g_drive.navigation, now_us());
        /* clang-format on */

        working.motion = feedback;
        working.state  = g_drive.state;
        working.reason = g_drive.reason;
        run_timing_increment(&working.samples);

        if (g_drive.output.sample_gap_us > working.max_gap_us)
        {
            working.max_gap_us = g_drive.output.sample_gap_us;
        }

        if (g_drive.output.sample_gap_us > S3_SAMPLE_PERIOD_US)
        {
            run_timing_increment(&working.above_target);
        }

        /* clang-format off */
        working.b_is_collecting    = g_sampling.b_is_collecting;
        working.calibration_sensor = g_sampling.sensor;
        working.white              =
            ir_calibration_summary(&g_collector[g_sampling.sensor].white);
        working.black              =
            ir_calibration_summary(&g_collector[g_sampling.sensor].black);
        /* clang-format on */
        integration_codec_status_packet_t packet;
        const bool                        b_is_status_packed =
            integration_codec_pack_status(&working, &packet);

        if (!b_is_status_packed)
        {
            s3_drive_stop(&g_drive);
        }
#if APP_IR_DEBUG
        ir_debug_packet_t      ir_packet;
        control_debug_packet_t control_packet;
        const bool             b_is_control_packed =
            capture_control_debug(&working, &control_packet);
        bool b_is_ir_packed = false;

        if (ir_debug_due(frame.timestamp_us, &g_ir_last, &gb_has_ir_tick))
        {
            static const ir_acquisition_sensor_id_t order[IR_DEBUG_CHANNELS] = {
                S3_SENSOR_LEFT, S3_SENSOR_RIGHT, S3_SENSOR_BARCODE};
            ir_debug_sample_t preview = {0};
            preview.timestamp_us      = frame.timestamp_us;
            preview.sequence          = g_ir_sequence++;

            for (unsigned channel_index = 0U; channel_index < IR_DEBUG_CHANNELS;
                 ++channel_index)
            {
                const ir_acquisition_sensor_id_t value_id =
                    order[channel_index];
                const ir_acquisition_sample_t * p_raw = &frame.sensor[value_id];

                /* A channel index is below three, so the raw and
                 * normalized masks remain within one byte. The stored
                 * surface code occupies two bits. */
                if (((p_raw->b_is_enabled) && (p_raw->b_is_valid)) &&
                    (0 == p_raw->error))
                {
                    /* clang-format off */
                    preview.raw_valid          =
                        (uint8_t)((unsigned)preview.raw_valid |
                                                  (1U << channel_index));
                    preview.raw[channel_index] = p_raw->raw;
                    /* clang-format on */
                    const line_navigation_sensor_t * p_sensor =
                        &working.navigation.sensor[value_id];

                    if (p_sensor->b_is_valid)
                    {
                        /* clang-format off */
                        preview.normalised_valid        =
                            (uint8_t)((unsigned)preview.normalised_valid |
                                      (1U << channel_index));
                        preview.darkness[channel_index] = p_sensor->darkness;
                        preview.surface[channel_index]  =
                            (uint8_t)p_sensor->surface;
                        /* clang-format on */
                    }
                }
            }

            b_is_ir_packed = ir_debug_pack(&preview, &ir_packet);
        }
#endif
        uint8_t        remote_flags = REMOTE_AVAILABLE;
        const uint32_t remote_now   = now_us();

        if (drive_running(g_drive.state))
        {
            remote_flags |= (unsigned)REMOTE_RUNNING;
        }

        const bool b_is_motor_safe =
            ((((feedback.b_is_feedback_valid) && (feedback.b_is_braked)) &&
              ((unsigned int)MOTION_IDLE == feedback.state)) &&
             ((unsigned int)MOTION_FAULT_NONE == feedback.fault)) &&
            ((remote_now - feedback.timestamp_us) <= 200000U);

        if ((((((!g_sampling.b_is_collecting) &&
                (((unsigned int)S3_DRIVE_OFF == g_drive.state) ||
                 ((unsigned int)S3_DRIVE_HALTED == g_drive.state))) &&
               (S3_RUN_INACTIVE == (int)g_drive.navigation.lifecycle)) &&
              (line_navigation_readiness(&g_drive.navigation, remote_now) ==
               0U)) &&
             (terr_link_ready(remote_now))) &&
            b_is_motor_safe)
        {
            remote_flags |= (unsigned)REMOTE_CAN_START;
        }

        if (((unsigned int)S3_DRIVE_FAULT == g_drive.state) &&
            (terr_link_reset_ready(remote_now)))
        {
            remote_flags |= (unsigned)REMOTE_CAN_RESET;
        }

        if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK)
        {
            g_remote.flags      = remote_flags;
            g_remote.sampled_us = remote_now;

            if ((((g_shared.b_has_stop) || (g_shared.b_has_command)) ||
                 (g_shared.b_has_reset)) ||
                (g_remote.b_is_stop_held))
            {
                /* Clearing two bits of the byte leaves a byte value. */
                g_remote.flags = (uint8_t)((unsigned)g_remote.flags &
                                           ~((unsigned)REMOTE_CAN_START |
                                             (unsigned)REMOTE_CAN_RESET));
            }

            if (b_is_status_packed)
            {
                g_status = packet;
            }

            g_shared.b_has_status = b_is_status_packed;
#if APP_IR_DEBUG
            if (b_is_ir_packed)
            {
                g_ir_debug              = ir_packet;
                g_shared.b_has_ir_debug = true;
            }

            if (b_is_control_packed)
            {
                g_control_debug              = control_packet;
                g_shared.b_has_control_debug = true;
            }
#endif
            (void)tk_unl_mtx(gh_drive_mutex);
        }

        if (b_is_status_packed)
        {
            vehicle_controller_observe(&working);
        }
#if APP_DIAGNOSTICS
        robot_diagnostics_record(ROBOT_DIAG_IR, started_us,
                                 robot_diagnostics_now_us());
#endif
        /* Discard the accumulated wake count and sleep until the next
         * tick. The loop does not use the sleep status as sample data. */
        (void)tk_can_wup(TSK_SELF);
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

#if S3_BARCODE_CAPTURE
/** @brief Open the capture for an accepted START while holding its mutex.
 * A failed mutex operation leaves the capture and upload marker unchanged.
 */
static inline void start_capture_run (void)
{
    if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK)
    {
        bc_capture_begin_run(&g_bcap, now_us());
        gb_has_capture        = true;
        gb_is_capture_pending = false;
        (void)tk_unl_mtx(gh_drive_mutex);
    }
}

/** @brief Append one sampled observation and finish on a stopped drive.
 * @param p_capture_input Caller-owned stable input assembled before locking.
 * @note Both stored samples and the upload cursor share the drive mutex.
 */
static inline void
append_capture_sample (const bc_capture_input_t * p_capture_input)
{
    if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK)
    {
        bc_capture_record(&g_bcap, p_capture_input);

        if (!drive_running(g_drive.state))
        {
            bc_capture_end_run(&g_bcap);
            g_bcap_net_cursor     = (bc_capture_cursor_t){0};
            gb_is_capture_pending = true;
        }

        (void)tk_unl_mtx(gh_drive_mutex);
    }
}
#endif

#if S3_BARCODE_CAPTURE
/* After a run, print the barcode trace over USB, a bounded number of lines
 * per report period. Restarts whenever a USB terminal is (re)opened, so the
 * car can run untethered and be plugged in afterwards. */
/** @brief While stopped, print up to 40 capture lines per report pass using
 * the private cursor. Reopening a terminal can restart a retained
 * trace.
 */
static void report_barcode_capture (bool b_is_running)
{
    static bool                b_was_running;
    static bool                b_was_open;
    static bool                b_is_dumping_capture;
    static bc_capture_cursor_t cursor;
    static char                line[200];
    const bool                 b_is_open = bcap_terminal_open();
    unsigned int               lines;

    if (b_is_running)
    {
        b_is_dumping_capture = false;
    }
    else if ((b_was_running || (b_is_open && (!b_was_open))) && gb_has_capture)
    {
        b_is_dumping_capture = true;
        cursor               = (bc_capture_cursor_t){0};
        tm_printf((UB *)"[BCAP] barcode sensor trace follows (copy from here "
                        "to [BCAP] end)\n");
    }
    else
    {
        /* Continue with the existing capture cursor. */
    }

    b_was_running = b_is_running;
    b_was_open    = b_is_open;

    for (lines = 0U; (b_is_dumping_capture && b_is_open) &&
                     (lines < BCAP_USB_LINES_PER_POLL);
         lines++)
    {
        bool b_has_more = false;

        if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK)
        {
            b_has_more =
                bc_capture_format_next(&g_bcap, &cursor, line, sizeof line);
            (void)tk_unl_mtx(gh_drive_mutex);
        }

        if (!b_has_more)
        {
            b_is_dumping_capture = false;
            break;
        }

        tm_printf((UB *)"%s\n", line);
    }
}

#endif
/** @brief Read the latest published status after each one-second delay and
 * print the enabled diagnostic reports. The kernel entry arguments are
 * unused.
 * @details Skip this reporting interval when the snapshot is unavailable.
 * Report optional side-owner snapshots independently. Lock the capture only
 * while reading shared state. Limit its console upload to bounded batches.
 */
static void sensor_report_task (INT code, void * p_unused)
{
    static s3_integration_status_t status;
    (void)code;
    (void)p_unused;

    for (;;)
    {
        /* Pace USB operator prompts at one update per second. */
        (void)tk_dly_tsk(1000U);

        if (!s3_integration_status(&status))
        {
            continue;
        }
#if APP_S5
        s5_codec_observation_t obstacle;

        if (s5_api_snapshot(&obstacle))
        {
            const unsigned status = s5_api_controller_status();
            tm_printf((UB *)"[S5] state=%u fault=%u route=%u range_mm=%u "
                            "valid=%u no_echo=%u scan=%u profile=%u "
                            "width_mm=%u left_mm=%u right_mm=%u\n",
                      status & 7U, (status >> 3) & 7U, (status >> 6) & 3U,
                      (unsigned)obstacle.range_mm,
                      (unsigned)obstacle.b_is_range_valid,
                      (unsigned)obstacle.b_has_no_echo,
                      (unsigned)obstacle.scan_id,
                      (unsigned)obstacle.profile.b_is_valid,
                      (unsigned)obstacle.profile.width_mm,
                      (unsigned)obstacle.profile.left_clear_mm,
                      (unsigned)obstacle.profile.right_clear_mm);
        }
#endif
#if APP_S4
        terr_data_result_t terrain;
        uint16_t           terrain_run;
        uint16_t           terrain_humps;

        if (buddy4_read(&terrain, &terrain_run, &terrain_humps))
        {
            /* Published tilt is bounded to ±90 degrees and turn rate to
             * ±360 deg/s. The completed peak is nonnegative and at most
             * 0.5 m. The print conversions truncate 0.1-degree, deg/s
             * and millimetre values within the destination types. */
            tm_printf((UB *)"[S4] run=%u state=%u flags=%u pitch_x10=%d "
                            "roll_x10=%d turn_dps=%d humps=%u peak_mm=%u "
                            "impact=%u lifted=%u overturned=%u\n",
                      (unsigned)terrain_run, (unsigned)terrain.state,
                      (unsigned)terrain.flags, (int)(terrain.pitch_deg * 10),
                      (int)(terrain.roll_deg * 10), (int)terrain.turn_dps,
                      (unsigned)terrain_humps,
                      (unsigned)(terrain.peak_m * 1000),
                      (unsigned)terrain.b_has_impact,
                      (unsigned)((unsigned int)TERRAIN_SAFETY_LIFTED ==
                                 terrain.safety),
                      (unsigned)((unsigned int)TERRAIN_SAFETY_OVERTURNED ==
                                 terrain.safety));
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
            (unsigned)status.state, (unsigned)status.reason,
            (unsigned)status.navigation.readiness_blockers,
            (unsigned)status.navigation.accepted_calibration,
            motion_state_name(status.motion.state),
            (unsigned)status.motion.b_is_feedback_valid, (int)status.adc_status,
            (unsigned)status.navigation.sample_gap_us,
            (unsigned)status.max_gap_us, (unsigned)status.above_target,
            (unsigned)S3_SAMPLE_PERIOD_MS);
        tm_printf((UB *)"  RUN timing: start_us=%u active=%u samples=%u "
                        "max_gap_us=%u above_target=%u over30ms=%u "
                        "last_over30_us=%u\n",
                  (unsigned)status.run_timing.started_us,
                  (unsigned)drive_running(status.state),
                  (unsigned)status.run_timing.samples,
                  (unsigned)status.run_timing.max_gap_us,
                  (unsigned)status.run_timing.above_target,
                  (unsigned)status.run_timing.above_safety,
                  (unsigned)status.run_timing.last_safety_us);
        tm_printf(
            (UB *)"  motion_id=%u stopped=%u odom_epoch=%u report_losses=%u "
                  "motion_fault=%s\n",
            (unsigned)status.motion.command_id,
            (unsigned)status.motion.b_is_braked, (unsigned)status.motion.epoch,
            (unsigned)status.motion.report_loss_count,
            motion_fault_name(status.motion.fault));
        tm_printf((UB *)"  command=%u ok=%u collecting=%u sensor=%u white n=%u "
                        "[%u,%u] black n=%u [%u,%u]\n",
                  (unsigned)status.command_sequence,
                  (unsigned)status.b_did_command_succeed,
                  (unsigned)status.b_is_collecting,
                  (unsigned)status.calibration_sensor,
                  (unsigned)status.white.count, (unsigned)status.white.minimum,
                  (unsigned)status.white.maximum, (unsigned)status.black.count,
                  (unsigned)status.black.minimum,
                  (unsigned)status.black.maximum);
        /* Live calibrated readings: 0 = white, 1000 = black. Meaningful
         * only after that sensor's calibration has been accepted. */
        tm_printf(
            (UB *)"  IR dark: L(GP%u)=%u R(GP%u)=%u BAR(GP%u)=%u | "
                  "line present=%u error=%d junction=%u\n",
            S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_LEFT),
            (unsigned)status.navigation.sensor[S3_SENSOR_LEFT].darkness,
            S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_RIGHT),
            (unsigned)status.navigation.sensor[S3_SENSOR_RIGHT].darkness,
            S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_BARCODE),
            (unsigned)status.navigation.sensor[S3_SENSOR_BARCODE].darkness,
            (unsigned)status.navigation.line.b_is_line_present,
            (int)status.navigation.line.error,
            (unsigned)status.navigation.line.b_has_junction_candidate);
#if S3_BARCODE_CAPTURE
        {
            uint16_t decoded                        = 0U;
            uint16_t rejected                       = 0U;
            uint16_t timing                         = 0U;
            uint16_t overflow                       = 0U;
            uint8_t  episodes                       = 0U;
            char     letters[sizeof g_bcap.letters] = "-";

            if (tk_loc_mtx(gh_drive_mutex, TMO_FEVR) == E_OK)
            {
                decoded  = g_bcap.decoded;
                rejected = g_bcap.rejected;
                timing   = g_bcap.timing;
                overflow = g_bcap.overflow;
                episodes = g_bcap.episodes;

                if (g_bcap.letter_count != 0U)
                {
                    for (unsigned int letter_index = 0U;
                         letter_index < sizeof letters; letter_index++)
                    {
                        letters[letter_index] = g_bcap.letters[letter_index];
                    }
                }

                (void)tk_unl_mtx(gh_drive_mutex);
            }

            tm_printf(
                (UB *)"  BARCODE this run: decoded=%u letters=%s rejected=%u "
                      "timing_error=%u overflow=%u dark_episodes=%u\n",
                (unsigned)decoded, letters, (unsigned)rejected,
                (unsigned)timing, (unsigned)overflow, (unsigned)episodes);
        }
        report_barcode_capture(drive_running(status.state));
#endif
    }
}

/** @brief Stop and delete allocated cyclic and task resources, delete the
 * shared mutex, then clear their handles during startup failure
 * recovery.
 * @note Attempt the remaining cleanup even after a kernel operation fails.
 * The caller keeps the original startup failure as its return value.
 */
static void cleanup (void)
{
    if (gh_sample_cycle > 0)
    {
        (void)tk_stp_cyc(gh_sample_cycle);
    }

    if (gh_sample_task > 0)
    {
        (void)tk_ter_tsk(gh_sample_task);
        (void)tk_del_tsk(gh_sample_task);
    }

    if (gh_report_task > 0)
    {
        (void)tk_ter_tsk(gh_report_task);
        (void)tk_del_tsk(gh_report_task);
    }

    if (gh_sample_cycle > 0)
    {
        (void)tk_del_cyc(gh_sample_cycle);
    }

    if (gh_drive_mutex > 0)
    {
        (void)tk_del_mtx(gh_drive_mutex);
    }

    gh_sample_task = gh_report_task = gh_sample_cycle = gh_drive_mutex = 0;
}

/** @brief Raise only a positive wheel request below the configured floor. The
 * argument and return value are metres per second.
 */
static float32_t floor_wheel (float32_t mps)
{
    return ((mps > 0.0f) && (mps < S3_MIN_WHEEL_MPS)) ? S3_MIN_WHEEL_MPS : mps;
}

/** @brief Apply the forward floor independently to left and right requests in
 * metres per second and return the motion subsystem's result.
 */
static motion_result_t velocity_with_floor (float32_t left_mps,
                                            float32_t right_mps)
{
    return motion_request_velocity(floor_wheel(left_mps),
                                   floor_wheel(right_mps));
}

#if S3_BUILTIN_CALIBRATION
/* Feed the measured ranges through the SAME validation as a button
 * calibration (sample count, non-overlap, reflectance init). Returns the
 * mask of sensors accepted; any sensor that fails still needs the buttons. */
/** @brief Validate the built-in raw ADC ranges through the calibration
 * collectors and return the mask of sensors accepted by navigation.
 */
static uint32_t apply_builtin_calibration (void)
{
    static const uint16_t table[S3_SENSOR_COUNT][4] = {
        S3_BUILTIN_CAL_BARCODE, S3_BUILTIN_CAL_LEFT, S3_BUILTIN_CAL_RIGHT};
    unsigned int sensor_index;

    for (sensor_index = 0U; sensor_index < (unsigned int)S3_SENSOR_COUNT;
         sensor_index++)
    {
        ir_calibration_collector_t collector;
        ir_calibration_begin(&collector);
        (void)ir_calibration_add(&collector, IR_SURFACE_WHITE,
                                 table[sensor_index][0]);
        (void)ir_calibration_add(&collector, IR_SURFACE_BLACK,
                                 table[sensor_index][2]);

        /* Each midpoint averages two uint16_t samples. Their promoted
         * sum fits an int and the average fits uint16_t. */
        for (unsigned int midpoint_index = 0U;
             midpoint_index < CAL_MIDPOINT_SAMPLES; midpoint_index++)
        {
            (void)ir_calibration_add(
                &collector, IR_SURFACE_WHITE,
                (uint16_t)((unsigned)(table[sensor_index][0] +
                                      table[sensor_index][1]) /
                           2U));
            (void)ir_calibration_add(
                &collector, IR_SURFACE_BLACK,
                (uint16_t)((unsigned)(table[sensor_index][2] +
                                      table[sensor_index][3]) /
                           2U));
        }

        (void)ir_calibration_add(&collector, IR_SURFACE_WHITE,
                                 table[sensor_index][1]);
        (void)ir_calibration_add(&collector, IR_SURFACE_BLACK,
                                 table[sensor_index][3]);
        /* This loop visits sensor selectors zero through two. Each
         * index is an existing ir_acquisition_sensor_id_t value. */
        (void)line_navigation_set_calibration(
            &g_drive.navigation, (ir_acquisition_sensor_id_t)sensor_index,
            &collector);
    }

    return g_drive.navigation.accepted_calibration;
}

#endif

#else
/** @brief Ignore link changes when motion integration is disabled.
 */
void remote_control_link (uint32_t epoch, bool b_is_up)
{
    (void)epoch;
    (void)b_is_up;
}

/** @brief Discard all command bytes when motion integration is disabled.
 */
void remote_control_receive (const uint8_t * p_packet, unsigned length)
{
    (void)p_packet;
    (void)length;
}

/** @brief Return zero bytes and leave the output buffer untouched.
 */
unsigned remote_control_reply_take (uint8_t * p_packet, unsigned capacity)
{
    (void)p_packet;
    (void)capacity;
    return 0;
}

/** @brief Return zero bytes and leave the status buffer untouched.
 */
unsigned remote_control_status_take (uint8_t * p_packet, unsigned capacity)
{
    (void)p_packet;
    (void)capacity;
    return 0;
}

/** @brief Return zero in the disabled profile.
 */
uint32_t remote_control_stop_sequence (void)
{
    return 0;
}

/** @brief Ignore the physical button state in the disabled profile.
 */
void remote_control_physical_stop (bool b_is_held)
{
    (void)b_is_held;
}

/** @brief Return false without queuing a request.
 */
bool s3_integration_submit (const s3_integration_request_t * p_request)
{
    (void)p_request;
    return false;
}

/** @brief Return false without writing the caller's status.
 */
bool s3_integration_status (s3_integration_status_t * p_status)
{
    (void)p_status;
    return false;
}

/** @brief Return E_NOSPT when integrated motion is disabled.
 */
int32_t s3_integration_init (void)
{
    return E_NOSPT;
}

/** @brief Return E_NOSPT without starting tasks.
 */
int32_t s3_integration_start (void)
{
    return E_NOSPT;
}
#endif

/* End of file. */

