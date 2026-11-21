/** @file buddy4.c
 * @brief Own IMU acquisition, terrain processing and protected snapshots and
 * events.
 * @see app_program/subsystem4/README.md for the terrain data path.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 * Unsigned casts spell out the original modulo conversion.
 */
#include "buddy4.h"
#include "../build_assert.h"
#include "../app_types.h"
#ifndef APP_S4
#define APP_S4 0
#endif
#ifndef APP_S4_BENCH
#define APP_S4_BENCH 0
#endif
#if APP_S4
#ifdef BUDDY4_TRANSPORT_TEST
#include "transport_main.h"
#endif
#include <sys/sysdef.h>
#include <tk/device.h>
#if defined(BUDDY4_TRANSPORT_TEST) && defined(__clang__)
/* The BSP header has no final newline. */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wnewline-eof"
#endif
#include "../../device/include/dev_i2c.h"
#if defined(BUDDY4_TRANSPORT_TEST) && defined(__clang__)
#pragma clang diagnostic pop
#endif
#include "../subsystem2/motion.h"
#if APP_IMU_DEBUG
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"
#endif
/* The host transport fixture substitutes these three operations. */
#ifdef BUDDY4_TRANSPORT_TEST
#define BUDDY4_IMU_INIT buddy4_transport_imu_init
#define BUDDY4_IMU_READ buddy4_transport_imu_read
#define BUDDY4_TERRAIN_PROCESS buddy4_transport_observe
#else
#define BUDDY4_IMU_INIT imu_sensor_init
#define BUDDY4_IMU_READ imu_sensor_read
#define BUDDY4_TERRAIN_PROCESS terr_data_process
#endif
/* This car's test obstacle raises the right wheel only. Keep the portable
 * monitor's longitudinal default available through an explicit build choice. */
#ifndef S4_HUMP_GEOMETRY
#define S4_HUMP_GEOMETRY TERRAIN_HUMP_RIGHT_WHEEL
#endif
#ifndef S4_WHEEL_TRACK_M
/* Wheel-centre spacing; matches Buddy 2 TRACK_WIDTH_M (116 mm). */
#define S4_WHEEL_TRACK_M (0.116f)
#endif
#define S4_QUEUE_LEN 8U
/* Wake every kernel tick to check for the accelerometer's 50 Hz output. */
#define S4_IMU_POLL_MS 10U

/* All fields in this packed state are read and updated under gh_terrain_mutex.
 * Head is 0..7; count includes the full eight-event state. */
typedef struct
{
    uint16_t head : 3;
    uint16_t count : 4;
    bool     b_has_published : 1;
    bool     b_has_lost_events : 1;
    bool     b_is_reset_pending : 1;
    bool     b_should_recalibrate : 1;
    bool     b_has_pending_mag_cal : 1;
} subsystem4_queue_state_t;

#if APP_IMU_DEBUG
#define IMU_DEBUG_VERSION 2U
#define IMU_DEBUG_HEADER_BYTES 12U
#define IMU_DEBUG_ROW_BYTES 28U
#define IMU_DEBUG_BATCH_ROWS 5U
#define IMU_DEBUG_PACKET_BYTES                                                 \
    (IMU_DEBUG_HEADER_BYTES + IMU_DEBUG_BATCH_ROWS * IMU_DEBUG_ROW_BYTES)
#define IMU_DEBUG_QUEUE_ROWS 128U
#define IMU_DEBUG_FLUSH_US 250000U

typedef enum
{
    IMU_DEBUG_VALID     = 1U << 0,
    IMU_DEBUG_EVALUATED = 1U << 1,
    IMU_DEBUG_OVERRUN   = 1U << 2,
    IMU_DEBUG_IMPACT    = 1U << 3,
    IMU_DEBUG_PENDING   = 1U << 4,
    IMU_DEBUG_UPWARD    = 1U << 5,
    IMU_DEBUG_TILTED    = 1U << 6
} buddy4_debug_flag_t;

typedef struct
{
    uint32_t timestamp_us;
    uint32_t sequence;
    int32_t  wheel_delta_um;
    int32_t  hump_travel_um;
    int16_t  raw_mg[3];
    int16_t  calibrated_mg[3];
    uint16_t run;
    uint8_t  flags;
    uint8_t  safety;
    uint8_t  hump_flags;
    uint8_t  hump_reason;
} imu_debug_row_t;

/* The largest IMU debug packet must fit in MQTT staging. */
BUILD_ASSERT(IMU_DEBUG_PACKET_BYTES <= MQTT_UTK_IMU_DEBUG_MAX,
             buddy4_c_check_a_t);
#endif

static ID      gh_terrain_mutex;
static ID      gh_terrain_task;
static ID      gh_terrain_cycle;
static ID      gh_imu_device;
static uint8_t g_snapshot[TERRAIN_PACKET_SIZE];
static uint8_t g_events[S4_QUEUE_LEN][TERRAIN_PACKET_SIZE];

static subsystem4_queue_state_t g_queue;
static uint16_t                 g_run;
static uint16_t                 g_published_run;
static uint16_t                 g_humps;
static float32_t                g_offset[3];
static float32_t                g_scale[3];

#if APP_IMU_DEBUG
static ID              gh_imu_debug_mutex;
static imu_debug_row_t g_imu_debug_rows[IMU_DEBUG_QUEUE_ROWS];
static unsigned        g_imu_debug_head;
static unsigned        g_imu_debug_count;
static uint32_t        g_imu_debug_sequence;
static uint32_t        g_imu_debug_boot_tag;

#endif

static uint32_t now_us (void);
static bool bus_read (void * p_ctx, uint8_t addr, uint8_t reg, uint8_t * p_out,
                      unsigned byte_count);
static bool bus_write (void * p_ctx, uint8_t addr, uint8_t reg, uint8_t value);
static bool publish (const terr_data_result_t * p_terrain, uint16_t * p_run);
static void process_sample (terr_data_monitor_t *        p_monitor,
                            const imu_sensor_sample_t *  p_sample,
                            const terr_data_odometry_t * p_odometry);
static void imu_poll_tick (void * p_unused);
static void terrain_task (INT stacd, void * p_exinf);
static bool request_reset (bool b_should_recalibrate, uint16_t * p_token);

#if APP_IMU_DEBUG
static int16_t imu_debug_mg (float32_t acceleration_g);
static int32_t imu_debug_um (float32_t metres);
static void    imu_debug_record (const imu_sensor_sample_t * p_sample,
                                 const terr_data_monitor_t * p_monitor,
                                 uint16_t                    run);
static void    imu_debug_put16 (uint8_t * p_bytes, uint16_t value);
static void    imu_debug_put32 (uint8_t * p_bytes, uint32_t value);

/** @brief Take up to five consecutive rows from one run under the debug mutex.
 * A short tail waits up to 250 ms. Return the encoded byte count, or
 * zero without consuming rows when the buffer, lock or batch is
 * unavailable.
 */
unsigned int mqtt_utk_imu_debug_take (uint8_t * p_bytes, unsigned int size)
{
    if ((((0 == p_bytes) || (size < IMU_DEBUG_PACKET_BYTES)) ||
         (gh_imu_debug_mutex <= 0)) ||
        (tk_loc_mtx(gh_imu_debug_mutex, TMO_POL) != E_OK))
    {
        return 0;
    }

    if (0U == g_imu_debug_count)
    {
        tk_unl_mtx(gh_imu_debug_mutex);
        return 0;
    }

    const imu_debug_row_t * p_first_row = &g_imu_debug_rows[g_imu_debug_head];
    unsigned                count       = 1;

    while ((count < IMU_DEBUG_BATCH_ROWS) && (count < g_imu_debug_count))
    {
        const imu_debug_row_t * p_next_row =
            &g_imu_debug_rows[(g_imu_debug_head + count) %
                              IMU_DEBUG_QUEUE_ROWS];

        if ((p_next_row->run != p_first_row->run) ||
            (p_next_row->sequence != (p_first_row->sequence + count)))
        {
            break;
        }

        ++count;
    }

    if (((count < IMU_DEBUG_BATCH_ROWS) && (count == g_imu_debug_count)) &&
        ((now_us() - p_first_row->timestamp_us) < IMU_DEBUG_FLUSH_US))
    {
        tk_unl_mtx(gh_imu_debug_mutex);
        return 0;
    }

    /* The batch cap keeps the row count within one byte. */
    p_bytes[0] = IMU_DEBUG_VERSION;
    p_bytes[1] = (uint8_t)count;
    imu_debug_put16(p_bytes + 2, p_first_row->run);
    imu_debug_put32(p_bytes + 4, p_first_row->sequence);
    imu_debug_put32(p_bytes + 8, g_imu_debug_boot_tag);
    uint8_t * p_output  = p_bytes + IMU_DEBUG_HEADER_BYTES;
    unsigned  row_index = 0;

    while (row_index < count)
    {
        const imu_debug_row_t * p_row =
            &g_imu_debug_rows[(g_imu_debug_head + row_index) %
                              IMU_DEBUG_QUEUE_ROWS];
        imu_debug_put32(p_output, p_row->timestamp_us);

        /* Convert signed acceleration to its low 16 bits, preserving the
         * wire representation for both positive and negative readings. */
        for (unsigned axis = 0; axis < (unsigned)IMU_AXIS_COUNT; ++axis)
        {
            imu_debug_put16(p_output + 4 + (2U * axis),
                            (uint16_t)p_row->raw_mg[axis]);
            imu_debug_put16(p_output + 10 + (2U * axis),
                            (uint16_t)p_row->calibrated_mg[axis]);
        }

        /* The signed distances use their low 32 bits. The reserved
         * INT32_MIN value retains its bit pattern for unavailable data. */
        p_output[16] = p_row->flags;
        p_output[17] = p_row->safety;
        imu_debug_put32(p_output + 18, (uint32_t)p_row->wheel_delta_um);
        imu_debug_put32(p_output + 22, (uint32_t)p_row->hump_travel_um);
        p_output[26] = p_row->hump_flags;
        p_output[27] = p_row->hump_reason;
        ++row_index;
        p_output += IMU_DEBUG_ROW_BYTES;
    }

    g_imu_debug_head   = (g_imu_debug_head + count) % IMU_DEBUG_QUEUE_ROWS;
    g_imu_debug_count -= count;
    tk_unl_mtx(gh_imu_debug_mutex);
    return IMU_DEBUG_HEADER_BYTES + (count * IMU_DEBUG_ROW_BYTES);
}
#endif

/** @brief Create the terrain mutex once and return the kernel result. A failed
 * optional debug mutex leaves terrain acquisition available.
 */
ER subsystem4_init (void)
{
    T_CMTX mutex_spec = {0};

    if (gh_terrain_mutex > 0)
    {
        return E_OBJ;
    }

    mutex_spec.mtxatr = TA_INHERIT;
    gh_terrain_mutex  = tk_cre_mtx(&mutex_spec);
#if APP_IMU_DEBUG
    /* Optional logging must never prevent the safety task from starting. */
    if (gh_terrain_mutex > 0)
    {
        gh_imu_debug_mutex = tk_cre_mtx(&mutex_spec);

        if (gh_imu_debug_mutex < 0)
        {
            tm_printf((UB *)"[S4] IMU debug queue unavailable\n");
        }
    }
#endif
    return gh_terrain_mutex < 0 ? gh_terrain_mutex : E_OK;
}

/** @brief Create the owner task and its 10 ms cyclic wakeup. Return E_OBJ for
 * invalid startup order. Release newly created task and timer
 * resources when timer or task startup fails.
 * @note Preserve the creation or start error across best-effort rollback.
 * Stop the timer before deleting it if task startup fails afterward.
 */
ER subsystem4_start (void)
{
    T_CTSK task_spec = {0};
    T_CCYC cycle     = {0};
    ER     value_er;

    if ((gh_terrain_mutex <= 0) || (gh_terrain_task > 0))
    {
        return E_OBJ;
    }

    /* T_CTSK uses the kernel FP address type. The startup routine
     * receives an INT start code and void * context. See tk/syscall.h. */
    task_spec.task    = (FP)terrain_task;
    task_spec.itskpri = S4_PRI_MAIN;
    task_spec.stksz   = 4096;
    task_spec.tskatr  = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    task_spec.tskatr |= TA_ASSPRC;
    task_spec.assprc  = TP_PRC1;
#endif
    gh_terrain_task = tk_cre_tsk(&task_spec);

    if (gh_terrain_task < 0)
    {
        return gh_terrain_task;
    }

    /* T_CCYC uses the kernel FP address type. The cyclic handler receives
     * one void * extended-information argument. See tk/syscall.h. */
    cycle.cycatr     = TA_HLNG;
    cycle.cychdr     = (FP)imu_poll_tick;
    cycle.cyctim     = S4_IMU_POLL_MS;
    cycle.cycphs     = S4_IMU_POLL_MS;
    gh_terrain_cycle = tk_cre_cyc(&cycle);

    if (gh_terrain_cycle <= 0)
    {
        value_er = gh_terrain_cycle;
    }
    else
    {
        value_er                      = tk_sta_cyc(gh_terrain_cycle);
        const bool b_is_cycle_started = E_OK == value_er;

        if (b_is_cycle_started)
        {
            value_er = tk_sta_tsk(gh_terrain_task, 0);
        }

        if (E_OK == value_er)
        {
            return E_OK;
        }

        if (b_is_cycle_started)
        {
            (void)tk_stp_cyc(gh_terrain_cycle);
        }

        (void)tk_del_cyc(gh_terrain_cycle);
    }

    (void)tk_del_tsk(gh_terrain_task);
    gh_terrain_task  = 0;
    gh_terrain_cycle = 0;
    return value_er;
}

/** @brief Poll the mutex for a coherent result, run token and hump count. A
 * pending reset, missing publication, null output or busy lock leaves
 * outputs untouched. A snapshot older than 150 ms still succeeds with
 * fault and unknown state.
 */
bool buddy4_read (terr_data_result_t * p_out, uint16_t * p_token,
                  uint16_t * p_humps)
{
    if (((((0 == p_out) || (0 == p_token)) || (0 == p_humps)) ||
         (gh_terrain_mutex <= 0)) ||
        (tk_loc_mtx(gh_terrain_mutex, TMO_POL) != E_OK))
    {
        return false;
    }

    if (((!g_queue.b_has_published) || (g_queue.b_is_reset_pending)) ||
        (g_published_run != g_run))
    {
        tk_unl_mtx(gh_terrain_mutex);
        return false;
    }

    terr_data_unpack(g_snapshot, p_out);
    *p_token = g_published_run;
    *p_humps = g_humps;
    tk_unl_mtx(gh_terrain_mutex);

    if ((now_us() - p_out->timestamp_us) > 150000U)
    {
        /* clang-format off */
        p_out->flags              =
            (p_out->flags & (TERRAIN_PEAK_VALID | TERRAIN_CALIBRATED)) |
            TERRAIN_FAULT;
        p_out->state              = TERRAIN_UNKNOWN;
        p_out->b_has_impact       = false;
        p_out->b_is_hump_complete = false;
        /* clang-format on */
    }

    return true;
}

/** @brief Read the latest terrain result with the same freshness and lock
 * rules as buddy4_read. Discard its run token and count.
 */
bool buddy4_snapshot (terr_data_result_t * p_out)
{
    uint16_t token;
    uint16_t humps;
    return buddy4_read(p_out, &token, &humps);
}

/** @brief Pop the oldest event and return whether a prior overflow lost
 * events. Clear the loss flag only on delivery. An empty queue,
 * unavailable lock or missing output returns false without writing
 * either output.
 */
bool buddy4_event (terr_data_result_t * p_out, bool * pb_has_lost_events)
{
    bool b_did_succeed = false;

    if ((((0 == p_out) || (0 == pb_has_lost_events)) ||
         (gh_terrain_mutex <= 0)) ||
        (tk_loc_mtx(gh_terrain_mutex, TMO_POL) != E_OK))
    {
        return false;
    }

    if (g_queue.count)
    {
        terr_data_unpack(g_events[g_queue.head], p_out);
        /* Widen the bounded queue index before wrapping it. */
        g_queue.head = ((unsigned)g_queue.head + 1U) % S4_QUEUE_LEN;
        --g_queue.count;
        *pb_has_lost_events       = g_queue.b_has_lost_events;
        g_queue.b_has_lost_events = false;
        b_did_succeed             = true;
    }

    tk_unl_mtx(gh_terrain_mutex);
    return b_did_succeed;
}

/** @brief Request a new run, optionally restarting level calibration. True
 * means the owner accepted a pending reset.
 */
bool buddy4_reset_run (bool b_should_recalibrate)
{
    return request_reset(b_should_recalibrate, NULL);
}

/** @brief Request a reset that retains level calibration and return its token
 * through p_token. A null pointer or rejected request returns false.
 */
bool buddy4_begin_run (uint16_t * p_token)
{
    return p_token && (request_reset(false, p_token));
}

/** @brief Validate three finite axis offsets and scale factors using the
 * terrain monitor, then queue a private copy under the mutex. Invalid
 * inputs or contention leave pending calibration intact.
 */
bool buddy4_set_mag_calibration (const float32_t p_offset[3],
                                 const float32_t p_scale[3])
{
    terr_data_monitor_t check;
    terr_data_init(&check);

    if (((!terr_data_set_mag_cal(&check, p_offset, p_scale)) ||
         (gh_terrain_mutex <= 0)) ||
        (tk_loc_mtx(gh_terrain_mutex, TMO_POL) != E_OK))
    {
        return false;
    }

    {
        unsigned axis_index;

        for (axis_index = 0; axis_index < (unsigned int)IMU_AXIS_COUNT;
             ++axis_index)
        {
            g_offset[axis_index] = p_offset[axis_index];
            g_scale[axis_index]  = p_scale[axis_index];
        }
    }
    g_queue.b_has_pending_mag_cal = true;
    tk_unl_mtx(gh_terrain_mutex);
    return true;
}

#ifdef BUDDY4_TRANSPORT_TEST
/** @brief Inject a terrain publication into the original queue path. */
void buddy4_transport_publish (const terr_data_result_t * p_terrain)
{
    (void)publish(p_terrain, NULL);
}

/** @brief Process one caller-owned IMU sample with its matching odometry. */
void buddy4_transport_process (terr_data_monitor_t *        p_monitor,
                               const imu_sensor_sample_t *  p_sample,
                               const terr_data_odometry_t * p_odometry)
{
    process_sample(p_monitor, p_sample, p_odometry);
}

/** @brief Return the selected geometry for the fixture's publication check. */
terr_data_hump_geometry_t buddy4_transport_geometry (void)
{
    return S4_HUMP_GEOMETRY;
}

/** @brief Return the configured wheel track in metres. */
float32_t buddy4_transport_track_m (void)
{
    return S4_WHEEL_TRACK_M;
}

/** @brief Seed the count at a saturation boundary while the owner is idle. */
void buddy4_transport_set_hump_count (uint16_t count)
{
    g_humps = count;
}

/** @brief Seed a run token before testing its rollover. */
void buddy4_transport_set_run (uint16_t token)
{
    g_run = token;
}

#if APP_IMU_DEBUG
/** @brief Clear pending debug rows in the single-threaded fixture. */
void buddy4_transport_clear_debug (void)
{
    g_imu_debug_head  = 0;
    g_imu_debug_count = 0;
}

/** @brief Return the debug sequence without consuming a row. */
uint32_t buddy4_transport_debug_sequence (void)
{
    return g_imu_debug_sequence;
}

/** @brief Return the terrain run token used in subsequent debug rows. */
uint16_t buddy4_transport_debug_run (void)
{
    return g_run;
}

/** @brief Return the configured row capacity of the debug queue. */
unsigned buddy4_transport_debug_capacity (void)
{
    return IMU_DEBUG_QUEUE_ROWS;
}

/** @brief Add a diagnostic row without advancing the terrain estimator. */
void buddy4_transport_record_debug (const imu_sensor_sample_t * p_sample,
                                    const terr_data_monitor_t * p_monitor)
{
    imu_debug_record(p_sample, p_monitor, g_run);
}
#endif
#endif

/** @brief Read the low 32 bits of the hardware clock in microseconds.
 * Elapsed-time checks use unsigned subtraction across wrap.
 */
static uint32_t now_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

#if APP_IMU_DEBUG
/** @brief Round a sensor acceleration in g to the nearest milligravity, with
 * ties away from zero. The sensor and calibration bounds keep the
 * result in int16_t range.
 */
static int16_t imu_debug_mg (float32_t acceleration_g)
{
    /* The sensor spans -4 to +4 g. Level calibration adds at most 0.15 g. */
    float32_t milligravity = acceleration_g * 1000.0f;
    return (int16_t)(milligravity + (milligravity < 0.0f ? -0.5f : 0.5f));
}

/** @brief Round metres to signed micrometres for the debug packet. Return
 * INT32_MIN for an unordered or out-of-range scaled value.
 */
static int32_t imu_debug_um (float32_t metres)
{
    float32_t micrometres = metres * 1000000.0f;
    /* Leave INT32_MIN available as a sentinel for an unavailable distance. */
    if (!((micrometres > -2147483648.0f) && (micrometres < 2147483648.0f)))
    {
        return INT32_MIN;
    }

    return (int32_t)(micrometres + (micrometres < 0.0f ? -0.5f : 0.5f));
}

/** @brief Build one row for the published run. The terrain owner advances the
 * sequence before polling the debug mutex. Contention drops the row
 * and a full queue discards its oldest row.
 */
static void imu_debug_record (const imu_sensor_sample_t * p_sample,
                              const terr_data_monitor_t * p_monitor,
                              uint16_t                    run)
{
    if (gh_imu_debug_mutex <= 0)
    {
        return;
    }

    /* The calibrated flag is set only when this sample reached the detector. */
    const bool b_is_evaluated =
        (p_monitor->result.flags & TERRAIN_CALIBRATED) != 0U;
    /* The first four flags use bits zero through three. Safety is
     * the two-bit monitor code. Both values fit their output bytes. */
    const bool      b_is_valid = p_sample->b_is_valid;
    imu_debug_row_t row        = {
               .timestamp_us   = p_sample->timestamp_us,
               .sequence       = g_imu_debug_sequence++,
               .wheel_delta_um = INT32_MIN,
               .hump_travel_um = INT32_MIN,
               .run            = run,
               .flags =
            (uint8_t)((b_is_valid ? IMU_DEBUG_VALID : 0) |
                      (b_is_evaluated ? IMU_DEBUG_EVALUATED : 0) |
                      (p_sample->b_has_overrun ? IMU_DEBUG_OVERRUN : 0) |
                      (p_monitor->result.b_has_impact ? IMU_DEBUG_IMPACT : 0)),
               .safety = (uint8_t)p_monitor->result.safety};

    if ((unsigned)TERRAIN_HUMP_RIGHT_WHEEL == p_monitor->hump_geometry)
    {
        if (p_monitor->hump.debug.b_is_wheel_delta_valid)
        {
            row.wheel_delta_um =
                imu_debug_um(p_monitor->hump.debug.wheel_delta_m);
        }

        row.hump_travel_um = imu_debug_um(p_monitor->hump.debug.travel_m);
        row.hump_flags     = p_monitor->hump.debug.flags;
        row.hump_reason    = p_monitor->hump.debug.reason;
    }

    if (b_is_evaluated)
    {
        /* Each additional mask is 16, 32 or 64 and fits the row byte.
         * Convert each mask to unsigned before merging it. */
        if ((p_monitor->b_is_safety_tracking) &&
            ((unsigned)TERRAIN_SAFETY_NONE == p_monitor->result.safety))
        {
            row.flags |= (unsigned)IMU_DEBUG_PENDING;
        }

        if (p_monitor->b_is_moving_upward)
        {
            row.flags |= (unsigned)IMU_DEBUG_UPWARD;
        }

        if (p_monitor->b_has_lift_tilt)
        {
            row.flags |= (unsigned)IMU_DEBUG_TILTED;
        }
    }

    for (unsigned axis = 0; axis < (unsigned)IMU_AXIS_COUNT; ++axis)
    {
        if (b_is_valid)
        {
            row.raw_mg[axis] = imu_debug_mg(p_sample->accel_g[axis]);
        }

        if (b_is_evaluated)
        {
            /* Match the unfiltered vector used in safety_update(). */
            row.calibrated_mg[axis] = imu_debug_mg(
                p_sample->accel_g[axis] - p_monitor->calibration.bias_g[axis]);
        }
    }

    /* Debug contention may cost a row, detectable by its sequence gap. */
    if (tk_loc_mtx(gh_imu_debug_mutex, TMO_POL) != E_OK)
    {
        return;
    }

    if (0U == g_imu_debug_boot_tag)
    {
        /* A best-effort session tag from startup timing and the first reading.
         */
        uint32_t tag = row.timestamp_us;

        for (unsigned axis = 0; axis < (unsigned)IMU_AXIS_COUNT; ++axis)
        {
            /* Mix each signed sample's low 16 bits into the boot tag. */
            tag = (tag ^ (uint16_t)row.raw_mg[axis]) * 16777619U;
        }

        g_imu_debug_boot_tag = tag | 1U;
    }

    if (IMU_DEBUG_QUEUE_ROWS == g_imu_debug_count)
    {
        g_imu_debug_head = (g_imu_debug_head + 1U) % IMU_DEBUG_QUEUE_ROWS;
        --g_imu_debug_count;
    }

    unsigned tail =
        (g_imu_debug_head + g_imu_debug_count) % IMU_DEBUG_QUEUE_ROWS;
    g_imu_debug_rows[tail] = row;
    ++g_imu_debug_count;
    tk_unl_mtx(gh_imu_debug_mutex);
}

/** @brief Write both bytes of value least significant first. The caller
 * supplies two writable bytes.
 */
static void imu_debug_put16 (uint8_t * p_bytes, uint16_t value)
{
    /* The first conversion retains the low octet. The unsigned shift
     * puts the high octet within 0..255 before the second conversion. */
    p_bytes[0] = (uint8_t)value;
    p_bytes[1] = (uint8_t)((unsigned)value >> 8U);
}

/** @brief Write four bytes to the caller's buffer in little-endian order.
 */
static void imu_debug_put32 (uint8_t * p_bytes, uint32_t value)
{
    /* Store the low half-word followed by the high half-word. The
     * first conversion discards the high bits. The second retains them. */
    imu_debug_put16(p_bytes, (uint16_t)value);
    imu_debug_put16(p_bytes + 2, (uint16_t)(value >> 16));
}
#endif

/** @brief Read byte_count bytes from one register into p_out using the open I2C
 * device. The bus context is unused. Success requires E_OK and a
 * complete execute request.
 */
static bool bus_read (void * p_ctx, uint8_t addr, uint8_t reg, uint8_t * p_out,
                      unsigned byte_count)
{
    SZ         actual  = 0;
    T_I2C_EXEC request = {0};
    (void)p_ctx;
    request.sadr     = addr;
    request.snd_size = 1;
    request.snd_data = &reg;
    request.rcv_size = (SZ)byte_count;
    request.rcv_data = p_out;
    ER value_er      = tk_swri_dev(gh_imu_device, TDN_I2C_EXEC, &request,
                                   sizeof(request), &actual);
    return (E_OK == value_er) && (actual == (SZ)sizeof(request));
}

/** @brief Send a register and its one-byte value to the addressed I2C device.
 * The context is unused. Return true only for a successful two-byte
 * write.
 */
static bool bus_write (void * p_ctx, uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t write_bytes[2] = {reg, value};
    SZ      actual         = 0;
    (void)p_ctx;
    ER value_er = tk_swri_dev(gh_imu_device, addr, write_bytes, 2, &actual);
    return (E_OK == value_er) && (2 == actual);
}

/** @brief Under the terrain mutex, publish the latest result and count
 * completed valid humps up to UINT16_MAX. Queue changes, impact or
 * completion, evicting the oldest event on overflow. On success,
 * optionally return the current run token. A pending reset or failed
 * lock rejects publication.
 */
static bool publish (const terr_data_result_t * p_terrain, uint16_t * p_run)
{
    if (tk_loc_mtx(gh_terrain_mutex, TMO_FEVR) != E_OK)
    {
        return false;
    }

    terr_data_result_t previous;

    if (g_queue.b_is_reset_pending)
    {
        tk_unl_mtx(gh_terrain_mutex);
        return false;
    }

    terr_data_unpack(g_snapshot, &previous);

    if (((((p_terrain->state != previous.state) ||
           (p_terrain->flags != previous.flags)) ||
          (p_terrain->safety != previous.safety)) ||
         (p_terrain->b_is_hump_complete)) ||
        (p_terrain->b_has_impact))
    {
        if (S4_QUEUE_LEN == g_queue.count)
        {
            /* Widen the bounded queue index before wrapping it. */
            g_queue.head = ((unsigned)g_queue.head + 1U) % S4_QUEUE_LEN;
            --g_queue.count;
            g_queue.b_has_lost_events = true;
        }

        /* Head and count stay in the nonnegative queue domain. */
        terr_data_pack(
            p_terrain,
            g_events[(unsigned)(g_queue.head + g_queue.count) % S4_QUEUE_LEN]);
        ++g_queue.count;
    }

    if (((p_terrain->b_is_hump_complete) &&
         (p_terrain->flags & TERRAIN_PEAK_VALID)) &&
        ((int)g_humps < UINT16_MAX))
    {
        ++g_humps;
    }

    terr_data_pack(p_terrain, g_snapshot);
    g_published_run         = g_run;
    g_queue.b_has_published = true;

    if (p_run)
    {
        *p_run = g_published_run;
    }

    tk_unl_mtx(gh_terrain_mutex);
    return true;
}

/** @brief Update the owner's monitor with one timestamped IMU sample and its
 * odometry, then publish the result. In the debug profile, only a
 * successful publication can enqueue an IMU row.
 */
static void process_sample (terr_data_monitor_t *        p_monitor,
                            const imu_sensor_sample_t *  p_sample,
                            const terr_data_odometry_t * p_odometry)
{
    BUDDY4_TERRAIN_PROCESS(p_monitor, p_sample, p_odometry);
#if APP_IMU_DEBUG
    uint16_t run;

    if (publish(&p_monitor->result, &run))
    {
        imu_debug_record(p_sample, p_monitor, run);
    }
#else
    (void)publish(&p_monitor->result, NULL);
#endif
}

/** @brief Wake the terrain task on a cyclic timer tick. The kernel callback
 * argument is unused.
 */
static void imu_poll_tick (void * p_unused)
{
    (void)p_unused;
    (void)tk_wup_tsk(gh_terrain_task);
}

/** @brief Own the IMU bus and terrain monitor. Apply queued resets and
 * calibration before sampling. Retry a disconnected sensor at
 * one-second intervals and invalidate prolonged read failures. Drain
 * accumulated wakes before sleeping for the next tick.
 */
static void terrain_task (INT stacd, void * p_exinf)
{
    terr_data_monitor_t monitor;
    imu_sensor_bus_t    bus            = {NULL, bus_read, bus_write};
    uint32_t            last_good      = 0;
    uint32_t            report_at      = 0;
    uint32_t            retry_at       = 0;
    bool                b_is_connected = false;
    (void)stacd;
    (void)p_exinf;
    terr_data_init(&monitor);

    if (!terr_data_set_geometry(&monitor, S4_HUMP_GEOMETRY, S4_WHEEL_TRACK_M))
    {
        tm_printf((UB *)"[S4] invalid hump geometry configuration\n");
        (void)tk_stp_cyc(gh_terrain_cycle);
        tk_ext_tsk();
        return;
    }

    gh_imu_device = tk_opn_dev((UB *)"iicb", TD_UPDATE);

    if (gh_imu_device < 0)
    {
        tm_printf((UB *)"[S4] iicb open failed: %d\n", gh_imu_device);
        (void)tk_stp_cyc(gh_terrain_cycle);
        tk_ext_tsk();
        return;
    }

    for (;;)
    {
        const uint32_t       now    = now_us();
        imu_sensor_sample_t  sample = {0};
        terr_data_odometry_t odo    = {0};

        if (tk_loc_mtx(gh_terrain_mutex, TMO_FEVR) == E_OK)
        {
            if (g_queue.b_is_reset_pending)
            {
                terr_data_reset_run(&monitor);

                if (g_queue.b_should_recalibrate)
                {
                    imu_calibration_reset(&monitor.calibration);
                }

                g_queue.b_is_reset_pending   = false;
                g_queue.b_should_recalibrate = false;
                g_queue.count                = 0;
                g_queue.head                 = 0;
                g_queue.b_has_lost_events    = false;
                g_humps                      = 0;
            }

            if (g_queue.b_has_pending_mag_cal)
            {
                terr_data_set_mag_cal(&monitor, g_offset, g_scale);
                g_queue.b_has_pending_mag_cal = false;
            }

            tk_unl_mtx(gh_terrain_mutex);
        }

        if ((!b_is_connected) &&
            ((0U == retry_at) || ((now - retry_at) >= 1000000U)))
        {
            retry_at       = now;
            b_is_connected = BUDDY4_IMU_INIT(&bus);

            if (b_is_connected)
            {
                tk_dly_tsk(160);
                last_good = now_us();
            }
            else
            {
                sample.timestamp_us = now;
                process_sample(&monitor, &sample, &odo);
            }
        }

        if (b_is_connected)
        {
#if APP_S4_BENCH
            odo.b_is_stationary =
                true; /* Bench: user keeps car level/still for startup. */
#else
            motion_feedback_t feedback = {0};
            /* Waiting for the motion mutex can produce feedback newer than
             * this loop's start. Check its age against a clock read after the
             * copy, so fresh odometry is not rejected as an unsigned wrap. */
            if (((((motion_get_feedback(&feedback)) &&
                   (feedback.b_is_feedback_valid)) &&
                  (motion_is_calibrated())) &&
                 ((unsigned int)MOTION_FAULT_NONE == feedback.fault)) &&
                ((now_us() - feedback.timestamp_us) <= 50000U))
            {
                odo.b_is_valid      = true;
                odo.b_is_stationary = feedback.b_is_braked;
                odo.distance_m      = feedback.centre_m;
                odo.epoch           = feedback.epoch;
            }
#endif
            if (BUDDY4_IMU_READ(&bus, now_us(), &sample))
            {
                last_good = sample.timestamp_us;
                process_sample(&monitor, &sample, &odo);
            }
            else if ((now_us() - last_good) > 100000U)
            {
                sample.b_is_valid   = false;
                sample.timestamp_us = now_us();
                process_sample(&monitor, &sample, &odo);
                b_is_connected = false;
            }
            else
            {
                /* Keep the connection during the allowed read-failure
                 * interval. */
            }
        }
#if APP_S4_BENCH
        if ((now_us() - report_at) >= 1000000U)
        {
            const terr_data_result_t * p_reading = &monitor.result;
            report_at                            = now_us();
            tm_printf(
                (UB *)"[S4] connected=%d cal=%d flags=%u pitch_x10=%d "
                      "roll_x10=%d state=%u peak_mm=%d safety=%u\n",
                b_is_connected, monitor.calibration.b_is_calibrated,
                p_reading->flags, (int)(p_reading->pitch_deg * 10),
                (int)(p_reading->roll_deg * 10), (unsigned)p_reading->state,
                (int)(p_reading->peak_m * 1000), (unsigned)p_reading->safety);
        }
#else
        (void)report_at;
#endif
        /* Skip ticks accumulated while busy, including during sensor startup.
         */
        (void)tk_can_wup(TSK_SELF);
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/** @brief Poll the terrain mutex to queue one run reset. On acceptance,
 * advance the wrapping token, clear pending events and optionally
 * return the token. Defer monitor reset and optional level calibration
 * to the owner.
 */
static bool request_reset (bool b_should_recalibrate, uint16_t * p_token)
{
    if ((gh_terrain_mutex <= 0) ||
        (tk_loc_mtx(gh_terrain_mutex, TMO_POL) != E_OK))
    {
        return false;
    }

    if (g_queue.b_is_reset_pending)
    {
        tk_unl_mtx(gh_terrain_mutex);
        return false;
    }

    ++g_run;

    if (p_token)
    {
        *p_token = g_run;
    }

    g_queue.b_is_reset_pending   = true;
    g_queue.b_should_recalibrate = b_should_recalibrate;
    g_queue.b_has_published      = false;
    g_queue.head                 = 0;
    g_queue.count                = 0;
    g_queue.b_has_lost_events    = false;
    tk_unl_mtx(gh_terrain_mutex);
    return true;
}
#else
/** @brief Return E_OK in the disabled profile so application startup can
 * continue.
 */
ER subsystem4_init (void)
{
    return E_OK;
}

/** @brief Return E_OK without creating the terrain task in this profile.
 */
ER subsystem4_start (void)
{
    return E_OK;
}

/** @brief Return false and leave the caller's terrain result untouched.
 */
bool buddy4_snapshot (terr_data_result_t * p_out)
{
    (void)p_out;
    return false;
}

/** @brief Return false without writing an event or overflow flag.
 */
bool buddy4_event (terr_data_result_t * p_out, bool * pb_has_lost_events)
{
    (void)p_out;
    (void)pb_has_lost_events;
    return false;
}

/** @brief Return false because the disabled owner cannot accept a reset.
 */
bool buddy4_reset_run (bool b_should_recalibrate)
{
    (void)b_should_recalibrate;
    return false;
}

/** @brief Return false without writing a run token.
 */
bool buddy4_begin_run (uint16_t * p_token)
{
    (void)p_token;
    return false;
}

/** @brief Return false in this profile and leave all three caller outputs
 * untouched.
 */
bool buddy4_read (terr_data_result_t * p_out, uint16_t * p_token,
                  uint16_t * p_humps)
{
    (void)p_out;
    (void)p_token;
    (void)p_humps;
    return false;
}

/** @brief Return false without retaining the supplied calibration arrays.
 */
bool buddy4_set_mag_calibration (const float32_t p_offset[3],
                                 const float32_t p_scale[3])
{
    (void)p_offset;
    (void)p_scale;
    return false;
}
#endif

/* End of file. */

