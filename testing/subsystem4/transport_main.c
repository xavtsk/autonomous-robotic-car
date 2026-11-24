/** @file transport_main.c
 * @brief Production S4 task/transport with fake RTOS and sensor boundaries.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include <assert.h>
#include "../../app_program/app_types.h"
#include "transport_main.h"
#include <stdbool.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <tk/device.h>

#if APP_IMU_DEBUG
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"

#define IMU_DEBUG_BATCH_ROWS 5U
#endif

#define QUEUED_HUMP_EVENT_COUNT 10U
#define QUEUED_LIFT_EVENT_COUNT 12U

static terr_data_odometry_t      g_seen_odometry;
static terr_data_hump_geometry_t g_seen_geometry;
static float32_t                 g_seen_track_m;

static uint32_t       g_clock_us = 1000;
static uint32_t       g_read_delay_us;
static uint32_t       g_feedback_age_us;
static uint32_t       g_pose_future_us;
static motion_state_t g_feedback_state = MOTION_IDLE;
static unsigned       g_owner_remaining;
static bool           gb_mutex_locked[3];
static bool           gb_should_fail_lock;
static bool           gb_should_fail_debug_lock;
static ID             gh_next_mutex;
static jmp_buf        g_task_exit;

static void ( * gp_entry)(INT start_code, void * p_extra_info) = NULL;
static void ( * gp_cycle)(void * p_context)                    = NULL;

static unsigned g_cycle_ms;
static bool     gb_is_cycle_running;
static bool     gb_has_pending_wake;
ER              tk_dly_tsk (unsigned delay_ms);

static void owner_cycles (unsigned count);
static void owner_cycle (void);
#if APP_IMU_DEBUG
static void test_imu_debug (void);
static void test_hump_imu_debug (void);
static void test_retained_hump_event (void);
#endif

/** @brief Fail the fixture assertion if the owner unexpectedly exits its
 * task.
 */
void tk_ext_tsk (void)
{
    assert(!"Unexpected task exit");
}

/** @brief Check the requested priority and stack size, retain the task entry
 * and return ID two.
 */
ID tk_cre_tsk (const T_CTSK * p_config)
{
    assert(18U == p_config->itskpri && 4096U == p_config->stksz);
    gp_entry = (void (*)(INT, void *))p_config->task;
    return 2;
}

/** @brief Acknowledge the scheduler request without running a kernel
 * callback.
 */
ER tk_sta_tsk (ID h_task, INT value)
{
    (void)h_task;
    (void)value;
    return E_OK;
}

/** @brief Acknowledge teardown without removing a real kernel object.
 */
ER tk_del_tsk (ID h_task)
{
    (void)h_task;
    return E_OK;
}

/** @brief Require a 10 ms cycle and phase, then retain the supplied handler
 * and period.
 */
ID tk_cre_cyc (const T_CCYC * p_cycle)
{
    assert(TA_HLNG == p_cycle->cycatr && 10U == p_cycle->cyctim &&
           10U == p_cycle->cycphs);
    gp_cycle   = (void (*)(void *))p_cycle->cychdr;
    g_cycle_ms = p_cycle->cyctim;
    return 3;
}

/** @brief Require cyclic ID three and update the simulated timerâ€™s running
 * state.
 */
ER tk_sta_cyc (ID h_cycle)
{
    assert(3 == h_cycle);
    gb_is_cycle_running = true;
    return E_OK;
}

/** @brief Require cyclic ID three and update the simulated timerâ€™s running
 * state.
 */
ER tk_stp_cyc (ID h_cycle)
{
    assert(3 == h_cycle);
    gb_is_cycle_running = false;
    return E_OK;
}

/** @brief Require the stopped cyclic handler with ID three before
 * acknowledging deletion.
 */
ER tk_del_cyc (ID h_cycle)
{
    assert(3 == h_cycle && !gb_is_cycle_running);
    return E_OK;
}

/** @brief Mark a wake pending for the fixture task with ID two.
 */
ER tk_wup_tsk (ID h_task)
{
    assert(2 == h_task);
    gb_has_pending_wake = true;
    return E_OK;
}

/** @brief Require the current task, clear its pending wake and return zero.
 */
INT tk_can_wup (ID h_task)
{
    assert(TSK_SELF == h_task);
    gb_has_pending_wake = false;
    return 0;
}

/** @brief Verify priority inheritance and allocate the next of two simulated
 * mutex IDs.
 */
ID tk_cre_mtx (const T_CMTX * p_mutex)
{
    assert(TA_INHERIT == p_mutex->mtxatr && gh_next_mutex < 2);
    return ++gh_next_mutex;
}

/** @brief Require a valid unlocked mutex and the expected wait mode. Inject
 * a timeout for the selected lock, otherwise acquire it.
 */
ER tk_loc_mtx (ID h_mutex, TMO timeout)
{
    assert(h_mutex > 0 && h_mutex <= 2 && !gb_mutex_locked[h_mutex]);

    if (2 == h_mutex)
    {
        assert(TMO_POL == timeout);
    }

    if (((1 == h_mutex) && gb_should_fail_lock) ||
        ((2 == h_mutex) && gb_should_fail_debug_lock))
    {
        return E_TMOUT;
    }

    gb_mutex_locked[h_mutex] = true;
    return E_OK;
}

/** @brief Check the simulated mutex ID and ownership before clearing its lock
 * flag.
 */
ER tk_unl_mtx (ID h_mutex)
{
    assert(h_mutex > 0 && h_mutex <= 2 && gb_mutex_locked[h_mutex]);
    gb_mutex_locked[h_mutex] = false;
    return E_OK;
}

/** @brief Discard the formatted console output and return zero.
 */
int tm_printf (const UB * p_format, ...)
{
    (void)p_format;
    return 0;
}

/** @brief Return the current simulated microsecond count for the timer read.
 */
uint32_t runtime_read_clock (uint32_t address)
{
    (void)address;
    return g_clock_us;
}

/** @brief Advance simulated time by the configured read delay and return
 * braked feedback with the requested timestamp skew.
 */
bool motion_get_feedback (motion_feedback_t * p_out)
{
    /* Model a motion update while S4 waits for the feedback mutex. */
    g_clock_us += g_read_delay_us;
    *p_out      = (motion_feedback_t){
             .b_is_feedback_valid = true,
             .b_is_braked         = true,
             .state               = g_feedback_state,
             .timestamp_us = g_clock_us - g_feedback_age_us + g_pose_future_us};
    return true;
}

/** @brief Report that the simulated motion controller already has
 * calibration.
 */
bool motion_is_calibrated (void)
{
    return true;
}

/** @brief Report the full requested transfer length and return success
 * without accessing a device.
 */
ER tk_swri_dev (ID h_device, int start, const void * p_buffer, SZ length,
                SZ * p_actual)
{
    (void)h_device;
    (void)start;
    (void)p_buffer;
    *p_actual = length;
    return E_OK;
}

/** @brief Check the IMU bus device name and return the fixture device ID.
 */
ID tk_opn_dev (UB * p_name, unsigned mode)
{
    assert(!strcmp((char *)p_name, "iicb"));
    (void)mode;
    return 3;
}

/** @brief Require both mutexes to be free, advance the clock by 160 ms and
 * invoke the scheduled cycle.
 */
ER tk_dly_tsk (unsigned milliseconds)
{
    assert(!gb_mutex_locked[1] && !gb_mutex_locked[2] && 160U == milliseconds);
    g_clock_us += milliseconds * 1000U;
    gp_cycle(NULL); /* A wake queued during sensor startup. */
    return E_OK;
}

/** @brief Drive one simulated scheduler wake while both locks are free.
 * Return to the fixture when the requested owner cycles finish.
 */
ER tk_slp_tsk (TMO timeout)
{
    assert(TMO_FEVR == timeout && !gb_mutex_locked[1] && !gb_mutex_locked[2]);
    assert(gb_is_cycle_running && !gb_has_pending_wake);
    g_clock_us += g_cycle_ms * 1000U;
    gp_cycle(NULL);
    assert(gb_has_pending_wake);
    gb_has_pending_wake = false;

    if (--g_owner_remaining == 0U)
    {
        longjmp(g_task_exit, 1);
    }

    return E_OK;
}

/** @brief Inject terrain measurements and queue conditions. Check run
 * tokens, freshness, hump retention and transport failures.
 */
int main (void)
{
    terr_data_result_t result = {.timestamp_us = g_clock_us,
                                 .flags =
                                     TERRAIN_CALIBRATED | TERRAIN_PEAK_VALID,
                                 .peak_m = 0.05f,
                                 .state  = TERRAIN_MOVING};
    terr_data_result_t out;
    uint16_t           token;
    uint16_t           humps;
    uint16_t           requested;
    bool               b_has_lost_sample;
    assert(!buddy4_read(&out, &token, &humps));
    assert(subsystem4_init() == E_OK && subsystem4_start() == E_OK);
    assert(!buddy4_read(&out, &token, &humps));
    buddy4_transport_publish(&result);
    assert(buddy4_read(&out, &token, &humps) && 0 == (int)token &&
           0 == (int)humps);
    result.b_is_hump_complete = true;

    for (unsigned case_index = 0; case_index < QUEUED_HUMP_EVENT_COUNT;
         case_index++)
    {
        buddy4_transport_publish(&result);
    }

    assert(buddy4_read(&out, &token, &humps) && 10 == (int)humps);
    assert(buddy4_event(&out, &b_has_lost_sample) && b_has_lost_sample);
    assert(buddy4_begin_run(&requested) && 1 == (int)requested);
    assert(!buddy4_begin_run(&token));
    assert(!buddy4_read(&out, &token, &humps) &&
           !buddy4_event(&out, &b_has_lost_sample));
    buddy4_transport_publish(&result);
    assert(!buddy4_read(&out, &token,
                        &humps)); /* Suppress the previous run's sample. */
    owner_cycle();
    assert(buddy4_transport_geometry() == g_seen_geometry &&
           buddy4_transport_track_m() == g_seen_track_m);
    assert(buddy4_read(&out, &token, &humps) && token == requested &&
           0 == (int)humps);
    assert(!(out.flags & TERRAIN_PEAK_VALID));

    while (buddy4_event(&out, &b_has_lost_sample))
    {
        /* Drain earlier events before the next publication. */
    }

    result.timestamp_us       = g_clock_us;
    result.b_is_hump_complete = false;
    result.b_has_impact       = false;
    buddy4_transport_publish(&result);

    while (buddy4_event(&out, &b_has_lost_sample))
    {
        /* Drain the previous events before testing the lifted state. */
    }

    result.safety = TERRAIN_SAFETY_LIFTED;

    for (unsigned case_index = 0; case_index < QUEUED_LIFT_EVENT_COUNT;
         case_index++)
    {
        buddy4_transport_publish(&result);
    }

    assert(buddy4_read(&out, &token, &humps) &&
           (unsigned)TERRAIN_SAFETY_LIFTED == out.safety);
    assert(buddy4_event(&out, &b_has_lost_sample) && !b_has_lost_sample &&
           (unsigned)TERRAIN_SAFETY_LIFTED == out.safety);
    assert(!buddy4_event(&out, &b_has_lost_sample));
    result.safety = TERRAIN_SAFETY_OVERTURNED;
    buddy4_transport_publish(&result);
    result.safety = TERRAIN_SAFETY_NONE;
    buddy4_transport_publish(&result);
    assert(buddy4_read(&out, &token, &humps) &&
           (unsigned)TERRAIN_SAFETY_NONE == out.safety);
    assert(buddy4_event(&out, &b_has_lost_sample) && !b_has_lost_sample &&
           (unsigned)TERRAIN_SAFETY_OVERTURNED == out.safety);

    while (buddy4_event(&out, &b_has_lost_sample))
    {
        /* Drain pending events before the next scenario. */
    }

    result.b_has_impact = true;
    buddy4_transport_publish(&result);
    assert(buddy4_event(&out, &b_has_lost_sample) && out.b_has_impact);
    buddy4_transport_set_hump_count(UINT16_MAX);
    result.b_is_hump_complete = true;
    buddy4_transport_publish(&result);
    assert(buddy4_read(&out, &token, &humps) && UINT16_MAX == humps);
    g_clock_us += 160000U;
    assert(buddy4_read(&out, &token, &humps) && (out.flags & TERRAIN_FAULT));
    assert((out.flags & TERRAIN_PEAK_VALID) && !out.b_has_impact &&
           !out.b_is_hump_complete);
    gb_should_fail_lock = true;
    assert(!buddy4_read(&out, &token, &humps));
    assert(!buddy4_begin_run(&token));
    gb_should_fail_lock = false;
    buddy4_transport_set_run(UINT16_MAX);
    assert(buddy4_begin_run(&requested) && 0 == (int)requested);
    owner_cycle();
    assert(buddy4_read(&out, &token, &humps) && 0 == (int)token &&
           0 == (int)humps);
    /* Observe the second loop so sensor startup delay is not the source of
     * the race. Fresh feedback may be published after S4's loop-start clock. */
    const uint32_t owner_start_us = g_clock_us;
    owner_cycles(2);
    assert(g_seen_odometry.b_is_valid && g_seen_odometry.b_is_stationary);
    assert(g_clock_us - owner_start_us ==
           180000U); /* 160 ms startup and two 10 ms polls. */
    g_read_delay_us  = 1000;
    g_feedback_state = MOTION_TURN;
    owner_cycles(2);
    assert(g_seen_odometry.b_is_valid &&
           "Fresh feedback after a mutex wait must remain valid");
    assert(g_seen_odometry.b_is_pivoting);
    g_feedback_state  = MOTION_DRIVE;
    g_feedback_age_us = 50000;
    owner_cycles(2);
    assert(g_seen_odometry.b_is_valid && (!g_seen_odometry.b_is_pivoting));
    g_feedback_age_us = 50001;
    owner_cycles(2);
    assert(!g_seen_odometry.b_is_valid && !g_seen_odometry.b_is_stationary);
    g_feedback_age_us = 0;
    g_pose_future_us  = 1;
    owner_cycles(2);
    assert(!g_seen_odometry.b_is_valid);
    g_pose_future_us = 0;
    /* Leave 500 us after the first loop so the second feedback read crosses
     * the uint32_t hardware-clock wrap during its 1 ms mutex wait. */
    g_clock_us = UINT32_MAX - (161500U + (g_cycle_ms * 1000U));
    owner_cycles(2);
    assert(g_seen_odometry.b_is_valid);
#if APP_IMU_DEBUG
    test_imu_debug();
    test_hump_imu_debug();
    test_retained_hump_event();
#endif
    puts("PASS: real S4 transport/task reset acknowledgement, "
         "count/saturation, event overflow, safety/impact delivery, stale "
         "validity, token wrap and feedback freshness");
    return 0;
}

/** @brief Capture odometry, geometry and wheel spacing at entry, then
 * execute the real terrain monitor update.
 */
void buddy4_transport_observe (terr_data_monitor_t *        p_monitor,
                               const imu_sensor_sample_t *  p_sample,
                               const terr_data_odometry_t * p_odometry)
{
    g_seen_odometry = *p_odometry;
    g_seen_geometry = p_monitor->hump_geometry;
    g_seen_track_m  = p_monitor->wheel_track_m;
    terr_data_process(p_monitor, p_sample, p_odometry);
}

/** @brief Report a successful IMU initialization without a bus transaction.
 */
bool buddy4_transport_imu_init (const imu_sensor_bus_t * p_bus)
{
    (void)p_bus;
    return true;
}

/** @brief Supply a valid, level one-g acceleration sample at the requested
 * timestamp.
 */
bool buddy4_transport_imu_read (const imu_sensor_bus_t * p_bus, uint32_t now,
                                imu_sensor_sample_t * p_sample)
{
    (void)p_bus;
    *p_sample = (imu_sensor_sample_t){
        .timestamp_us = now, .b_is_valid = true, .accel_g = {0, 0, 1}};
    return true;
}

/** @brief Run the retained terrain task entry for the requested positive
 * number of scheduler wakeups.
 */
static void owner_cycles (unsigned count)
{
    assert(count > 0U);
    g_owner_remaining = count;

    if (!setjmp(g_task_exit))
    {
        gp_entry(0, NULL);
    }
}

/** @brief Run the terrain owner through one scheduler wakeup.
 */
static void owner_cycle (void)
{
    owner_cycles(1);
}
#if APP_IMU_DEBUG
/** @brief Read two little-endian bytes from the supplied packet as an
 * unsigned 16-bit word.
 */
static uint16_t wire16 (const uint8_t * p_bytes)
{
    return (uint16_t)((unsigned)p_bytes[0] | ((unsigned)p_bytes[1] << 8U));
}

/** @brief Read four little-endian bytes from the supplied packet as an
 * unsigned 32-bit word.
 */
static uint32_t wire32 (const uint8_t * p_bytes)
{
    return (uint32_t)wire16(p_bytes) | ((uint32_t)wire16(p_bytes + 2) << 16);
}

/** @brief Check raw IMU debug packet size, field encoding and progress after
 * a simulated queue overflow.
 */
static void test_imu_debug (void)
{
    terr_data_monitor_t monitor;
    terr_data_init(&monitor);
    monitor.calibration.b_is_calibrated = true;
    monitor.calibration.bias_g[0]       = 0.01f;
    monitor.calibration.bias_g[2]       = 0.02f;

    imu_sensor_sample_t        sample   = {.b_is_valid    = true,
                                           .b_has_overrun = true,
                                           .accel_g = {-0.02f, 0.03f, 1.19f}};
    const terr_data_odometry_t odometry = {0};
    uint8_t                    packet[MQTT_UTK_IMU_DEBUG_MAX];
    uint16_t                   next_run;
    buddy4_transport_clear_debug();
    const uint32_t first_sequence  = buddy4_transport_debug_sequence();
    const uint16_t first_run       = buddy4_transport_debug_run();
    const uint32_t first_timestamp = g_clock_us = 2000000U;

    for (unsigned index = 0; index < IMU_DEBUG_BATCH_ROWS; ++index)
    {
        sample.timestamp_us = g_clock_us;
        buddy4_transport_process(&monitor, &sample, &odometry);
        g_clock_us += 20000U;

        if (index < 4U)
        {
            assert(mqtt_utk_imu_debug_take(packet, sizeof packet) == 0U);
        }
    }

    assert(mqtt_utk_imu_debug_take(packet, 151) == 0U);
    assert(mqtt_utk_imu_debug_take(packet, sizeof packet) == 152U);
    assert(2 == (int)packet[0] && 5 == (int)packet[1]);
    assert(wire16(packet + 2) == first_run &&
           wire32(packet + 4) == first_sequence);
    const uint32_t boot_tag = wire32(packet + 8);
    assert(boot_tag != 0U);
    const uint8_t * p_row = packet + 12;
    assert(wire32(p_row) == first_timestamp);
    assert(wire16(p_row + 4) == (uint16_t)-20 && wire16(p_row + 6) == 30U &&
           wire16(p_row + 8) == 1190U);
    assert(wire16(p_row + 10) == (uint16_t)-30 && wire16(p_row + 12) == 30U &&
           wire16(p_row + 14) == 1170U);
    assert(p_row[16] == (1U | 2U | 4U | 16U | 32U) &&
           TERRAIN_SAFETY_NONE == (int)p_row[17]);
    assert(wire32(p_row + 18) == (uint32_t)INT32_MIN &&
           wire32(p_row + 22) == (uint32_t)INT32_MIN);
    assert(0 == (int)p_row[26] && 0 == (int)p_row[27]);
    p_row += 2 * 28;
    assert(p_row[16] == (1U | 2U | 4U | 32U) &&
           TERRAIN_SAFETY_LIFTED == (int)p_row[17]);

    /* A fault row followed by a new run flushes as its own packet. */
    sample = (imu_sensor_sample_t){.timestamp_us = g_clock_us};
    buddy4_transport_process(&monitor, &sample, &odometry);
    assert(buddy4_begin_run(&next_run));
    buddy4_transport_process(
        &monitor, &sample,
        &odometry); /* The pending reset blocks this sample. */
    owner_cycle();
    assert(mqtt_utk_imu_debug_take(packet, sizeof packet) == 40U);
    assert(1 == (int)packet[1] && wire16(packet + 2) == first_run &&
           wire32(packet + 4) == first_sequence + 5U);
    assert(0 == (int)packet[28] && TERRAIN_SAFETY_LIFTED == (int)packet[29]);
    g_clock_us += 250000U;
    assert(mqtt_utk_imu_debug_take(packet, sizeof packet) == 40U);
    assert(wire16(packet + 2) == next_run &&
           wire32(packet + 4) == first_sequence + 6U);
    assert(wire32(packet + 8) == boot_tag && 1U == packet[28]);

    /* Lock contention is visible as a gap, even within the same run. */
    owner_cycle();
    const uint32_t before_gap = buddy4_transport_debug_sequence() - 1U;
    gb_should_fail_debug_lock = true;
    owner_cycle();
    terr_data_result_t latest;
    uint16_t           token;
    uint16_t           humps;
    assert(buddy4_read(&latest, &token, &humps) && token == next_run);
    gb_should_fail_debug_lock = false;
    owner_cycle();
    assert(mqtt_utk_imu_debug_take(packet, sizeof packet) == 40U);
    assert(wire32(packet + 4) == before_gap);
    g_clock_us += 250000U;
    assert(mqtt_utk_imu_debug_take(packet, sizeof packet) == 40U);
    assert(wire32(packet + 4) == before_gap + 2U);

    const uint32_t before_overflow = buddy4_transport_debug_sequence();
    owner_cycles(buddy4_transport_debug_capacity() + 2U);
    assert(mqtt_utk_imu_debug_take(packet, sizeof packet) == 152U);
    assert(wire32(packet + 4) == before_overflow + 2U);
}

/** @brief Check right-wheel hump diagnostic fields and unavailable-value
 * encodings in captured IMU rows.
 */
static void test_hump_imu_debug (void)
{
    terr_data_monitor_t monitor;
    terr_data_init(&monitor);
    monitor.hump_geometry = TERRAIN_HUMP_RIGHT_WHEEL;

    monitor.hump.debug =
        (hump_detection_debug_t){.wheel_delta_m          = -0.000275f,
                                 .travel_m               = 0.011325f,
                                 .b_is_wheel_delta_valid = true,
                                 .flags                  = HUMP_DEBUG_DISCARDED,
                                 .reason = HUMP_REASON_REVERSE_TRAVEL};

    imu_sensor_sample_t sample = {
        .timestamp_us = g_clock_us, .b_is_valid = true, .accel_g = {0, 0, 1}};
    uint8_t packet[MQTT_UTK_IMU_DEBUG_MAX];
    buddy4_transport_clear_debug();
    buddy4_transport_record_debug(&sample, &monitor);

    /* clang-format off */
    monitor.hump.debug  =
        (hump_detection_debug_t){.b_is_wheel_delta_valid = false,
                                 .reason = HUMP_REASON_ODOMETRY_UNAVAILABLE};
    sample.timestamp_us = g_clock_us += 40000U;
    /* clang-format on */
    buddy4_transport_record_debug(&sample, &monitor);
    g_clock_us += 250000U;
    assert(mqtt_utk_imu_debug_take(packet, sizeof packet) == 68U);
    assert(2 == (int)packet[0] && 2 == (int)packet[1]);
    const uint8_t * p_row = packet + 12;
    assert(wire32(p_row + 18) == (uint32_t)-275 &&
           wire32(p_row + 22) == 11325U);
    assert(4 == (int)p_row[26] && 10 == (int)p_row[27]);
    p_row += 28;
    assert(wire32(p_row + 18) == (uint32_t)INT32_MIN &&
           wire32(p_row + 22) == 0U);
    assert(0 == (int)p_row[26] && 4 == (int)p_row[27]);
}

/** @brief A discarded candidate survives raw logging contention, a new run,
 * repeated reads, and a missing or delayed acknowledgement.
 */
static void test_retained_hump_event (void)
{
    const unsigned      queue_capacity = 8U;
    const uint16_t      first_run      = buddy4_transport_debug_run();
    terr_data_monitor_t monitor;
    terr_data_init(&monitor);
    monitor.hump_geometry               = TERRAIN_HUMP_RIGHT_WHEEL;
    imu_sensor_sample_t        sample   = {.timestamp_us = g_clock_us};
    const terr_data_odometry_t odometry = {0};
    uint8_t                    packet[24];
    gb_should_fail_debug_lock = true;

    for (unsigned index = 0; index <= queue_capacity; ++index)
    {
        monitor.hump.b_is_hump_tracking = true;
        monitor.hump.peak_m             = 0.01f;
        monitor.hump.travel_m           = 0.03f;
        buddy4_transport_process(&monitor, &sample, &odometry);
        sample.timestamp_us += 20000U;
    }

    gb_should_fail_debug_lock = false;
    uint16_t next_run;
    assert(buddy4_begin_run(&next_run));
    assert(next_run != first_run);
    assert(0U == mqtt_utk_hump_debug_peek(packet, sizeof packet - 1U));
    assert(sizeof packet == mqtt_utk_hump_debug_peek(packet, sizeof packet));
    assert(1 == packet[0] && HUMP_REASON_INVALID_IMU == packet[1]);
    assert(first_run == wire16(packet + 2) && 0U != wire32(packet + 4));
    assert(g_clock_us == wire32(packet + 12));
    assert(30000U == wire32(packet + 16) && 10000U == wire16(packet + 20));
    assert(1U == wire16(packet + 22));
    const uint32_t first = wire32(packet + 8);
    mqtt_utk_hump_debug_ack(first + 1U);
    gb_should_fail_lock = true;
    mqtt_utk_hump_debug_ack(first);
    gb_should_fail_lock = false;
    for (unsigned index = 0; index < queue_capacity; ++index)
    {
        assert(sizeof packet ==
               mqtt_utk_hump_debug_peek(packet, sizeof packet));
        assert((first + index) == wire32(packet + 8));
        mqtt_utk_hump_debug_ack(first + index);
    }
    assert(0U == mqtt_utk_hump_debug_peek(packet, sizeof packet));
}
#endif

/* End of file. */

