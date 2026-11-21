/** @file runtime_main.c
 * @brief Execute the production runtime with fake task/ADC boundaries.
 *
 * This proves
 * queue/snapshot ownership, not scheduling deadlines or physical calibration.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "runtime_main.h"
#include "../../app_program/controller/s3_integration.h"
#include "../../app_program/controller/run_timing.h"
#include "../../app_program/controller/bc_capture.h"
#include "../../app_program/controller/ir_debug.h"
#if TM_MQTT
#include "mqtt_utk.h"
#endif
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static jmp_buf  g_task_exit;
static unsigned g_sleeps;
static unsigned g_locks;
static unsigned g_observed;
static unsigned g_freeze_calls;
static unsigned g_stops;
static uint32_t g_clock_us;
static int32_t  g_adc_error;
static uint16_t g_raw                 = 150;
static bool     gb_is_feedback_braked = true;
static bool     gb_has_feedback_fault;
static bool     gb_should_clear_fault = true;
static uint8_t  g_feedback_command_id;
static unsigned g_reset_calls;
static unsigned g_cancel_calls;
static unsigned g_velocity_requests;

static void ( * gp_during_start)(void) = NULL;

static bool     gb_is_test_mutex_busy;
static uint32_t g_debug_motion_advance;
static uint32_t g_debug_motion_age;
static bool     gb_is_motion_out_of_range;

static void cycle (void);

#if !defined(TEST_SLOPE_LINK)
static void remote_send (uint8_t action, uint32_t request_id, uint32_t epoch,
                         uint32_t timestamp);

static void remote_expect (uint32_t request_id, uint8_t result, uint8_t reason);

static void stop_during_start (void);

static void reconnect_during_start (void);
#endif

/** @brief Verify the priority-inheritance attribute and return the configured
 * fake mutex ID.
 */
ID tk_cre_mtx (const T_CMTX * p_mutex_config)
{
    assert(TA_INHERIT == p_mutex_config->mtxatr);
    return 1;
}

/** @brief Return a polling timeout when busy. Otherwise require the sampling
 * mutex to be unlocked and claim it for the fixture.
 */
ER tk_loc_mtx (ID h_mutex, TMO timeout)
{
    if (gb_is_test_mutex_busy)
    {
        assert(TMO_POL == timeout);
        return E_TMOUT;
    }

    assert(1 == h_mutex && !g_locks);
    g_locks = 1;
    return E_OK;
}

/** @brief Require the acquired fake mutex, then release it and report
 * success.
 */
ER tk_unl_mtx (ID h_mutex)
{
    assert(1 == h_mutex && g_locks);
    g_locks = 0;
    return E_OK;
}

/** @brief Check the sample period, initial phase and handler before returning
 * cyclic ID two.
 */
ID tk_cre_cyc (const T_CCYC * p_cyclic_config)
{
    assert(S3_SAMPLE_PERIOD_MS == p_cyclic_config->cyctim &&
           S3_SAMPLE_PERIOD_MS == p_cyclic_config->cycphs &&
           p_cyclic_config->cychdr);
    return 2;
}

/** @brief Acknowledge the scheduler request without running a kernel
 * callback.
 */
ER tk_stp_cyc (ID h_cycle)
{
    (void)h_cycle;
    return E_OK;
}

/** @brief Acknowledge teardown without removing a real kernel object.
 */
ER tk_del_cyc (ID h_cycle)
{
    (void)h_cycle;
    return E_OK;
}

/** @brief Acknowledge teardown without removing a real kernel object.
 */
ER tk_del_mtx (ID h_mutex)
{
    (void)h_mutex;
    return E_OK;
}

/** @brief Acknowledge teardown without removing a real kernel object.
 */
ER tk_del_tsk (ID h_task)
{
    (void)h_task;
    return E_OK;
}

/** @brief Acknowledge teardown without removing a real kernel object.
 */
ER tk_ter_tsk (ID h_task)
{
    (void)h_task;
    return E_OK;
}

/** @brief Acknowledge the scheduler request without running a kernel
 * callback.
 */
ER tk_sta_cyc (ID h_cycle)
{
    (void)h_cycle;
    return E_OK;
}

/** @brief Acknowledge a wake request without scheduling a task.
 */
ER tk_wup_tsk (ID h_task)
{
    (void)h_task;
    return E_OK;
}

/** @brief Report an empty wake queue without blocking the calling task.
 */
ER tk_can_wup (ID h_task)
{
    (void)h_task;
    return E_OK;
}

/** @brief Require an unlocked, indefinite sleep. Transfer control back to
 * the fixture on its second sleep boundary.
 */
ER tk_slp_tsk (TMO timeout)
{
    assert(TMO_FEVR == timeout && !g_locks);

    if (++g_sleeps == 2U)
    {
        longjmp(g_task_exit, 1);
    }

    return E_OK;
}

/** @brief Complete the requested delay immediately without advancing the fake
 * clock.
 */
ER tk_dly_tsk (unsigned milliseconds)
{
    (void)milliseconds;
    return E_OK;
}

/** @brief Return the fixture task ID without scheduling an entry point.
 */
ID app_start_task (FP p_handler, PRI priority, UW core, const char * p_name)
{
    (void)p_handler;
    (void)priority;
    (void)core;
    (void)p_name;
    return 3;
}

/** @brief Require that printing occurs outside the lock and discard the text.
 */
int tm_printf (const UB * p_format, ...)
{
    (void)p_format;
    assert(!g_locks);
    return 0;
}

/** @brief Return the current simulated microsecond count for the timer read.
 */
uint32_t runtime_read_clock (uint32_t address)
{
    (void)address;
    return g_clock_us;
}

/** @brief Check that initialization occurs outside the lock and all three
 * simulated IR channels are enabled.
 */
int32_t ir_sensor_init (const ir_acquisition_config_t * p_config)
{
    assert(!g_locks && 7 == (int)p_config->enabled_mask);
    return E_OK;
}

/** @brief Fill all three raw IR channels with the fixture value and
 * timestamp, then return the injected ADC result.
 */
int32_t ir_sensor_read_frame (ir_acquisition_frame_t * p_frame)
{
    assert(!g_locks);
    *p_frame = (ir_acquisition_frame_t){.timestamp_us = g_clock_us};

    for (unsigned sensor_index = 0; sensor_index < (unsigned)S3_SENSOR_COUNT;
         sensor_index++)
    {
        p_frame->sensor[sensor_index] =
            (ir_acquisition_sample_t){g_clock_us, g_raw, 0, true, true};
    }

    return g_adc_error;
}

/** @brief Copy the simulated motion timestamp, epoch, command, brake and
 * fault state while outside the sampling lock.
 */
bool motion_get_feedback (motion_feedback_t * p_feedback)
{
    assert(!g_locks);
    *p_feedback = (motion_feedback_t){
        .timestamp_us        = g_clock_us,
        .epoch               = 1,
        .b_is_feedback_valid = true,
        .b_is_braked         = gb_is_feedback_braked,
        .command_id          = g_feedback_command_id,
        .state = gb_has_feedback_fault ? MOTION_FAULT : MOTION_IDLE,
        .fault =
            gb_has_feedback_fault ? MOTION_FAULT_STALL : MOTION_FAULT_NONE};
    return true;
}

/** @brief Advance the simulated clock by the injected read delay and supply
 * a diagnostic snapshot with the chosen age and speed bounds.
 */
bool motion_try_debug_snapshot (motion_debug_snapshot_t * p_snapshot)
{
    assert(!g_locks);
    g_clock_us += g_debug_motion_advance;
    *p_snapshot = (motion_debug_snapshot_t){
        .timestamp_us     = g_clock_us - g_debug_motion_age,
        .b_is_debug_valid = true,
        .b_is_braked      = gb_is_feedback_braked,
        .setpoint_mps     = {0.04f, -0.05f},
        .measured_mps     = {0.035f, -0.042f},
        .encoder          = {-123, 456},
        .duty             = {0.2f, -0.3f}};

    if (gb_is_motion_out_of_range)
    {
        p_snapshot->measured_mps[0] = 10.0f;
    }

    return true;
}

/** @brief Count an unlocked velocity request and set the simulated active
 * command ID to one.
 */
motion_result_t motion_request_velocity (float32_t left_mps,
                                         float32_t right_mps)
{
    (void)left_mps;
    (void)right_mps;
    assert(!g_locks);
    g_velocity_requests++;
    g_feedback_command_id = 1;
    return MOTION_OK;
}

/** @brief Accept the simulated motion command immediately without starting a
 * motor.
 */
motion_result_t motion_request_drive (float32_t distance_m, float32_t speed_mps)
{
    (void)distance_m;
    (void)speed_mps;
    return MOTION_OK;
}

/** @brief Acknowledge a manoeuvre requested outside the sampling lock.
 */
motion_result_t motion_request_manoeuvre (motion_manoeuvre_t manoeuvre_kind)
{
    (void)manoeuvre_kind;
    assert(!g_locks);
    return MOTION_OK;
}

/** @brief Count a stop outside the lock and clear the simulated active
 * command ID.
 */
void stop (void)
{
    assert(!g_locks);
    g_stops++;
    g_feedback_command_id = 0;
}

/** @brief Return false for an empty motion report queue while outside the
 * lock.
 */
bool motion_get_report (motion_report_t * p_report)
{
    (void)p_report;
    assert(!g_locks);
    return false;
}

/** @brief Return the fixed command ID used in simulated motion feedback.
 */
uint8_t motion_command_id (void)
{
    return 1;
}

/** @brief Count a reset outside the sampling lock and clear the simulated
 * fault when the fixture enables that response.
 */
bool motion_reset (void)
{
    assert(!g_locks);
    g_reset_calls++;

    if (gb_should_clear_fault)
    {
        gb_has_feedback_fault = false;
    }

    return true;
}

/** @brief Verify that the navigation lease callback runs outside the
 * sampling lock.
 */
void motion_navigation_heartbeat (bool b_is_enabled)
{
    (void)b_is_enabled;
    assert(!g_locks);
}

/** @brief Count a configuration freeze and invoke the optional interleaving
 * hook before acknowledging start preparation.
 */
bool vehicle_controller_prep_start (line_navigation_t * p_navigation)
{
    (void)p_navigation;
    assert(!g_locks);
    g_freeze_calls++;

    if (gp_during_start)
    {
        gp_during_start();
    }

    return true;
}

/** @brief Acknowledge the simulated map update after checking that the
 * sampling lock is free.
 */
bool vehicle_controller_set_mapping (const line_navigation_action_t actions[26])
{
    (void)actions;
    assert(!g_locks);
    return true;
}

/** @brief Count cancellation of a prepared start and require the sampling
 * lock to be free.
 */
void vehicle_controller_cancel_start (void)
{
    assert(!g_locks);
    g_cancel_calls++;
}

/** @brief Accept completion without changing the fixture's configuration.
 */
void vehicle_controller_commit_start (void)
{
}

/** @brief Count each non-null observation delivered outside the sampling
 * lock.
 */
void vehicle_controller_observe (const s3_integration_status_t * p_status)
{
    assert(!g_locks && p_status);
    g_observed++;
}

/** @brief Reject report-task labels in the sampling-only fixture. */
const char * motion_state_name (motion_state_t state)
{
    (void)state;
    assert(false);
    return "";
}

/** @brief Reject report-task labels in the sampling-only fixture. */
const char * motion_fault_name (motion_fault_t fault)
{
    (void)fault;
    assert(false);
    return "";
}

#define CAL_SURFACE_CASES (2)
#define CAL_OBSERVE_WINDOW_US (1100000U)
#define RESET_OBSERVE_WINDOW_US (200000U)
#define CANCEL_OBSERVE_WINDOW_US (600000U)
#define RESET_TIMEOUT_LIMIT_US (510000U)
#define DEBUG_OBSERVE_WINDOW_US (50000U)
#define REPLY_LAST_ISSUED_ID (10U)
#define REPLY_FIRST_RETAINED_ID (5U)

#if defined(TEST_SLOPE_LINK)
/** @brief Copy the single-threaded fixture counters and borrow the drive state
 * until the next simulated cycle. The observer never advances the clock.
 */
runtime_main_view_t runtime_main_view (void)
{
    return (runtime_main_view_t){.p_drive = s3_integration_runtime_test_drive(),
                                 .clock_us          = g_clock_us,
                                 .lock_depth        = g_locks,
                                 .command_id        = g_feedback_command_id,
                                 .stops             = g_stops,
                                 .velocity_requests = g_velocity_requests,
                                 .freeze_calls      = g_freeze_calls,
                                 .cancel_calls      = g_cancel_calls};
}

/** @brief Run exactly one sampling period on the simulated microsecond clock.
 */
void runtime_main_cycle (void)
{
    cycle();
}

/** @brief Set the raw count returned by the ADC fake on subsequent reads. */
void runtime_main_set_adc (uint16_t counts)
{
    g_raw = counts;
}

/** @brief Apply the supplied collector to one sensor of the simulated drive.
 * Return the navigation owner's validation result.
 */
bool runtime_main_apply_calibration (
    ir_acquisition_sensor_id_t         sensor,
    const ir_calibration_collector_t * p_values)
{
    return s3_integration_runtime_test_calibration(sensor, p_values);
}

/** @brief Execute a START immediately at the current simulated time and
 * return the same acceptance result as the controller command handler.
 */
bool runtime_main_start_now (void)
{
    return s3_integration_runtime_test_start(g_clock_us);
}
#else
/** @brief Exercise queued requests, packed status, calibration and reset
 * through the sampling owner with a simulated clock.
 * @details The fake RTOS yields after a sample period. The test first
 * calibrates every sensor via the real request path. It then checks run
 * timing, braking and RESET acknowledgement, including cancellations by
 * STOP. Optional profiles exercise capture upload and debug telemetry.
 * Remote cases cover stale epochs, STOP during START, reconnect during
 * RESET and the reply ring's fixed capacity.
 */
int main (void)
{
    const s3_drive_t *       p_drive = s3_integration_runtime_test_drive();
    s3_integration_status_t  status;
    s3_integration_request_t cmd = {.kind   = S3_CONTROL_WHITE,
                                    .sensor = S3_SENSOR_BARCODE};
    assert(!s3_integration_submit(&cmd));
    assert(s3_integration_init() == E_OK);
    assert(!s3_integration_status(&status));
    assert(s3_integration_submit(&cmd));
    cmd.kind = S3_CONTROL_BLACK;
    assert(!s3_integration_submit(&cmd));
    cmd.kind = S3_CONTROL_STOP;
    assert(s3_integration_submit(&cmd));
    cmd.kind = S3_CONTROL_START;
    assert(!s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_status(&status));
    assert(1U == status.command_sequence && status.b_did_command_succeed &&
           !status.b_is_collecting && g_stops > 0U);
    assert(s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_status(&status) && !status.b_did_command_succeed &&
           0U == g_freeze_calls);
    cmd.kind = S3_CONTROL_WHITE;
    assert(s3_integration_submit(&cmd));
    cycle();
    cycle();
    assert(s3_integration_status(&status) && status.b_is_collecting &&
           1U == status.white.count);
    assert(status.navigation.sensor[0].b_is_valid && 1U == status.motion.epoch);
    cmd.kind = S3_CONTROL_STOP;
    assert(s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_status(&status) && !status.b_is_collecting);
    g_adc_error = -1234567;
    cycle();
    assert(s3_integration_status(&status) && status.adc_status == -1234567);
    assert(6U == g_observed && 0U == g_freeze_calls);
    /* Complete production calibration flow, no shortcut: WHITE then BLACK
     * collected for 1 s each, then APPLY, for every sensor. The earlier reset
     * bug kept both minima at 0, so this valid pair was always rejected. */
    g_adc_error = 0;
    cycle();
    assert(0U == p_drive->navigation.accepted_calibration);

    for (unsigned channel = 0; channel < (unsigned)S3_SENSOR_COUNT; channel++)
    {
        for (unsigned surface = 0; surface < (unsigned)CAL_SURFACE_CASES;
             surface++)
        {
            uint16_t                    base = surface ? 2500 : 300;
            const s3_integration_kind_t collection_kind =
                surface ? S3_CONTROL_BLACK : S3_CONTROL_WHITE;
            cmd = (s3_integration_request_t){
                .kind   = collection_kind,
                .sensor = (ir_acquisition_sensor_id_t)channel,
            };
            assert(s3_integration_submit(&cmd));

            for (unsigned sample_index = 0;
                 sample_index < (CAL_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
                 sample_index++)
            {
                g_raw = (uint16_t)(base + (sample_index % 7U));
                cycle();
            }

            assert(s3_integration_status(&status) &&
                   status.b_did_command_succeed && !status.b_is_collecting);
            assert(status.calibration_sensor == channel);
        }

        assert(status.white.count >= 10U && 300 == (int)status.white.minimum &&
               306 == (int)status.white.maximum);
        assert(status.black.count >= 10U && 2500 == (int)status.black.minimum &&
               2506 == (int)status.black.maximum);
        cmd = (s3_integration_request_t){
            .kind   = S3_CONTROL_APPLY_CALIBRATION,
            .sensor = (ir_acquisition_sensor_id_t)channel};
        assert(s3_integration_submit(&cmd));
        cycle();
        assert(s3_integration_status(&status) && status.b_did_command_succeed);
        assert(status.navigation.accepted_calibration ==
               ((1U << (channel + 1U)) - 1U));
    }
    /* Overlapping surfaces must still be rejected and leave the sensor's
     * accepted calibration untouched. */
    cmd = (s3_integration_request_t){.kind   = S3_CONTROL_WHITE,
                                     .sensor = S3_SENSOR_LEFT};
    assert(s3_integration_submit(&cmd));

    for (unsigned sample_index = 0;
         sample_index < (CAL_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
         sample_index++)
    {
        g_raw = (uint16_t)(1000U + (sample_index % 7U));
        cycle();
    }

    cmd.kind = S3_CONTROL_BLACK;
    assert(s3_integration_submit(&cmd));

    for (unsigned sample_index = 0;
         sample_index < (CAL_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
         sample_index++)
    {
        g_raw = (uint16_t)(1003U + (sample_index % 7U));
        cycle();
    }

    cmd.kind = S3_CONTROL_APPLY_CALIBRATION;
    assert(s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_status(&status) && !status.b_did_command_succeed);
    assert(S3_ALL_SENSORS == p_drive->navigation.accepted_calibration);
    g_raw = 150;
    cycle();
    /* A START refused by Buddy 2's stillness check must not leave the
     * pre-run map frozen: the freeze is undone exactly once. */
    assert(line_navigation_readiness(&p_drive->navigation, g_clock_us) == 0U);
    gb_is_feedback_braked = false;
    cmd.kind              = S3_CONTROL_START;
    assert(s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_status(&status) && !status.b_did_command_succeed &&
           (unsigned)S3_DRIVE_OFF == status.state);
    assert(1U == g_freeze_calls && 1U == g_cancel_calls);
    gb_is_feedback_braked = true;
    assert(s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_status(&status) && status.b_did_command_succeed &&
           (unsigned)S3_DRIVE_FOLLOW == status.state);
    assert(2U == g_freeze_calls && 1U == g_cancel_calls);
    /* Idle/boot maxima cannot be mistaken for the current run's statistics.
     * START resets the run counters; include the interval causing a fault. */
    assert(0U == status.run_timing.samples &&
           0U == status.run_timing.max_gap_us);
    g_raw = 900;
    cycle();
    assert(s3_integration_status(&status) &&
           (unsigned)S3_DRIVE_FOLLOW == status.state);
    assert(1U == status.run_timing.samples &&
           S3_SAMPLE_PERIOD_US == status.run_timing.max_gap_us);
    assert(0U == status.run_timing.above_target &&
           0U == status.run_timing.above_safety);
    g_clock_us += 120000U;
    cycle();
    assert(s3_integration_status(&status) &&
           (unsigned)S3_DRIVE_FAULT == status.state);
    assert((120000U + S3_SAMPLE_PERIOD_US) == status.run_timing.max_gap_us &&
           1U == status.run_timing.above_safety);
    assert(status.run_timing.last_safety_us == g_clock_us);
    run_timing_t frozen = status.run_timing;
    cycle();
    cycle();
    assert(s3_integration_status(&status));
    assert(status.run_timing.samples == frozen.samples &&
           status.run_timing.last_safety_us == frozen.last_safety_us);

    /* One queued RESET waits for fresh feedback/health, then acknowledges once.
     * Ordinary commands are blocked; accepting the request is not success. */
    uint32_t sequence = status.command_sequence;
    cmd.kind          = S3_CONTROL_RESET;
    assert(s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_status(&status) &&
           status.command_sequence == sequence && 1U == g_reset_calls);
    cmd.kind = S3_CONTROL_START;
    assert(!s3_integration_submit(&cmd));

    for (unsigned sample_index = 0;
         sample_index < (RESET_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
         sample_index++)
    {
        cycle();
    }

    assert(s3_integration_status(&status) &&
           status.command_sequence == sequence + 1U &&
           status.b_did_command_succeed &&
           (unsigned)S3_DRIVE_OFF == status.state);
    assert(!s3_integration_runtime_test_view().b_has_reset &&
           !s3_integration_runtime_test_view().b_waits_for_reset);

    /* A genuine motion fault is cleared by the ONE accepted request, but
     * completion still waits until the next feedback publication. */
    cmd.kind = S3_CONTROL_START;
    assert(s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_status(&status) &&
           (unsigned)S3_DRIVE_FOLLOW == status.state &&
           0U == status.run_timing.samples);
    gb_has_feedback_fault = true;
    cycle();
    assert(s3_integration_status(&status) &&
           (unsigned)S3_DRIVE_FAULT == status.state);
    sequence = status.command_sequence;
    cmd.kind = S3_CONTROL_RESET;
    assert(s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_status(&status) &&
           status.command_sequence == sequence && !gb_has_feedback_fault);
    cycle();
    assert(s3_integration_status(&status) &&
           status.command_sequence == sequence + 1U &&
           status.b_did_command_succeed &&
           (unsigned)S3_DRIVE_OFF == status.state);

    /* STOP cancels an in-flight reset; the old completion must not produce a
     * later success or a second sequence increment after cancellation. */
    cmd.kind = S3_CONTROL_START;
    assert(s3_integration_submit(&cmd));
    cycle();
    gb_has_feedback_fault = true;
    cycle();
    gb_should_clear_fault = false;
    cmd.kind              = S3_CONTROL_RESET;
    assert(s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_runtime_test_view().b_waits_for_reset &&
           s3_integration_status(&status));
    sequence = status.command_sequence;
    cmd.kind = S3_CONTROL_STOP;
    assert(s3_integration_submit(&cmd));
    cycle();
    assert(s3_integration_status(&status) &&
           (unsigned)S3_DRIVE_FAULT == status.state &&
           status.command_sequence == sequence + 1U);
    sequence = status.command_sequence;

    for (unsigned sample_index = 0;
         sample_index < (CANCEL_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
         sample_index++)
    {
        cycle();
    }

    assert(s3_integration_status(&status) &&
           status.command_sequence == sequence &&
           !s3_integration_runtime_test_view().b_waits_for_reset &&
           !s3_integration_runtime_test_view().b_has_reset);

    /* Persistent fault: bounded refusal with no automatic start. */
    cmd.kind = S3_CONTROL_RESET;
    assert(s3_integration_submit(&cmd));
    cycle();

    for (unsigned sample_index = 0;
         sample_index < (RESET_TIMEOUT_LIMIT_US / S3_SAMPLE_PERIOD_US);
         sample_index++)
    {
        cycle();
    }

    assert(s3_integration_status(&status) && !status.b_did_command_succeed &&
           (unsigned)S3_DRIVE_FAULT == status.state);
    assert(status.command_sequence == sequence + 1U &&
           !s3_integration_runtime_test_view().b_waits_for_reset &&
           !s3_integration_runtime_test_view().b_has_reset);
#if defined(TM_MQTT) && TM_MQTT
    {
        /* After the run ended, the barcode trace goes out over Wi-Fi once,
         * binary summary first and binary end last; then nothing more. */
        uint8_t  line[MQTT_UTK_DEBUG_PAYLOAD_MAX];
        unsigned packet_size;
        unsigned lines      = 0U;
        unsigned first_kind = 99;
        unsigned last_kind  = 99;
        /* A busy capture lock must skip, not wait or abandon the capture. */
        gb_is_test_mutex_busy = true;
        assert(mqtt_utk_debug_take(line, sizeof line) == 0U);
        gb_is_test_mutex_busy = false;
        assert(s3_integration_runtime_test_capture_pending());

        for (;;)
        {
            packet_size = mqtt_utk_debug_take(line, sizeof line);

            if (0U == packet_size)
            {
                break;
            }

            assert(packet_size <= BCAP_WIRE_MAX && packet_size >= 6U &&
                   BCAP_WIRE_VERSION == line[0]);

            if (lines++ == 0U)
            {
                first_kind = line[5] & 3U;
            }

            last_kind = line[5] & 3U;
        }

        assert(lines >= 2U && 0U == first_kind && 3U == last_kind);
        assert(mqtt_utk_debug_take(line, sizeof line) == 0U);
        puts("PASS: barcode trace published once over the Wi-Fi debug hook "
             "after the run");
    }
#endif
#if APP_IR_DEBUG && defined(TM_MQTT) && TM_MQTT
    {
        uint8_t           bytes[IR_DEBUG_PACKET_SIZE];
        ir_debug_packet_t preview;
        ir_debug_sample_t decoded;

        for (unsigned sample_index = 0;
             sample_index < (DEBUG_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
             sample_index++)
        {
            cycle();
        }

        assert(mqtt_utk_ir_debug_take(bytes, sizeof bytes - 1U) == 0U);
        assert(mqtt_utk_ir_debug_take(bytes, sizeof bytes) == 16U);
        memcpy(preview.bytes, bytes, sizeof bytes);
        assert(ir_debug_unpack(&preview, &decoded) && 7U == decoded.raw_valid);
        assert(decoded.raw[0] == g_raw && decoded.raw[1] == g_raw &&
               decoded.raw[2] == g_raw);
        assert(mqtt_utk_ir_debug_take(bytes, sizeof bytes) == 0U);

        for (unsigned sample_index = 0;
             sample_index < (DEBUG_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
             sample_index++)
        {
            cycle();
        }

        gb_is_test_mutex_busy = true;
        assert(mqtt_utk_ir_debug_take(bytes, sizeof bytes) == 0U);
        gb_is_test_mutex_busy = false;
        assert(mqtt_utk_ir_debug_take(bytes, sizeof bytes) == 16U);

        for (unsigned sample_index = 0;
             sample_index < (DEBUG_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
             sample_index++)
        {
            cycle();
        }

        g_clock_us += 100001U;
        assert(mqtt_utk_ir_debug_take(bytes, sizeof bytes) == 0U &&
               !s3_integration_runtime_test_view().b_has_ir_debug);
        puts("PASS: production IR preview latest-only mailbox, physical "
             "channel mapping, bounded age and nonblocking network access");
        {
            uint8_t                control[CONTROL_DEBUG_PACKET_SIZE];
            control_debug_packet_t debug_packet;
            control_debug_sample_t debug_sample;

            for (unsigned sample_index = 0;
                 sample_index < (RESET_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
                 sample_index++)
            {
                cycle();
            }

            assert(mqtt_utk_control_debug_take(control, sizeof control - 1U) ==
                   0U);
            assert(mqtt_utk_control_debug_take(control, sizeof control) == 70U);
            memcpy(debug_packet.bytes, control, sizeof control);
            assert(control_debug_unpack(&debug_packet, &debug_sample));
            assert(debug_sample.b_is_motion_valid &&
                   debug_sample.setpoint_right == -50 &&
                   debug_sample.encoder_left == -123 &&
                   debug_sample.measured_right == -42);
            assert(7U == debug_sample.accepted_calibration &&
                   debug_sample.left_white_limit <= 1000U);
            assert(mqtt_utk_control_debug_take(control, sizeof control) == 0U);

            for (unsigned sample_index = 0;
                 sample_index < (RESET_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
                 sample_index++)
            {
                cycle();
            }

            gb_is_test_mutex_busy = true;
            assert(mqtt_utk_control_debug_take(control, sizeof control) == 0U);
            gb_is_test_mutex_busy = false;
            assert(mqtt_utk_control_debug_take(control, sizeof control) == 70U);

            for (unsigned sample_index = 0;
                 sample_index < (RESET_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
                 sample_index++)
            {
                cycle();
            }

            g_clock_us += 400001U;
            assert(mqtt_utk_control_debug_take(control, sizeof control) == 0U &&
                   !s3_integration_runtime_test_view().b_has_control_debug);
            /* A newer motion tick after the ADC stamp is fresh, not wrapped
             * negative age. Old or out-of-range feedback remains invalid. */
            g_debug_motion_advance = 100;

            for (unsigned sample_index = 0;
                 sample_index < (RESET_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
                 sample_index++)
            {
                cycle();
            }

            assert(mqtt_utk_control_debug_take(control, sizeof control) == 70U);
            memcpy(debug_packet.bytes, control, sizeof control);
            assert(control_debug_unpack(&debug_packet, &debug_sample));
            assert(debug_sample.b_is_motion_valid &&
                   (int32_t)(debug_sample.motion_timestamp_us -
                             debug_sample.timestamp_us) > 0);
            g_debug_motion_advance = 0;
            g_debug_motion_age     = 60000;

            for (unsigned sample_index = 0;
                 sample_index < (RESET_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
                 sample_index++)
            {
                cycle();
            }

            assert(mqtt_utk_control_debug_take(control, sizeof control) == 70U);
            memcpy(debug_packet.bytes, control, sizeof control);
            assert(control_debug_unpack(&debug_packet, &debug_sample));
            assert(debug_sample.b_has_motion_snapshot &&
                   !debug_sample.b_is_motion_valid &&
                   0 == debug_sample.measured_left &&
                   0 == debug_sample.encoder_left);
            g_debug_motion_age        = 0;
            gb_is_motion_out_of_range = true;

            for (unsigned sample_index = 0;
                 sample_index < (RESET_OBSERVE_WINDOW_US / S3_SAMPLE_PERIOD_US);
                 sample_index++)
            {
                cycle();
            }

            assert(mqtt_utk_control_debug_take(control, sizeof control) == 70U);
            memcpy(debug_packet.bytes, control, sizeof control);
            assert(control_debug_unpack(&debug_packet, &debug_sample));
            assert(!debug_sample.b_is_motion_valid &&
                   0 == debug_sample.setpoint_left &&
                   0 == debug_sample.duty_left);
            gb_is_motion_out_of_range = false;
            puts("PASS: controller debug real runtime, signed "
                 "motion/calibration fields, latest-only nonblocking mailbox "
                 "and stale drop");
        }
    }
#endif
    /* Remote commands share the physical owner/gates, but never acknowledge
     * somebody else's GP command_sequence. No network callback moves motors. */
    remote_control_link(0x12345678U, true);
    sequence = status.command_sequence;
    remote_send(REMOTE_START, 1, 0x12345678U, g_clock_us);
    remote_expect(1, REMOTE_QUEUED, REMOTE_NONE);
    cycle();
    remote_expect(1, REMOTE_REJECTED,
                  REMOTE_NOT_READY); /* Reject START while faulted. */
    gb_should_clear_fault = true;
    remote_send(REMOTE_RESET, 2, 0x12345678U, g_clock_us);
    remote_expect(2, REMOTE_QUEUED, REMOTE_NONE);
    cycle();
    assert(s3_integration_runtime_test_view().b_waits_for_reset);
    cycle();
    remote_expect(2, REMOTE_EXECUTED, REMOTE_NONE);
    assert((unsigned)S3_DRIVE_OFF == p_drive->state &&
           s3_integration_status(&status));
    sequence = status.command_sequence;
    remote_send(REMOTE_START, 3, 0x12345678U, g_clock_us);
    remote_expect(3, REMOTE_QUEUED, REMOTE_NONE);
    assert((unsigned)S3_DRIVE_OFF == p_drive->state);
    cycle();
    remote_expect(3, REMOTE_EXECUTED, REMOTE_NONE);
    assert((unsigned)S3_DRIVE_FOLLOW == p_drive->state &&
           s3_integration_status(&status));
    assert(status.command_sequence == sequence);
    remote_send(REMOTE_START, 3, 0x12345678U, g_clock_us);
    remote_expect(3, REMOTE_REJECTED, REMOTE_REPLAY);
    remote_send(REMOTE_STOP, 4, 0x12345678U,
                0); /* Accept STOP with a stale timestamp. */
    remote_expect(4, REMOTE_QUEUED, REMOTE_NONE);
    cycle();
    remote_expect(4, REMOTE_EXECUTED, REMOTE_NONE);
    remote_send(REMOTE_START, 5, 0x12345678U, g_clock_us);
    remote_expect(5, REMOTE_QUEUED, REMOTE_NONE);
    remote_send(REMOTE_STOP, 6, 0x12345678U, g_clock_us);
    remote_expect(5, REMOTE_CANCELLED, REMOTE_SUPERSEDED);
    remote_expect(6, REMOTE_QUEUED, REMOTE_NONE);
    cycle();
    remote_expect(6, REMOTE_EXECUTED, REMOTE_NONE);
    assert((unsigned)S3_DRIVE_OFF == p_drive->state);
    remote_send(REMOTE_START, 5, 0x12345678U, g_clock_us);
    remote_expect(5, REMOTE_REJECTED,
                  REMOTE_REPLAY); /* Deliver the late START after STOP. */
    remote_send(REMOTE_START, 7, 0x12345678U, g_clock_us);
    remote_expect(7, REMOTE_QUEUED, REMOTE_NONE);
    cmd.kind = S3_CONTROL_STOP;
    assert(s3_integration_submit(&cmd));
    remote_expect(7, REMOTE_CANCELLED, REMOTE_SUPERSEDED);
    cycle();
    assert((unsigned)S3_DRIVE_OFF == p_drive->state);
    remote_send(REMOTE_START, 8, 0x12345678U, g_clock_us);
    remote_expect(8, REMOTE_QUEUED, REMOTE_NONE);
    g_clock_us += REMOTE_MAX_AGE_US;
    cycle();
    remote_expect(8, REMOTE_REJECTED, REMOTE_STALE);
    assert((unsigned)S3_DRIVE_OFF == p_drive->state);
    remote_send(REMOTE_START, 9, 0x12345678U,
                g_clock_us - REMOTE_MAX_AGE_US - 1U);
    remote_expect(9, REMOTE_REJECTED, REMOTE_STALE);
    remote_send(REMOTE_START, 10, 0x12345678U, g_clock_us);
    remote_expect(10, REMOTE_QUEUED, REMOTE_NONE);
    remote_control_link(0, false);
    remote_expect(10, REMOTE_CANCELLED, REMOTE_SUPERSEDED);
    remote_control_link(0x87654321U, true);
    cycle();
    assert((unsigned)S3_DRIVE_OFF == p_drive->state);
    remote_send(REMOTE_START, 11, 0x12345678U, g_clock_us);
    remote_expect(11, REMOTE_REJECTED, REMOTE_STALE);
    {
        uint8_t wire_packet[REMOTE_STATUS_SIZE];
        assert(remote_control_status_take(wire_packet, sizeof wire_packet) ==
                   sizeof wire_packet &&
               remote_control_get_u32(wire_packet + 1) == 0x87654321U);
        assert(((unsigned)wire_packet[13] & (unsigned)REMOTE_AVAILABLE) != 0U);
        assert(remote_control_status_take(wire_packet, sizeof wire_packet) ==
               0U);
        g_clock_us += 500000U;
        assert(remote_control_status_take(wire_packet, sizeof wire_packet) ==
                   sizeof wire_packet &&
               0 == (int)wire_packet[13]); /* A stalled owner is unavailable. */
    }
    remote_control_physical_stop(true);
    remote_send(REMOTE_START, 1, 0x87654321U, g_clock_us);
    remote_expect(1, REMOTE_REJECTED, REMOTE_NOT_READY);
    remote_send(REMOTE_RESET, 2, 0x87654321U, g_clock_us);
    remote_expect(2, REMOTE_REJECTED, REMOTE_NOT_READY);
    remote_control_physical_stop(false);
    remote_send(REMOTE_START, 3, 0x87654321U, g_clock_us);
    remote_expect(3, REMOTE_QUEUED, REMOTE_NONE);
    remote_control_physical_stop(true);
    cycle();
    remote_expect(3, REMOTE_REJECTED, REMOTE_STALE);
    assert((unsigned)S3_DRIVE_OFF == p_drive->state);
    remote_control_physical_stop(false);
    /* STOP between precheck and execute's state change: no success reply or
     * wheel request is allowed to escape before the next STOP sampling tick. */
    const unsigned before_velocity = g_velocity_requests;
    gp_during_start                = stop_during_start;
    remote_send(REMOTE_START, 4, 0x87654321U, g_clock_us);
    remote_expect(4, REMOTE_QUEUED, REMOTE_NONE);
    cycle();
    remote_expect(5, REMOTE_QUEUED, REMOTE_NONE);
    remote_expect(4, REMOTE_REJECTED, REMOTE_STALE);
    assert((unsigned)S3_DRIVE_OFF == p_drive->state &&
           g_velocity_requests == before_velocity);
    cycle();
    remote_expect(5, REMOTE_EXECUTED, REMOTE_NONE);
    gp_during_start = reconnect_during_start;
    remote_send(REMOTE_START, 6, 0x87654321U, g_clock_us);
    remote_expect(6, REMOTE_QUEUED, REMOTE_NONE);
    cycle();
    {
        uint8_t wire_packet[REMOTE_REPLY_SIZE];
        assert(remote_control_reply_take(wire_packet, sizeof wire_packet) ==
                   sizeof wire_packet &&
               remote_control_get_u32(wire_packet + 2) == 6U);
        assert((((unsigned)wire_packet[1] >> 2U) & 3U) ==
                   (unsigned)REMOTE_REJECTED &&
               (((unsigned)wire_packet[1] >> 4U) & 7U) ==
                   (unsigned)REMOTE_STALE);
        assert(remote_control_get_u32(wire_packet + 6) == 0x87654321U);
    }
    assert((unsigned)S3_DRIVE_OFF == p_drive->state &&
           g_velocity_requests == before_velocity);
    /* An asynchronous RESET cancelled on reconnect stays a remote operation
     * during safe owner housekeeping, never a spurious GP acknowledgement. */
    remote_send(REMOTE_START, 1, 0xabcddcbaU, g_clock_us);
    remote_expect(1, REMOTE_QUEUED, REMOTE_NONE);
    cycle();
    remote_expect(1, REMOTE_EXECUTED, REMOTE_NONE);
    gb_has_feedback_fault = true;
    cycle();
    assert((unsigned)S3_DRIVE_FAULT == p_drive->state);
    remote_send(REMOTE_RESET, 2, 0xabcddcbaU, g_clock_us);
    remote_expect(2, REMOTE_QUEUED, REMOTE_NONE);
    cycle();
    assert(s3_integration_runtime_test_view().b_waits_for_reset &&
           s3_integration_runtime_test_view().b_is_remote_reset);
    assert(s3_integration_status(&status));
    sequence = status.command_sequence;
    remote_control_link(0, false);
    {
        uint8_t wire_packet[REMOTE_REPLY_SIZE];
        assert(remote_control_reply_take(wire_packet, sizeof wire_packet) ==
                   sizeof wire_packet &&
               remote_control_get_u32(wire_packet + 2) == 2U);
        assert((((unsigned)wire_packet[1] >> 2U) & 3U) ==
                   (unsigned)REMOTE_CANCELLED &&
               remote_control_get_u32(wire_packet + 6) == 0xabcddcbaU);
    }
    remote_control_link(0x43211234U, true);
    cycle();
    assert(s3_integration_status(&status) &&
           status.command_sequence == sequence);
    assert((unsigned)S3_DRIVE_OFF == p_drive->state &&
           !s3_integration_runtime_test_view().b_waits_for_reset &&
           !s3_integration_runtime_test_view().b_is_remote_reset);
    {
        uint8_t wire_packet[REMOTE_REPLY_SIZE];
        assert(remote_control_reply_take(wire_packet, sizeof wire_packet) ==
               0U);
        uint8_t       vector[REMOTE_COMMAND_SIZE] = {3, REMOTE_START};
        const uint8_t expected[] = {0x03, 0x01, 0x78, 0x56, 0x34, 0x12, 0x44,
                                    0x33, 0x22, 0x11, 0xdd, 0xcc, 0xbb, 0xaa};
        remote_control_put_u32(vector + 2, 0x12345678U);
        remote_control_put_u32(vector + 6, 0x11223344U);
        remote_control_put_u32(vector + 10, 0xaabbccddU);
        assert(!memcmp(vector, expected, sizeof vector));
        const uint8_t reply_vector[] = {0x04, 0x05, 0x78, 0x56, 0x34,
                                        0x12, 0x44, 0x33, 0x22, 0x11};
        const remote_control_request_t queued_request = {
            .action                = REMOTE_START,
            .TEST_REMOTE_ID_MEMBER = 0x12345678U,
            .epoch                 = 0x11223344U,
            .b_is_request_valid    = true,
        };
        s3_integration_runtime_test_reply(&queued_request, REMOTE_EXECUTED,
                                          REMOTE_NONE);
        assert(remote_control_reply_take(wire_packet, sizeof wire_packet) ==
                   sizeof wire_packet &&
               !memcmp(wire_packet, reply_vector, sizeof wire_packet));
        assert(remote_control_is_newer(1U, 0xffffffffU) &&
               !remote_control_is_newer(0xfffffffeU, 1U));
        uint8_t       status_vector[REMOTE_STATUS_SIZE];
        const uint8_t status_expected[] = {0x03, 0x44, 0x33, 0x22, 0x11,
                                           0xdd, 0xcc, 0xbb, 0xaa, 0x78,
                                           0x56, 0x34, 0x12, 0x03};
        g_clock_us                      = 0xaabbccddU;
        s3_integration_runtime_test_seed_status(g_clock_us);
        assert(
            remote_control_status_take(status_vector, sizeof status_vector) ==
            sizeof status_vector);
        assert(!memcmp(status_vector, status_expected, sizeof status_vector));
    }
    /* All 96 valid enum combinations retain their values. The request ID
     * and epoch keep all 32 bits, including the STOP/current-epoch rule. */
    for (unsigned action = REMOTE_START; action <= (unsigned)REMOTE_RESET;
         action++)
    {
        for (unsigned result = REMOTE_QUEUED;
             result <= (unsigned)REMOTE_CANCELLED; result++)
        {
            for (unsigned reason = REMOTE_NONE;
                 reason <= (unsigned)REMOTE_UNAVAILABLE; reason++)
            {
                const remote_control_request_t request = {
                    .action                = action,
                    .TEST_REMOTE_ID_MEMBER = UINT32_MAX,
                    .epoch                 = UINT32_MAX,
                    .b_is_request_valid    = true,
                };
                uint8_t reply[REMOTE_REPLY_SIZE];
                s3_integration_runtime_test_reply(&request, (uint8_t)result,
                                                  (uint8_t)reason);
                assert(remote_control_reply_take(reply, sizeof reply) == 10U);
                assert(4U == reply[0] &&
                       reply[1] == (action | (result << 2) | (reason << 4)));
                assert(remote_control_get_u32(reply + 2) == UINT32_MAX);
                assert(remote_control_get_u32(reply + 6) ==
                       ((unsigned)REMOTE_STOP == action
                            ? s3_integration_runtime_test_view().epoch
                            : UINT32_MAX));
            }
        }
    }
    /* Full-ring replacement keeps the newest six results in FIFO order.
     * The compact head/count must represent the full and wrapped states. */
    for (uint32_t ident = 1U; ident <= REPLY_LAST_ISSUED_ID; ident++)
    {
        const remote_control_request_t request = {
            .action                = REMOTE_RESET,
            .TEST_REMOTE_ID_MEMBER = ident,
            .epoch                 = 1U,
            .b_is_request_valid    = true,
        };
        s3_integration_runtime_test_reply(&request, REMOTE_REJECTED,
                                          REMOTE_NOT_READY);
        assert(s3_integration_runtime_test_view().count <=
               REMOTE_REPLY_RING_SIZE);
    }

    assert(REMOTE_REPLY_RING_SIZE == s3_integration_runtime_test_view().count);

    for (uint32_t ident = REPLY_FIRST_RETAINED_ID;
         ident <= REPLY_LAST_ISSUED_ID; ident++)
    {
        remote_expect(ident, REMOTE_REJECTED, REMOTE_NOT_READY);
    }

    assert(0U == s3_integration_runtime_test_view().count);
    puts("PASS: remote queued/executed/fault gates, RESET acknowledgement, "
         "duplicate/late/expired START, remote/GP STOP priority and "
         "reconnect cancellation");
    puts("PASS: production runtime STOP precedence, packed status, "
         "calibration, signed errors, map freeze, per-run timing/fault "
         "interval, one-press asynchronous RESET, cancellation and timeout");
    return 0;
}

/** @brief Encode the action, request ID, epoch and timestamp into one
 * control packet and deliver it to the receiver.
 */
static void remote_send (uint8_t action, uint32_t request_id, uint32_t epoch,
                         uint32_t timestamp)
{
    uint8_t request_packet[REMOTE_COMMAND_SIZE] = {REMOTE_VERSION, action};
    remote_control_put_u32(request_packet + 2, request_id);
    remote_control_put_u32(request_packet + 6, epoch);
    remote_control_put_u32(request_packet + 10, timestamp);
    remote_control_receive(request_packet, sizeof request_packet);
}

/** @brief Take the queued reply and verify its length, version, request
 * identity, packed result and reserved bit.
 */
static void remote_expect (uint32_t request_id, uint8_t result, uint8_t reason)
{
    uint8_t reply_packet[REMOTE_REPLY_SIZE];
    assert(remote_control_reply_take(reply_packet, sizeof reply_packet) ==
           sizeof reply_packet);
    assert(REMOTE_REPLY_VERSION == reply_packet[0] &&
           remote_control_get_u32(reply_packet + 2) == request_id);
    assert((((unsigned)reply_packet[1] >> 2U) & 3U) == result &&
           (((unsigned)reply_packet[1] >> 4U) & 7U) == reason);
    assert((reply_packet[1] & 0x80U) == 0U);
}

/** @brief Clear the one-shot hook and inject a STOP at the current simulated
 * time during start preparation.
 */
static void stop_during_start (void)
{
    gp_during_start = 0;
    remote_send(REMOTE_STOP, 5, 0x87654321U, g_clock_us);
}

/** @brief Clear the one-shot hook and simulate a disconnect followed by a
 * new control epoch during start preparation.
 */
static void reconnect_during_start (void)
{
    gp_during_start = 0;
    remote_control_link(0, false);
    remote_control_link(0xabcddcbaU, true);
}
#endif /* TEST_SLOPE_LINK */

/** @brief Advance one IR sampling period and execute the sampling task until
 * its second sleep. Require the lock to be free on return.
 */
static void cycle (void)
{
    g_sleeps    = 0;
    g_clock_us += S3_SAMPLE_PERIOD_US;

    if (setjmp(g_task_exit) == 0)
    {
        s3_integration_runtime_test_cycle();
    }

    assert(!g_locks);
}

/* End of file. */

