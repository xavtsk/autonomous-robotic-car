/** @file cal_defaults_main.c
 * @brief Built-in calibration at boot (S3_BUILTIN_CALIBRATION=1), through the
 * production runtime and operator logic.
 *
 * Derived from operator_main.c.
 * Original header: End-to-end host test of the operator controls: simulated
 * GP20/GP21 presses
 * -> operator_logic -> the PRODUCTION s3_integration runtime (fake ADC/motion
 * boundaries, as in runtime_main.c) -> published status -> operator_logic.
 * Proves the button walkthrough really calibrates, starts and stops through
 * the single controller owner. Not a timing or hardware test.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "cal_defaults_main.h"
#include "../../app_program/app_types.h"
#include "../../app_program/controller/s3_integration.h"
#include "../../app_program/controller/op_logic.h"
#include "../../app_program/controller/builtin_calibration.h"
#include "../../app_program/subsystem3/ir_bench_config.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static jmp_buf  g_boundary;
static unsigned g_sleeps;
static unsigned g_locked;
static unsigned g_starts_requested;
static unsigned g_stops;
static uint32_t g_clock_value;
static uint16_t g_raw[3]              = {150, 150, 150};
static bool     gb_is_feedback_braked = true;
static uint8_t  g_fb_command_id; /* Echo the last accepted velocity command as
                                  * Buddy 2 does.
                                  */
static bool gb_has_feedback_fault;

static float32_t g_last_l;
static float32_t g_last_r;

/* Console capture: everything the operator would read on USB. */
static char   g_console[1U << 16U];
static size_t g_console_len;

static op_logic_t              g_operator;
static op_logic_button_t       g_go_b;
static op_logic_button_t       g_stop_b;
static bool                    gb_is_go_down;
static bool                    gb_is_stop_down;
static s3_integration_status_t g_fixture_status;

static void run_ms (unsigned milliseconds);

static void press_go (unsigned held_ms);

static void press_stop (void);

static size_t mark (void);

static bool said_since (size_t from, const char * p_text);

/** @brief Return the fixture mutex ID without allocating a kernel object.
 */
ID tk_cre_mtx (const T_CMTX * p_mutex_config)
{
    (void)p_mutex_config;
    return 1;
}

/** @brief Require the expected unlocked sampling mutex and mark it held in
 * the fixture.
 */
ER tk_loc_mtx (ID h_mutex, TMO timeout)
{
    (void)timeout;
    assert(1 == h_mutex && !g_locked);
    g_locked = 1;
    return E_OK;
}

/** @brief Require the acquired fake mutex, then release it and report
 * success.
 */
ER tk_unl_mtx (ID h_mutex)
{
    assert(1 == h_mutex && g_locked);
    g_locked = 0;
    return E_OK;
}

/** @brief Return the fixed cyclic-handler ID without starting a timer.
 */
ID tk_cre_cyc (const T_CCYC * p_cyclic_config)
{
    (void)p_cyclic_config;
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

/** @brief Require the lock to be free on sleep and return control to the
 * fixture at the second sleep boundary.
 */
ER tk_slp_tsk (TMO timeout)
{
    (void)timeout;
    assert(!g_locked);

    if (++g_sleeps == 2U)
    {
        longjmp(g_boundary, 1);
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
ID app_start_task (FP p_task_entry, PRI priority, UW core, const char * p_name)
{
    (void)p_task_entry;
    (void)priority;
    (void)core;
    (void)p_name;
    return 3;
}

/** @brief Return the current simulated microsecond count for the timer read.
 */
uint32_t runtime_read_clock (uint32_t address)
{
    (void)address;
    return g_clock_value;
}

/** @brief Report successful IR initialization without configuring physical
 * inputs.
 */
int32_t ir_sensor_init (const ir_acquisition_config_t * p_config)
{
    (void)p_config;
    return E_OK;
}

/** @brief Supply three valid timestamped IR readings with deterministic
 * jitter on each configured raw channel value.
 */
int32_t ir_sensor_read_frame (ir_acquisition_frame_t * p_frame)
{
    *p_frame = (ir_acquisition_frame_t){.timestamp_us = g_clock_value};

    for (unsigned sensor_index = 0;
         sensor_index < (sizeof g_raw / sizeof g_raw[0]); sensor_index++)
    {
        p_frame->sensor[sensor_index] = (ir_acquisition_sample_t){
            g_clock_value,
            (uint16_t)(g_raw[sensor_index] + (g_clock_value / 10000U % 5U)), 0,
            true, true};
    }

    return E_OK;
}

/** @brief Supply a valid simulated feedback sample with the configured
 * brake, command and fault state.
 */
bool motion_get_feedback (motion_feedback_t * p_feedback)
{
    *p_feedback =
        (motion_feedback_t){.timestamp_us        = g_clock_value,
                            .epoch               = 1,
                            .b_is_feedback_valid = true,
                            .b_is_braked         = gb_is_feedback_braked,
                            .command_id          = g_fb_command_id,
                            .fault = gb_has_feedback_fault ? MOTION_FAULT_STALL
                                                           : MOTION_FAULT_NONE};
    return true;
}

/** @brief Retain the requested wheel velocities and mark command ID one
 * active.
 */
motion_result_t motion_request_velocity (float32_t left_mps,
                                         float32_t right_mps)
{
    g_last_l        = left_mps;
    g_last_r        = right_mps;
    g_fb_command_id = 1;
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

/** @brief Accept the simulated motion command immediately without starting a
 * motor.
 */
motion_result_t motion_request_manoeuvre (motion_manoeuvre_t manoeuvre)
{
    (void)manoeuvre;
    return MOTION_OK;
}

/** @brief Count the stop request and clear the active simulated command ID.
 */
void stop (void)
{
    g_stops++;
    g_fb_command_id = 0;
}

/** @brief Report an empty motion queue and leave the output untouched.
 */
bool motion_get_report (motion_report_t * p_report)
{
    (void)p_report;
    return false;
}

/** @brief Return the fixed command ID used in simulated motion feedback.
 */
uint8_t motion_command_id (void)
{
    return 1;
}

/** @brief Accept the reset immediately without changing simulated motion
 * state.
 */
bool motion_reset (void)
{
    return true;
}

/** @brief Accept the navigation heartbeat without simulating a watchdog.
 */
void motion_navigation_heartbeat (bool b_is_enabled)
{
    (void)b_is_enabled;
}

/** @brief Count an accepted start preparation request.
 */
bool vehicle_controller_prep_start (line_navigation_t * p_navigation)
{
    (void)p_navigation;
    g_starts_requested++;
    return true;
}

/** @brief Accept the supplied mapping without modifying the fake
 * configuration.
 */
bool vehicle_controller_set_mapping (const line_navigation_action_t actions[26])
{
    (void)actions;
    return true;
}

/** @brief Accept the start handshake without changing the fixture’s
 * configuration.
 */
void vehicle_controller_cancel_start (void)
{
}

/** @brief Accept the start handshake without changing the fixture’s
 * configuration.
 */
void vehicle_controller_commit_start (void)
{
}

/** @brief Accept the controller observation without retaining a copy.
 */
void vehicle_controller_observe (const s3_integration_status_t * p_status)
{
    (void)p_status;
}

/** @brief Append formatted diagnostics to the bounded test console outside
 * the sampling lock. Return the formatter's character count.
 */
int tm_printf (const UB * p_format, ...)
{
    va_list arguments;
    int     written;
    assert(!g_locked);
    va_start(arguments, p_format);
    written =
        vsnprintf(g_console + g_console_len, sizeof(g_console) - g_console_len,
                  (const char *)p_format, arguments);
    va_end(arguments);

    if ((written > 0) &&
        ((g_console_len + (size_t)written) < sizeof(g_console)))
    {
        g_console_len += (size_t)written;
    }

    return written;
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

/** @brief Boot the runtime with built-in calibration and simulate button
 * input to verify readiness, speed limits and recalibration.
 */
int main (void)
{
    const op_logic_port_t port = {s3_integration_submit, tm_printf};
    size_t                console_begin;
    assert(s3_integration_init() == E_OK);
    const s3_drive_t * p_drive = s3_integration_test_drive();
    /* All three accepted through the normal validation path at init. */
    assert(S3_ALL_SENSORS == p_drive->navigation.accepted_calibration);
    {
        const uint16_t measured[S3_SENSOR_COUNT][4] = {
            S3_BUILTIN_CAL_BARCODE, S3_BUILTIN_CAL_LEFT, S3_BUILTIN_CAL_RIGHT};
        const uint16_t bench[S3_SENSOR_COUNT][2] = {
            {IR_BENCH_WHITE_RAW, IR_BENCH_BLACK_RAW},
            {IR_BENCH_LEFT_WHITE_RAW, IR_BENCH_LEFT_BLACK_RAW},
            {IR_BENCH_RIGHT_WHITE_RAW, IR_BENCH_RIGHT_BLACK_RAW}};

        for (unsigned sensor_index = 0;
             sensor_index < (unsigned)S3_SENSOR_COUNT; sensor_index++)
        {
            const ir_reflectance_calibration_t * p_calibration =
                &p_drive->navigation.config.calibration[sensor_index];
            ir_reflectance_t sensor;
            uint16_t         dark;
            /* The boot validation and bench must use the same references;
             * every measured endpoint must classify on the correct side. */
            assert(p_calibration->white_raw == bench[sensor_index][0] &&
                   p_calibration->black_raw == bench[sensor_index][1]);
            assert(ir_reflectance_init(&sensor, p_calibration));

            for (unsigned sample_index = 0;
                 sample_index < (sizeof measured[0] / sizeof measured[0][0]);
                 sample_index++)
            {
                assert(ir_reflectance_update(
                    &sensor, measured[sensor_index][sample_index], &dark));

                if (sample_index < 2U)
                {
                    assert((int)dark <= 400 &&
                           IR_SURFACE_WHITE == (int)sensor.surface);
                }
                else
                {
                    assert((int)dark >= 600 &&
                           IR_SURFACE_BLACK == (int)sensor.surface);
                }
            }
        }
    }
    assert(op_logic_init(&g_operator, &port));
    op_logic_button_init(&g_go_b);
    op_logic_button_init(&g_stop_b);
    run_ms(300);
    /* Straight to READY: no 6-step walkthrough. */
    assert(said_since(0, "Using built-in calibration."));
    assert(said_since(0, "READY: put the car on the line"));
    assert(!said_since(0, "Calibration 1/6"));
    /* Half-dark line inputs stay independent of each session's measured
     * calibration: line present, no junction, so a START drives. */
    for (unsigned sensor_index = 0; sensor_index < (unsigned)S3_SENSOR_COUNT;
         sensor_index++)
    {
        const ir_reflectance_calibration_t * p_calibration =
            &p_drive->navigation.config.calibration[sensor_index];

        /* Two 12-bit samples sum exactly before promoting to unsigned. */
        g_raw[sensor_index] =
            ((unsigned)S3_SENSOR_BARCODE == sensor_index)
                ? p_calibration->white_raw
                : (uint16_t)((unsigned)(p_calibration->white_raw +
                                        p_calibration->black_raw) /
                             2U);
    }

    run_ms(200);
    assert(p_drive->output.line.b_is_line_present &&
           !p_drive->output.line.b_has_junction_candidate);
    console_begin = mark();
    press_go(80);
    run_ms(3300);
    assert(said_since(console_begin, "RUNNING."));
    run_ms(500);
    assert((unsigned)S3_DRIVE_FOLLOW == p_drive->state);
    press_stop();
    assert((unsigned)S3_DRIVE_OFF == p_drive->state &&
           said_since(console_begin, "Run ended: OFF"));
    /* Minimum wheel speed: a slow positive request is raised, others pass. */
    const float32_t floor_mps = s3_integration_test_floor_mps();
    assert(s3_integration_test_velocity(0.159f, floor_mps * 0.5f) ==
               (unsigned)MOTION_OK &&
           0.159f == g_last_l && floor_mps == g_last_r);
    assert(s3_integration_test_velocity(0.10f, 0.10f) == (unsigned)MOTION_OK &&
           0.10f == g_last_l && 0.10f == g_last_r);
    assert(s3_integration_test_velocity(0.0f, -0.05f) == (unsigned)MOTION_OK &&
           0.0f == g_last_l && -0.05f == g_last_r);
    /* Hand recalibration still available and replaces the built-in values. */
    console_begin = mark();
    press_go(2100);
    assert(said_since(console_begin, "Recalibrating all sensors.") &&
           said_since(console_begin, "Calibration 1/6"));

    if (getenv("OP_TEST_VERBOSE"))
    {
        g_console[g_console_len] = 0;
        fputs(g_console, stdout);
    }

    puts("PASS: built-in calibration accepted at boot via normal validation,"
         " operator skips walkthrough, START/STOP work, minimum wheel speed "
         "floor, long press still recalibrates");
    return 0;
}

/** @brief Convert the simulated microsecond clock to whole milliseconds.
 */
static uint32_t now_ms (void)
{
    return g_clock_value / 1000U;
}

/** @brief Advance one 10 ms period by running the controller cycle and then
 * polling the operator.
 */
static void period (void)
{
    bool             b_has_stop_edge = false;
    bool             b_has_go_edge   = false;
    op_logic_press_t go_press;
    g_sleeps       = 0;
    g_clock_value += 10000U;

    if (setjmp(g_boundary) == 0)
    {
        s3_integration_test_sample_period();
    }

    assert(!g_locked);
    (void)op_logic_button_update(&g_stop_b, gb_is_stop_down, now_ms(),
                                 &b_has_stop_edge);
    go_press = op_logic_button_update(&g_go_b, gb_is_go_down, now_ms(),
                                      &b_has_go_edge);
    op_logic_step(
        &g_operator, now_ms(), go_press, op_logic_button_held(&g_stop_b),
        op_logic_button_held(&g_go_b),
        s3_integration_status(&g_fixture_status) ? &g_fixture_status : 0);
}

/** @brief Advance the operator fixture in 10 ms ticks through the requested
 * interval.
 */
static void run_ms (unsigned milliseconds)
{
    for (unsigned elapsed_ms = 0; elapsed_ms < milliseconds; elapsed_ms += 10U)
    {
        period();
    }
}

/** @brief Hold the simulated start button for the requested milliseconds,
 * release it and allow 60 ms for debounce.
 */
static void press_go (unsigned held_ms)
{
    gb_is_go_down = true;
    run_ms(held_ms);
    gb_is_go_down = false;
    run_ms(60);
}

/** @brief Hold the simulated stop button for 60 ms, then release it for 60
 * ms.
 */
static void press_stop (void)
{
    gb_is_stop_down = true;
    run_ms(60);
    gb_is_stop_down = false;
    run_ms(60);
}

/** @brief Return the current console offset so a check can inspect later
 * output.
 */
static size_t mark (void)
{
    return g_console_len;
}

/** @brief Terminate the fixture console buffer and search for the requested
 * text starting at the supplied offset.
 */
static bool said_since (size_t from, const char * p_text)
{
    g_console[g_console_len] = 0;
    return strstr(g_console + from, p_text) != NULL;
}

/* End of file. */

