/** @file operator_main.c
 * @brief End-to-end host test of the operator controls: simulated GP20/GP21
 * presses -> operator_logic -> the PRODUCTION s3_integration runtime
 * (fake ADC/motion boundaries, as in runtime_main.c) -> published status
 * -> operator_logic.
 *
 * Proves the button walkthrough really calibrates, starts and stops through
 * the single controller owner. Not a timing or hardware test.
 * Unsigned casts spell out the original modulo conversion.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "operator_main.h"
#include "../../app_program/controller/s3_integration.h"
#include "../../app_program/controller/op_logic.h"
#include "../../app_program/controller/remote_control.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static jmp_buf  g_boundary;
static unsigned g_sleeps;
static unsigned g_locks;
static unsigned g_starts_requested;
static unsigned g_stops;
static uint32_t g_clock_us;
static uint16_t g_raw[3]              = {150, 150, 150};
static bool     gb_is_feedback_braked = true;
static uint8_t  g_fb_command_id; /* Echo the last accepted velocity command as
                                  * Buddy 2 does.
                                  */
static bool gb_has_feedback_fault;

/* Console capture: everything the operator would read on USB. */
static char   g_console[1U << 16U];
static size_t g_console_length;

static op_logic_t              g_operator;
static op_logic_button_t       g_go_button;
static op_logic_button_t       g_stop_button;
static bool                    gb_is_go_down;
static bool                    gb_is_stop_down;
static s3_integration_status_t g_operator_status;
static uint32_t                g_remote_stop_seen;

static void period (void);

static void run_ms (unsigned milliseconds);

static void press_go (unsigned held_ms);

static void press_stop (void);

static size_t mark (void);

static bool said_since (size_t from, const char * p_text);

static void set_all (uint16_t darkness);

static void calibrate_all (void);

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
    assert(!g_locks);

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
ID app_start_task (FP p_handler, PRI priority, UW core, const char * p_name)
{
    (void)p_handler;
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
    return g_clock_us;
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
    *p_frame = (ir_acquisition_frame_t){.timestamp_us = g_clock_us};

    for (unsigned sensor_index = 0; sensor_index < (unsigned)S3_SENSOR_COUNT;
         sensor_index++)
    {
        p_frame->sensor[sensor_index] = (ir_acquisition_sample_t){
            g_clock_us,
            (uint16_t)(g_raw[sensor_index] + (g_clock_us / 10000U % 5U)), 0,
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
        (motion_feedback_t){.timestamp_us        = g_clock_us,
                            .epoch               = 1,
                            .b_is_feedback_valid = true,
                            .b_is_braked         = gb_is_feedback_braked,
                            .command_id          = g_fb_command_id,
                            .fault = gb_has_feedback_fault ? MOTION_FAULT_STALL
                                                           : MOTION_FAULT_NONE};
    return true;
}

/** @brief Accept both requested wheel velocities and assign the simulated
 * command ID one.
 */
motion_result_t motion_request_velocity (float32_t left_mps,
                                         float32_t right_mps)
{
    (void)left_mps;
    (void)right_mps;
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
motion_result_t motion_request_manoeuvre (motion_manoeuvre_t manoeuvre_kind)
{
    (void)manoeuvre_kind;
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

/** @brief Accept cancellation without changing the fixture's configuration.
 */
void vehicle_controller_cancel_start (void)
{
}

/** @brief Accept completion without changing the fixture's configuration.
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
    assert(!g_locks);
    va_start(arguments, p_format);
    written = vsnprintf(g_console + g_console_length,
                        sizeof(g_console) - g_console_length,
                        (const char *)p_format, arguments);
    va_end(arguments);

    if ((written > 0) &&
        ((g_console_length + (size_t)written) < sizeof(g_console)))
    {
        g_console_length += (size_t)written;
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

#define INHIBITED_GO_PRESS_COUNT 3

/** @brief Apply simulated GP20 and GP21 presses during calibration,
 * countdown and motion. Check the submitted requests and console
 * responses.
 * @details Exercise the initial release gate, both release orders after a
 * simultaneous press, and GP21 and dashboard STOP during the countdown.
 * Verify recoverable faults during driving and calibration, acceptance
 * of a fresh START after a halt, and a full white/black calibration.
 */
int main (void)
{
    const op_logic_port_t port = {s3_integration_submit, tm_printf};
    size_t                console_begin;
    assert(s3_integration_init() == E_OK);
    const s3_drive_t * p_drive = s3_integration_operator_test_drive();
    assert(op_logic_init(&g_operator, &port));
    op_logic_button_init(&g_go_button);
    op_logic_button_init(&g_stop_button);

    /* 0. GP21 held from power-on: STOP is sent once, and no GP20 press is
     *    accepted while it is held (GP21 never "arms", but holding counts). */
    gb_is_stop_down = true;
    run_ms(300);

    for (int press_index = 0; press_index < INHIBITED_GO_PRESS_COUNT;
         press_index++)
    {
        press_go(80);
    }

    run_ms(500);
    assert(said_since(0, "STOP sent."));
    assert(strstr(strstr(g_console, "STOP sent.") + 1, "STOP sent.") ==
           NULL); /* Send STOP once for this press. */
    assert(said_since(0, "Ignored GP20: release GP21 first."));
    assert(!said_since(0, "Sampling for 1 s"));
    gb_is_stop_down = false;
    run_ms(300);

    /* 1. GP20 held at power-up must not count as a press. */
    op_logic_button_init(&g_go_button);
    console_begin = mark();
    gb_is_go_down = true;
    run_ms(500);
    gb_is_go_down = false;
    run_ms(200);
    assert(said_since(0, "Calibration 1/6: put the front-LEFT line sensor "
                         "(GP27) over plain WHITE"));
    assert(!said_since(console_begin, "Sampling for 1 s"));

    /* 2. Full walkthrough calibrates all three sensors for real. */
    console_begin = mark();
    calibrate_all();
    assert(said_since(console_begin,
                      "Calibration 2/6: put the front-LEFT line sensor "
                      "(GP27) over the BLACK line"));
    assert(said_since(console_begin,
                      "front-RIGHT line sensor (GP26) calibrated."));
    assert(said_since(console_begin, "BARCODE sensor (GP28) calibrated."));
    assert(said_since(console_begin, "All 3 sensors calibrated."));
    assert(said_since(console_begin, "READY: put the car on the line"));
    assert(S3_ALL_SENSORS == p_drive->navigation.accepted_calibration);

    /* P1a. GP21 held, GP20 pressed and released: nothing starts. */
    set_all(300);
    console_begin   = mark();
    gb_is_stop_down = true;
    run_ms(100);
    press_go(80);
    run_ms(4000);
    assert(said_since(console_begin, "Ignored GP20: release GP21 first.") &&
           !said_since(console_begin, "Starting in"));
    assert(0U == g_starts_requested &&
           (unsigned)S3_DRIVE_OFF == p_drive->state);
    /* P1b. GP21 held through a whole countdown period: START never sent. */
    gb_is_stop_down = false;
    run_ms(100);
    console_begin = mark();
    press_go(80);
    run_ms(500);
    assert(said_since(console_begin, "Starting in 3..."));
    gb_is_stop_down = true;
    run_ms(4000);
    assert(said_since(console_begin, "Start cancelled.") &&
           !said_since(console_begin, "RUNNING"));
    assert(0U == g_starts_requested &&
           (unsigned)S3_DRIVE_OFF == p_drive->state);
    gb_is_stop_down = false;
    run_ms(100);
    /* P1c. Both pressed together, either release order: GP20 is ignored. */
    console_begin   = mark();
    gb_is_go_down   = true;
    gb_is_stop_down = true;
    run_ms(200);
    gb_is_stop_down = false;
    run_ms(100);
    gb_is_go_down = false;
    run_ms(4000);
    gb_is_go_down   = true;
    gb_is_stop_down = true;
    run_ms(200);
    gb_is_go_down = false;
    run_ms(100);
    gb_is_stop_down = false;
    run_ms(4000);
    assert(!said_since(console_begin, "Starting in") &&
           0U == g_starts_requested);

    /* 3. STOP during the countdown cancels; START is never requested. */
    set_all(300);
    console_begin = mark();
    press_go(80);
    run_ms(1000);
    assert(said_since(console_begin, "Starting in 3..."));
    press_stop();
    assert(said_since(console_begin, "STOP sent.") &&
           said_since(console_begin, "Start cancelled."));
    run_ms(3000);
    assert(0U == g_starts_requested &&
           (unsigned)S3_DRIVE_OFF == p_drive->state);

    /* Dashboard STOP also cancels a GP20 countdown which has not submitted
     * its START yet; no duplicate physical STOP steals the correlated reply. */
    console_begin = mark();
    press_go(80);
    run_ms(1000);
    assert((unsigned)OP_PHASE_COUNTDOWN == g_operator.phase);
    {
        uint8_t stop_request[REMOTE_COMMAND_SIZE] = {REMOTE_VERSION,
                                                     REMOTE_STOP};
        remote_control_put_u32(stop_request + 2, 1);
        remote_control_receive(stop_request, sizeof stop_request);
        assert(remote_control_stop_sequence() == 1U);
        run_ms(4000);
        assert(0U == g_starts_requested &&
               (unsigned)S3_DRIVE_OFF == p_drive->state &&
               (unsigned)OP_PHASE_READY == g_operator.phase);
        assert(said_since(console_begin, "Start cancelled."));
        assert(remote_control_reply_take(stop_request, sizeof stop_request) ==
                   REMOTE_REPLY_SIZE &&
               (((unsigned)stop_request[1] >> 2U) & 3U) ==
                   (unsigned)REMOTE_QUEUED);
        assert(remote_control_reply_take(stop_request, sizeof stop_request) ==
                   REMOTE_REPLY_SIZE &&
               (((unsigned)stop_request[1] >> 2U) & 3U) ==
                   (unsigned)REMOTE_EXECUTED);
    }

    /* 4. Buddy 2 not braked: START refused with a readable reason. */
    gb_is_feedback_braked = false;
    console_begin         = mark();
    press_go(80);
    run_ms(3300);
    assert(said_since(console_begin, "Starting in 1...") &&
           said_since(console_begin, "START refused: state=OFF"));
    assert((unsigned)S3_DRIVE_OFF == p_drive->state);
    gb_is_feedback_braked = true;

    /* 5. Real START; sensors all on white -> controller halts on line loss,
     *    and the operator reports why. */
    console_begin = mark();
    press_go(80);
    run_ms(3300);
    assert(said_since(console_begin, "RUNNING. GP21 = STOP."));
    run_ms(2000);
    assert((unsigned)S3_DRIVE_HALTED == p_drive->state);
    assert(said_since(console_begin, "Run ended: HALTED - line lost."));

    /* 6. START again from HALTED, then GP21 stops the run. */
    /* Both line sensors partly over the tape: line present, no junction. */
    g_raw[S3_SENSOR_LEFT]  = 900;
    g_raw[S3_SENSOR_RIGHT] = 900;
    console_begin          = mark();
    press_go(80);
    run_ms(3300);
    assert(said_since(console_begin, "RUNNING."));
    run_ms(1000);

    if (p_drive->state != (unsigned)S3_DRIVE_FOLLOW)
    {
        printf("state=%d reason=%d\n", p_drive->state, p_drive->reason);
    }

    assert((unsigned)S3_DRIVE_FOLLOW == p_drive->state &&
           !said_since(console_begin, "Run ended:"));
    {
        unsigned before = g_stops;
        press_stop();
        assert(g_stops > before);
    }
    assert((unsigned)S3_DRIVE_OFF == p_drive->state);
    assert(
        said_since(console_begin, "Run ended: OFF - ok/stopped by operator."));

    /* P3a. Motor fault while driving -> FAULT. Holding GP20 must NOT enter a
     *      calibration the controller would refuse; it asks for RESET. */
    console_begin = mark();
    press_go(80);
    run_ms(3300);
    assert(said_since(console_begin, "RUNNING."));
    gb_has_feedback_fault = true;
    run_ms(100);
    assert(
        (unsigned)S3_DRIVE_FAULT == p_drive->state &&
        said_since(console_begin, "Run ended: FAULT - motor/encoder fault."));
    assert(said_since(console_begin,
                      "FAULT: motor/encoder fault. Fix the cause, then "
                      "press GP20 to reset."));
    console_begin = mark();
    press_go(2100);
    run_ms(600); /* The fault stays active through the reset timeout. */
    assert(said_since(console_begin, "Reset refused") &&
           !said_since(console_begin, "Recalibrating") &&
           !said_since(console_begin, "Sampling refused"));
    assert((unsigned)S3_DRIVE_FAULT == p_drive->state);
    gb_has_feedback_fault = false;
    console_begin         = mark();
    press_go(80);
    run_ms(200);
    assert(said_since(console_begin, "Fault reset.") &&
           (unsigned)S3_DRIVE_OFF == p_drive->state);
    /* P3b. FAULT while at a calibration prompt: GP20 resets, then calibration
     *      continues (white-box fault injection; the reset path is real). */
    console_begin = mark();
    press_go(2100);
    assert(said_since(console_begin, "Calibration 1/6"));
    s3_integration_operator_test_sensor_fault();
    run_ms(100);
    console_begin = mark();
    run_ms(5100);
    assert(said_since(
        console_begin,
        "FAULT: IR sensor fault. Calibration is blocked. Press GP20 to reset"));
    press_go(80);
    run_ms(200);
    assert(said_since(console_begin, "Fault reset.") &&
           said_since(console_begin, "Calibration 1/6") &&
           !said_since(console_begin, "Sampling refused"));
    assert((unsigned)S3_DRIVE_OFF == p_drive->state);
    /* Finish calibration to return to READY. */
    calibrate_all();
    assert(said_since(console_begin, "All 3 sensors calibrated."));

    /* 7. Hold GP20 2 s in READY -> recalibration starts at step 1. */
    set_all(300);
    console_begin = mark();
    press_go(2100);
    assert(said_since(console_begin, "Recalibrating all sensors.") &&
           said_since(console_begin, "Calibration 1/6"));

    /* 8. Overlapping white/black is rejected and the same sensor is redone. */
    console_begin = mark();
    set_all(1000);
    press_go(80);
    run_ms(1200);
    press_go(80);
    run_ms(1300);
    assert(said_since(console_begin, "front-LEFT line sensor (GP27) REJECTED"));
    assert(said_since(console_begin, "Calibration 1/6"));

    /* 9. STOP while sampling returns to the same step. */
    console_begin = mark();
    press_go(80);
    run_ms(200);
    press_stop();
    run_ms(1500);
    assert(said_since(console_begin, "STOP sent.") &&
           said_since(console_begin, "Calibration 1/6"));
    assert(!said_since(console_begin, "Calibration 2/6"));

    if (getenv("OP_TEST_VERBOSE"))
    {
        g_console[g_console_length] = 0;
        fputs(g_console, stdout);
    }

    puts("PASS: operator buttons -> production runtime: GP21 held at "
         "power-on/before/during countdown and both-button release orders "
         "block START, power-up GP20 press ignored, full 3-sensor "
         "calibration, countdown + STOP cancel, refused START reason, "
         "START/line-lost report, restart + STOP, FAULT: long press asks "
         "for RESET (no calibration trap) and reset from a calibration "
         "prompt, long-press recalibration, overlap rejection, STOP during "
         "sampling");
    return 0;
}

/** @brief Convert the simulated microsecond clock to whole milliseconds.
 */
static uint32_t now_ms (void)
{
    return g_clock_us / 1000U;
}

/** @brief Advance one 10 ms period by running the controller cycle and then
 * polling the operator.
 */
static void period (void)
{
    bool             b_has_stop_edge = false;
    bool             b_has_go_edge   = false;
    op_logic_press_t go_press;
    g_sleeps    = 0;
    g_clock_us += 10000U;

    if (setjmp(g_boundary) == 0)
    {
        s3_integration_operator_test_cycle();
    }

    assert(!g_locks);
    (void)op_logic_button_update(&g_stop_button, gb_is_stop_down, now_ms(),
                                 &b_has_stop_edge);
    go_press = op_logic_button_update(&g_go_button, gb_is_go_down, now_ms(),
                                      &b_has_go_edge);
    remote_control_physical_stop(op_logic_button_held(&g_stop_button));
    const uint32_t remote_stop = remote_control_stop_sequence();

    if (remote_stop != g_remote_stop_seen)
    {
        g_remote_stop_seen = remote_stop;
        op_logic_remote_stop(&g_operator, now_ms());
    }

    op_logic_step(
        &g_operator, now_ms(), go_press, op_logic_button_held(&g_stop_button),
        op_logic_button_held(&g_go_button),
        s3_integration_status(&g_operator_status) ? &g_operator_status : 0);
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
    return g_console_length;
}

/** @brief Terminate the fixture console buffer and search for the requested
 * text starting at the supplied offset.
 */
static bool said_since (size_t from, const char * p_text)
{
    g_console[g_console_length] = 0;
    return strstr(g_console + from, p_text) != NULL;
}

/** @brief Set all three simulated IR raw readings to the supplied ADC value.
 */
static void set_all (uint16_t darkness)
{
    g_raw[0] = g_raw[1] = g_raw[2] = darkness;
}

/** @brief Drive the button and ADC fixtures through white and black
 * calibration for the left, right and barcode sensors in order.
 */
static void calibrate_all (void)
{
    /* Process LEFT, RIGHT and BARCODE, each WHITE then BLACK. */
    static const ir_acquisition_sensor_id_t order[3] = {
        S3_SENSOR_LEFT, S3_SENSOR_RIGHT, S3_SENSOR_BARCODE};

    for (unsigned run_index = 0; run_index < (sizeof order / sizeof order[0]);
         run_index++)
    {
        set_all(300);
        press_go(80);
        run_ms(1200);
        set_all(300);
        g_raw[order[run_index]] = 2500;
        press_go(80);
        run_ms(1300);
    }
}

/* End of file. */

