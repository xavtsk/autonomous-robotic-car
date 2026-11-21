/** @file map_freeze_main.c
 * @brief Map-freeze ownership with the REAL pieces wired together: Buddy 1 map
 * storage (command.c + telemetry_codec.c), the facade
 * (vehicle_controller.c), the production runtime (s3_integration.c) and
 * s3_drive/navigation.
 *
 * Only the
 * RTOS, ADC, motor and telemetry-publish boundaries are fake.
 * Contract: the map freezes at the first accepted START and stays frozen until
 * reboot; a rejected START may only undo a freeze that THIS attempt took.
 * Unsigned casts spell out the original modulo conversion.
 */
#define APP_H
#define APP_S3_MOTION 1
#define APP_DIAGNOSTICS 0
#define APP_TEST 0
#define S3_BUILTIN_CALIBRATION 0 /* Start these tests without calibration. */
/* Use the original Buddy 3 line and junction profile. The course_main.c
 * fixture covers tape between the sensors and straight travel over
 * unannounced crossings.
 */
#define S3_LINE_BETWEEN 0
#define S3_CROSS_STRAIGHT 0
#define S3_COURSE_PENDING_WAIT_US (5000000U)
#define S3_MAP_FREEZE_TEST
#include "map_freeze_main.h"
#include "../../app_program/app_types.h"
#include "../../app_program/controller/s3_integration.h"
#include "../../app_program/controller/vehicle_controller.h"
#include "../../app_program/subsystem1/buddy1.h"
#include "../../app_program/subsystem2/motion.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>

static jmp_buf  g_boundary;
static unsigned g_sleeps;
static unsigned g_locked[8];
static uint32_t g_clock_value;
static bool     gb_is_feedback_braked = true;
static uint8_t  g_fb_command_id;
ID              gh_buddy1_state_mutex = 5;
ID              gh_buddy1_commands    = 6;

static void cycle (void);

static bool submit (s3_integration_kind_t kind);

static ER try_map (char right);

/** @brief Return the fixture mutex ID without allocating a kernel object.
 */
ID tk_cre_mtx (const T_CMTX * p_mutex)
{
    (void)p_mutex;
    return 1;
}

/** @brief Require an available fixture mutex with a valid ID, mark it locked
 * and return success.
 */
ER tk_loc_mtx (ID h_mutex, TMO timeout)
{
    (void)timeout;
    assert(h_mutex > 0 && h_mutex < 8 && !g_locked[h_mutex]);
    g_locked[h_mutex] = 1;
    return E_OK;
}

/** @brief Check the simulated mutex ID and ownership before clearing its lock
 * flag.
 */
ER tk_unl_mtx (ID h_mutex)
{
    assert(h_mutex > 0 && h_mutex < 8 && g_locked[h_mutex]);
    g_locked[h_mutex] = 0;
    return E_OK;
}

/** @brief Return the fixed cyclic-handler ID without starting a timer.
 */
ID tk_cre_cyc (const T_CCYC * p_config)
{
    (void)p_config;
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

/** @brief Count sleeps and transfer control back to the fixture on the
 * second sleep.
 */
ER tk_slp_tsk (TMO timeout)
{
    (void)timeout;

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

/** @brief Accept a message-buffer send without copying the payload.
 */
ER tk_snd_mbf (ID h_message_buffer, const void * p_message, INT size,
               TMO timeout)
{
    (void)h_message_buffer;
    (void)p_message;
    (void)size;
    (void)timeout;
    return E_OK;
}

/** @brief Return a zero-length receive without modifying the destination
 * buffer.
 */
INT tk_rcv_mbf (ID h_message_buffer, void * p_message, TMO timeout)
{
    (void)h_message_buffer;
    (void)p_message;
    (void)timeout;
    return 0;
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

/** @brief Prepare timestamped valid readings for the three simulated IR
 * channels.
 */
int32_t ir_sensor_read_frame (ir_acquisition_frame_t * p_frame)
{
    *p_frame = (ir_acquisition_frame_t){.timestamp_us = g_clock_value};
    /* Partly dark line sensors show only the guide line. */
    p_frame->sensor[0] =
        (ir_acquisition_sample_t){g_clock_value, 150, 0, true, true};
    p_frame->sensor[1] = (ir_acquisition_sample_t){
        g_clock_value, (uint16_t)(400U + (g_clock_value / 10000U % 3U)), 0,
        true, true};
    p_frame->sensor[2] = (ir_acquisition_sample_t){
        g_clock_value, (uint16_t)(400U + (g_clock_value / 10000U % 3U)), 0,
        true, true};
    return E_OK;
}

/** @brief Copy the fixture's timestamp, brake state and active command ID
 * into a valid feedback sample.
 */
bool motion_get_feedback (motion_feedback_t * p_frame)
{
    *p_frame = (motion_feedback_t){.timestamp_us        = g_clock_value,
                                   .epoch               = 1,
                                   .b_is_feedback_valid = true,
                                   .b_is_braked         = gb_is_feedback_braked,
                                   .command_id          = g_fb_command_id};
    return true;
}

/** @brief Accept the velocity request and mark command ID one active.
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
motion_result_t motion_request_manoeuvre (motion_manoeuvre_t manoeuvre)
{
    (void)manoeuvre;
    return MOTION_OK;
}

/** @brief Clear the fixture's active motion command ID.
 */
void stop (void)
{
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

/** @brief Clear the output structure and report an unavailable motion
 * status.
 */
bool motion_get_status (motion_status_t * p_out)
{
    *p_out = (motion_status_t){0};
    return false;
}

/* Replace Buddy 1 telemetry at its publish boundary. */
/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_barcode (UB letter)
{
    (void)letter;
    return E_OK;
}

/** @brief Acknowledge fault updates without retaining the changed bits.
 */
ER buddy1_update_faults (UB changed, UB active)
{
    (void)changed;
    (void)active;
    return E_OK;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_robot_state (telemetry_codec_state_t state)
{
    (void)state;
    return E_OK;
}

/** @brief Acknowledge telemetry invalidation without changing a cached value.
 */
ER buddy1_invalidate (UB topic)
{
    (void)topic;
    return E_OK;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_motion (H left_mm_s, H right_mm_s)
{
    (void)left_mm_s;
    (void)right_mm_s;
    return E_OK;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_pose (UH distance_cm, H heading_deg)
{
    (void)distance_cm;
    (void)heading_deg;
    return E_OK;
}

/** @brief Accept a publication without adding it to a network queue.
 */
ER buddy1_queue_publish (UB topic, const UB * p_packet, UH length)
{
    (void)topic;
    (void)p_packet;
    (void)length;
    return E_OK;
}

/** @brief Report packet availability without writing the caller’s buffer in
 * this mapping fixture.
 */
BOOL buddy1_snapshot_packet (UB packet[BUDDY1_PAYLOAD_MAX])
{
    (void)packet;
    return TRUE;
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

/** @brief Drive START, cancellation and restart through the real map-freeze
 * code using simulated acquisition and motor boundaries.
 */
int main (void)
{
    assert(vehicle_controller_init() == E_OK);
    assert(s3_integration_init() == E_OK);
    const s3_drive_t * p_drive = s3_integration_test_drive();
    cycle();
    cycle();
    s3_integration_test_set_calibrated();
    cycle();
    assert(try_map('Y') == E_OK); /* Map Y to RIGHT before starting. */

    /* 1. First START refused by Buddy 2 (not braked): no run used the map,
     *    so the freeze this attempt took is undone. */
    gb_is_feedback_braked = false;
    assert(!submit(S3_CONTROL_START));
    assert((unsigned)S3_DRIVE_OFF == p_drive->state);
    assert(try_map('Z') == E_OK &&
           try_map('Y') == E_OK); /* The map is still configurable. */

    /* 2. Accepted START freezes the map and installs Y = RIGHT. */
    gb_is_feedback_braked = true;
    assert(submit(S3_CONTROL_START));
    assert((unsigned)S3_DRIVE_FOLLOW == p_drive->state);
    assert((unsigned)S3_NAV_RIGHT == p_drive->navigation.config.map['Y' - 'A']);
    assert(try_map('Z') == E_OBJ);

    /* 3. Duplicate START while driving: refused, map stays frozen, the run
     *    keeps its installed map. */
    assert(!submit(S3_CONTROL_START));
    assert((unsigned)S3_DRIVE_FOLLOW == p_drive->state);
    assert(try_map('Z') == E_OBJ);
    assert((unsigned)S3_NAV_RIGHT == p_drive->navigation.config.map['Y' - 'A']);

    /* 4. STOP: frozen until reboot. */
    assert(submit(S3_CONTROL_STOP));
    assert((unsigned)S3_DRIVE_OFF == p_drive->state && try_map('Z') == E_OBJ);

    /* 5. Later restart refused by Buddy 2: must NOT undo the earlier freeze. */
    gb_is_feedback_braked = false;
    assert(!submit(S3_CONTROL_START));
    assert(try_map('Z') == E_OBJ);

    /* 6. Restart accepted with the same frozen map. */
    gb_is_feedback_braked = true;
    assert(submit(S3_CONTROL_START));
    assert((unsigned)S3_DRIVE_FOLLOW == p_drive->state &&
           (unsigned)S3_NAV_RIGHT == p_drive->navigation.config.map['Y' - 'A']);
    assert(try_map('Z') == E_OBJ);

    puts("PASS: real map storage + facade + runtime: refused first START "
         "reopens, accepted START freezes, duplicate START while driving "
         "and refused restart keep the freeze, restart keeps the installed "
         "map");
    return 0;
}

/** @brief Advance the clock by 10 ms and run the sampling task to its sleep
 * boundary. Check that all mutexes were released.
 */
static void cycle (void)
{
    g_sleeps       = 0;
    g_clock_value += 10000U;

    if (setjmp(g_boundary) == 0)
    {
        s3_integration_test_sample_period();
    }

    for (unsigned lock_index = 0;
         lock_index < (sizeof g_locked / sizeof g_locked[0]); lock_index++)
    {
        assert(!g_locked[lock_index]);
    }
}

/** @brief Submit a control request, run the sampling cycle and return the
 * recorded command result after checking its sequence increment.
 */
static bool submit (s3_integration_kind_t kind)
{
    s3_integration_status_t status;
    uint32_t                seq;
    assert(s3_integration_status(&status));
    seq                                    = status.command_sequence;
    const s3_integration_request_t request = {.kind = kind};
    assert(s3_integration_submit(&request));
    cycle();
    assert(s3_integration_status(&status) &&
           status.command_sequence == seq + 1U);
    return status.b_did_command_succeed;
}

/** @brief Attempt to apply a fixture map with the supplied right-turn letter
 * and return the configuration result.
 */
static ER try_map (char right)
{
    const telemetry_codec_map_t proposed_map = {'A', 'B', (uint8_t)right, 'D'};
    return buddy1_apply_barcode_map(&proposed_map);
}

/* End of file. */

