/** @file controller_main.c
 * @brief Actual controller task + actual Buddy 3 packet codec, with RTOS, Buddy
 * 1 and motor-command boundary substitutes.
 *
 * No on-car timing is claimed.
 */
#include "controller_main.h"
#include "../../app_program/app_types.h"
#include "../../app_program/controller/vehicle_controller.h"
#include "../../app_program/subsystem1/buddy1.h"
#include "../../app_program/subsystem2/codec_motion.h"
#include "../../app_program/subsystem3/buddy3.h"
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static pthread_mutex_t        mailbox_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t        event_lock   = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t         event        = PTHREAD_COND_INITIALIZER;
static pthread_t              task;
static FP                     entry = NULL;
static unsigned               wake_count;
static unsigned               waiting;
static unsigned               gate;
static unsigned               shutdown_task;
static unsigned               velocities;
static unsigned               manoeuvres;
static unsigned               stops;
static unsigned               barcodes;
static unsigned               freezes;
static unsigned               order;
static unsigned               last_stop_order;
static unsigned               last_ir_order;
static float32_t              last_left;
static float32_t              last_right;
static motion_manoeuvre_t     last_manoeuvre;
static motion_state_t         polled_state = MOTION_IDLE;
static motion_fault_t         motor_fault  = MOTION_FAULT_NONE;
static line_navigation_mode_t last_mode    = S3_MODE_TRACK;
static UB                     fault_cache  = BUDDY1_FAULT_WIFI;
static UB                     last_barcode;
static ER                     fault_update_error;

/** @brief Verify the priority-inheritance attribute and return the configured
 * fake mutex ID.
 */
ID tk_cre_mtx (const T_CMTX * p_spec)
{
    assert(TA_INHERIT == p_spec->mtxatr);
    return 1;
}

/** @brief Require the mailbox mutex ID and indefinite wait, then acquire the
 * corresponding pthread mutex.
 */
ER tk_loc_mtx (ID id, TMO timeout)
{
    assert(1 == id && TMO_FEVR == timeout);
    assert(!pthread_mutex_lock(&mailbox_lock));
    return E_OK;
}

/** @brief Unlock the host mailbox mutex and assert that the operation
 * succeeded.
 */
ER tk_unl_mtx (ID id)
{
    assert(1 == id);
    assert(!pthread_mutex_unlock(&mailbox_lock));
    return E_OK;
}

/** @brief Increment the pending wake count under the host mutex and notify
 * waiting threads.
 */
ER tk_wup_tsk (ID id)
{
    assert(1 == id);
    pthread_mutex_lock(&event_lock);
    ++wake_count;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&event_lock);
    return E_OK;
}

/** @brief Wait on the simulated scheduler condition until a wake can be
 * consumed or the fixture asks the worker thread to exit.
 */
ER tk_slp_tsk (TMO timeout)
{
    assert(TMO_FEVR == timeout);
    pthread_mutex_lock(&event_lock);
    waiting = 1;
    pthread_cond_broadcast(&event);
    while ((!wake_count || gate) && !shutdown_task)
    {
        pthread_cond_wait(&event, &event_lock);
    }
    if (shutdown_task)
    {
        pthread_mutex_unlock(&event_lock);
        pthread_exit(NULL);
    }
    --wake_count;
    waiting = 0;
    pthread_mutex_unlock(&event_lock);
    return E_OK;
}

/** @brief Invoke the saved RTOS task entry on the test worker thread with a
 * zero start code and null context.
 */
static void * run_task (void * p_unused)
{
    (void)p_unused;
    ((void (*)(INT, void *))entry)(0, NULL);
    return NULL;
}

/** @brief Verify the controller task's name, core and priority, then start
 * its worker thread and return the fixture task ID.
 */
ID app_start_task (FP p_entry, PRI priority, UW core, const char * p_name)
{
    assert(VC_PRI_MAIN == priority && TP_PRC1 == core &&
           !strcmp(p_name, "vehicle-controller"));
    entry = p_entry;
    assert(!pthread_create(&task, NULL, run_task, NULL));
    return 1;
}

/** @brief Wait up to three seconds for the simulated controller to reach its
 * idle condition.
 */
static void idle (void)
{
    struct timespec deadline;
    assert(clock_gettime(CLOCK_REALTIME, &deadline) == 0);
    deadline.tv_sec += 3;
    pthread_mutex_lock(&event_lock);
    while (!waiting || wake_count || gate)
    {
        assert(pthread_cond_timedwait(&event, &event_lock, &deadline) == 0);
    }
    pthread_mutex_unlock(&event_lock);
}

/** @brief Change the simulated scheduling gate under the event lock and wake
 * its waiters.
 */
static void pause_task (unsigned value)
{
    pthread_mutex_lock(&event_lock);
    gate = value;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&event_lock);
}

/** @brief Validate the changed and active masks, apply the configured error
 * behavior and record fault publication order.
 */
ER buddy1_update_faults (UB changed, UB active)
{
    assert(changed && !(changed & ~BUDDY1_FAULT_MASK) && !(active & ~changed));
    if (fault_update_error != E_OK && fault_update_error != E_TMOUT)
    {
        return fault_update_error;
    }
    fault_cache = (UB)((fault_cache & (UB)~changed) | active);
    if (BUDDY1_FAULT_IR == changed && active)
    {
        last_ir_order = ++order;
    }
    return fault_update_error;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_motion (H left, H right)
{
    (void)left;
    (void)right;
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

/** @brief Count the barcode event and store its letter.
 */
ER buddy1_report_barcode (UB letter)
{
    ++barcodes;
    last_barcode = letter;
    return E_OK;
}

/** @brief Acknowledge telemetry invalidation without changing a cached value.
 */
ER buddy1_invalidate (UB type)
{
    (void)type;
    return E_OK;
}

/** @brief Count a simulated configuration freeze and acknowledge it.
 */
ER buddy1_configuration_close (void)
{
    ++freezes;
    return E_OK;
}

/** @brief Map A through D to the four fixture actions and return unknown for
 * any other letter.
 */
telemetry_codec_action_t buddy1_barcode_action (UB letter)
{
    if ('A' == letter)
    {
        return BUDDY1_ACTION_FORWARD;
    }
    if ('B' == letter)
    {
        return BUDDY1_ACTION_LEFT;
    }
    if ('C' == letter)
    {
        return BUDDY1_ACTION_RIGHT;
    }
    if ('D' == letter)
    {
        return BUDDY1_ACTION_UTURN;
    }
    return BUDDY1_ACTION_UNKNOWN;
}

/** @brief Remember the requested navigation mode and acknowledge the
 * request.
 */
ER subsystem3_set_mode (line_navigation_mode_t mode)
{
    last_mode = mode;
    return E_OK;
}

/** @brief Return the motor fault selected by the current scenario.
 */
motion_fault_t motion_get_fault (void)
{
    return motor_fault;
}

/** @brief Return the motion state selected by the test without consuming a
 * report.
 */
motion_state_t motion_poll (void)
{
    return polled_state;
}

/** @brief Count a requested manoeuvre and remember its value.
 */
motion_result_t motion_request_manoeuvre (motion_manoeuvre_t value)
{
    ++manoeuvres;
    last_manoeuvre = value;
    return MOTION_OK;
}

/** @brief Count a velocity update and retain the requested left and right
 * speeds.
 */
motion_result_t motion_request_velocity (float32_t left, float32_t right)
{
    ++velocities;
    last_left  = left;
    last_right = right;
    return MOTION_OK;
}

/** @brief Record a stop and its sequence number relative to the other
 * fixture events.
 */
void stop (void)
{
    ++stops;
    last_stop_order = ++order;
}

/** @brief Pack the injected motion state and fault with fixture wheel speeds
 * of 124 and 98 millimetres per second.
 */
bool snapshot_motion_read (codec_motion_snapshot_t * p_snapshot)
{
    return codec_motion_pack(p_snapshot, polled_state, motor_fault, false, true,
                             124, 98);
}

/** @brief Construct a valid navigation observation with a 10 ms sample gap
 * and predetermined steering values.
 */
static line_navigation_output_t observation (void)
{
    line_navigation_output_t value = {0};
    value.timestamp_us             = 10000U;
    value.sample_gap_us            = 10000U;
    for (unsigned sensor_index = 0; sensor_index < S3_SENSOR_COUNT;
         ++sensor_index)
    {
        value.sensor[sensor_index].valid    = true;
        value.sensor[sensor_index].darkness = 500U;
        value.sensor[sensor_index].surface  = IR_SURFACE_UNKNOWN;
    }
    value.line.valid         = true;
    value.line.present       = true;
    value.line.error         = -23;
    value.steering.valid     = true;
    value.steering.left_mps  = 0.1234567f;
    value.steering.right_mps = 0.09876543f;
    return value;
}

/** @brief Submit a navigation observation and wait for the controller to
 * return to idle.
 */
static void send (line_navigation_output_t * p_output)
{
    assert(vehicle_controller_submit_nav(p_output) == E_OK);
    idle();
}

/** @brief Copy a binary32 value into an unsigned word for exact comparison.
 */
static uint32_t bits (float32_t value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    return raw;
}

/** @brief Run the controller task with host scheduling and sensor messages.
 * Check drive commands, completion and stop or fault ownership.
 */
int main (void)
{
    assert(vehicle_controller_init() == E_OK);
    assert(vehicle_controller_start() == E_OK);
    idle();
    assert(vehicle_controller_state() == VC_WAITING && 1 == stops);
    line_navigation_output_t value = observation();
    send(&value);
    assert(!velocities &&
           !manoeuvres); /* Keep motion stopped before local START. */
    assert(vehicle_controller_begin() == E_OK);
    idle();
    assert(1 == freezes && vehicle_controller_state() == VC_LINE_FOLLOW);
    send(&value);
    assert(1 == velocities &&
           bits(last_left) == bits(value.steering.left_mps) &&
           bits(last_right) == bits(value.steering.right_mps));

    value.events                = S3_EVENT_BARCODE;
    value.barcode.letter        = 'B';
    value.barcode.b_is_reversed = true;
    value.barcode.timestamp_us  = 20000;
    value.barcode_status        = S3_BARCODE_DECODED;
    send(&value);
    assert(1 == barcodes && 'B' == last_barcode && !manoeuvres);
    value = observation();
    send(&value);
    assert(!manoeuvres);
    value.events         = S3_EVENT_JUNCTION;
    value.steering.valid = false;
    send(&value);
    assert(1 == manoeuvres && MANOEUVRE_LEFT == last_manoeuvre);
    assert(vehicle_controller_state() == VC_TURNING &&
           S3_MODE_SUPPRESSED == last_mode);
    polled_state = MOTION_COMPLETE;
    value        = observation();
    send(&value);
    assert(vehicle_controller_state() == VC_LINE_FOLLOW &&
           S3_MODE_TRACK == last_mode);
    polled_state = MOTION_IDLE;

    /* Newer ordinary samples must not erase a pending barcode. */
    pause_task(1);
    value                = observation();
    value.events         = S3_EVENT_BARCODE;
    value.barcode.letter = 'C';
    value.barcode_status = S3_BARCODE_DECODED;
    assert(vehicle_controller_submit_nav(&value) == E_OK);
    value = observation();
    assert(vehicle_controller_submit_nav(&value) == E_OK);
    pause_task(0);
    idle();
    assert(2 == barcodes && 'C' == last_barcode && 1 == manoeuvres);
    value.events         = S3_EVENT_JUNCTION;
    value.steering.valid = false;
    send(&value);
    assert(2 == manoeuvres && MANOEUVRE_RIGHT == last_manoeuvre);
    polled_state = MOTION_COMPLETE;
    value        = observation();
    send(&value);
    polled_state = MOTION_IDLE;

    /* Barcode and junction edge flags both survive one mailbox batch. */
    pause_task(1);
    value                = observation();
    value.events         = S3_EVENT_BARCODE;
    value.barcode.letter = 'D';
    value.barcode_status = S3_BARCODE_DECODED;
    assert(vehicle_controller_submit_nav(&value) == E_OK);
    value                = observation();
    value.events         = S3_EVENT_JUNCTION;
    value.steering.valid = false;
    assert(vehicle_controller_submit_nav(&value) == E_OK);
    pause_task(0);
    idle();
    assert(3 == barcodes && 3 == manoeuvres &&
           MANOEUVRE_U_TURN == last_manoeuvre);
    polled_state = MOTION_COMPLETE;
    value        = observation();
    send(&value);
    polled_state = MOTION_IDLE;

    /* Rejection occurs before narrowing/publication and does not mutate state.
     */
    value                    = observation();
    value.sensor[0].darkness = 1001;
    assert(vehicle_controller_submit_nav(&value) == E_PAR);
    assert(vehicle_controller_state() == VC_LINE_FOLLOW);

    motor_fault = MOTION_FAULT_STALL;
    fault_cache = BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL;
    fault_update_error =
        E_TMOUT; /* Update the protected mirror even with a full outbox. */
    value                = observation();
    value.events         = S3_EVENT_SENSOR_FAULT;
    value.fault_mask     = 2U;
    value.steering.valid = false;
    send(&value);
    assert(vehicle_controller_state() == VC_FAULT &&
           last_stop_order < last_ir_order);
    assert(fault_cache ==
           (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL | BUDDY1_FAULT_IR));
    unsigned previous_velocities = velocities;
    fault_update_error           = E_OK;
    value                        = observation();
    send(&value);
    assert(fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL));
    assert(vehicle_controller_state() == VC_FAULT &&
           velocities == previous_velocities);
    pause_task(1);
    assert(vehicle_controller_request_stop() == E_OK);
    value = observation();
    assert(vehicle_controller_submit_nav(&value) == E_OK);
    pause_task(0);
    idle();
    assert(vehicle_controller_state() == VC_WAITING);
    assert(
        velocities ==
        previous_velocities); /* A newer sample preserves the explicit STOP. */

    pthread_mutex_lock(&event_lock);
    shutdown_task = 1;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&event_lock);
    assert(!pthread_join(task, NULL));
    puts("PASS: actual controller task preserves exact steering, holds barcode "
         "until junction, sticky mailbox events, turn completion, explicit "
         "start/stop and fault ownership");
    return 0;
}

/* End of file. */

