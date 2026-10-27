/* Actual controller task + actual Buddy 3 packet codec, with RTOS, Buddy 1
 * and motor-command boundary substitutes. No on-car timing is claimed. */
#include "controller_test_kernel.h"
#include "../../app_program/controller/vehicle_controller.h"
#include "../../app_program/subsystem1/subsystem1.h"
#include "../../app_program/subsystem2/motion_codec.h"
#include "../../app_program/subsystem3/subsystem3.h"
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static pthread_mutex_t mailbox_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t event_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t event = PTHREAD_COND_INITIALIZER;
static pthread_t task;
static FP entry;
static unsigned wake_count, waiting, gate, shutdown_task;
static unsigned velocities, manoeuvres, stops, barcodes, freezes, order;
static unsigned last_stop_order, last_ir_order;
static float last_left, last_right;
static manoeuvre_t last_manoeuvre;
static motion_state_t polled_state = MOTION_IDLE;
static motion_fault_t motor_fault = MOTION_FAULT_NONE;
static s3_processing_mode_t last_mode = S3_MODE_TRACK;
static UB fault_cache = BUDDY1_FAULT_WIFI;
static UB last_barcode;
static ER fault_update_error;

ID tk_cre_mtx(const T_CMTX *value) { assert(value->mtxatr == TA_INHERIT); return 1; }
ER tk_loc_mtx(ID id, TMO timeout)
{ assert(id == 1 && timeout == TMO_FEVR); assert(!pthread_mutex_lock(&mailbox_lock)); return E_OK; }
ER tk_unl_mtx(ID id)
{ assert(id == 1); assert(!pthread_mutex_unlock(&mailbox_lock)); return E_OK; }
ER tk_wup_tsk(ID id)
{
    assert(id == 1); pthread_mutex_lock(&event_lock);
    ++wake_count; pthread_cond_broadcast(&event); pthread_mutex_unlock(&event_lock); return E_OK;
}
ER tk_slp_tsk(TMO timeout)
{
    assert(timeout == TMO_FEVR); pthread_mutex_lock(&event_lock);
    waiting = 1; pthread_cond_broadcast(&event);
    while ((!wake_count || gate) && !shutdown_task) pthread_cond_wait(&event, &event_lock);
    if (shutdown_task) { pthread_mutex_unlock(&event_lock); pthread_exit(NULL); }
    --wake_count; waiting = 0; pthread_mutex_unlock(&event_lock); return E_OK;
}
static void *run_task(void *unused)
{ (void)unused; ((void (*)(INT, void *))entry)(0, NULL); return NULL; }
ID app_start_task(FP function, PRI priority, UW core, const char *name)
{
    assert(priority == VC_PRI_MAIN && core == TP_PRC1 && !strcmp(name, "vehicle-controller"));
    entry = function; assert(!pthread_create(&task, NULL, run_task, NULL)); return 1;
}
static void idle(void)
{
    struct timespec deadline;
    assert(clock_gettime(CLOCK_REALTIME, &deadline) == 0); deadline.tv_sec += 3;
    pthread_mutex_lock(&event_lock);
    while (!waiting || wake_count || gate)
        assert(pthread_cond_timedwait(&event, &event_lock, &deadline) == 0);
    pthread_mutex_unlock(&event_lock);
}
static void pause_task(unsigned value)
{
    pthread_mutex_lock(&event_lock); gate = value;
    pthread_cond_broadcast(&event); pthread_mutex_unlock(&event_lock);
}

ER buddy1_update_faults(UB changed, UB active)
{
    assert(changed && !(changed & ~BUDDY1_FAULT_MASK) && !(active & ~changed));
    if (fault_update_error != E_OK && fault_update_error != E_TMOUT) return fault_update_error;
    fault_cache = (UB)((fault_cache & (UB)~changed) | active);
    if (changed == BUDDY1_FAULT_IR && active) last_ir_order = ++order;
    return fault_update_error;
}
ER buddy1_report_motion(H left, H right) { (void)left; (void)right; return E_OK; }
ER buddy1_report_robot_state(BUDDY1_ROBOT_STATE state) { (void)state; return E_OK; }
ER buddy1_report_barcode(UB letter) { ++barcodes; last_barcode = letter; return E_OK; }
ER buddy1_invalidate(UB type) { (void)type; return E_OK; }
ER buddy1_configuration_close(void) { ++freezes; return E_OK; }
BUDDY1_BARCODE_ACTION buddy1_barcode_action(UB letter)
{
    if (letter == 'A') return BUDDY1_ACTION_FORWARD;
    if (letter == 'B') return BUDDY1_ACTION_LEFT;
    if (letter == 'C') return BUDDY1_ACTION_RIGHT;
    if (letter == 'D') return BUDDY1_ACTION_UTURN;
    return BUDDY1_ACTION_UNKNOWN;
}
ER subsystem3_set_mode(s3_processing_mode_t mode) { last_mode = mode; return E_OK; }
motion_fault_t motion_get_fault(void) { return motor_fault; }
motion_state_t motion_poll(void) { return polled_state; }
motion_result_t motion_request_manoeuvre(manoeuvre_t value)
{ ++manoeuvres; last_manoeuvre = value; return MOTION_OK; }
motion_result_t motion_request_velocity(float left, float right)
{ ++velocities; last_left = left; last_right = right; return MOTION_OK; }
void stop(void) { ++stops; last_stop_order = ++order; }
bool motion_read_snapshot(motion_snapshot_t *value)
{ return motion_snapshot_pack(value, polled_state, motor_fault, false, true, 124, 98); }

static s3_navigation_output_t observation(void)
{
    s3_navigation_output_t value = {0};
    value.timestamp_us = 10000U; value.sample_gap_us = 10000U;
    for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i) {
        value.sensor[i].valid = true; value.sensor[i].darkness = 500U;
        value.sensor[i].surface = IR_SURFACE_UNKNOWN;
    }
    value.line.valid = true; value.line.present = true; value.line.error = -23;
    value.steering.valid = true;
    value.steering.left_mps = 0.1234567f; value.steering.right_mps = 0.09876543f;
    return value;
}
static void send(s3_navigation_output_t *value)
{ assert(vehicle_controller_submit_navigation(value) == E_OK); idle(); }
static uint32_t bits(float value) { uint32_t raw; memcpy(&raw, &value, sizeof(raw)); return raw; }

int main(void)
{
    assert(vehicle_controller_init() == E_OK);
    assert(vehicle_controller_start() == E_OK); idle();
    assert(vehicle_controller_state() == VC_WAITING && stops == 1);
    s3_navigation_output_t value = observation(); send(&value);
    assert(!velocities && !manoeuvres); /* no local begin, no motion */
    assert(vehicle_controller_begin() == E_OK); idle();
    assert(freezes == 1 && vehicle_controller_state() == VC_LINE_FOLLOW);
    send(&value);
    assert(velocities == 1 && bits(last_left) == bits(value.steering.left_mps)
                           && bits(last_right) == bits(value.steering.right_mps));

    value.events = S3_EVENT_BARCODE; value.barcode.letter = 'B';
    value.barcode.reversed = true; value.barcode.timestamp_us = 20000;
    value.barcode_status = S3_BARCODE_DECODED; send(&value);
    assert(barcodes == 1 && last_barcode == 'B' && !manoeuvres);
    value = observation(); send(&value); assert(!manoeuvres);
    value.events = S3_EVENT_JUNCTION; value.steering.valid = false; send(&value);
    assert(manoeuvres == 1 && last_manoeuvre == MANOEUVRE_LEFT);
    assert(vehicle_controller_state() == VC_TURNING && last_mode == S3_MODE_SUPPRESSED);
    polled_state = MOTION_COMPLETE; value = observation(); send(&value);
    assert(vehicle_controller_state() == VC_LINE_FOLLOW && last_mode == S3_MODE_TRACK);
    polled_state = MOTION_IDLE;

    /* Newer ordinary samples must not erase a pending barcode. */
    pause_task(1);
    value = observation(); value.events = S3_EVENT_BARCODE; value.barcode.letter = 'C';
    value.barcode_status = S3_BARCODE_DECODED;
    assert(vehicle_controller_submit_navigation(&value) == E_OK);
    value = observation();
    assert(vehicle_controller_submit_navigation(&value) == E_OK);
    pause_task(0); idle();
    assert(barcodes == 2 && last_barcode == 'C' && manoeuvres == 1);
    value.events = S3_EVENT_JUNCTION; value.steering.valid = false; send(&value);
    assert(manoeuvres == 2 && last_manoeuvre == MANOEUVRE_RIGHT);
    polled_state = MOTION_COMPLETE; value = observation(); send(&value); polled_state = MOTION_IDLE;

    /* Barcode and junction edge flags both survive one mailbox batch. */
    pause_task(1);
    value = observation(); value.events = S3_EVENT_BARCODE; value.barcode.letter = 'D';
    value.barcode_status = S3_BARCODE_DECODED;
    assert(vehicle_controller_submit_navigation(&value) == E_OK);
    value = observation(); value.events = S3_EVENT_JUNCTION; value.steering.valid = false;
    assert(vehicle_controller_submit_navigation(&value) == E_OK);
    pause_task(0); idle();
    assert(barcodes == 3 && manoeuvres == 3 && last_manoeuvre == MANOEUVRE_U_TURN);
    polled_state = MOTION_COMPLETE; value = observation(); send(&value); polled_state = MOTION_IDLE;

    /* Rejection occurs before narrowing/publication and does not mutate state. */
    value = observation(); value.sensor[0].darkness = 1001;
    assert(vehicle_controller_submit_navigation(&value) == E_PAR);
    assert(vehicle_controller_state() == VC_LINE_FOLLOW);

    motor_fault = MOTION_FAULT_STALL;
    fault_cache = BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL;
    fault_update_error = E_TMOUT; /* full outbox still updates the protected mirror */
    value = observation(); value.events = S3_EVENT_SENSOR_FAULT;
    value.fault_mask = 2U; value.steering.valid = false; send(&value);
    assert(vehicle_controller_state() == VC_FAULT && last_stop_order < last_ir_order);
    assert(fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL | BUDDY1_FAULT_IR));
    unsigned previous_velocities = velocities;
    fault_update_error = E_OK;
    value = observation(); send(&value);
    assert(fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL));
    assert(vehicle_controller_state() == VC_FAULT && velocities == previous_velocities);
    pause_task(1);
    assert(vehicle_controller_request_stop() == E_OK);
    value = observation();
    assert(vehicle_controller_submit_navigation(&value) == E_OK);
    pause_task(0); idle();
    assert(vehicle_controller_state() == VC_WAITING);
    assert(velocities == previous_velocities); /* newer sample cannot erase explicit stop */

    pthread_mutex_lock(&event_lock); shutdown_task = 1;
    pthread_cond_broadcast(&event); pthread_mutex_unlock(&event_lock);
    assert(!pthread_join(task, NULL));
    puts("PASS: actual controller task preserves exact steering, holds barcode until junction, sticky mailbox events, turn completion, explicit start/stop and fault ownership");
    return 0;
}
