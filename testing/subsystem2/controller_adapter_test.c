/* Exercise the actual controller's telemetry-only fault adapter. Control
 * functions below are inert test substitutes, not an on-car timing test. */
#include "host_kernel.h"
#include <assert.h>
#include <stdio.h>
#include "../../app_program/controller/vehicle_controller.c"

static UB fault_cache = BUDDY1_FAULT_WIFI;
static ER next_error;
static unsigned updates, motor_requests;

ER buddy1_update_faults(UB changed, UB active)
{
    ++updates;
    assert(changed == (BUDDY1_FAULT_STALL | BUDDY1_FAULT_ENCODER));
    assert((active & ~changed) == 0U);
    if (next_error != E_OK && next_error != E_TMOUT) return next_error;
    fault_cache = (UB)((fault_cache & ~changed) | active);
    return next_error;
}
ER buddy1_report_motion(H l, H r) { (void)l; (void)r; return E_OK; }
ER buddy1_report_robot_state(BUDDY1_ROBOT_STATE s) { (void)s; return E_OK; }
ER buddy1_report_barcode(UB c) { (void)c; return E_OK; }
ER buddy1_invalidate(UB type) { (void)type; return E_OK; }
ER buddy1_configuration_close(void) { return E_OK; }
BUDDY1_BARCODE_ACTION buddy1_barcode_action(UB c) { (void)c; return BUDDY1_ACTION_UNKNOWN; }
ER subsystem3_set_mode(s3_processing_mode_t m) { (void)m; return E_OK; }
motion_fault_t motion_get_fault(void) { return MOTION_FAULT_NONE; }
motion_state_t motion_poll(void) { return MOTION_IDLE; }
motion_result_t motion_request_manoeuvre(manoeuvre_t m) { (void)m; ++motor_requests; return MOTION_OK; }
motion_result_t motion_request_velocity(float l, float r) { (void)l; (void)r; ++motor_requests; return MOTION_OK; }
void stop(void) { ++motor_requests; }
bool motion_read_snapshot(motion_snapshot_t *p) { (void)p; return false; }
ER tk_loc_mtx(ID id, TMO t) { (void)id; (void)t; return E_OK; }
ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
ID tk_cre_mtx(const T_CMTX *p) { (void)p; return 1; }
ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
ER tk_slp_tsk(TMO t) { (void)t; return E_OK; }
ID app_start_task(FP entry, PRI pri, UW core, const char *name)
{ (void)entry; (void)pri; (void)core; (void)name; return 1; }

int main(void)
{
    report_motion_faults(MOTION_FAULT_NONE);
    assert(updates == 1 && fault_cache == BUDDY1_FAULT_WIFI);
    report_motion_faults(MOTION_FAULT_NONE); assert(updates == 1);
    report_motion_faults(MOTION_FAULT_STALL);
    assert(fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL));
    report_motion_faults(MOTION_FAULT_STALE_COMMAND);
    assert(fault_cache == BUDDY1_FAULT_WIFI);
    unsigned previous = updates;
    report_motion_faults(MOTION_FAULT_DIRECTION);
    report_motion_faults(MOTION_FAULT_TIMEOUT);
    assert(updates == previous); /* neither is falsely called stall/encoder */
    report_motion_faults(MOTION_FAULT_ENCODER);
    assert(fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_ENCODER));
    next_error = E_OBJ; report_motion_faults(MOTION_FAULT_NONE);
    assert(fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_ENCODER));
    next_error = E_OK; report_motion_faults(MOTION_FAULT_NONE);
    assert(fault_cache == BUDDY1_FAULT_WIFI);
    next_error = E_TMOUT; report_motion_faults(MOTION_FAULT_STALL);
    previous = updates; report_motion_faults(MOTION_FAULT_STALL);
    assert(updates == previous && fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL));
    assert(motor_requests == 0);
    puts("PASS: controller fault adapter preserves Wi-Fi, suppresses duplicate reports, handles queue-full/error, never commands motors");
    return 0;
}
