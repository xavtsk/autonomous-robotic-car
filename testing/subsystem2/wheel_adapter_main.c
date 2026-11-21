/** @file wheel_adapter_main.c
 * @brief Record the legacy controller fault-adapter fixture.
 *
 * The assertions reference report_motion_faults, the former private
 * controller entry. Current supported builds use the controller integration
 * fixture. The functions below are inert motor and kernel substitutes.
 */
#include "wheel_adapter_main.h"
#include "../../app_program/app_types.h"
#include <assert.h>
#include <stdio.h>
#include "../../app_program/controller/vehicle_controller.c"

static UB       fault_cache = BUDDY1_FAULT_WIFI;
static ER       next_error;
static unsigned updates;
static unsigned motor_requests;

/** @brief Require a motor-owned fault mask, count the update and apply the
 * fixture's injected error policy.
 */
ER buddy1_update_faults (UB changed, UB active)
{
    ++updates;
    assert(changed == (BUDDY1_FAULT_STALL | BUDDY1_FAULT_ENCODER));
    assert((active & ~changed) == 0U);
    if (next_error != E_OK && next_error != E_TMOUT)
    {
        return next_error;
    }
    fault_cache = (UB)((fault_cache & ~changed) | active);
    return next_error;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_motion (H l, H r)
{
    (void)l;
    (void)r;
    return E_OK;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_robot_state (telemetry_codec_state_t s)
{
    (void)s;
    return E_OK;
}

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_barcode (UB c)
{
    (void)c;
    return E_OK;
}

/** @brief Acknowledge telemetry invalidation without changing a cached value.
 */
ER buddy1_invalidate (UB type)
{
    (void)type;
    return E_OK;
}

/** @brief Acknowledge the configuration close without storing a map.
 */
ER buddy1_configuration_close (void)
{
    return E_OK;
}

/** @brief Report the unmapped action for every barcode in this fixture.
 */
telemetry_codec_action_t buddy1_barcode_action (UB c)
{
    (void)c;
    return BUDDY1_ACTION_UNKNOWN;
}

/** @brief Accept the processing mode in the telemetry-only fixture.
 */
ER subsystem3_set_mode (line_navigation_mode_t m)
{
    (void)m;
    return E_OK;
}

/** @brief Report a fault-free motor controller for every observation.
 */
motion_fault_t motion_get_fault (void)
{
    return MOTION_FAULT_NONE;
}

/** @brief Report an idle motion controller without advancing the simulation.
 */
motion_state_t motion_poll (void)
{
    return MOTION_IDLE;
}

/** @brief Record a simulated manoeuvre request and acknowledge it.
 */
motion_result_t motion_request_manoeuvre (motion_manoeuvre_t m)
{
    (void)m;
    ++motor_requests;
    return MOTION_OK;
}

/** @brief Record a simulated wheel-velocity request and acknowledge it.
 */
motion_result_t motion_request_velocity (float32_t l, float32_t r)
{
    (void)l;
    (void)r;
    ++motor_requests;
    return MOTION_OK;
}

/** @brief Count a stop as a motor request in the adapter fixture.
 */
void stop (void)
{
    ++motor_requests;
}

/** @brief Report that motion feedback is unavailable and preserve the output
 * buffer.
 */
bool snapshot_motion_read (codec_motion_snapshot_t * p)
{
    (void)p;
    return false;
}

/** @brief Acknowledge the mutex operation without changing fixture lock
 * state.
 */
ER tk_loc_mtx (ID id, TMO t)
{
    (void)id;
    (void)t;
    return E_OK;
}

/** @brief Acknowledge the mutex operation without changing fixture lock
 * state.
 */
ER tk_unl_mtx (ID id)
{
    (void)id;
    return E_OK;
}

/** @brief Return the fixture mutex ID without allocating a kernel object.
 */
ID tk_cre_mtx (const T_CMTX * p)
{
    (void)p;
    return 1;
}

/** @brief Acknowledge a wake request without scheduling a task.
 */
ER tk_wup_tsk (ID id)
{
    (void)id;
    return E_OK;
}

/** @brief Complete the sleep request immediately without suspending a task.
 */
ER tk_slp_tsk (TMO t)
{
    (void)t;
    return E_OK;
}

/** @brief Return the fixture task ID without scheduling an entry point.
 */
ID app_start_task (FP entry, PRI pri, UW core, const char * name)
{
    (void)entry;
    (void)pri;
    (void)core;
    (void)name;
    return 1;
}

/** @brief Exercise the legacy wheel fault-reporting interface and assert
 * that telemetry updates never request motor motion.
 */
int main (void)
{
    report_motion_faults(MOTION_FAULT_NONE);
    assert(1 == updates && BUDDY1_FAULT_WIFI == fault_cache);
    report_motion_faults(MOTION_FAULT_NONE);
    assert(1 == updates);
    report_motion_faults(MOTION_FAULT_STALL);
    assert(fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL));
    report_motion_faults(MOTION_FAULT_STALE_COMMAND);
    assert(BUDDY1_FAULT_WIFI == fault_cache);
    unsigned previous = updates;
    report_motion_faults(MOTION_FAULT_DIRECTION);
    report_motion_faults(MOTION_FAULT_TIMEOUT);
    assert(updates ==
           previous); /* Report neither a stall nor an encoder fault. */
    report_motion_faults(MOTION_FAULT_ENCODER);
    assert(fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_ENCODER));
    next_error = E_OBJ;
    report_motion_faults(MOTION_FAULT_NONE);
    assert(fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_ENCODER));
    next_error = E_OK;
    report_motion_faults(MOTION_FAULT_NONE);
    assert(BUDDY1_FAULT_WIFI == fault_cache);
    next_error = E_TMOUT;
    report_motion_faults(MOTION_FAULT_STALL);
    previous = updates;
    report_motion_faults(MOTION_FAULT_STALL);
    assert(updates == previous &&
           fault_cache == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL));
    assert(0 == motor_requests);
    puts("PASS: controller fault adapter preserves Wi-Fi, suppresses duplicate "
         "reports, handles queue-full/error, never commands motors");
    return 0;
}

/* End of file. */

