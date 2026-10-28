/* Team facade/telemetry adapter. The upstream s3 integration task is the ONE
 * navigation, motion-request and report-consumer owner. No second task here. */
#include "vehicle_controller.h"
#include "s3_integration.h"
#include "terrain_integration.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/motion_pack.h"
#if APP_S5
#include "../subsystem5/s5_api.h"
#include "../subsystem5/obstacle_avoidance.h"
#endif

static unsigned telemetry_divider;
static BUDDY1_ROBOT_STATE last_state;
static BUDDY1_BARCODE_ACTION last_action;
static unsigned last_faults;
static bool faults_seen;
/* True only between a prepare_start() that newly froze the Buddy 1 map and
 * the end of that same START attempt. Only the sampling owner touches it. */
static bool start_claimed_map;

static BUDDY1_BARCODE_ACTION action_for(s3_nav_action_t action)
{
    switch (action) {
    case S3_NAV_STRAIGHT: return BUDDY1_ACTION_FORWARD;
    case S3_NAV_LEFT: return BUDDY1_ACTION_LEFT;
    case S3_NAV_RIGHT: return BUDDY1_ACTION_RIGHT;
    case S3_NAV_UTURN: return BUDDY1_ACTION_UTURN;
    default: return BUDDY1_ACTION_UNKNOWN;
    }
}
static VEHICLE_CONTROLLER_STATE public_state(s3_drive_state_t state)
{
    switch (state) {
    case S3_DRIVE_FOLLOW: return VC_LINE_FOLLOW;
    case S3_DRIVE_BRAKING:
    case S3_DRIVE_MANOEUVRE:
    case S3_DRIVE_REACQUIRE: return VC_TURNING;
    case S3_DRIVE_HALTED: return VC_STOPPED;
    case S3_DRIVE_FAULT: return VC_FAULT;
    default: return VC_WAITING;
    }
}
static BUDDY1_ROBOT_STATE wire_state(s3_drive_state_t state)
{
    if (state == S3_DRIVE_FAULT) return BUDDY1_STATE_ERROR;
    if (state == S3_DRIVE_OFF || state == S3_DRIVE_HALTED) return BUDDY1_STATE_STOPPED;
    if (state == S3_DRIVE_MANOEUVRE) {
        if (last_action == BUDDY1_ACTION_LEFT) return BUDDY1_STATE_TURNING_LEFT;
        if (last_action == BUDDY1_ACTION_RIGHT) return BUDDY1_STATE_TURNING_RIGHT;
        if (last_action == BUDDY1_ACTION_UTURN) return BUDDY1_STATE_UTURN;
    }
    return BUDDY1_STATE_LINE_FOLLOWING;
}
ER vehicle_controller_init(void)
{
    terrain_control_init();
    telemetry_divider = 0U; last_state = BUDDY1_STATE_UNKNOWN;
    last_action = BUDDY1_ACTION_UNKNOWN; last_faults = 0U; faults_seen = false;
    start_claimed_map = false;
    return E_OK;
}
ER vehicle_controller_start(void) { return E_OK; }

ER vehicle_controller_begin(void)
{
    const s3_control_request_t request = {.kind = S3_CONTROL_START};
    return s3_integration_submit(&request) ? E_OK : E_OBJ;
}
ER vehicle_controller_request_stop(void)
{
    const s3_control_request_t request = {.kind = S3_CONTROL_STOP};
    return s3_integration_submit(&request) ? E_OK : E_OBJ;
}
VEHICLE_CONTROLLER_STATE vehicle_controller_state(void)
{
    /* Only the normal controller/UI task may use this convenience view.
     * Keep the temporary copy task-local, not a second shared snapshot. */
    s3_integration_status_t copy;
    return s3_integration_status(&copy) ? public_state(copy.state) : VC_WAITING;
}

/* Compatibility boundary: new sensor observations are processed in the SAME
 * owner as the upstream drive FSM. Never accept a second external producer. */
ER vehicle_controller_submit_navigation(const s3_navigation_output_t *output)
{ (void)output; return E_NOSPT; }

/* Called by the sampling/controller owner immediately before an explicit START.
 * Freeze and copy the Buddy 1 map atomically, then update its private nav context. */
bool vehicle_controller_prepare_start(s3_navigation_t *navigation)
{
    BUDDY1_BARCODE_MAP map;
    BOOL newly = FALSE;
    s3_nav_action_t expanded[26] = {0};
    start_claimed_map = false;
    if (!navigation || buddy1_configuration_claim(&map, &newly) != E_OK)
        return false;
    expanded[map.forward-'A'] = S3_NAV_STRAIGHT;
    expanded[map.left-'A'] = S3_NAV_LEFT;
    expanded[map.right-'A'] = S3_NAV_RIGHT;
    expanded[map.uturn-'A'] = S3_NAV_UTURN;
    if (!s3_navigation_set_mapping(navigation, expanded)) {
        /* Nothing installed: undo only a freeze this attempt took. */
        if (newly) (void)buddy1_configuration_reopen();
        return false;
    }
    start_claimed_map = newly != FALSE;
    return true;
}
void vehicle_controller_cancel_start(void)
{
    if (start_claimed_map) (void)buddy1_configuration_reopen();
    start_claimed_map = false;
}
void vehicle_controller_commit_start(void)
{
    start_claimed_map = false;
}
bool vehicle_controller_apply_mapping(const s3_nav_action_t map[26])
{
    BUDDY1_BARCODE_MAP reduced = {0};
    if (!map) return false;
    for (unsigned i=0U;i<26U;i++) {
        uint8_t *slot;
        switch (map[i]) {
        case S3_NAV_NONE: continue;
        case S3_NAV_STRAIGHT: slot=&reduced.forward; break;
        case S3_NAV_LEFT: slot=&reduced.left; break;
        case S3_NAV_RIGHT: slot=&reduced.right; break;
        case S3_NAV_UTURN: slot=&reduced.uturn; break;
        default: return false;
        }
        if (*slot) return false; /* Team protocol permits exactly four letters. */
        *slot=(uint8_t)('A'+i);
    }
    return buddy1_apply_barcode_map(&reduced) == E_OK;
}

/* Called only by the upstream sampling/controller owner, never under its
 * snapshot mutex. This mirror cannot command motors or consume reports. */
void vehicle_controller_observe(const s3_integration_status_t *status)
{
    motion_status_t motion;
    unsigned active=last_faults & (BUDDY1_FAULT_STALL|BUDDY1_FAULT_ENCODER);
    unsigned changed=BUDDY1_FAULT_IR;
    BUDDY1_ROBOT_STATE state;
    if (!status || mtx_buddy1_state <= 0) return;
    terrain_control_report();
    if (status->navigation.events & S3_EVENT_BARCODE) {
        last_action=action_for(status->navigation.action);
        (void)buddy1_report_barcode((UB)status->navigation.barcode.letter);
    }
    if (motion_get_status(&motion)) {
        active=0U;
        changed|=BUDDY1_FAULT_STALL|BUDDY1_FAULT_ENCODER;
        if (motion.fault_bits & MOTION_FAULT_BIT(MOTION_FAULT_STALL)) active|=BUDDY1_FAULT_STALL;
        if (motion.fault_bits & MOTION_FAULT_BIT(MOTION_FAULT_ENCODER)) active|=BUDDY1_FAULT_ENCODER;
    }
    if (status->navigation.fault_mask ||
        (status->navigation.latched_faults & S3_FAULT_SENSOR)) active|=BUDDY1_FAULT_IR;
    if (!faults_seen || active!=last_faults) {
        ER error=buddy1_update_faults((UB)changed,(UB)(active & changed));
        if (error==E_OK || error==E_TMOUT) { last_faults=active; faults_seen=true; }
    }
    state=wire_state(status->state);
#if APP_S5
    {
        s5_observation_t obstacle;
        static uint16_t reported_scan;
        static unsigned reported_route;
        static int reported_echo=-1;
        unsigned control=subsystem5_controller_status();
        unsigned obstacle_state=control&7U,route=control>>6;
        if (status->state==S3_DRIVE_MANOEUVRE && obstacle_state>=S5_WAIT_STOP && obstacle_state<=S5_FINISH)
            state=BUDDY1_STATE_AVOIDING_OBSTACLE;
        if (subsystem5_snapshot(&obstacle)) {
            if (reported_echo!=(int)obstacle.no_echo) {
                ER er=buddy1_update_faults(BUDDY1_FAULT_NO_ECHO,obstacle.no_echo?BUDDY1_FAULT_NO_ECHO:0U);
                if(er==E_OK || er==E_TMOUT) { reported_echo=(int)obstacle.no_echo; }
            }
            if (obstacle.scan_done && obstacle.profile.valid &&
                (reported_scan!=obstacle.scan_id || reported_route!=route)) {
                ER er=buddy1_report_obstacle(obstacle.profile.closest_mm,obstacle.profile.width_mm,
                    (BUDDY1_OBSTACLE_ROUTE)route);
                if(er==E_OK || er==E_TMOUT) { reported_scan=obstacle.scan_id;reported_route=route; }
            } else if (!obstacle.profile.valid) { (void)buddy1_invalidate(BUDDY1_TOPIC_OBSTACLE); }
        }
    }
#endif
    if (state!=last_state) {
        ER error=buddy1_report_robot_state(state);
        if (error==E_OK || error==E_TMOUT) last_state=state;
    }
    if (++telemetry_divider>=20U) {
        float left, right;
        telemetry_divider=0U;
        if (!motion_get_status(&motion) || !motion.b_valid || motion.b_busy) {
            (void)buddy1_invalidate(BUDDY1_TOPIC_MOTION); return;
        }
        left=motion.speed_left_mps*1000.0f; right=motion.speed_right_mps*1000.0f;
        /* Keep protocol v2 unchanged. New upstream calibrated top speed exceeds
         * +/-510 mm/s: mark unavailable, NEVER clip or silently widen the wire. */
        if (left!=left || right!=right || left < -BUDDY1_SPEED_MAX_MMS ||
            left > BUDDY1_SPEED_MAX_MMS || right < -BUDDY1_SPEED_MAX_MMS ||
            right > BUDDY1_SPEED_MAX_MMS) {
            (void)buddy1_invalidate(BUDDY1_TOPIC_MOTION);
        } else {
            (void)buddy1_report_motion((H)left,(H)right);
        }
    }
    if (status->state==S3_DRIVE_OFF) last_action=BUDDY1_ACTION_UNKNOWN;
}
