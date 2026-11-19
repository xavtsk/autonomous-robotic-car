/** @file vehicle_controller.c
 * @brief Route operator requests and adapt observations for the shared
 * controller owner.
 */
/* Team facade/telemetry adapter. The upstream s3 integration task is the ONE
 * navigation, motion-request and report-consumer owner. No second task here. */
#include "vehicle_controller.h"
#include "../app_types.h"
#include "s3_integration.h"
#include "terr_link.h"
#include "run_timing.h"
#include "../subsystem1/buddy1.h"
#include "../subsystem2/motion_pack.h"
#if APP_S5
#include "../subsystem5/s5_api.h"
#include "../subsystem5/avoid_obstacle.h"
#endif

#define MOTION_PERIOD_US 200000U
#define STOPPED_MOTION_PERIOD_US 1000000U
#define LIVE_PERIOD_CYCLES (200U / S3_SAMPLE_PERIOD_MS)
#if (200U % S3_SAMPLE_PERIOD_MS) != 0U || LIVE_PERIOD_CYCLES == 0U ||          \
    LIVE_PERIOD_CYCLES > 63U
#error "200 ms telemetry cadence must fit the six-bit cycle dividers"
#endif
static struct
{
    uint32_t last_us;
    s3_drive_state_t state;
    unsigned seen : 1, usable : 1, stopped : 1;
} g_telemetry;

/* Route pose (car/telemetry/pose): Buddy 2's encoder distance/heading relative
 * to the start of the current run. Only the sampling owner touches it. */
#define POSE_PERIOD_CYCLES LIVE_PERIOD_CYCLES

static struct
{
    unsigned running : 1;
    unsigned divider : 6;
    float32_t start_distance_m, start_heading_deg;
} g_pose;

static buddy1_robot_state_t g_last_state;
static buddy1_barcode_action_t g_last_action;
/* Events (barcode, state, obstacle summary, scan profile, fault changes) are
 * retried every cycle while Buddy 1's 4-slot outbox is full (E_TMOUT), so a
 * busy link delays them instead of dropping them. Periodic live samples
 * (motion, pose, terrain, avoidance) stay best effort: the next one follows. */
static UB g_pending_barcode; /* letter still to be queued, 0 = none */
static unsigned g_last_faults;
static bool g_b_faults_seen;
#if APP_S5
static struct
{
    uint16_t scan;
    unsigned route : 2, invalidated : 1;
    unsigned echo : 2;     /* 2 unknown, 0 healthy, 1 no echo. */
    uint16_t profile_scan; /* last scan id sent on car/telemetry/scan */
    unsigned profile_sent : 1;
    unsigned divider : 6; /* car/telemetry/avoidance every 200 ms */
} g_obstacle_report;

/* Raw readings of the scan whose profile was just sent, at most one message
 * every SAMPLE_PERIOD cycles (car/telemetry/scan_sample), and only while the
 * 4-slot outbox is empty: spare capacity only, so barcode/state/fault events
 * and command replies never find it full because of these readings. */
#define SAMPLE_PERIOD (40U / S3_SAMPLE_PERIOD_MS) /* 40 ms */
#if SAMPLE_PERIOD == 0U || SAMPLE_PERIOD > 15U
#error "scan-sample pacing must fit its four-bit divider"
#endif
static struct
{
    s5_sample_t sample[S5_SAMPLES_MAX];
    uint16_t id;
    unsigned count : 5, next : 5, peek : 1, pending : 1, loaded : 1,
        divider : 4;
} g_scan_samples;

/* Everything Buddy 5 knows, for the laptop: live range, servo angle and
 * bypass progress at 5 Hz (phase-shifted from the other 200 ms reports so the
 * four-message outbox is not filled in one cycle), and the full FR6 profile
 * once per finished scan or side peek. Unusable values are sent as zero with
 * their validity cleared, never as a plausible number. */
#define S5_LIVE_PERIOD LIVE_PERIOD_CYCLES

static void obstacle_live_report (const s5_observation_t * p_o, bool have,
                                  unsigned control)
{
    buddy1_avoidance_t value_a = {0};
    if (++g_obstacle_report.divider < S5_LIVE_PERIOD)
    {
        return;
    }
    g_obstacle_report.divider = 0U;
    if (have)
    {
        value_a.no_echo = p_o->no_echo ? 1U : 0U;
        if (p_o->range_valid && !p_o->no_echo &&
            p_o->range_mm >= BUDDY1_OBSTACLE_MIN_MM &&
            p_o->range_mm <= BUDDY1_OBSTACLE_MAX_MM)
        {
            value_a.range_valid = 1U;
            value_a.range_mm = p_o->range_mm;
        }
    }
    value_a.servo_deg = subsystem5_servo_angle();
    if (value_a.servo_deg > BUDDY1_SERVO_MAX_DEG)
    {
        value_a.servo_deg = 0U;
    }
    value_a.state = (uint8_t)(control & 7U);
    value_a.fault = (uint8_t)((control >> 3) & 7U);
    value_a.route = (uint8_t)((control >> 6) & 3U);
    value_a.leg = (uint8_t)((control >> 8) & 15U);
    value_a.pass = (uint8_t)((control >> 12) & 7U);
    value_a.swapped = (uint8_t)((control >> 15) & 1U);
    if (value_a.state > BUDDY1_S5_STATE_MAX)
    {
        value_a.state = 0U;
    }
    if (value_a.route > BUDDY1_ROUTE_RIGHT)
    {
        value_a.route = 0U;
    }
    if (value_a.pass > BUDDY1_S5_PASS_MAX)
    {
        value_a.pass = 0U;
    }
    (void)buddy1_report_avoidance(
        &value_a); /* QoS 0: the next report follows */
}

static void obstacle_profile_report (const s5_observation_t * p_o)
{
    const s5_profile_t * p_p = &p_o->profile;
    buddy1_scan_t value_q = {0};
    ER value_er;
    if (!p_o->scan_done || (g_obstacle_report.profile_sent &&
                            g_obstacle_report.profile_scan == p_o->scan_id))
    {
        return;
    }
    value_q.scan_id = (uint8_t)(p_o->scan_id & 0xFFU);
    if (p_p->valid)
    {
        value_q.valid = 1U;
        value_q.closest_mm = p_p->closest_mm;
        value_q.width_mm = p_p->width_mm;
        value_q.left_clear_mm = p_p->left_clear_mm;
        value_q.right_clear_mm = p_p->right_clear_mm;
        value_q.forward_mm = p_p->forward_mm;
        value_q.left_edge_mm = p_p->left_edge_mm;
        value_q.right_edge_mm = p_p->right_edge_mm;
    }
    value_er = buddy1_report_scan(&value_q);
    if (value_er == E_PAR && value_q.valid)
    {
        value_q = (buddy1_scan_t){0};
        value_q.scan_id = (uint8_t)(p_o->scan_id & 0xFFU);
        value_er = buddy1_report_scan(&value_q);
    }
    if (value_er == E_OK || value_er == E_PAR)
    { /* E_TMOUT: outbox full, retry next cycle */
        g_obstacle_report.profile_scan = p_o->scan_id;
        g_obstacle_report.profile_sent = 1U;
        g_scan_samples.id = p_o->scan_id;
        g_scan_samples.pending = 1U;
        g_scan_samples.loaded = 0U;
        g_scan_samples.divider = 0U;
    }
}

static void obstacle_samples_report (void)
{
    buddy1_scan_sample_t value_q = {0};
    unsigned angle, value_mm, count;
    bool valid, peek;
    uint16_t value_id;
    ER value_er;
    if (!g_scan_samples.pending || ++g_scan_samples.divider < SAMPLE_PERIOD)
    {
        return;
    }
    g_scan_samples.divider = 0U;
    if (!g_scan_samples.loaded)
    {
        if (!subsystem5_scan_samples(&value_id, &peek, g_scan_samples.sample,
                                     &count))
        {
            return; /* busy: retry */
        }
        if (value_id != g_scan_samples.id || count == 0U ||
            count > BUDDY1_SCAN_SAMPLES_MAX)
        {
            g_scan_samples.pending = 0U;
            return; /* no readings kept for this scan */
        }
        g_scan_samples.count = count;
        g_scan_samples.next = 0U;
        g_scan_samples.peek = peek ? 1U : 0U;
        g_scan_samples.loaded = 1U;
    }
    if (!buddy1_outbox_idle())
    {
        return; /* busy link: same reading later */
    }
    if (!s5_sample_unpack(&g_scan_samples.sample[g_scan_samples.next], &angle,
                          &value_mm, &valid))
    {
        value_er = E_PAR;
    }
    else
    {
        value_q.scan_id = (uint8_t)(g_scan_samples.id & 0xFFU);
        value_q.peek = (uint8_t)g_scan_samples.peek;
        value_q.index = (uint8_t)g_scan_samples.next;
        value_q.count = (uint8_t)g_scan_samples.count;
        value_q.angle_deg = (uint8_t)angle;
        value_q.valid = valid ? 1U : 0U;
        value_q.distance_mm = valid ? (uint16_t)value_mm : 0U;
        value_er = buddy1_report_scan_sample(&value_q);
    }
    if (value_er == E_TMOUT)
    {
        return; /* outbox full: same reading next time */
    }
    if (++g_scan_samples.next >= g_scan_samples.count)
    {
        g_scan_samples.pending = 0U;
    }
}
#endif
/* True only between a prepare_start() that newly froze the Buddy 1 map and
 * the end of that same START attempt. Only the sampling owner touches it. */
static bool g_start_claimed_map;

static buddy1_barcode_action_t action_for (s3_nav_action_t action)
{
    switch (action)
    {
    case S3_NAV_STRAIGHT:
        return BUDDY1_ACTION_FORWARD;
    case S3_NAV_LEFT:
        return BUDDY1_ACTION_LEFT;
    case S3_NAV_RIGHT:
        return BUDDY1_ACTION_RIGHT;
    case S3_NAV_UTURN:
        return BUDDY1_ACTION_UTURN;
    default:
        return BUDDY1_ACTION_UNKNOWN;
    }
}

static vehicle_controller_state_t public_state (s3_drive_state_t state)
{
    switch (state)
    {
    case S3_DRIVE_FOLLOW:
        return VC_LINE_FOLLOW;
    case S3_DRIVE_BRAKING:
    case S3_DRIVE_MANOEUVRE:
    case S3_DRIVE_REACQUIRE:
        return VC_TURNING;
    case S3_DRIVE_HALTED:
        return VC_STOPPED;
    case S3_DRIVE_FAULT:
        return VC_FAULT;
    default:
        return VC_WAITING;
    }
}

static buddy1_robot_state_t wire_state (s3_drive_state_t state)
{
    if (state == S3_DRIVE_FAULT)
    {
        return BUDDY1_STATE_ERROR;
    }
    if (state == S3_DRIVE_OFF || state == S3_DRIVE_HALTED)
    {
        return BUDDY1_STATE_STOPPED;
    }
    if (state == S3_DRIVE_MANOEUVRE)
    {
        if (g_last_action == BUDDY1_ACTION_LEFT)
        {
            return BUDDY1_STATE_TURNING_LEFT;
        }
        if (g_last_action == BUDDY1_ACTION_RIGHT)
        {
            return BUDDY1_STATE_TURNING_RIGHT;
        }
        if (g_last_action == BUDDY1_ACTION_UTURN)
        {
            return BUDDY1_STATE_UTURN;
        }
    }
    return BUDDY1_STATE_LINE_FOLLOWING;
}

static bool run_active (s3_drive_state_t state)
{
    return state == S3_DRIVE_FOLLOW || state == S3_DRIVE_BRAKING ||
           state == S3_DRIVE_MANOEUVRE || state == S3_DRIVE_REACQUIRE;
}

/* Repeating zero-speed packets need only a 1 Hz heartbeat while stopped (the
 * dashboard's motion freshness window is 3 s). Keep 5 Hz when moving or the
 * estimate is unavailable, and report transitions on the next observation. */
static void motion_report (const s3_integration_status_t * p_status,
                           const motion_status_t * p_m, bool available)
{
    const uint32_t now = p_status->navigation.timestamp_us;
    float32_t left = 0.0f, right = 0.0f;
    bool usable = available && p_m->b_valid && !p_m->b_busy;
    if (usable)
    {
        left = p_m->speed_left_mps * 1000.0f;
        right = p_m->speed_right_mps * 1000.0f;
        /* Keep the agreed wheel range unchanged. Calibrated top speed exceeds
         * +/-510 mm/s: mark unavailable, NEVER clip or widen the wire. */
        usable =
            left == left && right == right && left >= -BUDDY1_SPEED_MAX_MMS &&
            left <= BUDDY1_SPEED_MAX_MMS && right >= -BUDDY1_SPEED_MAX_MMS &&
            right <= BUDDY1_SPEED_MAX_MMS;
    }
    /* The estimator's smoothing can leave tiny nonzero floats after braking.
     * Only slow down when the brake/still acknowledgement agrees and both
     * integer-mm/s values already report zero. Representable wheel movement
     * restores 5 Hz even when the drive state says HALTED or FAULT. */
    const bool stopped = usable && !run_active(p_status->state) &&
                         p_m->b_stopped && (H)left == 0 && (H)right == 0;
    const uint32_t period =
        stopped ? STOPPED_MOTION_PERIOD_US : MOTION_PERIOD_US;
    if (g_telemetry.seen && g_telemetry.state == p_status->state &&
        g_telemetry.usable == usable && g_telemetry.stopped == stopped &&
        (uint32_t)(now - g_telemetry.last_us) < period)
    {
        return;
    }

    g_telemetry.seen = true;
    g_telemetry.state = p_status->state;
    g_telemetry.usable = usable;
    g_telemetry.stopped = stopped;
    g_telemetry.last_us = now;
    if (usable)
    {
        (void)buddy1_report_motion((H)left, (H)right);
    }
    else
    {
        (void)buddy1_invalidate(BUDDY1_TOPIC_MOTION);
    }
}

static int32_t round_to_int (float32_t value_x)
{
    return (int32_t)(value_x >= 0.0f ? value_x + 0.5f : value_x - 0.5f);
}

static void send_pose (const motion_status_t * p_m)
{
    int32_t value_cm =
        round_to_int((p_m->distance_m - g_pose.start_distance_m) * 100.0f);
    int32_t deg = round_to_int(p_m->heading_deg - g_pose.start_heading_deg);
    value_cm %= (int32_t)BUDDY1_POSE_DISTANCE_MOD;
    if (value_cm < 0)
    {
        value_cm += (int32_t)BUDDY1_POSE_DISTANCE_MOD;
    }
    deg %= 360;
    if (deg < 0)
    {
        deg += 360;
    }
    /* QoS 0 live data: a full outbox drops this sample; the next one,
     * 200 ms later, carries the current pose (the laptop unwraps distance). */
    (void)buddy1_report_pose((UH)value_cm, (H)deg);
}

/* One pose at START (0, 0), then every 200 ms while the run is active, and a
 * final one when it ends. Skipped while Buddy 2's estimate is unusable. */
static void pose_report (const s3_integration_status_t * p_status,
                         const motion_status_t * p_m, bool available)
{
    const bool active = run_active(p_status->state);
    const bool usable = available && p_m->b_valid && !p_m->b_busy &&
                        p_m->distance_m == p_m->distance_m &&
                        p_m->heading_deg == p_m->heading_deg;
    if (active && !g_pose.running)
    {
        if (!usable)
        {
            return; /* start reference needs a valid estimate */
        }
        g_pose.running = true;
        g_pose.divider = 0U;
        g_pose.start_distance_m = p_m->distance_m;
        g_pose.start_heading_deg = p_m->heading_deg;
        send_pose(p_m);
    }
    else if (active)
    {
        if (++g_pose.divider >= POSE_PERIOD_CYCLES)
        {
            g_pose.divider = 0U;
            if (usable)
            {
                send_pose(p_m);
            }
        }
    }
    else if (g_pose.running)
    {
        g_pose.running = false;
        if (usable)
        {
            send_pose(p_m);
        }
    }
}

ER vehicle_controller_init (void)
{
    terrain_control_init();
    g_pose.running = false;
    g_pose.divider = 0U;
    g_telemetry.seen = false;
    g_last_state = BUDDY1_STATE_UNKNOWN;
    g_last_action = BUDDY1_ACTION_UNKNOWN;
    g_last_faults = 0U;
    g_b_faults_seen = false;
    g_pending_barcode = 0U;
    g_start_claimed_map = false;
#if APP_S5
    g_obstacle_report.scan = 0;
    g_obstacle_report.route = 0;
    g_obstacle_report.echo = 2U;
    g_obstacle_report.invalidated = false;
    g_obstacle_report.profile_scan = 0U;
    g_obstacle_report.profile_sent = 0U;
    g_obstacle_report.divider = S5_LIVE_PERIOD / 4U;
    g_scan_samples.pending = 0U;
    g_scan_samples.loaded = 0U;
#endif
    return E_OK;
}

ER vehicle_controller_start (void)
{
    return E_OK;
}

ER vehicle_controller_begin (void)
{
    const s3_control_request_t request = {.kind = S3_CONTROL_START};
    return s3_integration_submit(&request) ? E_OK : E_OBJ;
}

ER vehicle_controller_request_stop (void)
{
    const s3_control_request_t request = {.kind = S3_CONTROL_STOP};
    return s3_integration_submit(&request) ? E_OK : E_OBJ;
}

vehicle_controller_state_t vehicle_controller_state (void)
{
    /* Only the normal controller/UI task may use this convenience view.
     * Keep the temporary copy task-local, not a second shared snapshot. */
    s3_integration_status_t copy;
    return s3_integration_status(&copy) ? public_state(copy.state) : VC_WAITING;
}

/* Compatibility boundary: new sensor observations are processed in the SAME
 * owner as the upstream drive FSM. Never accept a second external producer. */
ER vehicle_controller_submit_nav (const s3_navigation_output_t * p_output)
{
    (void)p_output;
    return E_NOSPT;
}

/* Called by the sampling/controller owner immediately before an explicit START.
 * Freeze and copy the Buddy 1 map atomically, then update its private nav
 * context. */
bool vehicle_controller_prep_start (s3_navigation_t * p_navigation)
{
    buddy1_barcode_map_t map;
    BOOL newly = FALSE;
    s3_nav_action_t expanded[26] = {0};
    g_start_claimed_map = false;
    if (!p_navigation || buddy1_configuration_claim(&map, &newly) != E_OK)
    {
        return false;
    }
    expanded[map.forward - 'A'] = S3_NAV_STRAIGHT;
    expanded[map.left - 'A'] = S3_NAV_LEFT;
    expanded[map.right - 'A'] = S3_NAV_RIGHT;
    expanded[map.uturn - 'A'] = S3_NAV_UTURN;
    if (!s3_navigation_set_mapping(p_navigation, expanded))
    {
        /* Nothing installed: undo only a freeze this attempt took. */
        if (newly)
        {
            (void)buddy1_configuration_reopen();
        }
        return false;
    }
    g_start_claimed_map = newly != FALSE;
    return true;
}

void vehicle_controller_cancel_start (void)
{
    if (g_start_claimed_map)
    {
        (void)buddy1_configuration_reopen();
    }
    g_start_claimed_map = false;
}

void vehicle_controller_commit_start (void)
{
    g_start_claimed_map = false;
}

bool vehicle_controller_set_mapping (const s3_nav_action_t p_map[26])
{
    buddy1_barcode_map_t reduced = {0};
    if (!p_map)
    {
        return false;
    }
    for (unsigned value_i = 0U; value_i < 26U; value_i++)
    {
        uint8_t * p_slot;
        switch (p_map[value_i])
        {
        case S3_NAV_NONE:
            continue;
        case S3_NAV_STRAIGHT:
            p_slot = &reduced.forward;
            break;
        case S3_NAV_LEFT:
            p_slot = &reduced.left;
            break;
        case S3_NAV_RIGHT:
            p_slot = &reduced.right;
            break;
        case S3_NAV_UTURN:
            p_slot = &reduced.uturn;
            break;
        default:
            return false;
        }
        if (*p_slot)
        {
            return false; /* Team protocol permits exactly four letters. */
        }
        *p_slot = (uint8_t)('A' + value_i);
    }
    return buddy1_apply_barcode_map(&reduced) == E_OK;
}

/* Called only by the upstream sampling/controller owner, never under its
 * snapshot mutex. This mirror cannot command motors or consume reports. */
void vehicle_controller_observe (const s3_integration_status_t * p_status)
{
    motion_status_t motion;
    bool motion_available;
    unsigned active =
        g_last_faults & (BUDDY1_FAULT_STALL | BUDDY1_FAULT_ENCODER);
    unsigned changed = BUDDY1_FAULT_IR;
    buddy1_robot_state_t state;
    if (!p_status || g_mtx_buddy1_state <= 0)
    {
        return;
    }
    terrain_control_report();
    if (p_status->navigation.events & S3_EVENT_BARCODE)
    {
        g_last_action = action_for(p_status->navigation.action);
        g_pending_barcode = (UB)p_status->navigation.barcode.letter;
    }
    if (g_pending_barcode &&
        buddy1_report_barcode(g_pending_barcode) != E_TMOUT)
    {
        g_pending_barcode = 0U;
    }
    /* One coherent read for faults, route and speed. Do not reacquire Buddy 2's
     * mutex or repeat encoder/heading arithmetic for each telemetry consumer.
     */
    motion_available = motion_get_status(&motion);
    if (motion_available)
    {
        active = 0U;
        changed |= BUDDY1_FAULT_STALL | BUDDY1_FAULT_ENCODER;
        if (motion.fault_bits & motion_fault_bit(MOTION_FAULT_STALL))
        {
            active |= BUDDY1_FAULT_STALL;
        }
        if (motion.fault_bits & motion_fault_bit(MOTION_FAULT_ENCODER))
        {
            active |= BUDDY1_FAULT_ENCODER;
        }
    }
    if (p_status->navigation.fault_mask ||
        (p_status->navigation.latched_faults & S3_FAULT_SENSOR))
    {
        active |= BUDDY1_FAULT_IR;
    }
    if (!g_b_faults_seen || active != g_last_faults)
    {
        ER error = buddy1_update_faults((UB)changed, (UB)(active & changed));
        if (error == E_OK)
        {
            g_last_faults = active;
            g_b_faults_seen = true;
        }
    }
    state = wire_state(p_status->state);
    pose_report(p_status, &motion, motion_available);
#if APP_S5
    {
        s5_observation_t obstacle;
        unsigned control = subsystem5_controller_status();
        unsigned obstacle_state = control & 7U, route = (control >> 6) & 3U;
        bool have_obstacle;
        if (p_status->state == S3_DRIVE_MANOEUVRE &&
            obstacle_state >= S5_WAIT_STOP && obstacle_state <= S5_FINISH)
        {
            state = BUDDY1_STATE_AVOIDING_OBSTACLE;
        }
        have_obstacle = subsystem5_snapshot(&obstacle);
        obstacle_live_report(&obstacle, have_obstacle, control);
        if (have_obstacle)
        {
            obstacle_profile_report(&obstacle);
            obstacle_samples_report();
            if (g_obstacle_report.echo != (unsigned)obstacle.no_echo)
            {
                ER value_er = buddy1_update_faults(
                    BUDDY1_FAULT_NO_ECHO,
                    obstacle.no_echo ? BUDDY1_FAULT_NO_ECHO : 0U);
                if (value_er == E_OK)
                {
                    g_obstacle_report.echo = (unsigned)obstacle.no_echo;
                }
            }
            if (obstacle.scan_done && obstacle.profile.valid &&
                (g_obstacle_report.scan != obstacle.scan_id ||
                 g_obstacle_report.route != route))
            {
                ER value_er = buddy1_report_obstacle(
                    obstacle.profile.closest_mm, obstacle.profile.width_mm,
                    (buddy1_obstacle_route_t)route);
                if (value_er == E_OK || value_er == E_PAR)
                {
                    g_obstacle_report.scan = obstacle.scan_id;
                    g_obstacle_report.route = route;
                    g_obstacle_report.invalidated = false;
                }
            }
            else if (!obstacle.profile.valid && !g_obstacle_report.invalidated)
            {
                if (buddy1_invalidate(BUDDY1_TOPIC_OBSTACLE) == E_OK)
                {
                    g_obstacle_report.invalidated = true;
                }
            }
        }
    }
#endif
    if (state != g_last_state)
    {
        ER error = buddy1_report_robot_state(state);
        if (error == E_OK)
        {
            g_last_state = state;
        }
    }
    motion_report(p_status, &motion, motion_available);
    if (p_status->state == S3_DRIVE_OFF)
    {
        g_last_action = BUDDY1_ACTION_UNKNOWN;
    }
}

/* End of file. */

