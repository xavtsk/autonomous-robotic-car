/** @file team_adapter_main.c
 * @brief Exercise controller-to-telemetry adapters and motion reporting
 * cadence.
 * The selected narrow integer operands fit in int.
 * Unsigned casts spell out the original modulo conversion.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "../../app_program/controller/vehicle_controller.h"
#include "../../app_program/controller/run_timing.h"
#define LIVE_CYCLES (200U / S3_SAMPLE_PERIOD_MS)
#define STOPPED_CYCLES (1000U / S3_SAMPLE_PERIOD_MS)
#include "team_adapter_main.h"
#include "../../app_program/subsystem2/motion_pack.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
ID                             gh_buddy1_state_mutex = 1;
static motion_status_t         g_status;
static s3_integration_status_t g_runtime;
static s3_integration_kind_t   g_queued;
static bool                    gb_is_queue_ok  = true;
static bool                    gb_is_status_ok = true;
static bool                    gb_is_frozen;
static unsigned                g_mask = BUDDY1_FAULT_WIFI;
static unsigned                g_updates;
static unsigned                g_invalidations;
static unsigned                g_reports;
static unsigned                g_states;
static unsigned                g_barcodes;
static unsigned                g_motion_reads;
static ER                      g_mirror_result = E_OK;
static telemetry_codec_state_t g_last_state;
static int                     g_left_report;
static int                     g_right_report;

/* E_TMOUT = Buddy 1 outbox full: the controller must retry the event. */
static ER g_event_result = E_OK;

static unsigned g_poses;
static int      g_pose_cm  = -1;
static int      g_pose_deg = -1;

static unsigned g_reopens;

static void observe_after (uint32_t elapsed_us);
static void cycles (unsigned count);
static void motion_cadence_test (void);

/** @brief Record the requested control kind and return the configured queue
 * result.
 */
bool s3_integration_submit (const s3_integration_request_t * p_request)
{
    g_queued = p_request->kind;
    return gb_is_queue_ok;
}

/** @brief Copy the current simulated integration status into the output and
 * report success.
 */
bool s3_integration_status (s3_integration_status_t * p_out)
{
    *p_out = g_runtime;
    return true;
}

/** @brief Count the read and copy the fixture status to the output,
 * including when the injected return value signals failure.
 */
bool motion_get_status (motion_status_t * p_out)
{
    g_motion_reads++;
    *p_out = g_status;
    return gb_is_status_ok;
}

/** @brief Record the fault update. Apply the changed bits on success or
 * timeout, reject Wi-Fi ownership in the fixture and return the
 * injected result.
 */
ER buddy1_update_faults (UB changed, UB active)
{
    assert(!(changed & BUDDY1_FAULT_WIFI));
    g_updates++;

    if ((E_OK == g_mirror_result) || (E_TMOUT == g_mirror_result))
    {
        g_mask = ((unsigned)g_mask & ~(unsigned)changed) | active;
    }

    return g_mirror_result;
}

/** @brief Count the attempted state publication. Save the state only when
 * the configured event result is successful.
 */
ER buddy1_report_robot_state (telemetry_codec_state_t state)
{
    g_states++;

    if (g_event_result != E_OK)
    {
        return g_event_result;
    }

    g_last_state = state;
    return E_OK;
}

/** @brief Require the fixture letter B, count the publication attempt and
 * return the configured event result.
 */
ER buddy1_report_barcode (UB letter)
{
    assert('B' == (int)letter);
    g_barcodes++;
    return g_event_result;
}

/** @brief Record both wheel speeds in millimetres per second and count the
 * motion report.
 */
ER buddy1_report_motion (H left, H right)
{
    g_reports++;
    g_left_report  = left;
    g_right_report = right;
    return E_OK;
}

/** @brief Check the modulo distance in centimetres and heading in degrees,
 * then save both values and count the report.
 */
ER buddy1_report_pose (UH distance_cm, H deg)
{
    assert(distance_cm < BUDDY1_POSE_DISTANCE_MOD && deg >= 0 && deg < 360);
    g_poses++;
    g_pose_cm  = distance_cm;
    g_pose_deg = deg;
    return E_OK;
}

/** @brief Require the motion topic and count one invalidation.
 */
ER buddy1_invalidate (UB type)
{
    assert(BUDDY1_TOPIC_MOTION == (int)type);
    g_invalidations++;
    return E_OK;
}

/** @brief Copy the four fixture mappings and freeze them. If requested, tell
 * the caller whether this claim acquired the freeze.
 */
ER buddy1_configuration_claim (telemetry_codec_map_t * p_map,
                               BOOL *                  pb_did_close)
{
    *p_map = (telemetry_codec_map_t){'A', 'B', 'C', 'D'};

    if (pb_did_close)
    {
        *pb_did_close = !gb_is_frozen;
    }

    gb_is_frozen = true;
    return E_OK;
}

/** @brief Reopen the simulated map and count the reopening.
 */
ER buddy1_configuration_reopen (void)
{
    gb_is_frozen = false;
    g_reopens++;
    return E_OK;
}

/** @brief Reject a frozen map. Otherwise accept only the fixture's four
 * assigned letters in their expected action slots.
 */
ER buddy1_apply_barcode_map (const telemetry_codec_map_t * p_map)
{
    if (gb_is_frozen)
    {
        return E_OBJ;
    }

    return ((('A' == (int)p_map->forward) && ('B' == (int)p_map->left)) &&
            ('C' == (int)p_map->right)) &&
                   ('D' == (int)p_map->uturn)
               ? E_OK
               : E_PAR;
}

/** @brief Copy the complete mapping into the simulated navigation
 * configuration and report success.
 */
bool line_navigation_set_mapping (line_navigation_t *            p_navigation,
                                  const line_navigation_action_t map[26])
{
    memcpy(p_navigation->config.map, map, sizeof(p_navigation->config.map));
    return true;
}

/** @brief Feed controller states into the telemetry adapter, including fault
 * transitions and timestamp wrap, and check the resulting reports.
 */
int main (void)
{
    assert(vehicle_controller_init() == E_OK &&
           vehicle_controller_start() == E_OK);
    assert(vehicle_controller_begin() == E_OK &&
           (unsigned)S3_CONTROL_START == g_queued);
    assert(vehicle_controller_request_stop() == E_OK &&
           (unsigned)S3_CONTROL_STOP == g_queued);
    gb_is_queue_ok = false;
    assert(vehicle_controller_begin() == E_OBJ);
    gb_is_queue_ok             = true;
    g_runtime.state            = S3_DRIVE_FOLLOW;
    g_status.b_is_status_valid = true;
    g_status.speed_left_mps    = 0.12f;
    g_status.speed_right_mps   = -0.12f;
    cycles(LIVE_CYCLES);
    assert(1U == g_reports && 120 == g_left_report && -120 == g_right_report);
    assert(LIVE_CYCLES ==
           g_motion_reads); /* Share one read per cycle across all reports. */
    assert((unsigned)BUDDY1_STATE_LINE_FOLLOWING == g_last_state &&
           BUDDY1_FAULT_WIFI == g_mask);
    /* clang-format off */
    g_status.fault_bits             =
        (unsigned)motion_pack_fault_bit(MOTION_FAULT_STALL) |
                          (unsigned)motion_pack_fault_bit(MOTION_FAULT_ENCODER);
    g_runtime.navigation.fault_mask = 2;
    /* clang-format on */
    cycles(1);
    assert(g_mask == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL |
                      BUDDY1_FAULT_ENCODER | BUDDY1_FAULT_IR));
    unsigned before = g_updates;
    cycles(1);
    assert(g_updates == before);
    gb_is_status_ok                 = false;
    g_runtime.navigation.fault_mask = 0;
    cycles(1);
    assert(g_mask ==
           (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL | BUDDY1_FAULT_ENCODER));
    gb_is_status_ok     = true;
    g_status.fault_bits = 0;
    cycles(1);
    assert(BUDDY1_FAULT_WIFI == g_mask);
    g_runtime.navigation.events         = S3_EVENT_BARCODE;
    g_runtime.navigation.barcode.letter = 'B';
    g_runtime.navigation.action         = S3_NAV_LEFT;
    cycles(1);
    assert(1U == g_barcodes);
    g_runtime.navigation.events = 0;
    cycles(3);
    assert(1U == g_barcodes); /* Each event gets one report. */
    /* Full outbox: barcode and state are retried every cycle, not dropped. */
    g_event_result              = E_TMOUT;
    g_runtime.navigation.events = S3_EVENT_BARCODE;
    cycles(1);
    g_runtime.navigation.events = 0;
    cycles(2);
    assert(4U == g_barcodes);
    g_runtime.state = S3_DRIVE_MANOEUVRE;
    {
        unsigned states_before = g_states;
        cycles(3);
        assert(g_states == states_before + 3U && 7U == g_barcodes &&
               g_last_state != (unsigned)BUDDY1_STATE_TURNING_LEFT);
    }
    g_event_result = E_OK;
    cycles(1);
    assert(8U == g_barcodes &&
           (unsigned)BUDDY1_STATE_TURNING_LEFT == g_last_state);
    {
        unsigned states_before = g_states;
        cycles(3);
        assert(8U == g_barcodes && g_states == states_before);
    }
    assert((unsigned)BUDDY1_STATE_TURNING_LEFT == g_last_state);
    g_runtime.state = S3_DRIVE_FAULT;
    cycles(1);
    assert((unsigned)BUDDY1_STATE_ERROR == g_last_state);
    g_status.speed_left_mps = 0.884f;
    before                  = g_invalidations;
    unsigned reports_before = g_reports;
    cycles(LIVE_CYCLES);
    assert(g_invalidations > before &&
           g_reports == reports_before); /* Never clip into a valid 510. */
    g_status.b_is_status_valid = false;
    before                     = g_invalidations;
    cycles(LIVE_CYCLES);
    assert(g_invalidations > before);
    g_runtime.navigation.fault_mask = 1;
    g_mirror_result                 = E_OBJ;
    before                          = g_updates;
    cycles(2);
    assert(g_updates == before + 2U);
    /* Full outbox (E_TMOUT): the fault change is retried until it is queued. */
    g_mirror_result = E_TMOUT;
    before          = g_updates;
    cycles(2);
    assert(g_updates == before + 2U);
    g_mirror_result = E_OK;
    cycles(1);
    before = g_updates;
    cycles(1);
    assert(g_updates == before);
    line_navigation_action_t map[26] = {S3_NAV_STRAIGHT, S3_NAV_LEFT,
                                        S3_NAV_RIGHT, S3_NAV_UTURN};
    assert(vehicle_controller_set_mapping(map));
    map[4] = S3_NAV_LEFT;
    assert(!vehicle_controller_set_mapping(map));
    map[4]                = S3_NAV_NONE;
    line_navigation_t nav = {0};
    assert(vehicle_controller_prep_start(&nav) && gb_is_frozen);
    assert((unsigned)S3_NAV_STRAIGHT == nav.config.map[0] &&
           (unsigned)S3_NAV_LEFT == nav.config.map[1]);
    assert(!vehicle_controller_set_mapping(map));
    vehicle_controller_cancel_start();
    assert(!gb_is_frozen && 1U == g_reopens);
    assert(vehicle_controller_set_mapping(map));
    /* Accepted START: the freeze belongs to the run; a stray cancel is a no-op.
     */
    assert(vehicle_controller_prep_start(&nav) && gb_is_frozen);
    vehicle_controller_commit_start();
    vehicle_controller_cancel_start();
    assert(gb_is_frozen && 1U == g_reopens);
    /* Later refused restart: map was already frozen, so it is NOT reopened. */
    assert(vehicle_controller_prep_start(&nav) && gb_is_frozen);
    vehicle_controller_cancel_start();
    assert(gb_is_frozen && 1U == g_reopens);
    assert(!vehicle_controller_set_mapping(map));
    assert(vehicle_controller_submit_nav(&g_runtime.navigation) == E_NOSPT);
    /* Route pose: relative to START, 5 Hz while running, final one at the end.
     */
    g_runtime                  = (s3_integration_status_t){0};
    g_status                   = (motion_status_t){0};
    g_status.b_is_status_valid = true;
    assert(vehicle_controller_init() == E_OK);
    g_poses              = 0;
    g_status.distance_m  = 1.00f;
    g_status.heading_deg = 350.0f;
    cycles(5);
    assert(0U == g_poses); /* The controller is stopped. */
    g_status.b_is_status_valid = false;
    g_runtime.state            = S3_DRIVE_FOLLOW;
    cycles(3);
    assert(0U == g_poses); /* No pose reference exists yet. */
    g_status.b_is_status_valid = true;
    cycles(1);
    assert(1U == g_poses && 0 == g_pose_cm && 0 == g_pose_deg); /* START */
    g_status.distance_m  = 1.25f;
    g_status.heading_deg = 10.0f;
    {
        uint32_t started = g_runtime.navigation.timestamp_us;
        cycles(LIVE_CYCLES / 2U);
        assert(1U == g_poses); /* Do not send again at 100 ms. */
        assert(g_runtime.navigation.timestamp_us - started == 100000U);
        cycles(LIVE_CYCLES - (LIVE_CYCLES / 2U) - 1U);
        assert(1U == g_poses);
    }
    cycles(1);
    assert(2U == g_poses && 25 == g_pose_cm &&
           20 == g_pose_deg); /* The heading wraps. */
    g_runtime.state      = S3_DRIVE_MANOEUVRE;
    g_status.distance_m  = 0.90f;
    g_status.heading_deg = 260.0f;
    cycles(LIVE_CYCLES);
    assert(3U == g_poses && (4096 - 10) == g_pose_cm &&
           270 == g_pose_deg); /* Reverse beyond the start pose. */
    g_status.distance_m = 42.004f;
    cycles(LIVE_CYCLES);
    assert(4U == g_poses &&
           (4100 % 4096) == g_pose_cm); /* A 41 m distance wraps. */
    g_status.b_is_status_valid = false;
    cycles(2U * LIVE_CYCLES);
    assert(4U == g_poses); /* Skip unusable feedback. */
    g_status.b_is_status_valid = true;
    g_runtime.state            = S3_DRIVE_HALTED;
    cycles(1);
    assert(5U == g_poses); /* Publish the final pose. */
    cycles(2U * LIVE_CYCLES);
    assert(5U == g_poses); /* A stopped drive sends no pose updates. */
    g_status.distance_m  = 5.0f;
    g_status.heading_deg = 90.0f;
    g_runtime.state      = S3_DRIVE_FOLLOW;
    cycles(1);
    assert(6U == g_poses && 0 == g_pose_cm &&
           0 == g_pose_deg); /* Start a new run. */
    motion_cadence_test();
    puts("PASS: one-owner facade, explicit queued start/stop, full "
         "fault-bit ownership, unknown-status preservation, map freeze, "
         "telemetry and out-of-range invalidation, route pose relative to "
         "START (wrap, reverse, final, new run), elapsed-time motion "
         "cadence (stopped 1 Hz, moving/invalid 5 Hz, immediate transitions,"
         " timestamp wrap)");
}

/** @brief Advance the navigation timestamp by the supplied microseconds and
 * present the current runtime observations to the controller.
 */
static void observe_after (uint32_t elapsed_us)
{
    g_runtime.navigation.timestamp_us += elapsed_us;
    vehicle_controller_observe(&g_runtime);
}

/** @brief Deliver the requested number of observations spaced at the
 * integrated IR sampling period.
 */
static void cycles (unsigned count)
{
    for (unsigned sample_index = 0; sample_index < count; sample_index++)
    {
        observe_after(S3_SAMPLE_PERIOD_US);
    }
}

/** @brief Check publication timing for a fresh motion snapshot, including
 * repeated timestamps and large gaps.
 */
static void motion_cadence_test (void)
{
    /* clang-format off */
    g_runtime                  = (s3_integration_status_t){0};
    g_status                   = (motion_status_t){0};
    g_status.b_is_status_valid = true;
    g_status.b_is_stopped      = true;
    gb_is_status_ok            = true;
    g_mirror_result            = E_OK;
    g_reports                  = g_invalidations = g_motion_reads = 0;
    /* clang-format on */
    assert(vehicle_controller_init() == E_OK);
    observe_after(0);
    assert(1U == g_reports); /* Publish the initial zero immediately. */
    cycles(STOPPED_CYCLES - 1U);
    observe_after(S3_SAMPLE_PERIOD_US - 1U);
    assert(1U == g_reports);
    observe_after(1);
    assert(2U == g_reports); /* Send a stopped heartbeat at 1 s. */
    g_runtime.state = S3_DRIVE_FOLLOW;
    cycles(1);
    assert(3U == g_reports);
    cycles(LIVE_CYCLES - 1U);
    assert(3U == g_reports);
    cycles(1);
    assert(4U == g_reports); /* START restores the 5 Hz rate. */
    g_status.speed_left_mps  = 0.12f;
    g_status.speed_right_mps = -0.12f;
    cycles(LIVE_CYCLES - 1U);
    assert(4U == g_reports);
    cycles(1);
    assert(5U == g_reports && 120 == g_left_report && -120 == g_right_report);
    g_runtime.state         = S3_DRIVE_HALTED;
    g_status.speed_left_mps = g_status.speed_right_mps = 0.0f;
    cycles(1);
    assert(6U == g_reports && 0 == g_left_report &&
           0 == g_right_report); /* Publish a final zero. */
    /* Filter residue that already reports zero does not defeat the idle rate;
     * a stopped state/brake flag cannot hide representable wheel movement. */
    g_status.speed_left_mps = 0.0004f;
    cycles(LIVE_CYCLES - 1U);
    assert(6U == g_reports);
    g_status.speed_left_mps = 0.002f;
    cycles(1);
    assert(7U == g_reports && 2 == g_left_report);
    cycles(LIVE_CYCLES - 1U);
    assert(7U == g_reports);
    cycles(1);
    assert(8U == g_reports);
    g_status.speed_left_mps = 0.0f;
    g_status.b_is_stopped   = false;
    cycles(LIVE_CYCLES);
    assert(9U == g_reports); /* The stop is still unacknowledged. */
    g_status.b_is_stopped = true;
    cycles(1);
    assert(10U == g_reports);
    g_runtime.state                 = S3_DRIVE_FAULT;
    g_runtime.navigation.fault_mask = 1;
    cycles(1);
    assert(11U == g_reports && (unsigned)BUDDY1_STATE_ERROR == g_last_state &&
           (g_mask & BUDDY1_FAULT_IR));
    gb_is_status_ok = false;
    cycles(1);
    assert(1U == g_invalidations); /* Publish invalidation immediately. */
    cycles(LIVE_CYCLES - 1U);
    assert(1U == g_invalidations);
    cycles(1);
    assert(2U == g_invalidations);
    gb_is_status_ok = true;
    cycles(1);
    assert(12U == g_reports); /* Publish recovery immediately. */
    g_status.b_is_busy = true;
    cycles(1);
    assert(3U == g_invalidations);
    g_status.b_is_busy = false;
    cycles(1);
    assert(13U == g_reports);
    g_status.b_is_status_valid = false;
    cycles(1);
    assert(4U == g_invalidations);
    g_status.b_is_status_valid = true;
    cycles(1);
    assert(14U == g_reports);
    g_status.speed_left_mps = 0.884f;
    cycles(1);
    assert(5U == g_invalidations && 14U == g_reports);
    g_status.speed_left_mps = 0.0f;
    cycles(1);
    assert(15U == g_reports);

    /* Cadence uses elapsed time, survives timestamp wrap and never catches up
     * by flooding the outbox after missed observations. */
    assert(vehicle_controller_init() == E_OK);
    g_reports                         = 0;
    g_runtime.navigation.timestamp_us = UINT32_MAX - 500000U;
    observe_after(0);
    assert(1U == g_reports);
    observe_after(999999U);
    assert(1U == g_reports);
    observe_after(1);
    assert(2U == g_reports);
    g_runtime.state = S3_DRIVE_FOLLOW;
    observe_after(0);
    assert(3U == g_reports);
    observe_after(199999U);
    assert(3U == g_reports);
    observe_after(1);
    assert(4U == g_reports);
    observe_after(600000U);
    assert(5U == g_reports);
    observe_after(0);
    assert(5U == g_reports);
}

/* End of file. */

