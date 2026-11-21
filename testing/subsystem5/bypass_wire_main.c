/** @file bypass_wire_main.c
 * @brief Exercise obstacle observations and scan samples through telemetry
 * adapters.
 * The selected narrow integer operands fit in int.
 * Unsigned casts spell out the original modulo conversion.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "../../app_program/controller/vehicle_controller.h"
#include "../../app_program/controller/run_timing.h"

#define REPORTED_SCAN_SAMPLE_COUNT 3U

#define LIVE_CYCLES (200U / S3_SAMPLE_PERIOD_MS)
#define SC                                                                     \
    (40U / S3_SAMPLE_PERIOD_MS) /* Pace scan samples in controller cycles. */
#include "../../app_program/subsystem1/buddy1.h"
#include "../../app_program/subsystem2/motion_pack.h"
#include "bypass_wire_main.h"
#include "../../app_program/subsystem5/avoid_obstacle.h"
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
static unsigned                g_obstacle_invalidations;
static ER                      g_mirror_result = E_OK;
static telemetry_codec_state_t g_last_state;
static int                     g_left_report;
static int                     g_right_report;

/* E_TMOUT = Buddy 1 outbox full: the controller must retry the event. */
static ER g_event_result = E_OK;

static unsigned g_reopens;

static s5_codec_observation_t      g_obstacle;
static bool                        gb_is_available;
static uint16_t                    g_control;
static uint8_t                     g_servo = 90;
static unsigned                    g_obstacle_reports;
static unsigned                    g_avoidance_reports;
static unsigned                    g_scan_reports;
static telemetry_codec_avoidance_t g_last_avoidance;
static telemetry_codec_scan_t      g_last_scan;

static ER g_scan_result     = E_OK;
static ER g_obstacle_result = E_OK;

/* Raw readings kept by Buddy 5 for the last scan (car/telemetry/scan_sample).
 */
static profile_obstacle_sample_t g_kept[S5_SAMPLES_MAX];
static unsigned                  g_kept_count;
static unsigned                  g_sample_reports;
static unsigned                  g_busy_reads;
static uint16_t                  g_kept_id;
static bool                      gb_has_kept_peek;
static ER                        g_sample_result   = E_OK;
static BOOL                      gb_is_outbox_idle = TRUE;

static telemetry_codec_scan_sample_t g_sent_samples[S5_SAMPLES_MAX + 4U];

static void cycles (unsigned count);
static int  baseline (void);

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

/** @brief Copy the fixture motion status and return the injected read
 * result.
 */
bool motion_get_status (motion_status_t * p_out)
{
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

/** @brief Accept the report without recording or transmitting it in this
 * fixture.
 */
ER buddy1_report_pose (UH distance_cm, H deg)
{
    (void)distance_cm;
    (void)deg;
    return E_OK;
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

/** @brief Count an invalidated obstacle or motion topic. Other topic IDs
 * fail the fixture assertion.
 */
ER buddy1_invalidate (UB type)
{
    if (BUDDY1_TOPIC_OBSTACLE == (int)type)
    {
        g_obstacle_invalidations++;
        return E_OK;
    }

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

/** @brief Copy the retained obstacle reading and return its configured
 * availability.
 */
bool s5_api_snapshot (s5_codec_observation_t * p_out)
{
    *p_out = g_obstacle;
    return gb_is_available;
}

/** @brief Return the last simulated obstacle-control status word.
 */
uint16_t s5_api_controller_status (void)
{
    return g_control;
}

/** @brief Return the current simulated servo angle in degrees.
 */
uint8_t s5_api_servo_angle (void)
{
    return g_servo;
}

/** @brief Check the avoidance state, route, pass and range encodings before
 * retaining the reported sample.
 */
ER buddy1_report_avoidance (const telemetry_codec_avoidance_t * p_first)
{
    assert(p_first && p_first->state <= BUDDY1_S5_STATE_MAX &&
           (int)p_first->route <= BUDDY1_ROUTE_RIGHT &&
           p_first->pass <= BUDDY1_S5_PASS_MAX);
    assert(p_first->range_valid
               ? (p_first->range_mm >= 30U && p_first->range_mm <= 4000U &&
                  !p_first->no_echo)
               : 0U == p_first->range_mm);
    g_avoidance_reports++;
    g_last_avoidance = *p_first;
    return E_OK;
}

/** @brief Count the attempted scan publication, retain a copy and return the
 * injected result.
 */
ER buddy1_report_scan (const telemetry_codec_scan_t * p_scan)
{
    assert(p_scan);
    g_scan_reports++;
    g_last_scan = *p_scan;
    return g_scan_result;
}

/** @brief Return the outbox-idle condition selected by the test.
 */
BOOL buddy1_outbox_idle (void)
{
    return gb_is_outbox_idle;
}

/** @brief Return false during injected lock contention. Otherwise copy the
 * retained scan identity, peek flag, count and samples.
 */
bool s5_api_scan_samples (uint16_t * p_id, bool * pb_is_peek,
                          profile_obstacle_sample_t out[S5_SAMPLES_MAX],
                          unsigned *                p_count)
{
    if (g_busy_reads)
    {
        g_busy_reads--;
        return false;
    }

    *p_id       = g_kept_id;
    *pb_is_peek = gb_has_kept_peek;
    *p_count    = g_kept_count;

    for (unsigned sample_index = 0; sample_index < g_kept_count; sample_index++)
    {
        out[sample_index] = g_kept[sample_index];
    }

    return true;
}

/** @brief Validate the sample index and count. Inject a pending publication
 * error or append the sample to the fixture history.
 */
ER buddy1_report_scan_sample (const telemetry_codec_scan_sample_t * p_sample)
{
    assert(p_sample && p_sample->index < p_sample->count &&
           p_sample->count <= BUDDY1_SCAN_SAMPLES_MAX);

    if (g_sample_result != E_OK)
    {
        ER result       = g_sample_result;
        g_sample_result = E_OK;
        return result;
    }

    assert(g_sample_reports < sizeof g_sent_samples / sizeof g_sent_samples[0]);
    g_sent_samples[g_sample_reports++] = *p_sample;
    return E_OK;
}

/** @brief Check the fixture distance, width and route, count the attempt and
 * return the injected obstacle publication result.
 */
ER buddy1_report_obstacle (UH distance, UH width, telemetry_codec_route_t route)
{
    assert(250 == (int)distance && 400 == (int)width &&
           route <= (unsigned)BUDDY1_ROUTE_RIGHT);
    g_obstacle_reports++;
    return g_obstacle_result;
}

/** @brief Publish obstacle and scan snapshots through the telemetry adapter
 * and check update cadence and complete scan profiles.
 * @details Feed controller observations through the real vehicle adapter.
 * Verify which topic and packet are sent as a scan becomes complete, and
 * exercise retry behavior when the fake network queue is full. The
 * fixture checks the wire fields through the telemetry decoder.
 */
int main (void)
{
    baseline();
    /* clang-format off */
    gb_is_available          = true;
    g_obstacle.b_has_no_echo = true;
    g_control                =
        (unsigned)S5_SCANNING | ((unsigned)S5_ROUTE_LEFT << 6U);
    g_runtime                =
        (s3_integration_status_t){.state = S3_DRIVE_MANOEUVRE};
    g_status                 = (motion_status_t){
         .fault_bits = motion_pack_fault_bit(MOTION_FAULT_STALL),
    };
    g_mask                   = BUDDY1_FAULT_WIFI;
    /* clang-format on */
    cycles(1);
    assert(g_mask ==
           (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL | BUDDY1_FAULT_NO_ECHO));
    assert((unsigned)BUDDY1_STATE_AVOIDING_OBSTACLE == g_last_state);
    g_obstacle.b_has_no_echo  = false;
    g_obstacle.scan_id        = 1;
    g_obstacle.b_is_scan_done = true;
    g_obstacle.profile        = (profile_obstacle_t){
               .b_is_valid = true, .closest_mm = 250, .width_mm = 400};
    cycles(1);
    assert(g_mask == (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL) &&
           1U == g_obstacle_reports);
    cycles(1);
    assert(1U == g_obstacle_reports);
    g_obstacle.profile.b_is_valid = false;
    {
        unsigned before = g_obstacle_invalidations;
        cycles(100);
        assert(g_obstacle_invalidations == before + 1U);
    }
    g_runtime.state = S3_DRIVE_FAULT;
    cycles(1);
    assert((unsigned)BUDDY1_STATE_ERROR == g_last_state);
    /* Live Buddy 5 status: every 200 ms, all controller fields unpacked. */
    {
        unsigned before = g_avoidance_reports;
        /* clang-format off */
        g_control                   =
            (uint16_t)((unsigned)S5_MOVING | ((unsigned)S5_OK << 3U) |
                               ((unsigned)S5_ROUTE_RIGHT << 6U) | (3U << 8) |
                               ((unsigned)S5_PASS_STEP << 12U) | (1U << 15));
        g_obstacle.b_is_range_valid = true;
        g_obstacle.b_has_no_echo    = false;
        g_obstacle.range_mm         = 412;
        g_servo                     = 150;
        /* clang-format on */
        cycles(2U * LIVE_CYCLES);
        assert(g_avoidance_reports == before + 2U);
        assert(g_last_avoidance.range_valid &&
               412U == g_last_avoidance.range_mm &&
               150U == g_last_avoidance.servo_deg);
        assert(S5_MOVING == (int)g_last_avoidance.state &&
               BUDDY1_ROUTE_RIGHT == (int)g_last_avoidance.route &&
               3U == g_last_avoidance.leg &&
               S5_PASS_STEP == (int)g_last_avoidance.pass &&
               g_last_avoidance.swapped);
        g_obstacle.b_is_range_valid = false;
        g_obstacle.range_mm         = 0;
        g_obstacle.b_has_no_echo    = true;
        cycles(LIVE_CYCLES);
        assert(!g_last_avoidance.range_valid &&
               0U == g_last_avoidance.range_mm && g_last_avoidance.no_echo);
        gb_is_available = false;
        cycles(LIVE_CYCLES);
        assert(!g_last_avoidance.range_valid && !g_last_avoidance.no_echo);
        gb_is_available          = true;
        g_obstacle.b_has_no_echo = false;
    }
    /* Full FR6 profile once per finished scan / side peek, incl. a failed one.
     */
    {
        unsigned before           = g_scan_reports;
        g_obstacle.scan_id        = 7;
        g_obstacle.b_is_scan_done = true;
        g_obstacle.profile        = (profile_obstacle_t){.b_is_valid     = true,
                                                         .closest_mm     = 250,
                                                         .width_mm       = 400,
                                                         .left_clear_mm  = 180,
                                                         .right_clear_mm = 320,
                                                         .left_edge_mm   = 150,
                                                         .right_edge_mm  = -250,
                                                         .forward_mm     = 380};
        cycles(5);
        assert(g_scan_reports == before + 1U);
        assert(g_last_scan.valid && 7U == g_last_scan.scan_id &&
               250U == g_last_scan.closest_mm && 400U == g_last_scan.width_mm &&
               180U == g_last_scan.left_clear_mm &&
               320U == g_last_scan.right_clear_mm &&
               150 == g_last_scan.left_edge_mm &&
               g_last_scan.right_edge_mm == -250 &&
               380U == g_last_scan.forward_mm);
        g_obstacle.scan_id            = 8;
        g_obstacle.profile.b_is_valid = false;
        cycles(1);
        assert(g_scan_reports == before + 2U && !g_last_scan.valid &&
               8U == g_last_scan.scan_id && 0U == g_last_scan.closest_mm);
    }
    /* Raw readings follow the profile one by one, 1 per 4 cycles, only
     * while the outbox is empty; a full outbox retries the same reading; a
     * busy Buddy 5 mutex is retried; another scan id is never reused. */
    {
        assert(profile_obstacle_sample_pack(90, 250, true, &g_kept[0]) &&
               profile_obstacle_sample_pack(95, 262, true, &g_kept[1]) &&
               profile_obstacle_sample_pack(100, 0, false, &g_kept[2]));
        g_kept_count                  = 3;
        g_kept_id                     = 9;
        gb_has_kept_peek              = false;
        g_busy_reads                  = 2;
        g_obstacle.scan_id            = 9;
        g_obstacle.b_is_scan_done     = true;
        g_obstacle.profile.b_is_valid = true;
        cycles(1);
        assert(9U == g_last_scan.scan_id && 0U == g_sample_reports);
        cycles(2U * SC);
        assert(0U == g_sample_reports); /* Inject two busy Buddy 5 reads. */
        gb_is_outbox_idle = FALSE;
        cycles(10U * SC);
        assert(0U == g_sample_reports); /* Wait behind queued events. */
        gb_is_outbox_idle = TRUE;
        cycles(SC);
        assert(1U == g_sample_reports);
        g_sample_result = E_TMOUT;
        cycles(SC);
        assert(1U == g_sample_reports); /* Fill the outbox. */
        cycles(2U * SC);
        assert(3U == g_sample_reports);
        cycles(10U * SC);
        assert(3U == g_sample_reports);

        for (unsigned sample_index = 0;
             sample_index < REPORTED_SCAN_SAMPLE_COUNT; sample_index++)
        {
            assert(9U == g_sent_samples[sample_index].scan_id &&
                   !g_sent_samples[sample_index].peek &&
                   g_sent_samples[sample_index].index == sample_index &&
                   3U == g_sent_samples[sample_index].count);
        }

        assert(90U == g_sent_samples[0].angle_deg && g_sent_samples[0].valid &&
               250U == g_sent_samples[0].distance_mm);
        assert(95U == g_sent_samples[1].angle_deg &&
               262U == g_sent_samples[1].distance_mm);
        assert(100U == g_sent_samples[2].angle_deg &&
               !g_sent_samples[2].valid && 0U == g_sent_samples[2].distance_mm);
        g_kept_id          = 9;
        g_obstacle.scan_id = 10;
        cycles(10U * SC);
        assert(10U == g_last_scan.scan_id && 3U == g_sample_reports);
        assert(profile_obstacle_sample_pack(135, 410, true, &g_kept[0]));
        g_kept_count       = 1;
        g_kept_id          = 11;
        gb_has_kept_peek   = true;
        g_obstacle.scan_id = 11;
        cycles(2U * SC);
        assert(4U == g_sample_reports && g_sent_samples[3].peek &&
               1U == g_sent_samples[3].count &&
               135U == g_sent_samples[3].angle_deg &&
               410U == g_sent_samples[3].distance_mm);
    }
    /* Full outbox: the scan profile and obstacle summary are retried every
     * cycle until queued (then exactly once more nothing). */
    {
        unsigned reports_before_retry   = g_scan_reports;
        unsigned obstacles_before_retry = g_obstacle_reports;
        g_scan_result = g_obstacle_result = E_TMOUT;

        g_obstacle.scan_id        = 12;
        g_obstacle.b_is_scan_done = true;
        g_obstacle.profile        = (profile_obstacle_t){
                   .b_is_valid = true, .closest_mm = 250, .width_mm = 400};
        cycles(3);
        assert(g_scan_reports == reports_before_retry + 3U &&
               g_obstacle_reports == obstacles_before_retry + 3U);
        g_scan_result = g_obstacle_result = E_OK;
        cycles(1);
        assert(g_scan_reports == reports_before_retry + 4U &&
               g_obstacle_reports == obstacles_before_retry + 4U);
        cycles(5);
        assert(g_scan_reports == reports_before_retry + 4U &&
               g_obstacle_reports == obstacles_before_retry + 4U);
    }
    /* Quarter-period initial phase, then exactly 200 ms between reports. */
    assert(vehicle_controller_init() == E_OK);
    {
        unsigned before  = g_avoidance_reports;
        uint32_t started = g_runtime.navigation.timestamp_us;
        cycles(LIVE_CYCLES - (LIVE_CYCLES / 4U) - 1U);
        assert(g_avoidance_reports == before);
        cycles(1);
        assert(g_avoidance_reports == before + 1U);
        assert(g_runtime.navigation.timestamp_us - started == 150000U);
        started = g_runtime.navigation.timestamp_us;
        cycles(LIVE_CYCLES / 2U);
        assert(g_avoidance_reports == before + 1U);
        assert(g_runtime.navigation.timestamp_us - started == 100000U);
        cycles(LIVE_CYCLES - (LIVE_CYCLES / 2U) - 1U);
        assert(g_avoidance_reports == before + 1U);
        cycles(1);
        assert(g_avoidance_reports == before + 2U);
        assert(g_runtime.navigation.timestamp_us - started == 200000U);
    }
    puts("PASS: production telemetry adapter preserves Wi-Fi/motor faults, "
         "owns only no-echo, publishes obstacle profile and avoidance/error "
         "state");
    puts("PASS: barcode, state, fault, scan-profile and obstacle events "
         "retried while the Buddy 1 outbox is full, sent once when space "
         "frees");
    puts("PASS: Buddy 5 raw scan readings after each profile, spare outbox "
         "capacity only, paced, retried when the outbox is full or Buddy 5 "
         "is busy, never under another scan id");
    puts("PASS: Buddy 5 live avoidance (range, no-echo, servo angle, "
         "state/fault/route/leg/pass/swap at 5 Hz) and full FR6 scan "
         "profile per scan");
}

/** @brief Feed one controller observation per sample. The sample clock paces
 * motion telemetry at 200 ms while moving and one second while
 * stopped.
 */
static void cycles (unsigned count)
{
    for (unsigned tick_index = 0; tick_index < count; tick_index++)
    {
        g_runtime.navigation.timestamp_us += S3_SAMPLE_PERIOD_US;
        vehicle_controller_observe(&g_runtime);
    }
}

/** @brief Exercise controller startup, mapping, motion telemetry and
 * invalidation before the obstacle wire checks.
 */
static int baseline (void)
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
    assert(1U == g_reports && 120 == g_left_report && g_right_report == -120);
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
    assert(1U == g_barcodes); /* Send one report per event. */
    /* Full outbox: barcode and state are retried every cycle, not dropped. */
    g_event_result              = E_TMOUT;
    g_runtime.navigation.events = S3_EVENT_BARCODE;
    cycles(1);
    g_runtime.navigation.events = 0;
    cycles(2);
    assert(4U == g_barcodes);
    g_runtime.state = S3_DRIVE_MANOEUVRE;
    {
        unsigned reports_before_retry = g_states;
        cycles(3);
        assert(g_states == reports_before_retry + 3U && 7U == g_barcodes &&
               g_last_state != (unsigned)BUDDY1_STATE_TURNING_LEFT);
    }
    g_event_result = E_OK;
    cycles(1);
    assert(8U == g_barcodes &&
           (unsigned)BUDDY1_STATE_TURNING_LEFT == g_last_state);
    {
        unsigned reports_before_retry = g_states;
        cycles(3);
        assert(8U == g_barcodes && g_states == reports_before_retry);
    }
    assert((unsigned)BUDDY1_STATE_TURNING_LEFT == g_last_state);
    g_runtime.state = S3_DRIVE_FAULT;
    cycles(1);
    assert((unsigned)BUDDY1_STATE_ERROR == g_last_state);
    g_status.speed_left_mps = 0.884f;
    before                  = g_invalidations;
    {
        /* Never clip into a valid 510. */
        unsigned sent = g_reports;
        cycles(LIVE_CYCLES);
        assert(g_invalidations > before && g_reports == sent);
    }
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
    puts("PASS: one-owner facade, explicit queued start/stop, full fault-bit "
         "ownership, unknown-status preservation, map freeze, telemetry and "
         "out-of-range invalidation");
    return 0;
}

/* End of file. */

