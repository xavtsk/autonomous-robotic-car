/** @file vehicle_controller.c
 * @brief Route operator requests and adapt observations for the shared
 * controller owner.
 * @see app_program/controller/README.md for the facade and telemetry paths.
 * Mask constants here are nonnegative. Converting to unsigned preserves
 * each mask while making bitwise operations unsigned.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 * Byte and half-word values fit in int, and uint32_t fits in int64_t.
 * The casts make the existing arithmetic promotions explicit.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 * Unsigned casts spell out the original modulo conversion.
 * Byte and flag masks are widened unsigned before bitwise operations.
 */
/* The integrated sampling task serializes navigation, motion requests and
 * report consumption. This adapter exposes that owner to operator controls
 * and telemetry without adding another task. */
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
typedef struct
{
    uint32_t         last_us;
    s3_drive_state_t state;
    unsigned         b_has_sample : 1;
    unsigned         b_is_usable : 1;
    unsigned         b_is_stopped : 1;
} vehicle_telemetry_t;

/* Route pose (car/telemetry/pose): Buddy 2's encoder distance/heading relative
 * to the start of the current run. Only the sampling owner touches it. */
#define POSE_PERIOD_CYCLES LIVE_PERIOD_CYCLES

typedef struct
{
    unsigned  b_is_running : 1;
    unsigned  divider : 6;
    float32_t start_distance_m;
    float32_t start_heading_deg;
} vehicle_pose_t;

#if APP_S5
typedef struct
{
    uint16_t scan;
    unsigned route : 2;
    unsigned b_has_invalidated : 1;
    unsigned echo : 2;     /* 2 unknown, 0 healthy, 1 no echo. */
    uint16_t profile_scan; /* Keep the last published scan ID. */
    unsigned b_has_sent_profile : 1;
    unsigned divider : 6; /* Publish avoidance every 200 ms. */
} obstacle_report_t;

/* Raw readings of the scan whose profile was just sent, at most one message
 * every SAMPLE_PERIOD cycles (car/telemetry/scan_sample), and only while the
 * 4-slot outbox is empty: spare capacity only, so barcode/state/fault events
 * and command replies never find it full because of these readings. */
#define SAMPLE_PERIOD                                                          \
    (40U / S3_SAMPLE_PERIOD_MS) /* Space scan samples by 40 ms. */
#if SAMPLE_PERIOD == 0U || SAMPLE_PERIOD > 15U
#error "scan-sample pacing must fit its four-bit divider"
#endif
typedef struct
{
    profile_obstacle_sample_t sample[S5_SAMPLES_MAX];
    uint16_t                  scan_id;
    unsigned                  count : 5;
    unsigned                  next : 5;
    unsigned                  b_is_peek : 1;
    unsigned                  b_is_pending : 1;
    unsigned                  b_is_loaded : 1;
    unsigned                  divider : 4;
} scan_publication_t;

/* Everything Buddy 5 knows, for the laptop: live range, servo angle and
 * bypass progress at 5 Hz (phase-shifted from the other 200 ms reports so the
 * four-message outbox is not filled in one cycle), and the full FR6 profile
 * once per finished scan or side peek. Unusable values are sent as zero with
 * their validity cleared, never as a plausible number. */
#define S5_LIVE_PERIOD LIVE_PERIOD_CYCLES

#endif

#if APP_S5
static inline void
report_obstacle_route (const s5_codec_observation_t * p_obstacle,
                       unsigned                       route);
#endif

static vehicle_telemetry_t g_telemetry;

static vehicle_pose_t g_pose;

static telemetry_codec_state_t  g_last_state;
static telemetry_codec_action_t g_last_action;
/* Events (barcode, state, obstacle summary, scan profile, fault changes) are
 * retried every cycle while Buddy 1's 4-slot outbox is full (E_TMOUT), so a
 * busy link delays them instead of dropping them. Periodic live samples
 * (motion, pose, terrain, avoidance) stay best effort: the next one follows. */
static UB       g_pending_barcode; /* Use zero when no letter is queued. */
static unsigned g_last_faults;
static bool     gb_has_fault_snapshot;

#if APP_S5
static obstacle_report_t  g_obstacle_report;
static scan_publication_t g_scan_samples;
#endif

/* True only between a prepare_start() that newly froze the Buddy 1 map and
 * the end of that same START attempt. Only the sampling owner touches it. */
static bool gb_has_start_claimed_map;

#if APP_S5
/** @brief At the configured reporting cadence, publish forward range, servo
 * angle and bypass status. Suppress invalid range values and retain
 * their validity indication. A full outbox can drop a live sample.
 */
static void obstacle_live_report (const s5_codec_observation_t * p_observation,
                                  bool b_has_observation, unsigned control)
{
    telemetry_codec_avoidance_t telemetry = {0};

    if (++g_obstacle_report.divider < S5_LIVE_PERIOD)
    {
        return;
    }

    g_obstacle_report.divider = 0U;

    if (b_has_observation)
    {
        telemetry.no_echo = p_observation->b_has_no_echo ? 1U : 0U;

        if ((((p_observation->b_is_range_valid) &&
              (!p_observation->b_has_no_echo)) &&
             (p_observation->range_mm >= BUDDY1_OBSTACLE_MIN_MM)) &&
            (p_observation->range_mm <= BUDDY1_OBSTACLE_MAX_MM))
        {
            telemetry.range_valid = 1U;
            telemetry.range_mm    = p_observation->range_mm;
        }
    }

    telemetry.servo_deg = s5_api_servo_angle();

    if (telemetry.servo_deg > BUDDY1_SERVO_MAX_DEG)
    {
        telemetry.servo_deg = 0U;
    }

    /* Each mask limits its field to four bits or fewer before narrowing. */
    telemetry.state   = (uint8_t)(control & 7U);
    telemetry.fault   = (uint8_t)((control >> 3) & 7U);
    telemetry.route   = (uint8_t)((control >> 6) & 3U);
    telemetry.leg     = (uint8_t)((control >> 8) & 15U);
    telemetry.pass    = (uint8_t)((control >> 12) & 7U);
    telemetry.swapped = (uint8_t)((control >> 15) & 1U);

    if (telemetry.state > BUDDY1_S5_STATE_MAX)
    {
        telemetry.state = 0U;
    }

    if ((int)telemetry.route > BUDDY1_ROUTE_RIGHT)
    {
        telemetry.route = 0U;
    }

    if (telemetry.pass > BUDDY1_S5_PASS_MAX)
    {
        telemetry.pass = 0U;
    }

    (void)buddy1_report_avoidance(
        &telemetry); /* The next QoS 0 report follows on schedule. */
}

/** @brief Prepare telemetry for a completed obstacle scan using its run-time
 * profile, clearance and scan ID. The sampling owner supplies the
 * observation.
 */
static void
obstacle_profile_report (const s5_codec_observation_t * p_observation)
{
    const profile_obstacle_t * p_profile = &p_observation->profile;
    telemetry_codec_scan_t     report    = {0};

    if ((!p_observation->b_is_scan_done) ||
        ((g_obstacle_report.b_has_sent_profile) &&
         (g_obstacle_report.profile_scan == p_observation->scan_id)))
    {
        return;
    }

    /* The report ID is the low octet. Keep the full observed scan ID below
     * for duplicate suppression and sample matching. */
    report.scan_id = (uint8_t)(p_observation->scan_id & 0xFFU);

    if (p_profile->b_is_valid)
    {
        report.valid             = 1U;
        report.closest_mm        = p_profile->closest_mm;
        report.width_mm          = p_profile->width_mm;
        report.left_clear_mm     = p_profile->left_clear_mm;
        report.right_clear_mm    = p_profile->right_clear_mm;
        report.forward_mm        = p_profile->forward_mm;
        report.left_edge_mm      = p_profile->left_edge_mm;
        report.right_edge_mm     = p_profile->right_edge_mm;
        report.face_width_min_mm = p_profile->face_width_min_mm;
        report.face_width_max_mm = p_profile->face_width_max_mm;
    }

    ER value_er = buddy1_report_scan(&report);

    if ((E_PAR == value_er) && (report.valid))
    {
        /* An invalid report retains only the same low-octet scan ID. */
        report         = (telemetry_codec_scan_t){0};
        report.scan_id = (uint8_t)(p_observation->scan_id & 0xFFU);
        value_er       = buddy1_report_scan(&report);
    }

    if ((E_OK == value_er) || (E_PAR == value_er))
    {
        /* A full outbox leaves this profile pending for a later report. */
        g_obstacle_report.profile_scan       = p_observation->scan_id;
        g_obstacle_report.b_has_sent_profile = 1U;
        g_scan_samples.scan_id               = p_observation->scan_id;
        g_scan_samples.b_is_pending          = 1U;
        g_scan_samples.b_is_loaded           = 0U;
        g_scan_samples.divider               = 0U;
    }
}

/** @brief Advance the owner's pending scan-sample upload at its bounded
 * cadence. Samples retain their scan ID and servo angle for the
 * laptop.
 */
static void obstacle_samples_report (void)
{
    telemetry_codec_scan_sample_t report = {0};
    unsigned                      angle;
    unsigned                      value_mm;
    unsigned                      count;
    bool                          b_is_valid;
    bool                          b_is_peek;
    uint16_t                      value_id;
    ER                            value_er;

    if ((!g_scan_samples.b_is_pending) ||
        (++g_scan_samples.divider < SAMPLE_PERIOD))
    {
        return;
    }

    g_scan_samples.divider = 0U;

    if (!g_scan_samples.b_is_loaded)
    {
        if (!s5_api_scan_samples(&value_id, &b_is_peek, g_scan_samples.sample,
                                 &count))
        {
            return; /* Retry when the snapshot is no longer busy. */
        }

        if (((value_id != g_scan_samples.scan_id) || (0U == count)) ||
            (count > BUDDY1_SCAN_SAMPLES_MAX))
        {
            g_scan_samples.b_is_pending = 0U;
            return; /* The scan retained no readings. */
        }

        g_scan_samples.count       = count;
        g_scan_samples.next        = 0U;
        g_scan_samples.b_is_peek   = b_is_peek ? 1U : 0U;
        g_scan_samples.b_is_loaded = 1U;
    }

    if (!buddy1_outbox_idle())
    {
        return; /* Keep this reading and retry when the link frees up. */
    }

    if (!profile_obstacle_sample_unpack(
            &g_scan_samples.sample[g_scan_samples.next], &angle, &value_mm,
            &b_is_valid))
    {
        value_er = E_PAR;
    }
    else
    {
        /* The narrow report ID is the low octet. Count is at most 16 and
         * the sample index is smaller. A successful unpack gives an angle
         * of 30..150 degrees and a valid distance of 30..4000 mm. */
        report.scan_id     = (uint8_t)(g_scan_samples.scan_id & 0xFFU);
        report.peek        = (uint8_t)g_scan_samples.b_is_peek;
        report.index       = (uint8_t)g_scan_samples.next;
        report.count       = (uint8_t)g_scan_samples.count;
        report.angle_deg   = (uint8_t)angle;
        report.valid       = b_is_valid ? 1U : 0U;
        report.distance_mm = b_is_valid ? (uint16_t)value_mm : 0U;
        value_er           = buddy1_report_scan_sample(&report);
    }

    if (E_TMOUT == value_er)
    {
        return; /* Retry this reading after the outbox frees up. */
    }

    if (++g_scan_samples.next >= g_scan_samples.count)
    {
        g_scan_samples.b_is_pending = 0U;
    }
}
#endif
/** @brief Translate a navigation action into its Buddy 1 barcode action.
 * Unknown actions return the wire-level UNKNOWN value.
 */
static telemetry_codec_action_t action_for (line_navigation_action_t action)
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

/** @brief Map drive phases to the operator-facing state. Braking, manoeuvring
 * and reacquisition all report TURNING.
 */
static vehicle_controller_state_t public_state (s3_drive_state_t state)
{
    switch (state)
    {
    case S3_DRIVE_FOLLOW:
        return VC_LINE_FOLLOW;
    case S3_DRIVE_BRAKING: /* All three phases are reported as turning. */
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

/** @brief Map drive state and the last barcode action to the telemetry state.
 * The last action distinguishes left, right and U-turn during
 * manoeuvres.
 */
static telemetry_codec_state_t wire_state (s3_drive_state_t state)
{
    if ((unsigned int)S3_DRIVE_FAULT == state)
    {
        return BUDDY1_STATE_ERROR;
    }

    if (((unsigned int)S3_DRIVE_OFF == state) ||
        ((unsigned int)S3_DRIVE_HALTED == state))
    {
        return BUDDY1_STATE_STOPPED;
    }

    if ((unsigned int)S3_DRIVE_MANOEUVRE == state)
    {
        if ((unsigned int)BUDDY1_ACTION_LEFT == g_last_action)
        {
            return BUDDY1_STATE_TURNING_LEFT;
        }

        if ((unsigned int)BUDDY1_ACTION_RIGHT == g_last_action)
        {
            return BUDDY1_STATE_TURNING_RIGHT;
        }

        if ((unsigned int)BUDDY1_ACTION_UTURN == g_last_action)
        {
            return BUDDY1_STATE_UTURN;
        }
    }

    return BUDDY1_STATE_LINE_FOLLOWING;
}

/** @brief Return true during following, braking, manoeuvring or line
 * reacquisition.
 */
static bool run_active (s3_drive_state_t state)
{
    return ((((unsigned int)S3_DRIVE_FOLLOW == state) ||
             ((unsigned int)S3_DRIVE_BRAKING == state)) ||
            ((unsigned int)S3_DRIVE_MANOEUVRE == state)) ||
           ((unsigned int)S3_DRIVE_REACQUIRE == state);
}

/* Repeating zero-speed packets need only a 1 Hz heartbeat while stopped (the
 * dashboard's motion freshness window is 3 s). Keep 5 Hz when moving or the
 * estimate is unavailable, and report transitions on the next observation. */
/** @brief Publish representable wheel speeds in signed mm/s, or invalidate the
 * motion section. Rate-limit steady readings and use a slower
 * heartbeat for confirmed stopped wheels.
 */
static void motion_report (const s3_integration_status_t * p_status,
                           const motion_status_t *         p_motion,
                           bool                            b_is_available)
{
    const uint32_t now   = p_status->navigation.timestamp_us;
    float32_t      left  = 0.0f;
    float32_t      right = 0.0f;
    bool b_is_usable     = (b_is_available && (p_motion->b_is_status_valid)) &&
                       (!p_motion->b_is_busy);

    if (b_is_usable)
    {
        left  = p_motion->speed_left_mps * 1000.0f;
        right = p_motion->speed_right_mps * 1000.0f;
        /* Keep the agreed wheel range unchanged. Calibrated top speed exceeds
         * +/-510 mm/s: mark unavailable, NEVER clip or widen the wire. */
        b_is_usable = (((((left == left) && (right == right)) &&
                         (left >= -BUDDY1_SPEED_MAX_MMS)) &&
                        (left <= BUDDY1_SPEED_MAX_MMS)) &&
                       (right >= -BUDDY1_SPEED_MAX_MMS)) &&
                      (right <= BUDDY1_SPEED_MAX_MMS);
    }
    /* The estimator's smoothing can leave tiny nonzero floats after braking.
     * Only slow down when the brake/still acknowledgement agrees and both
     * integer-mm/s values already report zero. Representable wheel movement
     * restores 5 Hz even when the drive state says HALTED or FAULT.
     * Both H casts are gated by the finite +/-510 mm/s range above. */
    const bool b_is_stopped =
        (((b_is_usable && (!run_active(p_status->state))) &&
          (p_motion->b_is_stopped)) &&
         (0 == (H)left)) &&
        (0 == (H)right);
    const uint32_t period =
        b_is_stopped ? STOPPED_MOTION_PERIOD_US : MOTION_PERIOD_US;

    if (((((g_telemetry.b_has_sample) &&
           (g_telemetry.state == p_status->state)) &&
          (g_telemetry.b_is_usable == b_is_usable)) &&
         (g_telemetry.b_is_stopped == b_is_stopped)) &&
        ((now - g_telemetry.last_us) < period))
    {
        return;
    }

    g_telemetry.b_has_sample = true;
    g_telemetry.state        = p_status->state;
    g_telemetry.b_is_usable  = b_is_usable;
    g_telemetry.b_is_stopped = b_is_stopped;
    g_telemetry.last_us      = now;

    if (b_is_usable)
    {
        /* Truncate each validated speed within +/-510 mm/s to H. */
        (void)buddy1_report_motion((H)left, (H)right);
    }
    else
    {
        (void)buddy1_invalidate(BUDDY1_TOPIC_MOTION);
    }
}

/** @brief Round a bounded, representable floating-point value to int32_t with
 * half values rounded away from zero.
 */
static int32_t round_to_int (float32_t operand)
{
    return (int32_t)(operand >= 0.0f ? operand + 0.5f : operand - 0.5f);
}

/** @brief Report encoder distance in centimetres modulo the wire range and
 * heading in degrees modulo 360, relative to the run origin.
 * @details Signed 20-bit encoder counts and the configured 2534 counts per
 * revolution, 65.4 mm wheel diameter and 116 mm track bound a single reading
 * to less than 43 m and 43000 degrees in magnitude. Both run-relative
 * differences therefore fit the int32_t conversions before modulo wrapping.
 * @see app_program/subsystem2/DATA_FORMAT.md for the encoder and pose units.
 */
static void send_pose (const motion_status_t * p_motion)
{
    int32_t value_cm =
        round_to_int((p_motion->distance_m - g_pose.start_distance_m) * 100.0f);
    int32_t deg =
        round_to_int(p_motion->heading_deg - g_pose.start_heading_deg);
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
    /* Modulo normalization gives 0..4095 cm and 0..359 degrees, which fit
     * UH and H. QoS 0 can drop this sample when the outbox is full. The next
     * pose carries current data and the dashboard unwraps distance. */
    (void)buddy1_report_pose((UH)value_cm, (H)deg);
}

/* One pose at START (0, 0), then every 200 ms while the run is active, and a
 * final one when it ends. Skipped while Buddy 2's estimate is unusable. */
/** @brief Track the start origin and publish pose at the configured cadence
 * and at the end of a run when estimates are usable.
 */
static void pose_report (const s3_integration_status_t * p_status,
                         const motion_status_t * p_motion, bool b_is_available)
{
    const bool b_is_active = run_active(p_status->state);
    const bool b_is_usable =
        (((b_is_available && (p_motion->b_is_status_valid)) &&
          (!p_motion->b_is_busy)) &&
         (p_motion->distance_m == p_motion->distance_m)) &&
        (p_motion->heading_deg == p_motion->heading_deg);

    if (b_is_active && (!g_pose.b_is_running))
    {
        if (!b_is_usable)
        {
            return; /* Set the reference from a valid estimate. */
        }

        g_pose.b_is_running      = true;
        g_pose.divider           = 0U;
        g_pose.start_distance_m  = p_motion->distance_m;
        g_pose.start_heading_deg = p_motion->heading_deg;
        send_pose(p_motion);
    }
    else if (b_is_active)
    {
        const bool b_is_pose_due = (++g_pose.divider >= POSE_PERIOD_CYCLES);

        if (b_is_pose_due)
        {
            g_pose.divider = 0U;
        }

        if (b_is_pose_due && b_is_usable)
        {
            send_pose(p_motion);
        }
    }
    else if (g_pose.b_is_running)
    {
        g_pose.b_is_running = false;

        if (b_is_usable)
        {
            send_pose(p_motion);
        }
    }
    else
    {
        /* A stopped pose needs no running update. */
    }
}

/** @brief Reset the controller's telemetry history, map claim and terrain
 * adapter state. Return E_OK.
 */
ER vehicle_controller_init (void)
{
    terr_link_init();
    g_pose.b_is_running      = false;
    g_pose.divider           = 0U;
    g_telemetry.b_has_sample = false;
    g_last_state             = BUDDY1_STATE_UNKNOWN;
    g_last_action            = BUDDY1_ACTION_UNKNOWN;
    g_last_faults            = 0U;
    gb_has_fault_snapshot    = false;
    g_pending_barcode        = 0U;
    gb_has_start_claimed_map = false;
#if APP_S5
    g_obstacle_report.scan               = 0;
    g_obstacle_report.route              = 0;
    g_obstacle_report.echo               = 2U;
    g_obstacle_report.b_has_invalidated  = false;
    g_obstacle_report.profile_scan       = 0U;
    g_obstacle_report.b_has_sent_profile = 0U;
    g_obstacle_report.divider            = S5_LIVE_PERIOD / 4U;
    g_scan_samples.b_is_pending          = 0U;
    g_scan_samples.b_is_loaded           = 0U;
#endif
    return E_OK;
}

/** @brief Return E_OK. The integrated sampling task owns the controller cycle.
 */
ER vehicle_controller_start (void)
{
    return E_OK;
}

/** @brief Submit a START request to the sampling owner. Return E_OK if
 * accepted or E_OBJ otherwise.
 */
ER vehicle_controller_begin (void)
{
    const s3_integration_request_t request = {.kind = S3_CONTROL_START};
    return s3_integration_submit(&request) ? E_OK : E_OBJ;
}

/** @brief Submit STOP to the sampling mailbox and return E_OK on acceptance or
 * E_OBJ on failure.
 */
ER vehicle_controller_request_stop (void)
{
    const s3_integration_request_t request = {.kind = S3_CONTROL_STOP};
    return s3_integration_submit(&request) ? E_OK : E_OBJ;
}

/** @brief Read a task-local integration snapshot and return the public
 * controller state. An unavailable snapshot yields WAITING.
 */
vehicle_controller_state_t vehicle_controller_state (void)
{
    /* Only the normal controller/UI task may use this convenience view.
     * Keep the temporary copy task-local, not a second shared snapshot. */
    s3_integration_status_t copy;
    return s3_integration_status(&copy) ? public_state(copy.state)
                                        : (unsigned)VC_WAITING;
}

/* Compatibility boundary: new sensor observations are processed in the SAME
 * owner as the upstream drive FSM. Never accept a second external producer. */
/** @brief Reject an external navigation producer with E_NOSPT. The integrated
 * sampling owner already processes observations.
 */
ER vehicle_controller_submit_nav (const line_navigation_output_t * p_output)
{
    (void)p_output;
    return E_NOSPT;
}

/* Called by the sampling/controller owner immediately before an explicit START.
 * Freeze and copy the Buddy 1 map atomically, then update its private nav
 * context. */
/** @brief Atomically claim the active barcode map, expand it for navigation
 * and track whether this START newly froze the map. Return false if
 * preparation fails.
 */
bool vehicle_controller_prep_start (line_navigation_t * p_navigation)
{
    telemetry_codec_map_t    map;
    BOOL                     b_did_close                  = FALSE;
    line_navigation_action_t expanded[S3_BARCODE_LETTERS] = {0};
    gb_has_start_claimed_map                              = false;

    if ((0 == p_navigation) ||
        (buddy1_configuration_claim(&map, &b_did_close) != E_OK))
    {
        return false;
    }

    expanded[(int)map.forward - 'A'] = S3_NAV_STRAIGHT;
    expanded[(int)map.left - 'A']    = S3_NAV_LEFT;
    expanded[(int)map.right - 'A']   = S3_NAV_RIGHT;
    expanded[(int)map.uturn - 'A']   = S3_NAV_UTURN;

    if (!line_navigation_set_mapping(p_navigation, expanded))
    {
        /* Nothing installed: undo only a freeze this attempt took. */
        if (b_did_close)
        {
            (void)buddy1_configuration_reopen();
        }

        return false;
    }

    gb_has_start_claimed_map = b_did_close != (unsigned int)FALSE;
    return true;
}

/** @brief Release a map freeze only when the cancelled START acquired it, then
 * clear the temporary ownership flag.
 */
void vehicle_controller_cancel_start (void)
{
    if (gb_has_start_claimed_map)
    {
        (void)buddy1_configuration_reopen();
    }

    gb_has_start_claimed_map = false;
}

/** @brief Clear temporary claim ownership after START commits. The run keeps
 * its frozen map.
 */
void vehicle_controller_commit_start (void)
{
    gb_has_start_claimed_map = false;
}

/** @brief Reduce a 26-letter navigation map to the four directional slots.
 * Reject unsupported actions or duplicate slots and return whether
 * Buddy 1 accepted the map.
 */
bool vehicle_controller_set_mapping (
    const line_navigation_action_t p_map[S3_BARCODE_LETTERS])
{
    telemetry_codec_map_t reduced = {0};

    if (!p_map)
    {
        return false;
    }

    for (unsigned letter_index = 0U; letter_index < S3_BARCODE_LETTERS;
         letter_index++)
    {
        uint8_t * p_slot = NULL;

        switch (p_map[letter_index])
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

        /* The map loop bounds the index to 0..25. The resulting A..Z
         * code fits the destination byte. */
        *p_slot = (uint8_t)((unsigned int)'A' + letter_index);
    }

    return buddy1_apply_barcode_map(&reduced) == E_OK;
}

/** @brief Mirror the current integration state and sensor observations to
 * telemetry from the sampling owner, outside the snapshot mutex.
 * @details The supplied status is borrowed for this call. A missing status
 * returns immediately. Motion feedback updates the live motion and pose
 * streams, while enabled obstacle data can update range, scans and fault
 * flags. The mirror uses queue results to decide which reports to retry.
 */
void vehicle_controller_observe (const s3_integration_status_t * p_status)
{
    motion_status_t motion;
    unsigned        active =
        g_last_faults & (BUDDY1_FAULT_STALL | BUDDY1_FAULT_ENCODER);
    unsigned                changed = BUDDY1_FAULT_IR;
    telemetry_codec_state_t state;

    if ((0 == p_status) || (gh_buddy1_state_mutex <= 0))
    {
        return;
    }

    terr_link_report();

    if (p_status->navigation.events & (unsigned)S3_EVENT_BARCODE)
    {
        /* A decoded event supplies a Code 39 letter from A through Z.
         * Each value fits the unsigned-byte telemetry argument. */
        g_last_action     = action_for(p_status->navigation.action);
        g_pending_barcode = (UB)p_status->navigation.barcode.letter;
    }

    if (g_pending_barcode &&
        (buddy1_report_barcode(g_pending_barcode) != E_TMOUT))
    {
        g_pending_barcode = 0U;
    }
    /* One coherent read for faults, route and speed. Do not reacquire Buddy 2's
     * mutex or repeat encoder/heading arithmetic for each telemetry consumer.
     */
    const bool b_has_motion = motion_get_status(&motion);

    if (b_has_motion)
    {
        active   = 0U;
        changed |= BUDDY1_FAULT_STALL | BUDDY1_FAULT_ENCODER;

        if ((unsigned)motion.fault_bits &
            (unsigned)motion_pack_fault_bit(MOTION_FAULT_STALL))
        {
            active |= BUDDY1_FAULT_STALL;
        }

        if ((unsigned)motion.fault_bits &
            (unsigned)motion_pack_fault_bit(MOTION_FAULT_ENCODER))
        {
            active |= BUDDY1_FAULT_ENCODER;
        }
    }

    if ((p_status->navigation.fault_mask) ||
        (p_status->navigation.latched_faults & (unsigned)S3_FAULT_SENSOR))
    {
        active |= BUDDY1_FAULT_IR;
    }

    if ((!gb_has_fault_snapshot) || (active != g_last_faults))
    {
        /* This owner reports only the IR, stall and encoder bits. Both
         * changed and its active subset fit the wire's six-bit mask. */
        ER error = buddy1_update_faults((UB)changed, (UB)(active & changed));

        if (E_OK == error)
        {
            g_last_faults         = active;
            gb_has_fault_snapshot = true;
        }
    }

    state = wire_state(p_status->state);
    pose_report(p_status, &motion, b_has_motion);
#if APP_S5
    {
        s5_codec_observation_t obstacle;
        const unsigned         control        = s5_api_controller_status();
        const unsigned         obstacle_state = control & 7U;
        const unsigned         route          = (control >> 6) & 3U;

        if ((((unsigned int)S3_DRIVE_MANOEUVRE == p_status->state) &&
             (obstacle_state >= (unsigned int)S5_WAIT_STOP)) &&
            (obstacle_state <= (unsigned int)S5_FINISH))
        {
            state = BUDDY1_STATE_AVOIDING_OBSTACLE;
        }

        const bool b_has_obstacle = s5_api_snapshot(&obstacle);
        obstacle_live_report(&obstacle, b_has_obstacle, control);

        if (b_has_obstacle)
        {
            obstacle_profile_report(&obstacle);
            obstacle_samples_report();

            if ((g_obstacle_report.echo != (unsigned)obstacle.b_has_no_echo) &&
                (E_OK == buddy1_update_faults(BUDDY1_FAULT_NO_ECHO,
                                              obstacle.b_has_no_echo
                                                  ? BUDDY1_FAULT_NO_ECHO
                                                  : 0U)))
            {
                g_obstacle_report.echo = (unsigned)obstacle.b_has_no_echo;
            }

            if (((obstacle.b_is_scan_done) && (obstacle.profile.b_is_valid)) &&
                ((g_obstacle_report.scan != obstacle.scan_id) ||
                 (g_obstacle_report.route != route)))
            {
                report_obstacle_route(&obstacle, route);
            }
            else if (((!obstacle.profile.b_is_valid) &&
                      (!g_obstacle_report.b_has_invalidated)) &&
                     (buddy1_invalidate(BUDDY1_TOPIC_OBSTACLE) == E_OK))
            {
                g_obstacle_report.b_has_invalidated = true;
            }
            else
            {
                /* Leave the reported obstacle validity unchanged. */
            }
        }
    }
#endif
    if (state != g_last_state)
    {
        ER error = buddy1_report_robot_state(state);

        if (E_OK == error)
        {
            g_last_state = state;
        }
    }

    motion_report(p_status, &motion, b_has_motion);

    if ((unsigned int)S3_DRIVE_OFF == p_status->state)
    {
        g_last_action = BUDDY1_ACTION_UNKNOWN;
    }
}

#if APP_S5
/** @brief Record delivery or permanent rejection of a valid route report.
 * @param p_obstacle Borrowed valid observation from the controller owner.
 * @param route Encoded direction attached to this profile.
 * @note A full outbox leaves the existing route marker available for retry.
 */
static inline void
report_obstacle_route (const s5_codec_observation_t * p_obstacle,
                       unsigned                       route)
{
    ER value_er = buddy1_report_obstacle(p_obstacle->profile.closest_mm,
                                         p_obstacle->profile.width_mm,
                                         (telemetry_codec_route_t)route);

    if ((E_OK == value_er) || (E_PAR == value_er))
    {
        g_obstacle_report.scan              = p_obstacle->scan_id;
        g_obstacle_report.route             = route;
        g_obstacle_report.b_has_invalidated = false;
    }
}
#endif

/* End of file. */

