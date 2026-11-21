/** @file buddy1.c
 * @brief Buddy 1: task-context reporting API.
 * Widen the fault-change octet before inversion. The fault mask keeps the same
 * bits.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 * Byte and flag masks are widened unsigned before bitwise operations.
 */
#include "buddy1.h"

#define ALIGN_TO_INT(value) (((value) + sizeof(INT) - 1) & ~(sizeof(INT) - 1))
#define MESSAGE_STORAGE(type) (sizeof(INT) + ALIGN_TO_INT(sizeof(type)))

static ID gh_buddy1_outbox;
ID        gh_buddy1_commands;
ID        gh_buddy1_state_mutex;
/* Same compact layout as the snapshot reply. All access holds the mutex. */
static UB g_latest[BUDDY1_PAYLOAD_MAX] = {BUDDY1_WIRE_VERSION};

static ER store_section (UB                                 type,
                         const telemetry_codec_snapshot_t * p_snapshot);
static ER
queue_telemetry_section (UB                                 type,
                         const telemetry_codec_snapshot_t * p_snapshot);

/** @brief Create the telemetry and command message buffers, then their state
 * mutex. Repeated initialization returns E_OBJ. A later kernel error
 * releases the resources created earlier in this call.
 */
ER subsystem1_init (void)
{
    T_CMBF outbox      = {0};
    T_CMBF commands    = {0};
    T_CMTX state_mutex = {0};

    if (((gh_buddy1_outbox > 0) || (gh_buddy1_commands > 0)) ||
        (gh_buddy1_state_mutex > 0))
    {
        return E_OBJ;
    }

    /* clang-format off */
    outbox.mbfatr    = TA_TFIFO;
    outbox.bufsz     = (unsigned int)BUDDY1_QUEUE_MESSAGES *
                   MESSAGE_STORAGE(buddy1_telemetry_msg_t);
    outbox.maxmsz    = sizeof(buddy1_telemetry_msg_t);
    gh_buddy1_outbox = tk_cre_mbf(&outbox);
    /* clang-format on */

    if (gh_buddy1_outbox < E_OK)
    {
        return gh_buddy1_outbox;
    }

    /* clang-format off */
    commands.mbfatr    = TA_TFIFO;
    commands.bufsz     = (unsigned int)BUDDY1_COMMAND_MESSAGES *
                     MESSAGE_STORAGE(buddy1_command_msg_t);
    commands.maxmsz    = sizeof(buddy1_command_msg_t);
    gh_buddy1_commands = tk_cre_mbf(&commands);
    /* clang-format on */

    if (gh_buddy1_commands < E_OK)
    {
        ER value_er = gh_buddy1_commands;
        (void)tk_del_mbf(gh_buddy1_outbox);
        gh_buddy1_outbox = gh_buddy1_commands = 0;
        return value_er;
    }

    state_mutex.mtxatr    = TA_INHERIT;
    gh_buddy1_state_mutex = tk_cre_mtx(&state_mutex);

    if (gh_buddy1_state_mutex < E_OK)
    {
        ER value_er = gh_buddy1_state_mutex;
        (void)tk_del_mbf(gh_buddy1_commands);
        (void)tk_del_mbf(gh_buddy1_outbox);
        gh_buddy1_commands = gh_buddy1_outbox = gh_buddy1_state_mutex = 0;
        return value_er;
    }

    return E_OK;
}

/** @brief Start the command task in the MQTT profile and propagate any task
 * creation error. Other profiles return E_OK without a task.
 */
ER subsystem1_start (void)
{
#if defined(TM_MQTT) && TM_MQTT
    ID h_command_task = app_start_task(buddy1_command_task, S1_PRI_COMMAND, 0,
                                       "buddy1_command_task");
    return h_command_task < E_OK ? h_command_task : E_OK;
#else
    return E_OK;
#endif
}

/** @brief Validate the topic and payload length, copy the bytes into a queue
 * message and poll the outbox. Return E_PAR for invalid data or the
 * kernel send result.
 */
ER buddy1_queue_publish (UB type, const UB * p_payload, UH payload_len)
{
    buddy1_telemetry_msg_t message = {0};
    UH                     byte_index;

    if (!telemetry_codec_payload_valid(type, p_payload, payload_len))
    {
        return E_PAR;
    }

    message.type = type;

    for (byte_index = 0; byte_index < payload_len; ++byte_index)
    {
        message.payload[byte_index] = p_payload[byte_index];
    }
    /* TMO_POL: the caller never waits for buffer space or for MQTT. */
    return tk_snd_mbf(gh_buddy1_outbox, &message, sizeof(message), TMO_POL);
}

/** @brief Cache left and right signed wheel speeds in millimetres per second
 * and queue the encoded motion report. Return the validation, lock or
 * send result.
 */
ER buddy1_report_motion (H left_mms, H right_mms)
{
    telemetry_codec_snapshot_t snapshot = {0};
    snapshot.left_mms                   = left_mms;
    snapshot.right_mms                  = right_mms;
    return queue_telemetry_section(BUDDY1_TOPIC_MOTION, &snapshot);
}

/** @brief Resolve the letter against the current map, cache the decoded letter
 * and action, then queue the report. Propagate the reporting result.
 */
ER buddy1_report_barcode (UB decoded_letter)
{
    telemetry_codec_snapshot_t snapshot = {0};
    snapshot.barcode_letter             = decoded_letter;
    snapshot.barcode_action             = buddy1_barcode_action(decoded_letter);
    return queue_telemetry_section(BUDDY1_TOPIC_BARCODE, &snapshot);
}

/** @brief Cache the cumulative hump count and height in millimetres and queue
 * the encoded report. Propagate the reporting result.
 */
ER buddy1_report_hump (UH count, UH height_mm)
{
    telemetry_codec_snapshot_t snapshot = {0};
    snapshot.hump_count                 = count;
    snapshot.hump_height_mm             = height_mm;
    return queue_telemetry_section(BUDDY1_TOPIC_HUMP, &snapshot);
}

/** @brief Cache the heading in signed degrees and queue it. Return the
 * encoding, mutex or send result.
 */
ER buddy1_report_heading (H heading_deg)
{
    telemetry_codec_snapshot_t snapshot = {0};
    snapshot.heading_deg                = heading_deg;
    return queue_telemetry_section(BUDDY1_TOPIC_HEADING, &snapshot);
}

/** @brief Cache distance and width in millimetres together with the selected
 * route, then queue the obstacle report. Return the reporting result.
 */
ER buddy1_report_obstacle (UH distance_mm, UH width_mm,
                           telemetry_codec_route_t route)
{
    telemetry_codec_snapshot_t snapshot = {0};
    snapshot.obstacle_distance_mm       = distance_mm;
    snapshot.obstacle_width_mm          = width_mm;
    snapshot.obstacle_route             = route;
    return queue_telemetry_section(BUDDY1_TOPIC_OBSTACLE, &snapshot);
}

/** @brief Cache the wrapping centimetre distance and heading in degrees, then
 * queue the pose report. Return the reporting result.
 */
ER buddy1_report_pose (UH distance_cm_mod, H heading_deg)
{
    telemetry_codec_snapshot_t snapshot = {0};
    snapshot.pose_distance_cm           = distance_cm_mod;
    snapshot.pose_heading_deg           = heading_deg;
    return queue_telemetry_section(BUDDY1_TOPIC_POSE, &snapshot);
}

/** @brief Copy the supplied avoidance status into a live-only report. Reject a
 * null input with E_PAR and propagate encoding or send errors.
 */
ER buddy1_report_avoidance (const telemetry_codec_avoidance_t * p_avoidance)
{
    telemetry_codec_snapshot_t snapshot = {0};

    if (0 == p_avoidance)
    {
        return E_PAR;
    }

    snapshot.avoidance = *p_avoidance;
    return queue_telemetry_section(BUDDY1_TOPIC_AVOIDANCE, &snapshot);
}

/** @brief Encode and queue a live scan summary from the supplied copy. Reject
 * a null input with E_PAR and propagate encoding or send errors.
 */
ER buddy1_report_scan (const telemetry_codec_scan_t * p_scan)
{
    telemetry_codec_snapshot_t snapshot = {0};

    if (0 == p_scan)
    {
        return E_PAR;
    }

    snapshot.scan = *p_scan;
    return queue_telemetry_section(BUDDY1_TOPIC_SCAN, &snapshot);
}

/** @brief Encode and queue a live scan sample from the supplied copy. A null
 * input returns E_PAR.
 */
ER buddy1_report_scan_sample (const telemetry_codec_scan_sample_t * p_sample)
{
    telemetry_codec_snapshot_t snapshot = {0};

    if (0 == p_sample)
    {
        return E_PAR;
    }

    snapshot.scan_sample = *p_sample;
    return queue_telemetry_section(BUDDY1_TOPIC_SCAN_SAMPLE, &snapshot);
}

/** @brief Copy one terrain observation into a live-only report and return its
 * encoding or send result. A null input returns E_PAR.
 */
ER buddy1_report_terrain (const telemetry_codec_terrain_t * p_terrain)
{
    telemetry_codec_snapshot_t snapshot = {0};

    if (0 == p_terrain)
    {
        return E_PAR;
    }

    snapshot.terrain = *p_terrain;
    return queue_telemetry_section(BUDDY1_TOPIC_TERRAIN, &snapshot);
}

/** @brief Cache the public robot state and queue its encoded report. Return
 * the reporting result.
 */
ER buddy1_report_robot_state (telemetry_codec_state_t state)
{
    telemetry_codec_snapshot_t snapshot = {0};
    snapshot.robot_state                = state;
    return queue_telemetry_section(BUDDY1_TOPIC_STATE, &snapshot);
}

/** @brief Apply only the supplied changed fault bits under the state mutex.
 * Invalid masks return E_PAR. The updated faults remain cached if the
 * subsequent outbox poll fails.
 */
ER buddy1_update_faults (UB changed, UB active)
{
    telemetry_codec_snapshot_t snapshot;
    UB                         packet[2];

    if (((0U == changed) || ((changed & ~BUDDY1_FAULT_MASK) != 0U)) ||
        ((active & ~(unsigned)changed) != 0U))
    {
        return E_PAR;
    }

    if (gh_buddy1_state_mutex <= 0)
    {
        return E_OBJ;
    }

    ER value_er = tk_loc_mtx(gh_buddy1_state_mutex, TMO_FEVR);

    if (value_er < E_OK)
    {
        return value_er;
    }

    if (!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, g_latest, sizeof(g_latest),
                       &snapshot))
    {
        (void)tk_unl_mtx(gh_buddy1_state_mutex);
        return E_SYS;
    }

    /* clang-format off */
    snapshot.faults       =
        (UB)(((unsigned)snapshot.faults & ~(unsigned)changed) |
                          (unsigned)active);
    snapshot.faults_valid = 1;
    /* clang-format on */
    /* Both buffers have room for their fixed layouts. The decoded snapshot
     * was canonical and only validated fault bits changed. The caller
     * receives the publication result, independent of either byte count. */
    (void)buddy1_encode(BUDDY1_TOPIC_SNAPSHOT, &snapshot, g_latest,
                        sizeof(g_latest));
    (void)buddy1_encode(BUDDY1_TOPIC_FAULTS, &snapshot, packet, sizeof(packet));
    (void)tk_unl_mtx(gh_buddy1_state_mutex);
    return buddy1_queue_publish(BUDDY1_TOPIC_FAULTS, packet, sizeof(packet));
}

/** @brief Clear a supported cached section's validity bit and payload bits
 * under the state mutex. Return E_PAR for an invalid type or the mutex
 * acquisition error.
 */
ER buddy1_invalidate (UB telemetry_type)
{
    unsigned index;
    unsigned offset = 7;

    if ((int)telemetry_type > BUDDY1_TOPIC_FAULTS)
    {
        return E_PAR;
    }

    ER value_er = tk_loc_mtx(gh_buddy1_state_mutex, TMO_FEVR);

    if (value_er < E_OK)
    {
        return value_er;
    }

    g_latest[1] = (UB)((unsigned)g_latest[1] & ~(1U << telemetry_type));

    for (index = 0; index < telemetry_type; ++index)
    {
        offset += telemetry_codec_payload_bits((UB)index);
    }

    for (index = 0; index < telemetry_codec_payload_bits(telemetry_type);
         ++index)
    {
        unsigned bit = offset + index;
        g_latest[1U + (bit / 8U)] =
            (UB)((unsigned)g_latest[1U + (bit / 8U)] & ~(1U << (bit % 8U)));
    }

    (void)tk_unl_mtx(gh_buddy1_state_mutex);
    return E_OK;
}

/** @brief Copy the entire cached wire snapshot under the state mutex. The
 * caller supplies BUDDY1_PAYLOAD_MAX bytes. A null buffer or failed
 * lock returns FALSE before copying.
 */
BOOL buddy1_snapshot_packet (UB p_packet[BUDDY1_PAYLOAD_MAX])
{
    unsigned byte_index;

    if ((!p_packet) || (tk_loc_mtx(gh_buddy1_state_mutex, TMO_FEVR) < E_OK))
    {
        return FALSE;
    }

    for (byte_index = 0; byte_index < sizeof(g_latest); ++byte_index)
    {
        p_packet[byte_index] = g_latest[byte_index];
    }

    (void)tk_unl_mtx(gh_buddy1_state_mutex);
    return TRUE;
}

/** @brief Decode a mutex-protected copy of the cached wire snapshot into the
 * caller's output. A missing output, unavailable copy or decode
 * failure returns FALSE.
 */
BOOL buddy1_snapshot (telemetry_codec_snapshot_t * p_out)
{
    UB packet[BUDDY1_PAYLOAD_MAX];
    return (p_out && (buddy1_snapshot_packet(packet))) &&
           (buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, packet, sizeof(packet),
                          p_out));
}

/** @brief Read both cached wheel speeds in signed millimetres per second.
 * Missing outputs or an invalid or unavailable snapshot leave both
 * outputs untouched.
 */
BOOL buddy1_motion_snapshot (H * p_left_mms, H * p_right_mms)
{
    telemetry_codec_snapshot_t snapshot;

    if ((((0 == p_left_mms) || (0 == p_right_mms)) ||
         (!buddy1_snapshot(&snapshot))) ||
        (!snapshot.motion_valid))
    {
        return FALSE;
    }

    *p_left_mms  = snapshot.left_mms;
    *p_right_mms = snapshot.right_mms;
    return TRUE;
}

/** @brief Query the message buffer and return TRUE only when the handle is
 * valid, the query succeeds and the reported message size is zero.
 */
BOOL buddy1_outbox_idle (void)
{
    T_RMBF ref;
    return ((gh_buddy1_outbox > 0) &&
            (tk_ref_mbf(gh_buddy1_outbox, &ref) == E_OK)) &&
           (0 == ref.msgsz);
}

/** @brief Poll for one telemetry message in caller storage. Return TRUE only
 * when the kernel delivers exactly one complete message.
 */
BOOL buddy1_outbox_take (buddy1_telemetry_msg_t * p_out)
{
    return p_out && (tk_rcv_mbf(gh_buddy1_outbox, p_out, TMO_POL) ==
                     (INT)sizeof(*p_out));
}

/** @brief With the state mutex held, decode the cached snapshot, replace the
 * selected section and encode it back. Return E_SYS for an unreadable
 * cache and E_PAR for an unsupported section or encoding failure.
 */
static ER store_section (UB type, const telemetry_codec_snapshot_t * p_snapshot)
{
    telemetry_codec_snapshot_t cache;

    if (!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, g_latest, sizeof(g_latest),
                       &cache))
    {
        return E_SYS;
    }

    switch (type)
    {
    case BUDDY1_TOPIC_MOTION:
        cache.motion_valid = 1;
        cache.left_mms     = p_snapshot->left_mms;
        cache.right_mms    = p_snapshot->right_mms;
    break;
    case BUDDY1_TOPIC_BARCODE:
        cache.barcode_valid  = 1;
        cache.barcode_letter = p_snapshot->barcode_letter;
        cache.barcode_action = p_snapshot->barcode_action;
    break;
    case BUDDY1_TOPIC_HUMP:
        cache.hump_valid     = 1;
        cache.hump_count     = p_snapshot->hump_count;
        cache.hump_height_mm = p_snapshot->hump_height_mm;
    break;
    case BUDDY1_TOPIC_HEADING:
        cache.heading_valid = 1;
        cache.heading_deg   = p_snapshot->heading_deg;
    break;
    case BUDDY1_TOPIC_OBSTACLE:
        cache.obstacle_valid       = 1;
        cache.obstacle_distance_mm = p_snapshot->obstacle_distance_mm;
        cache.obstacle_width_mm    = p_snapshot->obstacle_width_mm;
        cache.obstacle_route       = p_snapshot->obstacle_route;
    break;
    case BUDDY1_TOPIC_STATE:
        cache.robot_state_valid = 1;
        cache.robot_state       = p_snapshot->robot_state;
    break;
    case BUDDY1_TOPIC_POSE:
        cache.pose_valid       = 1;
        cache.pose_distance_cm = p_snapshot->pose_distance_cm;
        cache.pose_heading_deg = p_snapshot->pose_heading_deg;
    break;
    default:
        return E_PAR;
    }

    return buddy1_encode(BUDDY1_TOPIC_SNAPSHOT, &cache, g_latest,
                         sizeof(g_latest))
               ? E_OK
               : E_PAR;
}

/** @brief Encode one topic. For cached topics, update the snapshot under its
 * mutex before polling the outbox. A send failure can therefore leave
 * the new value available to snapshot readers.
 */
static ER
queue_telemetry_section (UB type, const telemetry_codec_snapshot_t * p_snapshot)
{
    UB             payload[BUDDY1_PAYLOAD_MAX];
    const unsigned length =
        buddy1_encode(type, p_snapshot, payload, sizeof(payload));
    ER value_er;

    if (!length)
    {
        return E_PAR;
    }
    /* Terrain, avoidance, scan and scan samples are live-only: no snapshot
     * section, so skip the cache. Pose is cached since 19 Nov 2026 (the
     * snapshot reply carries distance/heading since START). */
    if (((((int)type != BUDDY1_TOPIC_TERRAIN) &&
          ((int)type != BUDDY1_TOPIC_AVOIDANCE)) &&
         ((int)type != BUDDY1_TOPIC_SCAN)) &&
        ((int)type != BUDDY1_TOPIC_SCAN_SAMPLE))
    {
        value_er = tk_loc_mtx(gh_buddy1_state_mutex, TMO_FEVR);

        if (value_er < E_OK)
        {
            return value_er;
        }

        value_er = store_section(type, p_snapshot);
        (void)tk_unl_mtx(gh_buddy1_state_mutex);

        if (value_er < E_OK)
        {
            return value_er;
        }
    }

    return buddy1_queue_publish(type, payload, (UH)length);
}

/* End of file. */

