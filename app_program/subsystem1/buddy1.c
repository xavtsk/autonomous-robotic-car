/** @file buddy1.c
 * @brief Buddy 1: task-context reporting API.
 */
#include "buddy1.h"

#define ALIGN_TO_INT(value) (((value) + sizeof(INT) - 1) & ~(sizeof(INT) - 1))
#define MESSAGE_STORAGE(type) (sizeof(INT) + ALIGN_TO_INT(sizeof(type)))

EXPORT ID g_mbf_buddy1_outbox;
EXPORT ID g_mbf_buddy1_commands;
EXPORT ID g_mtx_buddy1_state;
/* Same compact layout as the snapshot reply. All access holds the mutex. */
LOCAL UB g_latest[BUDDY1_PAYLOAD_MAX] = {BUDDY1_WIRE_VERSION};

EXPORT ER subsystem1_init (void)
{
    T_CMBF outbox = {0}, commands = {0};
    T_CMTX state_mutex = {0};
    if (g_mbf_buddy1_outbox > 0 || g_mbf_buddy1_commands > 0 ||
        g_mtx_buddy1_state > 0)
    {
        return E_OBJ;
    }
    outbox.mbfatr = TA_TFIFO;
    outbox.bufsz =
        BUDDY1_QUEUE_MESSAGES * MESSAGE_STORAGE(buddy1_telemetry_msg_t);
    outbox.maxmsz = sizeof(buddy1_telemetry_msg_t);
    g_mbf_buddy1_outbox = tk_cre_mbf(&outbox);
    if (g_mbf_buddy1_outbox < E_OK)
    {
        return g_mbf_buddy1_outbox;
    }
    commands.mbfatr = TA_TFIFO;
    commands.bufsz =
        BUDDY1_COMMAND_MESSAGES * MESSAGE_STORAGE(buddy1_command_msg_t);
    commands.maxmsz = sizeof(buddy1_command_msg_t);
    g_mbf_buddy1_commands = tk_cre_mbf(&commands);
    if (g_mbf_buddy1_commands < E_OK)
    {
        ER value_er = g_mbf_buddy1_commands;
        (void)tk_del_mbf(g_mbf_buddy1_outbox);
        g_mbf_buddy1_outbox = g_mbf_buddy1_commands = 0;
        return value_er;
    }
    state_mutex.mtxatr = TA_INHERIT;
    g_mtx_buddy1_state = tk_cre_mtx(&state_mutex);
    if (g_mtx_buddy1_state < E_OK)
    {
        ER value_er = g_mtx_buddy1_state;
        (void)tk_del_mbf(g_mbf_buddy1_commands);
        (void)tk_del_mbf(g_mbf_buddy1_outbox);
        g_mbf_buddy1_commands = g_mbf_buddy1_outbox = g_mtx_buddy1_state = 0;
        return value_er;
    }
    return E_OK;
}

EXPORT ER subsystem1_start (void)
{
#if defined(TM_MQTT) && TM_MQTT
    ID tid = app_start_task(buddy1_command_task, S1_PRI_COMMAND, 0,
                            "buddy1_command_task");
    return tid < E_OK ? tid : E_OK;
#else
    return E_OK;
#endif
}

EXPORT ER buddy1_queue_publish (UB type, const UB * p_payload, UH payload_len)
{
    buddy1_telemetry_msg_t message = {0};
    UH value_i;
    if (!buddy1_payload_valid(type, p_payload, payload_len))
    {
        return E_PAR;
    }
    message.type = type;
    for (value_i = 0; value_i < payload_len; ++value_i)
    {
        message.payload[value_i] = p_payload[value_i];
    }
    /* TMO_POL: the caller never waits for buffer space or for MQTT. */
    return tk_snd_mbf(g_mbf_buddy1_outbox, &message, sizeof(message), TMO_POL);
}

LOCAL ER store_section (UB type, const buddy1_snapshot_t * p_v)
{
    buddy1_snapshot_t cache;
    if (!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, g_latest, sizeof(g_latest),
                       &cache))
    {
        return E_SYS;
    }
    switch (type)
    {
    case BUDDY1_TOPIC_MOTION:
        cache.motion_valid = 1;
        cache.left_mms = p_v->left_mms;
        cache.right_mms = p_v->right_mms;
        break;
    case BUDDY1_TOPIC_BARCODE:
        cache.barcode_valid = 1;
        cache.barcode_letter = p_v->barcode_letter;
        cache.barcode_action = p_v->barcode_action;
        break;
    case BUDDY1_TOPIC_HUMP:
        cache.hump_valid = 1;
        cache.hump_count = p_v->hump_count;
        cache.hump_height_mm = p_v->hump_height_mm;
        break;
    case BUDDY1_TOPIC_HEADING:
        cache.heading_valid = 1;
        cache.heading_deg = p_v->heading_deg;
        break;
    case BUDDY1_TOPIC_OBSTACLE:
        cache.obstacle_valid = 1;
        cache.obstacle_distance_mm = p_v->obstacle_distance_mm;
        cache.obstacle_width_mm = p_v->obstacle_width_mm;
        cache.obstacle_route = p_v->obstacle_route;
        break;
    case BUDDY1_TOPIC_STATE:
        cache.robot_state_valid = 1;
        cache.robot_state = p_v->robot_state;
        break;
    case BUDDY1_TOPIC_POSE:
        cache.pose_valid = 1;
        cache.pose_distance_cm = p_v->pose_distance_cm;
        cache.pose_heading_deg = p_v->pose_heading_deg;
        break;
    default:
        return E_PAR;
    }
    return buddy1_encode(BUDDY1_TOPIC_SNAPSHOT, &cache, g_latest,
                         sizeof(g_latest))
               ? E_OK
               : E_PAR;
}

LOCAL ER report (UB type, const buddy1_snapshot_t * p_v)
{
    UB payload[BUDDY1_PAYLOAD_MAX];
    unsigned length = buddy1_encode(type, p_v, payload, sizeof(payload));
    ER value_er;
    if (!length)
    {
        return E_PAR;
    }
    /* Terrain, avoidance, scan and scan samples are live-only: no snapshot
     * section, so skip the cache. Pose is cached since 19 Nov 2026 (the
     * snapshot reply carries distance/heading since START). */
    if (type != BUDDY1_TOPIC_TERRAIN && type != BUDDY1_TOPIC_AVOIDANCE &&
        type != BUDDY1_TOPIC_SCAN && type != BUDDY1_TOPIC_SCAN_SAMPLE)
    {
        value_er = tk_loc_mtx(g_mtx_buddy1_state, TMO_FEVR);
        if (value_er < E_OK)
        {
            return value_er;
        }
        value_er = store_section(type, p_v);
        (void)tk_unl_mtx(g_mtx_buddy1_state);
        if (value_er < E_OK)
        {
            return value_er;
        }
    }
    return buddy1_queue_publish(type, payload, (UH)length);
}

EXPORT ER buddy1_report_motion (H left_mms, H right_mms)
{
    buddy1_snapshot_t value_v = {0};
    value_v.left_mms = left_mms;
    value_v.right_mms = right_mms;
    return report(BUDDY1_TOPIC_MOTION, &value_v);
}

EXPORT ER buddy1_report_barcode (UB letter)
{
    buddy1_snapshot_t value_v = {0};
    value_v.barcode_letter = letter;
    value_v.barcode_action = buddy1_barcode_action(letter);
    return report(BUDDY1_TOPIC_BARCODE, &value_v);
}

EXPORT ER buddy1_report_hump (UH count, UH height_mm)
{
    buddy1_snapshot_t value_v = {0};
    value_v.hump_count = count;
    value_v.hump_height_mm = height_mm;
    return report(BUDDY1_TOPIC_HUMP, &value_v);
}

EXPORT ER buddy1_report_heading (H heading_deg)
{
    buddy1_snapshot_t value_v = {0};
    value_v.heading_deg = heading_deg;
    return report(BUDDY1_TOPIC_HEADING, &value_v);
}

EXPORT ER buddy1_report_obstacle (UH distance_mm, UH width_mm,
                                  buddy1_obstacle_route_t route)
{
    buddy1_snapshot_t value_v = {0};
    value_v.obstacle_distance_mm = distance_mm;
    value_v.obstacle_width_mm = width_mm;
    value_v.obstacle_route = route;
    return report(BUDDY1_TOPIC_OBSTACLE, &value_v);
}

EXPORT ER buddy1_report_pose (UH distance_cm_mod, H heading_deg)
{
    buddy1_snapshot_t value_v = {0};
    value_v.pose_distance_cm = distance_cm_mod;
    value_v.pose_heading_deg = heading_deg;
    return report(BUDDY1_TOPIC_POSE, &value_v);
}

EXPORT ER buddy1_report_avoidance (const buddy1_avoidance_t * p_avoidance)
{
    buddy1_snapshot_t value_v = {0};
    if (!p_avoidance)
    {
        return E_PAR;
    }
    value_v.avoidance = *p_avoidance;
    return report(BUDDY1_TOPIC_AVOIDANCE, &value_v);
}

EXPORT ER buddy1_report_scan (const buddy1_scan_t * p_scan)
{
    buddy1_snapshot_t value_v = {0};
    if (!p_scan)
    {
        return E_PAR;
    }
    value_v.scan = *p_scan;
    return report(BUDDY1_TOPIC_SCAN, &value_v);
}

EXPORT ER buddy1_report_scan_sample (const buddy1_scan_sample_t * p_sample)
{
    buddy1_snapshot_t value_v = {0};
    if (!p_sample)
    {
        return E_PAR;
    }
    value_v.scan_sample = *p_sample;
    return report(BUDDY1_TOPIC_SCAN_SAMPLE, &value_v);
}

EXPORT ER buddy1_report_terrain (const buddy1_terrain_t * p_terrain)
{
    buddy1_snapshot_t value_v = {0};
    if (!p_terrain)
    {
        return E_PAR;
    }
    value_v.terrain = *p_terrain;
    return report(BUDDY1_TOPIC_TERRAIN, &value_v);
}

EXPORT ER buddy1_report_robot_state (buddy1_robot_state_t state)
{
    buddy1_snapshot_t value_v = {0};
    value_v.robot_state = state;
    return report(BUDDY1_TOPIC_STATE, &value_v);
}

EXPORT ER buddy1_update_faults (UB changed, UB active)
{
    buddy1_snapshot_t value_v;
    UB packet[2];
    ER value_er;
    if (changed == 0U || (changed & ~BUDDY1_FAULT_MASK) != 0U ||
        (active & ~changed) != 0U)
    {
        return E_PAR;
    }
    if (g_mtx_buddy1_state <= 0)
    {
        return E_OBJ;
    }
    value_er = tk_loc_mtx(g_mtx_buddy1_state, TMO_FEVR);
    if (value_er < E_OK)
    {
        return value_er;
    }
    if (!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, g_latest, sizeof(g_latest),
                       &value_v))
    {
        (void)tk_unl_mtx(g_mtx_buddy1_state);
        return E_SYS;
    }
    value_v.faults = (UB)((value_v.faults & (UB)~changed) | active);
    value_v.faults_valid = 1;
    (void)buddy1_encode(BUDDY1_TOPIC_SNAPSHOT, &value_v, g_latest,
                        sizeof(g_latest));
    (void)buddy1_encode(BUDDY1_TOPIC_FAULTS, &value_v, packet, sizeof(packet));
    (void)tk_unl_mtx(g_mtx_buddy1_state);
    return buddy1_queue_publish(BUDDY1_TOPIC_FAULTS, packet, sizeof(packet));
}

EXPORT ER buddy1_invalidate (UB type)
{
    unsigned value_i, offset = 7;
    ER value_er;
    if (type > BUDDY1_TOPIC_FAULTS)
    {
        return E_PAR;
    }
    value_er = tk_loc_mtx(g_mtx_buddy1_state, TMO_FEVR);
    if (value_er < E_OK)
    {
        return value_er;
    }
    g_latest[1] &= (UB) ~(1U << type);
    for (value_i = 0; value_i < type; ++value_i)
    {
        offset += buddy1_payload_bits((UB)value_i);
    }
    for (value_i = 0; value_i < buddy1_payload_bits(type); ++value_i)
    {
        unsigned bit = offset + value_i;
        g_latest[1 + bit / 8] &= (UB) ~(1U << (bit % 8));
    }
    (void)tk_unl_mtx(g_mtx_buddy1_state);
    return E_OK;
}

EXPORT BOOL buddy1_snapshot_packet (UB p_packet[BUDDY1_PAYLOAD_MAX])
{
    unsigned value_i;
    if (!p_packet || tk_loc_mtx(g_mtx_buddy1_state, TMO_FEVR) < E_OK)
    {
        return FALSE;
    }
    for (value_i = 0; value_i < sizeof(g_latest); ++value_i)
    {
        p_packet[value_i] = g_latest[value_i];
    }
    (void)tk_unl_mtx(g_mtx_buddy1_state);
    return TRUE;
}

EXPORT BOOL buddy1_snapshot (buddy1_snapshot_t * p_out)
{
    UB packet[BUDDY1_PAYLOAD_MAX];
    return p_out && buddy1_snapshot_packet(packet) &&
           buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, packet, sizeof(packet), p_out);
}

EXPORT BOOL buddy1_motion_snapshot (H * p_left_mms, H * p_right_mms)
{
    buddy1_snapshot_t value_v;
    if (!p_left_mms || !p_right_mms || !buddy1_snapshot(&value_v) ||
        !value_v.motion_valid)
    {
        return FALSE;
    }
    *p_left_mms = value_v.left_mms;
    *p_right_mms = value_v.right_mms;
    return TRUE;
}

EXPORT BOOL buddy1_outbox_idle (void)
{
    T_RMBF ref;
    return g_mbf_buddy1_outbox > 0 &&
           tk_ref_mbf(g_mbf_buddy1_outbox, &ref) == E_OK && ref.msgsz == 0;
}

EXPORT BOOL buddy1_outbox_take (buddy1_telemetry_msg_t * p_out)
{
    return p_out && tk_rcv_mbf(g_mbf_buddy1_outbox, p_out, TMO_POL) ==
                        (INT)sizeof(*p_out);
}

/* End of file. */

